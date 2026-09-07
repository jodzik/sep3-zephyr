#include <sep3-zephyr/sep3_zephyr.h>

#include <safe_c.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <errno.h>
#include <limits.h>
#include <string.h>

LOG_MODULE_REGISTER(sep3_zephyr);

enum Sep3ZephyrState {
    SEP3_ZEPHYR_STATE_UNINITIALIZED = 0,
    SEP3_ZEPHYR_STATE_RUNNING,
    SEP3_ZEPHYR_STATE_STOPPING,
    SEP3_ZEPHYR_STATE_STOPPED,
};

enum Sep3ZephyrTxEvent {
    SEP3_ZEPHYR_TX_EVENT_NONE = 0,
    SEP3_ZEPHYR_TX_EVENT_DONE,
    SEP3_ZEPHYR_TX_EVENT_ABORTED,
};

enum Sep3ZephyrCommandType {
    SEP3_ZEPHYR_COMMAND_REGISTER_READ = 0,
    SEP3_ZEPHYR_COMMAND_REGISTER_WRITE,
    SEP3_ZEPHYR_COMMAND_READ,
    SEP3_ZEPHYR_COMMAND_WRITE,
    SEP3_ZEPHYR_COMMAND_WRITE_NO_ANSWER,
    SEP3_ZEPHYR_COMMAND_SEND_READ_ANSWER,
    SEP3_ZEPHYR_COMMAND_SEND_WRITE_ANSWER,
    SEP3_ZEPHYR_COMMAND_SEND_ERROR_ANSWER,
    SEP3_ZEPHYR_COMMAND_STOP,
};

struct Sep3ZephyrCommand {
    enum Sep3ZephyrCommandType type;
    struct Sep3Zephyr *owner;
    struct k_sem completed;
    int result;
    union {
        struct {
            DataId data_id;
            Sep3ZephyrReadHandler handler;
            void *user;
        } register_read;
        struct {
            DataId data_id;
            bool allow_write_no_answer;
            Sep3ZephyrWriteHandler handler;
            void *user;
        } register_write;
        struct {
            DataId data_id;
            uint8_t *data;
            uint16_t data_capacity;
            uint16_t *data_size;
            struct Sep3ZephyrRequestResult *result;
        } read;
        struct {
            DataId data_id;
            uint8_t const *data;
            uint16_t data_size;
            struct Sep3ZephyrRequestResult *result;
        } write;
        struct {
            DataId data_id;
            uint8_t const *data;
            uint16_t data_size;
        } write_no_answer;
        struct {
            struct Sep3RequestToken token;
            uint8_t const *data;
            uint16_t data_size;
        } read_answer;
        struct {
            struct Sep3RequestToken token;
        } write_answer;
        struct {
            struct Sep3RequestToken token;
            uint8_t error_code;
            char const *message;
        } error_answer;
    } args;
};

static atomic_t g_token_epoch = ATOMIC_INIT(0);
static struct device const *g_claimed_uarts[CONFIG_SEP3_ZEPHYR_MAX_INSTANCES];
K_MUTEX_DEFINE(g_init_mutex);

static int _enter_api(struct Sep3Zephyr *self);
static void _leave_api(struct Sep3Zephyr *self);
static int _submit_command(struct Sep3Zephyr *self, struct Sep3ZephyrCommand *command);
static int _queue_stop_command(struct Sep3Zephyr *self, struct Sep3ZephyrCommand *command);
static void _uart_callback(struct device const *device, struct uart_event *event, void *user_data);
static void _uart_noop_callback(struct device const *device, struct uart_event *event, void *user_data);
static void _thread_entry(void *arg1, void *arg2, void *arg3);

static uint32_t _next_token_epoch(void)
{
    uint32_t epoch = 0;

    do {
        epoch = (uint32_t)atomic_inc(&g_token_epoch) + 1U;
    } while (0U == epoch);

    return epoch;
}

static int _claim_uart(struct Sep3Zephyr *self, struct device const *uart)
{
    int rc = ER_NO_MEM;

    for (uint8_t i = 0; i < ARRAY_SIZE(g_claimed_uarts); i++) {
        ASSERT(uart != g_claimed_uarts[i], ER_BUSY);
    }

    for (uint8_t i = 0; i < ARRAY_SIZE(g_claimed_uarts); i++) {
        if (NULL == g_claimed_uarts[i]) {
            g_claimed_uarts[i] = uart;
            self->uart_claim_index = (int8_t)i;
            rc = 0;
            break;
        }
    }

finally:

    return rc;
}

static void _release_uart(struct Sep3Zephyr *self)
{
    if (0 <= self->uart_claim_index && (size_t)self->uart_claim_index < ARRAY_SIZE(g_claimed_uarts)) {
        g_claimed_uarts[self->uart_claim_index] = NULL;
        self->uart_claim_index = -1;
    }
}

static int _enter_api(struct Sep3Zephyr *self)
{
    int rc = 0;
    bool is_locked = false;

    TRY(k_mutex_lock(&g_init_mutex, K_FOREVER));
    is_locked = true;
    ASSERT(SEP3_ZEPHYR_STATE_RUNNING == atomic_get(&self->state), ER_NO_DEV);
    atomic_inc(&self->active_call_count);

finally:

    if (is_locked) {
        k_mutex_unlock(&g_init_mutex);
    }

    return rc;
}

static void _leave_api(struct Sep3Zephyr *self)
{
    if (1 == atomic_dec(&self->active_call_count)) {
        k_sem_give(&self->active_calls_done);
    }
}

static struct Sep3ZephyrEndpoint *_find_endpoint(struct Sep3Zephyr *self, DataId const data_id)
{
    for (uint16_t i = 0; i < ARRAY_SIZE(self->endpoints); i++) {
        if (self->endpoints[i].is_used && data_id == self->endpoints[i].data_id) {
            return &self->endpoints[i];
        }
    }

    return NULL;
}

static struct Sep3ZephyrEndpoint *_allocate_endpoint(struct Sep3Zephyr *self, DataId const data_id)
{
    struct Sep3ZephyrEndpoint *endpoint = _find_endpoint(self, data_id);

    if (NULL != endpoint) {
        return endpoint;
    }

    for (uint16_t i = 0; i < ARRAY_SIZE(self->endpoints); i++) {
        if (!self->endpoints[i].is_used) {
            endpoint = &self->endpoints[i];
            endpoint->owner = self;
            endpoint->data_id = data_id;
            endpoint->is_used = true;
            self->endpoint_count++;
            return endpoint;
        }
    }

    return NULL;
}

static void _release_empty_endpoint(struct Sep3Zephyr *self, struct Sep3ZephyrEndpoint *endpoint)
{
    if (NULL == endpoint->on_read && NULL == endpoint->on_write) {
        memset(endpoint, 0, sizeof(*endpoint));
        self->endpoint_count--;
    }
}

static void _read_endpoint_adapter(
    struct Sep3 *core,
    struct Sep3RequestToken const *token,
    void *user)
{
    struct Sep3ZephyrEndpoint *endpoint = user;

    ARG_UNUSED(core);
    endpoint->on_read(endpoint->owner, token, endpoint->on_read_user);
}

static void _write_endpoint_adapter(
    struct Sep3 *core,
    struct Sep3RequestToken const *token,
    uint8_t const *data,
    uint16_t const data_size,
    bool const is_need_answer,
    void *user)
{
    struct Sep3ZephyrEndpoint *endpoint = user;

    ARG_UNUSED(core);
    endpoint->on_write(endpoint->owner, token, data, data_size, is_need_answer, endpoint->on_write_user);
}

static void _complete_command(struct Sep3ZephyrCommand *command, int const result)
{
    command->result = result;
    k_sem_give(&command->completed);
}

static void _request_callback(
    struct Sep3 *core,
    struct Sep3RequestResult const *core_result,
    void *user)
{
    struct Sep3ZephyrCommand *command = user;
    struct Sep3ZephyrRequestResult *result = NULL;
    uint8_t *data = NULL;
    uint16_t data_capacity = 0;
    uint16_t *data_size = NULL;
    int rc = core_result->result;

    ARG_UNUSED(core);

    if (SEP3_ZEPHYR_COMMAND_READ == command->type) {
        result = command->args.read.result;
        data = command->args.read.data;
        data_capacity = command->args.read.data_capacity;
        data_size = command->args.read.data_size;
    } else {
        result = command->args.write.result;
    }

    result->answer_type = core_result->answer_type;
    result->remote_error_code = core_result->remote_error_code;
    result->data_size = core_result->data_size;

    if (NULL != core_result->remote_error_message) {
        (void)strlcpy(result->remote_error_message, core_result->remote_error_message,
            sizeof(result->remote_error_message));
    }

    if (NULL != data_size) {
        *data_size = core_result->data_size;
    }

    if (0 == rc && 0U < core_result->data_size) {
        if (core_result->data_size > data_capacity) {
            rc = ER_OVERFLOW;
        } else {
            memmove(data, core_result->data, core_result->data_size);
        }
    }

    _complete_command(command, rc);
}

static int _transmit(
    struct Sep3 *core,
    uint8_t const *frame,
    uint16_t const frame_size,
    uint8_t const slot_id,
    void *user)
{
    int rc = 0;
    struct Sep3Zephyr *self = user;
    int uart_rc = 0;

    ARG_UNUSED(core);

    ASSERT(0 == atomic_get(&self->tx_active), ER_AGAIN);

    self->tx_slot_id = slot_id;
    self->tx_started_ms = k_uptime_get();
    atomic_set(&self->tx_event, SEP3_ZEPHYR_TX_EVENT_NONE);
    atomic_set(&self->tx_abort_requested, 0);
    atomic_set(&self->tx_active, 1);

    uart_rc = uart_tx(self->config.uart, frame, frame_size, (int32_t)self->config.uart_tx_timeout_us);
    if (0 != uart_rc) {
        atomic_set(&self->tx_active, 0);
        rc = -EBUSY == uart_rc ? ER_AGAIN : ER_IO;
    }

finally:

    return rc;
}

static int _register_read(struct Sep3Zephyr *self, struct Sep3ZephyrCommand *command)
{
    int rc = 0;
    struct Sep3ZephyrEndpoint *endpoint = _allocate_endpoint(self, command->args.register_read.data_id);

    ASSERT(NULL != endpoint, ER_NO_MEM);
    TRY(sep3__register_read_handler(&self->core, command->args.register_read.data_id,
        _read_endpoint_adapter, endpoint));

    endpoint->on_read = command->args.register_read.handler;
    endpoint->on_read_user = command->args.register_read.user;

finally:

    if (0 != rc && NULL != endpoint) {
        _release_empty_endpoint(self, endpoint);
    }

    return rc;
}

static int _register_write(struct Sep3Zephyr *self, struct Sep3ZephyrCommand *command)
{
    int rc = 0;
    struct Sep3ZephyrEndpoint *endpoint = _allocate_endpoint(self, command->args.register_write.data_id);

    ASSERT(NULL != endpoint, ER_NO_MEM);
    TRY(sep3__register_write_handler(&self->core, command->args.register_write.data_id,
        command->args.register_write.allow_write_no_answer, _write_endpoint_adapter, endpoint));

    endpoint->on_write = command->args.register_write.handler;
    endpoint->on_write_user = command->args.register_write.user;

finally:

    if (0 != rc && NULL != endpoint) {
        _release_empty_endpoint(self, endpoint);
    }

    return rc;
}

static int _start_read(struct Sep3Zephyr *self, struct Sep3ZephyrCommand *command)
{
    int rc = 0;

    TRY(sep3__read(&self->core, command->args.read.data_id, _request_callback, command));

finally:

    return rc;
}

static int _start_write(struct Sep3Zephyr *self, struct Sep3ZephyrCommand *command)
{
    int rc = 0;

    TRY(sep3__write(&self->core, command->args.write.data_id, command->args.write.data,
        command->args.write.data_size, _request_callback, command));

finally:

    return rc;
}

static void _handle_command(struct Sep3Zephyr *self, struct Sep3ZephyrCommand *command)
{
    int rc = 0;
    bool complete_now = true;

    switch (command->type) {
        case SEP3_ZEPHYR_COMMAND_REGISTER_READ:
            rc = _register_read(self, command);
            break;
        case SEP3_ZEPHYR_COMMAND_REGISTER_WRITE:
            rc = _register_write(self, command);
            break;
        case SEP3_ZEPHYR_COMMAND_READ:
            rc = _start_read(self, command);
            complete_now = 0 != rc;
            break;
        case SEP3_ZEPHYR_COMMAND_WRITE:
            rc = _start_write(self, command);
            complete_now = 0 != rc;
            break;
        case SEP3_ZEPHYR_COMMAND_WRITE_NO_ANSWER:
            rc = sep3__write_no_answer(&self->core, command->args.write_no_answer.data_id,
                command->args.write_no_answer.data, command->args.write_no_answer.data_size);
            break;
        case SEP3_ZEPHYR_COMMAND_SEND_READ_ANSWER:
            rc = sep3__send_read_answer(&self->core, &command->args.read_answer.token,
                command->args.read_answer.data, command->args.read_answer.data_size);
            break;
        case SEP3_ZEPHYR_COMMAND_SEND_WRITE_ANSWER:
            rc = sep3__send_write_answer(&self->core, &command->args.write_answer.token);
            break;
        case SEP3_ZEPHYR_COMMAND_SEND_ERROR_ANSWER:
            rc = sep3__send_error_answer(&self->core, &command->args.error_answer.token,
                command->args.error_answer.error_code, command->args.error_answer.message);
            break;
        case SEP3_ZEPHYR_COMMAND_STOP:
            if (self->core.outgoing.active || self->core.incoming.active || self->core.incoming.answer_pending ||
                self->core.transient_answer_pending || 0U != self->core.tx_count ||
                0 != atomic_get(&self->tx_active)) {
                rc = ER_BUSY;
                atomic_set(&self->state, SEP3_ZEPHYR_STATE_RUNNING);
            } else {
                self->stop_command = command;
                if (0 != atomic_get(&self->rx_disabled)) {
                    complete_now = false;
                } else {
                    rc = uart_rx_disable(self->config.uart);
                    if (-EFAULT == rc) {
                        atomic_set(&self->rx_disabled, 1);
                        rc = 0;
                    } else if (0 != rc) {
                        self->stop_command = NULL;
                        atomic_set(&self->state, SEP3_ZEPHYR_STATE_RUNNING);
                    }
                    complete_now = 0 != rc;
                }
            }
            break;
        default:
            rc = ER_INVAL;
            break;
    }

    if (complete_now) {
        _complete_command(command, rc);
    }
}

static void _handle_tx_event(struct Sep3Zephyr *self)
{
    int const event = atomic_set(&self->tx_event, SEP3_ZEPHYR_TX_EVENT_NONE);

    if (SEP3_ZEPHYR_TX_EVENT_NONE == event) {
        return;
    }

    if (0 == atomic_set(&self->tx_active, 0)) {
        LOG_WRN("Ignoring UART TX event without an active SEP3 frame");
        return;
    }
    atomic_set(&self->tx_abort_requested, 0);

    int const rc = sep3__handle_transmitted(&self->core, self->tx_slot_id,
        SEP3_ZEPHYR_TX_EVENT_DONE == event ? 0 : ER_IO);
    if (0 != rc) {
        LOG_ERR("Failed to complete SEP3 TX slot %u: %d", self->tx_slot_id, rc);
    }
}

static void _handle_received(struct Sep3Zephyr *self)
{
    uint8_t *data = NULL;
    uint32_t size = 0;

    if (0 != atomic_set(&self->rx_error, 0)) {
        LOG_WRN("UART RX data was lost; waiting for the next valid SEP3 frame");
    }

    if (SEP3_ZEPHYR_STATE_STOPPING == atomic_get(&self->state)) {
        ring_buf_reset(&self->rx_ring);
        return;
    }

    do {
        size = ring_buf_get_claim(&self->rx_ring, &data, UINT16_MAX);
        if (0U < size) {
            int const rc = sep3__handle_received(&self->core, data, (uint16_t)size);
            if (0 != rc) {
                LOG_ERR("Failed to process UART RX data: %d", rc);
            }
            ring_buf_get_finish(&self->rx_ring, size);
        }
    } while (0U < size);
}

static void _check_tx_watchdog(struct Sep3Zephyr *self)
{
    if (0 == atomic_get(&self->tx_active) || 0 != atomic_get(&self->tx_abort_requested)) {
        return;
    }

    int64_t const timeout_ms = DIV_ROUND_UP((int64_t)self->config.uart_tx_timeout_us, USEC_PER_MSEC);
    if (k_uptime_get() - self->tx_started_ms < timeout_ms) {
        return;
    }

    if (!atomic_cas(&self->tx_abort_requested, 0, 1)) {
        return;
    }

    int const rc = uart_tx_abort(self->config.uart);
    if (0 != rc && -EFAULT != rc) {
        atomic_set(&self->tx_abort_requested, 0);
        LOG_ERR("Failed to abort timed out UART TX: %d", rc);
    }
}

static void _restart_rx(struct Sep3Zephyr *self)
{
    if (SEP3_ZEPHYR_STATE_RUNNING != atomic_get(&self->state) || 0 == atomic_get(&self->rx_disabled)) {
        return;
    }

    for (uint8_t i = 0; i < ARRAY_SIZE(self->rx_buffers); i++) {
        if (atomic_cas(&self->rx_buffer_owned[i], 0, 1)) {
            int const rc = uart_rx_enable(self->config.uart, self->rx_buffers[i], sizeof(self->rx_buffers[i]),
                (int32_t)self->config.uart_rx_timeout_us);
            if (0 == rc) {
                atomic_set(&self->rx_disabled, 0);
            } else {
                atomic_set(&self->rx_buffer_owned[i], 0);
                LOG_ERR("Failed to restart UART RX: %d", rc);
            }
            return;
        }
    }
}

static bool _finish_stop_if_ready(struct Sep3Zephyr *self)
{
    if (SEP3_ZEPHYR_STATE_STOPPING != atomic_get(&self->state) || 0 == atomic_get(&self->rx_disabled)) {
        return false;
    }

    ring_buf_reset(&self->rx_ring);
    int const rc = uart_callback_set(self->config.uart, _uart_noop_callback, NULL);

    struct Sep3ZephyrCommand *command = self->stop_command;
    self->stop_command = NULL;
    _complete_command(command, 0 == rc ? 0 : ER_IO);

    return true;
}

static void _thread_entry(void *arg1, void *arg2, void *arg3)
{
    struct Sep3Zephyr *self = arg1;

    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    while (true) {
        (void)k_sem_take(&self->event_sem, K_MSEC(CONFIG_SEP3_ZEPHYR_PROCESS_PERIOD_MS));

        _handle_tx_event(self);
        _handle_received(self);
        _check_tx_watchdog(self);

        if (0 != atomic_set(&self->tx_error, 0)) {
            LOG_ERR("Received overlapping UART TX completion events");
        }

        struct Sep3ZephyrCommand *command = NULL;
        while (0 == k_msgq_get(&self->command_queue, &command, K_NO_WAIT)) {
            _handle_command(self, command);
            if (SEP3_ZEPHYR_STATE_STOPPING == atomic_get(&self->state)) {
                break;
            }
        }

        int64_t const now_ms = k_uptime_get();
        if (0 <= now_ms) {
            int const rc = sep3__process(&self->core, (uint64_t)now_ms);
            if (0 != rc) {
                LOG_ERR("SEP3 processing failed: %d", rc);
            }
        }

        _restart_rx(self);
        if (_finish_stop_if_ready(self)) {
            return;
        }
    }
}

static int _rx_buffer_index(struct Sep3Zephyr *self, uint8_t const *buffer)
{
    for (uint8_t i = 0; i < ARRAY_SIZE(self->rx_buffers); i++) {
        if (self->rx_buffers[i] == buffer) {
            return i;
        }
    }

    return -1;
}

static void _provide_rx_buffer(struct Sep3Zephyr *self)
{
    for (uint8_t i = 0; i < ARRAY_SIZE(self->rx_buffers); i++) {
        if (atomic_cas(&self->rx_buffer_owned[i], 0, 1)) {
            int const rc = uart_rx_buf_rsp(self->config.uart, self->rx_buffers[i], sizeof(self->rx_buffers[i]));
            if (0 != rc) {
                atomic_set(&self->rx_buffer_owned[i], 0);
                atomic_set(&self->rx_error, 1);
            }
            return;
        }
    }

    atomic_set(&self->rx_error, 1);
}

static void _uart_callback(struct device const *device, struct uart_event *event, void *user_data)
{
    struct Sep3Zephyr *self = user_data;

    ARG_UNUSED(device);

    switch (event->type) {
        case UART_TX_DONE:
            if (!atomic_cas(&self->tx_event, SEP3_ZEPHYR_TX_EVENT_NONE, SEP3_ZEPHYR_TX_EVENT_DONE)) {
                atomic_set(&self->tx_error, 1);
            }
            break;
        case UART_TX_ABORTED:
            if (!atomic_cas(&self->tx_event, SEP3_ZEPHYR_TX_EVENT_NONE, SEP3_ZEPHYR_TX_EVENT_ABORTED)) {
                atomic_set(&self->tx_error, 1);
            }
            break;
        case UART_RX_RDY: {
            uint8_t const *data = event->data.rx.buf + event->data.rx.offset;
            uint32_t const written = ring_buf_put(&self->rx_ring, data, event->data.rx.len);
            if (written != event->data.rx.len) {
                atomic_set(&self->rx_error, 1);
            }
            break;
        }
        case UART_RX_BUF_REQUEST:
            _provide_rx_buffer(self);
            break;
        case UART_RX_BUF_RELEASED: {
            int const index = _rx_buffer_index(self, event->data.rx_buf.buf);
            if (0 <= index) {
                atomic_set(&self->rx_buffer_owned[index], 0);
            } else {
                atomic_set(&self->rx_error, 1);
            }
            break;
        }
        case UART_RX_DISABLED:
            atomic_set(&self->rx_disabled, 1);
            break;
        case UART_RX_STOPPED:
            atomic_set(&self->rx_error, 1);
            break;
        default:
            break;
    }

    k_sem_give(&self->event_sem);
}

static void _uart_noop_callback(struct device const *device, struct uart_event *event, void *user_data)
{
    ARG_UNUSED(device);
    ARG_UNUSED(event);
    ARG_UNUSED(user_data);
}

static int _submit_command(struct Sep3Zephyr *self, struct Sep3ZephyrCommand *command)
{
    int rc = 0;
    bool is_locked = false;
    bool is_enqueued = false;

    ASSERT(NULL != self, ER_INVAL);
    ASSERT(NULL != command, ER_INVAL);
    ASSERT(!k_is_in_isr(), ER_NOT_PERM);

    TRY(k_mutex_lock(&self->command_mutex, K_FOREVER));
    is_locked = true;
    ASSERT(SEP3_ZEPHYR_STATE_RUNNING == atomic_get(&self->state), ER_NO_DEV);

    k_sem_init(&command->completed, 0, 1);
    command->owner = self;
    command->result = 0;
    rc = k_msgq_put(&self->command_queue, &command, K_NO_WAIT);
    ASSERT(0 == rc, ER_AGAIN);
    atomic_inc(&self->command_count);
    is_enqueued = true;
    k_mutex_unlock(&self->command_mutex);
    is_locked = false;
    k_sem_give(&self->event_sem);
    TRY(k_sem_take(&command->completed, K_FOREVER));
    rc = command->result;
    atomic_dec(&self->command_count);
    is_enqueued = false;

finally:

    if (is_locked) {
        k_mutex_unlock(&self->command_mutex);
    }
    if (is_enqueued) {
        atomic_dec(&self->command_count);
    }

    return rc;
}

static int _queue_stop_command(struct Sep3Zephyr *self, struct Sep3ZephyrCommand *command)
{
    int rc = 0;
    bool is_locked = false;

    ASSERT(NULL != self, ER_INVAL);
    ASSERT(NULL != command, ER_INVAL);

    TRY(k_mutex_lock(&self->command_mutex, K_FOREVER));
    is_locked = true;
    ASSERT(SEP3_ZEPHYR_STATE_RUNNING == atomic_get(&self->state), ER_BUSY);
    ASSERT(0 == atomic_get(&self->command_count), ER_BUSY);

    atomic_set(&self->state, SEP3_ZEPHYR_STATE_STOPPING);
    k_sem_init(&command->completed, 0, 1);
    command->owner = self;
    command->result = 0;
    rc = k_msgq_put(&self->command_queue, &command, K_NO_WAIT);
    if (0 != rc) {
        atomic_set(&self->state, SEP3_ZEPHYR_STATE_RUNNING);
        rc = ER_AGAIN;
        goto finally;
    }

    k_mutex_unlock(&self->command_mutex);
    is_locked = false;
    k_sem_give(&self->event_sem);

finally:

    if (is_locked) {
        k_mutex_unlock(&self->command_mutex);
    }

    return rc;
}

int sep3_zephyr_init(struct Sep3Zephyr *self, struct Sep3ZephyrConfig const *config)
{
    int rc = 0;
    struct Sep3Config core_config = {0};
    bool callback_set = false;
    bool rx_enabled = false;
    bool thread_created = false;
    bool init_locked = false;
    bool uart_claimed = false;

    ASSERT(NULL != self, ER_INVAL);
    ASSERT(NULL != config, ER_INVAL);
    ASSERT(!k_is_in_isr(), ER_NOT_PERM);
    ASSERT(NULL != config->uart, ER_INVAL);
    ASSERT(device_is_ready(config->uart), ER_NO_DEV);
    ASSERT(0U < config->request_timeout_ms, ER_INVAL);
    ASSERT(0U < config->incoming_request_timeout_ms, ER_INVAL);
    ASSERT(0U < config->uart_rx_timeout_us && config->uart_rx_timeout_us <= INT32_MAX, ER_INVAL);
    ASSERT(0U < config->uart_tx_timeout_us && config->uart_tx_timeout_us <= INT32_MAX, ER_INVAL);
    TRY(k_mutex_lock(&g_init_mutex, K_FOREVER));
    init_locked = true;
    ASSERT(SEP3_ZEPHYR_STATE_UNINITIALIZED == atomic_get(&self->state) ||
        SEP3_ZEPHYR_STATE_STOPPED == atomic_get(&self->state), ER_ALREADY);

    memset(self, 0, sizeof(*self));
    self->uart_claim_index = -1;
    self->config = *config;
    k_sem_init(&self->event_sem, 0, K_SEM_MAX_LIMIT);
    k_sem_init(&self->active_calls_done, 0, 1);
    k_mutex_init(&self->command_mutex);
    k_mutex_init(&self->request_mutex);
    k_msgq_init(&self->command_queue, (char *)self->command_queue_buffer,
        sizeof(self->command_queue_buffer[0]), ARRAY_SIZE(self->command_queue_buffer));
    ring_buf_init(&self->rx_ring, sizeof(self->rx_ring_buffer), self->rx_ring_buffer);
    TRY(_claim_uart(self, config->uart));
    uart_claimed = true;

    struct uart_driver_api const *uart_api = config->uart->api;
    ASSERT(NULL != uart_api->tx_abort, ER_NOT_SUPPORTED);

    core_config.buffers = &self->buffers;
    core_config.endpoints = self->core_endpoints;
    core_config.endpoint_capacity = ARRAY_SIZE(self->core_endpoints);
    core_config.request_timeout_ms = config->request_timeout_ms;
    core_config.incoming_request_timeout_ms = config->incoming_request_timeout_ms;
    core_config.token_epoch = _next_token_epoch();
    core_config.retry_count = config->retry_count;
    core_config.transmit = _transmit;
    core_config.transmit_user = self;
    TRY(sep3__init(&self->core, &core_config));

    self->thread_id = k_thread_create(&self->thread, self->thread_stack,
        K_KERNEL_STACK_SIZEOF(self->thread_stack), _thread_entry, self, NULL, NULL,
        K_PRIO_PREEMPT(CONFIG_SEP3_ZEPHYR_THREAD_PRIORITY), 0, K_FOREVER);
    ASSERT(NULL != self->thread_id, ER_NO_MEM);
    thread_created = true;

    TRY(uart_callback_set(config->uart, _uart_callback, self));
    callback_set = true;

    atomic_set(&self->rx_buffer_owned[0], 1);
    TRY(uart_rx_enable(config->uart, self->rx_buffers[0], sizeof(self->rx_buffers[0]),
        (int32_t)config->uart_rx_timeout_us));
    rx_enabled = true;

    atomic_set(&self->state, SEP3_ZEPHYR_STATE_RUNNING);
    if (NULL != config->thread_name) {
        int const name_rc = k_thread_name_set(self->thread_id, config->thread_name);
        if (0 != name_rc) {
            LOG_WRN("Failed to set SEP3 thread name: %d", name_rc);
        }
    }
    k_thread_start(self->thread_id);

finally:

    if (0 != rc) {
        if (rx_enabled) {
            (void)uart_rx_disable(config->uart);
        }
        if (callback_set) {
            (void)uart_callback_set(config->uart, _uart_noop_callback, NULL);
        }
        if (thread_created) {
            k_thread_abort(self->thread_id);
            (void)k_thread_join(self->thread_id, K_FOREVER);
        }
        if (uart_claimed) {
            _release_uart(self);
        }
        atomic_set(&self->state, SEP3_ZEPHYR_STATE_UNINITIALIZED);
    }

    if (init_locked) {
        k_mutex_unlock(&g_init_mutex);
    }

    return rc;
}

int sep3_zephyr_deinit(struct Sep3Zephyr *self)
{
    int rc = 0;
    int stop_rc = 0;
    bool init_locked = false;
    bool request_locked = false;
    struct Sep3ZephyrCommand command = {.type = SEP3_ZEPHYR_COMMAND_STOP};

    ASSERT(NULL != self, ER_INVAL);
    ASSERT(!k_is_in_isr(), ER_NOT_PERM);
    TRY(k_mutex_lock(&g_init_mutex, K_FOREVER));
    init_locked = true;
    ASSERT(SEP3_ZEPHYR_STATE_RUNNING == atomic_get(&self->state), ER_NO_DEV);
    ASSERT(self->thread_id != k_current_get(), ER_BUSY);
    rc = k_mutex_lock(&self->request_mutex, K_NO_WAIT);
    ASSERT(0 == rc, ER_BUSY);
    request_locked = true;
    TRY(_queue_stop_command(self, &command));
    k_mutex_unlock(&g_init_mutex);
    init_locked = false;

    TRY(k_sem_take(&command.completed, K_FOREVER));
    stop_rc = command.result;
    if (SEP3_ZEPHYR_STATE_STOPPING == atomic_get(&self->state)) {
        TRY(k_thread_join(self->thread_id, K_FOREVER));
        TRY(k_mutex_lock(&g_init_mutex, K_FOREVER));
        init_locked = true;
        _release_uart(self);
        k_mutex_unlock(&self->request_mutex);
        request_locked = false;
        while (0 != atomic_get(&self->active_call_count)) {
            TRY(k_sem_take(&self->active_calls_done, K_FOREVER));
        }
        atomic_set(&self->state, SEP3_ZEPHYR_STATE_STOPPED);
    }
    rc = stop_rc;

finally:

    if (request_locked) {
        k_mutex_unlock(&self->request_mutex);
    }
    if (init_locked) {
        k_mutex_unlock(&g_init_mutex);
    }

    return rc;
}

int sep3_zephyr_register_read_handler(
    struct Sep3Zephyr *self,
    DataId const data_id,
    Sep3ZephyrReadHandler const handler,
    void *const user)
{
    int rc = 0;
    bool api_entered = false;
    struct Sep3ZephyrCommand command = {
        .type = SEP3_ZEPHYR_COMMAND_REGISTER_READ,
        .args.register_read = {.data_id = data_id, .handler = handler, .user = user},
    };

    ASSERT(NULL != self, ER_INVAL);
    ASSERT(NULL != handler, ER_INVAL);
    TRY(_enter_api(self));
    api_entered = true;
    ASSERT(self->thread_id != k_current_get(), ER_BUSY);
    TRY(_submit_command(self, &command));

finally:

    if (api_entered) {
        _leave_api(self);
    }

    return rc;
}

int sep3_zephyr_register_write_handler(
    struct Sep3Zephyr *self,
    DataId const data_id,
    bool const allow_write_no_answer,
    Sep3ZephyrWriteHandler const handler,
    void *const user)
{
    int rc = 0;
    bool api_entered = false;
    struct Sep3ZephyrCommand command = {
        .type = SEP3_ZEPHYR_COMMAND_REGISTER_WRITE,
        .args.register_write = {
            .data_id = data_id,
            .allow_write_no_answer = allow_write_no_answer,
            .handler = handler,
            .user = user,
        },
    };

    ASSERT(NULL != self, ER_INVAL);
    ASSERT(NULL != handler, ER_INVAL);
    TRY(_enter_api(self));
    api_entered = true;
    ASSERT(self->thread_id != k_current_get(), ER_BUSY);
    TRY(_submit_command(self, &command));

finally:

    if (api_entered) {
        _leave_api(self);
    }

    return rc;
}

int sep3_zephyr_read(
    struct Sep3Zephyr *self,
    DataId const data_id,
    uint8_t *const data,
    uint16_t const data_capacity,
    uint16_t *const data_size,
    struct Sep3ZephyrRequestResult *const result)
{
    int rc = 0;
    bool api_entered = false;
    struct Sep3ZephyrCommand command = {
        .type = SEP3_ZEPHYR_COMMAND_READ,
        .args.read = {
            .data_id = data_id,
            .data = data,
            .data_capacity = data_capacity,
            .data_size = data_size,
            .result = result,
        },
    };

    ASSERT(NULL != self, ER_INVAL);
    ASSERT(NULL != data_size, ER_INVAL);
    ASSERT(NULL != result, ER_INVAL);
    ASSERT(0U == data_capacity || NULL != data, ER_INVAL);
    ASSERT(!k_is_in_isr(), ER_NOT_PERM);
    TRY(_enter_api(self));
    api_entered = true;
    ASSERT(self->thread_id != k_current_get(), ER_BUSY);

    memset(result, 0, sizeof(*result));
    *data_size = 0;
    TRY(k_mutex_lock(&self->request_mutex, K_FOREVER));
    rc = _submit_command(self, &command);
    k_mutex_unlock(&self->request_mutex);

finally:

    if (api_entered) {
        _leave_api(self);
    }

    return rc;
}

int sep3_zephyr_write(
    struct Sep3Zephyr *self,
    DataId const data_id,
    uint8_t const *const data,
    uint16_t const data_size,
    struct Sep3ZephyrRequestResult *const result)
{
    int rc = 0;
    bool api_entered = false;
    struct Sep3ZephyrCommand command = {
        .type = SEP3_ZEPHYR_COMMAND_WRITE,
        .args.write = {.data_id = data_id, .data = data, .data_size = data_size, .result = result},
    };

    ASSERT(NULL != self, ER_INVAL);
    ASSERT(NULL != result, ER_INVAL);
    ASSERT(0U == data_size || NULL != data, ER_INVAL);
    ASSERT(!k_is_in_isr(), ER_NOT_PERM);
    TRY(_enter_api(self));
    api_entered = true;
    ASSERT(self->thread_id != k_current_get(), ER_BUSY);

    memset(result, 0, sizeof(*result));
    TRY(k_mutex_lock(&self->request_mutex, K_FOREVER));
    rc = _submit_command(self, &command);
    k_mutex_unlock(&self->request_mutex);

finally:

    if (api_entered) {
        _leave_api(self);
    }

    return rc;
}

int sep3_zephyr_write_no_answer(
    struct Sep3Zephyr *self,
    DataId const data_id,
    uint8_t const *const data,
    uint16_t const data_size)
{
    int rc = 0;
    bool api_entered = false;
    struct Sep3ZephyrCommand command = {
        .type = SEP3_ZEPHYR_COMMAND_WRITE_NO_ANSWER,
        .args.write_no_answer = {.data_id = data_id, .data = data, .data_size = data_size},
    };

    ASSERT(NULL != self, ER_INVAL);
    ASSERT(0U == data_size || NULL != data, ER_INVAL);
    TRY(_enter_api(self));
    api_entered = true;
    ASSERT(self->thread_id != k_current_get(), ER_BUSY);
    TRY(_submit_command(self, &command));

finally:

    if (api_entered) {
        _leave_api(self);
    }

    return rc;
}

int sep3_zephyr_send_read_answer(
    struct Sep3Zephyr *self,
    struct Sep3RequestToken const *const token,
    uint8_t const *const data,
    uint16_t const data_size)
{
    int rc = 0;
    bool api_entered = false;
    struct Sep3ZephyrCommand command = {
        .type = SEP3_ZEPHYR_COMMAND_SEND_READ_ANSWER,
        .args.read_answer = {.data = data, .data_size = data_size},
    };

    ASSERT(NULL != self, ER_INVAL);
    ASSERT(NULL != token, ER_INVAL);
    ASSERT(0U == data_size || NULL != data, ER_INVAL);
    TRY(_enter_api(self));
    api_entered = true;

    if (self->thread_id == k_current_get()) {
        rc = sep3__send_read_answer(&self->core, token, data, data_size);
    } else {
        command.args.read_answer.token = *token;
        rc = _submit_command(self, &command);
    }

finally:

    if (api_entered) {
        _leave_api(self);
    }

    return rc;
}

int sep3_zephyr_send_write_answer(
    struct Sep3Zephyr *self,
    struct Sep3RequestToken const *const token)
{
    int rc = 0;
    bool api_entered = false;
    struct Sep3ZephyrCommand command = {.type = SEP3_ZEPHYR_COMMAND_SEND_WRITE_ANSWER};

    ASSERT(NULL != self, ER_INVAL);
    ASSERT(NULL != token, ER_INVAL);
    TRY(_enter_api(self));
    api_entered = true;

    if (self->thread_id == k_current_get()) {
        rc = sep3__send_write_answer(&self->core, token);
    } else {
        command.args.write_answer.token = *token;
        rc = _submit_command(self, &command);
    }

finally:

    if (api_entered) {
        _leave_api(self);
    }

    return rc;
}

int sep3_zephyr_send_error_answer(
    struct Sep3Zephyr *self,
    struct Sep3RequestToken const *const token,
    uint8_t const error_code,
    char const *const message)
{
    int rc = 0;
    bool api_entered = false;
    struct Sep3ZephyrCommand command = {
        .type = SEP3_ZEPHYR_COMMAND_SEND_ERROR_ANSWER,
        .args.error_answer = {.error_code = error_code, .message = message},
    };

    ASSERT(NULL != self, ER_INVAL);
    ASSERT(NULL != token, ER_INVAL);
    TRY(_enter_api(self));
    api_entered = true;

    if (self->thread_id == k_current_get()) {
        rc = sep3__send_error_answer(&self->core, token, error_code, message);
    } else {
        command.args.error_answer.token = *token;
        rc = _submit_command(self, &command);
    }

finally:

    if (api_entered) {
        _leave_api(self);
    }

    return rc;
}
