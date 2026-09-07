#include <sep3-zephyr/sep3_zephyr.h>

#include <safe_c.h>

#include <zephyr/drivers/serial/uart_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include <string.h>

enum {
    TEST_DATA_ID = 0x21,
    TEST_TIMEOUT_MS = 100,
};

struct TestLink {
    struct device const *peer;
    atomic_t drop;
    atomic_t error;
};

struct sep3_fixture {
    struct device const *uart_a;
    struct device const *uart_b;
    struct Sep3Zephyr peer_a;
    struct Sep3Zephyr peer_b;
    struct TestLink a_to_b;
    struct TestLink b_to_a;
    struct k_sem write_received;
    uint8_t received[SEP3_MAX_PAYLOAD_SIZE];
    uint16_t received_size;
    bool answer_expected;
};

struct ReadThreadArgs {
    struct Sep3Zephyr *peer;
    int rc;
    uint8_t data[sizeof(uint32_t)];
    uint16_t data_size;
    struct Sep3ZephyrRequestResult result;
};

static struct sep3_fixture g_fixture = {
    .uart_a = DEVICE_DT_GET(DT_NODELABEL(sep3_uart_a)),
    .uart_b = DEVICE_DT_GET(DT_NODELABEL(sep3_uart_b)),
};
static struct Sep3Zephyr g_duplicate_peer;
static struct k_thread g_read_thread_a;
static struct k_thread g_read_thread_b;
K_THREAD_STACK_DEFINE(g_read_thread_stack_a, 2048);
K_THREAD_STACK_DEFINE(g_read_thread_stack_b, 2048);

static uint8_t const g_read_answer[] = {0x11, 0x22, 0x33, 0x44};

static void _forward_uart(struct device const *device, size_t size, void *user_data)
{
    struct TestLink *link = user_data;
    uint8_t data[64];

    while (0U < size) {
        size_t const requested = MIN(size, sizeof(data));
        uint32_t const read_size = uart_emul_get_tx_data(device, data, requested);

        if (0U == read_size) {
            atomic_set(&link->error, 1);
            return;
        }

        if (0 == atomic_get(&link->drop)) {
            uint32_t const written = uart_emul_put_rx_data(link->peer, data, read_size);
            if (written != read_size) {
                atomic_set(&link->error, 1);
                return;
            }
        }

        size -= read_size;
    }
}

static void _read_handler(
    struct Sep3Zephyr *self,
    struct Sep3RequestToken const *token,
    void *user)
{
    int *handler_rc = user;

    *handler_rc = sep3_zephyr_send_read_answer(self, token, g_read_answer, sizeof(g_read_answer));
}

static void _error_handler(
    struct Sep3Zephyr *self,
    struct Sep3RequestToken const *token,
    void *user)
{
    int *handler_rc = user;

    *handler_rc = sep3_zephyr_send_error_answer(self, token, 7, "remote error");
}

static void _write_handler(
    struct Sep3Zephyr *self,
    struct Sep3RequestToken const *token,
    uint8_t const *data,
    uint16_t data_size,
    bool is_need_answer,
    void *user)
{
    struct sep3_fixture *fixture = user;

    fixture->received_size = data_size;
    fixture->answer_expected = is_need_answer;
    memmove(fixture->received, data, data_size);
    k_sem_give(&fixture->write_received);

    if (is_need_answer) {
        int const rc = sep3_zephyr_send_write_answer(self, token);
        if (0 != rc) {
            atomic_set(&fixture->b_to_a.error, 1);
        }
    }
}

static void _read_thread_entry(void *arg1, void *arg2, void *arg3)
{
    struct ReadThreadArgs *args = arg1;

    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    args->rc = sep3_zephyr_read(args->peer, TEST_DATA_ID, args->data, sizeof(args->data),
        &args->data_size, &args->result);
}

static struct Sep3ZephyrConfig _config(struct device const *uart, char const *name)
{
    struct Sep3ZephyrConfig config = {
        .uart = uart,
        .request_timeout_ms = TEST_TIMEOUT_MS,
        .incoming_request_timeout_ms = TEST_TIMEOUT_MS,
        .uart_rx_timeout_us = 1000,
        .uart_tx_timeout_us = 100000,
        .retry_count = 0,
        .thread_name = name,
    };

    return config;
}

static int _deinit_eventually(struct Sep3Zephyr *peer)
{
    int rc = ER_BUSY;
    int64_t const deadline = k_uptime_get() + 1000;

    while (ER_BUSY == rc && k_uptime_get() < deadline) {
        rc = sep3_zephyr_deinit(peer);
        if (ER_BUSY == rc) {
            k_sleep(K_MSEC(1));
        }
    }

    return rc;
}

static void *sep3_setup(void)
{
    zassert_true(device_is_ready(g_fixture.uart_a));
    zassert_true(device_is_ready(g_fixture.uart_b));

    return &g_fixture;
}

static void sep3_before(void *user_data)
{
    struct sep3_fixture *fixture = user_data;
    struct Sep3ZephyrConfig config_a = _config(fixture->uart_a, "sep3-test-a");
    struct Sep3ZephyrConfig config_b = _config(fixture->uart_b, "sep3-test-b");

    memset(fixture->received, 0, sizeof(fixture->received));
    fixture->received_size = 0;
    fixture->answer_expected = false;
    k_sem_init(&fixture->write_received, 0, 1);

    fixture->a_to_b.peer = fixture->uart_b;
    fixture->b_to_a.peer = fixture->uart_a;
    atomic_set(&fixture->a_to_b.drop, 0);
    atomic_set(&fixture->a_to_b.error, 0);
    atomic_set(&fixture->b_to_a.drop, 0);
    atomic_set(&fixture->b_to_a.error, 0);

    (void)uart_emul_flush_rx_data(fixture->uart_a);
    (void)uart_emul_flush_tx_data(fixture->uart_a);
    (void)uart_emul_flush_rx_data(fixture->uart_b);
    (void)uart_emul_flush_tx_data(fixture->uart_b);
    uart_emul_callback_tx_data_ready_set(fixture->uart_a, _forward_uart, &fixture->a_to_b);
    uart_emul_callback_tx_data_ready_set(fixture->uart_b, _forward_uart, &fixture->b_to_a);

    zassert_ok(sep3_zephyr_init(&fixture->peer_a, &config_a));
    zassert_ok(sep3_zephyr_init(&fixture->peer_b, &config_b));
}

static void sep3_after(void *user_data)
{
    struct sep3_fixture *fixture = user_data;

    zassert_ok(_deinit_eventually(&fixture->peer_a));
    zassert_ok(_deinit_eventually(&fixture->peer_b));
    uart_emul_callback_tx_data_ready_set(fixture->uart_a, NULL, NULL);
    uart_emul_callback_tx_data_ready_set(fixture->uart_b, NULL, NULL);
    zassert_equal(atomic_get(&fixture->a_to_b.error), 0);
    zassert_equal(atomic_get(&fixture->b_to_a.error), 0);
}

ZTEST_F(sep3, test_read)
{
    int handler_rc = ER_1;
    uint8_t data[sizeof(g_read_answer)] = {0};
    uint16_t data_size = 0;
    struct Sep3ZephyrRequestResult result = {0};

    zassert_ok(sep3_zephyr_register_read_handler(&fixture->peer_b, TEST_DATA_ID, _read_handler, &handler_rc));
    zassert_ok(sep3_zephyr_read(&fixture->peer_a, TEST_DATA_ID, data, sizeof(data), &data_size, &result));
    zassert_ok(handler_rc);
    zassert_equal(result.answer_type, SEP3_PACKET_READ_ANSWER);
    zassert_equal(data_size, sizeof(g_read_answer));
    zassert_mem_equal(data, g_read_answer, sizeof(g_read_answer));
}

ZTEST_F(sep3, test_concurrent_reads_are_serialized)
{
    int handler_rc = ER_1;
    struct ReadThreadArgs args_a = {.peer = &fixture->peer_a, .rc = ER_1};
    struct ReadThreadArgs args_b = {.peer = &fixture->peer_a, .rc = ER_1};

    zassert_ok(sep3_zephyr_register_read_handler(&fixture->peer_b, TEST_DATA_ID, _read_handler, &handler_rc));
    k_thread_create(&g_read_thread_a, g_read_thread_stack_a, K_THREAD_STACK_SIZEOF(g_read_thread_stack_a),
        _read_thread_entry, &args_a, NULL, NULL, K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
    k_thread_create(&g_read_thread_b, g_read_thread_stack_b, K_THREAD_STACK_SIZEOF(g_read_thread_stack_b),
        _read_thread_entry, &args_b, NULL, NULL, K_PRIO_PREEMPT(0), 0, K_NO_WAIT);

    zassert_ok(k_thread_join(&g_read_thread_a, K_SECONDS(1)));
    zassert_ok(k_thread_join(&g_read_thread_b, K_SECONDS(1)));
    zassert_ok(args_a.rc);
    zassert_ok(args_b.rc);
    zassert_equal(args_a.result.answer_type, SEP3_PACKET_READ_ANSWER);
    zassert_equal(args_b.result.answer_type, SEP3_PACKET_READ_ANSWER);
    zassert_mem_equal(args_a.data, g_read_answer, sizeof(g_read_answer));
    zassert_mem_equal(args_b.data, g_read_answer, sizeof(g_read_answer));
    zassert_ok(handler_rc);
}

ZTEST_F(sep3, test_reinitialize_stopped_instance)
{
    struct Sep3ZephyrConfig config = _config(fixture->uart_a, "sep3-test-a");

    zassert_ok(_deinit_eventually(&fixture->peer_a));
    zassert_equal(sep3_zephyr_write_no_answer(&fixture->peer_a, TEST_DATA_ID, NULL, 0), ER_NO_DEV);
    zassert_ok(sep3_zephyr_init(&fixture->peer_a, &config));
}

ZTEST_F(sep3, test_rejects_duplicate_uart_owner)
{
    struct Sep3ZephyrConfig config = _config(fixture->uart_a, "sep3-duplicate");

    zassert_equal(sep3_zephyr_init(&g_duplicate_peer, &config), ER_BUSY);
}

ZTEST_F(sep3, test_read_overflow)
{
    int handler_rc = ER_1;
    uint8_t data[2] = {0};
    uint16_t data_size = 0;
    struct Sep3ZephyrRequestResult result = {0};

    zassert_ok(sep3_zephyr_register_read_handler(&fixture->peer_b, TEST_DATA_ID, _read_handler, &handler_rc));
    zassert_equal(sep3_zephyr_read(&fixture->peer_a, TEST_DATA_ID, data, sizeof(data), &data_size, &result),
        ER_OVERFLOW);
    zassert_ok(handler_rc);
    zassert_equal(result.answer_type, SEP3_PACKET_READ_ANSWER);
    zassert_equal(data_size, sizeof(g_read_answer));
    zassert_mem_equal(data, (uint8_t[2]){0}, sizeof(data));
}

ZTEST_F(sep3, test_write_max_payload)
{
    uint8_t data[SEP3_MAX_PAYLOAD_SIZE];
    struct Sep3ZephyrRequestResult result = {0};

    for (uint16_t i = 0; i < sizeof(data); i++) {
        data[i] = (uint8_t)i;
    }

    zassert_ok(sep3_zephyr_register_write_handler(&fixture->peer_b, TEST_DATA_ID, true,
        _write_handler, fixture));
    zassert_ok(sep3_zephyr_write(&fixture->peer_a, TEST_DATA_ID, data, sizeof(data), &result));
    zassert_equal(result.answer_type, SEP3_PACKET_WRITE_ANSWER);
    zassert_equal(fixture->received_size, sizeof(data));
    zassert_true(fixture->answer_expected);
    zassert_mem_equal(fixture->received, data, sizeof(data));
}

ZTEST_F(sep3, test_write_rejects_oversize_payload)
{
    uint8_t data[SEP3_MAX_PAYLOAD_SIZE + 1] = {0};
    struct Sep3ZephyrRequestResult result = {0};

    zassert_equal(sep3_zephyr_write(&fixture->peer_a, TEST_DATA_ID, data, sizeof(data), &result), ER_OVERFLOW);
}

ZTEST_F(sep3, test_remote_error)
{
    int handler_rc = ER_1;
    uint16_t data_size = 0;
    struct Sep3ZephyrRequestResult result = {0};

    zassert_ok(sep3_zephyr_register_read_handler(&fixture->peer_b, TEST_DATA_ID, _error_handler, &handler_rc));
    zassert_ok(sep3_zephyr_read(&fixture->peer_a, TEST_DATA_ID, NULL, 0, &data_size, &result));
    zassert_ok(handler_rc);
    zassert_equal(result.answer_type, SEP3_PACKET_APP_ERROR_ANSWER);
    zassert_equal(result.remote_error_code, 7);
    zassert_equal(strcmp(result.remote_error_message, "remote error"), 0);
}

ZTEST_F(sep3, test_timeout)
{
    uint16_t data_size = 0;
    struct Sep3ZephyrRequestResult result = {0};

    atomic_set(&fixture->a_to_b.drop, 1);
    zassert_equal(sep3_zephyr_read(&fixture->peer_a, TEST_DATA_ID, NULL, 0, &data_size, &result),
        ER_TIMEDOUT);
}

ZTEST_F(sep3, test_write_no_answer)
{
    uint8_t const data[] = {1, 2, 3};

    zassert_ok(sep3_zephyr_register_write_handler(&fixture->peer_b, TEST_DATA_ID, true,
        _write_handler, fixture));
    zassert_ok(sep3_zephyr_write_no_answer(&fixture->peer_a, TEST_DATA_ID, data, sizeof(data)));
    zassert_ok(k_sem_take(&fixture->write_received, K_SECONDS(1)));
    zassert_false(fixture->answer_expected);
    zassert_equal(fixture->received_size, sizeof(data));
    zassert_mem_equal(fixture->received, data, sizeof(data));
}

ZTEST_SUITE(sep3, NULL, sep3_setup, sep3_before, sep3_after, NULL);
