#include <sep3-zephyr/sep3_zephyr.h>

#include <crc32.h>
#include <safe_c.h>

#include <zephyr/drivers/serial/uart_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include <string.h>

LOG_MODULE_REGISTER(sep3_test);

enum {
    TEST_DATA_ID = 0x21,
    HOLD_DATA_ID = 0x22,
    TEST_TIMEOUT_MS = 100,
    TEST_TRANSACTION_ID = 1,
    TEST_PACKET_TYPE_OFFSET = 0,
    TEST_PACKET_TRANSACTION_ID_OFFSET = 1,
    TEST_PACKET_DATA_ID_OFFSET = 3,
    TEST_PACKET_PAYLOAD_OFFSET = 4,
    TEST_PACKET_CHECKSUM_SIZE = 4,
    TEST_PACKET_MIN_SIZE = SEP3_PACKET_OVERHEAD_SIZE,
};

enum ReadMode {
    READ_MODE_SUCCESS = 0,
    READ_MODE_ERROR,
    READ_MODE_HOLD,
};

enum DedupWriteMode {
    DEDUP_WRITE_ANSWER = 0,
    DEDUP_WRITE_DEFER,
};

struct TestLink {
    struct device const *peer;
    atomic_t drop;
    atomic_t error;
};

struct CoreTransport {
    struct device const *uart;
    uint8_t slot_id;
};

struct DedupTransport {
    uint8_t frame[SEP3_MAX_ENCODED_FRAME_SIZE];
    uint16_t frame_size;
    uint8_t slot_id;
    uint8_t transmit_count;
};

struct DedupEndpoint {
    struct Sep3RequestToken token;
    uint8_t received[SEP3_MAX_PAYLOAD_SIZE];
    uint16_t received_size;
    uint16_t write_count;
    uint16_t read_count;
    int handler_rc;
    enum DedupWriteMode write_mode;
    bool payload_from_rx;
};

struct sep3_fixture {
    struct device const *uart_a;
    struct device const *uart_b;
    struct Sep3Zephyr peer_a;
    struct Sep3Zephyr peer_b;
    struct TestLink a_to_b;
    struct TestLink b_to_a;
    struct k_sem write_received;
    struct k_sem read_held;
    uint8_t received[SEP3_MAX_PAYLOAD_SIZE];
    uint16_t received_size;
    bool answer_expected;
    enum ReadMode read_mode;
    int handler_rc;
    struct Sep3RequestToken held_token;
};

struct ReadThreadArgs {
    struct Sep3Zephyr *peer;
    int rc;
    uint8_t data[sizeof(uint32_t)];
    uint16_t data_size;
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

static uint8_t g_max_payload[SEP3_MAX_PAYLOAD_SIZE];
static uint8_t g_oversize_payload[SEP3_MAX_PAYLOAD_SIZE + 1];
static struct Sep3 g_protocol_error_core;
static struct Sep3Buffers g_protocol_error_buffers;
static struct Sep3 g_tx_busy_core;
static struct Sep3Buffers g_tx_busy_buffers;
static uint8_t const g_read_answer[] = {0x11, 0x22, 0x33, 0x44};
static struct Sep3 g_dedup_core;
static struct Sep3Buffers g_dedup_buffers;
static struct Sep3Endpoint g_dedup_endpoints[1];
static struct DedupTransport g_dedup_transport;
static struct DedupEndpoint g_dedup_endpoint;
static uint8_t g_dedup_frame[SEP3_MAX_ENCODED_FRAME_SIZE];
static uint8_t g_dedup_answer_packet[SEP3_MAX_PACKET_SIZE];
static uint8_t g_dedup_payload[SEP3_MAX_PAYLOAD_SIZE];
static uint32_t g_dedup_epoch;

BUILD_ASSERT(SEP3_MAX_PAYLOAD_SIZE == CONFIG_SEP3_MAX_PAYLOAD_SIZE,
    "SEP3 payload configuration mismatch");
BUILD_ASSERT(DT_PROP(DT_NODELABEL(sep3_uart_a), rx_fifo_size) >= SEP3_MAX_ENCODED_FRAME_SIZE,
    "sep3_uart_a RX FIFO is too small");
BUILD_ASSERT(DT_PROP(DT_NODELABEL(sep3_uart_a), tx_fifo_size) >= SEP3_MAX_ENCODED_FRAME_SIZE,
    "sep3_uart_a TX FIFO is too small");
BUILD_ASSERT(DT_PROP(DT_NODELABEL(sep3_uart_b), rx_fifo_size) >= SEP3_MAX_ENCODED_FRAME_SIZE,
    "sep3_uart_b RX FIFO is too small");
BUILD_ASSERT(DT_PROP(DT_NODELABEL(sep3_uart_b), tx_fifo_size) >= SEP3_MAX_ENCODED_FRAME_SIZE,
    "sep3_uart_b TX FIFO is too small");
BUILD_ASSERT(1 == SEP3_TX_SLOT_COUNT, "SEP3 core is configured with one TX slot");

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
    struct sep3_fixture *fixture = user;

    switch (fixture->read_mode) {
        case READ_MODE_SUCCESS:
            fixture->handler_rc = sep3_zephyr_send_read_answer(
                self, token, g_read_answer, sizeof(g_read_answer));
            break;
        case READ_MODE_ERROR:
            fixture->handler_rc = sep3_zephyr_send_error_answer(self, token, 7, "remote error");
            break;
        case READ_MODE_HOLD:
            fixture->held_token = *token;
            fixture->handler_rc = 0;
            k_sem_give(&fixture->read_held);
            break;
        default:
            fixture->handler_rc = ER_INVAL;
            break;
    }
}

static int _core_transmit(
    struct Sep3 *core,
    uint8_t const *frame,
    uint16_t const frame_size,
    uint8_t const slot_id,
    void *user)
{
    struct CoreTransport *transport = user;
    uint32_t const written = uart_emul_put_rx_data(transport->uart, frame, frame_size);

    ARG_UNUSED(core);
    transport->slot_id = slot_id;
    if ((uint32_t)frame_size != written) {
        return ER_IO;
    }

    return 0;
}

static void _core_request_callback(
    struct Sep3 *core,
    struct Sep3RequestResult const *result,
    void *user)
{
    ARG_UNUSED(core);
    ARG_UNUSED(result);
    ARG_UNUSED(user);
}

static int _busy_core_transmit(
    struct Sep3 *core,
    uint8_t const *frame,
    uint16_t frame_size,
    uint8_t slot_id,
    void *user)
{
    ARG_UNUSED(core);
    ARG_UNUSED(frame);
    ARG_UNUSED(frame_size);
    ARG_UNUSED(slot_id);
    ARG_UNUSED(user);

    return ER_AGAIN;
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

static void _test_write_u32_le(uint8_t *const data, uint32_t const value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static int _build_test_packet(
    uint8_t *const packet,
    uint16_t const packet_capacity,
    Sep3PacketType const type,
    TransactionId const transaction_id,
    DataId const data_id,
    uint8_t const *const payload,
    uint16_t const payload_size,
    uint16_t *const packet_size)
{
    int rc = 0;
    uint16_t size = 0;
    uint32_t crc = 0;

    ASSERT(NULL != packet, ER_INVAL);
    ASSERT(NULL != packet_size, ER_INVAL);
    ASSERT(0U == payload_size || NULL != payload, ER_INVAL);
    ASSERT(payload_size <= SEP3_MAX_PAYLOAD_SIZE, ER_OVERFLOW);

    size = (uint16_t)(TEST_PACKET_MIN_SIZE + payload_size);
    ASSERT(size <= packet_capacity, ER_OVERFLOW);

    packet[TEST_PACKET_TYPE_OFFSET] = (uint8_t)type;
    packet[TEST_PACKET_TRANSACTION_ID_OFFSET] = (uint8_t)(transaction_id & 0xFFU);
    packet[TEST_PACKET_TRANSACTION_ID_OFFSET + 1U] = (uint8_t)(transaction_id >> 8);
    packet[TEST_PACKET_DATA_ID_OFFSET] = data_id;
    if (0U < payload_size) {
        memmove(&packet[TEST_PACKET_PAYLOAD_OFFSET], payload, payload_size);
    }

    crc = crc32__ieee(packet, (size_t)(size - TEST_PACKET_CHECKSUM_SIZE));
    _test_write_u32_le(&packet[size - TEST_PACKET_CHECKSUM_SIZE], crc);
    *packet_size = size;

finally:

    return rc;
}

static int _build_test_frame(
    uint8_t *const frame,
    uint16_t const frame_capacity,
    Sep3PacketType const type,
    TransactionId const transaction_id,
    DataId const data_id,
    uint8_t const *const payload,
    uint16_t const payload_size,
    uint16_t *const frame_size)
{
    int rc = 0;
    uint16_t packet_size = 0;
    int encoded_size = 0;

    ASSERT(NULL != frame, ER_INVAL);
    ASSERT(NULL != frame_size, ER_INVAL);

    TRY(_build_test_packet(frame, frame_capacity, type, transaction_id, data_id, payload, payload_size,
        &packet_size));

    encoded_size = framer7b__encode_in_place(frame, packet_size, frame_capacity);
    ASSERT(0 < encoded_size, ER_OVERFLOW);
    *frame_size = (uint16_t)encoded_size;

finally:

    return rc;
}

static int _decode_test_frame(
    uint8_t const *const frame,
    uint16_t const frame_size,
    uint8_t *const packet,
    uint16_t const packet_capacity,
    uint16_t *const packet_size)
{
    int rc = 0;
    struct Framer7bReceiver receiver;
    int decoded_size = 0;
    uint32_t crc = 0;

    ASSERT(NULL != frame, ER_INVAL);
    ASSERT(NULL != packet, ER_INVAL);
    ASSERT(NULL != packet_size, ER_INVAL);
    ASSERT(0 == framer7b_receiver__init(&receiver, packet, packet_capacity), ER_INVAL);

    for (uint16_t i = 0; i < frame_size; ++i) {
        decoded_size = framer7b_receiver__push(&receiver, frame[i]);
        ASSERT(0 <= decoded_size, ER_PROTO);
    }
    ASSERT(0 < decoded_size, ER_PROTO);
    ASSERT(TEST_PACKET_MIN_SIZE <= decoded_size, ER_PROTO);

    crc = crc32__ieee(packet, (size_t)(decoded_size - TEST_PACKET_CHECKSUM_SIZE));
    ASSERT((uint8_t)crc == packet[decoded_size - TEST_PACKET_CHECKSUM_SIZE], ER_PROTO);
    ASSERT((uint8_t)(crc >> 8) == packet[decoded_size - TEST_PACKET_CHECKSUM_SIZE + 1U], ER_PROTO);
    ASSERT((uint8_t)(crc >> 16) == packet[decoded_size - TEST_PACKET_CHECKSUM_SIZE + 2U], ER_PROTO);
    ASSERT((uint8_t)(crc >> 24) == packet[decoded_size - TEST_PACKET_CHECKSUM_SIZE + 3U], ER_PROTO);
    *packet_size = (uint16_t)decoded_size;

finally:

    return rc;
}

static int _dedup_transmit(
    struct Sep3 *core,
    uint8_t const *frame,
    uint16_t frame_size,
    uint8_t slot_id,
    void *user)
{
    int rc = 0;
    struct DedupTransport *transport = user;

    ARG_UNUSED(core);

    ASSERT(NULL != transport, ER_INVAL);
    ASSERT(NULL != frame, ER_INVAL);
    ASSERT(frame_size <= (uint16_t)sizeof(transport->frame), ER_OVERFLOW);

    memmove(transport->frame, frame, frame_size);
    transport->frame_size = frame_size;
    transport->slot_id = slot_id;
    ++transport->transmit_count;

finally:

    return rc;
}

static void _dedup_read_handler(
    struct Sep3 *self,
    struct Sep3RequestToken const *token,
    void *user)
{
    struct DedupEndpoint *endpoint = user;

    ++endpoint->read_count;
    endpoint->handler_rc = sep3__send_read_answer(self, token, g_read_answer, sizeof(g_read_answer));
}

static void _dedup_write_handler(
    struct Sep3 *self,
    struct Sep3RequestToken const *token,
    uint8_t const *data,
    uint16_t data_size,
    bool is_need_answer,
    void *user)
{
    struct DedupEndpoint *endpoint = user;

    ARG_UNUSED(is_need_answer);

    endpoint->payload_from_rx = (data == &g_dedup_buffers.rx[TEST_PACKET_PAYLOAD_OFFSET]);
    endpoint->received_size = data_size;
    if (0U < data_size) {
        memmove(endpoint->received, data, data_size);
    }
    ++endpoint->write_count;

    if (DEDUP_WRITE_DEFER == endpoint->write_mode) {
        endpoint->token = *token;
        return;
    }
    endpoint->handler_rc = sep3__send_write_answer(self, token);
}

static int _complete_dedup_tx(void)
{
    int rc = 0;
    uint8_t const slot_id = g_dedup_transport.slot_id;

    ASSERT(0U < g_dedup_transport.transmit_count, ER_AGAIN);

    g_dedup_transport.transmit_count = 0;
    g_dedup_transport.frame_size = 0;
    TRY(sep3__handle_transmitted(&g_dedup_core, slot_id, 0));

finally:

    return rc;
}

static int _check_captured_answer(
    Sep3PacketType const expected_type,
    TransactionId const expected_transaction_id,
    DataId const expected_data_id,
    uint8_t const *const expected_payload,
    uint16_t const expected_payload_size)
{
    int rc = 0;
    uint16_t packet_size = 0;
    uint16_t actual_payload_size = 0;
    TransactionId actual_transaction_id = 0;

    ASSERT(0U < g_dedup_transport.transmit_count, ER_AGAIN);
    TRY(_decode_test_frame(g_dedup_transport.frame, g_dedup_transport.frame_size, g_dedup_answer_packet,
        (uint16_t)sizeof(g_dedup_answer_packet), &packet_size));

    actual_transaction_id = (TransactionId)(g_dedup_answer_packet[TEST_PACKET_TRANSACTION_ID_OFFSET] |
        ((TransactionId)g_dedup_answer_packet[TEST_PACKET_TRANSACTION_ID_OFFSET + 1U] << 8));
    actual_payload_size = (uint16_t)(packet_size - TEST_PACKET_MIN_SIZE);

    ASSERT((uint8_t)expected_type == g_dedup_answer_packet[TEST_PACKET_TYPE_OFFSET], ER_PROTO);
    ASSERT(expected_transaction_id == actual_transaction_id, ER_PROTO);
    ASSERT(expected_data_id == g_dedup_answer_packet[TEST_PACKET_DATA_ID_OFFSET], ER_PROTO);
    ASSERT(expected_payload_size == actual_payload_size, ER_PROTO);
    if (0U < expected_payload_size) {
        ASSERT(NULL != expected_payload, ER_INVAL);
        ASSERT(0 == memcmp(&g_dedup_answer_packet[TEST_PACKET_PAYLOAD_OFFSET], expected_payload,
            expected_payload_size), ER_PROTO);
    }

finally:

    return rc;
}

static int _dedup_expect_answer(
    Sep3PacketType const expected_type,
    TransactionId const expected_transaction_id,
    DataId const expected_data_id,
    uint8_t const *const expected_payload,
    uint16_t const expected_payload_size)
{
    int rc = 0;

    TRY(_check_captured_answer(expected_type, expected_transaction_id, expected_data_id, expected_payload,
        expected_payload_size));
    TRY(_complete_dedup_tx());

finally:

    return rc;
}

static int _dedup_init(void)
{
    int rc = 0;
    uint32_t const token_epoch = ++g_dedup_epoch;
    struct Sep3Config const config = {
        .buffers = &g_dedup_buffers,
        .endpoints = g_dedup_endpoints,
        .endpoint_capacity = ARRAY_SIZE(g_dedup_endpoints),
        .token_epoch = token_epoch,
        .retry_count = 0,
        .transmit = _dedup_transmit,
        .transmit_user = &g_dedup_transport,
    };

    memset(&g_dedup_transport, 0, sizeof(g_dedup_transport));
    memset(&g_dedup_endpoint, 0, sizeof(g_dedup_endpoint));

    TRY(sep3__init(&g_dedup_core, &config));
    TRY(sep3__register_read_handler(&g_dedup_core, TEST_DATA_ID, TEST_TIMEOUT_MS, _dedup_read_handler,
        &g_dedup_endpoint));
    TRY(sep3__register_write_handler(&g_dedup_core, TEST_DATA_ID, TEST_TIMEOUT_MS, false,
        _dedup_write_handler, &g_dedup_endpoint));

finally:

    return rc;
}

static int _dedup_receive_request(
    Sep3PacketType const type,
    TransactionId const transaction_id,
    DataId const data_id,
    uint8_t const *const payload,
    uint16_t const payload_size)
{
    int rc = 0;
    uint16_t frame_size = 0;

    TRY(_build_test_frame(g_dedup_frame, (uint16_t)sizeof(g_dedup_frame), type, transaction_id, data_id,
        payload, payload_size, &frame_size));
    TRY(sep3__handle_received(&g_dedup_core, g_dedup_frame, frame_size));

finally:

    return rc;
}

static void _read_thread_entry(void *arg1, void *arg2, void *arg3)
{
    struct ReadThreadArgs *args = arg1;

    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    args->rc = sep3_zephyr_read(args->peer, TEST_DATA_ID, TEST_TIMEOUT_MS, args->data,
        sizeof(args->data), &args->data_size);
}

static struct Sep3ZephyrConfig _config(struct device const *uart, char const *name)
{
    struct Sep3ZephyrConfig config = {
        .uart = uart,
        .uart_rx_timeout_us = 1000,
        .uart_tx_timeout_us = 100000,
        .retry_count = 0,
        .thread_name = name,
    };

    return config;
}

static void *sep3_setup(void)
{
    struct Sep3ZephyrConfig config_a = _config(g_fixture.uart_a, "sep3-test-a");
    struct Sep3ZephyrConfig config_b = _config(g_fixture.uart_b, "sep3-test-b");

    zassert_true(device_is_ready(g_fixture.uart_a));
    zassert_true(device_is_ready(g_fixture.uart_b));

    g_fixture.a_to_b.peer = g_fixture.uart_b;
    g_fixture.b_to_a.peer = g_fixture.uart_a;
    k_sem_init(&g_fixture.write_received, 0, 1);
    k_sem_init(&g_fixture.read_held, 0, 1);
    uart_emul_callback_tx_data_ready_set(g_fixture.uart_a, _forward_uart, &g_fixture.a_to_b);
    uart_emul_callback_tx_data_ready_set(g_fixture.uart_b, _forward_uart, &g_fixture.b_to_a);

    zassert_ok(sep3_zephyr_init(&g_fixture.peer_a, &config_a));
    zassert_ok(sep3_zephyr_init(&g_fixture.peer_b, &config_b));
    zassert_ok(sep3_zephyr_register_read_handler(
        &g_fixture.peer_b, TEST_DATA_ID, TEST_TIMEOUT_MS, _read_handler, &g_fixture));
    zassert_ok(sep3_zephyr_register_read_handler(
        &g_fixture.peer_b, HOLD_DATA_ID, TEST_TIMEOUT_MS, _read_handler, &g_fixture));
    zassert_ok(sep3_zephyr_register_write_handler(
        &g_fixture.peer_b, TEST_DATA_ID, TEST_TIMEOUT_MS, true, _write_handler, &g_fixture));

    return &g_fixture;
}

static void sep3_before(void *user_data)
{
    struct sep3_fixture *fixture = user_data;

    memset(fixture->received, 0, sizeof(fixture->received));
    fixture->received_size = 0;
    fixture->answer_expected = false;
    fixture->read_mode = READ_MODE_SUCCESS;
    fixture->handler_rc = ER_1;
    memset(&fixture->held_token, 0, sizeof(fixture->held_token));
    k_sem_reset(&fixture->write_received);
    k_sem_reset(&fixture->read_held);

    atomic_set(&fixture->a_to_b.drop, 0);
    atomic_set(&fixture->a_to_b.error, 0);
    atomic_set(&fixture->b_to_a.drop, 0);
    atomic_set(&fixture->b_to_a.error, 0);

    (void)uart_emul_flush_rx_data(fixture->uart_a);
    (void)uart_emul_flush_tx_data(fixture->uart_a);
    (void)uart_emul_flush_rx_data(fixture->uart_b);
    (void)uart_emul_flush_tx_data(fixture->uart_b);
}

static void sep3_after(void *user_data)
{
    struct sep3_fixture *fixture = user_data;

    zassert_equal(atomic_get(&fixture->a_to_b.error), 0);
    zassert_equal(atomic_get(&fixture->b_to_a.error), 0);
}

ZTEST_F(sep3, test_crc32_known_check_value)
{
    static char const data[] = "123456789";

    zassert_equal(crc32__ieee((uint8_t const *)data, sizeof(data) - 1U), 0xCBF43926U);
}

ZTEST_F(sep3, test_invalid_packet_crc_is_ignored)
{
    uint8_t const payload[] = {1, 2, 3};
    uint8_t frame[SEP3_MAX_ENCODED_FRAME_SIZE];
    uint16_t packet_size = 0;
    int encoded_size = 0;

    zassert_ok(_dedup_init());
    zassert_ok(_build_test_packet(frame, (uint16_t)sizeof(frame), SEP3_PACKET_WRITE,
        TEST_TRANSACTION_ID, TEST_DATA_ID, payload, sizeof(payload), &packet_size));

    frame[TEST_PACKET_PAYLOAD_OFFSET] ^= 0x01U;
    encoded_size = framer7b__encode_in_place(frame, packet_size, (uint16_t)sizeof(frame));
    zassert_true(0 < encoded_size);

    zassert_ok(sep3__handle_received(&g_dedup_core, frame, (uint16_t)encoded_size));
    zassert_ok(sep3__process(&g_dedup_core, 0));
    zassert_equal(g_dedup_endpoint.write_count, 0);
    zassert_equal(g_dedup_transport.transmit_count, 0);
}

ZTEST_F(sep3, test_read)
{
    uint8_t data[sizeof(g_read_answer)] = {0};
    uint16_t data_size = 0;

    zassert_ok(sep3_zephyr_read(&fixture->peer_a, TEST_DATA_ID, TEST_TIMEOUT_MS, data, sizeof(data),
        &data_size));
    zassert_ok(fixture->handler_rc);
    zassert_equal(data_size, sizeof(g_read_answer));
    zassert_mem_equal(data, g_read_answer, sizeof(g_read_answer));
}

ZTEST_F(sep3, test_concurrent_reads_are_serialized)
{
    struct ReadThreadArgs args_a = {.peer = &fixture->peer_a, .rc = ER_1};
    struct ReadThreadArgs args_b = {.peer = &fixture->peer_a, .rc = ER_1};

    k_thread_create(&g_read_thread_a, g_read_thread_stack_a, K_THREAD_STACK_SIZEOF(g_read_thread_stack_a),
        _read_thread_entry, &args_a, NULL, NULL, K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
    k_thread_create(&g_read_thread_b, g_read_thread_stack_b, K_THREAD_STACK_SIZEOF(g_read_thread_stack_b),
        _read_thread_entry, &args_b, NULL, NULL, K_PRIO_PREEMPT(0), 0, K_NO_WAIT);

    zassert_ok(k_thread_join(&g_read_thread_a, K_SECONDS(1)));
    zassert_ok(k_thread_join(&g_read_thread_b, K_SECONDS(1)));
    zassert_ok(args_a.rc);
    zassert_ok(args_b.rc);
    zassert_mem_equal(args_a.data, g_read_answer, sizeof(g_read_answer));
    zassert_mem_equal(args_b.data, g_read_answer, sizeof(g_read_answer));
    zassert_ok(fixture->handler_rc);
}

ZTEST_F(sep3, test_rejects_second_init)
{
    struct Sep3ZephyrConfig config = _config(fixture->uart_a, "sep3-test-a");

    zassert_equal(sep3_zephyr_init(&fixture->peer_a, &config), ER_ALREADY);
}

ZTEST_F(sep3, test_rejects_duplicate_uart_owner)
{
    struct Sep3ZephyrConfig config = _config(fixture->uart_a, "sep3-duplicate");

    zassert_equal(sep3_zephyr_init(&g_duplicate_peer, &config), ER_BUSY);
    zassert_equal(sep3_zephyr_init(&g_duplicate_peer, &config), ER_ALREADY);
    zassert_equal(sep3_zephyr_write_no_answer(&g_duplicate_peer, TEST_DATA_ID, NULL, 0), ER_NO_DEV);
}

ZTEST_F(sep3, test_read_overflow)
{
    uint8_t data[2] = {0};
    uint16_t data_size = 0;

    zassert_equal(sep3_zephyr_read(&fixture->peer_a, TEST_DATA_ID, TEST_TIMEOUT_MS, data, sizeof(data),
        &data_size), ER_OVERFLOW);
    zassert_ok(fixture->handler_rc);
    zassert_equal(data_size, sizeof(g_read_answer));
    zassert_mem_equal(data, (uint8_t[2]){0}, sizeof(data));
}

ZTEST_F(sep3, test_write_max_payload)
{
    for (uint16_t i = 0; i < sizeof(g_max_payload); i++) {
        g_max_payload[i] = (uint8_t)i;
    }

    zassert_ok(sep3_zephyr_write(&fixture->peer_a, TEST_DATA_ID, TEST_TIMEOUT_MS, g_max_payload,
        sizeof(g_max_payload)));
    zassert_equal(fixture->received_size, sizeof(g_max_payload));
    zassert_true(fixture->answer_expected);
    zassert_mem_equal(fixture->received, g_max_payload, sizeof(g_max_payload));
}

ZTEST_F(sep3, test_write_rejects_oversize_payload)
{
    zassert_equal(sep3_zephyr_write(&fixture->peer_a, TEST_DATA_ID, TEST_TIMEOUT_MS, g_oversize_payload,
        sizeof(g_oversize_payload)), ER_OVERFLOW);
}

ZTEST_F(sep3, test_remote_error)
{
    uint16_t data_size = 0;

    fixture->read_mode = READ_MODE_ERROR;
    zassert_equal(sep3_zephyr_read(&fixture->peer_a, TEST_DATA_ID, TEST_TIMEOUT_MS, NULL, 0, &data_size),
        ER_PROTO_INTERNAL);
    zassert_ok(fixture->handler_rc);
    zassert_equal(data_size, 0);
}

ZTEST_F(sep3, test_remote_protocol_error)
{
    struct Sep3Endpoint endpoints[1] = {0};
    struct CoreTransport transport = {.uart = fixture->uart_b};
    struct Sep3Config const config = {
        .buffers = &g_protocol_error_buffers,
        .endpoints = endpoints,
        .endpoint_capacity = ARRAY_SIZE(endpoints),
        .token_epoch = 1,
        .retry_count = 0,
        .transmit = _core_transmit,
        .transmit_user = &transport,
    };
    uint16_t data_size = 0;

    fixture->read_mode = READ_MODE_HOLD;
    zassert_ok(sep3__init(&g_protocol_error_core, &config));
    zassert_ok(sep3__read(&g_protocol_error_core, HOLD_DATA_ID, TEST_TIMEOUT_MS, _core_request_callback,
        NULL));
    zassert_ok(sep3__handle_transmitted(&g_protocol_error_core, transport.slot_id, 0));
    zassert_ok(k_sem_take(&fixture->read_held, K_SECONDS(1)));

    zassert_equal(sep3_zephyr_read(&fixture->peer_a, TEST_DATA_ID, TEST_TIMEOUT_MS, NULL, 0, &data_size),
        ER_PROTO);
    zassert_equal(data_size, 0);
    atomic_set(&fixture->b_to_a.drop, 1);
    zassert_ok(sep3_zephyr_send_read_answer(
        &fixture->peer_b, &fixture->held_token, g_read_answer, sizeof(g_read_answer)));
    k_sleep(K_MSEC(10));
    atomic_set(&fixture->b_to_a.drop, 0);
}

ZTEST_F(sep3, test_timeout)
{
    uint16_t data_size = 0;

    atomic_set(&fixture->a_to_b.drop, 1);
    zassert_equal(sep3_zephyr_read(&fixture->peer_a, TEST_DATA_ID, TEST_TIMEOUT_MS, NULL, 0, &data_size),
        ER_TIMEDOUT);
}

ZTEST_F(sep3, test_zero_timeout_is_rejected)
{
    struct Sep3Config const core_config = {
        .buffers = &g_protocol_error_buffers,
        .endpoint_capacity = 0,
        .token_epoch = 1,
        .retry_count = 0,
        .transmit = _busy_core_transmit,
        .transmit_user = NULL,
    };
    uint16_t data_size = 0;

    zassert_ok(sep3__init(&g_protocol_error_core, &core_config));
    zassert_equal(sep3__register_read_handler(
        &g_protocol_error_core, TEST_DATA_ID, 0U, _dedup_read_handler, NULL), ER_INVAL);
    zassert_equal(sep3__register_write_handler(
        &g_protocol_error_core, TEST_DATA_ID, 0U, false, _dedup_write_handler, NULL), ER_INVAL);
    zassert_equal(sep3__read(&g_protocol_error_core, TEST_DATA_ID, 0U, _core_request_callback, NULL),
        ER_INVAL);
    zassert_equal(sep3__write(
        &g_protocol_error_core, TEST_DATA_ID, 0U, NULL, 0, _core_request_callback, NULL), ER_INVAL);

    zassert_equal(sep3_zephyr_register_read_handler(
        &fixture->peer_b, TEST_DATA_ID, 0U, _read_handler, fixture), ER_INVAL);
    zassert_equal(sep3_zephyr_register_write_handler(
        &fixture->peer_b, TEST_DATA_ID, 0U, true, _write_handler, fixture), ER_INVAL);
    zassert_equal(sep3_zephyr_read(&fixture->peer_a, TEST_DATA_ID, 0U, NULL, 0, &data_size), ER_INVAL);
    zassert_equal(sep3_zephyr_write(&fixture->peer_a, TEST_DATA_ID, 0U, NULL, 0), ER_INVAL);
}

ZTEST_F(sep3, test_write_no_answer)
{
    uint8_t const data[] = {1, 2, 3};

    zassert_ok(sep3_zephyr_write_no_answer(&fixture->peer_a, TEST_DATA_ID, data, sizeof(data)));
    zassert_ok(k_sem_take(&fixture->write_received, K_SECONDS(1)));
    zassert_false(fixture->answer_expected);
    zassert_equal(fixture->received_size, sizeof(data));
    zassert_mem_equal(fixture->received, data, sizeof(data));
}

ZTEST_F(sep3, test_write_no_answer_rejects_when_tx_slot_busy)
{
    struct Sep3Endpoint endpoints[1] = {0};
    struct Sep3Config const config = {
        .buffers = &g_tx_busy_buffers,
        .endpoints = endpoints,
        .endpoint_capacity = ARRAY_SIZE(endpoints),
        .token_epoch = 1,
        .retry_count = 0,
        .transmit = _busy_core_transmit,
        .transmit_user = NULL,
    };
    uint8_t const data[] = {1, 2, 3};

    zassert_ok(sep3__init(&g_tx_busy_core, &config));
    zassert_ok(sep3__write_no_answer(&g_tx_busy_core, TEST_DATA_ID, data, sizeof(data)));
    zassert_ok(sep3__process(&g_tx_busy_core, 1));
    zassert_equal(sep3__write_no_answer(&g_tx_busy_core, TEST_DATA_ID, data, sizeof(data)), ER_AGAIN);
}

ZTEST_F(sep3, test_incoming_read_duplicate_replays_answer)
{
    zassert_ok(_dedup_init());

    zassert_ok(_dedup_receive_request(SEP3_PACKET_READ, TEST_TRANSACTION_ID, TEST_DATA_ID, NULL, 0));
    zassert_equal(g_dedup_endpoint.read_count, 1);
    zassert_ok(g_dedup_endpoint.handler_rc);
    zassert_ok(_dedup_expect_answer(SEP3_PACKET_READ_ANSWER, TEST_TRANSACTION_ID, TEST_DATA_ID,
        g_read_answer, sizeof(g_read_answer)));

    zassert_ok(_dedup_receive_request(SEP3_PACKET_READ, TEST_TRANSACTION_ID, TEST_DATA_ID, NULL, 0));
    zassert_equal(g_dedup_endpoint.read_count, 1);
    zassert_ok(_dedup_expect_answer(SEP3_PACKET_READ_ANSWER, TEST_TRANSACTION_ID, TEST_DATA_ID,
        g_read_answer, sizeof(g_read_answer)));
}

ZTEST_F(sep3, test_incoming_write_duplicate_replays_answer)
{
    uint8_t const payload[] = {1, 2, 3};

    zassert_ok(_dedup_init());

    zassert_ok(_dedup_receive_request(SEP3_PACKET_WRITE, TEST_TRANSACTION_ID, TEST_DATA_ID, payload,
        sizeof(payload)));
    zassert_equal(g_dedup_endpoint.write_count, 1);
    zassert_ok(g_dedup_endpoint.handler_rc);
    zassert_ok(_dedup_expect_answer(SEP3_PACKET_WRITE_ANSWER, TEST_TRANSACTION_ID, TEST_DATA_ID, NULL, 0));

    zassert_ok(_dedup_receive_request(SEP3_PACKET_WRITE, TEST_TRANSACTION_ID, TEST_DATA_ID, payload,
        sizeof(payload)));
    zassert_equal(g_dedup_endpoint.write_count, 1);
    zassert_ok(_dedup_expect_answer(SEP3_PACKET_WRITE_ANSWER, TEST_TRANSACTION_ID, TEST_DATA_ID, NULL, 0));
}

ZTEST_F(sep3, test_incoming_write_deferred_duplicate_waits_for_answer)
{
    uint8_t const payload[] = {1, 2, 3};

    zassert_ok(_dedup_init());
    g_dedup_endpoint.write_mode = DEDUP_WRITE_DEFER;

    zassert_ok(_dedup_receive_request(SEP3_PACKET_WRITE, TEST_TRANSACTION_ID, TEST_DATA_ID, payload,
        sizeof(payload)));
    zassert_equal(g_dedup_endpoint.write_count, 1);
    zassert_equal(g_dedup_transport.transmit_count, 0);

    zassert_ok(_dedup_receive_request(SEP3_PACKET_WRITE, TEST_TRANSACTION_ID, TEST_DATA_ID, payload,
        sizeof(payload)));
    zassert_equal(g_dedup_endpoint.write_count, 1);
    zassert_equal(g_dedup_transport.transmit_count, 0);

    zassert_ok(sep3__send_write_answer(&g_dedup_core, &g_dedup_endpoint.token));
    zassert_ok(_dedup_expect_answer(SEP3_PACKET_WRITE_ANSWER, TEST_TRANSACTION_ID, TEST_DATA_ID, NULL, 0));

    zassert_ok(_dedup_receive_request(SEP3_PACKET_WRITE, TEST_TRANSACTION_ID, TEST_DATA_ID, payload,
        sizeof(payload)));
    zassert_equal(g_dedup_endpoint.write_count, 1);
    zassert_ok(_dedup_expect_answer(SEP3_PACKET_WRITE_ANSWER, TEST_TRANSACTION_ID, TEST_DATA_ID, NULL, 0));
}

ZTEST_F(sep3, test_incoming_write_duplicate_after_timeout_is_ignored)
{
    uint8_t const payload[] = {1, 2, 3};

    zassert_ok(_dedup_init());
    zassert_ok(sep3__process(&g_dedup_core, 0));
    g_dedup_endpoint.write_mode = DEDUP_WRITE_DEFER;

    zassert_ok(_dedup_receive_request(SEP3_PACKET_WRITE, TEST_TRANSACTION_ID, TEST_DATA_ID, payload,
        sizeof(payload)));
    zassert_equal(g_dedup_endpoint.write_count, 1);
    zassert_equal(g_dedup_transport.transmit_count, 0);

    zassert_ok(sep3__process(&g_dedup_core, TEST_TIMEOUT_MS));

    zassert_ok(_dedup_receive_request(SEP3_PACKET_WRITE, TEST_TRANSACTION_ID, TEST_DATA_ID, payload,
        sizeof(payload)));
    zassert_equal(g_dedup_endpoint.write_count, 1);
    zassert_equal(g_dedup_transport.transmit_count, 0);

    zassert_ok(_dedup_receive_request(SEP3_PACKET_WRITE, TEST_TRANSACTION_ID + 1, TEST_DATA_ID, payload,
        sizeof(payload)));
    zassert_equal(g_dedup_endpoint.write_count, 2);
    zassert_equal(g_dedup_transport.transmit_count, 0);
}

ZTEST_F(sep3, test_incoming_write_payload_conflict)
{
    uint8_t const payload[] = {1, 2, 3};
    uint8_t const conflict[] = {1, 2, 4};
    uint8_t const expected_error[] = {(uint8_t)SEP3_PROTOCOL_ERROR_TRANSACTION_CONFLICT};

    zassert_ok(_dedup_init());

    zassert_ok(_dedup_receive_request(SEP3_PACKET_WRITE, TEST_TRANSACTION_ID, TEST_DATA_ID, payload,
        sizeof(payload)));
    zassert_equal(g_dedup_endpoint.write_count, 1);
    zassert_ok(g_dedup_endpoint.handler_rc);
    zassert_ok(_dedup_expect_answer(SEP3_PACKET_WRITE_ANSWER, TEST_TRANSACTION_ID, TEST_DATA_ID, NULL, 0));

    zassert_ok(_dedup_receive_request(SEP3_PACKET_WRITE, TEST_TRANSACTION_ID, TEST_DATA_ID, conflict,
        sizeof(conflict)));
    zassert_equal(g_dedup_endpoint.write_count, 1);
    zassert_ok(_dedup_expect_answer(SEP3_PACKET_PROTO_ERROR_ANSWER, TEST_TRANSACTION_ID, TEST_DATA_ID,
        expected_error, sizeof(expected_error)));
}

ZTEST_F(sep3, test_incoming_write_size_conflict)
{
    uint8_t const payload[] = {1, 2, 3};
    uint8_t const conflict[] = {1, 2};
    uint8_t const expected_error[] = {(uint8_t)SEP3_PROTOCOL_ERROR_TRANSACTION_CONFLICT};

    zassert_ok(_dedup_init());

    zassert_ok(_dedup_receive_request(SEP3_PACKET_WRITE, TEST_TRANSACTION_ID, TEST_DATA_ID, payload,
        sizeof(payload)));
    zassert_equal(g_dedup_endpoint.write_count, 1);
    zassert_ok(g_dedup_endpoint.handler_rc);
    zassert_ok(_dedup_expect_answer(SEP3_PACKET_WRITE_ANSWER, TEST_TRANSACTION_ID, TEST_DATA_ID, NULL, 0));

    zassert_ok(_dedup_receive_request(SEP3_PACKET_WRITE, TEST_TRANSACTION_ID, TEST_DATA_ID, conflict,
        sizeof(conflict)));
    zassert_equal(g_dedup_endpoint.write_count, 1);
    zassert_ok(_dedup_expect_answer(SEP3_PACKET_PROTO_ERROR_ANSWER, TEST_TRANSACTION_ID, TEST_DATA_ID,
        expected_error, sizeof(expected_error)));
}

ZTEST_F(sep3, test_incoming_write_data_id_conflict)
{
    uint8_t const payload[] = {1, 2, 3};
    uint8_t const expected_error[] = {(uint8_t)SEP3_PROTOCOL_ERROR_TRANSACTION_CONFLICT};

    zassert_ok(_dedup_init());

    zassert_ok(_dedup_receive_request(SEP3_PACKET_WRITE, TEST_TRANSACTION_ID, TEST_DATA_ID, payload,
        sizeof(payload)));
    zassert_equal(g_dedup_endpoint.write_count, 1);
    zassert_ok(g_dedup_endpoint.handler_rc);
    zassert_ok(_dedup_expect_answer(SEP3_PACKET_WRITE_ANSWER, TEST_TRANSACTION_ID, TEST_DATA_ID, NULL, 0));

    zassert_ok(_dedup_receive_request(SEP3_PACKET_WRITE, TEST_TRANSACTION_ID, HOLD_DATA_ID, payload,
        sizeof(payload)));
    zassert_equal(g_dedup_endpoint.write_count, 1);
    zassert_ok(_dedup_expect_answer(SEP3_PACKET_PROTO_ERROR_ANSWER, TEST_TRANSACTION_ID, HOLD_DATA_ID,
        expected_error, sizeof(expected_error)));
}

ZTEST_F(sep3, test_incoming_write_type_conflict)
{
    uint8_t const payload[] = {7};
    uint8_t const expected_error[] = {(uint8_t)SEP3_PROTOCOL_ERROR_TRANSACTION_CONFLICT};

    zassert_ok(_dedup_init());

    zassert_ok(_dedup_receive_request(SEP3_PACKET_WRITE, TEST_TRANSACTION_ID, TEST_DATA_ID, payload,
        sizeof(payload)));
    zassert_equal(g_dedup_endpoint.write_count, 1);
    zassert_ok(g_dedup_endpoint.handler_rc);
    zassert_ok(_dedup_expect_answer(SEP3_PACKET_WRITE_ANSWER, TEST_TRANSACTION_ID, TEST_DATA_ID, NULL, 0));

    zassert_ok(_dedup_receive_request(SEP3_PACKET_READ, TEST_TRANSACTION_ID, TEST_DATA_ID, payload,
        sizeof(payload)));
    zassert_equal(g_dedup_endpoint.write_count, 1);
    zassert_equal(g_dedup_endpoint.read_count, 0);
    zassert_ok(_dedup_expect_answer(SEP3_PACKET_PROTO_ERROR_ANSWER, TEST_TRANSACTION_ID, TEST_DATA_ID,
        expected_error, sizeof(expected_error)));
}

ZTEST_F(sep3, test_incoming_write_max_payload_from_rx_buffer)
{
    for (uint16_t i = 0; i < (uint16_t)sizeof(g_dedup_payload); ++i) {
        g_dedup_payload[i] = (uint8_t)i;
    }

    zassert_ok(_dedup_init());

    zassert_ok(_dedup_receive_request(SEP3_PACKET_WRITE, TEST_TRANSACTION_ID, TEST_DATA_ID, g_dedup_payload,
        sizeof(g_dedup_payload)));
    zassert_equal(g_dedup_endpoint.write_count, 1);
    zassert_true(g_dedup_endpoint.payload_from_rx);
    zassert_equal(g_dedup_endpoint.received_size, sizeof(g_dedup_payload));
    zassert_mem_equal(g_dedup_endpoint.received, g_dedup_payload, sizeof(g_dedup_payload));
    zassert_ok(g_dedup_endpoint.handler_rc);
    zassert_ok(_dedup_expect_answer(SEP3_PACKET_WRITE_ANSWER, TEST_TRANSACTION_ID, TEST_DATA_ID, NULL, 0));

    zassert_ok(_dedup_receive_request(SEP3_PACKET_WRITE, TEST_TRANSACTION_ID, TEST_DATA_ID, g_dedup_payload,
        sizeof(g_dedup_payload)));
    zassert_equal(g_dedup_endpoint.write_count, 1);
    zassert_ok(_dedup_expect_answer(SEP3_PACKET_WRITE_ANSWER, TEST_TRANSACTION_ID, TEST_DATA_ID, NULL, 0));
}

ZTEST_SUITE(sep3, NULL, sep3_setup, sep3_before, sep3_after, NULL);
