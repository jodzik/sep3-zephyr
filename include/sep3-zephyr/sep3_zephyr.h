/** @file sep3_zephyr.h Blocking SEP3 wrapper for Zephyr async UART. */

#ifndef SEP3_ZEPHYR_H_
#define SEP3_ZEPHYR_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <sep3.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/ring_buffer.h>

#ifdef __cplusplus
extern "C" {
#endif

struct Sep3Zephyr;
struct Sep3ZephyrCommand;

/** Result of a completed confirmed request.
 *
 * A return value of zero from sep3_zephyr_read() or sep3_zephyr_write() means
 * that a valid SEP3 answer was received. Inspect answer_type to distinguish a
 * successful answer from a remote application or protocol error.
 */
struct Sep3ZephyrRequestResult {
    enum Sep3PacketType answer_type;
    uint8_t remote_error_code;
    uint16_t data_size;
    char remote_error_message[SEP3_MAX_PAYLOAD_SIZE];
};

/** Wrapper configuration. */
struct Sep3ZephyrConfig {
    struct device const *uart;
    uint32_t request_timeout_ms;
    uint32_t incoming_request_timeout_ms;
    uint32_t uart_rx_timeout_us;
    uint32_t uart_tx_timeout_us;
    uint8_t retry_count;
    char const *thread_name;
};

/** READ endpoint callback. Runs in the SEP3 owner thread.
 *
 * token remains valid only until the callback returns. Copy the token by value
 * when a response will be sent later from another thread.
 */
typedef void (*Sep3ZephyrReadHandler)(
    struct Sep3Zephyr *self,
    struct Sep3RequestToken const *token,
    void *user);

/** WRITE endpoint callback. Runs in the SEP3 owner thread.
 *
 * token and data remain valid only until this callback returns. Copy the token
 * by value when a response will be sent later. A blocking read or write must
 * not be started from an endpoint callback.
 */
typedef void (*Sep3ZephyrWriteHandler)(
    struct Sep3Zephyr *self,
    struct Sep3RequestToken const *token,
    uint8_t const *data,
    uint16_t data_size,
    bool is_need_answer,
    void *user);

/** Internal endpoint callback storage. Public only for static allocation. */
struct Sep3ZephyrEndpoint {
    struct Sep3Zephyr *owner;
    DataId data_id;
    bool is_used;
    Sep3ZephyrReadHandler on_read;
    void *on_read_user;
    Sep3ZephyrWriteHandler on_write;
    void *on_write_user;
};

/** SEP3 Zephyr instance.
 *
 * Allocate statically or zero-initialize before the first call to
 * sep3_zephyr_init(). Application code must not access the fields directly.
 */
struct Sep3Zephyr {
    struct Sep3 core;
    struct Sep3Buffers buffers;
    struct Sep3Endpoint core_endpoints[CONFIG_SEP3_ZEPHYR_ENDPOINT_CAPACITY];
    struct Sep3ZephyrEndpoint endpoints[CONFIG_SEP3_ZEPHYR_ENDPOINT_CAPACITY];
    struct Sep3ZephyrConfig config;

    struct k_thread thread;
    k_tid_t thread_id;
    K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_SEP3_ZEPHYR_THREAD_STACK_SIZE);

    struct k_msgq command_queue;
    struct Sep3ZephyrCommand *command_queue_buffer[CONFIG_SEP3_ZEPHYR_COMMAND_QUEUE_SIZE];
    struct k_sem event_sem;
    struct k_sem active_calls_done;
    struct k_mutex command_mutex;
    struct k_mutex request_mutex;
    atomic_t command_count;
    atomic_t active_call_count;

    struct ring_buf rx_ring;
    uint8_t rx_ring_buffer[CONFIG_SEP3_ZEPHYR_RX_RING_SIZE];
    uint8_t rx_buffers[2][CONFIG_SEP3_ZEPHYR_RX_BUFFER_SIZE];
    atomic_t rx_buffer_owned[2];
    atomic_t rx_disabled;
    atomic_t rx_error;

    atomic_t tx_active;
    atomic_t tx_event;
    atomic_t tx_error;
    atomic_t tx_abort_requested;
    uint8_t tx_slot_id;
    int64_t tx_started_ms;

    atomic_t state;
    uint16_t endpoint_count;
    int8_t uart_claim_index;
    struct Sep3ZephyrCommand *stop_command;
};

/** Initialize and start an SEP3 instance on an async UART device.
 *
 * The wrapper requires exclusive ownership of the UART async API for its whole
 * lifetime. The driver must implement uart_tx_abort() and emit exactly one
 * UART_TX_DONE or UART_TX_ABORTED event for every accepted transmission,
 * including after a successful abort.
 */
int sep3_zephyr_init(struct Sep3Zephyr *self, struct Sep3ZephyrConfig const *config);

/** Stop an idle instance.
 *
 * Returns ER_BUSY while a request or transport transmission is active.
 */
int sep3_zephyr_deinit(struct Sep3Zephyr *self);

/** Register a READ endpoint. Safe to call from any application thread. */
int sep3_zephyr_register_read_handler(
    struct Sep3Zephyr *self,
    DataId data_id,
    Sep3ZephyrReadHandler handler,
    void *user);

/** Register a WRITE endpoint. Safe to call from any application thread. */
int sep3_zephyr_register_write_handler(
    struct Sep3Zephyr *self,
    DataId data_id,
    bool allow_write_no_answer,
    Sep3ZephyrWriteHandler handler,
    void *user);

/** Perform a blocking READ transaction.
 *
 * data_size receives the remote payload size. ER_OVERFLOW is returned without
 * copying a partial payload when data_capacity is too small.
 */
int sep3_zephyr_read(
    struct Sep3Zephyr *self,
    DataId data_id,
    uint8_t *data,
    uint16_t data_capacity,
    uint16_t *data_size,
    struct Sep3ZephyrRequestResult *result);

/** Perform a blocking WRITE transaction. */
int sep3_zephyr_write(
    struct Sep3Zephyr *self,
    DataId data_id,
    uint8_t const *data,
    uint16_t data_size,
    struct Sep3ZephyrRequestResult *result);

/** Queue a best-effort WRITE_NO_ANSWER packet.
 *
 * Success means that SEP3 copied the packet into its internal queue, not that
 * the remote peer received it.
 */
int sep3_zephyr_write_no_answer(
    struct Sep3Zephyr *self,
    DataId data_id,
    uint8_t const *data,
    uint16_t data_size);

/** Send a successful READ response. */
int sep3_zephyr_send_read_answer(
    struct Sep3Zephyr *self,
    struct Sep3RequestToken const *token,
    uint8_t const *data,
    uint16_t data_size);

/** Send a successful WRITE response. */
int sep3_zephyr_send_write_answer(
    struct Sep3Zephyr *self,
    struct Sep3RequestToken const *token);

/** Send an application error response. */
int sep3_zephyr_send_error_answer(
    struct Sep3Zephyr *self,
    struct Sep3RequestToken const *token,
    uint8_t error_code,
    char const *message);

#ifdef __cplusplus
}
#endif

#endif
