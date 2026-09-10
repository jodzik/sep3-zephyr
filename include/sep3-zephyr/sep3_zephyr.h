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

/** Wrapper configuration. */
struct Sep3ZephyrConfig {
    struct device const *uart;
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
 * Allocate statically or zero-initialize before the one allowed call to
 * sep3_zephyr_init(). The instance and any resources acquired during
 * initialization remain owned until reboot, including after an initialization
 * failure. Application code must not access the fields directly.
 */
struct Sep3Zephyr {
    struct Sep3 core;
    struct Sep3Buffers buffers;
    struct Sep3Endpoint core_endpoints[CONFIG_SEP3_ZEPHYR_ENDPOINT_CAPACITY];
    struct Sep3ZephyrEndpoint endpoints[CONFIG_SEP3_ZEPHYR_ENDPOINT_CAPACITY];
    struct device const *uart;
    uint32_t uart_rx_timeout_us;
    uint32_t uart_tx_timeout_us;

    struct k_thread thread;
    k_tid_t thread_id;
    K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_SEP3_ZEPHYR_THREAD_STACK_SIZE);

    struct k_msgq command_queue;
    struct Sep3ZephyrCommand *command_queue_buffer[CONFIG_SEP3_ZEPHYR_COMMAND_QUEUE_SIZE];
    struct k_sem event_sem;
    struct k_mutex command_mutex;
    struct k_mutex request_mutex;

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
};

/** Initialize and start an SEP3 instance on an async UART device.
 *
 * The wrapper requires exclusive ownership of the UART async API for its whole
 * lifetime. The driver must implement uart_tx_abort() and emit exactly one
 * UART_TX_DONE or UART_TX_ABORTED event for every accepted transmission,
 * including after a successful abort.
 *
 * The function may be called only once per instance per boot. Validation
 * failures before initialization starts leave the instance reusable. Any later
 * failure leaves the instance and any acquired resources owned until reboot;
 * the caller must enter its reboot flow.
 */
int sep3_zephyr_init(struct Sep3Zephyr *self, struct Sep3ZephyrConfig const *config);

/** Register a READ endpoint and its incoming response timeout. */
int sep3_zephyr_register_read_handler(
    struct Sep3Zephyr *self,
    DataId data_id,
    uint32_t incoming_request_timeout_ms,
    Sep3ZephyrReadHandler handler,
    void *user);

/** Register a WRITE endpoint and its incoming response timeout. */
int sep3_zephyr_register_write_handler(
    struct Sep3Zephyr *self,
    DataId data_id,
    uint32_t incoming_request_timeout_ms,
    bool allow_write_no_answer,
    Sep3ZephyrWriteHandler handler,
    void *user);

/** Perform a blocking READ transaction with the local response timeout_ms.
 *
 * data_size receives the remote payload size. ER_OVERFLOW is returned without
 * copying a partial payload when data_capacity is too small. A remote
 * application error returns ER_PROTO_INTERNAL; a remote protocol error returns
 * ER_PROTO.
 */
int sep3_zephyr_read(
    struct Sep3Zephyr *self,
    DataId data_id,
    uint32_t timeout_ms,
    uint8_t *data,
    uint16_t data_capacity,
    uint16_t *data_size);

/** Perform a blocking WRITE transaction with the local response timeout_ms.
 *
 * A remote application error returns ER_PROTO_INTERNAL; a remote protocol
 * error returns ER_PROTO.
 */
int sep3_zephyr_write(
    struct Sep3Zephyr *self,
    DataId data_id,
    uint32_t timeout_ms,
    uint8_t const *data,
    uint16_t data_size);

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
