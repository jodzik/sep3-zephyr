# sep3-zephyr

Blocking wrapper протокола [SEP3](https://github.com/jodzik/sep3) для Zephyr
async UART API.

## Архитектура

Каждый `struct Sep3Zephyr` содержит отдельный owner-thread. Только этот поток
обращается к `sep3-c`, поэтому callbacks ядра никогда не исполняются из UART ISR
или из application thread.

Публичные функции передают в owner-thread указатель на команду и блокируют
вызывающий поток до ее обработки. Payload при этом дополнительно не копируется:
его синхронно копирует само ядро SEP3. Подтверждаемые `read` и `write`
сериализуются, поскольку ядро поддерживает одну исходящую транзакцию.

## Подключение

Добавьте модуль до `find_package(Zephyr)`:

```cmake
list(APPEND EXTRA_ZEPHYR_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/src/lib/sep3-zephyr)
```

Минимальная конфигурация:

```conf
CONFIG_SERIAL=y
CONFIG_UART_ASYNC_API=y
CONFIG_SEP3_ZEPHYR=y
```

Пример инициализации:

```c
#include <sep3-zephyr/sep3_zephyr.h>

static struct Sep3Zephyr g_sep3;

static int init_sep3(struct device const *uart)
{
    struct Sep3ZephyrConfig const config = {
        .uart = uart,
        .request_timeout_ms = 1000,
        .incoming_request_timeout_ms = 1000,
        .uart_rx_timeout_us = 1000,
        .uart_tx_timeout_us = 100000,
        .retry_count = 2,
        .thread_name = "sep3",
    };

    return sep3_zephyr_init(&g_sep3, &config);
}
```

`uart_rx_timeout_us` должен быть конечным, чтобы драйвер своевременно отдавал
короткие кадры через `UART_RX_RDY`. `uart_tx_timeout_us` используется как timeout
драйвера и как watchdog wrapper-а; при его истечении вызывается
`uart_tx_abort()`.

Wrapper требует эксклюзивного владения UART async API. Один UART нельзя
одновременно передавать нескольким экземплярам или использовать из другого
компонента. Драйвер должен поддерживать `uart_tx_abort()`, иначе инициализация
вернет `ER_NOT_SUPPORTED`.

## Результаты запросов

Нулевой return code `sep3_zephyr_read()` или `sep3_zephyr_write()` означает, что
получен корректный SEP3 answer. Удаленная ошибка также является корректным
answer, поэтому ее нужно определить по `result.answer_type`, а код и сообщение
читать из `remote_error_code` и `remote_error_message`.

При `ER_OVERFLOW` из `sep3_zephyr_read()` частичный payload не копируется, а
`data_size` содержит требуемый размер буфера.

## Endpoint callbacks

Endpoint callbacks выполняются в owner-thread. Из callback разрешено сразу
вызывать `sep3_zephyr_send_*_answer()`, но нельзя выполнять blocking `read` или
`write`.

Указатели на token и входящий payload действительны только во время callback.
Для отложенного ответа `struct Sep3RequestToken` необходимо скопировать по
значению; отправить сохраненный token затем можно из любого application thread.

## Тесты

Тесты используют два соединенных в памяти `zephyr,uart-emul`:

```sh
west build -p always -b native_sim/native/64 \
    -d build/sep3-zephyr-test -t run src/lib/sep3-zephyr/tests
```
