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

Ядро использует один encoded TX-slot. Пока UART TX активен, сохранённый incoming
answer и временная протокольная ошибка ожидают освобождения слота, а
`sep3_zephyr_write_no_answer()` возвращает `ER_AGAIN`.

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

Максимальный payload ядра задаётся отдельно:

```conf
CONFIG_SEP3_MAX_PAYLOAD_SIZE=244
```

Допустимый диапазон — `64..2046`; значение по умолчанию — `244`.
`SEP3_MAX_PACKET_SIZE` включает 8 байт protocol overhead (4 заголовка + 4 CRC32),
`SEP3_MAX_ENCODED_BODY_SIZE` и `SEP3_MAX_ENCODED_FRAME_SIZE` вычисляются из
`SEP3_MAX_PACKET_SIZE` в `sep3.h`.
`CONFIG_SEP3_ZEPHYR_RX_RING_SIZE` должен быть не меньше
`SEP3_MAX_ENCODED_FRAME_SIZE`; значение по умолчанию — `290`.
`CONFIG_SEP3_ZEPHYR` выбирает `CONFIG_CRC`, так как на Zephyr `crc32__ieee()`
использует `crc32_ieee()`. Для payload больше 244 может также потребоваться
увеличить `CONFIG_SEP3_ZEPHYR_THREAD_STACK_SIZE`; при 2046 `Sep3Buffers`
занимает примерно 8.6 KiB на экземпляр.

Пример инициализации:

```c
#include <sep3-zephyr/sep3_zephyr.h>

static struct Sep3Zephyr g_sep3;

static int init_sep3(struct device const *uart)
{
    struct Sep3ZephyrConfig const config = {
        .uart = uart,
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

Protocol timeout задаётся локально:
incoming timeout endpoint при регистрации, а timeout `READ`/`WRITE`
— в каждом вызове:

```c
uint16_t data_size = 0;
uint8_t data[64];

sep3_zephyr_register_read_handler(&g_sep3, 1, 1000, read_handler, NULL);
sep3_zephyr_read(&g_sep3, 1, 2000, data, sizeof(data), &data_size);
```

Оба значения должны быть ненулевыми; `0` возвращает `ER_INVAL`. Исходящий
timeout измеряется от успешной первоначальной передачи и применяется к каждой
повторной попытке транзакции.

Wrapper требует эксклюзивного владения UART async API. Один UART нельзя
одновременно передавать нескольким экземплярам или использовать из другого
компонента. Драйвер должен поддерживать `uart_tx_abort()`, иначе инициализация
вернет `ER_NOT_SUPPORTED`.

## Lifecycle

Каждый экземпляр можно инициализировать только один раз за boot. Публичного
`deinit` нет: после успешного запуска owner-thread, UART callback и UART остаются
активными до reboot. Экземпляр должен иметь статическое время жизни.

Ошибки валидации до начала инициализации позволяют исправить параметры и
повторить вызов. Любая последующая ошибка терминально блокирует экземпляр, а уже
полученные им ресурсы не освобождаются до reboot; вызывающий код должен перейти
в reboot flow. Переданный config после
возврата из `sep3_zephyr_init()` больше не нужен: wrapper сохраняет только UART и
runtime UART timeouts, а protocol settings копирует ядро.

## Результаты запросов

Нулевой return code `sep3_zephyr_read()` или `sep3_zephyr_write()` означает, что
получен успешный SEP3 answer. Для удаленного `APP_ERROR_ANSWER` wrapper выводит
код и сообщение в log и возвращает `ER_PROTO_INTERNAL`. Для
`PROTO_ERROR_ANSWER` wrapper выводит код в log и возвращает `ER_PROTO`.

При `ER_OVERFLOW` из `sep3_zephyr_read()` частичный payload не копируется, а
`data_size` содержит требуемый размер буфера.

## Endpoint callbacks

Endpoint callbacks выполняются в owner-thread. Из callback разрешено сразу
вызывать `sep3_zephyr_send_*_answer()`, но нельзя выполнять blocking `read` или
`write`.

Указатели на token и входящий payload действительны только во время callback.
Payload указывает на decoded RX-буфер ядра SEP3. Для отложенного ответа
`struct Sep3RequestToken` необходимо скопировать по значению; отправить
сохраненный token затем можно из любого application thread.

Ядро не хранит полный входящий запрос: повтор идентифицируется по точным полям
заголовка, размеру пакета и CRC32 payload (CRC-32/ISO-HDLC). При коллизии CRC32
отличающийся payload может быть ошибочно принят за точный повтор; тогда handler
не вызовется повторно, а удалённой стороне будет отправлен сохранённый ответ.

## Тесты

Тесты используют два соединенных в памяти `zephyr,uart-emul`:

```sh
west build -p always -b native_sim/native/64 \
    -d build/sep3-zephyr-test -t run src/lib/sep3-zephyr/tests
```
