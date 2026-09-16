#include "modbus_rx_assembler.h"
#include "uart_dma_rx_buffer.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

typedef struct
{
    uint8_t frames[4][MODBUS_RX_ASSEMBLER_MAX_FRAME_SIZE];
    size_t lengths[4];
    size_t count;
    size_t stop_after;
} frame_log_t;

static unsigned tests_run;

#define CHECK(condition)                                                               \
    do                                                                                 \
    {                                                                                  \
        if (!(condition))                                                              \
        {                                                                              \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition);         \
            return false;                                                              \
        }                                                                              \
    } while (0)

#define RUN(test)                                                                      \
    do                                                                                 \
    {                                                                                  \
        ++tests_run;                                                                   \
        if (!(test)())                                                                 \
        {                                                                              \
            return 1;                                                                  \
        }                                                                              \
    } while (0)

static bool record_frame(const uint8_t *frame, size_t length, void *context)
{
    frame_log_t *log = context;

    if ((log == NULL) || (frame == NULL) ||
        (length > MODBUS_RX_ASSEMBLER_MAX_FRAME_SIZE) ||
        (log->count >= ARRAY_SIZE(log->frames)))
    {
        return false;
    }
    (void)memcpy(log->frames[log->count], frame, length);
    log->lengths[log->count] = length;
    ++log->count;
    return (log->stop_after == 0u) || (log->count < log->stop_after);
}

static bool test_complete_read_response(void)
{
    static const uint8_t response[] = {
        0x01u, 0x03u, 0x08u, 0x00u, 0x01u, 0x00u, 0x00u,
        0x03u, 0xE8u, 0x00u, 0x00u, 0x12u, 0x34u,
    };
    modbus_rx_assembler_t assembler;
    frame_log_t log = {0};

    modbus_rx_assembler_init(&assembler);
    CHECK(modbus_rx_assembler_feed(&assembler,
                                   response,
                                   sizeof(response),
                                   record_frame,
                                   &log) == sizeof(response));
    CHECK(log.count == 1u);
    CHECK(log.lengths[0] == sizeof(response));
    CHECK(memcmp(log.frames[0], response, sizeof(response)) == 0);
    return true;
}

static bool test_single_byte_and_arbitrary_splits(void)
{
    static const uint8_t response[] = {
        0x01u, 0x03u, 0x08u, 0x00u, 0x01u, 0x00u, 0x00u,
        0x03u, 0xE8u, 0x00u, 0x00u, 0x12u, 0x34u,
    };
    static const size_t pieces[] = {2u, 1u, 4u, 3u, 3u};
    modbus_rx_assembler_t assembler;
    frame_log_t log = {0};
    size_t offset;
    size_t index;

    modbus_rx_assembler_init(&assembler);
    for (offset = 0u; offset < sizeof(response); ++offset)
    {
        CHECK(modbus_rx_assembler_feed(&assembler,
                                       &response[offset],
                                       1u,
                                       record_frame,
                                       &log) == 1u);
    }
    CHECK(log.count == 1u);

    modbus_rx_assembler_reset(&assembler);
    (void)memset(&log, 0, sizeof(log));
    offset = 0u;
    for (index = 0u; index < ARRAY_SIZE(pieces); ++index)
    {
        CHECK(modbus_rx_assembler_feed(&assembler,
                                       &response[offset],
                                       pieces[index],
                                       record_frame,
                                       &log) == pieces[index]);
        offset += pieces[index];
    }
    CHECK(offset == sizeof(response));
    CHECK(log.count == 1u);
    return true;
}

static bool test_write_and_exception_responses(void)
{
    static const uint8_t write_single[] = {
        0x01u, 0x06u, 0x00u, 0x38u, 0x00u, 0x01u, 0xC9u, 0xCCu,
    };
    static const uint8_t write_multiple[] = {
        0x01u, 0x10u, 0x00u, 0x39u, 0x00u, 0x02u, 0x91u, 0xCDu,
    };
    static const uint8_t exception[] = {0x01u, 0x90u, 0x03u, 0x0Cu, 0x01u};
    modbus_rx_assembler_t assembler;
    frame_log_t log = {0};

    modbus_rx_assembler_init(&assembler);
    CHECK(modbus_rx_assembler_feed(&assembler,
                                   write_single,
                                   sizeof(write_single),
                                   record_frame,
                                   &log) == sizeof(write_single));
    CHECK(modbus_rx_assembler_feed(&assembler,
                                   write_multiple,
                                   sizeof(write_multiple),
                                   record_frame,
                                   &log) == sizeof(write_multiple));
    CHECK(modbus_rx_assembler_feed(&assembler,
                                   exception,
                                   sizeof(exception),
                                   record_frame,
                                   &log) == sizeof(exception));
    CHECK(log.count == 3u);
    CHECK(log.lengths[0] == 8u);
    CHECK(log.lengths[1] == 8u);
    CHECK(log.lengths[2] == 5u);
    return true;
}

static bool test_two_frames_and_shared_boundary_block(void)
{
    static const uint8_t first[] = {
        0x01u, 0x06u, 0x00u, 0x38u, 0x00u, 0x00u, 0x08u, 0x0Cu,
    };
    static const uint8_t second[] = {
        0x01u, 0x10u, 0x00u, 0x39u, 0x00u, 0x02u, 0x91u, 0xCDu,
    };
    uint8_t joined[sizeof(first) + sizeof(second)];
    modbus_rx_assembler_t assembler;
    frame_log_t log = {0};

    (void)memcpy(joined, first, sizeof(first));
    (void)memcpy(&joined[sizeof(first)], second, sizeof(second));
    modbus_rx_assembler_init(&assembler);
    CHECK(modbus_rx_assembler_feed(&assembler,
                                   joined,
                                   sizeof(joined),
                                   record_frame,
                                   &log) == sizeof(joined));
    CHECK(log.count == 2u);
    CHECK(memcmp(log.frames[0], first, sizeof(first)) == 0);
    CHECK(memcmp(log.frames[1], second, sizeof(second)) == 0);

    modbus_rx_assembler_reset(&assembler);
    (void)memset(&log, 0, sizeof(log));
    CHECK(modbus_rx_assembler_feed(&assembler,
                                   first,
                                   sizeof(first) - 2u,
                                   record_frame,
                                   &log) == sizeof(first) - 2u);
    CHECK(log.count == 0u);
    CHECK(modbus_rx_assembler_feed(&assembler,
                                   &joined[sizeof(first) - 2u],
                                   sizeof(joined) - (sizeof(first) - 2u),
                                   record_frame,
                                   &log) == sizeof(joined) - (sizeof(first) - 2u));
    CHECK(log.count == 2u);
    return true;
}

static bool test_incomplete_input_waits_for_more(void)
{
    static const uint8_t partial[] = {0x01u, 0x03u, 0x08u, 0x00u};
    modbus_rx_assembler_t assembler;
    frame_log_t log = {0};

    modbus_rx_assembler_init(&assembler);
    CHECK(modbus_rx_assembler_feed(&assembler,
                                   partial,
                                   sizeof(partial),
                                   record_frame,
                                   &log) == sizeof(partial));
    CHECK(log.count == 0u);
    CHECK(modbus_rx_assembler_pending_length(&assembler) == sizeof(partial));
    return true;
}

static bool test_invalid_byte_counts_and_oversize_are_rejected(void)
{
    static const uint8_t invalid_zero[] = {0x01u, 0x03u, 0x00u};
    static const uint8_t invalid_odd[] = {0x01u, 0x03u, 0x07u};
    static const uint8_t invalid_large[] = {0x01u, 0x03u, 0x0Au};
    modbus_rx_assembler_t assembler;
    frame_log_t log = {0};
    modbus_rx_assembler_stats_t stats;

    modbus_rx_assembler_init(&assembler);
    (void)modbus_rx_assembler_feed(&assembler,
                                   invalid_zero,
                                   sizeof(invalid_zero),
                                   record_frame,
                                   &log);
    (void)modbus_rx_assembler_feed(&assembler,
                                   invalid_odd,
                                   sizeof(invalid_odd),
                                   record_frame,
                                   &log);
    (void)modbus_rx_assembler_feed(&assembler,
                                   invalid_large,
                                   sizeof(invalid_large),
                                   record_frame,
                                   &log);
    stats = modbus_rx_assembler_get_stats(&assembler);
    CHECK(log.count == 0u);
    CHECK(stats.invalid_lengths == 3u);
    CHECK(stats.buffer_overflows == 1u);
    CHECK(modbus_rx_assembler_pending_length(&assembler) == 0u);
    return true;
}

static bool test_reset_and_handler_stop_discard_residual(void)
{
    static const uint8_t partial[] = {0x01u, 0x03u, 0x08u, 0x00u};
    static const uint8_t frame[] = {
        0x01u, 0x06u, 0x00u, 0x38u, 0x00u, 0x00u, 0x08u, 0x0Cu,
    };
    uint8_t two_frames[sizeof(frame) * 2u];
    modbus_rx_assembler_t assembler;
    frame_log_t log = {0};

    modbus_rx_assembler_init(&assembler);
    (void)modbus_rx_assembler_feed(&assembler,
                                   partial,
                                   sizeof(partial),
                                   record_frame,
                                   &log);
    modbus_rx_assembler_reset(&assembler);
    CHECK(modbus_rx_assembler_pending_length(&assembler) == 0u);

    (void)memcpy(two_frames, frame, sizeof(frame));
    (void)memcpy(&two_frames[sizeof(frame)], frame, sizeof(frame));
    log.stop_after = 1u;
    CHECK(modbus_rx_assembler_feed(&assembler,
                                   two_frames,
                                   sizeof(two_frames),
                                   record_frame,
                                   &log) == sizeof(frame));
    CHECK(log.count == 1u);
    CHECK(modbus_rx_assembler_pending_length(&assembler) == 0u);
    return true;
}

static bool test_crc_is_not_checked_and_next_frame_still_works(void)
{
    static const uint8_t invalid_crc[] = {
        0x01u, 0x06u, 0x00u, 0x38u, 0x00u, 0x00u, 0x00u, 0x00u,
    };
    static const uint8_t valid_shape[] = {
        0x01u, 0x06u, 0x00u, 0x38u, 0x00u, 0x00u, 0x08u, 0x0Cu,
    };
    modbus_rx_assembler_t assembler;
    frame_log_t log = {0};

    modbus_rx_assembler_init(&assembler);
    (void)modbus_rx_assembler_feed(&assembler,
                                   invalid_crc,
                                   sizeof(invalid_crc),
                                   record_frame,
                                   &log);
    (void)modbus_rx_assembler_feed(&assembler,
                                   valid_shape,
                                   sizeof(valid_shape),
                                   record_frame,
                                   &log);
    CHECK(log.count == 2u);
    CHECK(memcmp(log.frames[0], invalid_crc, sizeof(invalid_crc)) == 0);
    CHECK(memcmp(log.frames[1], valid_shape, sizeof(valid_shape)) == 0);
    return true;
}

static bool test_dma_positions_without_and_with_wrap(void)
{
    uint8_t dma[8] = {0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u};
    uint8_t value;
    uart_dma_rx_buffer_t buffer;
    size_t index;

    uart_dma_rx_buffer_init(&buffer);
    CHECK(uart_dma_rx_buffer_publish(&buffer, dma, sizeof(dma), 3u, false));
    for (index = 0u; index < 3u; ++index)
    {
        CHECK(uart_dma_rx_buffer_take(&buffer, &value));
        CHECK(value == dma[index]);
    }
    CHECK(!uart_dma_rx_buffer_take(&buffer, &value));

    CHECK(uart_dma_rx_buffer_publish(&buffer, dma, sizeof(dma), 8u, true));
    for (index = 3u; index < sizeof(dma); ++index)
    {
        CHECK(uart_dma_rx_buffer_take(&buffer, &value));
        CHECK(value == dma[index]);
    }
    dma[0] = 8u;
    dma[1] = 9u;
    CHECK(uart_dma_rx_buffer_publish(&buffer, dma, sizeof(dma), 2u, false));
    CHECK(uart_dma_rx_buffer_take(&buffer, &value) && (value == 8u));
    CHECK(uart_dma_rx_buffer_take(&buffer, &value) && (value == 9u));
    return true;
}

static bool test_dma_duplicate_positions_and_continuous_updates(void)
{
    uint8_t dma[8] = {0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u};
    uint8_t value;
    uart_dma_rx_buffer_t buffer;

    uart_dma_rx_buffer_init(&buffer);
    CHECK(uart_dma_rx_buffer_publish(&buffer, dma, sizeof(dma), 4u, false));
    CHECK(uart_dma_rx_buffer_publish(&buffer, dma, sizeof(dma), 4u, false));
    CHECK(uart_dma_rx_buffer_count(&buffer) == 4u);
    while (uart_dma_rx_buffer_take(&buffer, &value))
    {
    }
    CHECK(uart_dma_rx_buffer_publish(&buffer, dma, sizeof(dma), 8u, true));
    CHECK(uart_dma_rx_buffer_count(&buffer) == 4u);
    CHECK(uart_dma_rx_buffer_publish(&buffer, dma, sizeof(dma), 8u, false));
    CHECK(uart_dma_rx_buffer_count(&buffer) == 4u);
    return true;
}

static bool test_dma_ring_overflow_is_counted_and_recoverable(void)
{
    uint8_t dma[UART_DMA_RX_RING_SIZE + 1u];
    uart_dma_rx_buffer_t buffer;
    uart_dma_rx_buffer_stats_t stats;
    size_t index;

    for (index = 0u; index < sizeof(dma); ++index)
    {
        dma[index] = (uint8_t)index;
    }
    uart_dma_rx_buffer_init(&buffer);
    CHECK(!uart_dma_rx_buffer_publish(&buffer,
                                      dma,
                                      sizeof(dma),
                                      (uint16_t)sizeof(dma),
                                      true));
    CHECK(uart_dma_rx_buffer_take_overflow(&buffer));
    CHECK(!uart_dma_rx_buffer_take_overflow(&buffer));
    stats = uart_dma_rx_buffer_get_stats(&buffer);
    CHECK(stats.overrun_events == 1u);
    CHECK(stats.dropped_bytes == 2u);
    uart_dma_rx_buffer_discard(&buffer);
    CHECK(uart_dma_rx_buffer_count(&buffer) == 0u);
    return true;
}

static bool test_dma_invalid_position_is_rejected(void)
{
    uint8_t dma[8] = {0};
    uart_dma_rx_buffer_t buffer;
    uart_dma_rx_buffer_stats_t stats;

    uart_dma_rx_buffer_init(&buffer);
    CHECK(!uart_dma_rx_buffer_publish(&buffer, dma, sizeof(dma), 9u, false));
    stats = uart_dma_rx_buffer_get_stats(&buffer);
    CHECK(stats.invalid_positions == 1u);
    return true;
}

int main(void)
{
    RUN(test_complete_read_response);
    RUN(test_single_byte_and_arbitrary_splits);
    RUN(test_write_and_exception_responses);
    RUN(test_two_frames_and_shared_boundary_block);
    RUN(test_incomplete_input_waits_for_more);
    RUN(test_invalid_byte_counts_and_oversize_are_rejected);
    RUN(test_reset_and_handler_stop_discard_residual);
    RUN(test_crc_is_not_checked_and_next_frame_still_works);
    RUN(test_dma_positions_without_and_with_wrap);
    RUN(test_dma_duplicate_positions_and_continuous_updates);
    RUN(test_dma_ring_overflow_is_counted_and_recoverable);
    RUN(test_dma_invalid_position_is_rejected);
    printf("all %u modbus_rx_assembler tests passed\n", tests_run);
    return 0;
}
