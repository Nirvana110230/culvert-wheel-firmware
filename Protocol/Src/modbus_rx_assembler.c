#include "modbus_rx_assembler.h"

#include <string.h>

#define MODBUS_FUNCTION_READ_HOLDING 0x03u
#define MODBUS_FUNCTION_WRITE_SINGLE 0x06u
#define MODBUS_FUNCTION_WRITE_MULTIPLE 0x10u
#define MODBUS_EXCEPTION_MASK 0x80u
#define MODBUS_EXCEPTION_FRAME_SIZE 5u
#define MODBUS_WRITE_RESPONSE_SIZE 8u

static bool function_is_supported(uint8_t function)
{
    return (function == MODBUS_FUNCTION_READ_HOLDING) ||
           (function == MODBUS_FUNCTION_WRITE_SINGLE) ||
           (function == MODBUS_FUNCTION_WRITE_MULTIPLE);
}

static bool function_is_supported_exception(uint8_t function)
{
    return ((function & MODBUS_EXCEPTION_MASK) != 0u) &&
           function_is_supported((uint8_t)(function & ~MODBUS_EXCEPTION_MASK));
}

static void clear_candidate(modbus_rx_assembler_t *assembler)
{
    assembler->used = 0u;
    assembler->expected_length = 0u;
}

static void discard_candidate(modbus_rx_assembler_t *assembler)
{
    assembler->stats.discarded_bytes += (uint32_t)assembler->used;
    clear_candidate(assembler);
}

static void discard_first_byte(modbus_rx_assembler_t *assembler)
{
    if (assembler->used > 1u)
    {
        (void)memmove(assembler->buffer,
                      &assembler->buffer[1],
                      assembler->used - 1u);
    }
    --assembler->used;
    assembler->expected_length = 0u;
    ++assembler->stats.discarded_bytes;
}

static bool determine_expected_length(modbus_rx_assembler_t *assembler)
{
    const uint8_t function = assembler->buffer[1];

    if (function_is_supported_exception(function))
    {
        assembler->expected_length = MODBUS_EXCEPTION_FRAME_SIZE;
        return true;
    }
    if ((function == MODBUS_FUNCTION_WRITE_SINGLE) ||
        (function == MODBUS_FUNCTION_WRITE_MULTIPLE))
    {
        assembler->expected_length = MODBUS_WRITE_RESPONSE_SIZE;
        return true;
    }
    if (function == MODBUS_FUNCTION_READ_HOLDING)
    {
        uint8_t byte_count;

        if (assembler->used < 3u)
        {
            return true;
        }
        byte_count = assembler->buffer[2];
        if ((byte_count == 0u) || ((byte_count & 1u) != 0u) ||
            (byte_count > MODBUS_RX_ASSEMBLER_MAX_READ_DATA))
        {
            ++assembler->stats.invalid_lengths;
            if (byte_count > MODBUS_RX_ASSEMBLER_MAX_READ_DATA)
            {
                ++assembler->stats.buffer_overflows;
            }
            discard_candidate(assembler);
            return false;
        }
        assembler->expected_length = (size_t)byte_count + 5u;
        return true;
    }

    ++assembler->stats.unsupported_functions;
    discard_first_byte(assembler);
    return false;
}

void modbus_rx_assembler_init(modbus_rx_assembler_t *assembler)
{
    if (assembler != NULL)
    {
        (void)memset(assembler, 0, sizeof(*assembler));
    }
}

void modbus_rx_assembler_reset(modbus_rx_assembler_t *assembler)
{
    if (assembler != NULL)
    {
        discard_candidate(assembler);
        ++assembler->stats.explicit_resets;
    }
}

size_t modbus_rx_assembler_feed(modbus_rx_assembler_t *assembler,
                                const uint8_t *data,
                                size_t length,
                                modbus_rx_frame_handler_t handler,
                                void *context)
{
    size_t consumed;

    if ((assembler == NULL) || (data == NULL) || (handler == NULL))
    {
        return 0u;
    }

    for (consumed = 0u; consumed < length; ++consumed)
    {
        if (assembler->used >= sizeof(assembler->buffer))
        {
            ++assembler->stats.buffer_overflows;
            discard_candidate(assembler);
        }

        assembler->buffer[assembler->used] = data[consumed];
        ++assembler->used;

        while ((assembler->used >= 2u) &&
               (assembler->expected_length == 0u) &&
               !determine_expected_length(assembler))
        {
        }

        if ((assembler->expected_length != 0u) &&
            (assembler->used == assembler->expected_length))
        {
            const size_t frame_length = assembler->used;
            bool keep_receiving;

            ++assembler->stats.frames_emitted;
            assembler->used = 0u;
            assembler->expected_length = 0u;
            keep_receiving = handler(assembler->buffer, frame_length, context);
            if (!keep_receiving)
            {
                return consumed + 1u;
            }
        }
    }
    return consumed;
}

size_t modbus_rx_assembler_pending_length(const modbus_rx_assembler_t *assembler)
{
    return (assembler == NULL) ? 0u : assembler->used;
}

modbus_rx_assembler_stats_t modbus_rx_assembler_get_stats(
    const modbus_rx_assembler_t *assembler)
{
    modbus_rx_assembler_stats_t stats = {0};

    if (assembler != NULL)
    {
        stats = assembler->stats;
    }
    return stats;
}
