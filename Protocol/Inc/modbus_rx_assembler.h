#ifndef MODBUS_RX_ASSEMBLER_H
#define MODBUS_RX_ASSEMBLER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Current MSSD reads return four registers (eight data bytes) at most. */
#define MODBUS_RX_ASSEMBLER_MAX_READ_DATA 8u
#define MODBUS_RX_ASSEMBLER_MAX_FRAME_SIZE \
    (MODBUS_RX_ASSEMBLER_MAX_READ_DATA + 5u)

typedef bool (*modbus_rx_frame_handler_t)(const uint8_t *frame,
                                          size_t length,
                                          void *context);

typedef struct
{
    uint32_t frames_emitted;
    uint32_t unsupported_functions;
    uint32_t invalid_lengths;
    uint32_t buffer_overflows;
    uint32_t discarded_bytes;
    uint32_t explicit_resets;
} modbus_rx_assembler_stats_t;

typedef struct
{
    uint8_t buffer[MODBUS_RX_ASSEMBLER_MAX_FRAME_SIZE];
    size_t used;
    size_t expected_length;
    modbus_rx_assembler_stats_t stats;
} modbus_rx_assembler_t;

void modbus_rx_assembler_init(modbus_rx_assembler_t *assembler);

/* Clears a partial candidate without changing accumulated diagnostics. */
void modbus_rx_assembler_reset(modbus_rx_assembler_t *assembler);

/*
 * Consumes bytes until input is exhausted or handler asks to stop.
 * The returned count is the number of input bytes consumed synchronously.
 */
size_t modbus_rx_assembler_feed(modbus_rx_assembler_t *assembler,
                                const uint8_t *data,
                                size_t length,
                                modbus_rx_frame_handler_t handler,
                                void *context);

size_t modbus_rx_assembler_pending_length(const modbus_rx_assembler_t *assembler);
modbus_rx_assembler_stats_t modbus_rx_assembler_get_stats(
    const modbus_rx_assembler_t *assembler);

#endif
