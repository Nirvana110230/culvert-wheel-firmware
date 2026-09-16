#ifndef JETSON_PROTOCOL_H
#define JETSON_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define JETSON_PROTOCOL_RX_HEAD 0xFCu
#define JETSON_PROTOCOL_TX_HEAD 0xFBu
#define JETSON_PROTOCOL_MAX_PAYLOAD 32u
#define JETSON_PROTOCOL_MAX_FRAME_SIZE (JETSON_PROTOCOL_MAX_PAYLOAD + 5u)
#define JETSON_PROTOCOL_FRAME_TIMEOUT_MS 100u
#define JETSON_PROTOCOL_QUEUE_CAPACITY 4u

typedef enum
{
    JETSON_FUNC_MOTION = 0x12,
    JETSON_FUNC_REQUEST_DATA = 0x50,
    JETSON_FUNC_LEGACY_VERSION_OR_SELF_CHECK = 0x51,
    JETSON_FUNC_LEGACY_INSPECT_CTRL = 0xA3,
    JETSON_FUNC_LEGACY_EMERGENCY = 0xA4,
    JETSON_FUNC_LEGACY_MODE_REPORT = 0xA5,
    JETSON_FUNC_INSPECT_REQUEST = 0xB0,
    JETSON_FUNC_INSPECT_ACCEPTED = 0xB1,
    JETSON_FUNC_INSPECT_REJECTED = 0xB2,
    JETSON_FUNC_INSPECT_FINISHED = 0xB3,
    JETSON_FUNC_INSPECT_CANCELLED = 0xB4,
    JETSON_FUNC_STATUS = 0xB5,
    JETSON_FUNC_HEARTBEAT = 0xB6
} jetson_function_t;

typedef struct
{
    uint8_t function;
    uint8_t payload[JETSON_PROTOCOL_MAX_PAYLOAD];
    uint8_t payload_length;
    uint32_t timestamp_ms;
} jetson_frame_t;

typedef struct
{
    int16_t vx;
    int16_t vy;
    int16_t vz;
    uint8_t mode;
    uint32_t timestamp_ms;
} jetson_motion_frame_t;

typedef struct
{
    uint32_t bad_header;
    uint32_t bad_length;
    uint32_t bad_checksum;
    uint32_t bad_function;
    uint32_t frame_timeouts;
    uint32_t queue_overflows;
} jetson_protocol_stats_t;

typedef struct
{
    uint8_t bytes[JETSON_PROTOCOL_MAX_FRAME_SIZE];
    uint8_t used;
    uint8_t expected_size;
    uint32_t frame_start_ms;
    jetson_frame_t queue[JETSON_PROTOCOL_QUEUE_CAPACITY];
    uint8_t queue_head;
    uint8_t queue_tail;
    uint8_t queue_count;
    jetson_protocol_stats_t stats;
} jetson_protocol_t;

void jetson_protocol_init(jetson_protocol_t *parser);
void jetson_protocol_reset_partial_frame(jetson_protocol_t *parser);
void jetson_protocol_feed(jetson_protocol_t *parser,
                          const uint8_t *data,
                          size_t length,
                          uint32_t now_ms);
void jetson_protocol_poll(jetson_protocol_t *parser, uint32_t now_ms);
bool jetson_protocol_take_frame(jetson_protocol_t *parser, jetson_frame_t *frame);
bool jetson_protocol_decode_motion(const jetson_frame_t *frame,
                                   jetson_motion_frame_t *motion);
bool jetson_protocol_build(uint8_t direction_head,
                           uint8_t function,
                           const uint8_t *payload,
                           uint8_t payload_length,
                           uint8_t *output,
                           size_t capacity,
                           size_t *written);

#endif
