#include "jetson_protocol.h"

#include <string.h>

static uint8_t checksum(const uint8_t *data, size_t length)
{
    uint8_t result = 0u;
    size_t index;

    for (index = 0u; index < length; ++index)
    {
        result = (uint8_t)(result + data[index]);
    }
    return result;
}

static bool payload_length_is_valid(uint8_t function, uint8_t length)
{
    switch (function)
    {
        case JETSON_FUNC_MOTION:
            return length == 7u;
        case JETSON_FUNC_REQUEST_DATA:
            return length == 2u;
        case JETSON_FUNC_LEGACY_VERSION_OR_SELF_CHECK:
            return (length == 1u) || (length == 2u);
        case JETSON_FUNC_LEGACY_INSPECT_CTRL:
            return (length == 1u) || (length == 2u);
        case JETSON_FUNC_LEGACY_EMERGENCY:
        case JETSON_FUNC_LEGACY_MODE_REPORT:
            return length == 1u;
        case JETSON_FUNC_INSPECT_REQUEST:
        case JETSON_FUNC_INSPECT_ACCEPTED:
        case JETSON_FUNC_INSPECT_REJECTED:
        case JETSON_FUNC_INSPECT_FINISHED:
        case JETSON_FUNC_INSPECT_CANCELLED:
            return length == 0u;
        case JETSON_FUNC_STATUS:
            return length == 4u;
        case JETSON_FUNC_HEARTBEAT:
            return length == 1u;
        default:
            return false;
    }
}

static void restart_from_byte(jetson_protocol_t *parser, uint8_t byte, uint32_t now_ms)
{
    parser->used = 0u;
    parser->expected_size = 0u;
    if (byte == 0xFFu)
    {
        parser->bytes[0] = byte;
        parser->used = 1u;
        parser->frame_start_ms = now_ms;
    }
}

static void queue_frame(jetson_protocol_t *parser, uint32_t now_ms)
{
    const uint8_t payload_length = (uint8_t)(parser->bytes[2] - 3u);
    jetson_frame_t *frame;

    if (!payload_length_is_valid(parser->bytes[3], payload_length))
    {
        ++parser->stats.bad_function;
        return;
    }
    if (parser->queue_count >= JETSON_PROTOCOL_QUEUE_CAPACITY)
    {
        ++parser->stats.queue_overflows;
        return;
    }

    frame = &parser->queue[parser->queue_head];
    frame->function = parser->bytes[3];
    frame->payload_length = payload_length;
    frame->timestamp_ms = now_ms;
    if (payload_length != 0u)
    {
        (void)memcpy(frame->payload, &parser->bytes[4], payload_length);
    }
    parser->queue_head = (uint8_t)((parser->queue_head + 1u) %
                                   JETSON_PROTOCOL_QUEUE_CAPACITY);
    ++parser->queue_count;
}

void jetson_protocol_init(jetson_protocol_t *parser)
{
    if (parser != NULL)
    {
        (void)memset(parser, 0, sizeof(*parser));
    }
}

void jetson_protocol_reset_partial_frame(jetson_protocol_t *parser)
{
    if (parser != NULL)
    {
        parser->used = 0u;
        parser->expected_size = 0u;
    }
}

void jetson_protocol_feed(jetson_protocol_t *parser,
                          const uint8_t *data,
                          size_t length,
                          uint32_t now_ms)
{
    size_t index;

    if ((parser == NULL) || ((data == NULL) && (length != 0u)))
    {
        return;
    }

    jetson_protocol_poll(parser, now_ms);
    for (index = 0u; index < length; ++index)
    {
        const uint8_t byte = data[index];

        if (parser->used == 0u)
        {
            if (byte == 0xFFu)
            {
                restart_from_byte(parser, byte, now_ms);
            }
            continue;
        }
        if (parser->used == 1u)
        {
            if (byte == JETSON_PROTOCOL_RX_HEAD)
            {
                parser->bytes[1] = byte;
                parser->used = 2u;
            }
            else
            {
                ++parser->stats.bad_header;
                restart_from_byte(parser, byte, now_ms);
            }
            continue;
        }
        if (parser->used == 2u)
        {
            if ((byte < 3u) || (byte > (JETSON_PROTOCOL_MAX_PAYLOAD + 3u)))
            {
                ++parser->stats.bad_length;
                restart_from_byte(parser, byte, now_ms);
                continue;
            }
            parser->bytes[2] = byte;
            parser->used = 3u;
            parser->expected_size = (uint8_t)(byte + 2u);
            continue;
        }

        parser->bytes[parser->used++] = byte;
        if (parser->used == parser->expected_size)
        {
            const uint8_t received = parser->bytes[parser->expected_size - 1u];
            const uint8_t calculated =
                checksum(&parser->bytes[2], (size_t)parser->bytes[2] - 1u);

            if (received == calculated)
            {
                queue_frame(parser, now_ms);
            }
            else
            {
                ++parser->stats.bad_checksum;
            }
            restart_from_byte(parser, byte, now_ms);
        }
    }
}

void jetson_protocol_poll(jetson_protocol_t *parser, uint32_t now_ms)
{
    if ((parser != NULL) && (parser->used != 0u) &&
        ((uint32_t)(now_ms - parser->frame_start_ms) >=
         JETSON_PROTOCOL_FRAME_TIMEOUT_MS))
    {
        ++parser->stats.frame_timeouts;
        jetson_protocol_reset_partial_frame(parser);
    }
}

bool jetson_protocol_take_frame(jetson_protocol_t *parser, jetson_frame_t *frame)
{
    if ((parser == NULL) || (frame == NULL) || (parser->queue_count == 0u))
    {
        return false;
    }
    *frame = parser->queue[parser->queue_tail];
    parser->queue_tail = (uint8_t)((parser->queue_tail + 1u) %
                                   JETSON_PROTOCOL_QUEUE_CAPACITY);
    --parser->queue_count;
    return true;
}

static int16_t read_i16_le(const uint8_t *data)
{
    const uint16_t raw = (uint16_t)data[0] | ((uint16_t)data[1] << 8u);
    return (int16_t)raw;
}

bool jetson_protocol_decode_motion(const jetson_frame_t *frame,
                                   jetson_motion_frame_t *motion)
{
    if ((frame == NULL) || (motion == NULL) ||
        (frame->function != JETSON_FUNC_MOTION) ||
        (frame->payload_length != 7u))
    {
        return false;
    }
    motion->mode = frame->payload[0];
    motion->vx = read_i16_le(&frame->payload[1]);
    motion->vy = read_i16_le(&frame->payload[3]);
    motion->vz = read_i16_le(&frame->payload[5]);
    motion->timestamp_ms = frame->timestamp_ms;
    return true;
}

bool jetson_protocol_decode_self_check(const jetson_frame_t *frame,
                                       bool *passed,
                                       uint8_t *error_code)
{
    uint8_t error;

    if ((frame == NULL) || (passed == NULL) || (error_code == NULL) ||
        (frame->function != JETSON_FUNC_LEGACY_VERSION_OR_SELF_CHECK) ||
        (frame->payload_length != 2u))
    {
        return false;
    }
    error = frame->payload[1];
    *error_code = error;
    *passed = (frame->payload[0] == 1u) && (error == 0u);
    return (frame->payload[0] <= 1u);
}

bool jetson_protocol_decode_legacy_inspect_reply(const jetson_frame_t *frame,
                                                 uint8_t *command,
                                                 bool *accepted)
{
    if ((frame == NULL) || (command == NULL) || (accepted == NULL) ||
        (frame->function != JETSON_FUNC_LEGACY_INSPECT_CTRL) ||
        (frame->payload_length != 2u) ||
        (frame->payload[0] > 1u) || (frame->payload[1] > 1u))
    {
        return false;
    }
    *command = frame->payload[0];
    *accepted = (frame->payload[1] == 1u);
    return true;
}

bool jetson_protocol_build(uint8_t direction_head,
                           uint8_t function,
                           const uint8_t *payload,
                           uint8_t payload_length,
                           uint8_t *output,
                           size_t capacity,
                           size_t *written)
{
    const uint8_t length_field = (uint8_t)(payload_length + 3u);
    const size_t frame_size = (size_t)payload_length + 5u;

    if (written != NULL)
    {
        *written = 0u;
    }
    if ((output == NULL) || (written == NULL) ||
        ((payload == NULL) && (payload_length != 0u)) ||
        ((direction_head != JETSON_PROTOCOL_RX_HEAD) &&
         (direction_head != JETSON_PROTOCOL_TX_HEAD)) ||
        (payload_length > JETSON_PROTOCOL_MAX_PAYLOAD) ||
        !payload_length_is_valid(function, payload_length) ||
        (capacity < frame_size))
    {
        return false;
    }

    output[0] = 0xFFu;
    output[1] = direction_head;
    output[2] = length_field;
    output[3] = function;
    if (payload_length != 0u)
    {
        (void)memcpy(&output[4], payload, payload_length);
    }
    output[frame_size - 1u] = checksum(&output[2], frame_size - 3u);
    *written = frame_size;
    return true;
}
