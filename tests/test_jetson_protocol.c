#include "jetson_protocol.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

static unsigned tests_run;

#define CHECK(condition)                                                                    \
    do                                                                                      \
    {                                                                                       \
        if (!(condition))                                                                   \
        {                                                                                   \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition);               \
            return false;                                                                   \
        }                                                                                   \
    } while (0)

#define RUN(test)                                                                           \
    do                                                                                      \
    {                                                                                       \
        if (!(test()))                                                                      \
        {                                                                                   \
            return 1;                                                                       \
        }                                                                                   \
        ++tests_run;                                                                        \
    } while (0)

static const uint8_t motion_golden[] = {
    0xFF, 0xFC, 0x0A, 0x12, 0x01, 0x00, 0x80, 0x00, 0x00, 0xFF, 0x7F, 0x1B};

static bool take_motion(jetson_protocol_t *parser, jetson_motion_frame_t *motion)
{
    jetson_frame_t frame;

    return jetson_protocol_take_frame(parser, &frame) &&
           jetson_protocol_decode_motion(&frame, motion);
}

static bool test_old_motion_golden_len_checksum_and_signed_values(void)
{
    jetson_protocol_t parser;
    jetson_motion_frame_t motion;

    jetson_protocol_init(&parser);
    CHECK(sizeof(motion_golden) == ((size_t)motion_golden[2] + 2u));
    jetson_protocol_feed(&parser, motion_golden, sizeof(motion_golden), 42u);
    CHECK(take_motion(&parser, &motion));
    CHECK((motion.mode == 1u) && (motion.vx == INT16_MIN));
    CHECK((motion.vy == 0) && (motion.vz == INT16_MAX));
    CHECK(motion.timestamp_ms == 42u);
    return true;
}

static bool test_old_receive_golden_frames(void)
{
    static const uint8_t frames[] = {
        0xFF, 0xFC, 0x05, 0x51, 0x01, 0x00, 0x57,
        0xFF, 0xFC, 0x05, 0x50, 0x51, 0x00, 0xA6};
    jetson_protocol_t parser;
    jetson_frame_t frame;

    jetson_protocol_init(&parser);
    jetson_protocol_feed(&parser, frames, sizeof(frames), 1u);
    CHECK(jetson_protocol_take_frame(&parser, &frame));
    CHECK((frame.function == 0x51u) && (frame.payload_length == 2u));
    CHECK(jetson_protocol_take_frame(&parser, &frame));
    CHECK((frame.function == 0x50u) && (frame.payload_length == 2u));
    CHECK(!jetson_protocol_take_frame(&parser, &frame));
    return true;
}

static bool test_old_transmit_golden_frames(void)
{
    static const uint8_t self_check_payload[] = {0x00};
    static const uint8_t self_check_expected[] = {0xFF, 0xFB, 0x04, 0x51,
                                                   0x00, 0x55};
    static const uint8_t inspect_payload[] = {0x01};
    static const uint8_t inspect_expected[] = {0xFF, 0xFB, 0x04, 0xA3,
                                                0x01, 0xA8};
    static const uint8_t emergency_expected[] = {0xFF, 0xFB, 0x04, 0xA4,
                                                  0x01, 0xA9};
    static const uint8_t mode_expected[] = {0xFF, 0xFB, 0x04, 0xA5, 0x01, 0xAA};
    uint8_t output[16];
    size_t written;

    CHECK(jetson_protocol_build(JETSON_PROTOCOL_TX_HEAD,
                                JETSON_FUNC_LEGACY_VERSION_OR_SELF_CHECK,
                                self_check_payload,
                                sizeof(self_check_payload),
                                output,
                                sizeof(output),
                                &written));
    CHECK((written == sizeof(self_check_expected)) &&
          (memcmp(output, self_check_expected, written) == 0));
    CHECK(jetson_protocol_build(JETSON_PROTOCOL_TX_HEAD,
                                JETSON_FUNC_LEGACY_INSPECT_CTRL,
                                inspect_payload,
                                sizeof(inspect_payload),
                                output,
                                sizeof(output),
                                &written));
    CHECK((written == sizeof(inspect_expected)) &&
          (memcmp(output, inspect_expected, written) == 0));
    CHECK(jetson_protocol_build(JETSON_PROTOCOL_TX_HEAD,
                                JETSON_FUNC_LEGACY_EMERGENCY,
                                inspect_payload,
                                sizeof(inspect_payload),
                                output,
                                sizeof(output),
                                &written));
    CHECK((written == sizeof(emergency_expected)) &&
          (memcmp(output, emergency_expected, written) == 0));
    CHECK(jetson_protocol_build(JETSON_PROTOCOL_TX_HEAD,
                                JETSON_FUNC_LEGACY_MODE_REPORT,
                                inspect_payload,
                                sizeof(inspect_payload),
                                output,
                                sizeof(output),
                                &written));
    CHECK((written == sizeof(mode_expected)) &&
          (memcmp(output, mode_expected, written) == 0));
    return true;
}

static bool test_single_byte_segments_and_leading_noise(void)
{
    static const uint8_t noise[] = {0x01, 0xFF, 0x00};
    jetson_protocol_t parser;
    jetson_motion_frame_t motion;
    size_t index;

    jetson_protocol_init(&parser);
    jetson_protocol_feed(&parser, noise, sizeof(noise), 0u);
    for (index = 0u; index < sizeof(motion_golden); ++index)
    {
        jetson_protocol_feed(&parser, &motion_golden[index], 1u, (uint32_t)index);
    }
    CHECK(take_motion(&parser, &motion));
    CHECK(parser.stats.bad_header == 1u);
    return true;
}

static bool test_checksum_error_resynchronizes_to_next_frame(void)
{
    uint8_t stream[(sizeof(motion_golden) * 2u)];
    jetson_protocol_t parser;
    jetson_motion_frame_t motion;

    (void)memcpy(stream, motion_golden, sizeof(motion_golden));
    stream[sizeof(motion_golden) - 1u] ^= 1u;
    (void)memcpy(&stream[sizeof(motion_golden)], motion_golden, sizeof(motion_golden));

    jetson_protocol_init(&parser);
    jetson_protocol_feed(&parser, stream, sizeof(stream), 10u);
    CHECK(parser.stats.bad_checksum == 1u);
    CHECK(take_motion(&parser, &motion));
    return true;
}

static bool test_reject_bad_lengths_and_unknown_function(void)
{
    static const uint8_t too_short[] = {0xFF, 0xFC, 0x02};
    static const uint8_t too_long[] = {0xFF, 0xFC, JETSON_PROTOCOL_MAX_PAYLOAD + 4u};
    static const uint8_t unknown[] = {0xFF, 0xFC, 0x03, 0x99, 0x9C};
    jetson_protocol_t parser;

    jetson_protocol_init(&parser);
    jetson_protocol_feed(&parser, too_short, sizeof(too_short), 0u);
    jetson_protocol_feed(&parser, too_long, sizeof(too_long), 0u);
    jetson_protocol_feed(&parser, unknown, sizeof(unknown), 0u);
    CHECK(parser.stats.bad_length == 2u);
    CHECK(parser.stats.bad_function == 1u);
    return true;
}

static bool test_build_minimum_and_largest_defined_payload(void)
{
    static const uint8_t status_payload[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t output[JETSON_PROTOCOL_MAX_FRAME_SIZE];
    size_t written;

    CHECK(jetson_protocol_build(JETSON_PROTOCOL_TX_HEAD,
                                JETSON_FUNC_INSPECT_REQUEST,
                                NULL,
                                0u,
                                output,
                                sizeof(output),
                                &written));
    CHECK((written == 5u) && (output[2] == 3u));
    CHECK(jetson_protocol_build(JETSON_PROTOCOL_TX_HEAD,
                                JETSON_FUNC_STATUS,
                                status_payload,
                                sizeof(status_payload),
                                output,
                                sizeof(output),
                                &written));
    CHECK((written == 9u) && (output[2] == 7u));
    return true;
}

static bool test_build_rejects_wrong_payload_and_small_output(void)
{
    static const uint8_t status_payload[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t oversized[JETSON_PROTOCOL_MAX_PAYLOAD] = {0u};
    uint8_t output[9];
    size_t written = 99u;

    CHECK(!jetson_protocol_build(JETSON_PROTOCOL_TX_HEAD,
                                 JETSON_FUNC_STATUS,
                                 oversized,
                                 sizeof(oversized),
                                 output,
                                 sizeof(output),
                                 &written));
    CHECK(written == 0u);
    CHECK(!jetson_protocol_build(JETSON_PROTOCOL_TX_HEAD,
                                 JETSON_FUNC_STATUS,
                                 status_payload,
                                 sizeof(status_payload),
                                 output,
                                 sizeof(output) - 1u,
                                 &written));
    CHECK(written == 0u);
    return true;
}

static bool test_timeout_at_exact_deadline(void)
{
    static const uint8_t partial[] = {0xFF, 0xFC, 0x0A};
    jetson_protocol_t parser;

    jetson_protocol_init(&parser);
    jetson_protocol_feed(&parser, partial, sizeof(partial), 10u);
    jetson_protocol_poll(&parser, 10u + JETSON_PROTOCOL_FRAME_TIMEOUT_MS - 1u);
    CHECK(parser.used != 0u);
    jetson_protocol_poll(&parser, 10u + JETSON_PROTOCOL_FRAME_TIMEOUT_MS);
    CHECK((parser.stats.frame_timeouts == 1u) && (parser.used == 0u));
    return true;
}

static bool test_timeout_handles_tick_wrap(void)
{
    static const uint8_t partial[] = {0xFF, 0xFC, 0x0A};
    jetson_protocol_t parser;
    const uint32_t started = UINT32_MAX - 49u;
    const uint32_t deadline = started + JETSON_PROTOCOL_FRAME_TIMEOUT_MS;

    jetson_protocol_init(&parser);
    jetson_protocol_feed(&parser, partial, sizeof(partial), started);
    jetson_protocol_poll(&parser, deadline);
    CHECK((parser.stats.frame_timeouts == 1u) && (parser.used == 0u));
    return true;
}

static bool test_queue_uses_all_declared_slots_and_counts_overflow(void)
{
    static const uint8_t frame[] = {0xFF, 0xFC, 0x03, 0xB0, 0xB3};
    jetson_protocol_t parser;
    jetson_frame_t received;
    size_t index;

    jetson_protocol_init(&parser);
    for (index = 0u; index < JETSON_PROTOCOL_QUEUE_CAPACITY; ++index)
    {
        jetson_protocol_feed(&parser, frame, sizeof(frame), (uint32_t)index);
    }
    for (index = 0u; index < JETSON_PROTOCOL_QUEUE_CAPACITY; ++index)
    {
        CHECK(jetson_protocol_take_frame(&parser, &received));
    }
    CHECK(!jetson_protocol_take_frame(&parser, &received));

    for (index = 0u; index < JETSON_PROTOCOL_QUEUE_CAPACITY + 1u; ++index)
    {
        jetson_protocol_feed(&parser, frame, sizeof(frame), (uint32_t)index);
    }
    CHECK(parser.stats.queue_overflows == 1u);
    return true;
}

int main(void)
{
    RUN(test_old_motion_golden_len_checksum_and_signed_values);
    RUN(test_old_receive_golden_frames);
    RUN(test_old_transmit_golden_frames);
    RUN(test_single_byte_segments_and_leading_noise);
    RUN(test_checksum_error_resynchronizes_to_next_frame);
    RUN(test_reject_bad_lengths_and_unknown_function);
    RUN(test_build_minimum_and_largest_defined_payload);
    RUN(test_build_rejects_wrong_payload_and_small_output);
    RUN(test_timeout_at_exact_deadline);
    RUN(test_timeout_handles_tick_wrap);
    RUN(test_queue_uses_all_declared_slots_and_counts_overflow);
    printf("all %u jetson_protocol tests passed\n", tests_run);
    return 0;
}
