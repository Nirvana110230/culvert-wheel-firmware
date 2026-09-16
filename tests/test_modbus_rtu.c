#include "modbus_rtu.h"

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

static void append_crc(uint8_t *frame, size_t body_length)
{
    const uint16_t crc = modbus_rtu_crc16(frame, body_length);

    frame[body_length] = (uint8_t)crc;
    frame[body_length + 1u] = (uint8_t)(crc >> 8u);
}

static bool test_known_crc_vector(void)
{
    static const uint8_t vector[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0A};

    CHECK(modbus_rtu_crc16(vector, sizeof(vector)) == 0xCDC5u);
    CHECK(modbus_rtu_crc16(NULL, 0u) == 0xFFFFu);
    return true;
}

static bool test_build_read_request_and_capacity(void)
{
    static const uint8_t expected[] = {0x01, 0x03, 0x00, 0x00,
                                       0x00, 0x0A, 0xC5, 0xCD};
    uint8_t output[8];
    size_t written = 99u;

    CHECK(modbus_rtu_build_read(1u, 0u, 10u, output, sizeof(output), &written));
    CHECK((written == sizeof(expected)) &&
          (memcmp(output, expected, sizeof(expected)) == 0));
    CHECK(!modbus_rtu_build_read(1u, 0u, 1u, output, 7u, &written));
    CHECK(written == 0u);
    CHECK(!modbus_rtu_build_read(0u, 0u, 1u, output, sizeof(output), &written));
    CHECK(!modbus_rtu_build_read(1u,
                                 0u,
                                 MODBUS_RTU_MAX_READ_REGISTERS + 1u,
                                 output,
                                 sizeof(output),
                                 &written));
    return true;
}

static bool test_build_write_single_request(void)
{
    static const uint8_t expected[] = {0x11, 0x06, 0x00, 0x01,
                                       0x00, 0x03, 0x9A, 0x9B};
    uint8_t output[8];
    size_t written;

    CHECK(modbus_rtu_build_write_single(0x11u,
                                        0x0001u,
                                        0x0003u,
                                        output,
                                        sizeof(output),
                                        &written));
    CHECK((written == sizeof(expected)) &&
          (memcmp(output, expected, sizeof(expected)) == 0));
    return true;
}

static bool test_build_write_multiple_request(void)
{
    static const uint16_t values[] = {0x1234u, 0x5678u};
    uint8_t output[13];
    size_t written;

    CHECK(modbus_rtu_build_write_multiple(1u,
                                          2u,
                                          values,
                                          2u,
                                          output,
                                          sizeof(output),
                                          &written));
    CHECK(written == sizeof(output));
    CHECK((output[1] == 0x10u) && (output[6] == 4u));
    CHECK((output[7] == 0x12u) && (output[8] == 0x34u));
    CHECK(!modbus_rtu_build_write_multiple(1u,
                                           2u,
                                           values,
                                           2u,
                                           output,
                                           sizeof(output) - 1u,
                                           &written));
    CHECK(written == 0u);
    return true;
}

static bool test_validate_read_response(void)
{
    uint8_t request[8];
    uint8_t response[] = {0x01, 0x03, 0x04, 0x12, 0x34, 0x56, 0x78, 0x00, 0x00};
    size_t request_length;

    CHECK(modbus_rtu_build_read(1u,
                                0x0020u,
                                2u,
                                request,
                                sizeof(request),
                                &request_length));
    append_crc(response, sizeof(response) - 2u);
    CHECK(modbus_rtu_validate_response(request,
                                       request_length,
                                       response,
                                       sizeof(response)));
    CHECK(!modbus_rtu_validate_response(request,
                                        request_length,
                                        response,
                                        sizeof(response) - 1u));
    return true;
}

static bool test_validate_write_echoes(void)
{
    static const uint16_t values[] = {0x1234u, 0x5678u};
    uint8_t single[8];
    uint8_t multiple[13];
    uint8_t multiple_response[] = {0x01, 0x10, 0x00, 0x02,
                                   0x00, 0x02, 0x00, 0x00};
    size_t single_length;
    size_t multiple_length;

    CHECK(modbus_rtu_build_write_single(1u,
                                        2u,
                                        3u,
                                        single,
                                        sizeof(single),
                                        &single_length));
    CHECK(modbus_rtu_validate_response(single, single_length, single, single_length));

    CHECK(modbus_rtu_build_write_multiple(1u,
                                          2u,
                                          values,
                                          2u,
                                          multiple,
                                          sizeof(multiple),
                                          &multiple_length));
    append_crc(multiple_response, 6u);
    CHECK(modbus_rtu_validate_response(multiple,
                                       multiple_length,
                                       multiple_response,
                                       sizeof(multiple_response)));
    multiple_response[5] = 3u;
    append_crc(multiple_response, 6u);
    CHECK(!modbus_rtu_validate_response(multiple,
                                        multiple_length,
                                        multiple_response,
                                        sizeof(multiple_response)));
    return true;
}

static bool test_validate_exception_response(void)
{
    uint8_t request[8];
    uint8_t response[] = {0x01, 0x83, 0x02, 0x00, 0x00};
    size_t request_length;

    CHECK(modbus_rtu_build_read(1u,
                                0u,
                                1u,
                                request,
                                sizeof(request),
                                &request_length));
    append_crc(response, 3u);
    CHECK(modbus_rtu_validate_response(request,
                                       request_length,
                                       response,
                                       sizeof(response)));
    CHECK(!modbus_rtu_validate_response(request,
                                        request_length,
                                        response,
                                        sizeof(response) - 1u));
    return true;
}

static bool test_reject_crc_slave_function_and_echo_errors(void)
{
    uint8_t request[8];
    uint8_t response[8];
    size_t request_length;

    CHECK(modbus_rtu_build_write_single(1u,
                                        2u,
                                        3u,
                                        request,
                                        sizeof(request),
                                        &request_length));
    (void)memcpy(response, request, sizeof(response));
    response[7] ^= 1u;
    CHECK(!modbus_rtu_validate_response(request,
                                        request_length,
                                        response,
                                        sizeof(response)));

    (void)memcpy(response, request, sizeof(response));
    response[0] = 2u;
    append_crc(response, 6u);
    CHECK(!modbus_rtu_validate_response(request,
                                        request_length,
                                        response,
                                        sizeof(response)));

    (void)memcpy(response, request, sizeof(response));
    response[1] = 0x03u;
    append_crc(response, 6u);
    CHECK(!modbus_rtu_validate_response(request,
                                        request_length,
                                        response,
                                        sizeof(response)));

    (void)memcpy(response, request, sizeof(response));
    response[3] ^= 1u;
    append_crc(response, 6u);
    CHECK(!modbus_rtu_validate_response(request,
                                        request_length,
                                        response,
                                        sizeof(response)));
    return true;
}

static bool test_reject_truncated_and_trailing_bytes(void)
{
    uint8_t request[8];
    uint8_t response[] = {0x01, 0x03, 0x02, 0x00, 0x0A, 0x00, 0x00, 0x00};
    size_t request_length;

    CHECK(modbus_rtu_build_read(1u,
                                0u,
                                1u,
                                request,
                                sizeof(request),
                                &request_length));
    append_crc(response, 5u);
    CHECK(!modbus_rtu_validate_response(request, request_length, response, 6u));
    CHECK(!modbus_rtu_validate_response(request,
                                        request_length,
                                        response,
                                        sizeof(response)));
    return true;
}

int main(void)
{
    RUN(test_known_crc_vector);
    RUN(test_build_read_request_and_capacity);
    RUN(test_build_write_single_request);
    RUN(test_build_write_multiple_request);
    RUN(test_validate_read_response);
    RUN(test_validate_write_echoes);
    RUN(test_validate_exception_response);
    RUN(test_reject_crc_slave_function_and_echo_errors);
    RUN(test_reject_truncated_and_trailing_bytes);
    printf("all %u modbus_rtu tests passed\n", tests_run);
    return 0;
}
