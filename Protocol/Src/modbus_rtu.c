#include "modbus_rtu.h"

enum
{
    MODBUS_FUNC_READ_HOLDING = 0x03u,
    MODBUS_FUNC_WRITE_SINGLE = 0x06u,
    MODBUS_FUNC_WRITE_MULTIPLE = 0x10u,
    MODBUS_EXCEPTION_MASK = 0x80u
};

static void write_u16_be(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)(value >> 8u);
    destination[1] = (uint8_t)value;
}

static uint16_t read_u16_be(const uint8_t *source)
{
    return (uint16_t)(((uint16_t)source[0] << 8u) | source[1]);
}

static bool slave_is_unicast(uint8_t slave)
{
    return (slave >= 1u) && (slave <= 247u);
}

static bool slave_is_write_target(uint8_t slave)
{
    return slave <= 247u;
}

static bool append_crc(uint8_t *output, size_t body_length, size_t capacity, size_t *written)
{
    uint16_t crc;

    if (written != NULL)
    {
        *written = 0u;
    }
    if ((output == NULL) || (written == NULL) || (capacity < 2u) ||
        (body_length > (capacity - 2u)))
    {
        return false;
    }

    crc = modbus_rtu_crc16(output, body_length);
    output[body_length] = (uint8_t)crc;
    output[body_length + 1u] = (uint8_t)(crc >> 8u);
    *written = body_length + 2u;
    return true;
}

static bool crc_is_valid(const uint8_t *frame, size_t length)
{
    uint16_t expected;

    if ((frame == NULL) || (length < 2u))
    {
        return false;
    }
    expected = modbus_rtu_crc16(frame, length - 2u);
    return (frame[length - 2u] == (uint8_t)expected) &&
           (frame[length - 1u] == (uint8_t)(expected >> 8u));
}

uint16_t modbus_rtu_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFu;
    size_t byte_index;

    if ((data == NULL) && (length != 0u))
    {
        return 0u;
    }

    for (byte_index = 0u; byte_index < length; ++byte_index)
    {
        uint8_t bit_index;

        crc ^= data[byte_index];
        for (bit_index = 0u; bit_index < 8u; ++bit_index)
        {
            if ((crc & 1u) != 0u)
            {
                crc = (uint16_t)((crc >> 1u) ^ 0xA001u);
            }
            else
            {
                crc >>= 1u;
            }
        }
    }
    return crc;
}

bool modbus_rtu_build_read(uint8_t slave,
                           uint16_t address,
                           uint16_t count,
                           uint8_t *output,
                           size_t capacity,
                           size_t *written)
{
    if (written != NULL)
    {
        *written = 0u;
    }
    if (!slave_is_unicast(slave) || (count == 0u) ||
        (count > MODBUS_RTU_MAX_READ_REGISTERS) || (output == NULL) ||
        (written == NULL) || (capacity < 8u))
    {
        return false;
    }

    output[0] = slave;
    output[1] = MODBUS_FUNC_READ_HOLDING;
    write_u16_be(&output[2], address);
    write_u16_be(&output[4], count);
    return append_crc(output, 6u, capacity, written);
}

bool modbus_rtu_build_write_single(uint8_t slave,
                                   uint16_t address,
                                   uint16_t value,
                                   uint8_t *output,
                                   size_t capacity,
                                   size_t *written)
{
    if (written != NULL)
    {
        *written = 0u;
    }
    if (!slave_is_write_target(slave) || (output == NULL) || (written == NULL) ||
        (capacity < 8u))
    {
        return false;
    }

    output[0] = slave;
    output[1] = MODBUS_FUNC_WRITE_SINGLE;
    write_u16_be(&output[2], address);
    write_u16_be(&output[4], value);
    return append_crc(output, 6u, capacity, written);
}

bool modbus_rtu_build_write_multiple(uint8_t slave,
                                     uint16_t address,
                                     const uint16_t *values,
                                     uint16_t count,
                                     uint8_t *output,
                                     size_t capacity,
                                     size_t *written)
{
    size_t index;
    size_t body_length;
    size_t frame_length;

    if (written != NULL)
    {
        *written = 0u;
    }
    if (!slave_is_write_target(slave) || (values == NULL) || (count == 0u) ||
        (count > MODBUS_RTU_MAX_WRITE_REGISTERS) || (output == NULL) ||
        (written == NULL))
    {
        return false;
    }

    body_length = 7u + ((size_t)count * 2u);
    frame_length = body_length + 2u;
    if (capacity < frame_length)
    {
        return false;
    }

    output[0] = slave;
    output[1] = MODBUS_FUNC_WRITE_MULTIPLE;
    write_u16_be(&output[2], address);
    write_u16_be(&output[4], count);
    output[6] = (uint8_t)(count * 2u);
    for (index = 0u; index < count; ++index)
    {
        write_u16_be(&output[7u + (index * 2u)], values[index]);
    }
    return append_crc(output, body_length, capacity, written);
}

static bool request_is_valid(const uint8_t *request, size_t length)
{
    uint16_t count;

    if ((request == NULL) || (length < 8u) || !slave_is_unicast(request[0]) ||
        !crc_is_valid(request, length))
    {
        return false;
    }

    switch (request[1])
    {
        case MODBUS_FUNC_READ_HOLDING:
            count = read_u16_be(&request[4]);
            return (length == 8u) && (count != 0u) &&
                   (count <= MODBUS_RTU_MAX_READ_REGISTERS);
        case MODBUS_FUNC_WRITE_SINGLE:
            return length == 8u;
        case MODBUS_FUNC_WRITE_MULTIPLE:
            count = read_u16_be(&request[4]);
            return (count != 0u) && (count <= MODBUS_RTU_MAX_WRITE_REGISTERS) &&
                   (request[6] == (uint8_t)(count * 2u)) &&
                   (length == (9u + ((size_t)count * 2u)));
        default:
            return false;
    }
}

static modbus_rtu_validation_result_t validation_result(
    modbus_rtu_validation_status_t status,
    uint8_t exception_code)
{
    modbus_rtu_validation_result_t result;

    result.status = status;
    result.exception_code = exception_code;
    return result;
}

modbus_rtu_validation_result_t modbus_rtu_validate_response_detailed(
    const uint8_t *request,
    size_t request_length,
    const uint8_t *response,
    size_t response_length)
{
    uint16_t requested_count;
    size_t expected_length;

    if ((request == NULL) || (response == NULL))
    {
        return validation_result(MODBUS_RTU_VALIDATION_ARGUMENT_ERROR, 0u);
    }
    if (!request_is_valid(request, request_length))
    {
        return validation_result(MODBUS_RTU_VALIDATION_REQUEST_ERROR, 0u);
    }
    if (response_length < 5u)
    {
        return validation_result(MODBUS_RTU_VALIDATION_LENGTH_ERROR, 0u);
    }
    if (response[0] != request[0])
    {
        return validation_result(MODBUS_RTU_VALIDATION_SLAVE_ERROR, 0u);
    }

    if (response[1] == (uint8_t)(request[1] | MODBUS_EXCEPTION_MASK))
    {
        if (response_length != 5u)
        {
            return validation_result(MODBUS_RTU_VALIDATION_LENGTH_ERROR, 0u);
        }
        if (!crc_is_valid(response, response_length))
        {
            return validation_result(MODBUS_RTU_VALIDATION_CRC_ERROR, 0u);
        }
        return validation_result(MODBUS_RTU_VALIDATION_EXCEPTION, response[2]);
    }
    if (response[1] != request[1])
    {
        return validation_result(MODBUS_RTU_VALIDATION_FUNCTION_ERROR, 0u);
    }

    switch (request[1])
    {
        case MODBUS_FUNC_READ_HOLDING:
            requested_count = read_u16_be(&request[4]);
            expected_length = 5u + ((size_t)requested_count * 2u);
            if ((response_length != expected_length) ||
                (response[2] != (uint8_t)(requested_count * 2u)))
            {
                return validation_result(MODBUS_RTU_VALIDATION_LENGTH_ERROR, 0u);
            }
            break;
        case MODBUS_FUNC_WRITE_SINGLE:
        case MODBUS_FUNC_WRITE_MULTIPLE:
            if (response_length != 8u)
            {
                return validation_result(MODBUS_RTU_VALIDATION_LENGTH_ERROR, 0u);
            }
            break;
        default:
            return validation_result(MODBUS_RTU_VALIDATION_REQUEST_ERROR, 0u);
    }

    if (!crc_is_valid(response, response_length))
    {
        return validation_result(MODBUS_RTU_VALIDATION_CRC_ERROR, 0u);
    }

    switch (request[1])
    {
        case MODBUS_FUNC_READ_HOLDING:
            return validation_result(MODBUS_RTU_VALIDATION_OK, 0u);
        case MODBUS_FUNC_WRITE_SINGLE:
        case MODBUS_FUNC_WRITE_MULTIPLE:
            if ((response[2] != request[2]) || (response[3] != request[3]) ||
                (response[4] != request[4]) || (response[5] != request[5]))
            {
                return validation_result(MODBUS_RTU_VALIDATION_ECHO_ERROR, 0u);
            }
            return validation_result(MODBUS_RTU_VALIDATION_OK, 0u);
        default:
            return validation_result(MODBUS_RTU_VALIDATION_REQUEST_ERROR, 0u);
    }
}

bool modbus_rtu_validate_response(const uint8_t *request,
                                  size_t request_length,
                                  const uint8_t *response,
                                  size_t response_length)
{
    const modbus_rtu_validation_result_t result =
        modbus_rtu_validate_response_detailed(request,
                                              request_length,
                                              response,
                                              response_length);

    return (result.status == MODBUS_RTU_VALIDATION_OK) ||
           (result.status == MODBUS_RTU_VALIDATION_EXCEPTION);
}
