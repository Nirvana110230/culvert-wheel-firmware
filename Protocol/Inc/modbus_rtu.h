#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MODBUS_RTU_MAX_READ_REGISTERS 125u
#define MODBUS_RTU_MAX_WRITE_REGISTERS 123u
#define MODBUS_RTU_MAX_ADU_SIZE 256u

typedef enum
{
    MODBUS_RTU_VALIDATION_OK = 0,
    MODBUS_RTU_VALIDATION_ARGUMENT_ERROR,
    MODBUS_RTU_VALIDATION_REQUEST_ERROR,
    MODBUS_RTU_VALIDATION_LENGTH_ERROR,
    MODBUS_RTU_VALIDATION_CRC_ERROR,
    MODBUS_RTU_VALIDATION_SLAVE_ERROR,
    MODBUS_RTU_VALIDATION_FUNCTION_ERROR,
    MODBUS_RTU_VALIDATION_ECHO_ERROR,
    MODBUS_RTU_VALIDATION_EXCEPTION
} modbus_rtu_validation_status_t;

typedef struct
{
    modbus_rtu_validation_status_t status;
    uint8_t exception_code;
} modbus_rtu_validation_result_t;

uint16_t modbus_rtu_crc16(const uint8_t *data, size_t length);

bool modbus_rtu_build_read(uint8_t slave,
                           uint16_t address,
                           uint16_t count,
                           uint8_t *output,
                           size_t capacity,
                           size_t *written);

bool modbus_rtu_build_write_single(uint8_t slave,
                                   uint16_t address,
                                   uint16_t value,
                                   uint8_t *output,
                                   size_t capacity,
                                   size_t *written);

bool modbus_rtu_build_write_multiple(uint8_t slave,
                                     uint16_t address,
                                     const uint16_t *values,
                                     uint16_t count,
                                     uint8_t *output,
                                     size_t capacity,
                                     size_t *written);

/* Validates one exact response ADU against its complete request ADU. */
bool modbus_rtu_validate_response(const uint8_t *request,
                                  size_t request_length,
                                  const uint8_t *response,
                                  size_t response_length);

/* Returns a specific validation failure while preserving the legacy bool API. */
modbus_rtu_validation_result_t modbus_rtu_validate_response_detailed(
    const uint8_t *request,
    size_t request_length,
    const uint8_t *response,
    size_t response_length);

#endif
