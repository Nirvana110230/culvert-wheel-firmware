#include "mssd_driver.h"

#include <limits.h>
#include <string.h>

static uint16_t read_u16_be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8u) | data[1]);
}

static int32_t read_i32_be(const uint8_t *data)
{
    const uint32_t bits = ((uint32_t)data[0] << 24u) |
                          ((uint32_t)data[1] << 16u) |
                          ((uint32_t)data[2] << 8u) |
                          (uint32_t)data[3];

    if (bits <= (uint32_t)INT32_MAX)
    {
        return (int32_t)bits;
    }
    if (bits == 0x80000000u)
    {
        return INT32_MIN;
    }
    return -(int32_t)((~bits) + 1u);
}

static bool channel_is_single(mssd_channel_t channel)
{
    return (channel == MSSD_CHANNEL_RIGHT) || (channel == MSSD_CHANNEL_LEFT);
}

static bool driver_is_terminal(const mssd_driver_t *driver)
{
    return (driver->state == MSSD_DRIVER_COMPLETE) ||
           (driver->state == MSSD_DRIVER_PARTIAL_COMMIT) ||
           (driver->state == MSSD_DRIVER_FAILED);
}

static bool operation_is_read(mssd_operation_t operation)
{
    return (operation == MSSD_OPERATION_READ_RIGHT_STATUS) ||
           (operation == MSSD_OPERATION_READ_LEFT_STATUS) ||
           (operation == MSSD_OPERATION_READ_CONFIGURATION);
}

static bool configuration_allows_rpm(const mssd_driver_t *driver)
{
    return driver->configuration.valid &&
           (driver->configuration.system_mode == 1u) &&
           (driver->configuration.control_source == 0u);
}

static bool brake_mapping_is_valid(const mssd_brake_mapping_t *mapping)
{
    return mapping->mapping_valid &&
           (mapping->engage_raw <= 1u) &&
           (mapping->release_raw <= 1u) &&
           (mapping->engage_raw != mapping->release_raw);
}

static void clear_error(mssd_driver_t *driver)
{
    driver->error.code = MSSD_ERROR_NONE;
    driver->error.modbus_status = MODBUS_RTU_VALIDATION_OK;
    driver->error.exception_code = 0u;
}

static void start_result(mssd_driver_t *driver)
{
    driver->result.completion = MSSD_COMPLETION_NONE;
    driver->result.right = MSSD_SIDE_NOT_ATTEMPTED;
    driver->result.left = MSSD_SIDE_NOT_ATTEMPTED;
    clear_error(driver);
}

static void set_side_result(mssd_driver_t *driver,
                            mssd_channel_t channel,
                            mssd_side_result_t result)
{
    if (channel == MSSD_CHANNEL_RIGHT)
    {
        driver->result.right = result;
    }
    else if (channel == MSSD_CHANNEL_LEFT)
    {
        driver->result.left = result;
    }
}

static bool response_matches_request(const uint8_t *request,
                                     size_t request_length,
                                     const uint8_t *response,
                                     size_t response_length)
{
    modbus_rtu_validation_result_t validation;

    if ((request == NULL) || (request_length == 0u) ||
        (response == NULL) || (response_length == 0u))
    {
        return false;
    }
    validation = modbus_rtu_validate_response_detailed(request,
                                                       request_length,
                                                       response,
                                                       response_length);
    return (validation.status == MODBUS_RTU_VALIDATION_OK) ||
           (validation.status == MODBUS_RTU_VALIDATION_EXCEPTION);
}

static void remember_completed_request(mssd_driver_t *driver)
{
    (void)memcpy(driver->last_completed_request,
                 driver->request,
                 driver->request_length);
    driver->last_completed_request_length = driver->request_length;
}

static mssd_request_status_t start_request(mssd_driver_t *driver,
                                           size_t request_length,
                                           uint32_t now_ms)
{
    if (!rs485_transport_begin(driver->transport,
                               driver->request,
                               request_length,
                               now_ms))
    {
        driver->error.code = MSSD_ERROR_TRANSPORT;
        return MSSD_REQUEST_TRANSPORT_REJECTED;
    }
    driver->request_length = request_length;
    driver->read_attempts = 0u;
    driver->current_request_is_read = operation_is_read(driver->operation);
    driver->state = MSSD_DRIVER_WAITING_TRANSPORT;
    return MSSD_REQUEST_ACCEPTED;
}

static bool restart_read_request(mssd_driver_t *driver, uint32_t now_ms)
{
    if (!rs485_transport_acknowledge(driver->transport, now_ms))
    {
        return false;
    }
    if (!rs485_transport_begin(driver->transport,
                               driver->request,
                               driver->request_length,
                               now_ms))
    {
        return false;
    }
    ++driver->read_attempts;
    driver->state = MSSD_DRIVER_WAITING_TRANSPORT;
    return true;
}

static bool build_status_request(mssd_driver_t *driver,
                                 mssd_channel_t channel,
                                 size_t *request_length)
{
    const uint16_t address = (channel == MSSD_CHANNEL_RIGHT) ?
                                 MSSD_REG_RIGHT_PHASE_CURRENT :
                                 MSSD_REG_LEFT_PHASE_CURRENT;

    return modbus_rtu_build_read(driver->config.slave_address,
                                 address,
                                 4u,
                                 driver->request,
                                 sizeof(driver->request),
                                 request_length);
}

static bool build_configuration_request(mssd_driver_t *driver,
                                        size_t *request_length)
{
    return modbus_rtu_build_read(driver->config.slave_address,
                                 MSSD_REG_SYSTEM_MODE,
                                 4u,
                                 driver->request,
                                 sizeof(driver->request),
                                 request_length);
}

static bool build_target_request(mssd_driver_t *driver,
                                 mssd_channel_t channel,
                                 int32_t rpm,
                                 size_t *request_length)
{
    const uint32_t bits = (uint32_t)rpm;
    const uint16_t values[2] = {
        (uint16_t)(bits >> 16u),
        (uint16_t)bits,
    };
    const uint16_t address = (channel == MSSD_CHANNEL_RIGHT) ?
                                 MSSD_REG_RIGHT_TARGET_HIGH :
                                 MSSD_REG_LEFT_TARGET_HIGH;

    return modbus_rtu_build_write_multiple(driver->config.slave_address,
                                            address,
                                            values,
                                            2u,
                                            driver->request,
                                            sizeof(driver->request),
                                            request_length);
}

static bool build_stop_request(mssd_driver_t *driver,
                               mssd_channel_t channel,
                               size_t *request_length)
{
    const uint16_t address = (channel == MSSD_CHANNEL_RIGHT) ?
                                 MSSD_REG_RIGHT_STOP :
                                 MSSD_REG_LEFT_STOP;

    return modbus_rtu_build_write_single(driver->config.slave_address,
                                         address,
                                         (uint16_t)driver->stop_type,
                                         driver->request,
                                         sizeof(driver->request),
                                         request_length);
}

static bool build_brake_request(mssd_driver_t *driver,
                                mssd_channel_t channel,
                                size_t *request_length)
{
    const mssd_brake_mapping_t *mapping =
        (channel == MSSD_CHANNEL_RIGHT) ?
            &driver->config.right_brake : &driver->config.left_brake;
    const uint16_t address = (channel == MSSD_CHANNEL_RIGHT) ?
                                 MSSD_REG_RIGHT_BRAKE :
                                 MSSD_REG_LEFT_BRAKE;
    const uint16_t value = (driver->brake_intent == MSSD_BRAKE_ENGAGE) ?
                               mapping->engage_raw : mapping->release_raw;

    return modbus_rtu_build_write_single(driver->config.slave_address,
                                         address,
                                         value,
                                         driver->request,
                                         sizeof(driver->request),
                                         request_length);
}

static bool operation_has_left_step(const mssd_driver_t *driver)
{
    return ((driver->operation == MSSD_OPERATION_WRITE_TARGET_PAIR) ||
            (driver->operation == MSSD_OPERATION_STOP_BOTH) ||
            (driver->operation == MSSD_OPERATION_BRAKE_BOTH)) &&
           (driver->current_channel == MSSD_CHANNEL_RIGHT);
}

static bool build_left_sequence_step(mssd_driver_t *driver,
                                     size_t *request_length)
{
    driver->current_channel = MSSD_CHANNEL_LEFT;
    switch (driver->operation)
    {
        case MSSD_OPERATION_WRITE_TARGET_PAIR:
            return build_target_request(driver,
                                        MSSD_CHANNEL_LEFT,
                                        driver->left_target_rpm,
                                        request_length);
        case MSSD_OPERATION_STOP_BOTH:
            return build_stop_request(driver, MSSD_CHANNEL_LEFT, request_length);
        case MSSD_OPERATION_BRAKE_BOTH:
            return build_brake_request(driver, MSSD_CHANNEL_LEFT, request_length);
        default:
            return false;
    }
}

static bool advance_to_left_step(mssd_driver_t *driver, uint32_t now_ms)
{
    size_t request_length = 0u;

    if (driver->result.right == MSSD_SIDE_SUCCEEDED)
    {
        remember_completed_request(driver);
    }
    if (!rs485_transport_acknowledge(driver->transport, now_ms) ||
        !build_left_sequence_step(driver, &request_length))
    {
        return false;
    }
    if (!rs485_transport_begin(driver->transport,
                               driver->request,
                               request_length,
                               now_ms))
    {
        return false;
    }
    driver->request_length = request_length;
    driver->current_request_is_read = false;
    driver->state = MSSD_DRIVER_WAITING_TRANSPORT;
    return true;
}

static void parse_status_response(mssd_driver_t *driver, const uint8_t *data)
{
    mssd_motor_status_t *status = (driver->current_channel == MSSD_CHANNEL_RIGHT) ?
                                      &driver->right_status : &driver->left_status;

    status->phase_current_raw = read_u16_be(&data[3]);
    status->phase_current_milliamps =
        (uint32_t)status->phase_current_raw * 10u;
    status->rpm = read_i32_be(&data[5]);
    status->fault_code = read_u16_be(&data[9]);
    status->valid = true;
}

static void parse_configuration_response(mssd_driver_t *driver,
                                         const uint8_t *data)
{
    driver->configuration.system_mode = read_u16_be(&data[3]);
    driver->configuration.control_source = read_u16_be(&data[5]);
    driver->configuration.drive_mode = read_u16_be(&data[7]);
    driver->configuration.foc_mode = read_u16_be(&data[9]);
    driver->configuration.valid = true;
}

static void finish_success(mssd_driver_t *driver,
                           const uint8_t *data,
                           uint32_t now_ms)
{
    if ((driver->operation == MSSD_OPERATION_READ_RIGHT_STATUS) ||
        (driver->operation == MSSD_OPERATION_READ_LEFT_STATUS))
    {
        parse_status_response(driver, data);
    }
    else if (driver->operation == MSSD_OPERATION_READ_CONFIGURATION)
    {
        parse_configuration_response(driver, data);
    }

    set_side_result(driver, driver->current_channel, MSSD_SIDE_SUCCEEDED);
    if (operation_has_left_step(driver))
    {
        if (!advance_to_left_step(driver, now_ms))
        {
            driver->result.completion = MSSD_COMPLETION_PARTIAL_COMMIT;
            driver->error.code = MSSD_ERROR_PARTIAL_COMMIT;
            driver->state = MSSD_DRIVER_PARTIAL_COMMIT;
        }
        return;
    }

    remember_completed_request(driver);
    if (driver->result.right == MSSD_SIDE_FAILED)
    {
        driver->result.completion = MSSD_COMPLETION_PARTIAL_COMMIT;
        driver->error.code = MSSD_ERROR_PARTIAL_COMMIT;
        driver->state = MSSD_DRIVER_PARTIAL_COMMIT;
        return;
    }
    driver->result.completion = MSSD_COMPLETION_SUCCEEDED;
    driver->state = MSSD_DRIVER_COMPLETE;
}

static void finish_failure(mssd_driver_t *driver,
                           mssd_error_code_t error_code,
                           uint32_t now_ms)
{
    set_side_result(driver, driver->current_channel, MSSD_SIDE_FAILED);
    if (driver->operation == MSSD_OPERATION_READ_CONFIGURATION)
    {
        driver->configuration.valid = false;
    }
    else if (driver->operation == MSSD_OPERATION_READ_RIGHT_STATUS)
    {
        driver->right_status.valid = false;
    }
    else if (driver->operation == MSSD_OPERATION_READ_LEFT_STATUS)
    {
        driver->left_status.valid = false;
    }
    /* Each side can still stop or engage its brake after the other side fails. */
    if (((driver->operation == MSSD_OPERATION_STOP_BOTH) ||
         ((driver->operation == MSSD_OPERATION_BRAKE_BOTH) &&
          (driver->brake_intent == MSSD_BRAKE_ENGAGE))) &&
        (driver->current_channel == MSSD_CHANNEL_RIGHT) &&
        (error_code != MSSD_ERROR_ABORTED_FOR_SAFETY) &&
        advance_to_left_step(driver, now_ms))
    {
        return;
    }
    if ((driver->current_channel == MSSD_CHANNEL_LEFT) &&
        (driver->result.right == MSSD_SIDE_SUCCEEDED))
    {
        driver->result.completion = MSSD_COMPLETION_PARTIAL_COMMIT;
        driver->error.code = MSSD_ERROR_PARTIAL_COMMIT;
        driver->state = MSSD_DRIVER_PARTIAL_COMMIT;
    }
    else
    {
        driver->result.completion = MSSD_COMPLETION_FAILED;
        driver->error.code = error_code;
        driver->state = MSSD_DRIVER_FAILED;
    }
}

mssd_driver_config_t mssd_driver_default_config(void)
{
    mssd_driver_config_t config;

    (void)memset(&config, 0, sizeof(config));
    config.slave_address = MSSD_DEFAULT_SLAVE_ADDRESS;
    config.read_retry_limit = 0u;
    return config;
}

bool mssd_driver_init(mssd_driver_t *driver,
                      rs485_transport_t *transport,
                      const mssd_driver_config_t *config)
{
    if ((driver == NULL) || (transport == NULL) || (config == NULL) ||
        (transport->state != RS485_IDLE) ||
        (config->slave_address == 0u) || (config->slave_address > 247u) ||
        (config->read_retry_limit > MSSD_MAX_READ_RETRIES))
    {
        return false;
    }

    (void)memset(driver, 0, sizeof(*driver));
    driver->transport = transport;
    driver->config = *config;
    driver->state = MSSD_DRIVER_IDLE;
    clear_error(driver);
    return true;
}

void mssd_driver_set_motion_authorized(mssd_driver_t *driver, bool authorized)
{
    if (driver != NULL)
    {
        driver->motion_authorized = authorized;
    }
}

bool mssd_driver_get_motion_authorized(const mssd_driver_t *driver)
{
    return (driver != NULL) && driver->motion_authorized;
}

static mssd_request_status_t begin_operation(mssd_driver_t *driver,
                                             size_t request_length,
                                             uint32_t now_ms)
{
    mssd_request_status_t status;

    status = start_request(driver, request_length, now_ms);
    if (status != MSSD_REQUEST_ACCEPTED)
    {
        driver->operation = MSSD_OPERATION_NONE;
        driver->state = MSSD_DRIVER_IDLE;
    }
    return status;
}

static mssd_request_status_t prepare_operation(mssd_driver_t *driver)
{
    if (driver == NULL)
    {
        return MSSD_REQUEST_INVALID_ARGUMENT;
    }
    if ((driver->state != MSSD_DRIVER_IDLE) ||
        (driver->transport->state != RS485_IDLE))
    {
        driver->error.code = MSSD_ERROR_BUSY;
        return MSSD_REQUEST_BUSY;
    }
    start_result(driver);
    return MSSD_REQUEST_ACCEPTED;
}

mssd_request_status_t mssd_driver_request_status(mssd_driver_t *driver,
                                                 mssd_channel_t channel,
                                                 uint32_t now_ms)
{
    size_t request_length = 0u;
    mssd_request_status_t status = prepare_operation(driver);

    if (status != MSSD_REQUEST_ACCEPTED)
    {
        return status;
    }
    if (!channel_is_single(channel))
    {
        return MSSD_REQUEST_INVALID_ARGUMENT;
    }
    driver->operation = (channel == MSSD_CHANNEL_RIGHT) ?
                            MSSD_OPERATION_READ_RIGHT_STATUS :
                            MSSD_OPERATION_READ_LEFT_STATUS;
    driver->current_channel = channel;
    if (!build_status_request(driver, channel, &request_length))
    {
        driver->error.code = MSSD_ERROR_BUILD_FRAME;
        return MSSD_REQUEST_BUILD_FAILED;
    }
    return begin_operation(driver, request_length, now_ms);
}

mssd_request_status_t mssd_driver_request_configuration(mssd_driver_t *driver,
                                                        uint32_t now_ms)
{
    size_t request_length = 0u;
    mssd_request_status_t status = prepare_operation(driver);

    if (status != MSSD_REQUEST_ACCEPTED)
    {
        return status;
    }
    driver->operation = MSSD_OPERATION_READ_CONFIGURATION;
    driver->current_channel = MSSD_CHANNEL_BOTH;
    if (!build_configuration_request(driver, &request_length))
    {
        driver->error.code = MSSD_ERROR_BUILD_FRAME;
        return MSSD_REQUEST_BUILD_FAILED;
    }
    return begin_operation(driver, request_length, now_ms);
}

mssd_request_status_t mssd_driver_request_initialize_configuration(mssd_driver_t *driver,
                                                                   uint32_t now_ms)
{
    /* 0x007B..0x007E: independent speed, communication, FOC, speed/current. */
    static const uint16_t values[] = {1u, 0u, 1u, 1u};
    size_t request_length = 0u;
    mssd_request_status_t status = prepare_operation(driver);

    if (status != MSSD_REQUEST_ACCEPTED)
    {
        return status;
    }
    if (!modbus_rtu_build_write_multiple(driver->config.slave_address,
                                         MSSD_REG_SYSTEM_MODE,
                                         values,
                                         sizeof(values) / sizeof(values[0]),
                                         driver->request,
                                         sizeof(driver->request),
                                         &request_length))
    {
        driver->error.code = MSSD_ERROR_BUILD_FRAME;
        return MSSD_REQUEST_BUILD_FAILED;
    }
    driver->configuration.valid = false;
    driver->motion_authorized = false;
    driver->operation = MSSD_OPERATION_INITIALIZE_CONFIGURATION;
    driver->current_channel = MSSD_CHANNEL_BOTH;
    return begin_operation(driver, request_length, now_ms);
}

static mssd_request_status_t validate_motion_request(mssd_driver_t *driver)
{
    mssd_request_status_t status = prepare_operation(driver);

    if (status != MSSD_REQUEST_ACCEPTED)
    {
        return status;
    }
    if (!configuration_allows_rpm(driver))
    {
        driver->error.code = MSSD_ERROR_CONFIG_UNVERIFIED;
        return MSSD_REQUEST_CONFIG_UNVERIFIED;
    }
    if (!driver->motion_authorized)
    {
        driver->error.code = MSSD_ERROR_MOTION_NOT_AUTHORIZED;
        return MSSD_REQUEST_MOTION_NOT_AUTHORIZED;
    }
    return MSSD_REQUEST_ACCEPTED;
}

mssd_request_status_t mssd_driver_request_target_rpm(mssd_driver_t *driver,
                                                     mssd_channel_t channel,
                                                     int32_t rpm,
                                                     uint32_t now_ms)
{
    size_t request_length = 0u;
    mssd_request_status_t status;

    if ((driver == NULL) || !channel_is_single(channel))
    {
        return MSSD_REQUEST_INVALID_ARGUMENT;
    }
    status = validate_motion_request(driver);
    if (status != MSSD_REQUEST_ACCEPTED)
    {
        return status;
    }
    driver->operation = MSSD_OPERATION_WRITE_TARGET;
    driver->current_channel = channel;
    if (!build_target_request(driver, channel, rpm, &request_length))
    {
        driver->error.code = MSSD_ERROR_BUILD_FRAME;
        return MSSD_REQUEST_BUILD_FAILED;
    }
    return begin_operation(driver, request_length, now_ms);
}

mssd_request_status_t mssd_driver_request_target_pair(mssd_driver_t *driver,
                                                      int32_t right_rpm,
                                                      int32_t left_rpm,
                                                      uint32_t now_ms)
{
    size_t request_length = 0u;
    mssd_request_status_t status;

    if (driver == NULL)
    {
        return MSSD_REQUEST_INVALID_ARGUMENT;
    }
    status = validate_motion_request(driver);
    if (status != MSSD_REQUEST_ACCEPTED)
    {
        return status;
    }
    driver->operation = MSSD_OPERATION_WRITE_TARGET_PAIR;
    driver->current_channel = MSSD_CHANNEL_RIGHT;
    driver->right_target_rpm = right_rpm;
    driver->left_target_rpm = left_rpm;
    if (!build_target_request(driver,
                              MSSD_CHANNEL_RIGHT,
                              right_rpm,
                              &request_length))
    {
        driver->error.code = MSSD_ERROR_BUILD_FRAME;
        return MSSD_REQUEST_BUILD_FAILED;
    }
    return begin_operation(driver, request_length, now_ms);
}

mssd_request_status_t mssd_driver_request_stop(mssd_driver_t *driver,
                                               mssd_channel_t channel,
                                               mssd_stop_type_t stop_type,
                                               uint32_t now_ms)
{
    size_t request_length = 0u;
    mssd_request_status_t status = prepare_operation(driver);

    if (status != MSSD_REQUEST_ACCEPTED)
    {
        return status;
    }
    if ((channel != MSSD_CHANNEL_RIGHT) &&
        (channel != MSSD_CHANNEL_LEFT) &&
        (channel != MSSD_CHANNEL_BOTH))
    {
        return MSSD_REQUEST_INVALID_ARGUMENT;
    }
    if ((stop_type != MSSD_STOP_NORMAL) &&
        (stop_type != MSSD_STOP_EMERGENCY) &&
        (stop_type != MSSD_STOP_FREE))
    {
        return MSSD_REQUEST_INVALID_ARGUMENT;
    }
    driver->operation = (channel == MSSD_CHANNEL_BOTH) ?
                            MSSD_OPERATION_STOP_BOTH : MSSD_OPERATION_STOP;
    driver->current_channel = (channel == MSSD_CHANNEL_LEFT) ?
                                  MSSD_CHANNEL_LEFT : MSSD_CHANNEL_RIGHT;
    driver->stop_type = stop_type;
    if (!build_stop_request(driver, driver->current_channel, &request_length))
    {
        driver->error.code = MSSD_ERROR_BUILD_FRAME;
        return MSSD_REQUEST_BUILD_FAILED;
    }
    return begin_operation(driver, request_length, now_ms);
}

mssd_request_status_t mssd_driver_request_brake(mssd_driver_t *driver,
                                                mssd_channel_t channel,
                                                mssd_brake_intent_t intent,
                                                uint32_t now_ms)
{
    size_t request_length = 0u;
    mssd_request_status_t status = prepare_operation(driver);

    if (status != MSSD_REQUEST_ACCEPTED)
    {
        return status;
    }
    if (((channel != MSSD_CHANNEL_RIGHT) &&
         (channel != MSSD_CHANNEL_LEFT) &&
         (channel != MSSD_CHANNEL_BOTH)) ||
        ((intent != MSSD_BRAKE_ENGAGE) &&
         (intent != MSSD_BRAKE_RELEASE)))
    {
        return MSSD_REQUEST_INVALID_ARGUMENT;
    }
    if (((channel == MSSD_CHANNEL_RIGHT) || (channel == MSSD_CHANNEL_BOTH)) &&
        !brake_mapping_is_valid(&driver->config.right_brake))
    {
        driver->error.code = MSSD_ERROR_BRAKE_MAPPING_UNVERIFIED;
        return MSSD_REQUEST_BRAKE_MAPPING_UNVERIFIED;
    }
    if (((channel == MSSD_CHANNEL_LEFT) || (channel == MSSD_CHANNEL_BOTH)) &&
        !brake_mapping_is_valid(&driver->config.left_brake))
    {
        driver->error.code = MSSD_ERROR_BRAKE_MAPPING_UNVERIFIED;
        return MSSD_REQUEST_BRAKE_MAPPING_UNVERIFIED;
    }

    driver->operation = (channel == MSSD_CHANNEL_BOTH) ?
                            MSSD_OPERATION_BRAKE_BOTH : MSSD_OPERATION_BRAKE;
    driver->current_channel = (channel == MSSD_CHANNEL_LEFT) ?
                                  MSSD_CHANNEL_LEFT : MSSD_CHANNEL_RIGHT;
    driver->brake_intent = intent;
    if (!build_brake_request(driver, driver->current_channel, &request_length))
    {
        driver->error.code = MSSD_ERROR_BUILD_FRAME;
        return MSSD_REQUEST_BUILD_FAILED;
    }
    return begin_operation(driver, request_length, now_ms);
}

void mssd_driver_poll(mssd_driver_t *driver, uint32_t now_ms)
{
    if ((driver == NULL) ||
        ((driver->state != MSSD_DRIVER_WAITING_TRANSPORT) &&
         (driver->state != MSSD_DRIVER_WAITING_RESPONSE)))
    {
        return;
    }

    rs485_transport_poll(driver->transport, now_ms);
    if (driver->transport->state == RS485_WAIT_RESPONSE)
    {
        driver->state = MSSD_DRIVER_WAITING_RESPONSE;
    }
    else if ((driver->transport->state == RS485_TIMEOUT) ||
             (driver->transport->state == RS485_ERROR))
    {
        if ((driver->transport->state == RS485_TIMEOUT) &&
            driver->current_request_is_read &&
            (driver->read_attempts < driver->config.read_retry_limit) &&
            restart_read_request(driver, now_ms))
        {
            return;
        }
        finish_failure(driver,
                       (driver->transport->state == RS485_TIMEOUT) ?
                           MSSD_ERROR_TIMEOUT : MSSD_ERROR_TRANSPORT, now_ms);
    }
}

bool mssd_driver_abort_for_safety(mssd_driver_t *driver, uint32_t now_ms)
{
    if ((driver == NULL) ||
        ((driver->state != MSSD_DRIVER_WAITING_TRANSPORT) &&
         (driver->state != MSSD_DRIVER_WAITING_RESPONSE)))
    {
        return false;
    }
    if (!rs485_transport_abort(driver->transport, now_ms))
    {
        return false;
    }
    finish_failure(driver, MSSD_ERROR_ABORTED_FOR_SAFETY, now_ms);
    return true;
}

mssd_response_status_t mssd_driver_on_response(mssd_driver_t *driver,
                                               const uint8_t *data,
                                               size_t length,
                                               uint32_t now_ms)
{
    modbus_rtu_validation_result_t validation;

    if ((driver == NULL) || (data == NULL) || (length == 0u))
    {
        return MSSD_RESPONSE_INVALID_FRAME;
    }

    if ((driver->state == MSSD_DRIVER_IDLE) || driver_is_terminal(driver))
    {
        if (response_matches_request(driver->last_completed_request,
                                     driver->last_completed_request_length,
                                     data,
                                     length))
        {
            return MSSD_RESPONSE_DUPLICATE_FRAME;
        }
        if (response_matches_request(driver->request,
                                     driver->request_length,
                                     data,
                                     length))
        {
            return (driver->result.completion == MSSD_COMPLETION_SUCCEEDED) ?
                       MSSD_RESPONSE_DUPLICATE_FRAME : MSSD_RESPONSE_LATE_FRAME;
        }
        return (driver->state == MSSD_DRIVER_IDLE) ?
                   MSSD_RESPONSE_IDLE_FRAME : MSSD_RESPONSE_LATE_FRAME;
    }

    if ((driver->state != MSSD_DRIVER_WAITING_RESPONSE) ||
        (driver->transport->state != RS485_WAIT_RESPONSE))
    {
        if (response_matches_request(driver->last_completed_request,
                                     driver->last_completed_request_length,
                                     data,
                                     length))
        {
            return MSSD_RESPONSE_DUPLICATE_FRAME;
        }
        return MSSD_RESPONSE_UNEXPECTED_FRAME;
    }

    validation = modbus_rtu_validate_response_detailed(driver->request,
                                                       driver->request_length,
                                                       data,
                                                       length);
    if ((validation.status != MODBUS_RTU_VALIDATION_OK) &&
        (validation.status != MODBUS_RTU_VALIDATION_EXCEPTION))
    {
        driver->error.code = MSSD_ERROR_INVALID_RESPONSE;
        driver->error.modbus_status = validation.status;
        driver->error.exception_code = 0u;
        return MSSD_RESPONSE_INVALID_FRAME;
    }

    if (!rs485_transport_response_complete(driver->transport, now_ms))
    {
        finish_failure(driver, MSSD_ERROR_TRANSPORT, now_ms);
        return MSSD_RESPONSE_UNEXPECTED_FRAME;
    }

    if (validation.status == MODBUS_RTU_VALIDATION_EXCEPTION)
    {
        driver->error.modbus_status = validation.status;
        driver->error.exception_code = validation.exception_code;
        finish_failure(driver, MSSD_ERROR_MODBUS_EXCEPTION, now_ms);
        return MSSD_RESPONSE_ACCEPTED;
    }

    clear_error(driver);
    finish_success(driver, data, now_ms);
    return MSSD_RESPONSE_ACCEPTED;
}

bool mssd_driver_acknowledge(mssd_driver_t *driver, uint32_t now_ms)
{
    if ((driver == NULL) || !driver_is_terminal(driver) ||
        !rs485_transport_acknowledge(driver->transport, now_ms))
    {
        return false;
    }
    driver->state = MSSD_DRIVER_IDLE;
    driver->operation = MSSD_OPERATION_NONE;
    return true;
}

mssd_driver_state_t mssd_driver_get_state(const mssd_driver_t *driver)
{
    return (driver != NULL) ? driver->state : MSSD_DRIVER_UNINITIALIZED;
}

mssd_transaction_result_t mssd_driver_get_result(const mssd_driver_t *driver)
{
    const mssd_transaction_result_t empty = {
        MSSD_COMPLETION_NONE,
        MSSD_SIDE_NOT_ATTEMPTED,
        MSSD_SIDE_NOT_ATTEMPTED,
    };

    return (driver != NULL) ? driver->result : empty;
}

mssd_error_detail_t mssd_driver_get_error(const mssd_driver_t *driver)
{
    const mssd_error_detail_t invalid = {
        MSSD_ERROR_ARGUMENT,
        MODBUS_RTU_VALIDATION_ARGUMENT_ERROR,
        0u,
    };

    return (driver != NULL) ? driver->error : invalid;
}

bool mssd_driver_get_status(const mssd_driver_t *driver,
                            mssd_channel_t channel,
                            mssd_motor_status_t *status)
{
    if ((driver == NULL) || (status == NULL) || !channel_is_single(channel))
    {
        return false;
    }
    *status = (channel == MSSD_CHANNEL_RIGHT) ?
                  driver->right_status : driver->left_status;
    return status->valid;
}

bool mssd_driver_get_configuration(const mssd_driver_t *driver,
                                   mssd_configuration_snapshot_t *configuration)
{
    if ((driver == NULL) || (configuration == NULL))
    {
        return false;
    }
    *configuration = driver->configuration;
    return configuration->valid;
}
