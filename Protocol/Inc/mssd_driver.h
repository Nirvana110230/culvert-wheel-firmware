#ifndef MSSD_DRIVER_H
#define MSSD_DRIVER_H

#include "modbus_rtu.h"
#include "rs485_transport.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MSSD_DEFAULT_SLAVE_ADDRESS 1u
#define MSSD_MAX_READ_RETRIES 2u
#define MSSD_MAX_REQUEST_SIZE 13u

#define MSSD_REG_RIGHT_PHASE_CURRENT 0x0008u
#define MSSD_REG_RIGHT_SPEED_HIGH 0x0009u
#define MSSD_REG_RIGHT_SPEED_LOW 0x000Au
#define MSSD_REG_RIGHT_FAULT 0x000Bu
#define MSSD_REG_LEFT_PHASE_CURRENT 0x0013u
#define MSSD_REG_LEFT_SPEED_HIGH 0x0014u
#define MSSD_REG_LEFT_SPEED_LOW 0x0015u
#define MSSD_REG_LEFT_FAULT 0x0016u
#define MSSD_REG_RIGHT_STOP 0x0038u
#define MSSD_REG_RIGHT_TARGET_HIGH 0x0039u
#define MSSD_REG_RIGHT_TARGET_LOW 0x003Au
#define MSSD_REG_RIGHT_BRAKE 0x003Bu
#define MSSD_REG_LEFT_STOP 0x003Cu
#define MSSD_REG_LEFT_TARGET_HIGH 0x003Du
#define MSSD_REG_LEFT_TARGET_LOW 0x003Eu
#define MSSD_REG_LEFT_BRAKE 0x003Fu
#define MSSD_REG_SYSTEM_MODE 0x007Bu
#define MSSD_REG_CONTROL_SOURCE 0x007Cu
#define MSSD_REG_DRIVE_MODE 0x007Du
#define MSSD_REG_FOC_MODE 0x007Eu

typedef enum
{
    MSSD_CHANNEL_RIGHT = 0,
    MSSD_CHANNEL_LEFT,
    MSSD_CHANNEL_BOTH
} mssd_channel_t;

typedef enum
{
    MSSD_STOP_NORMAL = 0,
    MSSD_STOP_EMERGENCY = 1,
    MSSD_STOP_FREE = 2
} mssd_stop_type_t;

typedef enum
{
    MSSD_BRAKE_ENGAGE = 0,
    MSSD_BRAKE_RELEASE
} mssd_brake_intent_t;

typedef enum
{
    MSSD_DRIVER_UNINITIALIZED = 0,
    MSSD_DRIVER_IDLE,
    MSSD_DRIVER_WAITING_TRANSPORT,
    MSSD_DRIVER_WAITING_RESPONSE,
    MSSD_DRIVER_COMPLETE,
    MSSD_DRIVER_PARTIAL_COMMIT,
    MSSD_DRIVER_FAILED
} mssd_driver_state_t;

typedef enum
{
    MSSD_REQUEST_ACCEPTED = 0,
    MSSD_REQUEST_INVALID_ARGUMENT,
    MSSD_REQUEST_BUSY,
    MSSD_REQUEST_CONFIG_UNVERIFIED,
    MSSD_REQUEST_MOTION_NOT_AUTHORIZED,
    MSSD_REQUEST_BRAKE_MAPPING_UNVERIFIED,
    MSSD_REQUEST_BUILD_FAILED,
    MSSD_REQUEST_TRANSPORT_REJECTED
} mssd_request_status_t;

typedef enum
{
    MSSD_RESPONSE_ACCEPTED = 0,
    MSSD_RESPONSE_IDLE_FRAME,
    MSSD_RESPONSE_LATE_FRAME,
    MSSD_RESPONSE_DUPLICATE_FRAME,
    MSSD_RESPONSE_UNEXPECTED_FRAME,
    MSSD_RESPONSE_INVALID_FRAME
} mssd_response_status_t;

typedef enum
{
    MSSD_COMPLETION_NONE = 0,
    MSSD_COMPLETION_SUCCEEDED,
    MSSD_COMPLETION_FAILED,
    MSSD_COMPLETION_PARTIAL_COMMIT
} mssd_completion_t;

typedef enum
{
    MSSD_SIDE_NOT_ATTEMPTED = 0,
    MSSD_SIDE_SUCCEEDED,
    MSSD_SIDE_FAILED
} mssd_side_result_t;

typedef enum
{
    MSSD_ERROR_NONE = 0,
    MSSD_ERROR_ARGUMENT,
    MSSD_ERROR_BUSY,
    MSSD_ERROR_CONFIG_UNVERIFIED,
    MSSD_ERROR_MOTION_NOT_AUTHORIZED,
    MSSD_ERROR_BRAKE_MAPPING_UNVERIFIED,
    MSSD_ERROR_BUILD_FRAME,
    MSSD_ERROR_TRANSPORT,
    MSSD_ERROR_TIMEOUT,
    MSSD_ERROR_ABORTED_FOR_SAFETY,
    MSSD_ERROR_INVALID_RESPONSE,
    MSSD_ERROR_MODBUS_EXCEPTION,
    MSSD_ERROR_PARTIAL_COMMIT
} mssd_error_code_t;

typedef struct
{
    uint8_t engage_raw;
    uint8_t release_raw;
    bool mapping_valid;
} mssd_brake_mapping_t;

typedef struct
{
    uint8_t slave_address;
    uint8_t read_retry_limit;
    mssd_brake_mapping_t right_brake;
    mssd_brake_mapping_t left_brake;
} mssd_driver_config_t;

typedef struct
{
    uint16_t phase_current_raw;
    uint32_t phase_current_milliamps;
    int32_t rpm;
    uint16_t fault_code;
    bool valid;
} mssd_motor_status_t;

typedef struct
{
    uint16_t system_mode;
    uint16_t control_source;
    uint16_t drive_mode;
    uint16_t foc_mode;
    bool valid;
} mssd_configuration_snapshot_t;

typedef struct
{
    mssd_completion_t completion;
    mssd_side_result_t right;
    mssd_side_result_t left;
} mssd_transaction_result_t;

typedef struct
{
    mssd_error_code_t code;
    modbus_rtu_validation_status_t modbus_status;
    uint8_t exception_code;
} mssd_error_detail_t;

typedef enum
{
    MSSD_OPERATION_NONE = 0,
    MSSD_OPERATION_READ_RIGHT_STATUS,
    MSSD_OPERATION_READ_LEFT_STATUS,
    MSSD_OPERATION_READ_CONFIGURATION,
    MSSD_OPERATION_WRITE_TARGET,
    MSSD_OPERATION_WRITE_TARGET_PAIR,
    MSSD_OPERATION_STOP,
    MSSD_OPERATION_STOP_BOTH,
    MSSD_OPERATION_BRAKE,
    MSSD_OPERATION_BRAKE_BOTH
} mssd_operation_t;

typedef struct
{
    mssd_driver_state_t state;
    rs485_transport_t *transport;
    mssd_driver_config_t config;
    bool motion_authorized;
    mssd_motor_status_t right_status;
    mssd_motor_status_t left_status;
    mssd_configuration_snapshot_t configuration;
    mssd_transaction_result_t result;
    mssd_error_detail_t error;
    mssd_operation_t operation;
    mssd_channel_t current_channel;
    mssd_stop_type_t stop_type;
    mssd_brake_intent_t brake_intent;
    int32_t right_target_rpm;
    int32_t left_target_rpm;
    uint8_t read_attempts;
    bool current_request_is_read;
    uint8_t request[MSSD_MAX_REQUEST_SIZE];
    size_t request_length;
    uint8_t last_completed_request[MSSD_MAX_REQUEST_SIZE];
    size_t last_completed_request_length;
} mssd_driver_t;

mssd_driver_config_t mssd_driver_default_config(void);

/* Initializes software state only. It never starts a bus transaction. */
bool mssd_driver_init(mssd_driver_t *driver,
                      rs485_transport_t *transport,
                      const mssd_driver_config_t *config);

void mssd_driver_set_motion_authorized(mssd_driver_t *driver, bool authorized);
bool mssd_driver_get_motion_authorized(const mssd_driver_t *driver);

mssd_request_status_t mssd_driver_request_status(mssd_driver_t *driver,
                                                 mssd_channel_t channel,
                                                 uint32_t now_ms);
mssd_request_status_t mssd_driver_request_configuration(mssd_driver_t *driver,
                                                        uint32_t now_ms);
mssd_request_status_t mssd_driver_request_target_rpm(mssd_driver_t *driver,
                                                     mssd_channel_t channel,
                                                     int32_t rpm,
                                                     uint32_t now_ms);
mssd_request_status_t mssd_driver_request_target_pair(mssd_driver_t *driver,
                                                      int32_t right_rpm,
                                                      int32_t left_rpm,
                                                      uint32_t now_ms);
mssd_request_status_t mssd_driver_request_stop(mssd_driver_t *driver,
                                               mssd_channel_t channel,
                                               mssd_stop_type_t stop_type,
                                               uint32_t now_ms);
mssd_request_status_t mssd_driver_request_brake(mssd_driver_t *driver,
                                                mssd_channel_t channel,
                                                mssd_brake_intent_t intent,
                                                uint32_t now_ms);

void mssd_driver_poll(mssd_driver_t *driver, uint32_t now_ms);

/* Cancels only an active transaction and preserves any partial-commit evidence. */
bool mssd_driver_abort_for_safety(mssd_driver_t *driver, uint32_t now_ms);

/* Consumes response bytes synchronously and never retains the caller's pointer. */
mssd_response_status_t mssd_driver_on_response(mssd_driver_t *driver,
                                               const uint8_t *data,
                                               size_t length,
                                               uint32_t now_ms);

bool mssd_driver_acknowledge(mssd_driver_t *driver, uint32_t now_ms);
mssd_driver_state_t mssd_driver_get_state(const mssd_driver_t *driver);
mssd_transaction_result_t mssd_driver_get_result(const mssd_driver_t *driver);
mssd_error_detail_t mssd_driver_get_error(const mssd_driver_t *driver);
bool mssd_driver_get_status(const mssd_driver_t *driver,
                            mssd_channel_t channel,
                            mssd_motor_status_t *status);
bool mssd_driver_get_configuration(const mssd_driver_t *driver,
                                   mssd_configuration_snapshot_t *configuration);

#endif
