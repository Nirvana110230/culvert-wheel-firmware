#ifndef CHASSIS_APP_H
#define CHASSIS_APP_H

#include "app_state_machine.h"
#include "chassis_kinematics.h"
#include "jetson_protocol.h"
#include "jetson_uart_transport.h"
#include "motion_supervisor.h"
#include "mssd_driver.h"
#include "rc_input.h"
#include "rs485_transport.h"
#include "usart2_rs485_adapter.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CHASSIS_APP_STARTUP_CONFIGURATION = 0,
    CHASSIS_APP_STARTUP_RIGHT_STATUS,
    CHASSIS_APP_STARTUP_LEFT_STATUS,
    CHASSIS_APP_RUNNING
} chassis_app_startup_state_t;

typedef struct
{
    chassis_kinematics_config_t kinematics;
    motion_supervisor_config_t supervisor;
    mssd_driver_config_t mssd;
    rs485_timeouts_t rs485_timeouts;
    bool rc_mapping_verified;
    double rc_max_linear_mm_s;
    double rc_max_angular_mrad_s;
    uint32_t status_poll_period_ms;
    uint32_t target_refresh_period_ms;
    uint32_t inspect_request_timeout_ms;
} chassis_app_config_t;

typedef struct
{
    app_state_t app_state;
    motion_control_source_t control_source;
    bool rc_command_fresh;
    bool jetson_command_fresh;
    bool mssd_configuration_read;
    bool mssd_configuration_matches;
    bool motion_authorized;
    uint32_t motion_inhibit_reasons;
    int32_t left_target_rpm;
    int32_t right_target_rpm;
    int32_t left_feedback_rpm;
    int32_t right_feedback_rpm;
    uint16_t left_fault_code;
    uint16_t right_fault_code;
    mssd_completion_t last_mssd_completion;
    mssd_error_detail_t last_mssd_error;
    uint32_t partial_commit_count;
    uint32_t communication_timeouts;
    uint32_t crc_errors;
    uint32_t uart_errors;
    uint32_t dma_errors;
    uint32_t rx_overflows;
    motion_stop_reason_t stop_reason;
    bool motor_target_reached_available;
    bool brake_feedback_available;
} chassis_app_diagnostics_t;

typedef struct
{
    app_state_machine_t machine;
    rc_input_t rc_input;
    jetson_protocol_t protocol;
    jetson_uart_transport_t jetson_transport;
    motion_supervisor_t supervisor;
    rs485_transport_t transport;
    mssd_driver_t mssd;
    usart2_rs485_adapter_t usart2_adapter;
    chassis_app_config_t config;
    chassis_app_diagnostics_t diagnostics;
    chassis_app_startup_state_t startup_state;
    uint32_t inspect_request_started_ms;
    uint32_t next_status_poll_ms;
    uint32_t next_target_refresh_ms;
    uint8_t pending_jetson_function;
    bool inspect_request_active;
    bool safe_stop_pending;
    bool safe_stop_delivered;
    bool safe_stop_attempted;
    bool self_check_completed;
    mssd_stop_type_t pending_stop_type;
    motion_stop_reason_t pending_stop_reason;
    bool started;
} chassis_app_t;

/* Returns the offline-safe board configuration. All non-zero motion gates are off. */
chassis_app_config_t chassis_app_default_config(void);

/* Initializes software objects only; hardware receive paths remain stopped. */
bool chassis_app_init(chassis_app_t *app,
                      const chassis_app_config_t *config,
                      UART_HandleTypeDef *mssd_uart,
                      DMA_HandleTypeDef *rx_dma,
                      DMA_HandleTypeDef *tx_dma,
                      GPIO_TypeDef *de_port,
                      uint16_t de_pin);

/* Starts receive-only communication and RC capture. It sends no MSSD command. */
bool chassis_app_start(chassis_app_t *app);

/* Stops receive/capture paths and releases their callback dispatchers. */
bool chassis_app_stop(chassis_app_t *app);

/* Advances all foreground state machines without blocking. */
void chassis_app_poll(chassis_app_t *app, uint32_t now_ms);

/* Copies a stable foreground diagnostic snapshot. */
bool chassis_app_get_diagnostics(const chassis_app_t *app,
                                 chassis_app_diagnostics_t *diagnostics);

#endif
