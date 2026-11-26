#ifndef CHASSIS_APP_H
#define CHASSIS_APP_H

#include "app_state_machine.h"
#include "board_status_outputs.h"
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
    CHASSIS_APP_RUNNING,
    CHASSIS_APP_STARTUP_WRITE_CONFIGURATION
} chassis_app_startup_state_t;

/* States describe acknowledged bus commands; there is no brake position sensor. */
typedef enum
{
    WHEEL_BRAKE_UNKNOWN = 0,
    WHEEL_STOP_REQUIRED,
    WHEEL_ENGAGE_REQUIRED,
    WHEEL_PARKED,
    WHEEL_RELEASE_PENDING,
    WHEEL_RELEASED
} chassis_wheel_brake_state_t;

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
    uint32_t jetson_self_check_period_ms;
    uint32_t emergency_beep_half_period_ms;
    bool require_jetson_self_check;
    chassis_operating_profile_t profile;
    bool beep_active_high;
} chassis_app_config_t;

typedef struct
{
    bool self_check_pending;
    bool inspect_start_pending;
    bool inspect_stop_pending;
    bool emergency_pending;
    uint8_t emergency_value;
    bool mode_pending;
    uint8_t mode_value;
} chassis_jetson_outbox_t;

typedef enum
{
    CHASSIS_JETSON_TX_NONE = 0,
    CHASSIS_JETSON_TX_SELF_CHECK,
    CHASSIS_JETSON_TX_INSPECT_START,
    CHASSIS_JETSON_TX_INSPECT_STOP,
    CHASSIS_JETSON_TX_EMERGENCY,
    CHASSIS_JETSON_TX_MODE
} chassis_jetson_tx_kind_t;

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
    uint32_t rc_capture_drops;
    motion_stop_reason_t stop_reason;
    bool motor_target_reached_available;
    bool brake_feedback_available;
    bool jetson_self_check_received;
    bool jetson_self_check_passed;
    uint8_t jetson_self_check_error;
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
    board_status_outputs_t status_outputs;
    chassis_app_config_t config;
    chassis_app_diagnostics_t diagnostics;
    chassis_app_startup_state_t startup_state;
    uint32_t inspect_request_started_ms;
    uint32_t inspect_request_sent_ms;
    uint32_t inspect_request_generation;
    uint32_t inspect_tx_generation;
    uint32_t inspection_accepted_ms;
    uint32_t next_status_poll_ms;
    bool read_backoff_active;
    uint32_t next_target_refresh_ms;
    uint32_t next_self_check_request_ms;
    chassis_jetson_outbox_t jetson_outbox;
    chassis_jetson_tx_kind_t jetson_tx_kind;
    uint32_t jetson_tx_completion_snapshot;
    uint32_t jetson_receive_losses;
    uint8_t jetson_tx_payload;
    bool inspect_request_active;
    bool inspect_request_sent;
    bool safe_stop_pending;
    bool safe_stop_delivered;
    bool safe_stop_attempted;
    bool stop_failure_latched;
    bool recovery_stop_attempted;
    bool self_check_completed;
    bool jetson_self_check_received;
    bool jetson_self_check_passed;
    bool jetson_self_check_request_outstanding;
    bool jetson_tx_inflight;
    uint8_t jetson_self_check_error;
    mssd_stop_type_t pending_stop_type;
    mssd_stop_type_t inflight_stop_type;
    motion_stop_reason_t pending_stop_reason;
    motion_stop_reason_t fault_reason;
    bool started;
    chassis_wheel_brake_state_t wheel_brake_state;
    uint32_t right_feedback_ms;
    uint32_t left_feedback_ms;
    uint32_t wheel_retry_ms;
    uint32_t wheel_adapter_errors;
    uint32_t next_rc_command_ms;
    bool wheel_retry_delayed;
    bool wheel_recheck_pending;
    bool wheel_configuration_retry_requested;
} chassis_app_t;

/* Returns the offline-safe board configuration. All non-zero motion gates are off. */
chassis_app_config_t chassis_app_default_config(void);

/* Independent HT-10A wheel chassis using the original Keil motor parameters. */
chassis_app_config_t chassis_app_wheel_rc_config(void);

/* Initializes software objects only; hardware receive paths remain stopped. */
bool chassis_app_init(chassis_app_t *app,
                      const chassis_app_config_t *config,
                      UART_HandleTypeDef *mssd_uart,
                      DMA_HandleTypeDef *rx_dma,
                      DMA_HandleTypeDef *tx_dma,
                      GPIO_TypeDef *de_port,
                      uint16_t de_pin);

/* Attaches the evidence-backed PB8 emergency buzzer output. */
bool chassis_app_attach_status_outputs(chassis_app_t *app,
                                       GPIO_TypeDef *beep_port,
                                       uint16_t beep_pin);

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
