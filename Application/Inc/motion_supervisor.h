#ifndef MOTION_SUPERVISOR_H
#define MOTION_SUPERVISOR_H

#include "app_state_machine.h"
#include "mssd_driver.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MOTION_CONTROL_NONE = 0,
    MOTION_CONTROL_RC,
    MOTION_CONTROL_JETSON
} motion_control_source_t;

typedef enum
{
    MOTION_STOP_NONE = 0,
    MOTION_STOP_STANDBY,
    MOTION_STOP_EMERGENCY,
    MOTION_STOP_RC_SIGNAL_LOST,
    MOTION_STOP_JETSON_TIMEOUT,
    MOTION_STOP_CONFIGURATION,
    MOTION_STOP_KINEMATICS,
    MOTION_STOP_MOTOR_FAULT,
    MOTION_STOP_COMMUNICATION,
    MOTION_STOP_PARTIAL_COMMIT,
    MOTION_STOP_APPLICATION_LOCK
} motion_stop_reason_t;

typedef enum
{
    MOTION_INHIBIT_NONE = 0,
    MOTION_INHIBIT_APPLICATION_LOCK = (1u << 0),
    MOTION_INHIBIT_APP_STATE = (1u << 1),
    MOTION_INHIBIT_RC_STALE = (1u << 2),
    MOTION_INHIBIT_JETSON_STALE = (1u << 3),
    MOTION_INHIBIT_MSSD_CONFIGURATION = (1u << 4),
    MOTION_INHIBIT_KINEMATICS = (1u << 5),
    MOTION_INHIBIT_MOTOR_FEEDBACK = (1u << 6),
    MOTION_INHIBIT_MOTOR_FAULT = (1u << 7),
    MOTION_INHIBIT_COMMUNICATION = (1u << 8),
    MOTION_INHIBIT_PARTIAL_COMMIT = (1u << 9)
} motion_inhibit_reason_t;

typedef struct
{
    bool motion_enable;
    uint32_t rc_timeout_ms;
    uint32_t jetson_timeout_ms;
} motion_supervisor_config_t;

typedef struct
{
    double linear_mm_s;
    double angular_mrad_s;
    uint32_t timestamp_ms;
    bool valid;
} motion_command_t;

typedef struct
{
    motion_control_source_t control_source;
    double linear_mm_s;
    double angular_mrad_s;
    bool rc_fresh;
    bool jetson_fresh;
    bool motion_authorized;
    bool stop_requested;
    mssd_stop_type_t stop_type;
    motion_stop_reason_t stop_reason;
    uint32_t inhibit_reasons;
} motion_decision_t;

typedef struct
{
    motion_supervisor_config_t config;
    app_state_t app_state;
    motion_command_t rc_command;
    motion_command_t jetson_command;
    bool mssd_configuration_read;
    bool mssd_configuration_matches;
    bool kinematics_ready;
    bool right_feedback_valid;
    bool left_feedback_valid;
    uint16_t right_fault_code;
    uint16_t left_fault_code;
    bool communication_fault;
    bool partial_commit_latched;
    uint32_t partial_commit_count;
} motion_supervisor_t;

motion_supervisor_config_t motion_supervisor_default_config(void);
bool motion_supervisor_init(motion_supervisor_t *supervisor,
                            const motion_supervisor_config_t *config);
void motion_supervisor_set_app_state(motion_supervisor_t *supervisor,
                                     app_state_t state,
                                     uint32_t actions);
void motion_supervisor_submit_rc(motion_supervisor_t *supervisor,
                                 double linear_mm_s,
                                 double angular_mrad_s,
                                 uint32_t timestamp_ms);
void motion_supervisor_submit_jetson(motion_supervisor_t *supervisor,
                                     double linear_mm_s,
                                     double angular_mrad_s,
                                     uint32_t timestamp_ms);
void motion_supervisor_clear_commands(motion_supervisor_t *supervisor);
void motion_supervisor_set_mssd_configuration(motion_supervisor_t *supervisor,
                                              bool read,
                                              bool matches);
void motion_supervisor_set_kinematics_ready(motion_supervisor_t *supervisor,
                                            bool ready);
void motion_supervisor_set_motor_feedback(motion_supervisor_t *supervisor,
                                          bool right_valid,
                                          uint16_t right_fault_code,
                                          bool left_valid,
                                          uint16_t left_fault_code);
void motion_supervisor_set_communication_fault(motion_supervisor_t *supervisor,
                                               bool faulted);
void motion_supervisor_record_partial_commit(motion_supervisor_t *supervisor);
motion_decision_t motion_supervisor_poll(const motion_supervisor_t *supervisor,
                                         uint32_t now_ms);

#endif
