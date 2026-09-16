#include "chassis_app.h"

#include "rc_capture.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static const uint32_t DEFAULT_STATUS_POLL_PERIOD_MS = 100u;
static const uint32_t DEFAULT_TARGET_REFRESH_PERIOD_MS = 50u;
static const uint32_t DEFAULT_INSPECT_REQUEST_TIMEOUT_MS = 1000u;

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (uint32_t)(now_ms - deadline_ms) < 0x80000000u;
}

chassis_app_config_t chassis_app_default_config(void)
{
    chassis_app_config_t config;

    (void)memset(&config, 0, sizeof(config));
    config.kinematics = chassis_kinematics_default_config();
    config.supervisor = motion_supervisor_default_config();
    config.mssd = mssd_driver_default_config();
    config.rs485_timeouts.dma_ms = 20u;
    config.rs485_timeouts.uart_tc_ms = 20u;
    config.rs485_timeouts.response_ms = 100u;
    config.status_poll_period_ms = DEFAULT_STATUS_POLL_PERIOD_MS;
    config.target_refresh_period_ms = DEFAULT_TARGET_REFRESH_PERIOD_MS;
    config.inspect_request_timeout_ms = DEFAULT_INSPECT_REQUEST_TIMEOUT_MS;
    return config;
}

static bool config_is_valid(const chassis_app_config_t *config)
{
    return (config != NULL) &&
           (config->rs485_timeouts.dma_ms != 0u) &&
           (config->rs485_timeouts.uart_tc_ms != 0u) &&
           (config->rs485_timeouts.response_ms != 0u) &&
           (config->status_poll_period_ms != 0u) &&
           (config->target_refresh_period_ms != 0u) &&
           (config->inspect_request_timeout_ms != 0u) &&
           (!config->rc_mapping_verified ||
            ((config->rc_max_linear_mm_s > 0.0) &&
             isfinite(config->rc_max_linear_mm_s) &&
             (config->rc_max_angular_mrad_s > 0.0) &&
             isfinite(config->rc_max_angular_mrad_s)));
}

bool chassis_app_init(chassis_app_t *app,
                      const chassis_app_config_t *config,
                      UART_HandleTypeDef *mssd_uart,
                      DMA_HandleTypeDef *rx_dma,
                      DMA_HandleTypeDef *tx_dma,
                      GPIO_TypeDef *de_port,
                      uint16_t de_pin)
{
    rs485_port_t port;

    if ((app == NULL) || !config_is_valid(config))
    {
        return false;
    }
    (void)memset(app, 0, sizeof(*app));
    app->config = *config;
    app->machine = app_state_machine_initial();
    rc_input_init(&app->rc_input);
    jetson_protocol_init(&app->protocol);
    if (!motion_supervisor_init(&app->supervisor, &config->supervisor) ||
        !usart2_rs485_adapter_init(&app->usart2_adapter,
                                   mssd_uart,
                                   rx_dma,
                                   tx_dma,
                                   de_port,
                                   de_pin,
                                   &app->transport,
                                   &app->mssd))
    {
        return false;
    }
    port = usart2_rs485_adapter_get_port(&app->usart2_adapter);
    rs485_transport_init(&app->transport, port, config->rs485_timeouts);
    if ((app->transport.state != RS485_IDLE) ||
        !mssd_driver_init(&app->mssd, &app->transport, &config->mssd))
    {
        return false;
    }
    motion_supervisor_set_kinematics_ready(
        &app->supervisor,
        chassis_kinematics_config_allows_nonzero(&config->kinematics));
    app->startup_state = CHASSIS_APP_STARTUP_CONFIGURATION;
    app->diagnostics.motor_target_reached_available = false;
    app->diagnostics.brake_feedback_available = false;
    return true;
}

bool chassis_app_start(chassis_app_t *app)
{
    if ((app == NULL) || app->started ||
        !usart2_rs485_adapter_start(&app->usart2_adapter))
    {
        return false;
    }
    if (!jetson_uart_transport_start(&app->jetson_transport, &app->protocol))
    {
        (void)usart2_rs485_adapter_stop(&app->usart2_adapter);
        return false;
    }
    if (!rc_capture_start())
    {
        (void)jetson_uart_transport_stop(&app->jetson_transport);
        (void)usart2_rs485_adapter_stop(&app->usart2_adapter);
        return false;
    }
    app->started = true;
    return true;
}

bool chassis_app_stop(chassis_app_t *app)
{
    bool success = true;

    if ((app == NULL) || !app->started)
    {
        return false;
    }
    rc_capture_stop();
    if (!jetson_uart_transport_stop(&app->jetson_transport))
    {
        success = false;
    }
    if (!usart2_rs485_adapter_stop(&app->usart2_adapter))
    {
        success = false;
    }
    app->started = false;
    mssd_driver_set_motion_authorized(&app->mssd, false);
    return success;
}

static void apply_app_event(chassis_app_t *app, app_event_t event,
                            uint32_t now_ms)
{
    const app_transition_result_t result =
        app_state_machine_transition(app->machine, event);

    app->machine = result.machine;
    motion_supervisor_set_app_state(&app->supervisor,
                                    result.machine.state,
                                    result.actions);
    if ((result.actions & APP_ACTION_REQUEST_INSPECTION) != 0u)
    {
        app->inspect_request_active = true;
        app->inspect_request_started_ms = now_ms;
        app->pending_jetson_function = JETSON_FUNC_INSPECT_REQUEST;
    }
    if ((result.actions & APP_ACTION_CANCEL_INSPECTION) != 0u)
    {
        app->inspect_request_active = false;
        app->pending_jetson_function = JETSON_FUNC_INSPECT_CANCELLED;
    }
}

static void service_jetson_tx(chassis_app_t *app)
{
    uint8_t frame[JETSON_PROTOCOL_MAX_FRAME_SIZE];
    size_t length = 0u;

    if ((app->pending_jetson_function == 0u) ||
        !jetson_protocol_build(JETSON_PROTOCOL_TX_HEAD,
                               app->pending_jetson_function,
                               NULL,
                               0u,
                               frame,
                               sizeof(frame),
                               &length))
    {
        return;
    }
    if (jetson_uart_transport_send_async(&app->jetson_transport,
                                         frame,
                                         length))
    {
        app->pending_jetson_function = 0u;
    }
}

static void process_rc_events(chassis_app_t *app, uint32_t events,
                              uint32_t now_ms)
{
    static const struct
    {
        uint32_t mask;
        app_event_t event;
    } mappings[] = {
        {RC_INPUT_EVENT_EMERGENCY_ASSERTED, APP_EVENT_RC_EMERGENCY_ASSERTED},
        {RC_INPUT_EVENT_SIGNAL_LOST, APP_EVENT_RC_SIGNAL_LOST},
        {RC_INPUT_EVENT_SELECT_REMOTE, APP_EVENT_RC_SELECT_REMOTE},
        {RC_INPUT_EVENT_SELECT_STANDBY, APP_EVENT_RC_SELECT_STANDBY},
        {RC_INPUT_EVENT_NEUTRAL_CONFIRMED, APP_EVENT_RC_NEUTRAL_CONFIRMED},
        {RC_INPUT_EVENT_INSPECT_START, APP_EVENT_RC_INSPECT_START},
        {RC_INPUT_EVENT_EMERGENCY_RELEASED, APP_EVENT_RC_EMERGENCY_RELEASED},
    };
    size_t index;

    for (index = 0u; index < (sizeof(mappings) / sizeof(mappings[0])); ++index)
    {
        if ((events & mappings[index].mask) != 0u)
        {
            apply_app_event(app, mappings[index].event, now_ms);
        }
    }
}

static void drain_rc_capture(chassis_app_t *app, uint32_t now_ms)
{
    rc_capture_sample_t sample;
    uint32_t events = RC_INPUT_EVENT_NONE;
    rc_input_snapshot_t snapshot;

    while (rc_capture_take_sample(&sample))
    {
        events |= rc_input_process_sample(&app->rc_input,
                                          sample.channel,
                                          sample.pulse_us,
                                          sample.timestamp_ms);
    }
    events |= rc_input_poll(&app->rc_input, now_ms);
    process_rc_events(app, events, now_ms);
    snapshot = rc_input_get_snapshot(&app->rc_input);
    if (app->config.rc_mapping_verified && snapshot.signal_valid)
    {
        motion_supervisor_submit_rc(
            &app->supervisor,
            ((double)snapshot.ch2_normalized / 1000.0) *
                app->config.rc_max_linear_mm_s,
            ((double)snapshot.ch1_normalized / 1000.0) *
                app->config.rc_max_angular_mrad_s,
            now_ms);
    }
}

static void process_jetson_frame(chassis_app_t *app,
                                 const jetson_frame_t *frame,
                                 uint32_t now_ms)
{
    jetson_motion_frame_t motion;

    if (jetson_protocol_decode_motion(frame, &motion))
    {
        motion_supervisor_submit_jetson(&app->supervisor,
                                        (double)motion.vx,
                                        (double)motion.vz,
                                        motion.timestamp_ms);
        return;
    }
    switch (frame->function)
    {
        case JETSON_FUNC_INSPECT_ACCEPTED:
            app->inspect_request_active = false;
            apply_app_event(app, APP_EVENT_JETSON_INSPECT_ACCEPTED, now_ms);
            break;
        case JETSON_FUNC_INSPECT_REJECTED:
            app->inspect_request_active = false;
            apply_app_event(app, APP_EVENT_JETSON_INSPECT_REJECTED, now_ms);
            break;
        case JETSON_FUNC_INSPECT_FINISHED:
            apply_app_event(app, APP_EVENT_JETSON_INSPECT_FINISHED, now_ms);
            break;
        case JETSON_FUNC_INSPECT_CANCELLED:
            app->inspect_request_active = false;
            apply_app_event(app, APP_EVENT_JETSON_INSPECT_CANCELLED, now_ms);
            break;
        default:
            break;
    }
}

static void drain_jetson(chassis_app_t *app, uint32_t now_ms)
{
    jetson_frame_t frame;

    jetson_uart_transport_poll(&app->jetson_transport, now_ms);
    while (jetson_protocol_take_frame(&app->protocol, &frame))
    {
        process_jetson_frame(app, &frame, now_ms);
    }
    if (app->inspect_request_active &&
        ((uint32_t)(now_ms - app->inspect_request_started_ms) >=
         app->config.inspect_request_timeout_ms))
    {
        app->inspect_request_active = false;
        apply_app_event(app, APP_EVENT_INSPECT_REQUEST_TIMEOUT, now_ms);
    }
}

static void update_supervisor_feedback(chassis_app_t *app)
{
    mssd_configuration_snapshot_t configuration;
    mssd_motor_status_t right;
    mssd_motor_status_t left;
    const bool configuration_valid =
        mssd_driver_get_configuration(&app->mssd, &configuration);
    const bool right_valid =
        mssd_driver_get_status(&app->mssd, MSSD_CHANNEL_RIGHT, &right);
    const bool left_valid =
        mssd_driver_get_status(&app->mssd, MSSD_CHANNEL_LEFT, &left);
    const bool configuration_matches = configuration_valid &&
        (configuration.system_mode == 1u) &&
        (configuration.control_source == 0u);

    motion_supervisor_set_mssd_configuration(&app->supervisor,
                                             configuration_valid,
                                             configuration_matches);
    motion_supervisor_set_motor_feedback(&app->supervisor,
                                         right_valid,
                                         right_valid ? right.fault_code : 0u,
                                         left_valid,
                                         left_valid ? left.fault_code : 0u);
    app->diagnostics.mssd_configuration_read = configuration_valid;
    app->diagnostics.mssd_configuration_matches = configuration_matches;
    if (right_valid)
    {
        app->diagnostics.right_feedback_rpm = right.rpm;
        app->diagnostics.right_fault_code = right.fault_code;
    }
    if (left_valid)
    {
        app->diagnostics.left_feedback_rpm = left.rpm;
        app->diagnostics.left_fault_code = left.fault_code;
    }
}

static void advance_self_check(chassis_app_t *app, uint32_t now_ms)
{
    const bool both_feedback_valid = app->supervisor.right_feedback_valid &&
                                     app->supervisor.left_feedback_valid;
    const bool motor_fault = (app->supervisor.right_fault_code != 0u) ||
                             (app->supervisor.left_fault_code != 0u);

    if (app->self_check_completed ||
        (app->machine.state != APP_STATE_BOOT_SAFE))
    {
        return;
    }
    if (app->supervisor.mssd_configuration_read &&
        !app->supervisor.mssd_configuration_matches)
    {
        app->self_check_completed = true;
        apply_app_event(app, APP_EVENT_SELF_CHECK_FAILED, now_ms);
    }
    else if (both_feedback_valid && motor_fault)
    {
        app->self_check_completed = true;
        apply_app_event(app, APP_EVENT_MOTOR_FAULT, now_ms);
    }
    else if (app->supervisor.mssd_configuration_matches &&
             both_feedback_valid &&
             (app->machine.selected_mode != APP_MODE_SELECTION_INVALID))
    {
        app->self_check_completed = true;
        apply_app_event(app, APP_EVENT_SELF_CHECK_PASSED, now_ms);
    }
}

static void record_terminal_transaction(chassis_app_t *app, uint32_t now_ms)
{
    const mssd_driver_state_t state = mssd_driver_get_state(&app->mssd);
    const mssd_transaction_result_t result = mssd_driver_get_result(&app->mssd);
    const mssd_error_detail_t error = mssd_driver_get_error(&app->mssd);
    const mssd_operation_t operation = app->mssd.operation;

    if ((state != MSSD_DRIVER_COMPLETE) &&
        (state != MSSD_DRIVER_PARTIAL_COMMIT) &&
        (state != MSSD_DRIVER_FAILED))
    {
        return;
    }
    app->diagnostics.last_mssd_completion = result.completion;
    app->diagnostics.last_mssd_error = error;
    if (state == MSSD_DRIVER_PARTIAL_COMMIT)
    {
        motion_supervisor_record_partial_commit(&app->supervisor);
    }
    else if (state == MSSD_DRIVER_FAILED)
    {
        motion_supervisor_set_communication_fault(&app->supervisor, true);
        if (error.code == MSSD_ERROR_TIMEOUT)
        {
            ++app->diagnostics.communication_timeouts;
        }
        if (error.modbus_status == MODBUS_RTU_VALIDATION_CRC_ERROR)
        {
            ++app->diagnostics.crc_errors;
        }
    }
    if ((state == MSSD_DRIVER_COMPLETE) &&
        (operation == MSSD_OPERATION_READ_CONFIGURATION))
    {
        app->startup_state = CHASSIS_APP_STARTUP_RIGHT_STATUS;
    }
    else if ((state == MSSD_DRIVER_COMPLETE) &&
             (operation == MSSD_OPERATION_READ_RIGHT_STATUS))
    {
        app->startup_state = CHASSIS_APP_STARTUP_LEFT_STATUS;
    }
    else if ((state == MSSD_DRIVER_COMPLETE) &&
             (operation == MSSD_OPERATION_READ_LEFT_STATUS))
    {
        app->startup_state = CHASSIS_APP_RUNNING;
        app->next_status_poll_ms = now_ms + app->config.status_poll_period_ms;
    }
    if (operation == MSSD_OPERATION_STOP_BOTH)
    {
        app->safe_stop_pending = false;
        app->safe_stop_attempted = true;
        app->safe_stop_delivered = (state == MSSD_DRIVER_COMPLETE);
        if (state == MSSD_DRIVER_PARTIAL_COMMIT)
        {
            app->pending_stop_type = MSSD_STOP_EMERGENCY;
            app->pending_stop_reason = MOTION_STOP_PARTIAL_COMMIT;
        }
        else if (state == MSSD_DRIVER_FAILED)
        {
            app->pending_stop_type = MSSD_STOP_EMERGENCY;
            app->pending_stop_reason = MOTION_STOP_COMMUNICATION;
        }
    }
    update_supervisor_feedback(app);
    (void)mssd_driver_acknowledge(&app->mssd, now_ms);
}

static void latch_safe_stop(chassis_app_t *app,
                            const motion_decision_t *decision,
                            uint32_t now_ms)
{
    const bool changed = !app->safe_stop_attempted ||
        (app->pending_stop_type != decision->stop_type) ||
        (app->pending_stop_reason != decision->stop_reason);
    const mssd_driver_state_t state = mssd_driver_get_state(&app->mssd);

    if (!decision->stop_requested)
    {
        app->safe_stop_pending = false;
        app->safe_stop_delivered = false;
        app->safe_stop_attempted = false;
        return;
    }
    if (changed)
    {
        app->safe_stop_pending = true;
        app->safe_stop_delivered = false;
        app->safe_stop_attempted = false;
        app->pending_stop_type = decision->stop_type;
        app->pending_stop_reason = decision->stop_reason;
    }
    if (app->safe_stop_pending &&
        ((app->startup_state == CHASSIS_APP_RUNNING) ||
         (app->machine.state == APP_STATE_EMERGENCY) ||
         (app->machine.state == APP_STATE_FAULT) ||
         (decision->stop_reason == MOTION_STOP_COMMUNICATION) ||
         (decision->stop_reason == MOTION_STOP_PARTIAL_COMMIT)) &&
        ((state == MSSD_DRIVER_WAITING_TRANSPORT) ||
         (state == MSSD_DRIVER_WAITING_RESPONSE)) &&
        (app->mssd.operation != MSSD_OPERATION_STOP_BOTH))
    {
        (void)mssd_driver_abort_for_safety(&app->mssd, now_ms);
    }
}

static bool schedule_startup_or_poll(chassis_app_t *app, uint32_t now_ms)
{
    mssd_request_status_t status = MSSD_REQUEST_BUSY;

    switch (app->startup_state)
    {
        case CHASSIS_APP_STARTUP_CONFIGURATION:
            status = mssd_driver_request_configuration(&app->mssd, now_ms);
            break;
        case CHASSIS_APP_STARTUP_RIGHT_STATUS:
            status = mssd_driver_request_status(&app->mssd,
                                                MSSD_CHANNEL_RIGHT,
                                                now_ms);
            break;
        case CHASSIS_APP_STARTUP_LEFT_STATUS:
            status = mssd_driver_request_status(&app->mssd,
                                                MSSD_CHANNEL_LEFT,
                                                now_ms);
            break;
        case CHASSIS_APP_RUNNING:
            if (time_reached(now_ms, app->next_status_poll_ms))
            {
                status = mssd_driver_request_status(&app->mssd,
                                                    MSSD_CHANNEL_RIGHT,
                                                    now_ms);
                if (status == MSSD_REQUEST_ACCEPTED)
                {
                    app->startup_state = CHASSIS_APP_STARTUP_RIGHT_STATUS;
                }
            }
            break;
        default:
            motion_supervisor_set_communication_fault(&app->supervisor, true);
            break;
    }
    return status == MSSD_REQUEST_ACCEPTED;
}

static void schedule_motion(chassis_app_t *app,
                            const motion_decision_t *decision,
                            uint32_t now_ms)
{
    chassis_kinematics_output_t output;
    chassis_kinematics_status_t status;

    if (mssd_driver_get_state(&app->mssd) != MSSD_DRIVER_IDLE)
    {
        return;
    }
    if (app->safe_stop_pending)
    {
        if (mssd_driver_request_stop(&app->mssd,
                                     MSSD_CHANNEL_BOTH,
                                     app->pending_stop_type,
                                     now_ms) == MSSD_REQUEST_ACCEPTED)
        {
            app->safe_stop_attempted = true;
        }
        return;
    }
    if (app->startup_state != CHASSIS_APP_RUNNING)
    {
        (void)schedule_startup_or_poll(app, now_ms);
        return;
    }
    if (time_reached(now_ms, app->next_status_poll_ms))
    {
        (void)schedule_startup_or_poll(app, now_ms);
        return;
    }
    if (!decision->motion_authorized ||
        !time_reached(now_ms, app->next_target_refresh_ms))
    {
        return;
    }
    status = chassis_kinematics_calculate(&app->config.kinematics,
                                          decision->linear_mm_s,
                                          decision->angular_mrad_s,
                                          &output);
    if ((status != CHASSIS_KINEMATICS_OK) &&
        (status != CHASSIS_KINEMATICS_ZERO_SAFE))
    {
        motion_supervisor_set_kinematics_ready(&app->supervisor, false);
        app->safe_stop_pending = true;
        app->safe_stop_delivered = false;
        app->pending_stop_type = MSSD_STOP_EMERGENCY;
        app->pending_stop_reason = MOTION_STOP_KINEMATICS;
        return;
    }
    mssd_driver_set_motion_authorized(&app->mssd, true);
    if (mssd_driver_request_target_pair(&app->mssd,
                                        output.right_motor_rpm,
                                        output.left_motor_rpm,
                                        now_ms) == MSSD_REQUEST_ACCEPTED)
    {
        app->diagnostics.right_target_rpm = output.right_motor_rpm;
        app->diagnostics.left_target_rpm = output.left_motor_rpm;
        app->next_target_refresh_ms = now_ms + app->config.target_refresh_period_ms;
    }
    else
    {
        mssd_driver_set_motion_authorized(&app->mssd, false);
    }
}

static void update_diagnostics(chassis_app_t *app,
                               const motion_decision_t *decision)
{
    app->diagnostics.app_state = app->machine.state;
    app->diagnostics.control_source = decision->control_source;
    app->diagnostics.rc_command_fresh = decision->rc_fresh;
    app->diagnostics.jetson_command_fresh = decision->jetson_fresh;
    app->diagnostics.motion_authorized =
        decision->motion_authorized &&
        mssd_driver_get_motion_authorized(&app->mssd);
    app->diagnostics.motion_inhibit_reasons = decision->inhibit_reasons;
    app->diagnostics.partial_commit_count = app->supervisor.partial_commit_count;
    app->diagnostics.stop_reason = decision->stop_reason;
    app->diagnostics.uart_errors = app->usart2_adapter.stats.uart_errors +
        app->jetson_transport.stats.uart_errors;
    app->diagnostics.crc_errors = app->usart2_adapter.stats.crc_errors;
    app->diagnostics.dma_errors = app->usart2_adapter.stats.dma_errors;
    app->diagnostics.rx_overflows = app->usart2_adapter.stats.rx_overruns +
        app->jetson_transport.stats.software_overruns;
}

void chassis_app_poll(chassis_app_t *app, uint32_t now_ms)
{
    motion_decision_t decision;

    if ((app == NULL) || !app->started)
    {
        return;
    }
    drain_rc_capture(app, now_ms);
    drain_jetson(app, now_ms);
    service_jetson_tx(app);
    usart2_rs485_adapter_poll(&app->usart2_adapter, now_ms);
    record_terminal_transaction(app, now_ms);
    update_supervisor_feedback(app);
    if ((app->usart2_adapter.stats.uart_errors != 0u) ||
        (app->usart2_adapter.stats.rx_overruns != 0u) ||
        (app->usart2_adapter.stats.recovery_failures != 0u))
    {
        motion_supervisor_set_communication_fault(&app->supervisor, true);
    }
    advance_self_check(app, now_ms);
    motion_supervisor_set_app_state(&app->supervisor, app->machine.state, 0u);
    decision = motion_supervisor_poll(&app->supervisor, now_ms);
    if ((decision.stop_reason == MOTION_STOP_JETSON_TIMEOUT) &&
        (app->machine.state == APP_STATE_INSPECTING))
    {
        apply_app_event(app, APP_EVENT_JETSON_MOTION_TIMEOUT, now_ms);
        decision = motion_supervisor_poll(&app->supervisor, now_ms);
    }
    latch_safe_stop(app, &decision, now_ms);
    mssd_driver_set_motion_authorized(&app->mssd, false);
    if ((app->startup_state != CHASSIS_APP_STARTUP_CONFIGURATION) ||
        (mssd_driver_get_state(&app->mssd) != MSSD_DRIVER_IDLE))
    {
        schedule_motion(app, &decision, now_ms);
    }
    else
    {
        (void)schedule_startup_or_poll(app, now_ms);
    }
    update_diagnostics(app, &decision);
}

bool chassis_app_get_diagnostics(const chassis_app_t *app,
                                 chassis_app_diagnostics_t *diagnostics)
{
    if ((app == NULL) || (diagnostics == NULL))
    {
        return false;
    }
    *diagnostics = app->diagnostics;
    return true;
}
