#include "chassis_app.h"

#include "rc_capture.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static const uint32_t DEFAULT_STATUS_POLL_PERIOD_MS = 100u;
static const uint32_t DEFAULT_TARGET_REFRESH_PERIOD_MS = 50u;
static const uint32_t DEFAULT_INSPECT_REQUEST_TIMEOUT_MS = 1000u;
static const uint32_t DEFAULT_SELF_CHECK_PERIOD_MS = 500u;
static const uint32_t DEFAULT_EMERGENCY_BEEP_HALF_PERIOD_MS = 200u;

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
    config.jetson_self_check_period_ms = DEFAULT_SELF_CHECK_PERIOD_MS;
    config.emergency_beep_half_period_ms =
        DEFAULT_EMERGENCY_BEEP_HALF_PERIOD_MS;
    config.require_jetson_self_check = true;
    config.beep_active_high = true;
    return config;
}

chassis_app_config_t chassis_app_wheel_rc_config(void)
{
    chassis_app_config_t config = chassis_app_default_config();
    config.profile = CHASSIS_STANDALONE_RC;
    config.require_jetson_self_check = false;
    /* HT-10A axis mapping measured on this board; scales from the Keil RC path. */
    config.rc_mapping_verified = true;
    config.rc_max_linear_mm_s = 500.0;
    config.rc_max_angular_mrad_s = 1500.0;
    config.kinematics.legacy_wheel_quantization = true;
    /* Command envelope of the original mixer at full simultaneous input. */
    config.kinematics.maximum_motor_rpm = 426.0;
    config.kinematics.direction_mapping_verified = true;
    config.kinematics.maximum_rpm_verified = true;
    config.supervisor.motion_enable = true;
    config.mssd.right_brake = (mssd_brake_mapping_t){1u, 0u, true};
    config.mssd.left_brake = config.mssd.right_brake;
    config.target_refresh_period_ms = 20u;
    config.status_poll_period_ms = 500u;
    return config;
}

static bool config_is_valid(const chassis_app_config_t *config)
{
    return (config != NULL) &&
           ((config->profile == CHASSIS_HOST_INSPECTION) ||
            (config->profile == CHASSIS_STANDALONE_RC)) &&
           (config->rs485_timeouts.dma_ms != 0u) &&
           (config->rs485_timeouts.uart_tc_ms != 0u) &&
           (config->rs485_timeouts.response_ms != 0u) &&
           (config->status_poll_period_ms != 0u) &&
           (config->target_refresh_period_ms != 0u) &&
           (config->inspect_request_timeout_ms != 0u) &&
           (config->jetson_self_check_period_ms != 0u) &&
           (config->emergency_beep_half_period_ms != 0u) &&
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
    app->machine.profile = config->profile;
    rc_input_init(&app->rc_input);
    app->rc_input.ignore_inspection_channel = config->profile == CHASSIS_STANDALONE_RC;
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
    app->startup_state = config->profile == CHASSIS_STANDALONE_RC ?
        CHASSIS_APP_STARTUP_WRITE_CONFIGURATION : CHASSIS_APP_STARTUP_CONFIGURATION;
    app->diagnostics.motor_target_reached_available = false;
    app->diagnostics.brake_feedback_available = false;
    return true;
}

bool chassis_app_attach_status_outputs(chassis_app_t *app,
                                       GPIO_TypeDef *beep_port,
                                       uint16_t beep_pin)
{
    if (app == NULL)
    {
        return false;
    }
    return board_status_outputs_init(&app->status_outputs,
                                     beep_port,
                                     beep_pin,
                                     app->config.beep_active_high,
                                     app->config.emergency_beep_half_period_ms);
}

bool chassis_app_start(chassis_app_t *app)
{
    if ((app == NULL) || app->started ||
        !usart2_rs485_adapter_start(&app->usart2_adapter))
    {
        return false;
    }
    const bool use_host = app->config.profile == CHASSIS_HOST_INSPECTION;
    if (use_host && !jetson_uart_transport_start(&app->jetson_transport, &app->protocol))
    {
        (void)usart2_rs485_adapter_stop(&app->usart2_adapter);
        return false;
    }
    if (!rc_capture_start(use_host))
    {
        if (use_host)
        {
            (void)jetson_uart_transport_stop(&app->jetson_transport);
        }
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
    board_status_outputs_stop(&app->status_outputs);
    if ((app->config.profile == CHASSIS_HOST_INSPECTION) &&
        !jetson_uart_transport_stop(&app->jetson_transport))
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

static app_transition_result_t apply_app_event(chassis_app_t *app,
                                               app_event_t event,
                                               uint32_t now_ms)
{
    const app_transition_result_t result =
        app_state_machine_transition(app->machine, event);

    if ((result.machine.state == APP_STATE_EMERGENCY) &&
        (app->config.profile == CHASSIS_STANDALONE_RC))
    {
        app->self_check_completed = false;
    }

    if ((result.machine.state == APP_STATE_REMOTE_ARMING) &&
        (app->machine.state != APP_STATE_REMOTE_ARMING))
    {
        rc_input_restart_neutral_confirmation(&app->rc_input);
    }
    /* Keep the stop even if later samples in this foreground pass change
       the state again before the motor scheduler runs. */
    if (app->config.profile == CHASSIS_STANDALONE_RC)
    {
        if (app->machine.state == APP_STATE_FAULT && event == APP_EVENT_RC_SELECT_STANDBY)
        {
            app->wheel_configuration_retry_requested = true;
        }
        if ((app->machine.state != result.machine.state) &&
            ((result.machine.state == APP_STATE_EMERGENCY) ||
             (result.machine.state == APP_STATE_FAULT) ||
             ((result.machine.state == APP_STATE_STANDBY) &&
              (app->wheel_brake_state != WHEEL_PARKED))))
        {
            app->wheel_brake_state = WHEEL_STOP_REQUIRED;
        }
        if ((app->machine.state == APP_STATE_EMERGENCY) &&
            (result.machine.state == APP_STATE_BOOT_SAFE))
        {
            app->wheel_recheck_pending = true;
        }
    }
    else if (((event == APP_EVENT_RC_EMERGENCY_ASSERTED) &&
         (result.machine.state == APP_STATE_EMERGENCY)) ||
        (((app->machine.state == APP_STATE_REMOTE) ||
          (app->machine.state == APP_STATE_INSPECTING)) &&
         ((result.actions & APP_ACTION_REQUEST_ZERO_SPEED) != 0u)))
    {
        const bool emergency = (result.machine.state == APP_STATE_EMERGENCY) ||
            (result.machine.state == APP_STATE_FAULT) ||
            (event == APP_EVENT_JETSON_MOTION_TIMEOUT);
        if (emergency || !app->safe_stop_pending ||
            (app->pending_stop_type != MSSD_STOP_EMERGENCY))
        {
            app->safe_stop_pending = true;
            app->safe_stop_attempted = false;
            app->safe_stop_delivered = false;
            app->pending_stop_type = emergency ? MSSD_STOP_EMERGENCY : MSSD_STOP_NORMAL;
            app->pending_stop_reason = MOTION_STOP_APPLICATION_LOCK;
            if (event == APP_EVENT_JETSON_MOTION_TIMEOUT)
            {
                app->pending_stop_reason = MOTION_STOP_JETSON_TIMEOUT;
            }
            else if (event == APP_EVENT_RC_EMERGENCY_ASSERTED)
            {
                app->pending_stop_reason = MOTION_STOP_EMERGENCY;
            }
        }
    }
    if ((result.machine.state == APP_STATE_FAULT) &&
        (app->machine.state != APP_STATE_FAULT))
    {
        if ((event == APP_EVENT_RC_SIGNAL_LOST) ||
            (event == APP_EVENT_RC_CAPTURE_LOST))
        {
            app->fault_reason = MOTION_STOP_RC_SIGNAL_LOST;
        }
        else if (event == APP_EVENT_COMMUNICATION_FAULT)
        {
            app->fault_reason = app->supervisor.partial_commit_latched ?
                MOTION_STOP_PARTIAL_COMMIT : MOTION_STOP_COMMUNICATION;
        }
        else if (event == APP_EVENT_SELF_CHECK_FAILED)
        {
            app->fault_reason = MOTION_STOP_CONFIGURATION;
        }
        else
        {
            app->fault_reason = MOTION_STOP_MOTOR_FAULT;
        }
        if (app->config.profile == CHASSIS_STANDALONE_RC)
        {
            app->wheel_configuration_retry_requested = false;
            app->next_status_poll_ms = now_ms + app->config.status_poll_period_ms;
            if (app->fault_reason == MOTION_STOP_MOTOR_FAULT)
            {
                app->startup_state = CHASSIS_APP_STARTUP_RIGHT_STATUS;
            }
        }
    }
    app->machine = result.machine;
    motion_supervisor_set_app_state(&app->supervisor,
                                    result.machine.state,
                                    result.actions);
    if ((result.actions & APP_ACTION_REQUEST_INSPECTION) != 0u)
    {
        app->inspect_request_active = true;
        ++app->inspect_request_generation;
        app->inspect_request_sent = false;
        app->inspect_request_started_ms = now_ms;
        app->jetson_outbox.inspect_start_pending = true;
    }
    if ((result.actions & APP_ACTION_CANCEL_INSPECTION) != 0u)
    {
        app->inspect_request_active = false;
        app->inspect_request_sent = false;
        app->jetson_outbox.inspect_start_pending = false;
        app->jetson_outbox.inspect_stop_pending = true;
    }
    return result;
}

static bool select_jetson_tx(chassis_app_t *app,
                             uint8_t *function,
                             uint8_t *payload,
                             bool **pending,
                             chassis_jetson_tx_kind_t *kind)
{
    if (app->jetson_outbox.emergency_pending)
    {
        *function = JETSON_FUNC_LEGACY_EMERGENCY;
        *payload = app->jetson_outbox.emergency_value;
        *pending = &app->jetson_outbox.emergency_pending;
        *kind = CHASSIS_JETSON_TX_EMERGENCY;
    }
    else if (app->jetson_outbox.inspect_stop_pending)
    {
        *function = JETSON_FUNC_LEGACY_INSPECT_CTRL;
        *payload = 0u;
        *pending = &app->jetson_outbox.inspect_stop_pending;
        *kind = CHASSIS_JETSON_TX_INSPECT_STOP;
    }
    else if (app->jetson_outbox.mode_pending)
    {
        *function = JETSON_FUNC_LEGACY_MODE_REPORT;
        *payload = app->jetson_outbox.mode_value;
        *pending = &app->jetson_outbox.mode_pending;
        *kind = CHASSIS_JETSON_TX_MODE;
    }
    else if (app->jetson_outbox.inspect_start_pending)
    {
        *function = JETSON_FUNC_LEGACY_INSPECT_CTRL;
        *payload = 1u;
        *pending = &app->jetson_outbox.inspect_start_pending;
        *kind = CHASSIS_JETSON_TX_INSPECT_START;
    }
    else if (app->jetson_outbox.self_check_pending)
    {
        *function = JETSON_FUNC_LEGACY_VERSION_OR_SELF_CHECK;
        *payload = 0u;
        *pending = &app->jetson_outbox.self_check_pending;
        *kind = CHASSIS_JETSON_TX_SELF_CHECK;
    }
    else
    {
        return false;
    }
    return true;
}

static void complete_jetson_tx(chassis_app_t *app)
{
    bool *pending = NULL;
    uint8_t current_payload = app->jetson_tx_payload;

    if (!app->jetson_tx_inflight)
    {
        return;
    }
    if (app->jetson_transport.stats.tx_completions ==
        app->jetson_tx_completion_snapshot)
    {
        if (!app->jetson_transport.tx_busy)
        {
            if (app->jetson_tx_kind == CHASSIS_JETSON_TX_SELF_CHECK)
            {
                app->jetson_self_check_request_outstanding = false;
            }
            app->jetson_tx_inflight = false;
            app->jetson_tx_kind = CHASSIS_JETSON_TX_NONE;
        }
        return;
    }

    switch (app->jetson_tx_kind)
    {
        case CHASSIS_JETSON_TX_SELF_CHECK:
            pending = &app->jetson_outbox.self_check_pending;
            break;
        case CHASSIS_JETSON_TX_INSPECT_START:
            if (app->inspect_tx_generation == app->inspect_request_generation)
            {
                app->inspect_request_sent = app->inspect_request_active;
                pending = &app->jetson_outbox.inspect_start_pending;
            }
            break;
        case CHASSIS_JETSON_TX_INSPECT_STOP:
            pending = &app->jetson_outbox.inspect_stop_pending;
            break;
        case CHASSIS_JETSON_TX_EMERGENCY:
            pending = &app->jetson_outbox.emergency_pending;
            current_payload = app->jetson_outbox.emergency_value;
            break;
        case CHASSIS_JETSON_TX_MODE:
            pending = &app->jetson_outbox.mode_pending;
            current_payload = app->jetson_outbox.mode_value;
            break;
        default:
            break;
    }
    if ((pending != NULL) && (current_payload == app->jetson_tx_payload))
    {
        *pending = false;
    }
    app->jetson_tx_inflight = false;
    app->jetson_tx_kind = CHASSIS_JETSON_TX_NONE;
}

static void service_jetson_tx(chassis_app_t *app, uint32_t now_ms)
{
    uint8_t frame[JETSON_PROTOCOL_MAX_FRAME_SIZE];
    uint8_t function;
    uint8_t payload;
    bool *pending;
    chassis_jetson_tx_kind_t kind;
    size_t length = 0u;

    complete_jetson_tx(app);
    if (app->jetson_tx_inflight ||
        !select_jetson_tx(app, &function, &payload, &pending, &kind) ||
        !jetson_protocol_build(JETSON_PROTOCOL_TX_HEAD,
                               function,
                               &payload,
                               1u,
                               frame,
                               sizeof(frame),
                               &length))
    {
        return;
    }
    const uint32_t completion_count = app->jetson_transport.stats.tx_completions;
    if (jetson_uart_transport_send_async(&app->jetson_transport,
                                         frame,
                                         length))
    {
        (void)pending;
        app->jetson_tx_inflight = true;
        app->jetson_tx_kind = kind;
        app->jetson_tx_payload = payload;
        app->jetson_tx_completion_snapshot = completion_count;
        if (kind == CHASSIS_JETSON_TX_INSPECT_START)
        {
            app->inspect_tx_generation = app->inspect_request_generation;
            app->inspect_request_sent_ms = now_ms;
        }
        if (function == JETSON_FUNC_LEGACY_VERSION_OR_SELF_CHECK)
        {
            app->jetson_self_check_request_outstanding = true;
            app->next_self_check_request_ms =
                now_ms + app->config.jetson_self_check_period_ms;
        }
    }
}

static void latch_rc_report(chassis_app_t *app,
                            app_event_t event,
                            app_state_machine_t before)
{
    const app_state_machine_t after = app->machine;

    if ((event == APP_EVENT_RC_EMERGENCY_ASSERTED) &&
        (before.state != APP_STATE_EMERGENCY) &&
        (after.state == APP_STATE_EMERGENCY))
    {
        app->jetson_outbox.emergency_pending = true;
        app->jetson_outbox.emergency_value = 1u;
    }
    else if ((event == APP_EVENT_RC_EMERGENCY_RELEASED) &&
             (before.state == APP_STATE_EMERGENCY) &&
             (after.state != APP_STATE_EMERGENCY))
    {
        app->jetson_outbox.emergency_pending = true;
        app->jetson_outbox.emergency_value = 0u;
        if (after.selected_mode != APP_MODE_SELECTION_INVALID)
        {
            app->jetson_outbox.mode_pending = true;
            app->jetson_outbox.mode_value =
                (after.selected_mode == APP_MODE_SELECTION_REMOTE) ? 0u : 1u;
        }
    }
    else if ((event == APP_EVENT_RC_SELECT_REMOTE) &&
             (after.state != APP_STATE_EMERGENCY) &&
             ((before.selected_mode != after.selected_mode) ||
              (before.state != after.state)))
    {
        app->jetson_outbox.mode_pending = true;
        app->jetson_outbox.mode_value = 0u;
    }
    else if ((event == APP_EVENT_RC_SELECT_STANDBY) &&
             (after.state != APP_STATE_EMERGENCY) &&
             ((before.selected_mode != after.selected_mode) ||
              (before.state != after.state)))
    {
        app->jetson_outbox.mode_pending = true;
        app->jetson_outbox.mode_value = 1u;
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
            const app_state_machine_t before = app->machine;

            if ((mappings[index].event == APP_EVENT_RC_EMERGENCY_RELEASED) &&
                !app->rc_input.snapshot.signal_valid)
            {
                app->rc_input.snapshot.emergency_asserted = true;
                continue;
            }
            (void)apply_app_event(app, mappings[index].event, now_ms);
            if ((mappings[index].event == APP_EVENT_RC_EMERGENCY_RELEASED) &&
                (app->machine.state == APP_STATE_EMERGENCY))
            {
                app->rc_input.snapshot.emergency_asserted = true;
            }
            if (app->config.profile == CHASSIS_HOST_INSPECTION)
            {
                latch_rc_report(app, mappings[index].event, before);
            }
        }
    }
}

static void drain_rc_capture(chassis_app_t *app, uint32_t now_ms)
{
    rc_capture_sample_t sample;
    rc_input_snapshot_t snapshot;
    rc_capture_snapshot_t capture;

    for (unsigned count = 0u; (count < RC_CAPTURE_QUEUE_CAPACITY) &&
         rc_capture_take_sample(&sample); ++count)
    {
        /* An IRQ can enqueue a sample one tick after the caller read now_ms. */
        if (!time_reached(now_ms, sample.timestamp_ms))
        {
            sample.timestamp_ms = now_ms;
        }
        process_rc_events(app, rc_input_poll(&app->rc_input, sample.timestamp_ms),
                          now_ms);
        const uint32_t events = rc_input_process_sample(&app->rc_input,
                                                        sample.channel,
                                                        sample.pulse_us,
                                                        sample.timestamp_ms);

        process_rc_events(app, events, now_ms);
        if (app->machine.state == APP_STATE_REMOTE &&
            app->config.profile == CHASSIS_STANDALONE_RC &&
            sample.channel != RC_INPUT_CH5 &&
            (uint32_t)sample.channel < RC_INPUT_CHANNEL_COUNT &&
            !app->rc_input.snapshot.channel_valid[sample.channel])
        {
            /* A later good pulse in this same queue must not erase the loss. */
            apply_app_event(app, APP_EVENT_RC_SIGNAL_LOST, now_ms);
        }
    }
    rc_capture_read_snapshot(&capture);
    if ((capture.dropped_samples != app->diagnostics.rc_capture_drops) &&
        (app->machine.state != APP_STATE_FAULT))
    {
        /* Missing events can include an entire emergency assertion. */
        (void)apply_app_event(app, APP_EVENT_RC_CAPTURE_LOST, now_ms);
    }
    app->diagnostics.rc_capture_drops = capture.dropped_samples;
    process_rc_events(app, rc_input_poll(&app->rc_input, now_ms), now_ms);
    snapshot = rc_input_get_snapshot(&app->rc_input);
    const bool wheel = app->config.profile == CHASSIS_STANDALONE_RC;
    if (wheel && app->machine.state != APP_STATE_REMOTE)
    {
        app->supervisor.rc_command.valid = false;
        return;
    }
    if (app->config.rc_mapping_verified && snapshot.signal_valid)
    {
        if (wheel && app->supervisor.rc_command.valid &&
            !time_reached(now_ms, app->next_rc_command_ms))
        {
            return;
        }
        const uint32_t oldest_axis_ms =
            time_reached(snapshot.last_valid_ms[RC_INPUT_CH1],
                         snapshot.last_valid_ms[RC_INPUT_CH2]) ?
                snapshot.last_valid_ms[RC_INPUT_CH2] :
                snapshot.last_valid_ms[RC_INPUT_CH1];
        motion_supervisor_submit_rc(
            &app->supervisor,
            snapshot.ch2_normalized * app->config.rc_max_linear_mm_s / 1000.0,
            snapshot.ch1_normalized * app->config.rc_max_angular_mrad_s / 1000.0,
            oldest_axis_ms);
        app->next_rc_command_ms = now_ms + app->config.target_refresh_period_ms;
    }
    else if (app->config.rc_mapping_verified)
    {
        app->supervisor.rc_command.valid = false;
    }
}

static void process_jetson_frame(chassis_app_t *app,
                                 const jetson_frame_t *frame,
                                 uint32_t now_ms)
{
    jetson_motion_frame_t motion;
    uint8_t inspect_command;
    uint8_t self_check_error;
    bool inspect_accepted;
    bool self_check_passed;

    if ((app->machine.state == APP_STATE_INSPECTING) &&
        !time_reached(frame->timestamp_ms, app->inspection_accepted_ms))
    {
        return;
    }
    if ((app->machine.state == APP_STATE_INSPECT_REQUESTED) &&
        !time_reached(frame->timestamp_ms, app->inspect_request_started_ms))
    {
        return;
    }
    if (jetson_protocol_decode_motion(frame, &motion))
    {
        if (app->machine.state != APP_STATE_INSPECTING)
        {
            return;
        }
        motion_supervisor_submit_jetson(&app->supervisor,
                                        (double)motion.vx,
                                        (double)motion.vz,
                                        motion.timestamp_ms);
        return;
    }
    if ((app->machine.state == APP_STATE_BOOT_SAFE) &&
        app->jetson_self_check_request_outstanding &&
        jetson_protocol_decode_self_check(frame,
                                          &self_check_passed,
                                          &self_check_error))
    {
        app->jetson_self_check_received = true;
        app->jetson_self_check_passed = self_check_passed;
        app->jetson_self_check_error = self_check_error;
        app->jetson_outbox.self_check_pending = false;
        app->jetson_self_check_request_outstanding = false;
        return;
    }
    const bool awaiting_reply = app->inspect_request_active &&
        app->inspect_request_sent &&
        (app->machine.state == APP_STATE_INSPECT_REQUESTED) &&
        time_reached(frame->timestamp_ms, app->inspect_request_sent_ms);
    if (jetson_protocol_decode_legacy_inspect_reply(frame,
                                                    &inspect_command,
                                                    &inspect_accepted))
    {
        if ((inspect_command == 1u) && awaiting_reply)
        {
            app->inspect_request_active = false;
            apply_app_event(app,
                            inspect_accepted ? APP_EVENT_JETSON_INSPECT_ACCEPTED :
                                               APP_EVENT_JETSON_INSPECT_REJECTED,
                            now_ms);
            if (inspect_accepted &&
                (app->machine.state == APP_STATE_INSPECTING))
            {
                app->inspection_accepted_ms = frame->timestamp_ms;
                motion_supervisor_submit_jetson(&app->supervisor,
                                                0.0,
                                                0.0,
                                                frame->timestamp_ms);
            }
        }
        else if ((inspect_command == 0u) && inspect_accepted)
        {
            app->inspect_request_active = false;
            apply_app_event(app, APP_EVENT_JETSON_INSPECT_CANCELLED, now_ms);
        }
        return;
    }
    switch (frame->function)
    {
        case JETSON_FUNC_INSPECT_ACCEPTED:
            if (!awaiting_reply)
            {
                break;
            }
            app->inspect_request_active = false;
            apply_app_event(app, APP_EVENT_JETSON_INSPECT_ACCEPTED, now_ms);
            if (app->machine.state == APP_STATE_INSPECTING)
            {
                app->inspection_accepted_ms = frame->timestamp_ms;
                motion_supervisor_submit_jetson(&app->supervisor,
                                                0.0,
                                                0.0,
                                                frame->timestamp_ms);
            }
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

    if (app->inspect_request_active &&
        ((uint32_t)(now_ms - app->inspect_request_started_ms) >=
         app->config.inspect_request_timeout_ms))
    {
        app->inspect_request_active = false;
        apply_app_event(app, APP_EVENT_INSPECT_REQUEST_TIMEOUT, now_ms);
    }
    if ((app->machine.state == APP_STATE_INSPECTING) &&
        app->supervisor.jetson_command.valid &&
        ((uint32_t)(now_ms - app->supervisor.jetson_command.timestamp_ms) >=
         app->config.supervisor.jetson_timeout_ms))
    {
        apply_app_event(app, APP_EVENT_JETSON_MOTION_TIMEOUT, now_ms);
    }
    jetson_uart_transport_poll(&app->jetson_transport, now_ms);
    complete_jetson_tx(app);
    const uint32_t losses = app->jetson_transport.stats.uart_errors +
        app->jetson_transport.stats.software_overruns +
        app->protocol.stats.queue_overflows;
    if ((losses != app->jetson_receive_losses) &&
        ((app->machine.state == APP_STATE_INSPECTING) ||
         (app->machine.state == APP_STATE_INSPECT_REQUESTED)))
    {
        motion_supervisor_set_communication_fault(&app->supervisor, true);
        apply_app_event(app, APP_EVENT_COMMUNICATION_FAULT, now_ms);
    }
    app->jetson_receive_losses = losses;
    while (jetson_protocol_take_frame(&app->protocol, &frame))
    {
        if (!time_reached(now_ms, frame.timestamp_ms))
        {
            frame.timestamp_ms = now_ms;
        }
        process_jetson_frame(app, &frame, now_ms);
    }
}

static void update_supervisor_feedback(chassis_app_t *app, uint32_t now_ms)
{
    mssd_configuration_snapshot_t configuration;
    mssd_motor_status_t right;
    mssd_motor_status_t left;
    const bool configuration_valid =
        mssd_driver_get_configuration(&app->mssd, &configuration);
    /* 500 ms feedback cycle plus one target pair and one status pair on 9600 8N1. */
    const uint32_t freshness_ms = app->config.status_poll_period_ms + 250u;
    const bool wheel = app->config.profile == CHASSIS_STANDALONE_RC;
    const bool right_valid = mssd_driver_get_status(&app->mssd, MSSD_CHANNEL_RIGHT, &right) &&
        (!wheel || (now_ms - app->right_feedback_ms < freshness_ms));
    const bool left_valid = mssd_driver_get_status(&app->mssd, MSSD_CHANNEL_LEFT, &left) &&
        (!wheel || (now_ms - app->left_feedback_ms < freshness_ms));
    const bool configuration_matches = configuration_valid &&
        (configuration.system_mode == 1u) &&
        (configuration.control_source == 0u) &&
        ((app->config.profile != CHASSIS_STANDALONE_RC) ||
         ((configuration.drive_mode == 1u) && (configuration.foc_mode == 1u)));

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
    const bool standalone = app->config.profile == CHASSIS_STANDALONE_RC;
    const bool host_check_required = !standalone && app->config.require_jetson_self_check;
    const bool jetson_ready = !host_check_required ||
                              app->jetson_self_check_passed;

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
    else if (host_check_required &&
             app->jetson_self_check_received &&
             !app->jetson_self_check_passed)
    {
        app->self_check_completed = true;
        apply_app_event(app, APP_EVENT_SELF_CHECK_FAILED, now_ms);
    }
    else if (app->supervisor.mssd_configuration_matches &&
             both_feedback_valid &&
             app->rc_input.snapshot.signal_valid &&
             jetson_ready &&
             (!standalone || ((app->wheel_brake_state == WHEEL_PARKED) &&
                              !app->wheel_recheck_pending &&
                              (app->machine.selected_mode == APP_MODE_SELECTION_STANDBY))) &&
             (app->machine.selected_mode != APP_MODE_SELECTION_INVALID))
    {
        app->self_check_completed = true;
        apply_app_event(app, APP_EVENT_SELF_CHECK_PASSED, now_ms);
    }
}

static void schedule_self_check(chassis_app_t *app, uint32_t now_ms)
{
    if (app->config.require_jetson_self_check &&
        !app->jetson_self_check_received &&
        (app->machine.state == APP_STATE_BOOT_SAFE) &&
        time_reached(now_ms, app->next_self_check_request_ms))
    {
        app->jetson_outbox.self_check_pending = true;
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
    else if ((state == MSSD_DRIVER_FAILED) &&
             (error.code != MSSD_ERROR_ABORTED_FOR_SAFETY))
    {
        motion_supervisor_set_communication_fault(&app->supervisor, true);
        if ((operation == MSSD_OPERATION_READ_CONFIGURATION) ||
            (operation == MSSD_OPERATION_READ_RIGHT_STATUS) ||
            (operation == MSSD_OPERATION_READ_LEFT_STATUS))
        {
            app->read_backoff_active = true;
            app->next_status_poll_ms = now_ms + app->config.status_poll_period_ms;
            if (app->config.profile == CHASSIS_STANDALONE_RC &&
                app->machine.state == APP_STATE_FAULT)
            {
                /* A controller that disappeared while parked may have rebooted. */
                app->wheel_brake_state = WHEEL_STOP_REQUIRED;
                app->fault_reason = MOTION_STOP_COMMUNICATION;
            }
        }
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
        (operation == MSSD_OPERATION_INITIALIZE_CONFIGURATION))
    {
        app->startup_state = CHASSIS_APP_STARTUP_CONFIGURATION;
    }
    else if ((state == MSSD_DRIVER_COMPLETE) &&
        (operation == MSSD_OPERATION_READ_CONFIGURATION))
    {
        app->startup_state = CHASSIS_APP_STARTUP_RIGHT_STATUS;
    }
    else if ((state == MSSD_DRIVER_COMPLETE) &&
             (operation == MSSD_OPERATION_READ_RIGHT_STATUS))
    {
        app->right_feedback_ms = now_ms;
        app->startup_state = CHASSIS_APP_STARTUP_LEFT_STATUS;
    }
    else if ((state == MSSD_DRIVER_COMPLETE) &&
             (operation == MSSD_OPERATION_READ_LEFT_STATUS))
    {
        app->left_feedback_ms = now_ms;
        app->startup_state = CHASSIS_APP_RUNNING;
        if (app->config.profile == CHASSIS_STANDALONE_RC)
        {
            if (app->wheel_brake_state == WHEEL_BRAKE_UNKNOWN)
            {
                app->wheel_brake_state = WHEEL_STOP_REQUIRED;
            }
        }
        else
        {
            app->next_status_poll_ms = now_ms + app->config.status_poll_period_ms;
        }
    }
    if (app->config.profile == CHASSIS_STANDALONE_RC)
    {
        if ((operation == MSSD_OPERATION_STOP_BOTH) ||
            (operation == MSSD_OPERATION_BRAKE_BOTH))
        {
            if (state == MSSD_DRIVER_COMPLETE)
            {
                if (operation == MSSD_OPERATION_STOP_BOTH)
                {
                    app->wheel_brake_state = WHEEL_ENGAGE_REQUIRED;
                    app->diagnostics.left_target_rpm = 0;
                    app->diagnostics.right_target_rpm = 0;
                }
                else if ((app->mssd.brake_intent == MSSD_BRAKE_ENGAGE) &&
                         (app->wheel_brake_state == WHEEL_ENGAGE_REQUIRED))
                {
                    app->wheel_brake_state = WHEEL_PARKED;
                }
                else if (app->wheel_brake_state == WHEEL_RELEASE_PENDING)
                {
                    app->wheel_brake_state = WHEEL_RELEASED;
                }
            }
            else
            {
                app->wheel_brake_state = WHEEL_STOP_REQUIRED;
                if (operation == MSSD_OPERATION_STOP_BOTH ||
                    app->mssd.brake_intent == MSSD_BRAKE_ENGAGE)
                {
                    app->wheel_retry_delayed = true;
                    app->wheel_retry_ms = now_ms + app->config.status_poll_period_ms;
                }
            }
        }
    }
    else if (operation == MSSD_OPERATION_STOP_BOTH)
    {
        const bool upgrade_pending = (state == MSSD_DRIVER_COMPLETE) &&
            (app->inflight_stop_type != app->pending_stop_type);
        app->safe_stop_pending = upgrade_pending;
        app->safe_stop_attempted = !upgrade_pending;
        app->safe_stop_delivered = !upgrade_pending &&
            (state == MSSD_DRIVER_COMPLETE);
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
        if (state != MSSD_DRIVER_COMPLETE)
        {
            app->stop_failure_latched = true;
            apply_app_event(app, APP_EVENT_COMMUNICATION_FAULT, now_ms);
            app->safe_stop_pending = false;
            app->safe_stop_attempted = true;
        }
    }
    else if (app->stop_failure_latched && !app->recovery_stop_attempted &&
             (state == MSSD_DRIVER_COMPLETE) &&
             ((operation == MSSD_OPERATION_READ_RIGHT_STATUS) ||
              (operation == MSSD_OPERATION_READ_LEFT_STATUS)))
    {
        /* A fresh matched status reply proves the link is back. One new
           stop pair is allowed per latched failure; never retry on a timer. */
        app->safe_stop_pending = true;
        app->safe_stop_attempted = false;
        app->pending_stop_type = MSSD_STOP_EMERGENCY;
        app->recovery_stop_attempted = true;
    }
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

    if (app->stop_failure_latched)
    {
        return;
    }
    if (!decision->stop_requested)
    {
        if (!app->safe_stop_pending)
        {
            app->safe_stop_delivered = false;
            app->safe_stop_attempted = false;
        }
        return;
    }
    if (changed && !(app->safe_stop_pending &&
                     (app->pending_stop_type == MSSD_STOP_EMERGENCY) &&
                     (decision->stop_type == MSSD_STOP_NORMAL)))
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

    if (app->read_backoff_active &&
        !time_reached(now_ms, app->next_status_poll_ms))
    {
        return false;
    }
    app->read_backoff_active = false;
    switch (app->startup_state)
    {
        case CHASSIS_APP_STARTUP_WRITE_CONFIGURATION:
            status = mssd_driver_request_initialize_configuration(&app->mssd, now_ms);
            break;
        case CHASSIS_APP_STARTUP_CONFIGURATION:
            status = mssd_driver_request_configuration(&app->mssd, now_ms);
            break;
        case CHASSIS_APP_STARTUP_RIGHT_STATUS:
            status = mssd_driver_request_status(&app->mssd,
                                                MSSD_CHANNEL_RIGHT,
                                                now_ms);
            if (status == MSSD_REQUEST_ACCEPTED && app->config.profile == CHASSIS_STANDALONE_RC)
            {
                app->next_status_poll_ms = now_ms + app->config.status_poll_period_ms;
            }
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
                    if (app->config.profile == CHASSIS_STANDALONE_RC)
                    {
                        app->next_status_poll_ms += app->config.status_poll_period_ms;
                        if (time_reached(now_ms, app->next_status_poll_ms))
                        {
                            app->next_status_poll_ms = now_ms + app->config.status_poll_period_ms;
                        }
                    }
                }
            }
            break;
        default:
            motion_supervisor_set_communication_fault(&app->supervisor, true);
            break;
    }
    return status == MSSD_REQUEST_ACCEPTED;
}

static void schedule_target(chassis_app_t *app,
                            const motion_decision_t *decision, uint32_t now_ms)
{
    chassis_kinematics_output_t output;
    chassis_kinematics_status_t status;

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
        if (app->config.profile == CHASSIS_STANDALONE_RC)
        {
            apply_app_event(app, APP_EVENT_SELF_CHECK_FAILED, now_ms);
        }
        else
        {
            app->safe_stop_pending = true;
            app->safe_stop_delivered = false;
            app->pending_stop_type = MSSD_STOP_EMERGENCY;
            app->pending_stop_reason = MOTION_STOP_KINEMATICS;
        }
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
        if (app->config.profile == CHASSIS_STANDALONE_RC)
        {
            apply_app_event(app, APP_EVENT_COMMUNICATION_FAULT, now_ms);
        }
    }
}

static void schedule_motion(chassis_app_t *app,
                            const motion_decision_t *decision,
                            uint32_t now_ms)
{
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
            app->inflight_stop_type = app->pending_stop_type;
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
    schedule_target(app, decision, now_ms);
}

/* Only the wheel profile owns mechanical brakes and writes the Keil mode block. */
static void schedule_wheel(chassis_app_t *app, const motion_decision_t *decision,
                           uint32_t now_ms)
{
    mssd_request_status_t status = MSSD_REQUEST_ACCEPTED;

    if (app->mssd.state != MSSD_DRIVER_IDLE ||
        (app->wheel_retry_delayed && !time_reached(now_ms, app->wheel_retry_ms)))
    {
        return;
    }
    app->wheel_retry_delayed = false;
    if (app->usart2_adapter.state == USART2_RS485_ADAPTER_STOPPED &&
        !usart2_rs485_adapter_reset(&app->usart2_adapter))
    {
        app->wheel_retry_delayed = true;
        app->wheel_retry_ms = now_ms + app->config.status_poll_period_ms;
        return;
    }
    if (app->wheel_brake_state == WHEEL_STOP_REQUIRED)
    {
        /* Both stop registers received 0 in the original car, including SWC. */
        status = mssd_driver_request_stop(&app->mssd, MSSD_CHANNEL_BOTH,
                                          MSSD_STOP_NORMAL, now_ms);
    }
    else if (app->wheel_brake_state == WHEEL_ENGAGE_REQUIRED)
    {
        status = mssd_driver_request_brake(&app->mssd, MSSD_CHANNEL_BOTH,
                                           MSSD_BRAKE_ENGAGE, now_ms);
    }
    else if (app->machine.state == APP_STATE_FAULT)
    {
        /* Poll the fault's source while parked. Rewriting mode registers on every
           persistent motor fault or readback mismatch would mask its cause. */
        if (app->fault_reason == MOTION_STOP_MOTOR_FAULT &&
            app->startup_state == CHASSIS_APP_STARTUP_LEFT_STATUS)
        {
            status = mssd_driver_request_status(&app->mssd, MSSD_CHANNEL_LEFT, now_ms);
        }
        else if (time_reached(now_ms, app->next_status_poll_ms))
        {
            if (app->fault_reason == MOTION_STOP_CONFIGURATION)
            {
                status = mssd_driver_request_configuration(&app->mssd, now_ms);
            }
            else if (app->fault_reason == MOTION_STOP_MOTOR_FAULT)
            {
                status = mssd_driver_request_status(&app->mssd, MSSD_CHANNEL_RIGHT, now_ms);
            }
            app->next_status_poll_ms = now_ms + app->config.status_poll_period_ms;
        }
    }
    else if (app->machine.state == APP_STATE_EMERGENCY)
    {
        return;
    }
    else if (app->startup_state != CHASSIS_APP_RUNNING ||
             time_reached(now_ms, app->next_status_poll_ms))
    {
        (void)schedule_startup_or_poll(app, now_ms);
    }
    else if (decision->motion_authorized && app->wheel_brake_state == WHEEL_PARKED)
    {
        status = mssd_driver_request_brake(&app->mssd, MSSD_CHANNEL_BOTH,
                                           MSSD_BRAKE_RELEASE, now_ms);
        if (status == MSSD_REQUEST_ACCEPTED)
        {
            app->wheel_brake_state = WHEEL_RELEASE_PENDING;
        }
    }
    else if (decision->motion_authorized && app->wheel_brake_state == WHEEL_RELEASED)
    {
        schedule_target(app, decision, now_ms);
    }
    if (status != MSSD_REQUEST_ACCEPTED)
    {
        apply_app_event(app, APP_EVENT_COMMUNICATION_FAULT, now_ms);
        app->wheel_brake_state = WHEEL_STOP_REQUIRED;
        app->wheel_retry_delayed = true;
        app->wheel_retry_ms = now_ms + app->config.status_poll_period_ms;
    }
}

static bool restart_wheel_self_check(chassis_app_t *app, uint32_t now_ms)
{
    if (app->wheel_brake_state != WHEEL_PARKED || app->mssd.state != MSSD_DRIVER_IDLE)
    {
        return false;
    }
    if (app->machine.state == APP_STATE_FAULT)
    {
        const rc_input_snapshot_t *rc = &app->rc_input.snapshot;
        if (!rc->signal_valid || rc->emergency_asserted || rc->mode != RC_INPUT_MODE_STANDBY ||
            app->usart2_adapter.state != USART2_RS485_ADAPTER_RUNNING)
        {
            return false;
        }
        if ((app->fault_reason == MOTION_STOP_CONFIGURATION &&
             !app->supervisor.mssd_configuration_matches &&
             !app->wheel_configuration_retry_requested) ||
            (app->fault_reason == MOTION_STOP_MOTOR_FAULT &&
             (app->startup_state != CHASSIS_APP_RUNNING ||
              !app->supervisor.right_feedback_valid || !app->supervisor.left_feedback_valid ||
              app->supervisor.right_fault_code != 0u || app->supervisor.left_fault_code != 0u)))
        {
            return false;
        }
        apply_app_event(app, APP_EVENT_WHEEL_FAULT_RECOVERY_IN_STANDBY, now_ms);
        app->wheel_recheck_pending = true;
    }
    if (app->wheel_recheck_pending && app->machine.state == APP_STATE_BOOT_SAFE)
    {
        app->wheel_recheck_pending = false;
        app->self_check_completed = false;
        app->mssd.configuration.valid = false;
        app->mssd.right_status.valid = false;
        app->mssd.left_status.valid = false;
        app->supervisor.communication_fault = false;
        app->supervisor.partial_commit_latched = false;
        app->fault_reason = MOTION_STOP_NONE;
        app->startup_state = CHASSIS_APP_STARTUP_WRITE_CONFIGURATION;
        app->read_backoff_active = false;
        motion_supervisor_clear_commands(&app->supervisor);
        return true;
    }
    return false;
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
    app->diagnostics.jetson_self_check_received =
        app->jetson_self_check_received;
    app->diagnostics.jetson_self_check_passed = app->jetson_self_check_passed;
    app->diagnostics.jetson_self_check_error = app->jetson_self_check_error;
}

void chassis_app_poll(chassis_app_t *app, uint32_t now_ms)
{
    motion_decision_t decision;

    if ((app == NULL) || !app->started)
    {
        return;
    }
    const bool wheel = app->config.profile == CHASSIS_STANDALONE_RC;
    drain_rc_capture(app, now_ms);
    if (app->config.profile == CHASSIS_HOST_INSPECTION)
    {
        drain_jetson(app, now_ms);
        schedule_self_check(app, now_ms);
        service_jetson_tx(app, now_ms);
    }
    /* A ready right-target reply can immediately launch the left target.
       Revoke before handing that reply to the driver. */
    motion_supervisor_set_app_state(&app->supervisor, app->machine.state, 0u);
    update_supervisor_feedback(app, now_ms);
    if ((app->mssd.operation == MSSD_OPERATION_WRITE_TARGET_PAIR) ||
        (wheel && app->mssd.operation == MSSD_OPERATION_BRAKE_BOTH &&
         app->mssd.brake_intent == MSSD_BRAKE_RELEASE))
    {
        decision = motion_supervisor_poll(&app->supervisor, now_ms);
        if (decision.stop_requested || app->safe_stop_pending ||
            (wheel && app->wheel_brake_state == WHEEL_STOP_REQUIRED))
        {
            (void)mssd_driver_abort_for_safety(&app->mssd, now_ms);
        }
    }
    else if (wheel && app->wheel_brake_state == WHEEL_STOP_REQUIRED &&
             app->mssd.operation != MSSD_OPERATION_STOP_BOTH &&
             app->mssd.operation != MSSD_OPERATION_BRAKE_BOTH)
    {
        (void)mssd_driver_abort_for_safety(&app->mssd, now_ms);
    }
    usart2_rs485_adapter_poll(&app->usart2_adapter, now_ms);
    record_terminal_transaction(app, now_ms);
    update_supervisor_feedback(app, now_ms);
    if ((app->machine.state != APP_STATE_FAULT) &&
        ((app->supervisor.right_fault_code != 0u) ||
         (app->supervisor.left_fault_code != 0u)))
    {
        apply_app_event(app, APP_EVENT_MOTOR_FAULT, now_ms);
    }
    const uint32_t adapter_errors = app->usart2_adapter.stats.uart_errors +
        app->usart2_adapter.stats.rx_overruns + app->usart2_adapter.stats.recovery_failures;
    if ((wheel && adapter_errors != app->wheel_adapter_errors) || (!wheel && adapter_errors != 0u))
    {
        motion_supervisor_set_communication_fault(&app->supervisor, true);
    }
    app->wheel_adapter_errors = adapter_errors;
    if ((app->machine.state != APP_STATE_FAULT) &&
        (app->supervisor.communication_fault ||
         app->supervisor.partial_commit_latched))
    {
        apply_app_event(app, APP_EVENT_COMMUNICATION_FAULT, now_ms);
    }
    if (wheel)
    {
        if (app->machine.state == APP_STATE_REMOTE &&
            (!app->supervisor.right_feedback_valid || !app->supervisor.left_feedback_valid))
        {
            apply_app_event(app, APP_EVENT_COMMUNICATION_FAULT, now_ms);
        }
        if (restart_wheel_self_check(app, now_ms))
        {
            update_supervisor_feedback(app, now_ms);
        }
    }
    advance_self_check(app, now_ms);
    motion_supervisor_set_app_state(&app->supervisor, app->machine.state, 0u);
    decision = motion_supervisor_poll(&app->supervisor, now_ms);
    if (wheel && app->machine.state == APP_STATE_REMOTE && !decision.rc_fresh)
    {
        apply_app_event(app, APP_EVENT_RC_SIGNAL_LOST, now_ms);
        decision = motion_supervisor_poll(&app->supervisor, now_ms);
    }
    if (!decision.jetson_fresh &&
        (app->machine.state == APP_STATE_INSPECTING))
    {
        apply_app_event(app, APP_EVENT_JETSON_MOTION_TIMEOUT, now_ms);
        decision = motion_supervisor_poll(&app->supervisor, now_ms);
    }
    if ((app->machine.state == APP_STATE_FAULT) &&
        (app->fault_reason != MOTION_STOP_NONE))
    {
        decision.stop_reason = app->fault_reason;
    }
    if (wheel)
    {
        mssd_driver_set_motion_authorized(&app->mssd, decision.motion_authorized &&
                                          app->wheel_brake_state == WHEEL_RELEASED);
        schedule_wheel(app, &decision, now_ms);
    }
    else
    {
        latch_safe_stop(app, &decision, now_ms);
        mssd_driver_set_motion_authorized(&app->mssd, false);
        if ((app->machine.state == APP_STATE_EMERGENCY) ||
            (app->machine.state == APP_STATE_FAULT) || app->supervisor.communication_fault ||
            (app->startup_state != CHASSIS_APP_STARTUP_CONFIGURATION) ||
            (mssd_driver_get_state(&app->mssd) != MSSD_DRIVER_IDLE))
        {
            schedule_motion(app, &decision, now_ms);
        }
        else
        {
            (void)schedule_startup_or_poll(app, now_ms);
        }
    }
    update_diagnostics(app, &decision);
    const app_state_t indicator_state = wheel && app->rc_input.snapshot.emergency_asserted ?
        APP_STATE_EMERGENCY : app->machine.state;
    board_status_outputs_poll(&app->status_outputs, indicator_state, now_ms);
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
