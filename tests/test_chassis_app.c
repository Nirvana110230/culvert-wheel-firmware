#include "chassis_app.h"
#include "rc_capture.h"
#include "usart.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

static unsigned tests_run;
static unsigned receive_starts;
static unsigned jetson_transmit_starts;
static uint32_t interrupt_mask;
static rc_capture_sample_t rc_samples[8];
static size_t rc_sample_count;
static size_t rc_sample_index;

#define CHECK(condition)                                                        \
    do                                                                          \
    {                                                                           \
        if (!(condition))                                                       \
        {                                                                       \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); \
            return false;                                                       \
        }                                                                       \
    } while (0)

#define RUN(test)                                                               \
    do                                                                          \
    {                                                                           \
        ++tests_run;                                                            \
        if (!(test)())                                                          \
        {                                                                       \
            return 1;                                                           \
        }                                                                       \
    } while (0)

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart,
                                               uint8_t *data,
                                               uint16_t size)
{
    if ((uart == NULL) || (data == NULL) || (size == 0u))
    {
        return HAL_ERROR;
    }
    ++receive_starts;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *uart)
{
    return (uart != NULL) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
    return (uart != NULL) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart)
{
    return (uart != NULL) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart,
                                        const uint8_t *data,
                                        uint16_t size)
{
    if ((uart == NULL) || (data == NULL) || (size == 0u))
    {
        return HAL_ERROR;
    }
    if (uart == &huart1)
    {
        ++jetson_transmit_starts;
    }
    return HAL_OK;
}

uint32_t HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *uart)
{
    return uart->rx_event_type;
}

uint32_t HAL_DMA_GetError(DMA_HandleTypeDef *dma)
{
    return dma->error;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    (void)pin;
    ++port->writes;
    port->last_state = state;
}

void test_hal_dma_disable_it(DMA_HandleTypeDef *dma, uint32_t interrupt)
{
    (void)dma;
    (void)interrupt;
}

uint32_t test_hal_uart_get_flag(UART_HandleTypeDef *uart, uint32_t flag)
{
    (void)flag;
    return uart->tc_flag;
}

uint32_t __get_PRIMASK(void)
{
    return interrupt_mask;
}

void __disable_irq(void)
{
    interrupt_mask = 1u;
}

void __enable_irq(void)
{
    interrupt_mask = 0u;
}

bool rc_capture_start(void)
{
    return true;
}

void rc_capture_stop(void)
{
}

bool rc_capture_take_sample(rc_capture_sample_t *sample)
{
    if ((sample == NULL) || (rc_sample_index >= rc_sample_count))
    {
        return false;
    }
    *sample = rc_samples[rc_sample_index++];
    return true;
}

static bool test_default_configuration_is_motion_locked(void)
{
    const chassis_app_config_t config = chassis_app_default_config();

    CHECK(!config.kinematics.direction_mapping_verified);
    CHECK(!config.kinematics.maximum_rpm_verified);
    CHECK(!config.supervisor.motion_enable);
    CHECK(!config.rc_mapping_verified);
    CHECK(!config.mssd.right_brake.mapping_valid);
    CHECK(!config.mssd.left_brake.mapping_valid);
    return true;
}

static bool test_startup_first_transaction_is_configuration_read(void)
{
    chassis_app_t app;
    chassis_app_config_t config = chassis_app_default_config();
    DMA_HandleTypeDef rx_dma = {0};
    DMA_HandleTypeDef tx_dma = {0};
    GPIO_TypeDef de_port = {0};
    jetson_frame_t frame = {
        .function = JETSON_FUNC_MOTION,
        .payload = {0u, 0xE8u, 0x03u, 0u, 0u, 0u, 0u},
        .payload_length = 7u,
        .timestamp_ms = 0u,
    };

    (void)memset(&huart1, 0, sizeof(huart1));
    (void)memset(&huart2, 0, sizeof(huart2));
    receive_starts = 0u;
    jetson_transmit_starts = 0u;
    rc_sample_count = 0u;
    rc_sample_index = 0u;
    huart2.hdmarx = &rx_dma;
    huart2.hdmatx = &tx_dma;
    CHECK(chassis_app_init(&app, &config, &huart2, &rx_dma, &tx_dma,
                           &de_port, 0x0080u));
    CHECK(chassis_app_start(&app));
    CHECK(receive_starts == 2u);

    app.machine.state = APP_STATE_INSPECTING;
    app.protocol.queue[0] = frame;
    app.protocol.queue_head = 1u;
    app.protocol.queue_count = 1u;
    chassis_app_poll(&app, 0u);
    CHECK(app.mssd.operation == MSSD_OPERATION_READ_CONFIGURATION);
    CHECK(app.transport.request[1] == 0x03u);
    CHECK(app.transport.request[2] == 0x00u);
    CHECK(app.transport.request[3] == 0x7Bu);
    CHECK(jetson_transmit_starts == 0u);
    CHECK(!mssd_driver_get_motion_authorized(&app.mssd));
    CHECK(chassis_app_stop(&app));
    CHECK(!app.started);
    return true;
}

static bool start_test_app(chassis_app_t *app,
                           chassis_app_config_t *config,
                           DMA_HandleTypeDef *rx_dma,
                           DMA_HandleTypeDef *tx_dma,
                           GPIO_TypeDef *de_port)
{
    (void)memset(&huart1, 0, sizeof(huart1));
    (void)memset(&huart2, 0, sizeof(huart2));
    receive_starts = 0u;
    jetson_transmit_starts = 0u;
    rc_sample_count = 0u;
    rc_sample_index = 0u;
    huart2.hdmarx = rx_dma;
    huart2.hdmatx = tx_dma;
    return chassis_app_init(app, config, &huart2, rx_dma, tx_dma,
                            de_port, 0x0080u) &&
           chassis_app_start(app);
}

static void provide_valid_mssd_state(chassis_app_t *app)
{
    app->mssd.configuration.system_mode = 1u;
    app->mssd.configuration.control_source = 0u;
    app->mssd.configuration.valid = true;
    app->mssd.right_status.valid = true;
    app->mssd.left_status.valid = true;
    app->startup_state = CHASSIS_APP_RUNNING;
    app->next_status_poll_ms = 1000u;
}

static bool test_default_lock_converts_nonzero_intent_to_stop_only(void)
{
    chassis_app_t app;
    chassis_app_config_t config = chassis_app_default_config();
    DMA_HandleTypeDef rx_dma = {0};
    DMA_HandleTypeDef tx_dma = {0};
    GPIO_TypeDef de_port = {0};

    CHECK(start_test_app(&app, &config, &rx_dma, &tx_dma, &de_port));
    provide_valid_mssd_state(&app);
    app.machine.state = APP_STATE_INSPECTING;
    motion_supervisor_set_app_state(&app.supervisor, APP_STATE_INSPECTING, 0u);
    motion_supervisor_submit_jetson(&app.supervisor, 1000.0, 500.0, 10u);
    chassis_app_poll(&app, 10u);

    CHECK(app.mssd.operation == MSSD_OPERATION_STOP_BOTH);
    CHECK(app.mssd.request[1] == 0x06u);
    CHECK(app.mssd.request[2] == 0x00u);
    CHECK(app.mssd.request[3] == 0x38u);
    CHECK(!mssd_driver_get_motion_authorized(&app.mssd));
    CHECK(app.diagnostics.right_target_rpm == 0);
    CHECK(app.diagnostics.left_target_rpm == 0);
    CHECK(chassis_app_stop(&app));
    return true;
}

static bool test_emergency_aborts_target_and_prioritizes_stop(void)
{
    chassis_app_t app;
    chassis_app_config_t config = chassis_app_default_config();
    DMA_HandleTypeDef rx_dma = {0};
    DMA_HandleTypeDef tx_dma = {0};
    GPIO_TypeDef de_port = {0};

    config.supervisor.motion_enable = true;
    config.kinematics.direction_mapping_verified = true;
    config.kinematics.maximum_rpm_verified = true;
    config.kinematics.maximum_motor_rpm = 1000.0;
    CHECK(start_test_app(&app, &config, &rx_dma, &tx_dma, &de_port));
    provide_valid_mssd_state(&app);
    app.machine.state = APP_STATE_REMOTE;
    app.machine.selected_mode = APP_MODE_SELECTION_REMOTE;
    motion_supervisor_set_app_state(&app.supervisor, APP_STATE_REMOTE, 0u);
    motion_supervisor_submit_rc(&app.supervisor, 1000.0, 0.0, 20u);
    chassis_app_poll(&app, 20u);
    CHECK(app.mssd.operation == MSSD_OPERATION_WRITE_TARGET_PAIR);
    CHECK(app.mssd.request[1] == 0x10u);
    CHECK(app.mssd.request[3] == 0x39u);

    app.machine.state = APP_STATE_EMERGENCY;
    chassis_app_poll(&app, 21u);
    CHECK(app.mssd.state == MSSD_DRIVER_FAILED);
    CHECK(app.transport.errors.aborts == 1u);
    chassis_app_poll(&app, 22u);
    CHECK(app.mssd.operation == MSSD_OPERATION_STOP_BOTH);
    CHECK(app.mssd.request[1] == 0x06u);
    CHECK(app.mssd.request[3] == 0x38u);
    CHECK(chassis_app_stop(&app));
    return true;
}

static bool test_standby_ch5_queues_one_inspection_request(void)
{
    chassis_app_t app;
    chassis_app_config_t config = chassis_app_default_config();
    DMA_HandleTypeDef rx_dma = {0};
    DMA_HandleTypeDef tx_dma = {0};
    GPIO_TypeDef de_port = {0};
    size_t index;

    CHECK(start_test_app(&app, &config, &rx_dma, &tx_dma, &de_port));
    app.machine.state = APP_STATE_STANDBY;
    app.machine.selected_mode = APP_MODE_SELECTION_STANDBY;
    motion_supervisor_set_app_state(&app.supervisor, APP_STATE_STANDBY, 0u);
    for (index = 0u; index < 3u; ++index)
    {
        rc_samples[index].channel = RC_INPUT_CH5;
        rc_samples[index].pulse_us = 2000u;
        rc_samples[index].timestamp_ms = (uint32_t)index;
    }
    rc_sample_count = 3u;
    chassis_app_poll(&app, 2u);
    CHECK(app.machine.state == APP_STATE_INSPECT_REQUESTED);
    CHECK(app.jetson_transport.tx_busy);
    CHECK(app.jetson_transport.tx_buffer[0] == 0xFFu);
    CHECK(app.jetson_transport.tx_buffer[1] == 0xFBu);
    CHECK(app.jetson_transport.tx_buffer[3] == JETSON_FUNC_LEGACY_INSPECT_CTRL);
    CHECK(app.jetson_transport.tx_buffer[4] == 1u);
    CHECK(jetson_transmit_starts == 1u);
    CHECK(chassis_app_stop(&app));
    return true;
}

static bool test_boot_waits_for_deployed_jetson_self_check(void)
{
    chassis_app_t app;
    chassis_app_config_t config = chassis_app_default_config();
    DMA_HandleTypeDef rx_dma = {0};
    DMA_HandleTypeDef tx_dma = {0};
    GPIO_TypeDef de_port = {0};
    jetson_frame_t response = {
        .function = JETSON_FUNC_LEGACY_VERSION_OR_SELF_CHECK,
        .payload = {1u, 0u},
        .payload_length = 2u,
        .timestamp_ms = 1u,
    };

    CHECK(start_test_app(&app, &config, &rx_dma, &tx_dma, &de_port));
    provide_valid_mssd_state(&app);
    app.machine.selected_mode = APP_MODE_SELECTION_STANDBY;
    chassis_app_poll(&app, 0u);
    CHECK(app.machine.state == APP_STATE_BOOT_SAFE);
    CHECK(app.jetson_transport.tx_busy);
    CHECK(app.jetson_transport.tx_buffer[3] ==
          JETSON_FUNC_LEGACY_VERSION_OR_SELF_CHECK);
    CHECK(app.jetson_transport.tx_buffer[4] == 0u);

    app.protocol.queue[0] = response;
    app.protocol.queue_head = 1u;
    app.protocol.queue_count = 1u;
    chassis_app_poll(&app, 1u);
    CHECK(app.machine.state == APP_STATE_STANDBY);
    CHECK(app.diagnostics.jetson_self_check_passed);
    CHECK(chassis_app_stop(&app));
    return true;
}

static bool test_unsolicited_self_check_is_ignored_outside_boot(void)
{
    chassis_app_t app;
    chassis_app_config_t config = chassis_app_default_config();
    DMA_HandleTypeDef rx_dma = {0};
    DMA_HandleTypeDef tx_dma = {0};
    GPIO_TypeDef de_port = {0};
    jetson_frame_t response = {
        .function = JETSON_FUNC_LEGACY_VERSION_OR_SELF_CHECK,
        .payload = {1u, 0u},
        .payload_length = 2u,
        .timestamp_ms = 1u,
    };

    CHECK(start_test_app(&app, &config, &rx_dma, &tx_dma, &de_port));
    provide_valid_mssd_state(&app);
    app.machine.state = APP_STATE_STANDBY;
    app.protocol.queue[0] = response;
    app.protocol.queue_head = 1u;
    app.protocol.queue_count = 1u;
    chassis_app_poll(&app, 1u);
    CHECK(!app.jetson_self_check_received);
    CHECK(chassis_app_stop(&app));
    return true;
}

static bool test_inspection_stop_survives_jetson_tx_busy(void)
{
    chassis_app_t app;
    chassis_app_config_t config = chassis_app_default_config();
    DMA_HandleTypeDef rx_dma = {0};
    DMA_HandleTypeDef tx_dma = {0};
    GPIO_TypeDef de_port = {0};
    size_t index;

    CHECK(start_test_app(&app, &config, &rx_dma, &tx_dma, &de_port));
    provide_valid_mssd_state(&app);
    app.machine.state = APP_STATE_INSPECTING;
    app.machine.selected_mode = APP_MODE_SELECTION_STANDBY;
    app.jetson_transport.tx_busy = true;
    for (index = 0u; index < 3u; ++index)
    {
        rc_samples[index].channel = RC_INPUT_CH6;
        rc_samples[index].pulse_us = 1000u;
        rc_samples[index].timestamp_ms = (uint32_t)index;
    }
    rc_sample_count = 3u;
    rc_sample_index = 0u;
    chassis_app_poll(&app, 2u);
    CHECK(app.machine.state == APP_STATE_REMOTE_ARMING);
    CHECK(app.jetson_outbox.inspect_stop_pending);

    jetson_uart_transport_dispatch_tx_complete();
    chassis_app_poll(&app, 3u);
    CHECK(app.jetson_transport.tx_busy);
    CHECK(app.jetson_transport.tx_buffer[3] == JETSON_FUNC_LEGACY_INSPECT_CTRL);
    CHECK(app.jetson_transport.tx_buffer[4] == 0u);
    CHECK(app.jetson_outbox.inspect_stop_pending);
    jetson_uart_transport_dispatch_tx_complete();
    chassis_app_poll(&app, 4u);
    CHECK(!app.jetson_outbox.inspect_stop_pending);
    CHECK(chassis_app_stop(&app));
    return true;
}

static bool test_stop_precedes_new_start_and_retries_after_uart_error(void)
{
    chassis_app_t app;
    chassis_app_config_t config = chassis_app_default_config();
    DMA_HandleTypeDef rx_dma = {0};
    DMA_HandleTypeDef tx_dma = {0};
    GPIO_TypeDef de_port = {0};
    const unsigned starts_before_error = 1u;

    CHECK(start_test_app(&app, &config, &rx_dma, &tx_dma, &de_port));
    provide_valid_mssd_state(&app);
    app.jetson_outbox.inspect_stop_pending = true;
    app.jetson_outbox.inspect_start_pending = true;
    chassis_app_poll(&app, 1u);
    CHECK(app.jetson_transport.tx_buffer[4] == 0u);
    CHECK(app.jetson_outbox.inspect_stop_pending);
    CHECK(app.jetson_outbox.inspect_start_pending);

    jetson_uart_transport_dispatch_uart_error();
    chassis_app_poll(&app, 2u);
    CHECK(jetson_transmit_starts == (starts_before_error + 1u));
    CHECK(app.jetson_transport.tx_busy);
    CHECK(app.jetson_transport.tx_buffer[4] == 0u);
    CHECK(app.jetson_outbox.inspect_stop_pending);

    jetson_uart_transport_dispatch_tx_complete();
    chassis_app_poll(&app, 3u);
    CHECK(!app.jetson_outbox.inspect_stop_pending);
    CHECK(app.jetson_transport.tx_busy);
    CHECK(app.jetson_transport.tx_buffer[4] == 1u);
    CHECK(app.jetson_outbox.inspect_start_pending);
    jetson_uart_transport_dispatch_tx_complete();
    chassis_app_poll(&app, 4u);
    CHECK(!app.jetson_outbox.inspect_start_pending);
    CHECK(chassis_app_stop(&app));
    return true;
}

static bool test_rc_emergency_event_order_and_signal_loss_are_safe(void)
{
    chassis_app_t app;
    chassis_app_config_t config = chassis_app_default_config();
    DMA_HandleTypeDef rx_dma = {0};
    DMA_HandleTypeDef tx_dma = {0};
    GPIO_TypeDef de_port = {0};
    size_t index;

    CHECK(start_test_app(&app, &config, &rx_dma, &tx_dma, &de_port));
    provide_valid_mssd_state(&app);
    app.machine.state = APP_STATE_EMERGENCY;
    app.machine.selected_mode = APP_MODE_SELECTION_STANDBY;
    app.rc_input.snapshot.emergency_asserted = true;
    for (index = 0u; index < (size_t)RC_INPUT_CHANNEL_COUNT; ++index)
    {
        app.rc_input.channel_seen[index] = true;
        app.rc_input.snapshot.channel_valid[index] = true;
        app.rc_input.snapshot.last_valid_ms[index] = 10u;
    }
    for (index = 0u; index < 3u; ++index)
    {
        rc_samples[index] =
            (rc_capture_sample_t){RC_INPUT_CH7, 1000u, 10u};
    }
    rc_samples[3] = (rc_capture_sample_t){RC_INPUT_CH7, 2000u, 10u};
    rc_sample_count = 4u;
    chassis_app_poll(&app, 10u);
    CHECK(app.machine.state == APP_STATE_EMERGENCY);
    CHECK(app.jetson_outbox.emergency_pending);
    CHECK(app.jetson_outbox.emergency_value == 1u);
    CHECK(chassis_app_stop(&app));

    CHECK(start_test_app(&app, &config, &rx_dma, &tx_dma, &de_port));
    provide_valid_mssd_state(&app);
    app.machine.state = APP_STATE_EMERGENCY;
    app.machine.selected_mode = APP_MODE_SELECTION_STANDBY;
    app.rc_input.snapshot.emergency_asserted = true;
    app.rc_input.monitoring_started = true;
    for (index = 0u; index < (size_t)RC_INPUT_CHANNEL_COUNT; ++index)
    {
        app.rc_input.channel_seen[index] = true;
        app.rc_input.snapshot.channel_valid[index] = true;
        app.rc_input.snapshot.last_valid_ms[index] = 0u;
    }
    for (index = 0u; index < 3u; ++index)
    {
        rc_samples[index] =
            (rc_capture_sample_t){RC_INPUT_CH7, 1000u, 200u};
    }
    rc_sample_count = 3u;
    chassis_app_poll(&app, 200u);
    CHECK(app.machine.state == APP_STATE_EMERGENCY);
    CHECK(!app.jetson_outbox.emergency_pending);
    CHECK(chassis_app_stop(&app));
    return true;
}

static bool test_acceptance_starts_with_fresh_zero_speed_grace(void)
{
    chassis_app_t app;
    chassis_app_config_t config = chassis_app_default_config();
    DMA_HandleTypeDef rx_dma = {0};
    DMA_HandleTypeDef tx_dma = {0};
    GPIO_TypeDef de_port = {0};
    jetson_frame_t accepted = {
        .function = JETSON_FUNC_LEGACY_INSPECT_CTRL,
        .payload = {1u, 1u},
        .payload_length = 2u,
        .timestamp_ms = 10u,
    };

    config.supervisor.motion_enable = true;
    config.kinematics.direction_mapping_verified = true;
    config.kinematics.maximum_rpm_verified = true;
    config.kinematics.maximum_motor_rpm = 1000.0;
    CHECK(start_test_app(&app, &config, &rx_dma, &tx_dma, &de_port));
    provide_valid_mssd_state(&app);
    app.machine.state = APP_STATE_INSPECT_REQUESTED;
    app.machine.selected_mode = APP_MODE_SELECTION_STANDBY;
    app.inspect_request_active = true;
    app.safe_stop_attempted = true;
    motion_supervisor_set_app_state(&app.supervisor,
                                    APP_STATE_INSPECT_REQUESTED,
                                    0u);
    app.protocol.queue[0] = accepted;
    app.protocol.queue_head = 1u;
    app.protocol.queue_count = 1u;

    chassis_app_poll(&app, 10u);
    CHECK(app.machine.state == APP_STATE_INSPECTING);
    CHECK(app.supervisor.jetson_command.valid);
    CHECK(app.supervisor.jetson_command.linear_mm_s == 0.0);
    CHECK(app.supervisor.jetson_command.angular_mrad_s == 0.0);
    CHECK(app.supervisor.jetson_command.timestamp_ms == 10u);
    CHECK(chassis_app_stop(&app));
    return true;
}

int main(void)
{
    RUN(test_default_configuration_is_motion_locked);
    RUN(test_startup_first_transaction_is_configuration_read);
    RUN(test_default_lock_converts_nonzero_intent_to_stop_only);
    RUN(test_emergency_aborts_target_and_prioritizes_stop);
    RUN(test_standby_ch5_queues_one_inspection_request);
    RUN(test_boot_waits_for_deployed_jetson_self_check);
    RUN(test_unsolicited_self_check_is_ignored_outside_boot);
    RUN(test_inspection_stop_survives_jetson_tx_busy);
    RUN(test_stop_precedes_new_start_and_retries_after_uart_error);
    RUN(test_rc_emergency_event_order_and_signal_loss_are_safe);
    RUN(test_acceptance_starts_with_fresh_zero_speed_grace);
    printf("all %u chassis_app tests passed\n", tests_run);
    return 0;
}
