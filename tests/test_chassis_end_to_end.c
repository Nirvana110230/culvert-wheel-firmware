#include "chassis_app.h"
#include "modbus_rtu.h"
#include "rc_capture.h"
#include "usart.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define CAPTURED_TX_CAPACITY 64u
#define CAPTURED_FRAME_CAPACITY 40u
#define RC_SAMPLE_CAPACITY 64u
#define STARTUP_TRANSACTION_STEP_LIMIT 16u

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

typedef struct
{
    uint8_t bytes[CAPTURED_FRAME_CAPACITY];
    uint16_t length;
    bool jetson;
} captured_tx_t;

typedef struct
{
    chassis_app_t app;
    DMA_HandleTypeDef rx_dma;
    DMA_HandleTypeDef tx_dma;
    GPIO_TypeDef de_port;
    GPIO_TypeDef beep_port;
    uint32_t now_ms;
} fixture_t;

static captured_tx_t captured_tx[CAPTURED_TX_CAPACITY];
static size_t captured_tx_count;
static rc_capture_sample_t rc_samples[RC_SAMPLE_CAPACITY];
static size_t rc_sample_count;
static size_t rc_sample_index;
static uint32_t interrupt_mask;
static unsigned tests_run;

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
    return ((uart != NULL) && (data != NULL) && (size != 0u)) ? HAL_OK : HAL_ERROR;
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
    captured_tx_t *capture;

    if ((uart == NULL) || (data == NULL) || (size == 0u) ||
        (size > CAPTURED_FRAME_CAPACITY) ||
        (captured_tx_count >= CAPTURED_TX_CAPACITY))
    {
        return HAL_ERROR;
    }
    capture = &captured_tx[captured_tx_count++];
    (void)memcpy(capture->bytes, data, size);
    capture->length = size;
    capture->jetson = (uart == &huart1);
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

static void reset_fakes(void)
{
    (void)memset(&huart1, 0, sizeof(huart1));
    (void)memset(&huart2, 0, sizeof(huart2));
    (void)memset(captured_tx, 0, sizeof(captured_tx));
    captured_tx_count = 0u;
    rc_sample_count = 0u;
    rc_sample_index = 0u;
    interrupt_mask = 0u;
}

static bool fixture_start(fixture_t *fixture, chassis_app_config_t config)
{
    reset_fakes();
    (void)memset(fixture, 0, sizeof(*fixture));
    huart2.hdmarx = &fixture->rx_dma;
    huart2.hdmatx = &fixture->tx_dma;
    if (!chassis_app_init(&fixture->app,
                          &config,
                          &huart2,
                          &fixture->rx_dma,
                          &fixture->tx_dma,
                          &fixture->de_port,
                          0x0080u) ||
        !chassis_app_attach_status_outputs(&fixture->app,
                                           &fixture->beep_port,
                                           0x0100u) ||
        !chassis_app_start(&fixture->app))
    {
        return false;
    }
    return true;
}

static void append_crc(uint8_t *frame, size_t payload_length)
{
    const uint16_t crc = modbus_rtu_crc16(frame, payload_length);

    frame[payload_length] = (uint8_t)crc;
    frame[payload_length + 1u] = (uint8_t)(crc >> 8u);
}

static int32_t target_rpm_from_request(const uint8_t *request)
{
    const uint32_t raw = ((uint32_t)request[7] << 24u) |
                         ((uint32_t)request[8] << 16u) |
                         ((uint32_t)request[9] << 8u) |
                         request[10];

    return (int32_t)raw;
}

static void publish_modbus_bytes(fixture_t *fixture,
                                 const uint8_t *data,
                                 size_t length)
{
    usart2_rs485_adapter_t *adapter = &fixture->app.usart2_adapter;
    size_t offset = 0u;

    while (offset < length)
    {
        const size_t start = adapter->rx_buffer.dma_position;
        const size_t available = USART2_RS485_RX_DMA_SIZE - start;
        const size_t chunk = ((length - offset) < available) ?
                                 (length - offset) : available;

        (void)memcpy(&adapter->rx_dma_buffer[start], &data[offset], chunk);
        offset += chunk;
        if ((start + chunk) == USART2_RS485_RX_DMA_SIZE)
        {
            huart2.rx_event_type = HAL_UART_RXEVENT_TC;
            usart2_rs485_adapter_dispatch_rx_event(
                &huart2, USART2_RS485_RX_DMA_SIZE);
        }
        else
        {
            huart2.rx_event_type = HAL_UART_RXEVENT_IDLE;
            usart2_rs485_adapter_dispatch_rx_event(
                &huart2, (uint16_t)(start + chunk));
        }
    }
}

static void publish_jetson_bytes(fixture_t *fixture,
                                 const uint8_t *data,
                                 size_t length)
{
    jetson_uart_transport_t *transport = &fixture->app.jetson_transport;
    size_t offset = 0u;

    while (offset < length)
    {
        const size_t start = transport->dma_position;
        const size_t available = JETSON_UART_DMA_BUFFER_SIZE - start;
        const size_t chunk = ((length - offset) < available) ?
                                 (length - offset) : available;

        (void)memcpy(&transport->rx_dma[start], &data[offset], chunk);
        offset += chunk;
        jetson_uart_transport_on_rx_event(
            transport,
            (uint16_t)(((start + chunk) == JETSON_UART_DMA_BUFFER_SIZE) ?
                           JETSON_UART_DMA_BUFFER_SIZE : (start + chunk)));
    }
}

static bool build_modbus_response(const mssd_driver_t *driver,
                                  uint8_t *response,
                                  size_t *length)
{
    if (driver->operation == MSSD_OPERATION_READ_CONFIGURATION)
    {
        const uint8_t frame[] = {
            0x01u, 0x03u, 0x08u, 0x00u, 0x01u, 0x00u, 0x00u,
            0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u};
        (void)memcpy(response, frame, sizeof(frame));
        append_crc(response, sizeof(frame) - 2u);
        *length = sizeof(frame);
        return true;
    }
    if ((driver->operation == MSSD_OPERATION_READ_RIGHT_STATUS) ||
        (driver->operation == MSSD_OPERATION_READ_LEFT_STATUS))
    {
        const uint8_t frame[] = {
            0x01u, 0x03u, 0x08u, 0x00u, 0x64u, 0x00u, 0x00u,
            0x00u, 0x7Bu, 0x00u, 0x00u, 0x00u, 0x00u};
        (void)memcpy(response, frame, sizeof(frame));
        append_crc(response, sizeof(frame) - 2u);
        *length = sizeof(frame);
        return true;
    }
    if (((driver->request[1] == 0x06u) &&
         (driver->request_length == 8u)) ||
        ((driver->request[1] == 0x10u) &&
         (driver->request_length >= 6u)))
    {
        (void)memcpy(response, driver->request, 6u);
        append_crc(response, 6u);
        *length = 8u;
        return true;
    }
    return false;
}

static bool complete_current_mssd_transaction(fixture_t *fixture)
{
    uint8_t response[MODBUS_RTU_MAX_ADU_SIZE];
    size_t response_length;
    unsigned guard;

    for (guard = 0u; guard < 20u; ++guard)
    {
        if (fixture->app.transport.state == RS485_DMA_TX)
        {
            usart2_rs485_adapter_dispatch_tx_dma_irq(&fixture->tx_dma);
        }
        if (fixture->app.transport.state == RS485_WAIT_UART_TC)
        {
            huart2.tc_flag = UART_FLAG_TC;
        }
        chassis_app_poll(&fixture->app, ++fixture->now_ms);
        if (fixture->app.transport.state == RS485_WAIT_RESPONSE)
        {
            break;
        }
    }
    if ((fixture->app.transport.state != RS485_WAIT_RESPONSE) ||
        !build_modbus_response(&fixture->app.mssd,
                               response,
                               &response_length))
    {
        return false;
    }
    publish_modbus_bytes(fixture, response, response_length);
    chassis_app_poll(&fixture->app, ++fixture->now_ms);
    return true;
}

static bool complete_until_idle(fixture_t *fixture)
{
    unsigned guard;

    for (guard = 0u; guard < 10u; ++guard)
    {
        if (mssd_driver_get_state(&fixture->app.mssd) == MSSD_DRIVER_IDLE)
        {
            return true;
        }
        if (!complete_current_mssd_transaction(fixture))
        {
            return false;
        }
    }
    return false;
}

static bool reach_current_response_wait(fixture_t *fixture)
{
    unsigned guard;

    for (guard = 0u; guard < 20u; ++guard)
    {
        if (fixture->app.transport.state == RS485_DMA_TX)
        {
            usart2_rs485_adapter_dispatch_tx_dma_irq(&fixture->tx_dma);
        }
        if (fixture->app.transport.state == RS485_WAIT_UART_TC)
        {
            huart2.tc_flag = UART_FLAG_TC;
        }
        chassis_app_poll(&fixture->app, ++fixture->now_ms);
        if (fixture->app.transport.state == RS485_WAIT_RESPONSE)
        {
            return true;
        }
    }
    return false;
}

static bool feed_jetson_frame(fixture_t *fixture,
                              uint8_t function,
                              const uint8_t *payload,
                              uint8_t payload_length)
{
    uint8_t frame[JETSON_PROTOCOL_MAX_FRAME_SIZE];
    size_t length;

    if (!jetson_protocol_build(JETSON_PROTOCOL_RX_HEAD,
                               function,
                               payload,
                               payload_length,
                               frame,
                               sizeof(frame),
                               &length))
    {
        return false;
    }
    publish_jetson_bytes(fixture, frame, length);
    chassis_app_poll(&fixture->app, ++fixture->now_ms);
    return true;
}

static void make_application_ready(fixture_t *fixture, app_state_t state)
{
    fixture->app.mssd.configuration.system_mode = 1u;
    fixture->app.mssd.configuration.control_source = 0u;
    fixture->app.mssd.configuration.valid = true;
    fixture->app.mssd.right_status.valid = true;
    fixture->app.mssd.left_status.valid = true;
    fixture->app.startup_state = CHASSIS_APP_RUNNING;
    fixture->app.next_status_poll_ms = fixture->now_ms + 1000u;
    fixture->app.self_check_completed = true;
    fixture->app.jetson_self_check_received = true;
    fixture->app.jetson_self_check_passed = true;
    fixture->app.machine.state = state;
    fixture->app.machine.selected_mode =
        (state == APP_STATE_REMOTE) ? APP_MODE_SELECTION_REMOTE :
                                      APP_MODE_SELECTION_STANDBY;
    fixture->app.safe_stop_attempted = true;
    fixture->app.pending_stop_type = MSSD_STOP_NORMAL;
    fixture->app.pending_stop_reason =
        (state == APP_STATE_STANDBY) ? MOTION_STOP_STANDBY : MOTION_STOP_NONE;
    motion_supervisor_set_app_state(&fixture->app.supervisor, state, 0u);
}

static bool test_startup_reads_configuration_and_both_statuses(void)
{
    fixture_t fixture;
    chassis_app_config_t config = chassis_app_default_config();
    static const uint8_t self_check[] = {1u, 0u};
    bool saw_configuration = false;
    bool saw_right_status = false;
    bool saw_left_status = false;
    unsigned guard;

    CHECK(fixture_start(&fixture, config));
    fixture.app.machine.selected_mode = APP_MODE_SELECTION_STANDBY;
    chassis_app_poll(&fixture.app, fixture.now_ms);
    CHECK(fixture.app.jetson_transport.tx_busy);
    CHECK(fixture.app.jetson_transport.tx_buffer[3] ==
          JETSON_FUNC_LEGACY_VERSION_OR_SELF_CHECK);
    CHECK(feed_jetson_frame(&fixture,
                            JETSON_FUNC_LEGACY_VERSION_OR_SELF_CHECK,
                            self_check,
                            sizeof(self_check)));

    for (guard = 0u; guard < STARTUP_TRANSACTION_STEP_LIMIT; ++guard)
    {
        const mssd_operation_t operation = fixture.app.mssd.operation;
        const mssd_driver_state_t state =
            mssd_driver_get_state(&fixture.app.mssd);

        if (operation == MSSD_OPERATION_READ_CONFIGURATION)
        {
            saw_configuration = true;
        }
        else if (operation == MSSD_OPERATION_READ_RIGHT_STATUS)
        {
            saw_right_status = true;
        }
        else if (operation == MSSD_OPERATION_READ_LEFT_STATUS)
        {
            saw_left_status = true;
        }
        if ((fixture.app.startup_state == CHASSIS_APP_RUNNING) &&
            (state == MSSD_DRIVER_IDLE) && saw_configuration &&
            saw_right_status && saw_left_status)
        {
            break;
        }
        if (state == MSSD_DRIVER_IDLE)
        {
            chassis_app_poll(&fixture.app, ++fixture.now_ms);
        }
        else
        {
            CHECK(complete_current_mssd_transaction(&fixture));
        }
    }
    CHECK(guard < STARTUP_TRANSACTION_STEP_LIMIT);
    CHECK(saw_configuration && saw_right_status && saw_left_status);
    CHECK(fixture.app.mssd.configuration.valid);
    CHECK(fixture.app.mssd.right_status.valid);
    CHECK(fixture.app.mssd.left_status.valid);
    CHECK(fixture.app.machine.state == APP_STATE_STANDBY);
    CHECK(chassis_app_stop(&fixture.app));
    return true;
}

static bool test_inspection_finish_clears_old_speed_and_remote_retakes(void)
{
    fixture_t fixture;
    chassis_app_config_t config = chassis_app_default_config();
    static const uint8_t accepted[] = {1u, 1u};
    static const uint8_t motion[] = {0u, 0xE8u, 0x03u, 0u, 0u, 0u, 0u};
    size_t index;

    config.supervisor.motion_enable = true;
    config.kinematics.direction_mapping_verified = true;
    config.kinematics.left_motor_direction = 1;
    config.kinematics.right_motor_direction = 1;
    config.kinematics.maximum_rpm_verified = true;
    config.kinematics.maximum_motor_rpm = 1000.0;
    config.rc_mapping_verified = true;
    config.rc_max_linear_mm_s = 1000.0;
    config.rc_max_angular_mrad_s = 1000.0;
    CHECK(fixture_start(&fixture, config));
    make_application_ready(&fixture, APP_STATE_STANDBY);

    for (index = 0u; index < 3u; ++index)
    {
        rc_samples[index] =
            (rc_capture_sample_t){RC_INPUT_CH5, 2000u, fixture.now_ms};
    }
    rc_sample_count = 3u;
    chassis_app_poll(&fixture.app, ++fixture.now_ms);
    CHECK(fixture.app.machine.state == APP_STATE_INSPECT_REQUESTED);
    CHECK(fixture.app.jetson_transport.tx_buffer[3] ==
          JETSON_FUNC_LEGACY_INSPECT_CTRL);
    CHECK(fixture.app.jetson_transport.tx_buffer[4] == 1u);

    jetson_uart_transport_dispatch_tx_complete();
    chassis_app_poll(&fixture.app, ++fixture.now_ms);
    CHECK(complete_until_idle(&fixture));
    CHECK(feed_jetson_frame(&fixture,
                            JETSON_FUNC_LEGACY_INSPECT_CTRL,
                            accepted,
                            sizeof(accepted)));
    CHECK(fixture.app.machine.state == APP_STATE_INSPECTING);
    CHECK(fixture.app.mssd.operation == MSSD_OPERATION_WRITE_TARGET_PAIR);
    CHECK(target_rpm_from_request(fixture.app.mssd.request) == 0);
    CHECK(complete_until_idle(&fixture));
    CHECK(feed_jetson_frame(&fixture,
                            JETSON_FUNC_MOTION,
                            motion,
                            sizeof(motion)));
    CHECK(mssd_driver_get_state(&fixture.app.mssd) == MSSD_DRIVER_IDLE);
    fixture.now_ms = fixture.app.next_target_refresh_ms;
    chassis_app_poll(&fixture.app, fixture.now_ms);
    CHECK(fixture.app.mssd.operation == MSSD_OPERATION_WRITE_TARGET_PAIR);
    CHECK(fixture.app.mssd.request[1] == 0x10u);
    CHECK(fixture.app.mssd.request[3] == 0x39u);
    CHECK(target_rpm_from_request(fixture.app.mssd.request) == 492);
    CHECK(complete_until_idle(&fixture));

    CHECK(feed_jetson_frame(&fixture,
                            JETSON_FUNC_INSPECT_FINISHED,
                            NULL,
                            0u));
    CHECK(fixture.app.machine.state == APP_STATE_STANDBY);
    CHECK(!fixture.app.supervisor.jetson_command.valid);
    CHECK(fixture.app.jetson_outbox.inspect_stop_pending == false);

    rc_sample_count = 0u;
    rc_sample_index = 0u;
    for (index = 0u; index < 3u; ++index)
    {
        rc_samples[rc_sample_count++] =
            (rc_capture_sample_t){RC_INPUT_CH6, 1000u, fixture.now_ms};
        rc_samples[rc_sample_count++] =
            (rc_capture_sample_t){RC_INPUT_CH1, 1500u, fixture.now_ms};
        rc_samples[rc_sample_count++] =
            (rc_capture_sample_t){RC_INPUT_CH2, 1500u, fixture.now_ms};
    }
    chassis_app_poll(&fixture.app, ++fixture.now_ms);
    CHECK(fixture.app.machine.state == APP_STATE_REMOTE);
    CHECK(!fixture.app.supervisor.jetson_command.valid);
    CHECK(chassis_app_stop(&fixture.app));
    return true;
}

static bool test_jetson_cancellation_clears_speed_without_echoing_stop(void)
{
    fixture_t fixture;
    chassis_app_config_t config = chassis_app_default_config();
    static const uint8_t motion[] = {0u, 0xF4u, 0x01u, 0u, 0u, 0u, 0u};

    config.supervisor.motion_enable = true;
    config.kinematics.direction_mapping_verified = true;
    config.kinematics.maximum_rpm_verified = true;
    config.kinematics.maximum_motor_rpm = 1000.0;
    CHECK(fixture_start(&fixture, config));
    make_application_ready(&fixture, APP_STATE_INSPECTING);
    CHECK(feed_jetson_frame(&fixture, JETSON_FUNC_MOTION,
                            motion, sizeof(motion)));
    CHECK(fixture.app.supervisor.jetson_command.valid);
    CHECK(feed_jetson_frame(&fixture,
                            JETSON_FUNC_INSPECT_CANCELLED,
                            NULL,
                            0u));
    CHECK(fixture.app.machine.state == APP_STATE_STANDBY);
    CHECK(!fixture.app.supervisor.jetson_command.valid);
    CHECK(!fixture.app.jetson_outbox.inspect_stop_pending);
    CHECK(fixture.app.safe_stop_pending);
    CHECK(mssd_driver_get_state(&fixture.app.mssd) == MSSD_DRIVER_FAILED);
    chassis_app_poll(&fixture.app, ++fixture.now_ms);
    CHECK(fixture.app.mssd.operation == MSSD_OPERATION_STOP_BOTH);
    CHECK(chassis_app_stop(&fixture.app));
    return true;
}

static bool test_rc_loss_motor_fault_and_uart_error_request_safe_stop(void)
{
    fixture_t fixture;
    chassis_app_config_t config = chassis_app_default_config();
    size_t index;

    config.supervisor.motion_enable = true;
    config.kinematics.direction_mapping_verified = true;
    config.kinematics.maximum_rpm_verified = true;
    config.kinematics.maximum_motor_rpm = 1000.0;
    CHECK(fixture_start(&fixture, config));
    make_application_ready(&fixture, APP_STATE_REMOTE);
    for (index = 0u; index < (size_t)RC_INPUT_CHANNEL_COUNT; ++index)
    {
        fixture.app.rc_input.channel_seen[index] = true;
        fixture.app.rc_input.snapshot.channel_valid[index] = true;
        fixture.app.rc_input.snapshot.last_valid_ms[index] = fixture.now_ms;
    }
    fixture.app.rc_input.snapshot.signal_valid = true;
    fixture.app.rc_input.monitoring_started = true;
    motion_supervisor_submit_rc(&fixture.app.supervisor,
                                100.0,
                                0.0,
                                fixture.now_ms);
    fixture.now_ms += RC_INPUT_TIMEOUT_MS + 1u;
    chassis_app_poll(&fixture.app, fixture.now_ms);
    CHECK(fixture.app.machine.state == APP_STATE_FAULT);
    CHECK(fixture.app.pending_stop_type == MSSD_STOP_EMERGENCY);
    CHECK(chassis_app_stop(&fixture.app));

    CHECK(fixture_start(&fixture, config));
    make_application_ready(&fixture, APP_STATE_INSPECTING);
    fixture.app.mssd.right_status.fault_code = 1u;
    motion_supervisor_submit_jetson(&fixture.app.supervisor,
                                    100.0,
                                    0.0,
                                    fixture.now_ms);
    chassis_app_poll(&fixture.app, fixture.now_ms);
    CHECK(fixture.app.pending_stop_reason == MOTION_STOP_MOTOR_FAULT);
    CHECK(fixture.app.mssd.operation == MSSD_OPERATION_STOP_BOTH);
    CHECK(chassis_app_stop(&fixture.app));

    CHECK(fixture_start(&fixture, config));
    make_application_ready(&fixture, APP_STATE_INSPECTING);
    usart2_rs485_adapter_dispatch_uart_error(&huart2);
    chassis_app_poll(&fixture.app, fixture.now_ms);
    CHECK(fixture.app.pending_stop_reason == MOTION_STOP_COMMUNICATION);
    CHECK(fixture.app.mssd.operation == MSSD_OPERATION_STOP_BOTH);
    CHECK(chassis_app_stop(&fixture.app));
    return true;
}

static bool test_partial_commit_latches_fault_and_prioritizes_stop(void)
{
    fixture_t fixture;
    chassis_app_config_t config = chassis_app_default_config();
    uint8_t exception[] = {0x01u, 0x90u, 0x03u, 0x00u, 0x00u};

    config.supervisor.motion_enable = true;
    config.kinematics.direction_mapping_verified = true;
    config.kinematics.maximum_rpm_verified = true;
    config.kinematics.maximum_motor_rpm = 1000.0;
    CHECK(fixture_start(&fixture, config));
    make_application_ready(&fixture, APP_STATE_INSPECTING);
    motion_supervisor_submit_jetson(&fixture.app.supervisor,
                                    500.0,
                                    0.0,
                                    fixture.now_ms);
    chassis_app_poll(&fixture.app, fixture.now_ms);
    CHECK(fixture.app.mssd.operation == MSSD_OPERATION_WRITE_TARGET_PAIR);
    CHECK(complete_current_mssd_transaction(&fixture));
    CHECK(fixture.app.mssd.current_channel == MSSD_CHANNEL_LEFT);
    CHECK(reach_current_response_wait(&fixture));
    append_crc(exception, 3u);
    publish_modbus_bytes(&fixture, exception, sizeof(exception));
    chassis_app_poll(&fixture.app, ++fixture.now_ms);
    CHECK(fixture.app.diagnostics.partial_commit_count == 1u);
    CHECK(fixture.app.pending_stop_reason == MOTION_STOP_PARTIAL_COMMIT);
    CHECK(fixture.app.mssd.operation == MSSD_OPERATION_STOP_BOTH);
    CHECK(chassis_app_stop(&fixture.app));
    return true;
}

static bool test_emergency_and_timeout_preempt_motion(void)
{
    fixture_t fixture;
    chassis_app_config_t config = chassis_app_default_config();
    size_t index;

    config.supervisor.motion_enable = true;
    config.kinematics.direction_mapping_verified = true;
    config.kinematics.maximum_rpm_verified = true;
    config.kinematics.maximum_motor_rpm = 1000.0;
    CHECK(fixture_start(&fixture, config));
    make_application_ready(&fixture, APP_STATE_INSPECTING);
    motion_supervisor_submit_jetson(&fixture.app.supervisor,
                                    500.0,
                                    0.0,
                                    fixture.now_ms);
    chassis_app_poll(&fixture.app, fixture.now_ms);
    CHECK(fixture.app.mssd.operation == MSSD_OPERATION_WRITE_TARGET_PAIR);

    for (index = 0u; index < 1u; ++index)
    {
        rc_samples[index] =
            (rc_capture_sample_t){RC_INPUT_CH7, 2000u, fixture.now_ms + 1u};
    }
    rc_sample_count = 1u;
    chassis_app_poll(&fixture.app, ++fixture.now_ms);
    CHECK(fixture.app.machine.state == APP_STATE_EMERGENCY);
    CHECK(fixture.beep_port.last_state == GPIO_PIN_SET);
    CHECK(fixture.app.safe_stop_pending);
    chassis_app_poll(&fixture.app, ++fixture.now_ms);
    CHECK(fixture.app.mssd.operation == MSSD_OPERATION_STOP_BOTH);
    CHECK(fixture.app.mssd.request[1] == 0x06u);
    CHECK(fixture.app.mssd.request[3] == 0x38u);
    CHECK(chassis_app_stop(&fixture.app));

    CHECK(fixture_start(&fixture, config));
    fixture.now_ms = UINT32_MAX - 100u;
    make_application_ready(&fixture, APP_STATE_INSPECTING);
    motion_supervisor_submit_jetson(&fixture.app.supervisor,
                                    0.0,
                                    0.0,
                                    fixture.now_ms);
    fixture.now_ms += config.supervisor.jetson_timeout_ms;
    for (index = 0u; index < (size_t)RC_INPUT_CHANNEL_COUNT; ++index)
    {
        fixture.app.rc_input.channel_seen[index] = true;
        fixture.app.rc_input.snapshot.channel_valid[index] = true;
        fixture.app.rc_input.snapshot.last_valid_ms[index] = fixture.now_ms;
    }
    fixture.app.rc_input.snapshot.signal_valid = true;
    chassis_app_poll(&fixture.app, fixture.now_ms);
    CHECK(fixture.app.machine.state == APP_STATE_STANDBY);
    CHECK(fixture.app.pending_stop_reason == MOTION_STOP_JETSON_TIMEOUT);
    CHECK(fixture.app.pending_stop_type == MSSD_STOP_EMERGENCY);
    CHECK(chassis_app_stop(&fixture.app));
    return true;
}

static bool test_default_lock_never_builds_nonzero_or_forbidden_writes(void)
{
    fixture_t fixture;
    chassis_app_config_t config = chassis_app_default_config();
    size_t index;

    CHECK(fixture_start(&fixture, config));
    make_application_ready(&fixture, APP_STATE_INSPECTING);
    motion_supervisor_submit_jetson(&fixture.app.supervisor,
                                    1000.0,
                                    1000.0,
                                    fixture.now_ms);
    chassis_app_poll(&fixture.app, fixture.now_ms);
    CHECK(fixture.app.mssd.operation == MSSD_OPERATION_STOP_BOTH);
    CHECK(!mssd_driver_get_motion_authorized(&fixture.app.mssd));

    for (index = 0u; index < captured_tx_count; ++index)
    {
        const captured_tx_t *frame = &captured_tx[index];

        if (!frame->jetson && (frame->length >= 6u))
        {
            const uint16_t address =
                ((uint16_t)frame->bytes[2] << 8u) | frame->bytes[3];
            CHECK(address != 0x003Bu);
            CHECK(address != 0x003Fu);
            CHECK(address != 0x00ABu);
            CHECK(address != 0x00B3u);
            CHECK(address != 0x00C3u);
            CHECK(!((frame->bytes[1] == 0x06u) &&
                    (address >= 0x007Bu) && (address <= 0x007Eu)));
            CHECK(!((frame->bytes[1] == 0x10u) &&
                    (address >= 0x007Bu) && (address <= 0x007Eu)));
            CHECK(!((frame->bytes[1] == 0x10u) &&
                    (address >= 0x0038u) && (address <= 0x003Fu)));
        }
    }
    CHECK(chassis_app_stop(&fixture.app));
    return true;
}

int main(void)
{
    RUN(test_startup_reads_configuration_and_both_statuses);
    RUN(test_inspection_finish_clears_old_speed_and_remote_retakes);
    RUN(test_jetson_cancellation_clears_speed_without_echoing_stop);
    RUN(test_emergency_and_timeout_preempt_motion);
    RUN(test_rc_loss_motor_fault_and_uart_error_request_safe_stop);
    RUN(test_partial_commit_latches_fault_and_prioritizes_stop);
    RUN(test_default_lock_never_builds_nonzero_or_forbidden_writes);
    printf("all %u chassis_end_to_end tests passed\n", tests_run);
    return 0;
}
