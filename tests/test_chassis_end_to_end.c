#include "chassis_app.h"
#include "modbus_rtu.h"
#include "rc_capture.h"
#include "usart.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define CAPTURED_TX_CAPACITY 512u
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
static bool fail_next_mssd_receive;

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
    if (uart == &huart2 && fail_next_mssd_receive)
    {
        fail_next_mssd_receive = false;
        return HAL_ERROR;
    }
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

uint32_t HAL_GetTick(void)
{
    return 0u; /* This fixture publishes RX with an explicit capture tick. */
}

void __disable_irq(void)
{
    interrupt_mask = 1u;
}

void __enable_irq(void)
{
    interrupt_mask = 0u;
}

bool rc_capture_start(bool queue_inspection_channel)
{
    (void)queue_inspection_channel;
    return true;
}

void rc_capture_stop(void)
{
}

void rc_capture_read_snapshot(rc_capture_snapshot_t *snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
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
    fail_next_mssd_receive = false;
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
                &huart2, USART2_RS485_RX_DMA_SIZE, fixture->now_ms);
        }
        else
        {
            huart2.rx_event_type = HAL_UART_RXEVENT_IDLE;
            usart2_rs485_adapter_dispatch_rx_event(
                &huart2, (uint16_t)(start + chunk), fixture->now_ms);
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
                           JETSON_UART_DMA_BUFFER_SIZE : (start + chunk)),
            fixture->now_ms);
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
            0x00u, 0x01u, 0x00u, 0x01u, 0x00u, 0x00u};
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
    for (unsigned i = 0u; i < RC_INPUT_CHANNEL_COUNT; ++i)
    {
        const uint16_t pulse = (i == RC_INPUT_CH6) ? 2000u :
            ((i == RC_INPUT_CH7) ? 1000u : 1500u);
        rc_samples[rc_sample_count++] =
            (rc_capture_sample_t){(rc_input_channel_t)i, pulse, fixture.now_ms};
    }
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

typedef struct
{
    uint16_t steering;
    uint16_t throttle;
    uint16_t mode;
    uint16_t emergency;
    bool bus_online;
    bool rc_online;
    bool wrong_readback;
    bool suppress_status;
    bool reject_left_release;
    bool reject_right_stop;
    bool reject_left_engage;
    uint16_t right_fault;
    int32_t right_target;
    int32_t left_target;
    unsigned configuration_writes;
    unsigned configuration_reads;
    unsigned right_reads;
    unsigned left_reads;
    uint32_t last_status_ms[2];
    uint32_t max_status_gap_ms[2];
} wheel_bus_t;

static wheel_bus_t wheel_bus_initial(void)
{
    return (wheel_bus_t){.steering = 1500u, .throttle = 1500u, .mode = 2000u,
                         .emergency = 1000u, .bus_online = true, .rc_online = true};
}

static uint16_t frame_address(const uint8_t *frame)
{
    return ((uint16_t)frame[2] << 8u) | frame[3];
}

static bool wheel_tick(fixture_t *fixture, wheel_bus_t *bus)
{
    chassis_app_t *app = &fixture->app;
    rs485_transport_t *transport = &app->transport;
    const uint32_t now = ++fixture->now_ms;
    const uint32_t elapsed = now - transport->state_enter_ms;
    if (bus->rc_online && now % 20u == 1u)
    {
        rc_sample_index = 0u;
        rc_sample_count = 4u;
        rc_samples[0] = (rc_capture_sample_t){RC_INPUT_CH7, bus->emergency, now};
        rc_samples[1] = (rc_capture_sample_t){RC_INPUT_CH6, bus->mode, now};
        rc_samples[2] = (rc_capture_sample_t){RC_INPUT_CH1, bus->steering, now};
        rc_samples[3] = (rc_capture_sample_t){RC_INPUT_CH2, bus->throttle, now};
    }
    /* 8N1: ten wire bits per byte. Let DMA and the returning frame occupy the bus. */
    if (transport->state == RS485_DMA_TX &&
        elapsed >= (transport->request_length * 10000u + 9599u) / 9600u)
    {
        usart2_rs485_adapter_dispatch_tx_dma_irq(&fixture->tx_dma);
    }
    else if (transport->state == RS485_WAIT_UART_TC)
    {
        huart2.tc_flag = UART_FLAG_TC;
    }
    if (bus->bus_online && transport->state == RS485_WAIT_RESPONSE)
    {
        const uint8_t *request = app->mssd.request;
        const uint16_t address = frame_address(request);
        uint8_t response[MODBUS_RTU_MAX_ADU_SIZE];
        size_t length;
        CHECK(build_modbus_response(&app->mssd, response, &length));
        const bool status_read = request[1] == 0x03u && address != MSSD_REG_SYSTEM_MODE;
        if (!(bus->suppress_status && status_read) &&
            elapsed >= 4u + (length * 10000u + 9599u) / 9600u)
        {
            if (address == MSSD_REG_SYSTEM_MODE && request[1] == 0x10u)
            {
                const uint8_t expected[] = {1u, 0x10u, 0u, 0x7bu, 0u, 4u, 8u,
                                            0u, 1u, 0u, 0u, 0u, 1u, 0u, 1u};
                CHECK(app->mssd.request_length == sizeof(expected) + 2u);
                CHECK(memcmp(request, expected, sizeof(expected)) == 0);
                ++bus->configuration_writes;
            }
            else if (address == MSSD_REG_SYSTEM_MODE && request[1] == 0x03u)
            {
                CHECK(bus->configuration_writes > 0u);
                ++bus->configuration_reads;
                if (bus->wrong_readback)
                {
                    response[10] = 0u;
                }
            }
            else if (request[1] == 0x03u)
            {
                CHECK(bus->configuration_reads > 0u);
                const bool right = address == MSSD_REG_RIGHT_PHASE_CURRENT;
                CHECK(right || address == MSSD_REG_LEFT_PHASE_CURRENT);
                const uint32_t rpm = (uint32_t)(right ? bus->right_target : bus->left_target);
                const size_t side = right ? 0u : 1u;
                const uint32_t gap = now - bus->last_status_ms[side];
                if (bus->last_status_ms[side] != 0u && gap > bus->max_status_gap_ms[side])
                {
                    bus->max_status_gap_ms[side] = gap;
                }
                bus->last_status_ms[side] = now;
                response[5] = (uint8_t)(rpm >> 24u);
                response[6] = (uint8_t)(rpm >> 16u);
                response[7] = (uint8_t)(rpm >> 8u);
                response[8] = (uint8_t)rpm;
                response[10] = right ? (uint8_t)bus->right_fault : 0u;
                if (right) ++bus->right_reads;
                else ++bus->left_reads;
            }
            else if (request[1] == 0x10u)
            {
                CHECK(app->wheel_brake_state == WHEEL_RELEASED);
                CHECK(app->mssd.motion_authorized);
                if (address == MSSD_REG_RIGHT_TARGET_HIGH)
                    bus->right_target = target_rpm_from_request(request);
                else
                {
                    CHECK(address == MSSD_REG_LEFT_TARGET_HIGH);
                    bus->left_target = target_rpm_from_request(request);
                }
            }
            else if ((bus->reject_left_release && address == MSSD_REG_LEFT_BRAKE && request[5] == 0u) ||
                     (bus->reject_left_engage && address == MSSD_REG_LEFT_BRAKE && request[5] == 1u) ||
                     (bus->reject_right_stop && address == MSSD_REG_RIGHT_STOP))
            {
                response[1] = 0x86u;
                response[2] = 0x03u;
                length = 5u;
            }
            else if (address == MSSD_REG_RIGHT_STOP || address == MSSD_REG_LEFT_STOP)
            {
                CHECK(request[4] == 0u && request[5] == 0u);
                if (address == MSSD_REG_RIGHT_STOP) bus->right_target = 0;
                else bus->left_target = 0;
            }
            append_crc(response, length - 2u);
            publish_modbus_bytes(fixture, response, length);
        }
    }
    chassis_app_poll(app, now);
    return true;
}

static bool wheel_run(fixture_t *fixture, wheel_bus_t *bus, unsigned ms)
{
    for (unsigned i = 0u; i < ms; ++i)
    {
        CHECK(wheel_tick(fixture, bus));
    }
    return true;
}

static bool saw_command(size_t begin, uint16_t address, uint8_t function, int32_t value)
{
    for (size_t i = begin; i < captured_tx_count; ++i)
    {
        const uint8_t *frame = captured_tx[i].bytes;
        if (!captured_tx[i].jetson && frame_address(frame) == address && frame[1] == function &&
            (function == 0x10u ? target_rpm_from_request(frame) : (int32_t)frame[5]) == value)
        {
            return true;
        }
    }
    return false;
}

static bool saw_parking_sequence(size_t begin)
{
    const uint16_t address[] = {MSSD_REG_RIGHT_STOP, MSSD_REG_LEFT_STOP,
                                MSSD_REG_RIGHT_BRAKE, MSSD_REG_LEFT_BRAKE};
    size_t step = 0u;
    for (size_t i = begin; i < captured_tx_count && step < 4u; ++i)
    {
        const uint8_t *frame = captured_tx[i].bytes;
        if (!captured_tx[i].jetson && frame[1] == 0x06u &&
            frame_address(frame) == address[step] && frame[5] == (step >= 2u))
        {
            ++step;
        }
    }
    return step == 4u;
}

static bool test_standalone_rc_without_host(void)
{
    fixture_t fixture;
    wheel_bus_t bus = wheel_bus_initial();
    bus.mode = 1000u;
    bus.throttle = 2000u;
    CHECK(fixture_start(&fixture, chassis_app_wheel_rc_config()));
    CHECK(wheel_run(&fixture, &bus, 350u));
    CHECK(bus.configuration_writes == 1u && bus.configuration_reads == 1u);
    CHECK(bus.right_reads == 1u && bus.left_reads == 1u);
    CHECK(fixture.app.machine.state == APP_STATE_BOOT_SAFE);
    CHECK(saw_parking_sequence(0u));
    bus.mode = 2000u;
    CHECK(wheel_run(&fixture, &bus, 80u));
    CHECK(fixture.app.machine.state == APP_STATE_STANDBY);
    const uint8_t old_ack[] = {1u, 1u};
    const uint8_t old_motion[] = {0u, 0xe8u, 3u, 0u, 0u, 0u, 0u};
    CHECK(feed_jetson_frame(&fixture, JETSON_FUNC_LEGACY_INSPECT_CTRL, old_ack, sizeof(old_ack)));
    CHECK(feed_jetson_frame(&fixture, JETSON_FUNC_MOTION, old_motion, sizeof(old_motion)));
    CHECK(!fixture.app.supervisor.jetson_command.valid && !fixture.app.inspect_request_active);
    bus.mode = 1000u;
    CHECK(wheel_run(&fixture, &bus, 100u));
    CHECK(fixture.app.machine.state == APP_STATE_REMOTE_ARMING);
    CHECK(!saw_command(0u, MSSD_REG_RIGHT_TARGET_HIGH, 0x10u, 244));
    bus.throttle = 1500u;
    CHECK(wheel_run(&fixture, &bus, 240u));
    CHECK(fixture.app.machine.state == APP_STATE_REMOTE);
    CHECK(fixture.app.wheel_brake_state == WHEEL_RELEASED);
    CHECK(saw_command(0u, MSSD_REG_RIGHT_BRAKE, 0x06u, 0));
    CHECK(saw_command(0u, MSSD_REG_LEFT_BRAKE, 0x06u, 0));
    size_t begin = captured_tx_count;
    /* Inject an axis edge between control ticks without delaying switch handling. */
    while (fixture.app.next_rc_command_ms - fixture.now_ms < 10u)
    {
        CHECK(wheel_tick(&fixture, &bus));
    }
    const uint32_t due = fixture.app.next_rc_command_ms;
    rc_sample_index = 0u;
    rc_sample_count = 1u;
    rc_samples[0] = (rc_capture_sample_t){RC_INPUT_CH2, 2000u, fixture.now_ms + 1u};
    CHECK(wheel_tick(&fixture, &bus));
    CHECK(fixture.app.supervisor.rc_command.linear_mm_s == 0.0 && fixture.now_ms < due);
    bus.throttle = 2000u;
    CHECK(wheel_run(&fixture, &bus, 1600u));
    CHECK(bus.right_target == 244 && bus.left_target == 244);
    CHECK(fixture.app.diagnostics.right_feedback_rpm == 244 && fixture.app.diagnostics.left_feedback_rpm == 244);
    CHECK(bus.right_reads >= 5u && bus.left_reads >= 5u);
    CHECK(bus.max_status_gap_ms[0] <= 570u && bus.max_status_gap_ms[1] <= 570u);
    CHECK(!saw_command(begin, MSSD_REG_RIGHT_BRAKE, 0x06u, 1));
    bus.steering = 1000u;
    CHECK(wheel_run(&fixture, &bus, 250u));
    CHECK(bus.right_target == 426 && bus.left_target == 57);
    begin = captured_tx_count;
    bus.steering = 1500u;
    bus.throttle = 1500u;
    CHECK(wheel_run(&fixture, &bus, 250u));
    CHECK(saw_command(begin, MSSD_REG_RIGHT_TARGET_HIGH, 0x10u, 0));
    CHECK(saw_command(begin, MSSD_REG_LEFT_TARGET_HIGH, 0x10u, 0));
    CHECK(!saw_command(begin, MSSD_REG_RIGHT_BRAKE, 0x06u, 1));
    CHECK(!saw_command(begin, MSSD_REG_LEFT_BRAKE, 0x06u, 1));
    bus.throttle = 2000u;
    CHECK(wheel_run(&fixture, &bus, 150u));
    begin = captured_tx_count;
    bus.emergency = 2000u;
    CHECK(wheel_run(&fixture, &bus, 300u));
    CHECK(saw_parking_sequence(begin));
    CHECK(bus.right_target == 0 && bus.left_target == 0);
    CHECK(fixture.app.machine.state == APP_STATE_EMERGENCY || fixture.app.machine.state == APP_STATE_FAULT);
    CHECK(!fixture.app.diagnostics.motion_authorized);
    bus.emergency = 1000u;
    CHECK(wheel_run(&fixture, &bus, 150u));
    CHECK(!fixture.app.diagnostics.motion_authorized);
    bus.mode = 2000u;
    CHECK(wheel_run(&fixture, &bus, 460u));
    CHECK(fixture.app.machine.state == APP_STATE_STANDBY);
    CHECK(bus.configuration_writes == 2u && bus.configuration_reads == 2u);
    bus.mode = 1000u;
    CHECK(wheel_run(&fixture, &bus, 120u));
    CHECK(fixture.app.machine.state == APP_STATE_REMOTE_ARMING);
    bus.throttle = 1500u;
    CHECK(wheel_run(&fixture, &bus, 220u));
    bus.throttle = 1000u;
    CHECK(wheel_run(&fixture, &bus, 220u));
    CHECK(bus.right_target == -244 && bus.left_target == -244);
    begin = captured_tx_count;
    bus.mode = 2000u;
    CHECK(wheel_run(&fixture, &bus, 300u));
    CHECK(saw_parking_sequence(begin));
    for (size_t i = 0u; i < captured_tx_count; ++i) CHECK(!captured_tx[i].jetson);
    CHECK(chassis_app_stop(&fixture.app));
    return true;
}

static bool test_wheel_profile_rejects_wrong_drive_configuration(void)
{
    fixture_t fixture;
    wheel_bus_t bus = wheel_bus_initial();
    bus.wrong_readback = true;
    CHECK(fixture_start(&fixture, chassis_app_wheel_rc_config()));
    CHECK(wheel_run(&fixture, &bus, 160u));
    CHECK(bus.configuration_writes > 0 && bus.configuration_reads > 0);
    CHECK(fixture.app.machine.state == APP_STATE_FAULT);
    CHECK(!fixture.app.diagnostics.motion_authorized);
    CHECK(wheel_run(&fixture, &bus, 1400u));
    CHECK(bus.configuration_writes == 1u && bus.configuration_reads >= 3u);
    bus.wrong_readback = false;
    CHECK(wheel_run(&fixture, &bus, 800u));
    CHECK(fixture.app.machine.state == APP_STATE_STANDBY);
    CHECK(chassis_app_stop(&fixture.app));
    return true;
}

static bool test_wheel_brake_failures_and_late_controller(void)
{
    fixture_t fixture;
    wheel_bus_t bus = wheel_bus_initial();
    bus.bus_online = false;
    CHECK(fixture_start(&fixture, chassis_app_wheel_rc_config()));
    CHECK(wheel_run(&fixture, &bus, 500u));
    CHECK(fixture.app.machine.state == APP_STATE_FAULT);
    CHECK(!fixture.app.diagnostics.motion_authorized && fixture.app.diagnostics.communication_timeouts > 0u);
    bus.bus_online = true;
    bus.reject_right_stop = true;
    CHECK(wheel_run(&fixture, &bus, 850u));
    CHECK(fixture.app.wheel_brake_state == WHEEL_STOP_REQUIRED);
    CHECK(saw_command(0u, MSSD_REG_LEFT_STOP, 0x06u, 0));
    CHECK(!saw_command(0u, MSSD_REG_LEFT_BRAKE, 0x06u, 1));
    bus.reject_right_stop = false;
    bus.reject_left_engage = true;
    CHECK(wheel_run(&fixture, &bus, 800u));
    CHECK(fixture.app.machine.state == APP_STATE_FAULT);
    CHECK(!saw_command(0u, MSSD_REG_RIGHT_BRAKE, 0x06u, 0));
    bus.reject_left_engage = false;
    CHECK(wheel_run(&fixture, &bus, 1100u));
    CHECK(fixture.app.machine.state == APP_STATE_STANDBY);
    CHECK(bus.configuration_writes == 1u && bus.configuration_reads == 1u);
    bus.mode = 1000u;
    bus.reject_left_release = true;
    CHECK(wheel_run(&fixture, &bus, 350u));
    CHECK(fixture.app.machine.state == APP_STATE_FAULT);
    CHECK(fixture.app.fault_reason == MOTION_STOP_PARTIAL_COMMIT);
    CHECK(fixture.app.wheel_brake_state == WHEEL_PARKED);
    CHECK(!saw_command(0u, MSSD_REG_RIGHT_TARGET_HIGH, 0x10u, 0));
    CHECK(bus.left_target == 0 && bus.right_target == 0);
    CHECK(chassis_app_stop(&fixture.app));
    return true;
}

static bool test_wheel_fault_feedback_rc_and_uart_recovery(void)
{
    fixture_t fixture;
    wheel_bus_t bus = wheel_bus_initial();
    CHECK(fixture_start(&fixture, chassis_app_wheel_rc_config()));
    CHECK(wheel_run(&fixture, &bus, 350u));
    bus.mode = 1000u;
    CHECK(wheel_run(&fixture, &bus, 350u));
    bus.throttle = 2000u;
    CHECK(wheel_run(&fixture, &bus, 180u));
    CHECK(bus.right_target == 244 && bus.left_target == 244);
    size_t begin = captured_tx_count;
    bus.right_fault = 2u;
    CHECK(wheel_run(&fixture, &bus, 850u));
    CHECK(fixture.app.machine.state == APP_STATE_FAULT && saw_parking_sequence(begin));
    bus.mode = 2000u;
    CHECK(wheel_run(&fixture, &bus, 1500u));
    CHECK(fixture.app.machine.state == APP_STATE_FAULT && bus.configuration_writes == 1u);
    bus.right_fault = 0u;
    CHECK(wheel_run(&fixture, &bus, 800u));
    CHECK(fixture.app.machine.state == APP_STATE_STANDBY && bus.configuration_writes == 2u);
    bus.mode = 1000u;
    CHECK(wheel_run(&fixture, &bus, 100u));
    CHECK(fixture.app.machine.state == APP_STATE_REMOTE_ARMING && bus.right_target == 0);
    bus.throttle = 1500u;
    CHECK(wheel_run(&fixture, &bus, 250u));
    bus.throttle = 2000u;
    CHECK(wheel_run(&fixture, &bus, 180u));
    CHECK(bus.right_target == 244 && bus.left_target == 244);
    begin = captured_tx_count;
    bus.rc_online = false;
    CHECK(wheel_run(&fixture, &bus, 350u));
    CHECK(fixture.app.machine.state == APP_STATE_FAULT && saw_parking_sequence(begin));
    bus.rc_online = true;
    CHECK(wheel_run(&fixture, &bus, 200u));
    CHECK(fixture.app.machine.state == APP_STATE_FAULT && !fixture.app.diagnostics.motion_authorized);
    bus.mode = 2000u;
    CHECK(wheel_run(&fixture, &bus, 450u));
    CHECK(fixture.app.machine.state == APP_STATE_STANDBY);
    fail_next_mssd_receive = true;
    usart2_rs485_adapter_dispatch_uart_error(&huart2);
    CHECK(wheel_run(&fixture, &bus, 1000u));
    CHECK(fixture.app.usart2_adapter.stats.recovery_failures == 1u);
    CHECK(fixture.app.usart2_adapter.state == USART2_RS485_ADAPTER_RUNNING);
    CHECK(fixture.app.machine.state == APP_STATE_STANDBY && bus.configuration_writes == 4u);
    CHECK(chassis_app_stop(&fixture.app));
    return true;
}

static bool test_wheel_swc_preempts_right_release_reply(void)
{
    fixture_t fixture;
    wheel_bus_t bus = wheel_bus_initial();
    CHECK(fixture_start(&fixture, chassis_app_wheel_rc_config()));
    CHECK(wheel_run(&fixture, &bus, 350u));
    bus.mode = 1000u;
    for (unsigned i = 0u; i < 300u; ++i)
    {
        CHECK(wheel_tick(&fixture, &bus));
        if (fixture.app.wheel_brake_state == WHEEL_RELEASE_PENDING &&
            fixture.app.transport.state == RS485_WAIT_RESPONSE)
        {
            break;
        }
    }
    CHECK(fixture.app.mssd.current_channel == MSSD_CHANNEL_RIGHT);
    CHECK(fixture.app.wheel_brake_state == WHEEL_RELEASE_PENDING);
    CHECK(fixture.app.transport.state == RS485_WAIT_RESPONSE);
    uint8_t ack[MODBUS_RTU_MAX_ADU_SIZE];
    size_t length;
    CHECK(build_modbus_response(&fixture.app.mssd, ack, &length));
    publish_modbus_bytes(&fixture, ack, length);
    rc_sample_index = 0u;
    rc_sample_count = 1u;
    rc_samples[0] = (rc_capture_sample_t){RC_INPUT_CH7, 2000u, ++fixture.now_ms};
    size_t begin = captured_tx_count;
    chassis_app_poll(&fixture.app, fixture.now_ms);
    CHECK(fixture.app.machine.state == APP_STATE_EMERGENCY);
    bus.emergency = 2000u;
    CHECK(wheel_run(&fixture, &bus, 250u));
    CHECK(saw_parking_sequence(begin));
    CHECK(!saw_command(begin, MSSD_REG_LEFT_BRAKE, 0x06u, 0));
    CHECK(!saw_command(begin, MSSD_REG_RIGHT_TARGET_HIGH, 0x10u, 0));
    CHECK(chassis_app_stop(&fixture.app));
    return true;
}

static bool test_wheel_stale_feedback_preempts_status_retries(void)
{
    fixture_t fixture;
    wheel_bus_t bus = wheel_bus_initial();
    /* Stretch the status timeout to test the independent age limit. */
    chassis_app_config_t config = chassis_app_wheel_rc_config();
    config.rs485_timeouts.response_ms = 1000u;
    CHECK(fixture_start(&fixture, config));
    CHECK(wheel_run(&fixture, &bus, 350u));
    bus.mode = 1000u;
    CHECK(wheel_run(&fixture, &bus, 450u));
    bus.throttle = 2000u;
    CHECK(wheel_run(&fixture, &bus, 160u));
    CHECK(bus.right_target == 244 && bus.left_target == 244);
    const uint32_t oldest = bus.last_status_ms[0] < bus.last_status_ms[1] ?
        bus.last_status_ms[0] : bus.last_status_ms[1];
    bus.suppress_status = true;
    CHECK(wheel_run(&fixture, &bus, oldest + 749u - fixture.now_ms));
    CHECK(fixture.app.machine.state == APP_STATE_REMOTE);
    CHECK(wheel_tick(&fixture, &bus));
    CHECK(fixture.app.machine.state == APP_STATE_FAULT);
    CHECK(!fixture.app.diagnostics.motion_authorized);
    CHECK(wheel_run(&fixture, &bus, 200u));
    CHECK(fixture.app.wheel_brake_state == WHEEL_PARKED && bus.right_target == 0 && bus.left_target == 0);
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
    CHECK(fixture.app.machine.state == APP_STATE_REMOTE_ARMING);
    for (index = 0u; index < 2u; ++index)
    {
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
    CHECK(fixture.app.mssd.operation == MSSD_OPERATION_STOP_BOTH);
    CHECK(!fixture.app.supervisor.communication_fault);
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
    CHECK(fixture.app.machine.state == APP_STATE_FAULT);
    CHECK(fixture.app.mssd.operation == MSSD_OPERATION_STOP_BOTH);
    fixture.app.mssd.right_status.fault_code = 0u;
    chassis_app_poll(&fixture.app, ++fixture.now_ms);
    CHECK(fixture.app.machine.state == APP_STATE_FAULT);
    CHECK(!fixture.app.supervisor.jetson_command.valid);
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

static bool reject_current_stop(fixture_t *fixture)
{
    uint8_t exception[] = {0x01u, 0x86u, 0x03u, 0u, 0u};
    CHECK(reach_current_response_wait(fixture));
    append_crc(exception, 3u);
    publish_modbus_bytes(fixture, exception, sizeof(exception));
    chassis_app_poll(&fixture->app, ++fixture->now_ms);
    return true;
}

static bool test_failed_stop_rearms_once_after_new_status(void)
{
    for (unsigned partial = 0u; partial <= 1u; ++partial)
    {
        fixture_t fixture;
        chassis_app_config_t config = chassis_app_default_config();
        CHECK(fixture_start(&fixture, config));
        make_application_ready(&fixture, APP_STATE_STANDBY);
        chassis_app_poll(&fixture.app, fixture.now_ms);
        CHECK(fixture.app.mssd.operation == MSSD_OPERATION_STOP_BOTH);
        CHECK(reject_current_stop(&fixture));
        CHECK(fixture.app.mssd.current_channel == MSSD_CHANNEL_LEFT);
        if (partial)
        {
            CHECK(complete_current_mssd_transaction(&fixture));
        }
        else
        {
            CHECK(reject_current_stop(&fixture));
        }
        CHECK(fixture.app.machine.state == APP_STATE_FAULT);
        CHECK(fixture.app.stop_failure_latched && !fixture.app.safe_stop_delivered);
        CHECK(!fixture.app.safe_stop_pending && !fixture.app.recovery_stop_attempted);
        for (unsigned i = 0u; i < 3u; ++i)
        {
            chassis_app_poll(&fixture.app, ++fixture.now_ms);
            CHECK(fixture.app.mssd.state == MSSD_DRIVER_IDLE);
        }
        fixture.app.next_status_poll_ms = fixture.now_ms;
        chassis_app_poll(&fixture.app, ++fixture.now_ms);
        CHECK(fixture.app.mssd.operation == MSSD_OPERATION_READ_RIGHT_STATUS);
        CHECK(complete_current_mssd_transaction(&fixture));
        CHECK(fixture.app.mssd.operation == MSSD_OPERATION_STOP_BOTH);
        CHECK(fixture.app.recovery_stop_attempted);
        CHECK(fixture.app.mssd.request[5] == MSSD_STOP_EMERGENCY);
        if (partial)
        {
            CHECK(complete_current_mssd_transaction(&fixture));
            CHECK(complete_current_mssd_transaction(&fixture));
        }
        else
        {
            CHECK(reject_current_stop(&fixture));
            CHECK(reject_current_stop(&fixture));
        }
        CHECK(complete_current_mssd_transaction(&fixture)); /* Pending left status. */
        CHECK(!fixture.app.safe_stop_pending);
        CHECK(fixture.app.safe_stop_delivered == (partial != 0u));
        CHECK(fixture.app.mssd.state == MSSD_DRIVER_IDLE);
        CHECK(fixture.app.machine.state == APP_STATE_FAULT);
        CHECK(!mssd_driver_get_motion_authorized(&fixture.app.mssd));
        CHECK(chassis_app_stop(&fixture.app));
    }
    return true;
}

static bool test_due_status_reads_precede_target_refresh(void)
{
    fixture_t fixture;
    chassis_app_config_t config = chassis_app_default_config();
    config.supervisor.motion_enable = true;
    config.kinematics.direction_mapping_verified = true;
    config.kinematics.maximum_rpm_verified = true;
    config.kinematics.maximum_motor_rpm = 1000.0;
    CHECK(fixture_start(&fixture, config));
    make_application_ready(&fixture, APP_STATE_INSPECTING);
    motion_supervisor_submit_jetson(&fixture.app.supervisor, 100.0, 0.0, fixture.now_ms);
    fixture.app.next_status_poll_ms = fixture.now_ms;
    chassis_app_poll(&fixture.app, fixture.now_ms);
    CHECK(fixture.app.mssd.operation == MSSD_OPERATION_READ_RIGHT_STATUS);
    CHECK(complete_current_mssd_transaction(&fixture));
    CHECK(fixture.app.mssd.operation == MSSD_OPERATION_READ_LEFT_STATUS);
    CHECK(complete_current_mssd_transaction(&fixture));
    CHECK(fixture.app.mssd.operation == MSSD_OPERATION_WRITE_TARGET_PAIR);
    CHECK(chassis_app_stop(&fixture.app));
    return true;
}

int main(void)
{
    RUN(test_due_status_reads_precede_target_refresh);
    RUN(test_failed_stop_rearms_once_after_new_status);
    RUN(test_startup_reads_configuration_and_both_statuses);
    RUN(test_standalone_rc_without_host);
    RUN(test_wheel_profile_rejects_wrong_drive_configuration);
    RUN(test_wheel_brake_failures_and_late_controller);
    RUN(test_wheel_fault_feedback_rc_and_uart_recovery);
    RUN(test_wheel_swc_preempts_right_release_reply);
    RUN(test_wheel_stale_feedback_preempts_status_retries);
    RUN(test_inspection_finish_clears_old_speed_and_remote_retakes);
    RUN(test_jetson_cancellation_clears_speed_without_echoing_stop);
    RUN(test_emergency_and_timeout_preempt_motion);
    RUN(test_rc_loss_motor_fault_and_uart_error_request_safe_stop);
    RUN(test_partial_commit_latches_fault_and_prioritizes_stop);
    RUN(test_default_lock_never_builds_nonzero_or_forbidden_writes);
    printf("all %u chassis_end_to_end tests passed\n", tests_run);
    return 0;
}
