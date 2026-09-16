#include "mssd_driver.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
    bool de;
    bool tx_allowed;
    bool rx_allowed;
    bool dma_allowed;
    bool tc;
    unsigned set_de_calls;
} mock_port_t;

static unsigned tests_run;

#define CHECK(condition)                                                                    \
    do                                                                                      \
    {                                                                                       \
        if (!(condition))                                                                   \
        {                                                                                   \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition);               \
            return false;                                                                   \
        }                                                                                   \
    } while (0)

#define RUN(test)                                                                           \
    do                                                                                      \
    {                                                                                       \
        if (!(test()))                                                                      \
        {                                                                                   \
            return 1;                                                                       \
        }                                                                                   \
        ++tests_run;                                                                        \
    } while (0)

static bool mock_set_de(bool transmit, void *context)
{
    mock_port_t *mock = context;

    ++mock->set_de_calls;
    if ((transmit && !mock->tx_allowed) || (!transmit && !mock->rx_allowed))
    {
        return false;
    }
    mock->de = transmit;
    return true;
}

static bool mock_start_dma(const uint8_t *data, size_t length, void *context)
{
    (void)data;
    (void)length;
    return ((mock_port_t *)context)->dma_allowed;
}

static bool mock_uart_tc(void *context)
{
    return ((mock_port_t *)context)->tc;
}

static void append_crc(uint8_t *frame, size_t body_length)
{
    const uint16_t crc = modbus_rtu_crc16(frame, body_length);

    frame[body_length] = (uint8_t)crc;
    frame[body_length + 1u] = (uint8_t)(crc >> 8u);
}

static bool setup(mssd_driver_t *driver,
                  rs485_transport_t *transport,
                  mock_port_t *mock,
                  mssd_driver_config_t config)
{
    const rs485_port_t port = {
        mock_set_de,
        mock_start_dma,
        mock_uart_tc,
        mock,
    };
    const rs485_timeouts_t timeouts = {10u, 10u, 20u};

    *mock = (mock_port_t){false, true, true, true, false, 0u};
    rs485_transport_init(transport, port, timeouts);
    CHECK(mssd_driver_init(driver, transport, &config));
    return true;
}

static bool reach_waiting_response(mssd_driver_t *driver,
                                   rs485_transport_t *transport,
                                   mock_port_t *mock,
                                   uint32_t now_ms)
{
    mssd_driver_poll(driver, now_ms);
    CHECK(transport->state == RS485_DMA_TX);
    CHECK(rs485_transport_dma_complete(transport, now_ms + 1u));
    mock->tc = true;
    mssd_driver_poll(driver, now_ms + 2u);
    CHECK(transport->state == RS485_SWITCH_TO_RX);
    mssd_driver_poll(driver, now_ms + 3u);
    CHECK(transport->state == RS485_WAIT_RESPONSE);
    CHECK(mssd_driver_get_state(driver) == MSSD_DRIVER_WAITING_RESPONSE);
    return true;
}

static size_t make_write_response(const rs485_transport_t *transport, uint8_t *response)
{
    (void)memcpy(response, transport->request, 6u);
    append_crc(response, 6u);
    return 8u;
}

static bool complete_write(mssd_driver_t *driver,
                           rs485_transport_t *transport,
                           mock_port_t *mock,
                           uint32_t now_ms)
{
    uint8_t response[8];
    const size_t response_length = make_write_response(transport, response);

    CHECK(reach_waiting_response(driver, transport, mock, now_ms));
    CHECK(mssd_driver_on_response(driver,
                                  response,
                                  response_length,
                                  now_ms + 4u) == MSSD_RESPONSE_ACCEPTED);
    return true;
}

static bool confirm_configuration(mssd_driver_t *driver,
                                  rs485_transport_t *transport,
                                  mock_port_t *mock,
                                  uint32_t now_ms)
{
    uint8_t response[] = {
        0x01u, 0x03u, 0x08u,
        0x00u, 0x01u,
        0x00u, 0x00u,
        0x00u, 0x01u,
        0x00u, 0x01u,
        0x00u, 0x00u,
    };

    CHECK(mssd_driver_request_configuration(driver, now_ms) == MSSD_REQUEST_ACCEPTED);
    CHECK(reach_waiting_response(driver, transport, mock, now_ms));
    append_crc(response, sizeof(response) - 2u);
    CHECK(mssd_driver_on_response(driver,
                                  response,
                                  sizeof(response),
                                  now_ms + 4u) == MSSD_RESPONSE_ACCEPTED);
    CHECK(mssd_driver_get_state(driver) == MSSD_DRIVER_COMPLETE);
    CHECK(mssd_driver_acknowledge(driver, now_ms + 5u));
    return true;
}

static bool test_init_is_software_only_and_defaults_are_safe(void)
{
    mssd_driver_t driver;
    rs485_transport_t transport;
    mock_port_t mock;
    const mssd_driver_config_t config = mssd_driver_default_config();
    unsigned calls_before;

    mock = (mock_port_t){false, true, true, true, false, 0u};
    {
        const rs485_port_t port = {
            mock_set_de,
            mock_start_dma,
            mock_uart_tc,
            &mock,
        };
        const rs485_timeouts_t timeouts = {10u, 10u, 20u};

        rs485_transport_init(&transport, port, timeouts);
    }
    calls_before = mock.set_de_calls;
    CHECK(mssd_driver_init(&driver, &transport, &config));
    CHECK(mock.set_de_calls == calls_before);
    CHECK(transport.state == RS485_IDLE);
    CHECK(mssd_driver_get_state(&driver) == MSSD_DRIVER_IDLE);
    CHECK(config.read_retry_limit == 0u);
    CHECK(!config.right_brake.mapping_valid);
    CHECK(!config.left_brake.mapping_valid);
    CHECK(!mssd_driver_get_motion_authorized(&driver));
    return true;
}

static bool test_status_and_configuration_read_requests(void)
{
    static const uint8_t right_expected[] = {
        0x01u, 0x03u, 0x00u, 0x08u, 0x00u, 0x04u, 0xC5u, 0xCBu,
    };
    static const uint8_t left_expected[] = {
        0x01u, 0x03u, 0x00u, 0x13u, 0x00u, 0x04u, 0xB5u, 0xCCu,
    };
    static const uint8_t config_expected[] = {
        0x01u, 0x03u, 0x00u, 0x7Bu, 0x00u, 0x04u, 0x34u, 0x10u,
    };
    mssd_driver_t driver;
    rs485_transport_t transport;
    mock_port_t mock;
    const mssd_driver_config_t config = mssd_driver_default_config();

    setup(&driver, &transport, &mock, config);
    CHECK(mssd_driver_request_status(&driver, MSSD_CHANNEL_RIGHT, 0u) ==
          MSSD_REQUEST_ACCEPTED);
    CHECK(memcmp(transport.request, right_expected, sizeof(right_expected)) == 0);

    setup(&driver, &transport, &mock, config);
    CHECK(mssd_driver_request_status(&driver, MSSD_CHANNEL_LEFT, 0u) ==
          MSSD_REQUEST_ACCEPTED);
    CHECK(memcmp(transport.request, left_expected, sizeof(left_expected)) == 0);

    setup(&driver, &transport, &mock, config);
    CHECK(mssd_driver_request_configuration(&driver, 0u) == MSSD_REQUEST_ACCEPTED);
    CHECK(memcmp(transport.request, config_expected, sizeof(config_expected)) == 0);
    return true;
}

static bool test_target_golden_frames(void)
{
    static const uint8_t right_expected[] = {
        0x01u, 0x10u, 0x00u, 0x39u, 0x00u, 0x02u, 0x04u,
        0x00u, 0x00u, 0x03u, 0xE8u, 0x30u, 0x6Fu,
    };
    static const uint8_t left_expected[] = {
        0x01u, 0x10u, 0x00u, 0x3Du, 0x00u, 0x02u, 0x04u,
        0xFFu, 0xFFu, 0xFEu, 0x0Cu, 0x71u, 0x63u,
    };
    mssd_driver_t driver;
    rs485_transport_t transport;
    mock_port_t mock;
    const mssd_driver_config_t config = mssd_driver_default_config();

    setup(&driver, &transport, &mock, config);
    CHECK(confirm_configuration(&driver, &transport, &mock, 0u));
    mssd_driver_set_motion_authorized(&driver, true);
    CHECK(mssd_driver_request_target_rpm(&driver,
                                         MSSD_CHANNEL_RIGHT,
                                         1000,
                                         10u) == MSSD_REQUEST_ACCEPTED);
    CHECK(memcmp(transport.request, right_expected, sizeof(right_expected)) == 0);

    setup(&driver, &transport, &mock, config);
    CHECK(confirm_configuration(&driver, &transport, &mock, 20u));
    mssd_driver_set_motion_authorized(&driver, true);
    CHECK(mssd_driver_request_target_rpm(&driver,
                                         MSSD_CHANNEL_LEFT,
                                         -500,
                                         30u) == MSSD_REQUEST_ACCEPTED);
    CHECK(memcmp(transport.request, left_expected, sizeof(left_expected)) == 0);
    return true;
}

static bool test_target_encodes_zero_and_int32_boundaries(void)
{
    static const struct
    {
        int32_t rpm;
        uint8_t bytes[4];
    } cases[] = {
        {0, {0x00u, 0x00u, 0x00u, 0x00u}},
        {INT32_MAX, {0x7Fu, 0xFFu, 0xFFu, 0xFFu}},
        {INT32_MIN, {0x80u, 0x00u, 0x00u, 0x00u}},
    };
    size_t index;

    for (index = 0u; index < (sizeof(cases) / sizeof(cases[0])); ++index)
    {
        mssd_driver_t driver;
        rs485_transport_t transport;
        mock_port_t mock;
        const mssd_driver_config_t config = mssd_driver_default_config();

        setup(&driver, &transport, &mock, config);
        CHECK(confirm_configuration(&driver, &transport, &mock, 0u));
        mssd_driver_set_motion_authorized(&driver, true);
        CHECK(mssd_driver_request_target_rpm(&driver,
                                             MSSD_CHANNEL_RIGHT,
                                             cases[index].rpm,
                                             10u) == MSSD_REQUEST_ACCEPTED);
        CHECK(memcmp(&transport.request[7], cases[index].bytes, 4u) == 0);
        CHECK(transport.request[2] == 0x00u);
        CHECK(transport.request[3] == 0x39u);
        CHECK(transport.request[4] == 0x00u);
        CHECK(transport.request[5] == 0x02u);
    }
    return true;
}

static bool test_stop_frames_use_only_function_06(void)
{
    static const struct
    {
        mssd_channel_t channel;
        mssd_stop_type_t type;
        uint8_t expected[8];
    } cases[] = {
        {MSSD_CHANNEL_RIGHT, MSSD_STOP_NORMAL,
         {0x01u, 0x06u, 0x00u, 0x38u, 0x00u, 0x00u, 0x08u, 0x07u}},
        {MSSD_CHANNEL_RIGHT, MSSD_STOP_EMERGENCY,
         {0x01u, 0x06u, 0x00u, 0x38u, 0x00u, 0x01u, 0xC9u, 0xC7u}},
        {MSSD_CHANNEL_RIGHT, MSSD_STOP_FREE,
         {0x01u, 0x06u, 0x00u, 0x38u, 0x00u, 0x02u, 0x89u, 0xC6u}},
        {MSSD_CHANNEL_LEFT, MSSD_STOP_NORMAL,
         {0x01u, 0x06u, 0x00u, 0x3Cu, 0x00u, 0x00u, 0x49u, 0xC6u}},
        {MSSD_CHANNEL_LEFT, MSSD_STOP_EMERGENCY,
         {0x01u, 0x06u, 0x00u, 0x3Cu, 0x00u, 0x01u, 0x88u, 0x06u}},
        {MSSD_CHANNEL_LEFT, MSSD_STOP_FREE,
         {0x01u, 0x06u, 0x00u, 0x3Cu, 0x00u, 0x02u, 0xC8u, 0x07u}},
    };
    size_t index;

    for (index = 0u; index < (sizeof(cases) / sizeof(cases[0])); ++index)
    {
        mssd_driver_t driver;
        rs485_transport_t transport;
        mock_port_t mock;
        const mssd_driver_config_t config = mssd_driver_default_config();

        setup(&driver, &transport, &mock, config);
        CHECK(mssd_driver_request_stop(&driver,
                                       cases[index].channel,
                                       cases[index].type,
                                       0u) == MSSD_REQUEST_ACCEPTED);
        CHECK(memcmp(transport.request, cases[index].expected, 8u) == 0);
        CHECK(transport.request[1] == 0x06u);
    }
    return true;
}

static bool test_status_response_parsing(void)
{
    uint8_t right_response[] = {
        0x01u, 0x03u, 0x08u,
        0x01u, 0x44u,
        0x00u, 0x00u, 0x01u, 0x37u,
        0x00u, 0x07u,
        0x00u, 0x00u,
    };
    uint8_t left_response[] = {
        0x01u, 0x03u, 0x08u,
        0x01u, 0x25u,
        0xFFu, 0xFFu, 0xFEu, 0x9Eu,
        0x00u, 0x03u,
        0x00u, 0x00u,
    };
    mssd_driver_t driver;
    rs485_transport_t transport;
    mock_port_t mock;
    mssd_motor_status_t status;
    const mssd_driver_config_t config = mssd_driver_default_config();

    setup(&driver, &transport, &mock, config);
    CHECK(mssd_driver_request_status(&driver, MSSD_CHANNEL_RIGHT, 0u) ==
          MSSD_REQUEST_ACCEPTED);
    CHECK(reach_waiting_response(&driver, &transport, &mock, 0u));
    append_crc(right_response, sizeof(right_response) - 2u);
    CHECK(mssd_driver_on_response(&driver,
                                  right_response,
                                  sizeof(right_response),
                                  4u) == MSSD_RESPONSE_ACCEPTED);
    CHECK(mssd_driver_get_status(&driver, MSSD_CHANNEL_RIGHT, &status));
    CHECK(status.valid);
    CHECK(status.phase_current_raw == 324u);
    CHECK(status.phase_current_milliamps == 3240u);
    CHECK(status.rpm == 311);
    CHECK(status.fault_code == 7u);

    setup(&driver, &transport, &mock, config);
    CHECK(mssd_driver_request_status(&driver, MSSD_CHANNEL_LEFT, 10u) ==
          MSSD_REQUEST_ACCEPTED);
    CHECK(reach_waiting_response(&driver, &transport, &mock, 10u));
    append_crc(left_response, sizeof(left_response) - 2u);
    CHECK(mssd_driver_on_response(&driver,
                                  left_response,
                                  sizeof(left_response),
                                  14u) == MSSD_RESPONSE_ACCEPTED);
    CHECK(mssd_driver_get_status(&driver, MSSD_CHANNEL_LEFT, &status));
    CHECK(status.rpm == -354);
    CHECK(status.fault_code == 3u);
    return true;
}

static bool test_configuration_and_motion_gates(void)
{
    mssd_driver_t driver;
    rs485_transport_t transport;
    mock_port_t mock;
    const mssd_driver_config_t config = mssd_driver_default_config();

    setup(&driver, &transport, &mock, config);
    mssd_driver_set_motion_authorized(&driver, true);
    CHECK(mssd_driver_request_target_rpm(&driver,
                                         MSSD_CHANNEL_RIGHT,
                                         1,
                                         0u) == MSSD_REQUEST_CONFIG_UNVERIFIED);
    CHECK(transport.state == RS485_IDLE);

    CHECK(confirm_configuration(&driver, &transport, &mock, 10u));
    mssd_driver_set_motion_authorized(&driver, false);
    CHECK(mssd_driver_request_target_rpm(&driver,
                                         MSSD_CHANNEL_RIGHT,
                                         1,
                                         20u) == MSSD_REQUEST_MOTION_NOT_AUTHORIZED);
    CHECK(transport.state == RS485_IDLE);

    mssd_driver_set_motion_authorized(&driver, true);
    CHECK(mssd_driver_request_target_rpm(&driver,
                                         MSSD_CHANNEL_RIGHT,
                                         1,
                                         21u) == MSSD_REQUEST_ACCEPTED);
    return true;
}

static bool test_wrong_operating_mode_does_not_authorize_rpm(void)
{
    uint8_t response[] = {
        0x01u, 0x03u, 0x08u,
        0x00u, 0x04u,
        0x00u, 0x00u,
        0x00u, 0x01u,
        0x00u, 0x01u,
        0x00u, 0x00u,
    };
    mssd_driver_t driver;
    rs485_transport_t transport;
    mock_port_t mock;
    mssd_configuration_snapshot_t snapshot;
    const mssd_driver_config_t config = mssd_driver_default_config();

    setup(&driver, &transport, &mock, config);
    CHECK(mssd_driver_request_configuration(&driver, 0u) == MSSD_REQUEST_ACCEPTED);
    CHECK(reach_waiting_response(&driver, &transport, &mock, 0u));
    append_crc(response, sizeof(response) - 2u);
    CHECK(mssd_driver_on_response(&driver,
                                  response,
                                  sizeof(response),
                                  4u) == MSSD_RESPONSE_ACCEPTED);
    CHECK(mssd_driver_get_configuration(&driver, &snapshot));
    CHECK(snapshot.system_mode == 4u);
    CHECK(snapshot.control_source == 0u);
    CHECK(mssd_driver_acknowledge(&driver, 5u));
    mssd_driver_set_motion_authorized(&driver, true);
    CHECK(mssd_driver_request_target_rpm(&driver,
                                         MSSD_CHANNEL_RIGHT,
                                         1,
                                         6u) == MSSD_REQUEST_CONFIG_UNVERIFIED);
    return true;
}

static bool test_busy_rejects_second_transaction(void)
{
    mssd_driver_t driver;
    rs485_transport_t transport;
    mock_port_t mock;
    const mssd_driver_config_t config = mssd_driver_default_config();

    setup(&driver, &transport, &mock, config);
    CHECK(mssd_driver_request_status(&driver, MSSD_CHANNEL_RIGHT, 0u) ==
          MSSD_REQUEST_ACCEPTED);
    CHECK(mssd_driver_request_status(&driver, MSSD_CHANNEL_LEFT, 0u) ==
          MSSD_REQUEST_BUSY);
    CHECK(transport.request[3] == 0x08u);
    return true;
}

static bool test_invalid_frames_do_not_end_response_wait(void)
{
    uint8_t response[] = {
        0x01u, 0x03u, 0x08u,
        0x00u, 0x01u,
        0x00u, 0x00u, 0x00u, 0x02u,
        0x00u, 0x00u,
        0x00u, 0x00u,
    };
    mssd_driver_t driver;
    rs485_transport_t transport;
    mock_port_t mock;
    mssd_error_detail_t error;
    const mssd_driver_config_t config = mssd_driver_default_config();

    setup(&driver, &transport, &mock, config);
    CHECK(mssd_driver_request_status(&driver, MSSD_CHANNEL_RIGHT, 0u) ==
          MSSD_REQUEST_ACCEPTED);
    CHECK(reach_waiting_response(&driver, &transport, &mock, 0u));
    append_crc(response, sizeof(response) - 2u);

    response[sizeof(response) - 1u] ^= 1u;
    CHECK(mssd_driver_on_response(&driver,
                                  response,
                                  sizeof(response),
                                  4u) == MSSD_RESPONSE_INVALID_FRAME);
    error = mssd_driver_get_error(&driver);
    CHECK(error.modbus_status == MODBUS_RTU_VALIDATION_CRC_ERROR);
    CHECK(transport.state == RS485_WAIT_RESPONSE);

    response[sizeof(response) - 1u] ^= 1u;
    response[0] = 2u;
    append_crc(response, sizeof(response) - 2u);
    CHECK(mssd_driver_on_response(&driver,
                                  response,
                                  sizeof(response),
                                  5u) == MSSD_RESPONSE_INVALID_FRAME);
    CHECK(mssd_driver_get_error(&driver).modbus_status ==
          MODBUS_RTU_VALIDATION_SLAVE_ERROR);

    response[0] = 1u;
    response[1] = 0x06u;
    append_crc(response, sizeof(response) - 2u);
    CHECK(mssd_driver_on_response(&driver,
                                  response,
                                  sizeof(response),
                                  6u) == MSSD_RESPONSE_INVALID_FRAME);
    CHECK(mssd_driver_get_error(&driver).modbus_status ==
          MODBUS_RTU_VALIDATION_FUNCTION_ERROR);

    response[1] = 0x03u;
    append_crc(response, sizeof(response) - 2u);
    CHECK(mssd_driver_on_response(&driver,
                                  response,
                                  sizeof(response) - 1u,
                                  7u) == MSSD_RESPONSE_INVALID_FRAME);
    CHECK(mssd_driver_get_error(&driver).modbus_status ==
          MODBUS_RTU_VALIDATION_LENGTH_ERROR);
    CHECK(transport.state == RS485_WAIT_RESPONSE);
    return true;
}

static bool test_exception_and_write_echo_mismatch(void)
{
    uint8_t exception[] = {0x01u, 0x83u, 0x02u, 0x00u, 0x00u};
    uint8_t response[8];
    mssd_driver_t driver;
    rs485_transport_t transport;
    mock_port_t mock;
    const mssd_driver_config_t config = mssd_driver_default_config();

    setup(&driver, &transport, &mock, config);
    CHECK(mssd_driver_request_status(&driver, MSSD_CHANNEL_RIGHT, 0u) ==
          MSSD_REQUEST_ACCEPTED);
    CHECK(reach_waiting_response(&driver, &transport, &mock, 0u));
    append_crc(exception, 3u);
    CHECK(mssd_driver_on_response(&driver,
                                  exception,
                                  sizeof(exception),
                                  4u) == MSSD_RESPONSE_ACCEPTED);
    CHECK(mssd_driver_get_state(&driver) == MSSD_DRIVER_FAILED);
    CHECK(mssd_driver_get_error(&driver).code == MSSD_ERROR_MODBUS_EXCEPTION);
    CHECK(mssd_driver_get_error(&driver).exception_code == 0x02u);

    setup(&driver, &transport, &mock, config);
    CHECK(mssd_driver_request_stop(&driver,
                                   MSSD_CHANNEL_RIGHT,
                                   MSSD_STOP_NORMAL,
                                   10u) == MSSD_REQUEST_ACCEPTED);
    CHECK(reach_waiting_response(&driver, &transport, &mock, 10u));
    (void)make_write_response(&transport, response);
    response[5] = 1u;
    append_crc(response, 6u);
    CHECK(mssd_driver_on_response(&driver,
                                  response,
                                  sizeof(response),
                                  14u) == MSSD_RESPONSE_INVALID_FRAME);
    CHECK(mssd_driver_get_error(&driver).modbus_status ==
          MODBUS_RTU_VALIDATION_ECHO_ERROR);
    CHECK(transport.state == RS485_WAIT_RESPONSE);
    return true;
}

static bool test_response_lifecycle_results_are_explicit(void)
{
    uint8_t response[8];
    uint8_t noise[] = {0x55u, 0xAAu};
    mssd_driver_t driver;
    rs485_transport_t transport;
    mock_port_t mock;
    const mssd_driver_config_t config = mssd_driver_default_config();

    setup(&driver, &transport, &mock, config);
    CHECK(mssd_driver_on_response(&driver, noise, sizeof(noise), 0u) ==
          MSSD_RESPONSE_IDLE_FRAME);

    CHECK(mssd_driver_request_stop(&driver,
                                   MSSD_CHANNEL_RIGHT,
                                   MSSD_STOP_NORMAL,
                                   1u) == MSSD_REQUEST_ACCEPTED);
    (void)make_write_response(&transport, response);
    CHECK(mssd_driver_on_response(&driver,
                                  response,
                                  sizeof(response),
                                  1u) == MSSD_RESPONSE_UNEXPECTED_FRAME);
    CHECK(complete_write(&driver, &transport, &mock, 1u));
    CHECK(mssd_driver_get_state(&driver) == MSSD_DRIVER_COMPLETE);
    CHECK(mssd_driver_on_response(&driver,
                                  response,
                                  sizeof(response),
                                  6u) == MSSD_RESPONSE_DUPLICATE_FRAME);
    CHECK(mssd_driver_acknowledge(&driver, 7u));
    CHECK(mssd_driver_on_response(&driver,
                                  response,
                                  sizeof(response),
                                  8u) == MSSD_RESPONSE_DUPLICATE_FRAME);

    setup(&driver, &transport, &mock, config);
    CHECK(mssd_driver_request_stop(&driver,
                                   MSSD_CHANNEL_RIGHT,
                                   MSSD_STOP_NORMAL,
                                   20u) == MSSD_REQUEST_ACCEPTED);
    (void)make_write_response(&transport, response);
    CHECK(reach_waiting_response(&driver, &transport, &mock, 20u));
    mssd_driver_poll(&driver, 43u);
    CHECK(mssd_driver_get_state(&driver) == MSSD_DRIVER_FAILED);
    CHECK(mssd_driver_on_response(&driver,
                                  response,
                                  sizeof(response),
                                  44u) == MSSD_RESPONSE_LATE_FRAME);
    return true;
}

static bool test_pair_target_reports_partial_commit(void)
{
    uint8_t right_response[8];
    uint8_t left_exception[] = {0x01u, 0x90u, 0x03u, 0x00u, 0x00u};
    mssd_driver_t driver;
    rs485_transport_t transport;
    mock_port_t mock;
    mssd_transaction_result_t result;
    const mssd_driver_config_t config = mssd_driver_default_config();

    setup(&driver, &transport, &mock, config);
    CHECK(confirm_configuration(&driver, &transport, &mock, 0u));
    mssd_driver_set_motion_authorized(&driver, true);
    CHECK(mssd_driver_request_target_pair(&driver, 1000, -500, 10u) ==
          MSSD_REQUEST_ACCEPTED);
    CHECK(transport.request[3] == 0x39u);
    (void)make_write_response(&transport, right_response);
    CHECK(reach_waiting_response(&driver, &transport, &mock, 10u));
    CHECK(mssd_driver_on_response(&driver,
                                  right_response,
                                  sizeof(right_response),
                                  14u) == MSSD_RESPONSE_ACCEPTED);
    CHECK(transport.state == RS485_SWITCH_TO_TX);
    CHECK(transport.request[3] == 0x3Du);

    mock.tc = false;
    CHECK(reach_waiting_response(&driver, &transport, &mock, 15u));
    append_crc(left_exception, 3u);
    CHECK(mssd_driver_on_response(&driver,
                                  left_exception,
                                  sizeof(left_exception),
                                  19u) == MSSD_RESPONSE_ACCEPTED);
    CHECK(mssd_driver_get_state(&driver) == MSSD_DRIVER_PARTIAL_COMMIT);
    result = mssd_driver_get_result(&driver);
    CHECK(result.completion == MSSD_COMPLETION_PARTIAL_COMMIT);
    CHECK(result.right == MSSD_SIDE_SUCCEEDED);
    CHECK(result.left == MSSD_SIDE_FAILED);
    return true;
}

static bool test_brake_mapping_is_validated_per_channel(void)
{
    mssd_driver_t driver;
    rs485_transport_t transport;
    mock_port_t mock;
    mssd_driver_config_t config = mssd_driver_default_config();

    setup(&driver, &transport, &mock, config);
    CHECK(mssd_driver_request_brake(&driver,
                                    MSSD_CHANNEL_RIGHT,
                                    MSSD_BRAKE_ENGAGE,
                                    0u) == MSSD_REQUEST_BRAKE_MAPPING_UNVERIFIED);
    CHECK(transport.state == RS485_IDLE);

    config.right_brake.engage_raw = 1u;
    config.right_brake.release_raw = 0u;
    config.right_brake.mapping_valid = true;
    setup(&driver, &transport, &mock, config);
    CHECK(mssd_driver_request_brake(&driver,
                                    MSSD_CHANNEL_RIGHT,
                                    MSSD_BRAKE_ENGAGE,
                                    0u) == MSSD_REQUEST_ACCEPTED);
    CHECK((transport.request[1] == 0x06u) && (transport.request[3] == 0x3Bu));
    CHECK((transport.request[4] == 0u) && (transport.request[5] == 1u));

    setup(&driver, &transport, &mock, config);
    CHECK(mssd_driver_request_brake(&driver,
                                    MSSD_CHANNEL_LEFT,
                                    MSSD_BRAKE_ENGAGE,
                                    0u) == MSSD_REQUEST_BRAKE_MAPPING_UNVERIFIED);
    CHECK(mssd_driver_request_brake(&driver,
                                    MSSD_CHANNEL_BOTH,
                                    MSSD_BRAKE_ENGAGE,
                                    0u) == MSSD_REQUEST_BRAKE_MAPPING_UNVERIFIED);

    config = mssd_driver_default_config();
    config.left_brake.engage_raw = 0u;
    config.left_brake.release_raw = 1u;
    config.left_brake.mapping_valid = true;
    setup(&driver, &transport, &mock, config);
    CHECK(mssd_driver_request_brake(&driver,
                                    MSSD_CHANNEL_LEFT,
                                    MSSD_BRAKE_ENGAGE,
                                    0u) == MSSD_REQUEST_ACCEPTED);
    CHECK((transport.request[1] == 0x06u) && (transport.request[3] == 0x3Fu));
    CHECK((transport.request[4] == 0u) && (transport.request[5] == 0u));
    return true;
}

static bool test_dual_brake_can_report_partial_commit(void)
{
    uint8_t right_response[8];
    uint8_t left_exception[] = {0x01u, 0x86u, 0x04u, 0x00u, 0x00u};
    mssd_driver_t driver;
    rs485_transport_t transport;
    mock_port_t mock;
    mssd_driver_config_t config = mssd_driver_default_config();
    mssd_transaction_result_t result;

    config.right_brake = (mssd_brake_mapping_t){1u, 0u, true};
    config.left_brake = (mssd_brake_mapping_t){0u, 1u, true};
    setup(&driver, &transport, &mock, config);
    CHECK(mssd_driver_request_brake(&driver,
                                    MSSD_CHANNEL_BOTH,
                                    MSSD_BRAKE_ENGAGE,
                                    0u) == MSSD_REQUEST_ACCEPTED);
    CHECK(transport.request[3] == 0x3Bu);
    CHECK(transport.request[5] == 1u);
    (void)make_write_response(&transport, right_response);
    CHECK(reach_waiting_response(&driver, &transport, &mock, 0u));
    CHECK(mssd_driver_on_response(&driver,
                                  right_response,
                                  sizeof(right_response),
                                  4u) == MSSD_RESPONSE_ACCEPTED);
    CHECK(transport.request[3] == 0x3Fu);
    CHECK(transport.request[5] == 0u);

    mock.tc = false;
    CHECK(reach_waiting_response(&driver, &transport, &mock, 5u));
    append_crc(left_exception, 3u);
    CHECK(mssd_driver_on_response(&driver,
                                  left_exception,
                                  sizeof(left_exception),
                                  9u) == MSSD_RESPONSE_ACCEPTED);
    result = mssd_driver_get_result(&driver);
    CHECK(result.completion == MSSD_COMPLETION_PARTIAL_COMMIT);
    CHECK(result.right == MSSD_SIDE_SUCCEEDED);
    CHECK(result.left == MSSD_SIDE_FAILED);
    return true;
}

static bool test_public_writes_never_cross_control_register_block(void)
{
    mssd_driver_t driver;
    rs485_transport_t transport;
    mock_port_t mock;
    const mssd_driver_config_t config = mssd_driver_default_config();

    setup(&driver, &transport, &mock, config);
    CHECK(confirm_configuration(&driver, &transport, &mock, 0u));
    mssd_driver_set_motion_authorized(&driver, true);
    CHECK(mssd_driver_request_target_pair(&driver, 1, 2, 10u) ==
          MSSD_REQUEST_ACCEPTED);
    CHECK(transport.request[1] == 0x10u);
    CHECK(transport.request[2] == 0x00u);
    CHECK(transport.request[3] == 0x39u);
    CHECK(transport.request[4] == 0x00u);
    CHECK(transport.request[5] == 0x02u);

    setup(&driver, &transport, &mock, config);
    CHECK(mssd_driver_request_stop(&driver,
                                   MSSD_CHANNEL_BOTH,
                                   MSSD_STOP_EMERGENCY,
                                   0u) == MSSD_REQUEST_ACCEPTED);
    CHECK(transport.request[1] == 0x06u);
    CHECK(transport.request[3] == 0x38u);
    return true;
}

static bool test_read_retry_limit_is_bounded_and_writes_never_retry(void)
{
    mssd_driver_t driver;
    rs485_transport_t transport;
    mock_port_t mock;
    mssd_driver_config_t config = mssd_driver_default_config();

    config.read_retry_limit = MSSD_MAX_READ_RETRIES + 1u;
    mock = (mock_port_t){false, true, true, true, false, 0u};
    CHECK(!mssd_driver_init(&driver, &transport, &config));

    config = mssd_driver_default_config();
    setup(&driver, &transport, &mock, config);
    CHECK(mssd_driver_request_stop(&driver,
                                   MSSD_CHANNEL_RIGHT,
                                   MSSD_STOP_NORMAL,
                                   0u) == MSSD_REQUEST_ACCEPTED);
    CHECK(reach_waiting_response(&driver, &transport, &mock, 0u));
    mssd_driver_poll(&driver, 23u);
    CHECK(mssd_driver_get_state(&driver) == MSSD_DRIVER_FAILED);
    CHECK(transport.state == RS485_TIMEOUT);
    return true;
}

static bool test_read_retry_reuses_the_exact_request_and_stops_at_limit(void)
{
    uint8_t original_request[8];
    mssd_driver_t driver;
    rs485_transport_t transport;
    mock_port_t mock;
    mssd_driver_config_t config = mssd_driver_default_config();

    config.read_retry_limit = 1u;
    setup(&driver, &transport, &mock, config);
    CHECK(mssd_driver_request_status(&driver, MSSD_CHANNEL_RIGHT, 0u) ==
          MSSD_REQUEST_ACCEPTED);
    (void)memcpy(original_request, transport.request, sizeof(original_request));
    CHECK(reach_waiting_response(&driver, &transport, &mock, 0u));

    mssd_driver_poll(&driver, 23u);
    CHECK(mssd_driver_get_state(&driver) == MSSD_DRIVER_WAITING_TRANSPORT);
    CHECK(transport.state == RS485_SWITCH_TO_TX);
    CHECK(memcmp(transport.request, original_request, sizeof(original_request)) == 0);

    mock.tc = false;
    CHECK(reach_waiting_response(&driver, &transport, &mock, 23u));
    mssd_driver_poll(&driver, 46u);
    CHECK(mssd_driver_get_state(&driver) == MSSD_DRIVER_FAILED);
    CHECK(transport.state == RS485_TIMEOUT);
    return true;
}

static bool test_active_transaction_can_be_cancelled_for_safety(void)
{
    uint8_t right_response[8];
    mssd_driver_t driver;
    rs485_transport_t transport;
    mock_port_t mock;
    mssd_transaction_result_t result;
    const mssd_driver_config_t config = mssd_driver_default_config();

    setup(&driver, &transport, &mock, config);
    CHECK(confirm_configuration(&driver, &transport, &mock, 0u));
    mssd_driver_set_motion_authorized(&driver, true);
    CHECK(mssd_driver_request_target_pair(&driver, 100, 200, 10u) ==
          MSSD_REQUEST_ACCEPTED);
    CHECK(reach_waiting_response(&driver, &transport, &mock, 10u));
    (void)make_write_response(&transport, right_response);
    CHECK(mssd_driver_on_response(&driver, right_response,
                                  sizeof(right_response), 14u) ==
          MSSD_RESPONSE_ACCEPTED);
    CHECK(driver.current_channel == MSSD_CHANNEL_LEFT);
    CHECK(mssd_driver_abort_for_safety(&driver, 15u));
    CHECK(transport.state == RS485_ERROR);
    CHECK(mssd_driver_get_state(&driver) == MSSD_DRIVER_PARTIAL_COMMIT);
    result = mssd_driver_get_result(&driver);
    CHECK(result.right == MSSD_SIDE_SUCCEEDED);
    CHECK(result.left == MSSD_SIDE_FAILED);
    CHECK(result.completion == MSSD_COMPLETION_PARTIAL_COMMIT);
    CHECK(mssd_driver_acknowledge(&driver, 16u));
    CHECK(mssd_driver_get_state(&driver) == MSSD_DRIVER_IDLE);
    return true;
}

int main(void)
{
    RUN(test_init_is_software_only_and_defaults_are_safe);
    RUN(test_status_and_configuration_read_requests);
    RUN(test_target_golden_frames);
    RUN(test_target_encodes_zero_and_int32_boundaries);
    RUN(test_stop_frames_use_only_function_06);
    RUN(test_status_response_parsing);
    RUN(test_configuration_and_motion_gates);
    RUN(test_wrong_operating_mode_does_not_authorize_rpm);
    RUN(test_busy_rejects_second_transaction);
    RUN(test_invalid_frames_do_not_end_response_wait);
    RUN(test_exception_and_write_echo_mismatch);
    RUN(test_response_lifecycle_results_are_explicit);
    RUN(test_pair_target_reports_partial_commit);
    RUN(test_brake_mapping_is_validated_per_channel);
    RUN(test_dual_brake_can_report_partial_commit);
    RUN(test_public_writes_never_cross_control_register_block);
    RUN(test_read_retry_limit_is_bounded_and_writes_never_retry);
    RUN(test_read_retry_reuses_the_exact_request_and_stops_at_limit);
    RUN(test_active_transaction_can_be_cancelled_for_safety);
    printf("all %u mssd_driver tests passed\n", tests_run);
    return 0;
}
