#include "chassis_kinematics.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

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

static chassis_kinematics_config_t verified_config(void)
{
    chassis_kinematics_config_t config = chassis_kinematics_default_config();

    config.left_motor_direction = 1;
    config.right_motor_direction = 1;
    config.direction_mapping_verified = true;
    config.maximum_motor_rpm = 10000.0;
    config.maximum_rpm_verified = true;
    return config;
}
static bool near(double actual, double expected, double tolerance)
{
    const double difference = actual - expected;
    return (difference >= -tolerance) && (difference <= tolerance);
}

static bool test_straight_turn_and_rotation(void)
{
    const chassis_kinematics_config_t config = verified_config();
    chassis_kinematics_output_t output;

    CHECK(chassis_kinematics_calculate(&config, 1000.0, 0.0, &output) ==
          CHASSIS_KINEMATICS_OK);
    CHECK(near(output.left_wheel_mm_s, 1000.0, 0.001));
    CHECK(near(output.right_wheel_mm_s, 1000.0, 0.001));
    CHECK(output.left_motor_rpm == 492);
    CHECK(output.right_motor_rpm == 492);

    CHECK(chassis_kinematics_calculate(&config, 1000.0, 1000.0, &output) ==
          CHASSIS_KINEMATICS_OK);
    CHECK(near(output.left_wheel_mm_s, 750.0, 0.001));
    CHECK(near(output.right_wheel_mm_s, 1250.0, 0.001));

    CHECK(chassis_kinematics_calculate(&config, 0.0, 1000.0, &output) ==
          CHASSIS_KINEMATICS_OK);
    CHECK(near(output.left_wheel_mm_s, -250.0, 0.001));
    CHECK(near(output.right_wheel_mm_s, 250.0, 0.001));
    return true;
}

static bool test_independent_directions_and_proportional_limit(void)
{
    chassis_kinematics_config_t config = verified_config();
    chassis_kinematics_output_t output;

    config.left_motor_direction = -1;
    CHECK(chassis_kinematics_calculate(&config, 1000.0, 0.0, &output) ==
          CHASSIS_KINEMATICS_OK);
    CHECK(output.left_motor_rpm == -492);
    CHECK(output.right_motor_rpm == 492);

    config.left_motor_direction = 1;
    config.maximum_motor_rpm = 100.0;
    CHECK(chassis_kinematics_calculate(&config, 1000.0, 500.0, &output) ==
          CHASSIS_KINEMATICS_OK);
    CHECK(output.limited);
    CHECK(output.right_motor_rpm == 100);
    CHECK(output.left_motor_rpm == 78);
    return true;
}

static bool test_nonzero_requires_verified_direction_and_limit(void)
{
    chassis_kinematics_config_t config = chassis_kinematics_default_config();
    chassis_kinematics_output_t output;

    CHECK(chassis_kinematics_calculate(&config, 0.0, 0.0, &output) ==
          CHASSIS_KINEMATICS_ZERO_SAFE);
    CHECK(output.left_motor_rpm == 0);
    CHECK(output.right_motor_rpm == 0);
    CHECK(chassis_kinematics_calculate(&config, 1.0, 0.0, &output) ==
          CHASSIS_KINEMATICS_DIRECTION_UNVERIFIED);

    config.direction_mapping_verified = true;
    config.left_motor_direction = 1;
    config.right_motor_direction = 1;
    CHECK(chassis_kinematics_calculate(&config, 1.0, 0.0, &output) ==
          CHASSIS_KINEMATICS_MAXIMUM_RPM_UNVERIFIED);
    return true;
}

static bool test_invalid_parameters_and_non_finite_inputs_are_rejected(void)
{
    chassis_kinematics_config_t config = verified_config();
    chassis_kinematics_output_t output;

    config.wheel_radius_mm = 0.0;
    CHECK(chassis_kinematics_calculate(&config, 1.0, 0.0, &output) ==
          CHASSIS_KINEMATICS_INVALID_CONFIG);
    config = verified_config();
    CHECK(chassis_kinematics_calculate(&config, NAN, 0.0, &output) ==
          CHASSIS_KINEMATICS_NUMERIC_ERROR);
    CHECK(chassis_kinematics_calculate(&config, 0.0, INFINITY, &output) ==
          CHASSIS_KINEMATICS_NUMERIC_ERROR);
    CHECK(chassis_kinematics_calculate(&config, DBL_MAX, DBL_MAX, &output) ==
          CHASSIS_KINEMATICS_NUMERIC_ERROR);
    return true;
}

static bool test_wheel_rc_command_envelope_matches_keil(void)
{
    chassis_kinematics_config_t config = verified_config();
    chassis_kinematics_output_t output;
    config.legacy_wheel_quantization = true;
    config.maximum_motor_rpm = 426.0;
    /* Golden commands from motor_velocity_to_rpm followed by motor_set_rpm. */
    const struct { int linear, angular, left, right; } commands[] = {
        {500, 0, 244, 244}, {-500, 0, -244, -244},
        {0, 1500, -182, 182}, {500, 1500, 57, 426},
        {-500, 1500, -426, -57}, {250, 751, 26, 213},
    };
    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i)
    {
        CHECK(chassis_kinematics_calculate(&config, commands[i].linear,
                                           commands[i].angular, &output) == CHASSIS_KINEMATICS_OK);
        CHECK(output.left_motor_rpm == commands[i].left);
        CHECK(output.right_motor_rpm == commands[i].right);
        CHECK(!output.limited);
    }
    return true;
}

int main(void)
{
    RUN(test_wheel_rc_command_envelope_matches_keil);
    RUN(test_straight_turn_and_rotation);
    RUN(test_independent_directions_and_proportional_limit);
    RUN(test_nonzero_requires_verified_direction_and_limit);
    RUN(test_invalid_parameters_and_non_finite_inputs_are_rejected);
    printf("all %u chassis_kinematics tests passed\n", tests_run);
    return 0;
}
