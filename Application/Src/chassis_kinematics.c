#include "chassis_kinematics.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define CHASSIS_PI 3.14159265358979323846

static bool direction_is_valid(int8_t direction)
{
    return (direction == -1) || (direction == 1);
}
static bool mechanical_config_is_valid(const chassis_kinematics_config_t *config)
{
    return isfinite(config->wheel_radius_mm) &&
           isfinite(config->half_track_mm) &&
           isfinite(config->gear_ratio) &&
           (config->wheel_radius_mm > 0.0) &&
           (config->half_track_mm > 0.0) &&
           (config->gear_ratio > 0.0) &&
           direction_is_valid(config->left_motor_direction) &&
           direction_is_valid(config->right_motor_direction);
}

static bool rounded_i32(double value, int32_t *result)
{
    double rounded;

    if (!isfinite(value) || (result == NULL) ||
        (value > (double)INT32_MAX) || (value < (double)INT32_MIN))
    {
        return false;
    }
    rounded = (value >= 0.0) ? value + 0.5 : value - 0.5;
    *result = (int32_t)rounded;
    return true;
}

chassis_kinematics_config_t chassis_kinematics_default_config(void)
{
    const chassis_kinematics_config_t config = {
        .wheel_radius_mm = 101.0,
        .half_track_mm = 250.0,
        .gear_ratio = 5.2,
        .left_motor_direction = 1,
        .right_motor_direction = 1,
        .direction_mapping_verified = false,
        .maximum_motor_rpm = 0.0,
        .maximum_rpm_verified = false,
    };

    return config;
}

bool chassis_kinematics_config_allows_nonzero(
    const chassis_kinematics_config_t *config)
{
    return (config != NULL) && mechanical_config_is_valid(config) &&
           config->direction_mapping_verified &&
           config->maximum_rpm_verified &&
           isfinite(config->maximum_motor_rpm) &&
           (config->maximum_motor_rpm > 0.0) &&
           (config->maximum_motor_rpm <= (double)INT32_MAX);
}

chassis_kinematics_status_t chassis_kinematics_calculate(
    const chassis_kinematics_config_t *config,
    double linear_mm_s,
    double angular_mrad_s,
    chassis_kinematics_output_t *output)
{
    double angular_component_mm_s;
    double circumference_mm;
    double left_motor_rpm;
    double right_motor_rpm;
    double peak_rpm;

    if ((config == NULL) || (output == NULL))
    {
        return CHASSIS_KINEMATICS_INVALID_ARGUMENT;
    }
    (void)memset(output, 0, sizeof(*output));
    if (!mechanical_config_is_valid(config))
    {
        return CHASSIS_KINEMATICS_INVALID_CONFIG;
    }
    if (!isfinite(linear_mm_s) || !isfinite(angular_mrad_s))
    {
        return CHASSIS_KINEMATICS_NUMERIC_ERROR;
    }
    if ((linear_mm_s == 0.0) && (angular_mrad_s == 0.0))
    {
        return CHASSIS_KINEMATICS_ZERO_SAFE;
    }
    if (!config->direction_mapping_verified)
    {
        return CHASSIS_KINEMATICS_DIRECTION_UNVERIFIED;
    }
    if (!config->maximum_rpm_verified ||
        !isfinite(config->maximum_motor_rpm) ||
        (config->maximum_motor_rpm <= 0.0) ||
        (config->maximum_motor_rpm > (double)INT32_MAX))
    {
        return CHASSIS_KINEMATICS_MAXIMUM_RPM_UNVERIFIED;
    }

    angular_component_mm_s = angular_mrad_s * config->half_track_mm / 1000.0;
    circumference_mm = 2.0 * CHASSIS_PI * config->wheel_radius_mm;
    if (config->legacy_wheel_quantization)
    {
        /* motor_velocity_to_rpm takes signed integral mm/s and mrad/s.
           Round the 101 mm wheel circumference to its original divisor, 635. */
        linear_mm_s = trunc(linear_mm_s);
        angular_component_mm_s = trunc(trunc(angular_mrad_s) * config->half_track_mm / 1000.0);
        circumference_mm = round(circumference_mm);
        if (circumference_mm < 1.0)
        {
            return CHASSIS_KINEMATICS_INVALID_CONFIG;
        }
    }
    output->left_wheel_mm_s = linear_mm_s - angular_component_mm_s;
    output->right_wheel_mm_s = linear_mm_s + angular_component_mm_s;
    output->left_wheel_rpm = output->left_wheel_mm_s * 60.0 / circumference_mm;
    output->right_wheel_rpm = output->right_wheel_mm_s * 60.0 / circumference_mm;
    if (config->legacy_wheel_quantization)
    {
        output->left_wheel_rpm = trunc(output->left_wheel_rpm);
        output->right_wheel_rpm = trunc(output->right_wheel_rpm);
    }
    left_motor_rpm = output->left_wheel_rpm * config->gear_ratio *
                     (double)config->left_motor_direction;
    right_motor_rpm = output->right_wheel_rpm * config->gear_ratio *
                      (double)config->right_motor_direction;
    if (config->legacy_wheel_quantization)
    {
        /* motor_set_rpm applies the reducer after wheel RPM quantization. */
        left_motor_rpm = trunc(left_motor_rpm);
        right_motor_rpm = trunc(right_motor_rpm);
    }
    if (!isfinite(left_motor_rpm) || !isfinite(right_motor_rpm))
    {
        (void)memset(output, 0, sizeof(*output));
        return CHASSIS_KINEMATICS_NUMERIC_ERROR;
    }

    peak_rpm = fabs(left_motor_rpm);
    if (fabs(right_motor_rpm) > peak_rpm)
    {
        peak_rpm = fabs(right_motor_rpm);
    }
    if (peak_rpm > config->maximum_motor_rpm)
    {
        const double scale = config->maximum_motor_rpm / peak_rpm;

        left_motor_rpm *= scale;
        right_motor_rpm *= scale;
        output->left_wheel_mm_s *= scale;
        output->right_wheel_mm_s *= scale;
        output->left_wheel_rpm *= scale;
        output->right_wheel_rpm *= scale;
        output->limited = true;
    }
    if (!rounded_i32(left_motor_rpm, &output->left_motor_rpm) ||
        !rounded_i32(right_motor_rpm, &output->right_motor_rpm))
    {
        (void)memset(output, 0, sizeof(*output));
        return CHASSIS_KINEMATICS_NUMERIC_ERROR;
    }
    return CHASSIS_KINEMATICS_OK;
}
