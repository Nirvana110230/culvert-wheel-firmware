#ifndef CHASSIS_KINEMATICS_H
#define CHASSIS_KINEMATICS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CHASSIS_KINEMATICS_OK = 0,
    CHASSIS_KINEMATICS_ZERO_SAFE,
    CHASSIS_KINEMATICS_INVALID_ARGUMENT,
    CHASSIS_KINEMATICS_INVALID_CONFIG,
    CHASSIS_KINEMATICS_DIRECTION_UNVERIFIED,
    CHASSIS_KINEMATICS_MAXIMUM_RPM_UNVERIFIED,
    CHASSIS_KINEMATICS_NUMERIC_ERROR
} chassis_kinematics_status_t;

typedef struct
{
    double wheel_radius_mm;
    double half_track_mm;
    double gear_ratio;
    int8_t left_motor_direction;
    int8_t right_motor_direction;
    bool direction_mapping_verified;
    double maximum_motor_rpm;
    bool maximum_rpm_verified;
    /* Original wheel firmware truncates tangential speed, wheel RPM and shaft RPM. */
    bool legacy_wheel_quantization;
} chassis_kinematics_config_t;

typedef struct
{
    double left_wheel_mm_s;
    double right_wheel_mm_s;
    double left_wheel_rpm;
    double right_wheel_rpm;
    int32_t left_motor_rpm;
    int32_t right_motor_rpm;
    bool limited;
} chassis_kinematics_output_t;

/* Returns the known mechanical baseline with both field-verification gates off. */
chassis_kinematics_config_t chassis_kinematics_default_config(void);

/* Inputs are body linear mm/s and angular mrad/s; outputs include shaft RPM. */
chassis_kinematics_status_t chassis_kinematics_calculate(
    const chassis_kinematics_config_t *config,
    double linear_mm_s,
    double angular_mrad_s,
    chassis_kinematics_output_t *output);

bool chassis_kinematics_config_allows_nonzero(
    const chassis_kinematics_config_t *config);

#endif
