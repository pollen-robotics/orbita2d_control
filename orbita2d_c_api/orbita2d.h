#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

int32_t forward_position(double ratio_a,
                         double ratio_b,
                         double angle_a,
                         double angle_b,
                         double *roll,
                         double *pitch);

int32_t forward_velocity(double ratio_a,
                         double ratio_b,
                         double velocity_a,
                         double velocity_b,
                         double *roll_velocity,
                         double *pitch_velocity);

int32_t forward_torque(double ratio_a,
                       double ratio_b,
                       double torque_a,
                       double torque_b,
                       double *roll_torque,
                       double *pitch_torque);

int32_t inverse_position(double ratio_a,
                         double ratio_b,
                         double roll,
                         double pitch,
                         double *angle_a,
                         double *angle_b);

int32_t inverse_velocity(double ratio_a,
                         double ratio_b,
                         double roll_velocity,
                         double pitch_velocity,
                         double *velocity_a,
                         double *velocity_b);

int32_t inverse_torque(double ratio_a,
                       double ratio_b,
                       double roll_torque,
                       double pitch_torque,
                       double *torque_a,
                       double *torque_b);
