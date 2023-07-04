#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

uint32_t orbita2d_controller_with_flipsky_serial(const char *serial_port_1,
                                                 const char *serial_port_2,
                                                 uint8_t id_1,
                                                 uint8_t id_2,
                                                 double offset_1,
                                                 double offset_2,
                                                 double ratio_1,
                                                 double ratio_2,
                                                 double lower_limit_1,
                                                 double upper_limit_1,
                                                 double lower_limit_2,
                                                 double upper_limit_2,
                                                 uint32_t *uid);

uint32_t orbita2d_is_torque_on(uint32_t uid, bool *is_on);

uint32_t orbita2d_enable_torque(uint32_t uid, bool reset_target);

uint32_t orbita2d_disable_torque(uint32_t uid);

uint32_t orbita2d_get_current_orientation(uint32_t uid, double (*pos)[2]);

uint32_t orbita2d_get_current_velocity(uint32_t uid, double (*vel)[2]);

uint32_t orbita2d_get_current_torque(uint32_t uid, double (*torque)[2]);

uint32_t orbita2d_get_target_orientation(uint32_t uid, double (*pos)[2]);

uint32_t orbita2d_set_target_orientation(uint32_t uid, const double (*pos)[2]);

uint32_t orbita2d_get_velocity_limit(uint32_t uid, double (*vel_limit)[2]);

uint32_t orbita2d_set_velocity_limit(uint32_t uid, const double (*vel_limit)[2]);

uint32_t orbita2d_get_torque_limit(uint32_t uid, double (*torque_limit)[2]);

uint32_t orbita2d_set_torque_limit(uint32_t uid, const double (*torque_limit)[2]);

uint32_t orbita2d_get_pid_gains(uint32_t uid, double *kp, double *ki, double *kd);

uint32_t orbita2d_set_pid_gains(uint32_t uid, double kp, double ki, double kd);

int32_t orbita2d_forward_position(double ratio_a,
                                  double ratio_b,
                                  double angle_a,
                                  double angle_b,
                                  double *roll,
                                  double *pitch);

int32_t orbita2d_forward_velocity(double ratio_a,
                                  double ratio_b,
                                  double velocity_a,
                                  double velocity_b,
                                  double *roll_velocity,
                                  double *pitch_velocity);

int32_t orbita2d_forward_torque(double ratio_a,
                                double ratio_b,
                                double torque_a,
                                double torque_b,
                                double *roll_torque,
                                double *pitch_torque);

int32_t orbita2d_inverse_position(double ratio_a,
                                  double ratio_b,
                                  double roll,
                                  double pitch,
                                  double *angle_a,
                                  double *angle_b);

int32_t orbita2d_inverse_velocity(double ratio_a,
                                  double ratio_b,
                                  double roll_velocity,
                                  double pitch_velocity,
                                  double *velocity_a,
                                  double *velocity_b);

int32_t orbita2d_inverse_torque(double ratio_a,
                                double ratio_b,
                                double roll_torque,
                                double pitch_torque,
                                double *torque_a,
                                double *torque_b);
