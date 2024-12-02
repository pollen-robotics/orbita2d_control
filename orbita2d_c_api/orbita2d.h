#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

extern "C" {

uint32_t orbita2d_controller_with_flipsky_serial(const char *serial_port_a,
                                                 const char *serial_port_b,
                                                 uint8_t id_a,
                                                 uint8_t id_b,
                                                 double offset_a,
                                                 double offset_b,
                                                 double ratio_a,
                                                 double ratio_b,
                                                 bool ring_inverted,
                                                 bool center_inverted,
                                                 double lower_limit_a,
                                                 double upper_limit_a,
                                                 double lower_limit_b,
                                                 double upper_limit_b,
                                                 bool use_cache,
                                                 uint32_t *uid);

uint32_t orbita2d_controller_from_config(const char *configfile, uint32_t *uid);

uint32_t orbita2d_is_torque_on(uint32_t uid, bool *is_on);

uint32_t orbita2d_enable_torque(uint32_t uid, bool reset_target);

uint32_t orbita2d_disable_torque(uint32_t uid);

uint32_t orbita2d_get_current_orientation(uint32_t uid, double (*pos)[2]);

uint32_t orbita2d_get_current_velocity(uint32_t uid, double (*vel)[2]);

uint32_t orbita2d_get_current_torque(uint32_t uid, double (*torque)[2]);

uint32_t orbita2d_get_target_orientation(uint32_t uid, double (*pos)[2]);

uint32_t orbita2d_set_target_orientation(uint32_t uid, const double (*pos)[2]);

uint32_t orbita2d_get_raw_motors_velocity_limit(uint32_t uid,
                                                double (*raw_motors_velocity_limit)[2]);

uint32_t orbita2d_set_raw_motors_velocity_limit(uint32_t uid,
                                                const double (*raw_motors_velocity_limit)[2]);

uint32_t orbita2d_get_raw_motors_torque_limit(uint32_t uid, double (*raw_motors_torque_limit)[2]);

uint32_t orbita2d_set_raw_motors_torque_limit(uint32_t uid,
                                              const double (*raw_motors_torque_limit)[2]);

uint32_t orbita2d_get_raw_motors_pid_gains(uint32_t uid, double (*pids)[6]);

uint32_t orbita2d_set_raw_motors_pid_gains(uint32_t uid, const double (*pids)[6]);

int32_t orbita2d_get_board_state(uint32_t uid, uint8_t *state);

int32_t orbita2d_set_board_state(uint32_t uid, const uint8_t *state);

int32_t orbita2d_forward_position(double ratio_a,
                                  double ratio_b,
                                  double angle_a,
                                  double angle_b,
                                  double *ring,
                                  double *center);

int32_t orbita2d_forward_velocity(double ratio_a,
                                  double ratio_b,
                                  double velocity_a,
                                  double velocity_b,
                                  double *ring_velocity,
                                  double *center_velocity);

int32_t orbita2d_forward_torque(double ratio_a,
                                double ratio_b,
                                double torque_a,
                                double torque_b,
                                double *ring_torque,
                                double *center_torque);

int32_t orbita2d_inverse_position(double ratio_a,
                                  double ratio_b,
                                  double ring,
                                  double center,
                                  double *angle_a,
                                  double *angle_b);

int32_t orbita2d_inverse_velocity(double ratio_a,
                                  double ratio_b,
                                  double ring_velocity,
                                  double center_velocity,
                                  double *velocity_a,
                                  double *velocity_b);

int32_t orbita2d_inverse_torque(double ratio_a,
                                double ratio_b,
                                double ring_torque,
                                double center_torque,
                                double *torque_a,
                                double *torque_b);

} // extern "C"
