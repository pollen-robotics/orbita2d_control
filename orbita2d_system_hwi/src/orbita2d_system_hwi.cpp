#include "orbita2d_system_hwi/orbita2d_system_hwi.hpp"

#include "orbita2d.h"

#include <cmath>
#include <string>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"

std::vector<float> parse_string_as_vec(std::string s) {
  std::string delimiter = ",";

  std::vector<float> v;

  size_t pos = 0;
  std::string token;
  while ((pos = s.find(delimiter)) != std::string::npos) {
    token = s.substr(0, pos);
    v.push_back(std::stof(token));
    s.erase(0, pos + delimiter.length());
  }
  v.push_back(std::stof(s));

  return v;
}


namespace orbita2d_system_hwi
{
CallbackReturn
Orbita2dSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  long unsigned int nb_joints_expected = 2;
  if (info.joints.size() != nb_joints_expected)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Orbita2dSystem"),
                 "Incorrect number of joints, expected %ld, got \"%s\"", nb_joints_expected,
                 std::to_string(info.joints.size()).c_str()
      );
    return CallbackReturn::ERROR;
  }


  const char *config_file;
  bool from_config=false;

  for (auto const& params : info.hardware_parameters)
  {
    if (params.first == "config_file") {
      config_file = params.second.c_str();
      from_config=true;
    }
    //else, init with "manual" parameters
    else{
      RCLCPP_INFO(
        rclcpp::get_logger("Orbita2dSystem"),
        "Parameter \"%s\" = %s", params.first.c_str(),params.second.c_str()
      );
    }
  }

  if(from_config)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("Orbita2dSystem"),
      "Loading config file: %s",config_file
    );

    if(orbita2d_controller_from_config(config_file, &(this->uid)) != 0)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("Orbita2dSystem"),
        "Error loading config file: %s",config_file
      );
      return CallbackReturn::ERROR;
    }

  }
  else{
    //TODO
    return CallbackReturn::ERROR;
  }


  this->clock_ = rclcpp::Clock();
  RCLCPP_INFO(
    rclcpp::get_logger("Orbita2dSystem"),
    "System \"%s\" init!", info_.name.c_str()
  );

  return CallbackReturn::SUCCESS;
}

CallbackReturn
Orbita2dSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set some default values
  auto ret= CallbackReturn::SUCCESS;
  for (int i=0; i < 2; i++) {
    hw_states_position_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_velocity_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_effort_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_temperature_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_torque_limit_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_speed_limit_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_torque_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_p_gain_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_i_gain_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_d_gain_[i] = std::numeric_limits<double>::quiet_NaN();
  }


  if(orbita2d_get_target_orientation(this->uid, &hw_commands_position_)!=0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Orbita2dSystem"),
      "Error getting target orientation"
    );
    ret= CallbackReturn::ERROR;
  }
  bool torque_on=false;
  if(orbita2d_is_torque_on(this->uid, &torque_on)!=0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Orbita2dSystem"),
      "Error getting torque status"
    );
    ret= CallbackReturn::ERROR;
  }
  hw_commands_torque_[0] = torque_on;
  hw_commands_torque_[1] = torque_on;

  hw_states_torque_[0] = torque_on;
  hw_states_torque_[1] = torque_on;


  //init the states to the read values

    //Velocity
  if (orbita2d_get_current_velocity(this->uid, &hw_states_velocity_) != 0) {


    RCLCPP_ERROR(
      rclcpp::get_logger("Orbita2dSystem"),
      "(%s) READ VELOCITY ERROR!", info_.name.c_str()
      );
    ret= CallbackReturn::ERROR;
  }

  //Current torque
  if (orbita2d_get_current_torque(this->uid, &hw_states_effort_) != 0) {



    RCLCPP_ERROR(
      rclcpp::get_logger("Orbita2dSystem"),
      "(%s) READ CURRENT TORQUE ERROR!", info_.name.c_str()
      );
    ret= CallbackReturn::ERROR;
  }
  //Torque limit
  if (orbita2d_get_raw_motors_torque_limit(this->uid, &hw_states_torque_limit_) != 0) {



    RCLCPP_ERROR(
      rclcpp::get_logger("Orbita2dSystem"),
      "(%s) READ TORQUE LIMIT ERROR!", info_.name.c_str()
      );
    ret= CallbackReturn::ERROR;
  }

  //velocity limit
  if (orbita2d_get_raw_motors_velocity_limit(this->uid, &hw_states_speed_limit_) != 0) {



    RCLCPP_ERROR(
      rclcpp::get_logger("Orbita2dSystem"),
      "(%s) READ SPEED LIMIT ERROR!", info_.name.c_str()
      );
    ret= CallbackReturn::ERROR;
  }

  //PID gains
  double pids[6];
  if (orbita2d_get_raw_motors_pid_gains(this->uid, &pids) != 0) {



    RCLCPP_ERROR(
      rclcpp::get_logger("Orbita2dSystem"),
      "(%s) READ PID GAINS ERROR!", info_.name.c_str()
      );
    ret= CallbackReturn::ERROR;
  }
  else{
    hw_states_p_gain_[0] = pids[0];
    hw_states_i_gain_[0] = pids[1];
    hw_states_d_gain_[0] = pids[2];
    hw_states_p_gain_[1] = pids[3];
    hw_states_i_gain_[1] = pids[4];
    hw_states_d_gain_[1] = pids[5];
  }

  if (orbita2d_get_current_orientation(this->uid, &hw_states_position_) != 0) {

    // ret=hardware_interface::return_type::ERROR;
    ret= CallbackReturn::ERROR;
    RCLCPP_ERROR(
      rclcpp::get_logger("Orbita2dSystem"),
      "(%s) READ ORIENTATION ERROR!", info_.name.c_str()
      );
  }

  //set the commands to the read values (otherwise some strange behaviour can happen)
  for (int i=0; i < 2; i++) {
    hw_commands_position_[i]=hw_states_position_[i];
    hw_commands_torque_limit_[i]=hw_states_torque_limit_[i];
    hw_commands_speed_limit_[i]=hw_states_speed_limit_[i];
    hw_commands_torque_[i]=hw_states_torque_[i];
    hw_commands_p_gain_[i]=hw_states_p_gain_[i];
    hw_commands_i_gain_[i]=hw_states_i_gain_[i];
    hw_commands_d_gain_[i]=hw_states_d_gain_[i];
  }


  this->last_timestamp_ = clock_.now();

  RCLCPP_INFO(
    rclcpp::get_logger("Orbita2dSystem"),
    "System \"%s\" successfully started!", info_.name.c_str()
  );
  return ret;
}

CallbackReturn
Orbita2dSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{

  //TODO: disable torque?

  RCLCPP_INFO(
    rclcpp::get_logger("Orbita2dSystem"),
    "System \"%s\" successfully deactivated!", info_.name.c_str()
  );
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Orbita2dSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (std::size_t i = 0; i < 2; i++)
  {
    auto joint = info_.joints[i];

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "temperature", &hw_states_temperature_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "torque_limit", &hw_states_torque_limit_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "speed_limit", &hw_states_speed_limit_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "torque", &hw_states_torque_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "p_gain", &hw_states_p_gain_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "i_gain", &hw_states_i_gain_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "d_gain", &hw_states_d_gain_[i]));

    RCLCPP_INFO(
      rclcpp::get_logger("Orbita2dSystem"),
      "export state interface (%s) \"%s\"!", info_.name.c_str(), joint.name.c_str()
      );
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Orbita2dSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < 2; i++)
  {
    auto joint = info_.joints[i];

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, "speed_limit", &hw_commands_speed_limit_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, "torque_limit", &hw_commands_torque_limit_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, "torque", &hw_commands_torque_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, "p_gain", &hw_commands_p_gain_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, "i_gain", &hw_commands_i_gain_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, "d_gain", &hw_commands_d_gain_[i]));

    RCLCPP_INFO(
      rclcpp::get_logger("Orbita2dSystem"),
      "export command interface (%s) \"%s\"!", info_.name.c_str(), joint.name.c_str()
      );
  }

  return command_interfaces;
}

hardware_interface::return_type
Orbita2dSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  current_timestamp = clock_.now();
  rclcpp::Duration duration = current_timestamp - last_timestamp_;
  last_timestamp_ = current_timestamp;
  auto ret=hardware_interface::return_type::OK;

  //Position
  if (orbita2d_get_current_orientation(this->uid, &hw_states_position_) != 0) {

    // ret=hardware_interface::return_type::ERROR;

    RCLCPP_ERROR(
      rclcpp::get_logger("Orbita2dSystem"),
      "(%s) READ ORIENTATION ERROR!", info_.name.c_str()
      );
  }

  //Torque on/off
  bool torque_on=false;
  if(orbita2d_is_torque_on(this->uid, &torque_on)!=0)
  {
    // ret=hardware_interface::return_type::ERROR;

    RCLCPP_ERROR(
      rclcpp::get_logger("Orbita2dSystem"),
      "(%s) Error getting torque status!",info_.name.c_str()
    );
  }
  hw_states_torque_[0] = torque_on;
  hw_states_torque_[1] = torque_on;


  //Velocity
  if (orbita2d_get_current_velocity(this->uid, &hw_states_velocity_) != 0) {

    // ret=hardware_interface::return_type::ERROR;

    RCLCPP_ERROR(
      rclcpp::get_logger("Orbita2dSystem"),
      "(%s) READ VELOCITY ERROR!", info_.name.c_str()
      );
  }

  //Current torque
  if (orbita2d_get_current_torque(this->uid, &hw_states_effort_) != 0) {

    // ret=hardware_interface::return_type::ERROR;

    RCLCPP_ERROR(
      rclcpp::get_logger("Orbita2dSystem"),
      "(%s) READ CURRENT TORQUE ERROR!", info_.name.c_str()
      );
  }
  //Torque limit
  if (orbita2d_get_raw_motors_torque_limit(this->uid, &hw_states_torque_limit_) != 0) {

    // ret=hardware_interface::return_type::ERROR;

    RCLCPP_ERROR(
      rclcpp::get_logger("Orbita2dSystem"),
      "(%s) READ TORQUE LIMIT ERROR!", info_.name.c_str()
      );
  }

  //velocity limit
  if (orbita2d_get_raw_motors_velocity_limit(this->uid, &hw_states_speed_limit_) != 0) {

    // ret=hardware_interface::return_type::ERROR;

    RCLCPP_ERROR(
      rclcpp::get_logger("Orbita2dSystem"),
      "(%s) READ SPEED LIMIT ERROR!", info_.name.c_str()
      );
  }

  //PID gains
  double pids[6];
  if (orbita2d_get_raw_motors_pid_gains(this->uid, &pids) != 0) {

    // ret=hardware_interface::return_type::ERROR;

    RCLCPP_ERROR(
      rclcpp::get_logger("Orbita2dSystem"),
      "(%s) READ PID GAINS ERROR!", info_.name.c_str()
      );
  }
  else{
    hw_states_p_gain_[0] = pids[0];
    hw_states_i_gain_[0] = pids[1];
    hw_states_d_gain_[0] = pids[2];
    hw_states_p_gain_[1] = pids[3];
    hw_states_i_gain_[1] = pids[4];
    hw_states_d_gain_[1] = pids[5];
  }

  //No temperature for now in Flipsky

  return ret;
}

hardware_interface::return_type
Orbita2dSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  auto ret=hardware_interface::return_type::OK;
    // RCLCPP_INFO(
    //   rclcpp::get_logger("Orbita2dSystem"),
    //   "(%s) WRITE ORIENTATION %f %f", info_.name.c_str(), hw_commands_position_[0], hw_commands_position_[1]

    //   );

  if (orbita2d_set_target_orientation(this->uid, &hw_commands_position_) != 0) {
    // ret=hardware_interface::return_type::ERROR;

    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("Orbita2dSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) WRITE ORIENTATION LIMIT ERROR!", info_.name.c_str()
      );
  }


  //We only change torque for both axes

  bool torque= (hw_commands_torque_[0]==1.0 && hw_commands_torque_[1]==1.0);
  //TODO: we can go with orbita2d_set_raw_torque_limit
  if(torque)
  {
    if (orbita2d_enable_torque(this->uid, false) != 0) { //do not reset target
      // ret=hardware_interface::return_type::ERROR;

      RCLCPP_ERROR_THROTTLE(
        rclcpp::get_logger("Orbita2dSystem"),
        clock_,
        LOG_THROTTLE_DURATION,
        "(%s) WRITE TORQUE ERROR!", info_.name.c_str()
        );
    }
  }
  else{
      if (orbita2d_disable_torque(this->uid) != 0) {
        // ret=hardware_interface::return_type::ERROR;

        RCLCPP_ERROR_THROTTLE(
          rclcpp::get_logger("Orbita2dSystem"),
          clock_,
          LOG_THROTTLE_DURATION,
          "(%s) WRITE TORQUE ERROR!", info_.name.c_str()
          );
      }
  }


  //speed limit
  //TODO

  // if(orbita2d_set_raw_motors_velocity_limit(this->uid, &hw_commands_speed_limit_) != 0)
  // {
  //   ret=hardware_interface::return_type::ERROR;

  //   RCLCPP_ERROR(
  //     rclcpp::get_logger("Orbita2dSystem"),
  //     "(%s) WRITE SPEED LIMIT ERROR!", info_.name.c_str()
  //     );
  // }

  // //torque limit
  // if(orbita2d_set_raw_motors_torque_limit(this->uid, &hw_commands_torque_limit_) != 0)
  // {
  //   ret=hardware_interface::return_type::ERROR;

  //   RCLCPP_ERROR(
  //     rclcpp::get_logger("Orbita2dSystem"),
  //     "(%s) WRITE TORQUE LIMIT ERROR!", info_.name.c_str()
  //     );
  // }

  // //pid gains
  // double pids[6];
  // pids[0] = hw_commands_p_gain_[0];
  // pids[1] = hw_commands_i_gain_[0];
  // pids[2] = hw_commands_d_gain_[0];
  // pids[3] = hw_commands_p_gain_[1];
  // pids[4] = hw_commands_i_gain_[1];
  // pids[5] = hw_commands_d_gain_[1];
  // if(orbita2d_set_raw_motors_pid_gains(this->uid, &pids) != 0)
  // {
  //   ret=hardware_interface::return_type::ERROR;

  //   RCLCPP_ERROR(
  //     rclcpp::get_logger("Orbita2dSystem"),
  //     "(%s) WRITE PID GAINS ERROR!", info_.name.c_str()
  //     );
  // }



  return ret;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  orbita2d_system_hwi::Orbita2dSystem,
  hardware_interface::SystemInterface)
