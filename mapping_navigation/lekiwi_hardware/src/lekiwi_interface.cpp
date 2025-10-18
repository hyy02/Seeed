#include "lekiwi_hardware/lekiwi_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <sstream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace lekiwi_controller
{
LeKiwiInterface::LeKiwiInterface()
  : SerialPort(-1), use_serial_(false), serial_baudrate_(1000000), torque_enabled_(true)
{
}

LeKiwiInterface::~LeKiwiInterface()
{
  if (use_serial_)
  {
    st3215_.end();
  }
  if (executor_)
  {
    executor_->cancel();
  }
  if (spin_thread_.joinable())
  {
    spin_thread_.join();
  }
}

CallbackReturn LeKiwiInterface::on_init(const hardware_interface::HardwareInfo& hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  use_serial_ = hardware_info.hardware_parameters.count("use_serial") ?
                    (hardware_info.hardware_parameters.at("use_serial") == "true") :
                    false;

  serial_port_ = hardware_info.hardware_parameters.count("serial_port") ?
                     hardware_info.hardware_parameters.at("serial_port") :
                     "/dev/ttyACM0";

  serial_baudrate_ = hardware_info.hardware_parameters.count("serial_baudrate") ?
                         std::stoi(hardware_info.hardware_parameters.at("serial_baudrate")) :
                         1000000;

  std::string joint_offsets_file = hardware_info.hardware_parameters.count("joint_offsets_file") ?
                                       hardware_info.hardware_parameters.at("joint_offsets_file") :
                                       "";

  if (!joint_offsets_file.empty())
  {
    if (!load_joint_offsets(joint_offsets_file))
    {
      RCLCPP_WARN(rclcpp::get_logger("LeKiwiInterface"), "Failed to load joint offsets file: %s, using defaults",
                  joint_offsets_file.c_str());
    }
  }

  calculate_zero_positions();

  size_t num_joints = info_.joints.size();
  position_commands_.resize(num_joints, 0.0);
  position_states_.resize(num_joints, 0.0);
  velocity_commands_.resize(num_joints, 0.0);
  velocity_states_.resize(num_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("LeKiwiInterface"), "Initialized hardware interface with %zu joints", num_joints);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LeKiwiInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    const std::string& joint_name = info_.joints[i].name;

    state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &position_states_[i]);

    if (joint_name.find("wheel") != std::string::npos)
    {
      state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LeKiwiInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    const std::string& joint_name = info_.joints[i].name;

    if (joint_name.find("wheel") == std::string::npos)
    {
      command_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &position_commands_[i]);
    }
    else
    {
      command_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]);
    }
  }
  return command_interfaces;
}

CallbackReturn LeKiwiInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("LeKiwiInterface"), "Activating LeKiwi hardware interface...");

  if (use_serial_)
  {
    if (!st3215_.begin(serial_baudrate_, serial_port_.c_str()))
    {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiInterface"), "Failed to initialize motors");
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("LeKiwiInterface"), "Serial communication initialized on %s", serial_port_.c_str());
  }

  node_ = rclcpp::Node::make_shared("lekiwi_driver");
  feedback_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "feedback", 10, std::bind(&LeKiwiInterface::feedback_callback, this, std::placeholders::_1));
  command_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("command", 10);

  torque_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "torque", std::bind(&LeKiwiInterface::torque_callback, this, std::placeholders::_1, std::placeholders::_2));

  calib_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "calibrate",
      std::bind(&LeKiwiInterface::calibration_callback, this, std::placeholders::_1, std::placeholders::_2));

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });

  std::string calib_file =
      info_.hardware_parameters.count("calibration_file") ? info_.hardware_parameters.at("calibration_file") : "";

  if (!calib_file.empty())
  {
    if (!load_calibration(calib_file))
    {
      RCLCPP_WARN(rclcpp::get_logger("LeKiwiInterface"), "Failed to load calibration file: %s", calib_file.c_str());
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("LeKiwiInterface"), "Hardware interface activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LeKiwiInterface::on_deactivate(const rclcpp_lifecycle::State&)
{
  if (executor_)
  {
    executor_->cancel();
  }
  if (spin_thread_.joinable())
  {
    spin_thread_.join();
  }

  if (use_serial_)
  {
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      uint8_t servo_id = static_cast<uint8_t>(i + 1);
      st3215_.EnableTorque(servo_id, 0);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("LeKiwiInterface"), "Hardware interface deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void LeKiwiInterface::feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  last_feedback_msg_ = msg;
}

hardware_interface::return_type LeKiwiInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  if (use_serial_ && torque_enabled_)
  {  // Only write if torque is enabled
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      uint8_t servo_id = static_cast<uint8_t>(i + 1);

      if (servo_id <= 6)
      {
        int joint_pos_cmd = radians_to_ticks(position_commands_[i], i);

        RCLCPP_DEBUG(rclcpp::get_logger("LeKiwiInterface"), "ARM Servo %d command: %.2f rad -> %d ticks", servo_id,
                     position_commands_[i], joint_pos_cmd);

        if (!st3215_.RegWritePosEx(servo_id, joint_pos_cmd, 2000, 255))
        {
          RCLCPP_WARN(rclcpp::get_logger("LeKiwiInterface"), "Failed to write position to arm servo %d", servo_id);
        }
      }
      else
      {
        double velocity_rad_s = velocity_commands_[i];

        double velocity_deg_s = velocity_rad_s * (180.0 / M_PI);
        double steps_per_deg = 4096.0 / 360.0;
        int16_t wheel_speed = static_cast<int16_t>(velocity_deg_s * steps_per_deg);

        wheel_speed *= servo_directions_[i];

        RCLCPP_DEBUG(rclcpp::get_logger("LeKiwiInterface"),
                     "WHEEL Servo %d velocity: %.3f rad/s -> %.1f deg/s -> %d raw", servo_id, velocity_rad_s,
                     velocity_deg_s, wheel_speed);

        if (!st3215_.WriteSpe(servo_id, wheel_speed, 50))
        {
          RCLCPP_WARN(rclcpp::get_logger("LeKiwiInterface"), "Failed to write velocity to wheel servo %d", servo_id);
        }
      }
    }

    st3215_.RegWriteAction();
  }

  if (command_publisher_)
  {
    sensor_msgs::msg::JointState cmd_msg;
    cmd_msg.header.stamp = node_->now();

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      cmd_msg.name.push_back(info_.joints[i].name);
      cmd_msg.position.push_back(position_commands_[i]);
      cmd_msg.velocity.push_back(velocity_commands_[i]);
    }

    command_publisher_->publish(cmd_msg);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LeKiwiInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  if (use_serial_)
  {
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      uint8_t servo_id = static_cast<uint8_t>(i + 1);

      std::this_thread::sleep_for(std::chrono::milliseconds(1));

      if (!torque_enabled_)
      {
        int raw_pos = st3215_.ReadPos(servo_id);
        if (raw_pos != -1)
        {
          position_states_[i] = ticks_to_radians(raw_pos, i);
        }
        continue;
      }

      if (st3215_.FeedBack(servo_id) != -1)
      {
        int raw_pos = st3215_.ReadPos(servo_id);
        position_states_[i] = ticks_to_radians(raw_pos, i);

        int raw_speed = st3215_.ReadSpeed(servo_id);
        double speed_deg_s = raw_speed * (360.0 / 4096.0);
        velocity_states_[i] = speed_deg_s * (M_PI / 180.0) * servo_directions_[i];

        double pwm = -1 * st3215_.ReadLoad(servo_id) / 10.0;
        int move = st3215_.ReadMove(servo_id);
        double temperature = st3215_.ReadTemper(servo_id);
        double voltage = st3215_.ReadVoltage(servo_id) / 10;
        double current = st3215_.ReadCurrent(servo_id) * 6.5 / 1000;

        RCLCPP_DEBUG(rclcpp::get_logger("LeKiwiInterface"),
                     "Servo %d: pos=%.2f rad, vel=%.3f rad/s, pwm=%.2f, temp=%.1f°C, V=%.1f, I=%.3f", servo_id,
                     position_states_[i], velocity_states_[i], pwm, temperature, voltage, current);
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("LeKiwiInterface"), "Failed to read feedback from servo %d", servo_id);
      }
    }
  }
  else
  {
    sensor_msgs::msg::JointState::SharedPtr feedback_copy;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      feedback_copy = last_feedback_msg_;
    }

    if (feedback_copy)
    {
      for (size_t i = 0; i < info_.joints.size(); ++i)
      {
        auto it = std::find(feedback_copy->name.begin(), feedback_copy->name.end(), info_.joints[i].name);
        if (it != feedback_copy->name.end())
        {
          size_t idx = std::distance(feedback_copy->name.begin(), it);
          if (idx < feedback_copy->position.size())
          {
            position_states_[i] = ticks_to_radians(feedback_copy->position[idx], i);
          }
        }
      }
    }
  }

  return hardware_interface::return_type::OK;
}

void LeKiwiInterface::calibrate_servo(uint8_t servo_id, int current_pos)
{
  size_t idx = servo_id - 1;
  int offset = current_pos - zero_positions_[idx];
  RCLCPP_INFO(rclcpp::get_logger("LeKiwiInterface"), "Servo %d: current=%d, zero=%d, offset=%d", servo_id, current_pos,
              zero_positions_[idx], offset);
}

double LeKiwiInterface::ticks_to_radians(int ticks, size_t servo_idx)
{
  if (servo_idx >= zero_positions_.size() || servo_idx >= servo_directions_.size())
  {
    return 0.0;
  }

  int zero_pos = zero_positions_[servo_idx];
  double angle_ticks = static_cast<double>(ticks - zero_pos);

  angle_ticks *= servo_directions_[servo_idx];

  return (angle_ticks / 4096.0) * 2.0 * M_PI;
}

int LeKiwiInterface::radians_to_ticks(double radians, size_t servo_idx)
{
  if (servo_idx >= zero_positions_.size() || servo_idx >= servo_directions_.size())
  {
    return 2048;
  }

  int zero_pos = zero_positions_[servo_idx];
  double angle_ticks = (radians / (2.0 * M_PI)) * 4096.0;

  angle_ticks *= servo_directions_[servo_idx];

  return zero_pos + static_cast<int>(angle_ticks);
}

void LeKiwiInterface::record_current_position()
{
  std::stringstream ss;
  ss << "{";

  bool first = true;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    uint8_t servo_id = static_cast<uint8_t>(i + 1);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    int pos = -1;
    for (int retry = 0; retry < 3 && pos == -1; retry++)
    {
      st3215_.FeedBack(servo_id);
      pos = st3215_.ReadPos(servo_id);
      if (pos == -1)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }

    if (!first)
    {
      ss << ",";
    }
    first = false;

    ss << "\"" << info_.joints[i].name << "\": {"
       << "\"ticks\": " << (pos != -1 ? pos : 0) << ","
       << "\"speed\": " << st3215_.ReadSpeed(servo_id) << ","
       << "\"load\": " << st3215_.ReadLoad(servo_id) << "}";
  }
  ss << "}";

  last_calibration_data_ = ss.str();
  RCLCPP_INFO(rclcpp::get_logger("LeKiwiInterface"), "Recorded positions: %s", last_calibration_data_.c_str());
}

void LeKiwiInterface::calibration_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                           std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  record_current_position();
  response->success = true;
  response->message = last_calibration_data_;
}

void LeKiwiInterface::set_torque_enable(bool enable)
{
  if (use_serial_)
  {
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      uint8_t servo_id = static_cast<uint8_t>(i + 1);

      if (!enable)
      {
        st3215_.Mode(servo_id, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        st3215_.EnableTorque(servo_id, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        st3215_.EnableTorque(servo_id, 0);
      }
      else
      {
        st3215_.Mode(servo_id, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        st3215_.EnableTorque(servo_id, 1);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    torque_enabled_ = enable;

    RCLCPP_INFO(rclcpp::get_logger("LeKiwiInterface"), "Torque %s for all servos", enable ? "enabled" : "disabled");
  }
}

void LeKiwiInterface::torque_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  bool new_state = !torque_enabled_;

  response->success = true;
  response->message = std::string("Torque ") + (new_state ? "enabled" : "disabled");

  set_torque_enable(new_state);

  RCLCPP_INFO(rclcpp::get_logger("LeKiwiInterface"), "Torque service called, response: %s", response->message.c_str());
}

bool LeKiwiInterface::load_calibration(const std::string& filepath)
{
  try
  {
    YAML::Node config = YAML::LoadFile(filepath);
    auto joints = config["joints"];
    if (!joints)
    {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiInterface"), "No joints section in calibration file");
      return false;
    }

    for (const auto& joint : joints)
    {
      std::string name = joint.first.as<std::string>();
      const auto& data = joint.second;

      if (!data["min"] || !data["center"] || !data["max"])
      {
        RCLCPP_ERROR(rclcpp::get_logger("LeKiwiInterface"), "Missing calibration data for joint %s", name.c_str());
        continue;
      }

      JointCalibration calib;
      calib.min_ticks = data["min"]["ticks"].as<int>();
      calib.center_ticks = data["center"]["ticks"].as<int>();
      calib.max_ticks = data["max"]["ticks"].as<int>();
      calib.range_ticks = calib.max_ticks - calib.min_ticks;

      joint_calibration_[name] = calib;

      RCLCPP_INFO(rclcpp::get_logger("LeKiwiInterface"), "Loaded calibration for %s: min=%d, center=%d, max=%d",
                  name.c_str(), calib.min_ticks, calib.center_ticks, calib.max_ticks);
    }
    return true;
  }
  catch (const YAML::Exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("LeKiwiInterface"), "Failed to load calibration: %s", e.what());
    return false;
  }
}

double LeKiwiInterface::normalize_position(const std::string& joint_name, int ticks)
{
  if (joint_calibration_.count(joint_name) == 0)
  {
    return 0.0;
  }

  const auto& calib = joint_calibration_[joint_name];
  double normalized = (ticks - calib.min_ticks) / calib.range_ticks;
  return std::clamp(normalized, 0.0, 1.0);
}

bool LeKiwiInterface::load_joint_offsets(const std::string& filepath)
{
  try
  {
    YAML::Node config = YAML::LoadFile(filepath);

    if (!config["joint_offsets"])
    {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiInterface"), "No 'joint_offsets' section found in config file");
      return false;
    }

    auto joint_offsets = config["joint_offsets"];

    int zero_base = joint_offsets["zero_position_base"] ? joint_offsets["zero_position_base"].as<int>() : 2048;

    std::vector<std::string> joint_names = { "shoulder_rotation", "shoulder_pitch",   "elbow",
                                             "wrist_pitch",       "wrist_roll",       "gripper",
                                             "left_wheel_drive",  "rear_wheel_drive", "right_wheel_drive" };

    for (const auto& joint_name : joint_names)
    {
      JointOffsetConfig config_entry;
      config_entry.zero_position_base = zero_base;
      config_entry.angle_offset_deg = 0.0;

      if (joint_offsets[joint_name] && joint_offsets[joint_name]["angle_offset_deg"])
      {
        config_entry.angle_offset_deg = joint_offsets[joint_name]["angle_offset_deg"].as<double>();
      }

      joint_offset_config_[joint_name] = config_entry;

      RCLCPP_INFO(rclcpp::get_logger("LeKiwiInterface"), "Loaded offset for %s: %.1f degrees", joint_name.c_str(),
                  config_entry.angle_offset_deg);
    }

    return true;
  }
  catch (const YAML::Exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("LeKiwiInterface"), "Error loading joint offsets config: %s", e.what());
    return false;
  }
}

void LeKiwiInterface::calculate_zero_positions()
{
  std::vector<std::string> joint_names = { "shoulder_rotation", "shoulder_pitch",   "elbow",
                                           "wrist_pitch",       "wrist_roll",       "gripper",
                                           "left_wheel_drive",  "rear_wheel_drive", "right_wheel_drive" };

  zero_positions_.clear();
  zero_positions_.reserve(joint_names.size());

  for (const auto& joint_name : joint_names)
  {
    int zero_pos = 2048;

    if (joint_offset_config_.find(joint_name) != joint_offset_config_.end())
    {
      const auto& config = joint_offset_config_[joint_name];
      int offset_ticks = static_cast<int>(config.angle_offset_deg / 360.0 * 4096);
      zero_pos = config.zero_position_base + offset_ticks;

      RCLCPP_INFO(rclcpp::get_logger("LeKiwiInterface"),
                  "Calculated zero position for %s: %d (offset: %.1f deg, %d ticks)", joint_name.c_str(), zero_pos,
                  config.angle_offset_deg, offset_ticks);
    }

    zero_positions_.push_back(zero_pos);
  }
}

}  // namespace lekiwi_controller

PLUGINLIB_EXPORT_CLASS(lekiwi_controller::LeKiwiInterface, hardware_interface::SystemInterface)
