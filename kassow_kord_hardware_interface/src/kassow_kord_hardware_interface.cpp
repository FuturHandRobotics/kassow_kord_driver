// Copyright (c) 2025 b»robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential
//
// Adapted for <Insert_Company_Name> that received unlimited, worldwide
// use and change right, except distributing this library separately
// of their product.

#include "kassow_kord_hardware_interface/kassow_kord_hardware_interface.hpp"

#include <algorithm>
#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * \file kassow_kord_hardware_interface.cpp
 * \brief Hardware interface for Kassow Kord robots using the kord-api.
 * params:
 *   - ip_address (string, required): IP address of the robot controller.
 *   - port (int, required): Port number for Kord connection.
 *   - session_id (int, required): Kord session ID.
 *   - waitSync_timeout_ms (int, required): Kord session ID.
 */
namespace kassow_kord_hardware_interface
{
namespace
{
const char * source_of_control_name(kr2::kord::protocol::ESourceOfControl v)
{
  switch (v)
  {
    case kr2::kord::protocol::ESourceOfControl::eUnknown:
      return "Unknown";
    case kr2::kord::protocol::ESourceOfControl::eDirect:
      return "Direct (local/pendant)";
    case kr2::kord::protocol::ESourceOfControl::eExternal:
      return "External (KORD)";
  }
  return "?";
}

const char * operation_mode_name(kr2::kord::protocol::EOperationMode v)
{
  switch (v)
  {
    case kr2::kord::protocol::EOperationMode::eUnknown:
      return "Unknown";
    case kr2::kord::protocol::EOperationMode::eManualReduced:
      return "ManualReduced";
    case kr2::kord::protocol::EOperationMode::eManualHigh:
      return "ManualHigh";
    case kr2::kord::protocol::EOperationMode::eAutomatic:
      return "Automatic";
    case kr2::kord::protocol::EOperationMode::eRecovery:
      return "Recovery";
    case kr2::kord::protocol::EOperationMode::eOffline:
      return "Offline";
    case kr2::kord::protocol::EOperationMode::eMaintenance:
      return "Maintenance";
  }
  return "?";
}

const char * motion_state_name(kr2::kord::protocol::EMotionState v)
{
  switch (v)
  {
    case kr2::kord::protocol::EMotionState::None:
      return "None";
    case kr2::kord::protocol::EMotionState::Standstill:
      return "Standstill";
    case kr2::kord::protocol::EMotionState::Tracking:
      return "Tracking";
    case kr2::kord::protocol::EMotionState::Stopping:
      return "Stopping";
    case kr2::kord::protocol::EMotionState::Resuming:
      return "Resuming";
    case kr2::kord::protocol::EMotionState::Jogging:
      return "Jogging";
    case kr2::kord::protocol::EMotionState::BackDrive:
      return "BackDrive";
    case kr2::kord::protocol::EMotionState::DirectJointControl:
      return "DirectJointControl";
    case kr2::kord::protocol::EMotionState::VelocityControl:
      return "VelocityControl";
    case kr2::kord::protocol::EMotionState::Halting:
      return "Halting";
    case kr2::kord::protocol::EMotionState::Halted:
      return "Halted";
    case kr2::kord::protocol::EMotionState::Suspended:
      return "Suspended";
    case kr2::kord::protocol::EMotionState::Paused:
      return "Paused";
  }
  return "?";
}
}  // namespace

hardware_interface::CallbackReturn KassowKordHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto & hw_params = info_.hardware_parameters;

  // Get required parameters
  if (hw_params.find("ip_address") != hw_params.end())
  {
    ip_address = hw_params.at("ip_address");
  }
  else
  {
    RCLCPP_FATAL(get_logger(), "Parameter 'ip_address' is required but not provided");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (hw_params.find("port") != hw_params.end())
  {
    port = std::stoi(hw_params.at("port"));
  }
  else
  {
    RCLCPP_FATAL(get_logger(), "Parameter 'port' is required but not provided");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (hw_params.find("session_id") != hw_params.end())
  {
    session_id = std::stoi(hw_params.at("session_id"));
  }
  else
  {
    RCLCPP_FATAL(get_logger(), "Parameter 'session_id' is required but not provided");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (hw_params.find("waitSync_timeout_ms") != hw_params.end())
  {
    waitSync_timeout_ms = std::stoi(hw_params.at("waitSync_timeout_ms"));
  }
  else
  {
    RCLCPP_FATAL(get_logger(), "Parameter 'waitSync_timeout_ms' is required but not provided");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != KORD_JOINT_COUNT)
  {
    RCLCPP_FATAL(
      get_logger(),
      "KassowKordHardwareInterface requires exactly 7 joints defined in the URDF/hardware config.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  size_t joint_index = 0;
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Validate we have one position command interface
    if (joint.command_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' command interface invalid. Expected exactly three command interfaces.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate state interfaces
    int required_interfaces = 0;

    for (const auto & command_interface : joint.command_interfaces)
    {
      if (
        command_interface.name == hardware_interface::HW_IF_POSITION ||
        command_interface.name == hardware_interface::HW_IF_VELOCITY ||
        command_interface.name == hardware_interface::HW_IF_ACCELERATION)
      {
        if (++required_interfaces == 3)
        {
          break;
        }
      }
    }

    if (required_interfaces != 3)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' missing required state interfaces. Expected position, velocity, and "
        "acceleration.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate we have four state interface
    if (joint.state_interfaces.size() != 4)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' state interface invalid. Expected exactly four state interfaces.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate state interfaces
    required_interfaces = 0;

    for (const auto & state_interface : joint.state_interfaces)
    {
      if (
        state_interface.name == hardware_interface::HW_IF_POSITION ||
        state_interface.name == hardware_interface::HW_IF_VELOCITY ||
        state_interface.name == hardware_interface::HW_IF_ACCELERATION ||
        state_interface.name == hardware_interface::HW_IF_EFFORT)
      {
        if (++required_interfaces == 4)
        {
          break;
        }
      }
    }

    if (required_interfaces != 4)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' missing required state interfaces. Expected position, velocity, acceleration, "
        "and effort.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    joint_position_itfs_[joint_index] = joint.name + "/" + hardware_interface::HW_IF_POSITION;
    joint_velocity_itfs_[joint_index] = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
    joint_acceleration_itfs_[joint_index] =
      joint.name + "/" + hardware_interface::HW_IF_ACCELERATION;
    joint_effort_itfs_[joint_index] = joint.name + "/" + hardware_interface::HW_IF_EFFORT;
    joint_index++;
  }

  kord_ = std::shared_ptr<kr2::kord::KordCore>(
    new kr2::kord::KordCore(ip_address, port, session_id, kr2::kord::UDP_CLIENT));

  // Initialize Control and Receiver Interfaces.
  ctl_iface_ = std::make_unique<kr2::kord::ControlInterface>(kord_);
  rcv_iface_ = std::make_unique<kr2::kord::ReceiverInterface>(kord_);

  RCLCPP_INFO(
    get_logger(), "KassowKordHardwareInterface on_init completed for %zu joints", KORD_JOINT_COUNT);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KassowKordHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "cleanup KassowKordHardwareInterface...");

  if (!clean_alarms())
  {
    RCLCPP_DEBUG(
      get_logger(), "clean_alarms() returned false during deactivate (continuing cleanup)");
  }

  kord_->disconnect();

  RCLCPP_INFO(get_logger(), "Successfully cleaned up");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KassowKordHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    get_logger(), "Connecting to Kassow Kord robot at ip %s | port %d | session id %d",
    ip_address.c_str(), port, session_id);
  if (!kord_->connect())
  {
    RCLCPP_FATAL(get_logger(), "Failed to connect to Kassow Kord robot.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "KassowKordHardwareInterface configured and connected");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KassowKordHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset robot errors and alarms as needed
  try
  {
    if (!clean_alarms())
    {
      RCLCPP_FATAL(get_logger(), "Failed to reset alarms to Kassow Kord robot.");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(), "Exception while resetting alarms: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read initial joint positions and set them as the initial command values.
  // T_REFERENCE_* is v4's name for what v3 called S_ACTUAL_* -- both resolve to
  // the same underlying RobotStatus members (positions_/speed_/accelerations_,
  // from the eJConfigurationArm wire field). S_SENSED_* is a different field
  // (eJSensedPosition) that existed under that same name in v3 too; it is not
  // the v4 replacement for S_ACTUAL_* and is not populated on every controller.
  rcv_iface_->fetchData();
  position_states = rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::T_REFERENCE_Q);
  velocity_states = rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::T_REFERENCE_QD);
  acceleration_states =
    rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::S_SENSED_ACCELERATIONS);
  torque_states = rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::S_SENSED_TRQ);

  for (size_t i = 0; i < KORD_JOINT_COUNT; ++i)
  {
    set_state(joint_position_itfs_[i], position_states[i]);
    set_state(joint_velocity_itfs_[i], velocity_states[i]);
    set_state(joint_acceleration_itfs_[i], acceleration_states[i]);
    set_state(joint_effort_itfs_[i], torque_states[i]);

    set_command(joint_position_itfs_[i], position_states[i]);
    set_command(joint_velocity_itfs_[i], velocity_states[i]);
    set_command(joint_acceleration_itfs_[i], acceleration_states[i]);
  }

  log_robot_state("on_activate");

  RCLCPP_INFO(
    get_logger(), "Initial joint positions [rad]: %.4f %.4f %.4f %.4f %.4f %.4f %.4f",
    position_states[0], position_states[1], position_states[2], position_states[3],
    position_states[4], position_states[5], position_states[6]);

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KassowKordHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return CallbackReturn::SUCCESS;
}

// read: fetch current joint states from the robot and populate ROS2 control state buffers.
hardware_interface::return_type KassowKordHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!kord_->waitSync(std::chrono::milliseconds(waitSync_timeout_ms)))
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Kord waitSync timeout");
    return hardware_interface::return_type::ERROR;
  }

  rcv_iface_->fetchData();

  if (rcv_iface_->systemAlarmState())
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Alarm detected, deactivating...");
    return hardware_interface::return_type::ERROR;
  }

  // See the note in on_activate(): T_REFERENCE_* is v4's name for v3's
  // S_ACTUAL_*, not S_SENSED_*.
  position_states = rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::T_REFERENCE_Q);
  velocity_states = rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::T_REFERENCE_QD);
  acceleration_states =
    rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::T_REFERENCE_QDD);
  torque_states = rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::S_SENSED_TRQ);

  for (size_t i = 0; i < KORD_JOINT_COUNT; ++i)
  {
    set_state(joint_position_itfs_[i], position_states[i]);
    set_state(joint_velocity_itfs_[i], velocity_states[i]);
    set_state(joint_acceleration_itfs_[i], acceleration_states[i]);
    set_state(joint_effort_itfs_[i], torque_states[i]);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KassowKordHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < KORD_JOINT_COUNT; ++i)
  {
    position_cmds[i] = get_command(joint_position_itfs_[i]);
    velocity_cmds[i] = get_command(joint_velocity_itfs_[i]);
    acceleration_cmds[i] = get_command(joint_acceleration_itfs_[i]);
  }

  if (!ctl_iface_->directJControl(position_cmds, velocity_cmds, acceleration_cmds))
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Kord failed to write joint positions");
    return hardware_interface::return_type::ERROR;
  }

  // directJControl() only reports that the frame was handed to the socket, not
  // that the controller acted on it. Periodically report what the arm is
  // actually doing, and how far the commands are from the measured state, so a
  // silently ignored command stream is visible here.
  {
    double max_err = 0.0;
    for (size_t i = 0; i < KORD_JOINT_COUNT; ++i)
    {
      max_err = std::max(max_err, std::abs(position_cmds[i] - position_states[i]));
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000, "cmd vs state: max |cmd-state| = %.5f rad | motion: %s",
      max_err, motion_state_name(rcv_iface_->getMotionState()));
  }

  return hardware_interface::return_type::OK;
}


void KassowKordHardwareInterface::log_robot_state(const char * context)
{
  const auto soc = rcv_iface_->getRobotSourceOfControl();
  const auto mode = rcv_iface_->getRobotOperationMode();
  const auto motion = rcv_iface_->getMotionState();

  RCLCPP_INFO(
    get_logger(), "[%s] source of control: %s | operation mode: %s | motion state: %s (%d)", context,
    soc.has_value() ? source_of_control_name(*soc) : "n/a",
    mode.has_value() ? operation_mode_name(*mode) : "n/a", motion_state_name(motion),
    static_cast<int>(motion));

  // Direct joint control frames only actually drive the arm once the
  // controller is in DirectJointControl; anything else means the commands are
  // being received and ignored.
  if (motion != kr2::kord::protocol::EMotionState::DirectJointControl)
  {
    RCLCPP_WARN(
      get_logger(),
      "[%s] controller is not in DirectJointControl -- joint commands will not move the arm.",
      context);
  }

  if (soc.has_value() && *soc != kr2::kord::protocol::ESourceOfControl::eExternal)
  {
    RCLCPP_WARN(
      get_logger(), "[%s] robot is not under External (KORD) control -- commands will be ignored.",
      context);
  }
}

// clean all alarms
bool KassowKordHardwareInterface::clean_alarms()
{
  // fetchStatus() waits for a fresh status update from the controller and then
  // copies it into the state structures; fetchData() alone only copies whatever
  // was last captured. Right after connect() that can still be empty, which
  // would make a robot with a live alarm look clean (or vice versa).
  if (!rcv_iface_->fetchStatus())
  {
    RCLCPP_WARN(get_logger(), "Could not fetch a fresh status; using last known state.");
    rcv_iface_->fetchData();
  }

  const auto alarm_state = rcv_iface_->systemAlarmState();

  // Nothing to clear. Sending clear requests anyway means blocking on an
  // acknowledgement the controller has no reason to send.
  if (alarm_state == 0)
  {
    RCLCPP_INFO(get_logger(), "No alarms to clear.");
    return true;
  }

  unsigned int motion_flags = rcv_iface_->getMotionFlags();
  unsigned int safety_flags = rcv_iface_->getRobotSafetyFlags();

  RCLCPP_INFO(get_logger(), "Motion flags: %d", motion_flags);
  RCLCPP_INFO(get_logger(), "Robot safety flags: %d", safety_flags);

  if (safety_flags & SafetyFlags::SAFETY_FLAG_USER_CONF_REQ)
  {
    RCLCPP_ERROR(get_logger(), "Errors cannot be cleared. User confirmation required.");
  }

  const bool is_halt = motion_flags & MotionFlags::MOTION_FLAG_HALT;
  const bool is_pstop = safety_flags & SafetyFlags::SAFETY_FLAG_PSTOP;

  if (is_halt && is_pstop)
  {
    RCLCPP_ERROR(get_logger(), "Halt Error.");
  }

  if (motion_flags & MotionFlags::MOTION_FLAG_SUSPENDED)
  {
    RCLCPP_ERROR(get_logger(), "Suspend Error.");
  }

  bool is_cbun = alarm_state & kr2::kord::protocol::CAT_CBUN_EVENT;

  // Check for KORD event
  const auto system_events = rcv_iface_->getSystemEvents();
  for (auto & event : system_events)
  {
    if (event.event_group_ == kr2::kord::protocol::eKordEvent)
    {
      is_cbun = true;
      break;
    }
  }

  if (is_cbun)
  {
    RCLCPP_ERROR(get_logger(), "CBun Error.");
  }

  // clear errors
  std::map<kr2::kord::ControlInterface::EClearRequest, std::string> commands_mapped = {
    {kr2::kord::ControlInterface::EClearRequest::CLEAR_HALT, "CLEAR_HALT"},
    {kr2::kord::ControlInterface::EClearRequest::CBUN_EVENT, "CBUN_EVENT"},
    {kr2::kord::ControlInterface::EClearRequest::CONTINUE_INIT, "CONTINUE_INIT"},
    {kr2::kord::ControlInterface::EClearRequest::UNSUSPEND, "UNSUSPEND"}};

  // Bound how long we wait for each acknowledgement. kord-api's own request
  // flows (KordCore::waitForResponse) poll with waitSync(1s) and no
  // F_SYNC_FULL_ROTATION; requiring a complete frame-id rotation inside 10 ms
  // and aborting on the first miss -- as this used to -- turns an unacked
  // command into a failed activation, which takes down ros2_control_node.
  constexpr auto ack_timeout = std::chrono::seconds(5);

  for (const auto & kv : commands_mapped)
  {
    const auto command = kv.first;
    const auto & name = kv.second;
    int64_t token = ctl_iface_->clearAlarmRequest(command);

    RCLCPP_INFO(
      get_logger(), "%s command sent with token: %ld", name.c_str(), static_cast<int64_t>(token));

    const auto deadline = std::chrono::steady_clock::now() + ack_timeout;
    int8_t status = -1;

    while (std::chrono::steady_clock::now() < deadline)
    {
      if (!kord_->waitSync(std::chrono::seconds(1)))
      {
        RCLCPP_WARN(get_logger(), "%s: sync wait failed while awaiting ack.", name.c_str());
        continue;
      }

      rcv_iface_->fetchData();

      status = rcv_iface_->getCommandStatus(token);
      if (status != -1)
      {
        break;
      }
    }

    if (status != -1)
    {
      RCLCPP_INFO(get_logger(), "%s command status: %d", name.c_str(), static_cast<int>(status));
    }
    else
    {
      RCLCPP_WARN(
        get_logger(), "%s: no command status within %lds; continuing.", name.c_str(),
        static_cast<long>(ack_timeout.count()));
    }
  }

  // The acknowledgements above are advisory -- what actually matters is whether
  // the alarm state went away. Report that explicitly rather than assuming the
  // clears took effect because the commands were sent.
  if (rcv_iface_->fetchStatus())
  {
    const auto remaining = rcv_iface_->systemAlarmState();
    if (remaining != 0)
    {
      RCLCPP_WARN(
        get_logger(), "Alarm state still set after clear attempts: 0x%x",
        static_cast<unsigned int>(remaining));
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Alarm state cleared.");
    }
  }

  return true;
}

}  // namespace kassow_kord_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kassow_kord_hardware_interface::KassowKordHardwareInterface, hardware_interface::SystemInterface)
