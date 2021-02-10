#include <roscco/ros_to_oscc.h>

const double E_STOP_BRAKE_COMMAND = 0.4;
const double E_STOP_ACCEL_COMMAND = 0;

RosToOscc::RosToOscc(ros::NodeHandle* public_nh, ros::NodeHandle* private_nh)
{
//   sigset_t mask;
//   sigset_t orig_mask;

//   sigemptyset(&mask);
//   sigemptyset(&orig_mask);
//   sigaddset(&mask, SIGIO);

//   // Temporary block of OSCC SIGIO while initializing ROS publication to prevent
//   // signal conflicts
//  if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0)
//  {
//    ROS_ERROR("Failed to block SIGIO");
//  }

  topic_brake_command_ =
      public_nh->subscribe<roscco::BrakeCommand>("brake_command", 10, &RosToOscc::brakeCommandCallback, this);

  topic_steering_command_ =
      public_nh->subscribe<roscco::SteeringCommand>("steering_command", 10, &RosToOscc::steeringCommandCallback, this);

  topic_throttle_command_ =
      public_nh->subscribe<roscco::ThrottleCommand>("throttle_command", 10, &RosToOscc::throttleCommandCallback, this);

  topic_enable_disable_command_ =
      public_nh->subscribe<roscco::EnableDisable>("enable_disable", 10, &RosToOscc::enableDisableCallback, this);

  e_stop_command_ =
      public_nh->subscribe<std_msgs::Bool>("e_stop_request", 10, &RosToOscc::eStopCallback, this);

//  if (sigprocmask(SIG_SETMASK, &orig_mask, NULL) < 0)
//  {
//    ROS_ERROR("Failed to unblock SIGIO");
//  }
};

void RosToOscc::brakeCommandCallback(const roscco::BrakeCommand::ConstPtr& msg)
{
  // if e_stop then bypass the teleop command and bring the car to a stop.
  if (checkAndSendEStop()) return;

  oscc_result_t ret = OSCC_ERROR;

  ret = oscc_publish_brake_position(msg->brake_position);

  if (ret == OSCC_ERROR)
  {
    ROS_ERROR("OSCC_ERROR occured while trying send the brake position.");
  }
  else if (ret == OSCC_WARNING)
  {
    ROS_WARN("OSCC_WARNING occured while trying send the brake position.");
  }
};

void RosToOscc::steeringCommandCallback(const roscco::SteeringCommand::ConstPtr& msg)
{
  oscc_result_t ret = OSCC_ERROR;
  double inverted_steering = msg->steering_torque;
  inverted_steering *= -1;
  ret = oscc_publish_steering_torque(inverted_steering);

  if (ret == OSCC_ERROR)
  {
    ROS_ERROR("OSCC_ERROR occured while trying send the steering torque.");
  }
  else if (ret == OSCC_WARNING)
  {
    ROS_WARN("OSCC_WARNING occured while trying send the steering torque.");
  }

  // if e-stop, then bring the car to a stop.
  checkAndSendEStop();
};

void RosToOscc::throttleCommandCallback(const roscco::ThrottleCommand::ConstPtr& msg)
{
  // if e_stop is active, bypass the teleop command and bring the car to a stop.
  if (checkAndSendEStop()) return;

  oscc_result_t ret = OSCC_ERROR;

  ret = oscc_publish_throttle_position(msg->throttle_position);

  if (ret == OSCC_ERROR)
  {
    ROS_ERROR("OSCC_ERROR occured while trying send the throttle position.");
  }
  else if (ret == OSCC_WARNING)
  {
    ROS_WARN("OSCC_WARNING occured while trying send the throttle position.");
  }
};

void RosToOscc::enableDisableCallback(const roscco::EnableDisable::ConstPtr& msg)
{
  oscc_result_t ret = OSCC_ERROR;

  ret = msg->enable_control ? oscc_enable() : oscc_disable();
  ROS_ERROR("Got enable disable");
		 
  if (ret == OSCC_ERROR)
  {
    ROS_ERROR("OSCC_ERROR occured while trying to enable or disable control.");
  }
  else if (ret == OSCC_WARNING)
  {
    ROS_WARN("OSCC_WARNING occured while trying to enable or disable control.");
  }
}

void RosToOscc::eStopCallback(const std_msgs::Bool::ConstPtr& msg)
{
    // set the status with the latest message data.
    e_stop_status_.setCurrent(msg->data);

    // process current and previous data to check for a rising or falling signal.
    e_stop_status_.checkForEvent();

    // if e-stop is requested, then bring the car to a stop.
    checkAndSendEStop();
}

bool RosToOscc::checkAndSendEStop()
{
    // if e_stop then bring the car to a stop.
    if (e_stop_status_.e_stop) {
        oscc_result_t brake_ret = OSCC_ERROR;
        oscc_result_t accel_ret = OSCC_ERROR;

        brake_ret = oscc_publish_brake_position(E_STOP_BRAKE_COMMAND);
        accel_ret = oscc_publish_throttle_position(E_STOP_ACCEL_COMMAND);

        if (brake_ret == OSCC_ERROR)
        {
            ROS_ERROR("OSCC_ERROR: occured while trying send the brake position. DURING E_STOP!!");
        }
        if (accel_ret == OSCC_ERROR)
        {
            ROS_ERROR("OSCC_ERROR: occured while trying send the accel position. DURING E_STOP!!");
        }
        return true;
    }
    return false;
}
