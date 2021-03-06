#ifndef ROS_TO_OSCC_H
#define ROS_TO_OSCC_H

#include <signal.h>

extern "C" {
#include <oscc.h>
}

#include <ros/ros.h>

#include <roscco/BrakeCommand.h>
#include <roscco/EnableDisable.h>
#include <roscco/SteeringCommand.h>
#include <roscco/ThrottleCommand.h>
#include <std_msgs/Bool.h>

// struct to enable detection of a rising or falling e_stop flag.
struct e_stop_status {
    bool previous_e_stop_state{false};
    bool e_stop{false};

    void setCurrent(bool message) {
        previous_e_stop_state = e_stop;
        e_stop = message;
    }

    bool isRising() {
        return (!previous_e_stop_state) && e_stop;
    }

    bool isFalling() {
        return previous_e_stop_state && (!e_stop);
    }

    void checkForEvent() {
        if (isRising()) {
            e_stop = true;
            ROS_DEBUG("OSCC_NODE: an emergency stop has been requested");
        }
        else if (isFalling()) {
            e_stop = false;
            ROS_DEBUG("OSCC_NODE: an emergency stop has been cancelled");
        }
    }
};

class RosToOscc
{
public:
  /**
   * @brief RosToOscc class initializer
   *
   * This function constructs ROS subscribers which can publish messages to OSCC API.
   *
   * @param public_nh  The public node handle to use for ROS subscribers.
   * @param private_nh The private node handle for ROS parameters.
   */
  RosToOscc(ros::NodeHandle* public_nh, ros::NodeHandle* private_nh);

  /**
   * @brief Callback function to publish ROS BrakeCommand messages to OSCC.
   *
   * This function is a callback that consume a ROS BrakeCommand message and publishes them to the OSCC API.
   *
   * @param msg ROS BrakeCommand message to be consumed.
   */
  void brakeCommandCallback(const roscco::BrakeCommand::ConstPtr& msg);

  /**
   * @brief Callback function to publish ROS SteeringCommand messages to OSCC.
   *
   * This function is a callback that consumes a ROS SteeringCommand message and publishes them to the OSCC API.
   *
   * @param msg ROS SteeringCommand message to be consumed.
   */
  void steeringCommandCallback(const roscco::SteeringCommand::ConstPtr& msg);

  /**
   * @brief Callback function to publish ROS ThrottleCommand messages to OSCC.
   *
   * This function is a callback that consumes a ROS ThrottleCommand message and publishes them to the OSCC API.
   *
   * @param msg ROS ThrottleCommand message to be consumed.
   */
  void throttleCommandCallback(const roscco::ThrottleCommand::ConstPtr& msg);

  /**
   * @brief Callback function to publish ROS EnableDisable messages to OSCC.
   *
   * This function is a callback that consumes a ROS EnableDisable message and publishes them to the OSCC API.
   *
   * @param msg ROS EnableDisable message to be consumed.
   */
  void enableDisableCallback(const roscco::EnableDisable::ConstPtr& msg);

  /**
   * @brief Callback function to listen to the e-stop request signal.
   *
   * This function is a callback that consumes a ROS boolean message and sets a boolean class variable.
   *
   * @param msg ROS boolean message to be consumed.
   */
  void eStopCallback(const std_msgs::Bool::ConstPtr& msg);

   /**
   * @brief checks the e-stop status and sends out e-stop messages on can if required.
   *
   *  Checks and sends the e-stop brake and accel commands on can if required.
   *  Returns true if e-stop is true, false if not.
   *
   * @param msg ROS boolean message to be consumed.
   */
  bool checkAndSendEStop();

private:
  ros::Subscriber topic_brake_command_;

  ros::Subscriber topic_steering_command_;

  ros::Subscriber topic_throttle_command_;

  ros::Subscriber topic_enable_disable_command_;

  ros::Subscriber e_stop_command_;

  e_stop_status e_stop_status_;
};

#endif  // ROS_TO_OSCC_H
