#include <string>

extern "C" {
#include <oscc.h>
}

#include <ros/ros.h>

#include <roscco/oscc_to_ros.h>
#include <roscco/ros_to_oscc.h>
#include <thread>
#include <chrono>
bool is_running = true;

void pubsub_thread() {
      std::thread::id this_id = std::this_thread::get_id(); 
    while (is_running) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));    
    }
}

int main(int argc, char* argv[])
{
  std::thread rosthread(pubsub_thread);

    sigset_t mask;
  sigset_t orig_mask;

  sigemptyset(&mask);
  sigemptyset(&orig_mask);
  sigaddset(&mask, SIGIO);

  // Temporary block of OSCC SIGIO while initializing ROS publication to prevent
  // signal conflicts
 if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0)
 {
   ROS_ERROR("Failed to block SIGIO");
 }

  ros::init(argc, argv, "roscco_node");

  ros::NodeHandle public_nh;
  ros::NodeHandle private_nh("~");

  int can_channel;
  private_nh.param<int>("can_channel", can_channel, 0);

  oscc_result_t ret = OSCC_ERROR;

  ret = oscc_init();

  if (ret != OSCC_OK)
  {
    ROS_ERROR("Could not initialize OSCC");
  }

  

  RosToOscc subcriber(&public_nh, &private_nh);
  OsccToRos publisher(&public_nh, &private_nh);
  ros::spin();

  is_running = false;
  rosthread.join();

  ret = oscc_disable();

  if (ret != OSCC_OK)
  {
    ROS_ERROR("Could not disable OSCC");
  }

  ret = oscc_close(can_channel);

  if (ret != OSCC_OK)
  {
    ROS_ERROR("Could not close OSCC connection");
  }

  ros::waitForShutdown();
}
