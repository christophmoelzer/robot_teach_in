#include <ros/ros.h>
#include <abb_libegm/egm_controller_interface.h>
#include <stdio.h>


int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "pose_controller_node");
  ros::NodeHandle node_handle;
  boost::asio::io_service io_service;
  boost::thread_group thread_group;
  abb::egm::EGMControllerInterface egm_interface(io_service, 6515);
  abb::egm::wrapper::Input input;
  abb::egm::wrapper::Output output;
  double current_z = 0;
  double new_z = 0;
  double delta_z = 0;
  double sign=1;

  if(!egm_interface.isInitialized())
  {
    ROS_ERROR("EGM interface failed to initialize (e.g. due to port already bound)");
    return 0;
  }

  thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));

  ROS_INFO("========== Pose controller loop (open-loop) sample ==========");
  bool wait = true;
  ROS_INFO("1: Wait for an EGM communication session to start...");
  while(ros::ok() && wait)
  {
    if(egm_interface.isConnected())
    {
      if(egm_interface.getStatus().rapid_execution_state() == abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED)
      {
        ROS_WARN("RAPID execution state is UNDEFINED (might happen first time after controller start/restart). Try to restart the RAPID program.");
      }
      else
      {
        wait = egm_interface.getStatus().rapid_execution_state() != abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
      }
    }
    ros::Duration(0.5).sleep();
  }
  
  wait = true;
  while(ros::ok())
  {
    // Wait for a new EGM message from the EGM client (with a timeout of 500 ms).
    if(egm_interface.waitForMessage(500))
    {
      egm_interface.read(&input);

      current_z = input.feedback().robot().cartesian().pose().position().z();
      new_z = current_z + 100 * sign;

      output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_z(new_z);

      egm_interface.write(output);

      bool wait = true;
      while (wait){
        egm_interface.read(&input);
        current_z = input.feedback().robot().cartesian().pose().position().z();
        delta_z = abs(current_z-new_z);
        //ROS_INFO_STREAM("current z: " << current_z);
        //ROS_INFO_STREAM("new z: " << new_z);
        ROS_INFO_STREAM("delta z: " << delta_z);
        //std::cout << std::endl;
        if (delta_z < 0.1){
          wait = false;
          sign *= -1;
        }

      }
    }
    else{
      std::cout << "timeout" << std::endl;
    }

    
  } 
  // Perform a clean shutdown.
  io_service.stop();
  thread_group.join_all();

  return 0;
}
