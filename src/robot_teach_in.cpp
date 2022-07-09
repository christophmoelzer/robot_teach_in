#include <ros/ros.h>
#include <abb_libegm/egm_controller_interface.h>
#include <signal.h>
#include <stdio.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>


boost::asio::io_service io_service;
boost::thread_group thread_group;
abb::egm::EGMControllerInterface egm_interface(io_service, 6515);

volatile sig_atomic_t stop;

void inthand(int signum){
  stop = 1;
}
class Debug_Info
{
  public:
  ros::Duration t_et;
  ros::Duration t_pt;
  ros::Time t_now;
  ros::Time t_start;
  bool in;
  bool q;
  bool aux;

  void init(){
    t_et = ros::Duration(0);
    t_pt = ros::Duration(0);
    t_now = ros::Time::now();
    t_start = ros::Time::now();
    in = false;
    q = false;
    aux = false;
  }

  void run(){
    if (q == true && false){
      std::cout << "in: " << in << std::endl;
      std::cout << "aux: " << aux << std::endl;
      std::cout << "q: " << q << std::endl;
      std::cout << "pt: " << t_pt << std::endl;
      std::cout << "start: " << t_start << std::endl;
      std::cout << "now: " << t_now << std::endl;
      std::cout << "et: " << t_et << std::endl;
      std::cout << std::endl;
      //std::cout << "" <<  << std::endl;
    }
    t_now = ros::Time::now();
    if (in == true && aux == false){
      aux = true;
      t_start = ros::Time::now();
    }
    t_et = t_now-t_start;
    if (in == true && t_et >= t_pt){
      q = true;
    }
    if (in == false){
      aux = false;
      q = false;
    }
  }


  
};

class EGM_Motion
{

    public:
    Debug_Info debug_info;
    


    void init(){
      t_prev = ros::Time::now();
        output.Clear();
        egm_interface.read(&input);
        current_pose.CopyFrom(input.feedback().robot().cartesian().pose());
        output.mutable_robot()->mutable_cartesian()->mutable_pose()->CopyFrom(current_pose);
        debug_info.init();
    }

    void get_current_pose(){
        egm_interface.read(&input);
        current_pose.CopyFrom(input.feedback().robot().cartesian().pose());
        output.mutable_robot()->mutable_cartesian()->mutable_pose()->CopyFrom(current_pose);
        
        std::system("clear");
        ROS_INFO_STREAM("x: " << current_pose.position().x());
        ROS_INFO_STREAM("y: " << current_pose.position().y());
        ROS_INFO_STREAM("z: " << current_pose.position().z());
        ROS_INFO_STREAM("rx: " << current_pose.euler().x());
        ROS_INFO_STREAM("ry: " << current_pose.euler().y());
        ROS_INFO_STREAM("rz: " << current_pose.euler().z());
        std::cout << std::endl;
    }

    void get_destination_pose(){
      destination_pose.CopyFrom(output.robot().cartesian().pose());
    }

    void step(double x=0, double y=0, double z=0, double rx=0, double ry=0, double rz=0){
      get_current_pose();
      double x_new, y_new, z_new, rx_new, ry_new, rz_new;

      x_new = current_pose.position().x() + x;
      y_new = current_pose.position().y() + y;
      z_new = current_pose.position().z() + z;
      rx_new = current_pose.euler().x() + rx;
      ry_new = current_pose.euler().y() + ry;
      rz_new = current_pose.euler().z() + rz;

      output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_x(x_new);
      output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_y(y_new);
      output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_z(z_new);
      output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_euler()->set_x(rx_new);
      output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_euler()->set_y(ry_new);
      output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_euler()->set_z(rz_new);
      egm_interface.write(output);
      bool wait = true;
        while (wait && !stop){
          if (check_position_reached() or true) {
            wait = false;
          }
        }
    }

    void step_x(double value){
      ROS_INFO_STREAM("move x: " << value << " mm");
      step(value, 0, 0, 0, 0, 0);
    }

    void step_y(double value){
      ROS_INFO_STREAM("move y: " << value << " mm");
      step(0, value, 0, 0, 0, 0);
    }

    void step_z(double value){
      ROS_INFO_STREAM("move z: " << value << " mm");
      step(0, 0, value, 0, 0, 0);
    }

    void step_rx(double value){
      ROS_INFO_STREAM("move rx: " << value << " °");
      step(0, 0, 0, value, 0, 0);
    }

    void step_ry(double value){
      ROS_INFO_STREAM("move ry: " << value << " °");
      step(0, 0, 0, 0, value, 0);
    }

    void step_rz(double value){
      ROS_INFO_STREAM("move rz: " << value << " °");
      step(0, 0, 0, 0, 0, value);
    }

    void step_up(double value){
        step(value);
    }

    void step_down(double value){
      step(-value);
    }

    bool check_position_reached(){
      get_current_pose();
      get_destination_pose();
      diff = destination_pose.position().z() - current_pose.position().z();
      //ROS_INFO_STREAM("dest_z:" << destination_pose.position().z() << " - curr_z:" << current_pose.position().z() << " = " << diff);
      debug_info.run();
      debug_info.t_pt = ros::Duration(0.2);
      debug_info.in = true;
      //if (abs(diff_old-diff)>0.1){
        if (debug_info.q == true){
          //ROS_INFO_STREAM("diff = " << abs(diff));
          //ROS_INFO_STREAM("J2: " << j2);
          //ROS_INFO_STREAM("z: " << z);
          std::cout << std::endl;
          debug_info.in = false;
        }
        diff_old = diff;
      //}
      
      if ((abs(diff)) < 0.01){
        t_now = ros::Time::now();
        t_elapsed = t_now -t_prev;
        t_prev = ros::Time::now();
        ROS_INFO_STREAM("Position reached " << t_elapsed);
        return true;
      }
      else{
        return false;
      }
    }

    


    private:
    double position_reference;
    double new_x, new_y;
    double diff_old, diff;
    abb::egm::wrapper::Input input;
    abb::egm::wrapper::CartesianPose current_pose;
    abb::egm::wrapper::CartesianPose destination_pose;
    abb::egm::wrapper::Output output;
    ros::Duration t_elapsed;
    ros::Time t_now;
    ros::Time t_prev;

  

};


int main(int argc, char** argv)
{
  //----------------------------------------------------------
  // Preparations
  //----------------------------------------------------------
  // Initialize the node.
  ros::init(argc, argv, "pose_controller_node");
  ros::NodeHandle node_handle;

  EGM_Motion robot;
  signal(SIGINT, inthand);


  ros::Publisher states_pub = node_handle.advertise<geometry_msgs::Pose>("/pose_state", 1);
  ros::Rate rate(250);
  //sensor_msgs::JointState msg;
  geometry_msgs::Pose msg;
  

  // Boost components for managing asynchronous UDP socket(s).
  
  

ROS_INFO_STREAM(egm_interface.getStatus().rapid_execution_state());
  if(!egm_interface.isInitialized())
  {
    ROS_ERROR("EGM interface failed to initialize (e.g. due to port already bound)");
    return 0;
  }

  // Spin up a thread to run the io_service.
  thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));

  

  //----------------------------------------------------------
  // Execute a pose controller loop.
  //
  // Note: The EGM communication session is started by the
  //       EGMRunPose RAPID instruction.
  //----------------------------------------------------------
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
  ROS_INFO_STREAM(egm_interface.getStatus().rapid_execution_state());

  robot.init();
  wait = true;
  while(ros::ok()  && !stop)
  {
    // Wait for a new EGM message from the EGM client (with a timeout of 500 ms).
    if(egm_interface.waitForMessage(10))
    {
      //robot.step_x(50.0);
      //ros::Duration(5).sleep();
      //robot.step_y(10.0);
      //ros::Duration(5).sleep();
      //robot.step_z(10.0);
      //ros::Duration(5).sleep();
      robot.step_ry(45.0);
      ros::Duration(15).sleep();
      robot.step_rz(90.0);
      ros::Duration(15).sleep();
      robot.step_ry(-45.0);
      ros::Duration(15).sleep();
    }

    
    states_pub.publish(msg);
    //ros::Duration(0.5).sleep();
    rate.sleep();
    
  } 
  ROS_INFO_STREAM(egm_interface.getStatus().rapid_execution_state());
  // Perform a clean shutdown.
  io_service.stop();
  thread_group.join_all();

  return 0;
}
