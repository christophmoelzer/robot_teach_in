/*
#################################################################
author: Christoph Mölzer
email: christoph.moelzer@technikum-wien.at
github: 
#################################################################


This node communicates with the arduino.
It subcribes to the button states and publishes the displayed text and the LED states.
It also contains a state machine for the different motion settings

*/

#include <ros/ros.h>
#include <abb_libegm/egm_controller_interface.h>
#include <signal.h>
#include <stdio.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>


boost::asio::io_service io_service;
boost::thread_group thread_group;
abb::egm::EGMControllerInterface egm_interface(io_service, 6515);

volatile sig_atomic_t stop;

void inthand(int signum){
  stop = 1;
}

class TON{
  public:
  ros::Duration et;
  ros::Duration pt;
  ros::Time now;
  ros::Time start;
  bool in;
  bool q;
  bool aux;

  void init(){
    et = ros::Duration(0);
    pt = ros::Duration(0);
    now = ros::Time::now();
    start = ros::Time::now();
    in = false;
    q = false;
    aux = false;
  }

  void run(){
    now = ros::Time::now();
    if (in == true && aux == false){
      aux = true;
      start = ros::Time::now();

    }
    et = now-start;
    if (in == true && et >= pt){
      q = true;
    }
    if (in == false){
      aux = false;
      q = false;
    }
  }

  private:
};

class TeachTool
{
  public:

  void init(){
    step = 0;
    entry = true;
    timer.in = false;
    timer.pt = ros::Duration(1);
    timer.run();
  }

  int state_machine(){

    switch (step)
    {
    case 0:   // Init
      // Entry
      if (entry == true){
        entry = false;
        ROS_INFO_STREAM("entry " << step);
      }
      // Cyclic
      timer.in = true;
      timer.pt = ros::Duration(1);

      // Exit
      if (timer.q == true){
        timer.in = false;
        entry = true;
        step = 20;
      }
      break;
    
    case 20:  // IDLE
      // Entry
      if (entry == true){
        entry = false;
        ROS_INFO_STREAM("entry " << step);

      }
      // Cyclic
      timer.in = true;
      timer.pt = ros::Duration(1);

      // Exit
      if (timer.q == true){
        timer.in = false;
        entry = true;
        step = 0;
      }
      break;

    case 100:  // Translation
      // Entry
      if (entry == true){
        entry = false;
        ROS_INFO_STREAM("entry " << step);

      }
      // Cyclic
      timer.in = true;
      timer.pt = ros::Duration(1);

      // Exit
      if (timer.q == true){
        timer.in = false;
        entry = true;
        step = 0;
      }
      break;
    
    case 200:  // Rotation
      // Entry
      if (entry == true){
        entry = false;
        ROS_INFO_STREAM("entry " << step);

      }
      // Cyclic
      timer.in = true;
      timer.pt = ros::Duration(1);

      // Exit
      if (timer.q == true){
        timer.in = false;
        entry = true;
        step = 0;
      }
      break;
    
    case 300:  // Lead Through
      // Entry
      if (entry == true){
        entry = false;
        ROS_INFO_STREAM("entry " << step);

      }
      // Cyclic
      timer.in = true;
      timer.pt = ros::Duration(1);

      // Exit
      if (timer.q == true){
        timer.in = false;
        entry = true;
        step = 0;
      }
      break;
    
    case 400:  // 
      // Entry
      if (entry == true){
        entry = false;
        ROS_INFO_STREAM("entry " << step);

      }
      // Cyclic
      timer.in = true;
      timer.pt = ros::Duration(1);

      // Exit
      if (timer.q == true){
        timer.in = false;
        entry = true;
        step = 0;
      }
      break;
    
    

    default:
      break;
    }

    timer.run();

    return step;
  }


  private:
  int step;
  bool entry;
  TON timer;

};

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


class Display{
  public:
  ros::NodeHandle n;
  std::string str_line_1;
  std::string str_line_2;
  std::string str_button_1;
  std::string str_button_2;

  ros::Publisher pub_line_1;
  std_msgs::String line_1;

  Display(ros::NodeHandle& _n){
    n = _n;
    pub_line_1 = n.advertise<std_msgs::String>("/display_line_1", 1);
  }

  void publish(){
    line_1.data = "foo";
    pub_line_1.publish(line_1);
  }
  
};

class Buttons{
  public:
  ros::NodeHandle n;
  ros::Subscriber sub_buttons;
  
  Buttons(ros::NodeHandle& _n){
    n=_n;
  }

  void converter(){
    bool buttons[32] = {true,false,false,false,false, // group 1
                        false,false,false,false,false, // group 2
                        false,false,false,false,false, // group 3
                        false,false,false,false,false, // group 4
                        false,false,false,false,false,
                        false,false,false,false,false,
                        false, false};
    int i=0;
    uint32_t n=13;
    int a[32];
    uint32_t result=0;

    for (i=0; i<sizeof(buttons); i++){
        ROS_INFO_STREAM(buttons[31-i]);
        if (buttons[31-i] == true){
            result+=pow(2,i);
        }
    }

    ROS_INFO_STREAM("result = " << result);
    n=result;
    for(i=0; n>0; i++)    
    {    
        a[i]=n%2;    
        n= n/2;  
    }    
    ROS_INFO_STREAM("Binary of the given number= ");    
    for(i=i-1 ;i>=0 ;i--)    
    {    
        ROS_INFO_STREAM(a[i]);    
    }   
}

  void callback(const std_msgs::UInt32::ConstPtr& msg){
    ;
  }

  void sub(){
    sub_buttons = n.subscribe("/topic", 100, callback);
  }

};

void callback(const std_msgs::UInt32::ConstPtr& msg){
  ROS_INFO_STREAM("int: " << msg->data);
}

int main(int argc, char** argv)
{
  //----------------------------------------------------------
  // Preparations
  //----------------------------------------------------------
  // Initialize the node.
  ros::init(argc, argv, "pose_controller_node");
  ros::NodeHandle node_handle;
  ros::Subscriber sub = node_handle.subscribe("myInt", 1000, callback);

  EGM_Motion robot;
  signal(SIGINT, inthand);


  /*ros::Publisher states_pub = node_handle.advertise<geometry_msgs::Pose>("/pose_state", 1);
  ros::Publisher led_color_pub = node_handle.advertise<std_msgs::String>("/led_color", 1);
  ros::Publisher display_line_1_pub = node_handle.advertise<std_msgs::String>("/display_line_1", 1);
  ros::Publisher display_line_2_pub = node_handle.advertise<std_msgs::String>("/display_line_2", 1);
  ros::Publisher display_button_1_pub = node_handle.advertise<std_msgs::String>("/display_button_1", 1);
  ros::Publisher display_button_2_pub = node_handle.advertise<std_msgs::String>("/display_button_2", 1);
  */
  ros::Rate rate(250);
  //sensor_msgs::JointState msg;
  geometry_msgs::Pose msg;
  

  // Boost components for managing asynchronous UDP socket(s).
  TeachTool tool;
  Display disp(node_handle);
  tool.init();
  while (!stop){
    tool.state_machine();
    disp.publish();
    rate.sleep();
  }
  return 0;

  if(!egm_interface.isInitialized())
  {
    ROS_ERROR("EGM interface failed to initialize (e.g. due to port already bound)");
    return 0;
  }

  // Spin up a thread to run the io_service.
  thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));
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
    //states_pub.publish(msg);
    //ros::Duration(0.5).sleep();
    rate.sleep();  
  } 
  // Perform a clean shutdown.
  io_service.stop();
  thread_group.join_all();

  return 0;
}
