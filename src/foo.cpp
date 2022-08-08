#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>


volatile sig_atomic_t stop;

void inthand(int signum){
  stop = 1;
}

class Communication{
    public:
    ros::NodeHandle n;
    ros::Publisher pub_1, pub_2, pub_3, pub_4, pub_5, pub_6 ;
    ros::Subscriber sub_1, sub_2, sub_3, sub_4, sub_5, sub_6;
    std_msgs::String msg_1, msg_2, msg_3, msg_4, msg_5;
    std_msgs::Bool msg_6;
    std::vector<std::string> button_names_translation,  button_names_rotation;
    std::vector<bool> button_values;

    Communication(){
        ROS_INFO_STREAM("Init communication");
        pub_1 = n.advertise<std_msgs::String>("/text_line_1", 1);
        pub_2 = n.advertise<std_msgs::String>("/text_line_2", 1);
        pub_3 = n.advertise<std_msgs::String>("/text_button_left", 1);
        pub_4 = n.advertise<std_msgs::String>("/text_button_right", 1);
        pub_5 = n.advertise<std_msgs::String>("/text_button_center", 1);
        pub_6 = n.advertise<std_msgs::Bool>("/life_sign_to_mcu", 1);

        sub_1 = n.subscribe("/pushed", 100, &Communication::callback_1, this);
        sub_2 = n.subscribe("theta", 100, &Communication::callback_2, this);
        sub_3 = n.subscribe("speed", 100, &Communication::callback_3, this);
        sub_4 = n.subscribe("life_sign_from_mcu", 100, &Communication::callback_4, this);
    }

    void init_buttons(){

      // Translation
      button_names_translation.clear();
      // x-axis +
      button_names_translation.push_back("z-");
      button_names_translation.push_back("z+");
      button_names_translation.push_back("y+");
      button_names_translation.push_back("y-");
      button_names_translation.push_back("x-");

      // x-axis -
      button_names_translation.push_back("z-");
      button_names_translation.push_back("z+");
      button_names_translation.push_back("y-");
      button_names_translation.push_back("y+");
      button_names_translation.push_back("x+");

      // y-axis +
      button_names_translation.push_back("z-");
      button_names_translation.push_back("z+");
      button_names_translation.push_back("x-");
      button_names_translation.push_back("x+");
      button_names_translation.push_back("y-");

      // y-axis -
      button_names_translation.push_back("z-");
      button_names_translation.push_back("z+");
      button_names_translation.push_back("x+");
      button_names_translation.push_back("x-");
      button_names_translation.push_back("y+");

      // Rotation
      button_names_rotation.clear();
      // x-axis +
      button_names_rotation.push_back("ry+");
      button_names_rotation.push_back("ry-");
      button_names_rotation.push_back("rz+");
      button_names_rotation.push_back("rz-");
      button_names_rotation.push_back("n.d.");

      // x-axis -
      button_names_rotation.push_back("ry-");
      button_names_rotation.push_back("ry+");
      button_names_rotation.push_back("rz+");
      button_names_rotation.push_back("rz-");
      button_names_rotation.push_back("n.d.");

      // y-axis +
      button_names_rotation.push_back("rx-");
      button_names_rotation.push_back("rx+");
      button_names_rotation.push_back("rz+");
      button_names_rotation.push_back("rz-");
      button_names_rotation.push_back("n.d.");

      // y-axis -
      button_names_rotation.push_back("rx+");
      button_names_rotation.push_back("rx-");
      button_names_rotation.push_back("rz+");
      button_names_rotation.push_back("rz-");
      button_names_rotation.push_back("n.d.");



    }

    void callback_1(const std_msgs::UInt32& msg){
        //ROS_INFO_STREAM("callback_buttons" << msg.data);
        converter(msg.data);
    }

    void callback_2(const std_msgs::Float32& msg){
        ROS_INFO_STREAM("callback_theta");
    }

    void callback_3(const std_msgs::UInt16& msg){
        ROS_INFO_STREAM("callback_speed");
    }

    void callback_4(const std_msgs::Bool& msg){
        ROS_INFO_STREAM("callback_life_sign_from_mcu");
    }

    void converter(uint32_t number){
        int a[32];
        int i=0;
        
        ROS_INFO_STREAM("number = " << number);
        for(i=0; number>0; i++)    
        {   //ROS_INFO_STREAM(number%2);
            if (number%2) {
              button_values.push_back(true);
              //ROS_INFO_STREAM("true");
            }
            else{
              button_values.push_back(false);
              //ROS_INFO_STREAM("false");
            }
            a[i]=number%2;    
            number= number/2;  
        }    
        //ROS_INFO_STREAM("Binary of the given number = ");    
        for(i=i-1 ;i>=0 ;i--)    
        {    
            ;//ROS_INFO_STREAM(a[i]);    
        }   
        //ROS_INFO_STREAM(button_values);
    }


    void publish(){
        msg_1.data = "l1";
        msg_2.data = "l2";
        msg_3.data = "btn_l";
        msg_4.data = "btn_r";
        msg_5.data = "btn_c";
        msg_6.data = true;
        pub_1.publish(msg_1);
        pub_2.publish(msg_2);
        pub_3.publish(msg_3);
        pub_4.publish(msg_4);
        pub_5.publish(msg_5);
        pub_6.publish(msg_6);
    }
  
};


int main(int argc, char** argv)
{
    signal(SIGINT, inthand);
    ros::init(argc, argv, "pose_controller_node");
    Communication com;
    ros::Rate rate(10);
    ROS_INFO_STREAM("NODE STARTED");

    while(!stop){
        //com.publish();
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO_STREAM("EXIT");

  return 0;
}
