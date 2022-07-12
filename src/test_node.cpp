#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <cmath>
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */


// Button structure
// front, left, right, top, bottom

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

void callback_display(const std_msgs::String::ConstPtr& msg){
  ROS_INFO_STREAM("int: " << msg->data.c_str());
}

// 20 Buttons -> publish
// 4 RGB Leds -> subscribe
// 1 Display -> subscribe

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("myInt", 1000);
  ros::Subscriber sub = n.subscribe<std_msgs::String>("myDisplay",1000,callback_display);

  ros::Rate loop_rate(1);

    converter();

  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    
    std::vector<int32_t> vec1 = {1, 2, 3};
    std_msgs::Int32 msg;
    

    std::stringstream ss;
    ss << "hello world " << count;
    //msg.data = ss.str();


    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}