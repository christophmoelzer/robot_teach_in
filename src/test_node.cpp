#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include <cmath>
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */


void converter(){
    bool buttons[16] = {true,false,false,false,false,false,false,false,false,false,false,false,1,false,false,false,};
    int i=0;
    uint16_t n=13;
    int a[16];
    uint16_t result=0;

    for (i=0; i<sizeof(buttons); i++){
        ROS_INFO_STREAM(buttons[15-i]);
        if (buttons[15-i] == true){
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


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Int16>("myInt", 1000);

  ros::Rate loop_rate(1);

    converter();

  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    
    std::vector<int16_t> vec1 = {1, 2, 3};
    std_msgs::Int16 msg;
    

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