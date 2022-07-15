#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>


volatile sig_atomic_t stop;

void inthand(int signum){
  stop = 1;
}

class myPublisher{
    public:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
    std_msgs::String msg;

    myPublisher(){
        ROS_INFO_STREAM("myPublisher");
        pub = n.advertise<std_msgs::String>("/display_line_1", 1);
        sub = n.subscribe("abc", 100, &myPublisher::callback, this);
    }

    void callback(const std_msgs::String& msg){
        ROS_INFO_STREAM("callback");
    }

    void publish(){
        msg.data = "foo";
        pub.publish(msg);
        ROS_INFO_STREAM("publish");
    }
  
};

/*
class mySubscriber{
  public:
  ros::NodeHandle n;
  ros::Subscriber sub_buttons;
  
  mySubscriber(ros::NodeHandle& _n){
    n = _n;
    ;sub_buttons = n.subscribe("/topic", 100, callback);
  }

  
  void callback(const std_msgs::UInt32::ConstPtr& msg){
    ;
  }

};
*/



int main(int argc, char** argv)
{
    signal(SIGINT, inthand);
    ros::init(argc, argv, "pose_controller_node");
    myPublisher pub;
    //ros::NodeHandle n;
    //ros::Subscriber sub;
    //sub = n.subscribe("abc", 100, callback);
    ros::Rate rate(10);
    ROS_INFO_STREAM("NODE STARTED");

    while(!stop){
        pub.publish();
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO_STREAM("EXIT");

  return 0;
}
