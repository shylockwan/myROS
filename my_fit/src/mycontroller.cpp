#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "my_fit/control_param.h"
#include <geometry_msgs/Twist.h>


static geometry_msgs::Twist  twist_msg;

 void  callback(const my_fit::control_param::ConstPtr& msg)
{
    ROS_INFO(" control_param is %f and %f !!!",msg->dist,	 msg->theta);
    float d=msg->dist;//dist>0:left  ; dist<0:right;
    float t=msg->theta;//theat>0:conterclockwise  ; theta<0:clockwise
   
     
      ////第二种情况；角度ok，距离越界
    int tempd=int(d/30);
    int tempt=int(t*10);
		     // ROS_INFO("tempd  :  %d  ,tempt  :  %d",tempd,tempt);
    float w=float(tempd)/10+float(tempt)/20;
    twist_msg.angular.x=0.1;
    twist_msg.angular.y=0;
    twist_msg.angular.z=0;
     twist_msg.linear.x=0;
     twist_msg.linear.y=0;
     twist_msg.linear.z=w;
  ROS_INFO("linear :%f , angular: %f",twist_msg.linear.x,twist_msg.angular.z);

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_controller");
  ros::NodeHandle nh_;
  ros::Subscriber sub_=nh_.subscribe("/myfit/control_param", 1000,callback);
  ROS_INFO("my_controller  has been init   and wait for arrival of control_param !!!");
  ros::Publisher pub_=nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  twist_msg.angular.x=0;
  twist_msg.angular.y=0;
  twist_msg.angular.z=0;
     twist_msg.linear.x=0;
     twist_msg.linear.y=0;
     twist_msg.linear.z=0;
  ros::Rate loop_rate(100);
 while (ros::ok())
  {
    pub_.publish(twist_msg);
   

    ros::spinOnce();

    loop_rate.sleep();
    
  }
  twist_msg.angular.x=0;
  twist_msg.angular.y=0;
  twist_msg.angular.z=0;
     twist_msg.linear.x=0;
     twist_msg.linear.y=0;
     twist_msg.linear.z=0;
     pub_.publish(twist_msg);
  return 0;
}

