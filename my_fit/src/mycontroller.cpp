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
    if(d>30&&abs(t)<0.1)
    {
   // ROS_INFO(" 1");
     twist_msg.angular.x=0;
     twist_msg.angular.y=0;
     twist_msg.angular.z=0.1;
     twist_msg.linear.x=0.1;
     twist_msg.linear.y=0;
     twist_msg.linear.z=0;
    	
    }
    if(d<-30&&abs(t)<0.1)
       {
      // ROS_INFO(" 2");
          twist_msg.angular.x=0;
   	  twist_msg.angular.y=0;
   	  twist_msg.angular.z=-0.1;
   	  twist_msg.linear.x=0.1;
    	  twist_msg.linear.y=0;
   	  twist_msg.linear.z=0;
       }
    ////第三种情况；距离ok，角度越界
    if(abs(d)<30&&t>0.1)
       {
    //   ROS_INFO(" 3");
       twist_msg.angular.x=0.0;
   	  twist_msg.angular.y=0.0;
   	  twist_msg.angular.z=-0.1;
   	  twist_msg.linear.x=0.0;
    	  twist_msg.linear.y=0.0;
   	  twist_msg.linear.z=0.0;
       	
       }
    if(abs(d)<30&&t<-0.1)
       {
       //ROS_INFO(" 4");
        twist_msg.angular.x=0.0;
   	  twist_msg.angular.y=0.0;
   	  twist_msg.angular.z=0.1;
   	  twist_msg.linear.x=0.0;
    	  twist_msg.linear.y=0.0;
   	  twist_msg.linear.z=0.0;
       	
       }
    ////第四种情况；距离和角度均不符；
    if((d<-30&&t<-0.1)||(d>30&&t>0.1)||(abs(d)<30&&abs(t)<0.1))
    {
   // ROS_INFO(" 5");
     twist_msg.angular.x=0.0;
   	  twist_msg.angular.y=0.0;
   	  twist_msg.angular.z=0.0;
   	  twist_msg.linear.x=0.1;
    	  twist_msg.linear.y=0.0;
   	  twist_msg.linear.z=0.0;
    	
    }
    if((d<-30&&t>0.1))
        {
    //    ROS_INFO(" 6");
         twist_msg.angular.x=0.0;
   	  twist_msg.angular.y=0.0;
   	  twist_msg.angular.z=-0.1;
   	  twist_msg.linear.x=0.0;
    	  twist_msg.linear.y=0.0;
   	  twist_msg.linear.z=0.0;
        	
        }
    if(d>30&&t<-0.1)
        {
      //  ROS_INFO(" 7");
        twist_msg.angular.x=0.0;
   	  twist_msg.angular.y=0.0;
   	  twist_msg.angular.z=0.1;
   	  twist_msg.linear.x=0.0;
    	  twist_msg.linear.y=0.0;
   	  twist_msg.linear.z=0.0;
      
        }
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

