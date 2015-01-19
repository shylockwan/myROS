#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "my_fit/control_param.h"
#include <geometry_msgs/Twist.h>

static std::list<float>* list_w;
static geometry_msgs::Twist  twist_msg;
static float weigh_w[5]={0.5,0.24,0.1,0.04,0.02};
 void  callback(const my_fit::control_param::ConstPtr& msg)
{
    ROS_INFO(" control_param is %f and %f !!!",msg->dist,	 msg->theta);
    float d=msg->dist;//dist>0:left  ; dist<0:right;
    float t=msg->theta;//theat>0:conterclockwise  ; theta<0:clockwise
   
   
      ////第二种情况；角度ok，距离越界
    int tempd=int(d/60);
    int tempt=int(t*10);
		     // ROS_INFO("tempd  :  %d  ,tempt  :  %d",tempd,tempt);
    float w=float(tempd)/30+float(tempt)/40;
    if (w>2)
       w=2;
    if (w<-2)
       w=-2;
      list_w->push_front(w);
      list_w->pop_back();
      int ind=0;
      w=0;
       for(std::list<float>::iterator it=list_w->begin();it!=list_w->end();it++,ind++)
      {
         w+=(*it)*weigh_w[ind];
			//std::cout<<(*it)*weigh_w[ind]<<std::endl;
			ROS_INFO("w :%f :",w);
			}
    twist_msg.angular.x=0;
    twist_msg.angular.y=0;
    twist_msg.angular.z=w;
     twist_msg.linear.x=0.1;
     twist_msg.linear.y=0;
     twist_msg.linear.z=0;
 // ROS_INFO("linear :%f , angular: %f",twist_msg.linear.x,twist_msg.angular.z);

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
   list_w=new std::list<float>(5,0);
  // weigh_w[0]=0.7;weigh_w[1]=0.19;weigh_w[2]=0.08;weigh_w[3]=0.02;weigh_w[4]=0.01;
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
     delete list_w;
  return 0;
}

