//==============================================================================
//函数名： autopatrol_ros类的声明部分
//作者：王华威
//日期:  2015-7-10
//功能: 等待客户端接收消息或者命令，切换状态并执行事件
//修改记录：
//==============================================================================
#include <my_events.hpp>
#include <state_director.hpp>

#include <ros/ros.h>
#include <ecl/threads/thread.hpp>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <autopatrol.h>
#pragma once

namespace autopatrol{

class AutoPatrol_ros:public myros::StateDirector
{
	public:
	AutoPatrol_ros(ros::NodeHandle& nh);
	~AutoPatrol_ros() ;
	bool onInit();
	bool processEvent();
	void stateUpdate();
    void callBack_start();
    void callBack_ps();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void enable();
	void disable();


	 ros::NodeHandle nh_;
	 AutoPatrolEvent e_;
	 RobotStatus rs_;
	 bool  shutdown_requested;
	 bool flag_eventFinish;
	private:
	//////////////////ros sub&pub//////////////////
	//ros::Subscriber sub_autopatrol;
	 image_transport::Subscriber  sub_img;

	ros::Publisher pub_autopatrol;
	ros::Publisher pub_vel;
    image_transport::ImageTransport it_;
    ros::Subscriber odom_sub;
    ///////////////////info&parameter///////////////

	StateExecutor* se_;
    StateExecutor* ready_se;
	StateExecutor* patrol_se;
	StateExecutor* route_se;

	ros::Rate update_rate;
	ros::Rate process_rate;
	cv::Mat* img_;//single channel

	/////////////////bool_flag///////////////////

	bool flag_sub;
	bool flag_enable;


	ecl::Thread thread;

	};
}
