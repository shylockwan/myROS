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

#pragma once
namespace establishmap{
class EstablishMap_ros:public myros::StateDirector{
public:
	EstablishMap_ros(ros::NodeHandle& nh);
	~EstablishMap_ros();
	bool onInit();
	void stateUpdate();

	 ros::NodeHandle nh_;
	 EstablishMapEvent e_;
	RobotStatus rs_;
	bool shutdown_requested;
	bool flag_eventFinish;
	bool flag_MapFinished;
private:


	ecl::Thread thread;
    ///////////////////info&parameter///////////////

	StateExecutor* se_;
    StateExecutor* ready_se;
	StateExecutor* waitloc_se;
	StateExecutor* waitauto_se;
	StateExecutor* logmap_se;

	ros::Rate update_rate;
	ros::Rate process_rate;

	/////////////////bool_flag///////////////////
	bool flag_ergod;
};
}

