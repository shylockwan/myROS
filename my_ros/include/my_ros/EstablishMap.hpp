//==============================================================================
//函数名： StateExecutor类的声明部分以及相关继承类的声明
//作者：王华威
//日期:  2015-7-10
//功能: 用于状态转换时执行相应动作
//修改记录：
//==============================================================================
#include <ros/ros.h>
#include "my_events.hpp"
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#pragma once
namespace establishmap{

class StateExecutor{
public:
	virtual bool onInit()=0;
	StateExecutor();
	virtual void stateHandle(const cv::Mat& src,AutoPatrolEvent& state_,RobotStatus& robot_status_)=0;
	virtual ~StateExecutor(){};
private:

};
class StateReady:public StateExecutor{
public:
	StateReady(){};
	~StateReady(){};
	bool onInit();
	void stateHandle(const cv::Mat& src,AutoPatrolEvent& state_,RobotStatus& robot_status_);

private:

};
class StateWait_SelfLoc:public StateExecutor{
public:
	StateWait_SelfLoc(){};
	~StateWait_SelfLoc(){};
	bool onInit();
	void stateHandle(const cv::Mat& src,AutoPatrolEvent& state_,RobotStatus& robot_status_);

private:

};
class StateWait_AutoPatr:public StateExecutor{
public:
	StateWait_AutoPatr(){};
	~StateWait_AutoPatr(){};
	bool onInit();
	void stateHandle(const cv::Mat& src,AutoPatrolEvent& state_,RobotStatus& robot_status_);
};

class StateLogMap:public StateExecutor{
public:
	StateLogMap(){};
	~StateLogMap(){};
	bool onInit();
	void stateHandle(const cv::Mat& src,AutoPatrolEvent& state_,RobotStatus& robot_status_);
};

}
