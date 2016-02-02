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
#include <AutoPatrol/autopatrol_service.h>
#include "whw_map.hpp"
#include <std_msgs/Empty.h>
#include<std_msgs/String.h>
#pragma once


void decodeQR(){};
namespace establishmap{

class StateExecutor{
public:
	virtual bool onInit()=0;
	StateExecutor(){};
	virtual void stateHandle(EstablishMapEvent& state_,RobotStatus& robot_status_)=0;
	virtual ~StateExecutor(){};
private:

};

class StateReady:public StateExecutor{
public:
	StateReady(){};
	~StateReady(){};
	bool onInit();
	void stateHandle(EstablishMapEvent& state_,RobotStatus& robot_status_);

private:

};

class StateWait_SelfLoc:public StateExecutor{
public:
	StateWait_SelfLoc(ros::NodeHandle nh)
	{
	}
	~StateWait_SelfLoc(){};
	bool onInit();
	void stateHandle(EstablishMapEvent& state_,RobotStatus& robot_status_);

private:

	ros::Subscriber sub_waitselfLoc;
	ros::NodeHandle waitselfLoc_nh;
	ros::ServiceClient client_selfLoc;
	ros::Publisher pub_resetOdom;

};

class StateWait_AutoPatr:public StateExecutor{
public:
	StateWait_AutoPatr(ros::NodeHandle nh):srvAp_nh(nh)
	{
	};
	~StateWait_AutoPatr(){};
	bool onInit();
	void stateHandle(EstablishMapEvent& state_,RobotStatus& robot_status_);
private:
	ros::NodeHandle srvAp_nh;
	ros::ServiceClient client_autoPatr;
};

class StateLogMap:public StateExecutor{
public:
	StateLogMap(whw::whwMap* map_,ros::NodeHandle nh):last_angle(0.0),m_(NULL),state_lm(nh){
		if(map_==NULL)
		{
			std::cout<<"Map对象未初始化"<<std::endl;
			return;
		}

		m_=map_;            
	};
	~StateLogMap(){};
	bool onInit();
	void stateHandle(EstablishMapEvent& state_,RobotStatus& robot_status_);
	void setPtrQr(cv::Mat* qr_){
		if(qr_==NULL){
			std::cout<<"qr_img对象未初始化"<<std::endl;
			return;
		}
		qr_img=qr_;
	}

private:
	whw::whwMap* m_;
	cv::Mat* qr_img;	
	float last_angle;
	ros::Publisher pub_qr;
	ros::ServiceClient qrdecode_client ;
	ros::NodeHandle state_lm;
};

class StateQrScan:public StateExecutor{
public:
	StateQrScan(whw::whwMap* map_,ros::NodeHandle nh):last_angle(0.0),m_(NULL),state_lm(nh){
		if(map_==NULL)
		{
			std::cout<<"Map对象未初始化"<<std::endl;
			return;
		}

		m_=map_;
	};
	~StateQrScan(){};
	bool onInit();
	void stateHandle(EstablishMapEvent& state_,RobotStatus& robot_status_);
	void getImg(cv::Mat* qr_);

private:
	whw::whwMap* m_;
	cv::Mat* qr_img;
	float last_angle;
	ros::Publisher pub_qr;
	ros::ServiceClient qrdecode_client ;
	ros::NodeHandle state_lm;
};

}
