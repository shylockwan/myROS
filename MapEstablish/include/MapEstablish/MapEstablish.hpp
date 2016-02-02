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
#include <libpkg/report.h>
#include "Map.h"
#pragma once


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
	StateWait_AutoPatr(ros::NodeHandle nh)
	{
		flag_set=0;
		client_autoPatr=nh.serviceClient<AutoPatrol::autopatrol_service>("/autopatrol_service");
		apresult_server=nh.advertiseService("/autotrack_map/report",&StateWait_AutoPatr::apresultService,this);
		ap_timeout=200;
		ap_result=-1;
	};
	~StateWait_AutoPatr(){};
	bool onInit();
	void stateHandle(EstablishMapEvent& state_,RobotStatus& robot_status_);

private:
	bool apresultService(libpkg::report::Request& req,libpkg::report::Response& resp );
	bool ap_result;
	bool flag_set;
	int ap_timeout;
	ros::ServiceClient client_autoPatr;
	ros::ServiceServer apresult_server;
};

class StateLogMap:public StateExecutor{
public:
	StateLogMap(Map* map_,ros::NodeHandle nh):m_(NULL){
		if(map_==NULL)
		{
			std::cout<<"Map对象未初始化"<<std::endl;
			return;
		}
		qr_img=NULL;
		m_=map_;
		last_angle=0.0;
	};
	~StateLogMap(){
		delete qr_img;
	};
	bool onInit();
	void stateHandle(EstablishMapEvent& state_,RobotStatus& robot_status_);
	void getImg(cv::Mat* qr_);

private:
	std::vector<std::vector< cv::Point> > contoursDetect(cv::Mat im);
	void detectContours(cv::Mat& img, std::vector<std::vector<cv::Point> >& contours, std::vector<cv::Vec4i>& hierarchy, bool flagRB=0);
	void LeastsquareFit1(std::vector<cv::Point2f >& ptset,cv::Mat& A);
	void calcContourAngle(std::vector<cv::Point2f> ptset, float& angle,
				cv::Point2f cen_);
	void calcRouteAngle(std::vector<cv::Point> pt_pics,const RobotStatus& rs,float& world_angle,const int& imgwidth_,const int& imgheight_);


	Map* m_;
	cv::Mat* qr_img;	
	float last_angle;
};

class StateQrScan:public StateExecutor{
public:
	StateQrScan(ros::NodeHandle nh){
		qr_img=NULL;
	};
	~StateQrScan(){delete qr_img;};
	bool onInit();
	void stateHandle(EstablishMapEvent& state_,RobotStatus& robot_status_);
	void getImg(cv::Mat* qr_);

private:
	cv::Mat* qr_img;

};

}
