#include <ros/ros.h>
#include "my_events.hpp"
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#pragma once
namespace autopatrol{
class StateExecutor{
public:
	virtual bool onInit()=0;
	StateExecutor();
	virtual void stateHandle(const cv::Mat& src,AutoPatrolEvent& event,RobotStatus& robot)=0;
	virtual ~StateExecutor(){};
	void setTwist(const float& lx_=0,const float& ly_=0,const float& lz_=0,
				const float& ax_=0,const float& ay_=0,const float& az_=0)
	{
			    twist_msg.angular.x = ax_;
				twist_msg.angular.y = ay_;
				twist_msg.angular.z = az_;
				twist_msg.linear.x = lx_;
				twist_msg.linear.y = ly_;
				twist_msg.linear.z = lz_;
	}
	void setTwist(const geometry_msgs::Twist& t_ )
		{
				    twist_msg.angular.x = t_.angular.x;
					twist_msg.angular.y = t_.angular.y;
					twist_msg.angular.z = t_.angular.z;
					twist_msg.linear.x = t_.linear.x;
					twist_msg.linear.y = t_.linear.y;
					twist_msg.linear.z = t_.linear.z;
					//std::cout << "setTwist mmmmmmmmmmmmmmmmmmmmmmmmmm  t_.angular.z=" << t_.angular.z<< std::endl;					
		}
	virtual  void reset(){};

	geometry_msgs::Twist getTwist();
	void getPointset(const cv::Mat& in, std::vector<cv::Point>& ptset) ;

	bool processImg(const cv::Mat& src);
	std::vector<cv::Point> ptset_line,ptset_QR;

private:
	geometry_msgs::Twist twist_msg;


};

///////////////////////////////////////////////////////////////////////
class StateReady:public StateExecutor{
public:
	StateReady(){};
	~StateReady(){};
	bool onInit();
	void stateHandle(const cv::Mat& src,AutoPatrolEvent& event,RobotStatus& robot);
private:

};

///////////////////////////////////////////////////////////////////////
class StateAP:public StateExecutor{
public:
	StateAP(){
		curve_counts=15;
	};
	~StateAP(){};
	bool onInit();
	void stateHandle(const cv::Mat& src,AutoPatrolEvent& event,RobotStatus& robot);
private:
	PID pid_;
	geometry_msgs::Twist calcObjTwist(std::vector<cv::Point> ptset,const int& width,const int &height,RobotStatus& rs);
	float myPID(float cur_,float obj_=0.0);
	float getDiffangle(cv::Point cen_);
	float getDiffdist(cv::Point cen_);
	void getObjPoint(std::vector<cv::Point>& ptset);
	cv::Point start_point,end_point,mid_point;

	int curve_counts;

	std::vector<cv::Point> ptset_pre; //上一帧
	geometry_msgs::Twist t_pre;
};

///////////////////////////////////////////////////////////////////////
class StateTurnAngle:public StateExecutor{
public:
	StateTurnAngle():delta_angle(0),flag_set(0),last_theta(0.0),cur_theta(0.0),qr_angle(0.0)
	{
		setObjAngle();
	};
	~StateTurnAngle(){};
	bool onInit();
	void stateHandle(const cv::Mat& src,AutoPatrolEvent& event,RobotStatus& robot);
	void setObjAngle(float obj_=0);
    float getObjAngle();
    void reset(){flag_set=0; delta_angle =0;}
	
private:
	bool flag_set;
	float qr_angle;
	float delta_angle;
	float obj_angle;
	float last_theta;
	float cur_theta;
	ros::NodeHandle n_;
};

///////////////////////////////////////////////////////////////////////
class StateUturn:public StateExecutor{
public:
	StateUturn():delta_angle(0),flag_set(0),last_theta(0.0),cur_theta(0.0){
		setObjAngle();
	};
	~StateUturn(){};
	bool onInit();
	void stateHandle(const cv::Mat& src,AutoPatrolEvent& event,RobotStatus& robot);
	void setObjAngle(float obj_=3.14);
    float getObjAngle();
    float haveTurnAngle();
    void reset(){ flag_set=0; delta_angle =0;}
private:
	bool flag_set;
	float delta_angle;
	float obj_angle;
	float last_theta;
	float cur_theta;
};

///////////////////////////////////////////////////////////////////////

class StateQrloc:public StateExecutor{
public:
	StateQrloc(){

	};
	~StateQrloc(){};
	bool onInit();
	void stateHandle(const cv::Mat& src,AutoPatrolEvent& event,RobotStatus& robot);

private:
	ros::ServiceClient client_qrdecode;

};

///////////////////////////////////////////////////////////////////////

class StateQrcover:public StateExecutor{
public:
	StateQrcover():flag_set(0),cur_coor(0),last_coor(0),obj_dist(0.0),delta_dist(0.0){
		;
	};
	~StateQrcover(){};
	bool onInit();
	void stateHandle(const cv::Mat& src,AutoPatrolEvent& event,RobotStatus& robot);

private:
	bool flag_set;
	cv::Point2f cur_coor;
	cv::Point2f last_coor;
	float obj_dist;
	float delta_dist;

};
}


