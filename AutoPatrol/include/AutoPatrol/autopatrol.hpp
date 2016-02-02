#include <ros/ros.h>
#include "my_events.hpp"
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>

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
		curve_counts=10;
		resetPID();
		file.open("path.txt",std::ios::out);
	};
	~StateAP(){file.close();};
	bool onInit();
	void stateHandle(const cv::Mat& src,AutoPatrolEvent& event,RobotStatus& robot);
	void resetPID(){
		pid_.resetError();
	}
private:
	std::fstream file;
	float theta_robot_world;
	cv::Point2f pt_robot_world;
	cv::Point2f pt_robot;
	PID pid_;
	geometry_msgs::Twist calcObjTwist(std::vector<cv::Point> ptset,const int& width,const int &height,RobotStatus& rs);
	int curve_counts;

	std::vector<cv::Point> ptset_pre; //上一帧图像点
	std::vector<cv::Point2f> ptset_world_pre; //上一帧世界点
	geometry_msgs::Twist t_pre;
	cv::Point2f obj_point;
	/////////////////////////////////////////////////////////////////////////////////////////
	cv::Point2f getObjPoint(std::vector<cv::Point>& ptset,const RobotStatus& robot,const int& width,const int& height);
	bool doGetObjPt(std::vector<cv::Point2f> &ptset, cv::Point2f& obj,const cv::Point2f& ptrobot);
	void drawPath(std::vector<cv::Point2f>& ptset,const cv::Mat& param);
	float pathFunc(float x, const cv::Mat& param) {
		if (param.empty()) {
			std::cout << "paramA has not been initialed" << std::endl;
			return -1;
		}

		//float a0,a1,a2,a3;
		//a0=paramA.at<double>(0,0);
		//a1=paramA.at<double>(1,0);
		//a2=paramA.at<double>(2,0);
		//a3=paramA.at<double>(3,0);
		float y = param.at<double>(0, 0) + param.at<double>(1, 0) * x
				+ param.at<double>(2, 0) * x * x
				+ param.at<double>(3, 0) * x * x * x;
		return y;
	}
	;

	float myfunc(float x, float y, cv::Point2f pt_robot, const float radius = 20.0) {
		float res = (x - pt_robot.x) * (x - pt_robot.x)
				+ (y - pt_robot.y) * (y - pt_robot.y) - radius * radius; //cm
		return res;
	}
	;
	void LeastsquareFit3(std::vector<cv::Point2f> &ptset, cv::Mat& paramA) {
		if (ptset.empty())
		{
			std::cout<<"LeastsquareFit3 ptset空"<<std::endl;
			return ;
		}
		cv::Mat_<double> F, Y;
		F.create(4, ptset.size());
		Y.create(ptset.size(), 1);
		for (int i = 0; i != ptset.size(); i++) {
			///X和Y置换得到x=f(y);
			double pt_x = ptset[i].y;
			F(0, i) = 1;
			F(1, i) = pt_x;
			F(2, i) = pt_x * pt_x;
			F(3, i) = pt_x * pt_x * pt_x;
			Y(i, 0) = ptset[i].x;

		} //for1 loop
		cv::Mat F_Ft;
		cv::mulTransposed(F, F_Ft, false);
		cv::Mat temp = (F_Ft.inv());
		//std::cout<<temp.rows<<"  "<<temp.cols<<std::endl<<F.rows<<" "<<F.cols;
		paramA = temp * F * Y; //a0 a1 a2 a3;opencv中貌似已经包含cv::solve函数用于求解

	} //LeastsquareFit;
//////////////////////////////PID///////////////////////////////////////////////////////////
	float myPID(float cur_,float obj_=0.0);
	float getDiffdist(cv::Point2f pt_obj,cv::Point2f pt_robot,float diffangle);
	float getDiffangle(cv::Point2f pt_obj,cv::Point2f pt_robot);
	float getDiffangleV2(std::vector<cv::Point2f> &ptset);
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


