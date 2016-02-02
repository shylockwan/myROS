//==============================================================================
//函数名： autopatrol_ros类的声明部分
//作者：王华威
//日期:  2015-7-10
//功能: 等待客户端接收消息或者命令，切换状态并执行事件
//修改记录：
//==============================================================================
#include <my_events.hpp>
#include <state_director.hpp>
#include <MapEstablish.hpp>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/SensorState.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include <ecl/threads.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <libpkg/report.h>
#include "Map.h"

#pragma once
namespace establishmap {
class EstablishMap_ros: public myros::StateDirector {
public:
	EstablishMap_ros(ros::NodeHandle& nh);
	~EstablishMap_ros();
	bool onInit();
	void stateUpdate();
	void enable();
	void disable();
	bool getState();
	void processEvent();
	void Report();
	void resetMap();
	void getRequestID(const int& id);
	bool flag_selfLoc;
private:
	////////////////////ROS///////////////////////////////////
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void encoderCallback(const kobuki_msgs::SensorState &msg);
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	ros::Subscriber odom_sub;
	ros::Subscriber encoder_sub;
	image_transport::Subscriber sub_img;
	//////////////////////////
	EstablishMapEvent event_;
	RobotStatus rs_;
	//////////////////////////////////////////
	bool flag_report;
	int report_;
	int cmd_from;
	libpkg::report repo_srv;
	ros::ServiceClient repo_client;
	void callForReport(const std::string& srvname);
	//////////////////////////multiThread///////////////////////
	ecl::Thread thread;
	ecl::Mutex mu_;
	///////////////////info&parameter///////////////
	Map* map_;
	StateExecutor* se_;
	StateExecutor* ready_se;
	StateExecutor* waitloc_se;//自定位用，暂不用
	StateExecutor* waitauto_se;
	StateExecutor* logmap_se;
	StateExecutor* qrscan_se;

	ros::Rate update_rate;
	ros::Rate process_rate;
	cv::Mat* img_; //single channel
	//////////////////////encoder_param////////////////////////
	float time1, time2, dtime, leftv1, leftv2, rightv1, rightv2;
	/////////////////bool_flag///////////////////
	bool flag_enable;
};
}

