//==============================================================================

#include <my_events.hpp>
#include <state_director.hpp>
#include "autopatrol.hpp"
#include <ros/ros.h>
#include <ecl/threads.hpp>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <libpkg/report.h>

#pragma once
namespace autopatrol{
class AutoPatrol_ros:public myros::StateDirector{
public:
	bool onInit();
	AutoPatrol_ros(ros::NodeHandle& nh);
	~AutoPatrol_ros();
	void stateUpdate();
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
	void enable();
	void disable();
	void processEvent();
	void Report();
	void getRequestID(const int& id);
	void getPathInfo(const PathLink& route);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) ;
	bool getState();
	void setQrAngle(const float& angle);
private:
	void callForReport(const std::string& srvname);
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	AutoPatrolEvent event_;
	image_transport::Subscriber  img_sub;
	ros::Subscriber odom_sub;

	ros::Publisher vel_pub;
	//std::vector<ros::ServiceClient> repo_client;
	int cmd_from;
	ros::ServiceClient repo_client;
	libpkg::report repo_srv;
	RobotStatus rs_;
	bool flag_qr;
	////////////////////////////////////////////////////////
	ecl::Thread event_thread;
	ecl::Mutex mu_;
	///////////////////State&parameter///////////////

	StateExecutor* se_; //Base
	StateExecutor* ready_se;
	StateExecutor* ap_se;
	StateExecutor* uturn_se;
	StateExecutor* turnangle_se;
	StateExecutor* qrloc_se;
	StateExecutor* qrcover_se;

	ros::Rate process_rate; //状态执行频率
	cv::Mat* img_; //single channel
	bool flag_enable;
};
}
