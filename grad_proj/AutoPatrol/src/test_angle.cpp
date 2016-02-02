#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

static float m_ThetaRobot;
static cv::Point2f m_PtRobot;
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	//ROS_INFO("(%f ,%f)  ,%f",msg->linear.x,msg->linear.y,msg->angular.z);

	geometry_msgs::Quaternion quat = msg->pose.pose.orientation;
	m_PtRobot.y = 100 * msg->pose.pose.position.x; //单位 m->cm
	m_PtRobot.x = -100 * msg->pose.pose.position.y;
	m_ThetaRobot = tf::getYaw(quat); //单位 rad
};
int main(int argc,char**argv)
{
	ros::init(argc,argv,"test_angle");
	ros::NodeHandle nh_;
	ros::Subscriber odom_sub=nh_.subscribe<nav_msgs::Odometry>("/odom", 1,
			&odomCallback);
	ros::Rate rate(5);
	while (ros::ok())
	{
		ros::spinOnce();
		std::cout<<"m_ThetaRobot="<<m_ThetaRobot<<"m_PtRobot= ("<<m_PtRobot.x<<" ,"<<m_PtRobot.y<<" )"<<std::endl;
		rate.sleep();
	}

}
