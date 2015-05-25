#include <ros/ros.h>
#include "my_ros/control_param.h"
#include <geometry_msgs/Twist.h>
#include <list>
#define RADIUS (0.1*0.2)
namespace myros {
class mycontroller {
public:
	ros::Subscriber sub_;
	ros::Publisher pub_;
	ros::NodeHandle node_;
	mycontroller(ros::NodeHandle n) :
			FUYANG(0.8), HEIGHT(32.0) ,node_(n){
		list_w = new std::list<float>(5, 0.0);
	}
	;
	~mycontroller() {
			delete list_w;
			 setTwist();
			 pub_.publish(twist_msg);
		}
		;
	bool init() {
		ros::param::get("fuyang", FUYANG);
		ros::param::get("height", HEIGHT);
		sub_ = node_.subscribe<my_ros::control_param>("/control_param", 1,&mycontroller::callBack,this);
		pub_ = node_.advertise<geometry_msgs::Twist>(
				"/mobile_base/commands/velocity", 1);
		ROS_INFO(
				"my_controller  has been init   and wait for arrival of control_param !!!");
		return true;
	};
	void callBack(const my_ros::control_param::ConstPtr& msg) {
		float weigh_w[5] = { 0.5, 0.3, 0.14, 0.04, 0.02 };
		ROS_INFO("dist: %f   theta1: %f   theta2: %f RADIUS:  %f ", msg->dist,
				msg->theta1, msg->theta2, RADIUS);
		float d = msg->dist; //dist>0:left  ; dist<0:right;
		float w = -msg->theta1; //theat>0:conterclockwise  ; theta<0:clockwise
		w /= (RADIUS * 20);
		if (w > 1.047)
			w = 1.047;
		if (w < -1.047)
			w = -1.047;
		list_w->push_front(w);
		list_w->pop_back();
		int ind = 0;
		w = 0;
		for (std::list<float>::iterator it = list_w->begin();
				it != list_w->end(); it++, ind++) {
			w += (*it) * weigh_w[ind];
			//std::cout<<(*it)*weigh_w[ind]<<std::endl;
			//ROS_INFO("w :%f :", w);
		}
		setTwist(0.1,0,0,0,0,w);
		//ROS_INFO("linear :%f , angular: %f",twist_msg.linear.x,twist_msg.angular.z);
	}
	void spin()
	{
		ros::spinOnce();
		pub_.publish(twist_msg);

	}
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
private:
	std::list<float>* list_w;
	float FUYANG, HEIGHT;
	geometry_msgs::Twist twist_msg;
};
}
