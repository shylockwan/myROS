#include <pluginlib/class_list_macros.h>
#include <boost/shared_ptr.hpp>
#include <class_mycontroller.hpp>
#include <ecl/threads/thread.hpp>
#include <nodelet/nodelet.h>

namespace myros {
class contro_nodelet: public nodelet::Nodelet {
public:
	contro_nodelet() :
			shutdown_requested_(false) {
	}
	;
	~contro_nodelet() {
		ROS_INFO("Waiting for update thread to finish.");
		shutdown_requested_ = true;
		mycontroller_.reset();
		//update_thread_.join();
	}
	void onInit() {
		ros::NodeHandle nh_=getPrivateNodeHandle();
		mycontroller_.reset(new mycontroller(nh_));
		if (mycontroller_->init())
		{
				this->update();
			}

	}
private:
	boost::shared_ptr<mycontroller> mycontroller_;
	bool shutdown_requested_;
//	ecl::Thread update_thread_;
	void update() {
		ros::Rate looprate(5);
		while (!shutdown_requested_ && ros::ok()) {
			mycontroller_->spin();
			looprate.sleep();
		}
	}

};
}
PLUGINLIB_EXPORT_CLASS(myros::contro_nodelet,nodelet::Nodelet);
