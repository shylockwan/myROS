#include <pluginlib/class_list_macros.h>
#include <boost/shared_ptr.hpp>
#include <class_mymap.hpp>
#include <ecl/threads/thread.hpp>
#include <nodelet/nodelet.h>

namespace myros {
class map_nodelet {
public:
	map_nodelet() :
			shutdown_requested_(false) {
	}
	;
	void onInit() {
		ros::NodeHandle nh_ =getPrivateNodeHandle();
		mymap_.reset(new mymap(nh_));
		if(mymap_->init())
		{
			this->update();
		}

	}
private:
	void update() {
		ros::Rate spin_rate(5);
		// enable the controller when loading the nodelet
		while (!shutdown_requested_ && ros::ok()) {
			ros::spinOnce();
			mymap_->updateMap();
			spin_rate.sleep();
		}
	}
	bool shutdown_requested_;
	boost::shared_ptr<mymap> mymap_;
};
}

PLUGINLIB_EXPORT_CLASS(myros::mymap, nodelet::Nodelet);
