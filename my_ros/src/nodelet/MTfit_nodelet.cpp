#include <pluginlib/class_list_macros.h>
#include <boost/shared_ptr.hpp>
#include <class_myfit.hpp>
#include <ecl/threads/thread.hpp>
#include <nodelet/nodelet.h>
#include <class_mymap.hpp>
#include "my_ros/control_param.h"
namespace myros
{
class fit_nodelet:public nodelet::Nodelet
{
public:
	fit_nodelet() : shutdown_requested_(false)  {ROS_INFO("fit_nodelet constructing" ); };
	  ~fit_nodelet()
	  {
	    ROS_INFO("Waiting for update thread to finish.");
	    shutdown_requested_ = true;
	    myfit_.reset();
	    mymap_.reset();

	    update_thread_.join();
	  }
	void onInit()
	{
		ros::NodeHandle nh_=getPrivateNodeHandle();
		//std::string name = nh_.getUnresolvedNamespace();
	    //int pos = name.find_last_of('/');
	    //name = name.substr(pos + 1);
		myfit_.reset(new myfit(nh_));
		mymap_.reset(new mymap(nh_));
		if(myfit_->init()  )
			{
				ROS_INFO("fit_Nodelet initialised");
			    //this->update();
				//ROS_INFO("Kobuki initialised. Spinning up update thread ... [ %s]",name);
				update_thread_.start(&fit_nodelet::update,*this);
				//ROS_INFO("Nodelet initialised");
			}


	}

private:
	//ros::Subscriber sub_;
	ecl::Thread update_thread_;
	boost::shared_ptr<myfit> myfit_;
	boost::shared_ptr<mymap> mymap_;
	 bool shutdown_requested_;
	 void update() {
		ros::Rate spin_rate(5);
		 // enable the controller when loading the nodelet
		ROS_INFO("fit_Nodelet updating by rate of 5hz");

		 while (! shutdown_requested_ && ros::ok())
		    {
			myfit_->spin();//myfit_接收usb_cam图像，调用imgCb处理
		    mymap_->updateMap(myfit_->pt_robot, myfit_->pt_obj);
			spin_rate.sleep();
		    }
		}


};
}

PLUGINLIB_EXPORT_CLASS(myros::fit_nodelet,nodelet::Nodelet);

