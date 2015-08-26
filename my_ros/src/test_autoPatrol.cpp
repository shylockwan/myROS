#include  <ros/ros.h>
#include <my_ros/autopatrol_service.h>

int main (int argc,char**argv)
{
	 ros::init(argc, argv, "test_autoPatrol_client");
	  ros::NodeHandle nh;
	  ros::ServiceClient ap_client=nh.serviceClient<my_ros::autopatrol_service>("autopatrol_service");
	  ROS_INFO("autoPatrol_client initialing");
	  my_ros::autopatrol_service aps_;
	  ros::Rate loop_rate(0.2);

	  	while(ros::ok())
	  {

	  		  if (ap_client.call(aps_))
	  			  {
	  			    ROS_INFO("length: %d",  aps_.response.ap_result);
	  			  }
	  			  else
	  			  {
	  			    ROS_ERROR("Failed to call service ");
	  			    return 1;
	  			  }

	  		  loop_rate.sleep();
	  	}
	  return 0;


}
