#include <ros/ros.h>
#include <AutoPatrol/autopatrol_service.h>
int main(int argc,char** argv)
{
	ros::init(argc,argv,"test_AP");
	ros::NodeHandle nh_;
	ros::ServiceClient test_client=nh_.serviceClient<AutoPatrol::autopatrol_service>("/autopatrol_service");
	AutoPatrol::autopatrol_service srv;
	int id_from;
	int id_to;
	float angle_from;
	float angle_to;
	bool flag_uturn;
	int cmd;
	int cmd_from=999;

	ros::param::get("testAP/id_from", id_from);
	ros::param::get("testAP/id_to", id_to);
	ros::param::get("testAP/angle_from", angle_from);
	ros::param::get("testAP/angle_to", angle_to);
	ros::param::get("testAP/cmd", cmd);
	ros::param::get("testAP/flag_uturn", flag_uturn);

	std::cout<<id_from<<"  "<<id_to<<std::endl;
	std::cout<<angle_from<<"  "<<angle_to<<std::endl;
	std::cout<<cmd<<"  "<<flag_uturn<<std::endl;
	srv.request.angle_from=angle_from;
	srv.request.angle_to=angle_to;
	srv.request.cmd=cmd;
	srv.request.cmd_from=cmd_from;
	srv.request.id_from=id_from;
	srv.request.id_to=id_to;
	srv.request.flag_uturn=flag_uturn;
	if(test_client.call(srv))
	{
		std::cout<<"return result is"<<srv.response.ap_result<<std::endl;
	}
	else
	{
		std::cout<<"call service fail"<<std::cout;
	}
	ros::spin();
	return 0;
}
