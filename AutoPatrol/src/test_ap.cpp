#include <ros/ros.h>
#include <AutoPatrol/autopatrol_service.h>
#include <libpkg/report.h>
#include <std_msgs/Empty.h>
bool repoCallback(libpkg::report::Request& req,libpkg::report::Response& resp)
{
	std::cout<<"get report from :"<<req.id<<" whose result is : "<<req.srv_result<<std::endl;
	return 0;
}
int main(int argc,char** argv)
{
	ros::init(argc,argv,"test_AP");
	ros::NodeHandle nh_;
	ros::ServiceClient test_client=nh_.serviceClient<AutoPatrol::autopatrol_service>("/autopatrol_service");
	AutoPatrol::autopatrol_service srv;
	ros::Publisher resetodom;
	resetodom=nh_.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry",1);
	resetodom.publish(std_msgs::Empty());
	int id_from=0;
	int id_to=0;
	float angle_from=0;
	float angle_to=0;
	int flag_uturn=0;
	int cmd=0;
	int cmd_from=999;
	float QR_angle=0.0;
	int flag_qr=0;
	ros::ServiceServer getrepo_serv=nh_.advertiseService("/autotrack_test/report",&repoCallback);
	ros::param::get("testAP/id_from", id_from);
	ros::param::get("testAP/id_to", id_to);
	ros::param::get("testAP/angle_from", angle_from);
	ros::param::get("testAP/angle_to", angle_to);
	ros::param::get("testAP/cmd", cmd);
	ros::param::get("testAP/flag_uturn", flag_uturn);
	ros::param::get("testAP/QR_angle", QR_angle);
	ros::param::get("testAP/flag_qr", flag_qr);

	std::cout<<"id_from "<<id_from<<"    id_to "<<id_to<<std::endl;
	std::cout<<"angle_from "<<angle_from<<"    angle_to "<<angle_to<<std::endl;
	std::cout<<"cmd "<<cmd<<"    flag_uturn "<<flag_uturn<<std::endl;
	std::cout<<"QR_angle "<<QR_angle<<"    flag_qr "<<flag_qr<<std::endl;

	srv.request.QR_angle=QR_angle;
	srv.request.flag_qr=flag_qr;
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
