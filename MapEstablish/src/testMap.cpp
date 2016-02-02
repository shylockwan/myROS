#include <ros/ros.h>
#include <MapEstablish/establishmap_service.h>
#include <libpkg/report.h>

bool repoCallback(libpkg::report::Request& req,libpkg::report::Response& resp)
{
	std::cout<<"get report from :"<<req.id<<" whose result is : "<<req.srv_result<<std::endl;
	return 0;
}
int main(int argc,char** argv)
{
	ros::init(argc,argv,"test_Map");
	ros::NodeHandle nh_;
	ros::ServiceClient test_client=nh_.serviceClient<MapEstablish::establishmap_service>("/establishmap_service");
	MapEstablish::establishmap_service srv;

	int cmd=-1;
	int cmd_from=999;
	ros::ServiceServer getrepo_serv=nh_.advertiseService("/map_test/report",&repoCallback);
	ros::param::get("testMap/cmd", cmd);


	std::cout<<"cmd: "<<cmd<<"  "<<std::endl;

	srv.request.cmd=cmd;
	srv.request.cmd_from=cmd_from;
	if(test_client.call(srv))
	{
		std::cout<<"return result is"<<srv.response.result<<std::endl;
	}
	else
	{
		std::cout<<"call service fail"<<std::cout;
	}
	ros::spin();
	return 0;
}
