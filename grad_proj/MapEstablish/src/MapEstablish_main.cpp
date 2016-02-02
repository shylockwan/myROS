#include  <ros/ros.h>
#include <MapEstablish_ros.hpp>
#include <MapEstablish/establishmap_service.h>
#include <std_msgs/Empty.h>
//
 static establishmap::EstablishMap_ros* em;
 static ros::Publisher resetodom;
bool establishmapService(MapEstablish::establishmap_service::Request& req,MapEstablish::establishmap_service::Response& rep){

	switch(req.cmd)//rep.result=0：地图构建启动//1：地图构建完成//2：地图正在构建//3：命令错误
	{
	case 0://命令重新启动resume
		if(em->getState()==0)
		{
			em->enable();
			rep.result=2;
		}
		else
		{
			em->disable();
			rep.result=1;
		}
		break;
	case 1:
		em->disable();//命令停止
		if(em->getState()==0)
			rep.result=2;
		else
			rep.result=1;
			break;
	case 2:// 命令空白启动
		em->disable();
		em->resetMap();
		resetodom.publish(std_msgs::Empty());
		em->enable();
		rep.result=0;
		break;
	default:
		rep.result=3;
		break;
	}

	return 1;
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"ServiceNode_EstablishMap");
	ros::NodeHandle nh_srv;
	ros::NodeHandle nh_em;
	ros::ServiceServer em_server=nh_srv.advertiseService("establishmap_service",establishmapService);
	em=new establishmap::EstablishMap_ros(nh_em);
	resetodom=nh_em.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry",1);
	resetodom.publish(std_msgs::Empty());
	ros::Rate looprate(5);
	if(em->onInit())
	{
		while (ros::ok()) {
			ros::spinOnce();
			em->stateUpdate();
			em->Report();
			looprate.sleep();
		}
	}
	delete em;
	return 0;
	}
