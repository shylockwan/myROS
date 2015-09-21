#include  <ros/ros.h>
#include <EstablishMap_ros.hpp>
#include <establishmap/establishmap_service.h>
//
 static establishmap::EstablishMap_ros* em;
bool establishmapService(establishmap::establishmap_service::Request& req,establishmap::establishmap_service::Response& rep){
	switch(req.cmd)//rep.result=0：地图构建启动//1：地图构建完成//2：地图正在构建//3：命令错误
	{
	case 0://命令重新启动
		if(em->getMapState()==0)
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
		if(em->getMapState()==0)
			rep.result=2;
		else
			rep.result=1;
			break;
	case 2:// 命令空白启动
		em->disable();
		em->resetMap();
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
	ros::Rate looprate(2);
	if(em->onInit())
	{
		while (ros::ok()) {
			ros::spinOnce();
			em->stateUpdate();
			em->mapReport();
			looprate.sleep();
		}
	}
	delete em;
	return 0;
	}
