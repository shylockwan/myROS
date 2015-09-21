//==============================================================================
//函数名： ROS上节点服务端启动程序
//作者：王华威
//日期:  2015-7-10
//功能: 调用autopatrol_ros类实现按需求循线移动
//修改记录：
//==============================================================================
#include <autopatrol_ros.h>
#include  <ros/ros.h>
#include <autotracking/autopatrol_service.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
 static autopatrol::AutoPatrol_ros* ap;

 bool autopatrolService(autotracking::autopatrol_service::Request& req,autotracking::autopatrol_service::Response& rep)
 {
 	/////
	ap->enable();
	ap->e_.next_route->angle_from=req.angle_from;
 	ap->e_.next_route->angle_to=req.angle_to;
 	ap->e_.next_route->id_from=req.id_from;
 	ap->e_.next_route->id_to=req.id_to;
 	ap->e_.next_route->flag_cover=req.ap_cmd;//0：正常循线，1：180度转弯
 	bool flag_result=ap->processEvent();
   	 rep.ap_result=flag_result;
        std::cout << "autopatrolService receive  autopatrol_service,  ap->e_.next_route->angle_from=" <<ap->e_.next_route->angle_from<< std::endl; 
        std::cout << "autopatrolService ap->e_.next_route->angle_to=" <<ap->e_.next_route->angle_to<< std::endl; 

 	return flag_result;
 }
 ;

int main (int argc,char**argv)
{
	  ros::init(argc, argv, "ServiceNode_AutoPatrol");
	  ros::NodeHandle nh;
	  ros::NodeHandle ap_nh;
	  ros::CallbackQueue que;
      nh.setCallbackQueue(&que);

	  ap=new autopatrol::AutoPatrol_ros(ap_nh);
	  ros::ServiceServer ap_server=nh.advertiseService("autopatrol_service",autopatrolService);

	  ros::Rate rate(5);
	  ROS_INFO("Node Auto Patrol is going to init");
	  if(ap->onInit())
	  {
		 // ros::spin();
		  while(ros::ok())
		  {
			  que.callAvailable();
			  rate.sleep();
		  }
	  }
	  else
			std::cout<<"wtf_________________________Auto Patrol started fail"<<std::endl;
	  ap->disable();
	  delete ap;
		std::cout<<"wtf_________________________"<<std::endl;
	  return 0;

	}
