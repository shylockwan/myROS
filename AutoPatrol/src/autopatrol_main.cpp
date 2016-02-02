#include <autopatrol_ros.hpp>
#include  <ros/ros.h>
#include <AutoPatrol/autopatrol_service.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>

 static autopatrol::AutoPatrol_ros* ap_ros;
static int num_client=0;
 bool autopatrolService(AutoPatrol::autopatrol_service::Request& req,AutoPatrol::autopatrol_service::Response& rep)
 {
	std::cout<<"南玻."<<num_client++<<" clients come"<<std::endl;
	PathLink r;
	r.setPath(req.id_from,req.angle_from,req.id_to,req.angle_to,req.flag_uturn);
	ap_ros->getRequestID(req.cmd_from);
	if(req.flag_qr==1)
		ap_ros->setQrAngle(req.QR_angle);
	switch(req.cmd)//rep.result=0：线条检测启动//1 已启动，稍候重试//2：中止目前行动//default：命令错误
	{
	case 0://命令启动
		if (ap_ros->getState() == 1) {
			std::cout << "模块运行中，请稍候重试" << std::endl;
			rep.ap_result = 1;
		} else {
			ap_ros->getPathInfo(r);
			ap_ros->enable();
			std::cout << "模块启动,线条检测......" << std::endl;
			rep.ap_result = 0;
		}
		break;
	case 2:
		ap_ros->disable();
		std::cout<<"循线模块中止"<<std::endl;
		rep.ap_result=2;
		break;
	default:
		std::cout<<"命令出错，请发送命令0线条检测启动；命令1路径选择与转弯启动；命令2停止"<<std::endl;
		break;
	}
	return true;
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"ServiceNode_AutoTrack");
	 cv::namedWindow("path", CV_WINDOW_KEEPRATIO | CV_WINDOW_NORMAL);
	ros::NodeHandle nh_srv;
	ros::NodeHandle nh_report;
	ros::NodeHandle nh_ap;
	ros::ServiceServer ld_server=nh_srv.advertiseService("/autopatrol_service",&autopatrolService);
	ap_ros=new autopatrol::AutoPatrol_ros(nh_ap);
	std::cout<<"ServiceNode_AutoTrack STARTing "<<std::endl;
	ros::Rate updaterate(10);
	if (ap_ros->onInit()) {
		while (ros::ok()) {

			ros::spinOnce();
			ap_ros->stateUpdate();
			ap_ros->Report();
			updaterate.sleep();
		}
	}
	delete ap_ros;
	return 0;


	}
