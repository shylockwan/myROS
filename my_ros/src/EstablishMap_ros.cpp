//==============================================================================
//函数名： EstablishMap_ros类的实现部分
//作者：王华威
//日期:  2015-8-25
//功能: 等待客户端接收消息或者命令，切换状态并执行事件
//修改记录：
//==============================================================================
#include <EstablishMap.hpp>
#include <EstablishMap_ros.hpp>

namespace establishmap{
EstablishMap_ros::EstablishMap_ros(ros::NodeHandle& nh){
	ready_se=new StateReady();
	waitloc_se=new StateWait_SelfLoc();
	waitauto_se=new StateWait_AutoPatr();
	logmap_se=new StateLogMap();
	se_=ready_se;
	e_.cur_state = EstablishMapEvent::ready;
	e_.next_state = EstablishMapEvent::ready;

};
EstablishMap_ros::~EstablishMap_ros(){
	    delete ready_se;
		delete waitloc_se;
		delete waitauto_se;
		delete logmap_se;
		se_=NULL;
		shutdown_requested = true;
		thread.join();
};
bool EstablishMap_ros::onInit(){
	return 1;
};
void EstablishMap_ros::stateUpdate(){

};
}
