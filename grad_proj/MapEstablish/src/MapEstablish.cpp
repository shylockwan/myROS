//==============================================================================
//函数名： EstablishMap类的实现部分
//作者：王华威
//日期:  2015-8-25
//功能: 等待客户端接收消息或者命令，切换状态并执行事件
//修改记录：
//==============================================================================
#include <MapEstablish.hpp>
#include <my_events.hpp>



namespace establishmap{
bool StateReady::onInit() {
	ROS_INFO("___________________________now Map is  Ready");
	return 1;
};

void StateReady::stateHandle(EstablishMapEvent& state_,
		RobotStatus& robot_status_) {

	if (state_.selfloc_state == 0){
		state_.next_state = EstablishMapEvent::wait_selfloc;
	}
	else {
		if (state_.map_state == 0){
			state_.next_state = EstablishMapEvent::logmap;
			state_.report_=0;
		}
		else
		state_.next_state = EstablishMapEvent::ready;
		state_.report_=1;
	}
	state_.flag_stateswitch = 1;
}
;

///////////////////////////////StateWait_SelfLoc////////////////////////////////////////////////////

bool StateWait_SelfLoc::onInit() {
	ROS_INFO("___________________________now Robot is  Wait_SelfLoc");
	return 1;
}
;
void StateWait_SelfLoc::stateHandle(EstablishMapEvent& state_,
		RobotStatus& robot_status_) {
	state_.selfloc_state=1;
	state_.next_state = EstablishMapEvent::logmap;
	state_.flag_stateswitch = 1;
	return;
	}

//////////////////////StateWait_AutoPatr/////////////////////////////////////////////////////////////

bool StateWait_AutoPatr::onInit() {
	ROS_INFO("___________________________now Robot is  Wait_AutoPatr");
	return 1;
}
;
void StateWait_AutoPatr::stateHandle(EstablishMapEvent& state_,
		RobotStatus& robot_status_) {
	if (state_.report_ != 0)
		return;
	AutoPatrol::autopatrol_service srv_;
	srv_.request.angle_from = state_.next_route->angle_from;
	srv_.request.angle_to = state_.next_route->angle_to;
	srv_.request.id_from = state_.next_route->id_from;
	srv_.request.id_to = state_.next_route->id_to;
	srv_.request.flag_uturn=state_.next_route->flag_uturn;
	srv_.request.QR_angle = state_.next_route->angle_qr;  //Hellen add 1113



}
;

///////////////////////////////////StateQrScan//////////////////////////////////////////////////////

bool StateQrScan::onInit(){
	ROS_INFO("___________________________now Robot is  Scaning QR");
		return 1;
};
void StateQrScan::stateHandle(EstablishMapEvent& state_,RobotStatus& robot_status_){

	decodeQR();

};
void StateQrScan::getImg(cv::Mat* qr_){
	if(qr_==NULL){
		std::cout<<"qr_img对象未初始化"<<std::endl;
		return;
	}
	qr_img=qr_;
}
}
;




