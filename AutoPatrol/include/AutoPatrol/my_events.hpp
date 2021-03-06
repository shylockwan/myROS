#pragma once
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#define QR_From int
#define QR_To int
typedef struct myQR{
	float angle_;
	std::string id_;
	myQR():angle_(0.0),id_("0"){}
}myQR;

typedef struct PathLink
{
	QR_From id_from;
	QR_To id_to;
	float angle_from;
	float angle_to;
	bool flag_uturn;

PathLink(QR_From f=0,float a_f=0,QR_To t=0,float a_t=0.0,bool flag=0, float a_q = 0.0):
	id_from(f),id_to(t),flag_uturn(flag),angle_from(a_f),angle_to(a_t){};

void setPath(QR_From f, float a_f, QR_To t = 0, float a_t = 0.0, bool flag =0) {
		id_from = f;		angle_from = a_f;
		id_to = t;		angle_to = a_t;
		flag_uturn = flag;
	}
	}PathLink;

//report case
//case	1
//ROS_INFO("__________CUR_SERVICE FINISHED__________");
//	case 2:
//		ROS_INFO("__________NO_OBJ_POINT__________");
//	case 3:
//		ROS_INFO("__________AP_ERROR__________");
//	case 4:
//		ROS_INFO("________ERROR NO INPUT IMG OR STATE die__________");
//	case 5:
//		ROS_INFO("________TurnAngle_ERROR__________");
//	case 6:
//		ROS_INFO("________Uturn_ERROR__________");
//	case 7:
//		ROS_INFO("________ERROR please loc QR first__________");

typedef struct AutoPatrolEvent
{

	enum State{
		ready=0,
		autopatrol,
		qrcover,
		angleturn,
		uturn
	}cur_state,next_state;
	PathLink* next_route;
	bool flag_stateswitch;
	bool flag_qrloc;
	int report_;//report=0  继续执行 //report=1 完成 // 2  NO_OBJ_POINT //3 aperror // 4 no input img or state_die..default
	AutoPatrolEvent():flag_qrloc(0),flag_stateswitch(1){
			next_route=new PathLink();
		}
	~AutoPatrolEvent(){
		delete next_route;
	}
}AutoPatrolEvent;

typedef struct RobotStatus{
	float
	g_Fuyang,
	g_Chuizhi,
	g_Shuiping,
	g_Height;
	cv::Point2f m_PtRobot;
	float m_ThetaRobot;
	int l_encoder;
	int r_encoder;
	myQR qr_;
	RobotStatus():g_Fuyang(0.0),g_Chuizhi(0.0),g_Shuiping(0.0),g_Height(0.0),
			m_ThetaRobot(0.0),l_encoder(0),r_encoder(0),m_PtRobot(0.0,0.0){
	};
}RobotStatus;

typedef struct PID{
	float set_point;
	float p_;
	float d_;
	float last_error; //error[-1]
	float pre_error;//error[-2]
	float sum_error;
	PID():set_point(0.0), p_(1.6),d_(1.0),last_error(0.0),pre_error(0.0),sum_error(0.0){};
	void resetError(){
		last_error=0.0;
		pre_error=0.0;
		sum_error=0.0;
	}
}PID;
