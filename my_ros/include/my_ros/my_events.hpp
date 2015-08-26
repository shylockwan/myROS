#pragma once
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#define QR_From int
#define QR_To int
typedef struct PathLink
{
	QR_From id_from;
	QR_To id_to;
	float angle_from;
	float angle_to;
	bool flag_cover;

PathLink(QR_From f=0,float a_f=0,QR_To t=0,float a_t=0.0,bool flag=0):
	id_from(f),id_to(t),flag_cover(0),angle_from(a_f),angle_to(a_t){};

void setPath(QR_From f, float a_f, QR_To t = 0, float a_t = 0.0, bool flag =0) {
		id_from = f;		angle_from = a_f;
		id_to = t;		angle_to = a_t;
		flag_cover = 0;
	}
	}PathLink;

typedef struct AutoPatrolEvent
{
	enum State{
		ready=0,
		autopatrol,
		pathselect
	}cur_state,next_state;
	PathLink* next_route;
	bool flag_eventFinish;
	~AutoPatrolEvent(){
		delete next_route;
	}
}AutoPatrolEvent;
typedef struct EstablishMapEvent
{
	enum State{
		ready=0,
		logmap,
		wait_selfloc,
		wait_ap
	}cur_state,next_state;


	~AutoPatrolEvent(){

	}
}EstablishMapEvent;
typedef struct RobotStatus{
	float
	g_Fuyang,
	g_Chuizhi,
	g_Shuiping,
	g_Height;
	cv::Point2f m_PtRobot;
	float m_ThetaRobot;
}RobotStatus;
