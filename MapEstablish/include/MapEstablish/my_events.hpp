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

typedef struct EstablishMapEvent
{
	enum State{
		ready=0,
		logmap,
		wait_selfloc,
		wait_ap,
		qrscan
	}cur_state,next_state;
    int map_state;//0->ing or not // 1->map established
	int report_;//0->no report//1->finish //2->ap_error//4-> no img or state init fail
    bool selfloc_state;
	bool flag_stateswitch;
	PathLink* next_route;
	EstablishMapEvent(){
		next_route=new PathLink();
	//	map_state=0;
		//selfloc_state=0;
		report_=0;
	}
	~EstablishMapEvent(){
		delete next_route;
	}
}EstablishMapEvent;


