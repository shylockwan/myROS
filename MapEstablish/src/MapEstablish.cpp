//==============================================================================
//函数名： EstablishMap类的实现部分
//作者：王华威
//日期:  2015-8-25
//功能: 等待客户端接收消息或者命令，切换状态并执行事件
//修改记录：
//==============================================================================
#include <MapEstablish.hpp>
#include <my_events.hpp>
#include <DP.h>

static float fakeqr[]={0.0,1.57};
static int fake_index=0;
bool decodeQR(float& angle,std::string& id){
	if(fake_index>5)
		fake_index=0;
	angle=fakeqr[fake_index];
	id="1";
	fake_index++;
	return 1;
};
path BFS(string string_vt);
bool colorThresh(cv::Mat& in, cv::Mat& out, cv::Mat& mask, int high_threshold,
		int low_threshold, bool flag_reverse = 0);

cv::Point2f frameFiled2World(cv::Point2f& field_pt, cv::Point2f PtRobot =
		cv::Point_<float>(0.0, 0.0), const float ThetaRobot = 0.0,
		const float gama0 = 1.00, const float height_ = 32.5);

cv::Point2f framePic2Field(const int& width, const int& height, cv::Point& pt,
		const float gama0 = 1.00, const float m_Height = 32.5,
		const float alpha0 = 0.262, const float beta0 = 0.34);
namespace establishmap{
bool StateReady::onInit() {
	ROS_INFO("___________________________now Map is  Ready");
	return 1;
};

void StateReady::stateHandle(EstablishMapEvent& state_,
		RobotStatus& robot_status_) {

	if (state_.selfloc_state == 0){
		state_.next_state = EstablishMapEvent::wait_selfloc;
		state_.report_=0;
	}
	else {
		if (state_.map_state == 0){
			state_.next_state = EstablishMapEvent::qrscan;
			state_.report_=0;
		}
		else {
			std::cout<<"Map has been established... u know it..."<<std::endl;
			state_.next_state = EstablishMapEvent::ready;
			state_.report_ = 1;
		}
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
	state_.next_state = EstablishMapEvent::ready;
	state_.flag_stateswitch = 1;
	state_.report_=0;
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
	if (flag_set == 0) {
		//if flag_set==0;发送循线请求
		AutoPatrol::autopatrol_service srv_;
		srv_.request.angle_from = state_.next_route->angle_from;
		srv_.request.angle_to = state_.next_route->angle_to;
		srv_.request.id_from = state_.next_route->id_from;
		srv_.request.id_to = state_.next_route->id_to;
		srv_.request.flag_uturn = state_.next_route->flag_uturn;
		srv_.request.QR_angle = robot_status_.qr_.angle_;
		srv_.request.cmd=0;
		srv_.request.cmd_from=2;
		srv_.request.flag_qr=1;
		if (client_autoPatr.call(srv_)) {
			std::cout
					<< "______________call for Ap success and waiting________________"
					<< std::endl;
			flag_set = 1;
			ap_timeout=200;
		} else {
			std::cout << "______________call for Ap fail________________"
					<< std::endl;
			state_.next_state = EstablishMapEvent::ready;
			state_.report_ = 2;
			state_.flag_stateswitch = 1;
		}
	}
	else
	{
		//if flag_set==1;等待循线完成 有空设置超时处理
		if( --ap_timeout <=0 )
		{
			state_.report_ = 2;
			state_.flag_stateswitch = 1;
			return ;
		}
		if(ap_result==0)
		{
			state_.next_state=EstablishMapEvent::qrscan;
			state_.flag_stateswitch=1;
			ap_result=-1;
		}
		else{
			state_.next_state=EstablishMapEvent::wait_ap;
			state_.flag_stateswitch=0;
		}
	}


}
;

bool StateWait_AutoPatr::apresultService(libpkg::report::Request& req,libpkg::report::Response& resp )
{
	std::cout<<"get report from ap.........."<<std::endl;
	if(req.srv_result==0)
		ap_result=0;
	else
		ap_result=1;
	resp.result=0;
	return ap_result;
	};

///////////////////////////////////StateQrScan//////////////////////////////////////////////////////

bool StateQrScan::onInit(){
	ROS_INFO("___________________________now Robot is  Scaning QR");
		return 1;
};
void StateQrScan::stateHandle(EstablishMapEvent& state_,RobotStatus& robot_status_){

	if(qr_img==NULL)
	{
		std::cout<<"qr_img==NULL"<<std::endl;
		state_.next_state=EstablishMapEvent::ready;
		state_.flag_stateswitch=1;
		state_.report_=5;
		return;
	}
#if 1 //dog fuck the zxing scan
	float result_angle=0.0;
	std::string result_id="0";
	if (decodeQR(result_angle,result_id)) {
		std::cout<<"scanQR success"<<std::endl;
		robot_status_.qr_.angle_=result_angle;
		robot_status_.qr_.id_=result_id;
		state_.next_state=EstablishMapEvent::logmap;
	} else {
		std::cout<<"scanQR fail"<<std::endl;
		state_.next_state=EstablishMapEvent::ready;
		state_.report_=4;
	}
#endif

	state_.flag_stateswitch=1;

};
void StateQrScan::getImg(cv::Mat* qr_){
	if(qr_==NULL){
		std::cout<<"qr_img对象未初始化"<<std::endl;
		if(qr_img!=NULL)
			delete qr_img;
		qr_img=NULL;
		return;
	}

	if(qr_img!=NULL)
		delete qr_img;

	qr_img=new cv::Mat(*qr_);
}


//////////////////////////////////StateLogMap//////////////////////////////////////////////

bool StateLogMap::onInit() {
	ROS_INFO("___________________________now Robot is  LogMap");
	return 1;
};

void StateLogMap::stateHandle(EstablishMapEvent& state_,
		RobotStatus& robot_status_) {

	std::vector<std::vector<cv::Point> > connect_domain = contoursDetect(*qr_img);
	std::vector<float> a_;//记录角度
	myQR qr=robot_status_.qr_;
	float qr_angle=qr.angle_;
	std::string qr_id="1";
	for (int i = 0; i < connect_domain.size(); i++) {
			float angle_;
			calcRouteAngle(connect_domain[i],robot_status_,angle_,qr_img->cols,qr_img->rows);
			ROS_INFO(" The angle Path %d angle_(before calc)  is %f_  qr_angle is %f_",i,angle_, qr_angle);
			angle_ = angle_-qr_angle;
			angle_=(angle_ > CV_PI?(angle_-(2*CV_PI)):angle_);
			angle_=(angle_ < -CV_PI?(angle_+(2*CV_PI)):angle_);
			/////////////////////////////////////////////////////
			ROS_INFO("  The angle Path %d relative to QR  is %f___________________________",i,angle_);
			a_.push_back(angle_);
	}

	if(!qr_id.empty()){
		   m_->qrcoordiate(qr_id,robot_status_.m_PtRobot.x,robot_status_.m_PtRobot.y,a_.size());
		   m_->addqr(qr_id,robot_status_.l_encoder,robot_status_.r_encoder,last_angle,a_);

		   path route_ = BFS(qr_id);
		   std::cout<<"=======================-------================="<<std::endl;
		state_.next_route->angle_to = route_.ag;
	    state_.next_route->id_from = route_.p;
	    state_.next_route->id_to = route_.q;
	    state_.next_route->flag_uturn= route_.re;
	    last_angle = state_.next_route->angle_to;
	    std::cout<<"==================next route info ================="<<std::endl;
	    std::cout<<" route_.angleto= "<<route_.ag<<", route_.p= "<<route_.p
	    		<<", route_.re= "<<route_.re<<", route_.q= "<<route_.q
	    		<<", route_.length= "<<route_.length<<", route_.lineid= "<<route_.lineid<<std::endl;
	    std::cout<<"==================next route info ================="<<std::endl;
	    if(route_.q==route_.p)
	    {
	    	  std::cout<<"==================Map finish ================="<<std::endl;
	    	  state_.next_state=EstablishMapEvent::ready;
	    	  state_.report_=1;
	    }
	    else
	    {
	    	state_.next_state=EstablishMapEvent::wait_ap;
	    }
	}
	state_.flag_stateswitch=1;
}

void StateLogMap::getImg(cv::Mat* qr_){
	if(qr_==NULL){
		std::cout<<"qr_img对象未初始化"<<std::endl;
		if(qr_img!=NULL)
			delete qr_img;
		qr_img=NULL;
		return;
	}
	if(qr_img!=NULL)
		delete qr_img;

	qr_img=new cv::Mat(*qr_);
}

void StateLogMap::detectContours(cv::Mat& img,std:: vector<std::vector<cv::Point> >& contours,std:: vector<cv::Vec4i>& hierarchy, bool flagRB) {//flag for Bure(0) or Red(1)
	//erode and dilate
	//float Cmin = 200;   //Hellen modified 1009
	float Cmin = 50;   //Hellen modified 1213

	if(flagRB == 1) {//for red
		cv::erode( img, img, cv::Mat(), cv::Point(-1,-1), 3);  //Hellen modified 1216
		cv::dilate(img, img, cv::Mat(), cv::Point(-1,-1), 3);  //Hellen modified 1216
		Cmin = 150;
	    findContours( img, contours, hierarchy,
        	CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
	}else {//for blue
		cv::dilate(img, img, cv::Mat(), cv::Point(-1,-1), 1);
		cv::erode( img, img, cv::Mat(), cv::Point(-1,-1), 3);
		//Cmin = 1000;
		Cmin = 250;    //Hellen modified 1213
        findContours( img, contours, hierarchy,
        	CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
	}
	//imshow("result1", img);


	std::vector<std::vector<cv::Point> >::iterator itc = contours.begin();
	std::vector<cv::Vec4i>::iterator ith=  hierarchy.begin();
	//std::cout<<"detect countour size= "<<contours.size()<<std::endl;

	while(itc != contours.end()) {//eliminate small contour
		float area = contourArea(*itc);
		if(area < Cmin) {
			itc = contours.erase(itc);
			ith = hierarchy.erase(ith);
		}
		else {
			++itc;
			++ith;
		}
	}
}//detectContours

std::vector<std::vector< cv::Point> > StateLogMap::contoursDetect(cv::Mat im)
{

	cv::Mat hsv,out;
	std::vector<cv::Mat> HSV;
	cvtColor(im, hsv, CV_BGR2HSV_FULL);
	split(hsv, HSV);
	colorThresh(HSV[0], out, HSV[1], 15, 240, 1);//red

    //HELLEN add 1126
	cv::dilate(out, out, cv::Mat(), cv::Point(-1,-1), 1);
	cv::erode( out, out, cv::Mat(), cv::Point(-1,-1), 1);
	cv::imwrite("route detect.jpg",out);
	std::vector<std::vector<cv::Point> >  contours;
	std::vector<cv::Vec4i> hierarchy;

	//Hellen modified 1009  add 4th para--- for red
	detectContours(out, contours, hierarchy, 1);

	std::vector<std::vector< cv::Point> > subContourPoints;
	std::vector<cv::Mat> subContours;
	if(contours.size() > 0) {
		int idx = 0;
    	for( ; idx >= 0; idx = hierarchy[idx][0] ) {
			cv::Mat img = cv::Mat::zeros(im.rows, im.cols, CV_8U);
        	drawContours( img, contours, idx, cv::Scalar(255), CV_FILLED, 8, hierarchy );

        	std::vector<cv::Point> contoursPoint;
			for(int nr=0; nr<im.rows; ++nr) {
				uchar* data = img.ptr<uchar>(nr);//row first addr
				for(int nc=0; nc<im.cols; ++nc) {
					if(data[nc] == 255)
						contoursPoint.push_back(cv::Point(nc, nr));
				}
			}
			subContourPoints.push_back(contoursPoint);
    	}
	}//else cout<<"no contours!!!"<<endl;

	return subContourPoints;
}//contoursDetect

void StateLogMap::calcContourAngle(std::vector<cv::Point2f> ptset, float& angle,
		cv::Point2f cen_) {
	if (ptset.empty())
		return;
	cv::Mat paramA = cv::Mat::zeros(2, 1, CV_32FC1);
	LeastsquareFit1(ptset, paramA);
	float temp_max = ptset[0].y;
	float temp_min = temp_max;
	float temp_max_x = ptset[0].x;
	float temp_min_x = temp_max_x;
	for (std::vector<cv::Point2f>::iterator it = ptset.begin();
			it != ptset.end(); it++) {
		float err_ = abs(it->y - cen_.y);
		if (err_ > abs(temp_max - cen_.y)) {
			temp_max = it->y;
		}
		if (err_ < abs(temp_min - cen_.y)) {
			temp_min = it->y;
		}
		float err_2 = abs(it->x - cen_.x);
		if (err_2 > abs(temp_max_x - cen_.x)) {
			temp_max_x = it->x;
		}
		if (err_2 < abs(temp_min_x - cen_.x)) {
			temp_min_x = it->x;
		}
	}

	float a1 = paramA.at<float>(1, 0);
	float deltay = temp_max - temp_min;
	float deltax = (temp_max_x - temp_min_x);
	if (a1 < 3 && a1 > -3) {
		if (a1 == 0) {
			if (deltax > 0)
				angle = 1.57;
			if (deltax < 0)
				angle = -1.57;
			if (deltax == 0)
				angle = 0;
		} else if (a1 > 0) {
			if (deltax >= 0)
				angle = atan(1 / a1);
			else
				angle = atan(1 / a1) - CV_PI;
		} else {
			if (deltax >= 0)
				angle = -atan(a1) + CV_PI / 2;
			else
				angle = -atan(a1) - CV_PI / 2;
		}
	} else {
		if (deltay >= 0)
			angle = atan(1 / a1);
		else
			angle = a1 > 0 ? atan(1 / a1) - CV_PI : atan(1 / a1) + CV_PI;
	}
	return;
}
;

void StateLogMap::LeastsquareFit1(std::vector<cv::Point2f >& ptset,cv::Mat& A)
{
	if (ptset.empty())
		return;
	cv::Mat_<float> F, Y,Z;
	F.create(2, ptset.size());
	Y.create(ptset.size(), 2);
	Z.create(ptset.size(), 2);
	for (int i = 0; i < ptset.size(); i++) {
		///X和Y置换得到x=f(y);
		Z(i, 0) = (float)ptset[i].x;  //Chenchao modified 1109
		Z(i, 1) = (float)ptset[i].y;
	}
	cv::Vec4f line;
   fitLine(Z,line,CV_DIST_L2,0,0.01,0.01);
   A.at<float>(1,0)=line[1]/line[0];
}
void StateLogMap::calcRouteAngle(std::vector<cv::Point> pt_pics,const RobotStatus& rs,float& world_angle,const int& imgwidth_,const int& imgheight_){
	if(pt_pics.empty())
	{
		std::cout<<"可视范围内未搜寻到待选路径"<<std::endl;
		return ;
	}
	std::vector<cv::Point2f> ptset_world;
	cv::Point2f pt_focal_field,pt_focal_world;
	cv::Point pt_focal_pic(imgwidth_/2,imgheight_/2);
	pt_focal_field=framePic2Field(imgwidth_, imgheight_,pt_focal_pic, rs.g_Fuyang,rs.g_Height, rs.g_Chuizhi, rs.g_Shuiping);
	//Hellen add 1128
	cv::Point2f tmp(0.0,0.0);
	pt_focal_world=frameFiled2World(pt_focal_field, tmp, 0.0,rs.g_Fuyang, rs.g_Height);
	for(std::vector<cv::Point>::iterator it=pt_pics.begin(); it!=pt_pics.end(); it++)
	{
		cv::Point2f pt_field,pt_world;
		pt_field=framePic2Field(imgwidth_, imgheight_, *it, rs.g_Fuyang,
						rs.g_Height, rs.g_Chuizhi, rs.g_Shuiping);
		pt_world = frameFiled2World(pt_field, tmp, 0.0, rs.g_Fuyang, rs.g_Height);
		ptset_world.push_back(pt_world);
	}
	calcContourAngle(ptset_world,world_angle,pt_focal_world);
	return;
}


}
;




