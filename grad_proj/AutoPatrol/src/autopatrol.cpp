#include <autopatrol.hpp>
bool colorThresh(cv::Mat& in, cv::Mat& out, cv::Mat& mask, int high_threshold, int low_threshold, bool flag_reverse=0);

namespace autopatrol{

static int R_SIZE=500;
static int B_SIZE=4000;
//StateExecutor
StateExecutor::StateExecutor() {
	ptset_line.clear();
	ptset_QR.clear();
}
geometry_msgs::Twist StateExecutor::getTwist() {
	return twist_msg;
}
void StateExecutor::getPointset(const cv::Mat& in,
		std::vector<cv::Point>& ptset) {
	if (in.empty() || in.channels() != 1)
	{
		std::cout<<"getPointset图片空"<<std::endl;
		return ;
	}

	ptset.clear();
	cv::Point pt;
	for (int i = 0; i < in.rows; i++) {
		const uchar* dataI = in.ptr<uchar>(i);
		for (int j = 0; j < in.cols; j++) {
			if (dataI[j] == 0)
				continue;
			else {
				pt.y = i;
				pt.x = j;
				ptset.push_back(pt);
			} //if

		} //for2 loop
	} //for1 loop
} ///getPointset();待优化：分类记录点或者去除孤立区域
bool StateExecutor::processImg(const cv::Mat& src) {
	if (src.empty())
	{
		std::cout<<"processImg图片空"<<std::endl;
		return -1;
	}

#if 1////RGB转HSV
	cv::Mat hsv;
	cv::cvtColor(src, hsv, CV_BGR2HSV_FULL);

	std::vector<cv::Mat> hsv_planes;
	split(hsv, hsv_planes);
	cv::Mat h_ = hsv_planes[0];
	cv::Mat s_ = hsv_planes[1];
	cv::Mat h_QR, h_line;
	cv::GaussianBlur(h_, h_, cv::Size(7, 7), 1);
	colorThresh(h_, h_QR, s_, 145, 175); //进行颜色滤波对蓝色，检查结果Mat h_QR
	colorThresh(h_, h_line, s_, 240, 15, 1);
	cv::Mat_<uchar> ker=(cv::Mat_<uchar>(3,3)<<1,1,1,1,1,1,1,1,1);
	cv::morphologyEx(h_QR,h_QR,cv::MORPH_OPEN,ker );
	cv::morphologyEx(h_line,h_line,cv::MORPH_OPEN,ker );
	; //进行颜色滤波对红色，检查结果Mat h_line
	getPointset(h_QR, ptset_QR);
	getPointset(h_line, ptset_line);

#endif
	return 1;
}
////////////////////////////////////////////////////////////////////////////////////////
bool StateReady::onInit() {
	std::cout << "___________________________now Robot is  Ready"
			<< std::endl;
	return true;
}
;
void StateReady::stateHandle(const cv::Mat& src, AutoPatrolEvent& event,RobotStatus& robot) {
	//判断图像中是否存在轨迹线或者二维码
	setTwist();
	//如果命令180度转弯，则跳转至uturn状态，不继续执行后续动作；
	if(event.next_route->flag_uturn==1)
	{
		event.next_state=AutoPatrolEvent::uturn;
		event.flag_stateswitch = 1;
		return;
	}

	processImg(src);
	//1.如果图像中不包含路径像素，或二维码像素，则保持ready态，report=1 显示完成；
	//2.如果图像中只包含路径像素，跳转至ap态
	//3.如果图像中包含二维码信息，
			//event.flag_qrloc==0跳转至qrloc态
			////event.flag_qrloc==1跳转至qrcover态
	if (ptset_QR.size() < B_SIZE && ptset_line.size() < R_SIZE) {
		event.next_state = AutoPatrolEvent::ready;
		event.report_=2;
		setTwist();

	} else if (R_SIZE < ptset_line.size() && ptset_QR.size() < B_SIZE) { //发现轨迹线色域，未发现QR色域进入ap态
		event.next_state = AutoPatrolEvent::autopatrol;
	} else //发现QR色域，进入loc态
	{
		if(event.flag_qrloc==0)
			event.next_state = AutoPatrolEvent::qrloc;
		else
			event.next_state = AutoPatrolEvent::qrcover;
	}
	event.flag_stateswitch = 1;
}
;
////////////////////////////////////////////////////////////////////////////////////////
bool StateAP::onInit() {
	std::cout << "___________________________now Robot is  autopatrolling"
			<< std::endl;
	return true;
}
;
void StateAP::stateHandle(const cv::Mat& src, AutoPatrolEvent& event,RobotStatus& robot) {
	if (src.empty()){
		std::cout<<"StateAutoPatrol stateHandle picture is empty"<<std::endl;
		return ;
	}
	event.flag_qrloc=0;
	//如果命令180度转弯，则跳转至uturn状态，不继续执行后续动作；
	if(event.next_route->flag_uturn==1)
	{
		event.next_state=AutoPatrolEvent::uturn;
		event.flag_stateswitch = 1;
		return;
	}


	processImg(src);
	//1.如果图像中不包含路径像素，或二维码像素，跳转ready态，report=3 ap_error；
	//2.如果图像中只包含路径像素，维持ap态
	//3.如果图像中包含二维码信息，跳转至qrloc态
	if (ptset_QR.size() < B_SIZE && ptset_line.size() < R_SIZE) {
		if (ptset_pre.size() > 5000) {//进入ap转弯等待
			setTwist(calcObjTwist(ptset_line, src.cols, src.rows, robot));
		} else {
			event.next_state = AutoPatrolEvent::ready;
			event.report_ = 3;
			event.flag_stateswitch = 1;
			setTwist();
		}

	} else if (R_SIZE < ptset_line.size() && ptset_QR.size() < B_SIZE) { //发现轨迹线色域，未发现QR色域进入ap态
		event.next_state = AutoPatrolEvent::autopatrol;
		setTwist( calcObjTwist(ptset_line, src.cols, src.rows, robot) );
		event.flag_stateswitch=0;
	} else //发现QR色域，进入loc态
	{
		event.next_state = AutoPatrolEvent::qrloc;
		pid_.resetError();
		event.flag_stateswitch=1;
	}
	return;
}
;
geometry_msgs::Twist StateAP::calcObjTwist(std::vector<cv::Point> ptset,
		const int& width, const int& height, RobotStatus& rs) {
	//判断输入的目标像素点个数，将其同上一帧做比较
		 //如果像素点锐减，有可能为直角转弯时发生,进入转弯倒计数，返回上一帧得到的转速；
		//否则更新ptset_world_pre;

	if (curve_counts==0 || float(ptset.size()) *1.7 >= float(ptset_pre.size())) {//如果转弯计数减至0或当前帧路径点足够多，
		//则清空前一帧路径点，否则循环递减转弯计数。
		ptset_pre.clear();
		ptset_pre = ptset; //保存当前帧获得的世界坐标
		curve_counts=15;
	}
	else{
		curve_counts--;
		return t_pre;
	}

	geometry_msgs::Twist t;
	t.angular.x = 0;
	t.angular.y = 0;
	t.angular.z = 0;
	t.linear.x = 0;
	t.linear.y = 0;
	t.linear.z = 0;
	float K1=1.0;float K2=0.005;
	float obj_t=0.0;

	//根据传入的ptset，获取头行点和尾行点的中点
	getObjPoint(ptset);
	obj_t+=getDiffangle( cv::Point(width/2,height/2) )*K1;
	obj_t+=getDiffdist( cv::Point(width/2,height/2) )*K2;
	//将获得的合计偏差值，加入PD闭环控制其中，计算输出角速度
	obj_t=myPID(obj_t);
	float max_min = 0.7;
	if (obj_t > max_min)
		obj_t = max_min;
	if (obj_t < -max_min)
		obj_t = -max_min;
	t.angular.z = -obj_t;
	t.linear.x = 0.1;
	t_pre=t;
	return t;
}
;

float StateAP::myPID(float cur_,float obj_)
{
	float result=0.0;
	pid_.set_point=obj_;
	float derror_,error_;
	error_=pid_.set_point-cur_;//p_
	pid_.sum_error+=error_;//i_
	derror_=error_-pid_.last_error;
	pid_.pre_error=pid_.last_error;
	pid_.last_error=error_;

	result=pid_.p_*error_+
			pid_.d_*derror_;

	return result;
}
float StateAP::getDiffangle(cv::Point cen_){
	float result=0.0;
	float delta_x=mid_point.x-end_point.x;
	float delta_y=mid_point.y-end_point.y;
	delta_y= abs(delta_y);

	if (delta_y <= 1) {
		if (delta_x > 1)
			result = 1.57;
		if (delta_x < -1)
			result = -1.57;
		if (abs(delta_x) <= 1)
			result = 0;
	} else
		result = atan((delta_x) / (delta_y));

	return result;
};

float StateAP::getDiffdist(cv::Point cen_){
	float result=0.0;
    result=end_point.x-cen_.x;
	return result;
};

void StateAP::getObjPoint(std::vector<cv::Point>& ptset){
	if(ptset.empty())
		return;
	std::vector<cv::Point>::iterator iter;
	//get end_point,mid_point,start_point;
	end_point=ptset[ptset.size()-1];
	start_point=ptset[0];
	float accum_=0.0;
	int count_=0;
	for( iter=ptset.end()-1;iter!=ptset.begin();iter--)
	{
		if( abs(iter->y - end_point.y )>2)
			break;
		accum_+=iter->x;
		count_++;
	}
	end_point.x=accum_/count_;

//	accum_=0.0;
//	count_=0;
//	for( iter=ptset.begin();iter!=ptset.end();iter++)
//	{
//		if( abs(iter->x - start_point.x )>3)
//			break;
//		accum_+=iter->x;
//		count_++;
//	}
//	start_point.x=accum_/count_;

	accum_=0.0;
	count_=0;
	mid_point.y=(start_point.y+end_point.y)/2;
	for( iter=ptset.begin()+1;iter!=ptset.end()-1;iter++)
	{
		if( abs(iter->y - mid_point.y )>2)
			continue;
		accum_+=iter->x;
		count_++;
	}
	mid_point.x=accum_/count_;

};
////////////////////////////////////////////////////////////////////////////////////////
 bool StateTurnAngle::onInit() {
	std::cout << "___________________________now Robot is  Truning Angle"
			<< std::endl;
	return true;
}
;
void StateTurnAngle::stateHandle(const cv::Mat& src, AutoPatrolEvent& event,RobotStatus& robot) {
    //To be continue..
	//检测命令是否初次跳转至该状态。如果flag_set==0，初始化起始朝向，目标朝向，夹角

	//如果flag_set=1,计算剩余夹角
		//如果剩余夹角<=0，则停止，flag_set=0 flag_stateswitch=1 跳转至ap态;
		//如果剩余夹角>0,则维持原转速，维持angleturn态
	//已知命令传入的转角为路径线同二维码的夹角，因此根据二维碼角度计算当前机器人所在方位，方法如下
		//if qr_angle>=0,机器人方位角为 PI-qr_angle;
		//if qr_angle<0,机器人方位角为  -PI-qr_angle;
	//之后计算命令转角和二维码夹角，公式如下：
		//theta_1=机器人方位角-路径角；
		//theta_1=(theta_1 > CV_PI?(theta_1-(2*CV_PI)):theta_1);
		//theta_1=(theta_1 < -CV_PI?(theta_1+(2*CV_PI)):theta_1);
	if(flag_set==0)
	{
		qr_angle=robot.qr_angle;
		float route_angle=event.next_route->angle_to;
		route_angle=(route_angle>=0? (CV_PI-route_angle) : (-CV_PI-route_angle));
		obj_angle=route_angle-qr_angle;
		obj_angle=(obj_angle > CV_PI?(obj_angle-(2*CV_PI)):obj_angle);
		obj_angle=(obj_angle < -CV_PI?(obj_angle+(2*CV_PI)):obj_angle);
		delta_angle=0;
		flag_set=1;
	}


	if( abs(obj_angle)- abs(delta_angle)>0.1)
	{
        cur_theta = fabs(robot.m_ThetaRobot);
        delta_angle += fabs(cur_theta-last_theta);
        last_theta = cur_theta;
        if(obj_angle>0)
        	setTwist(0,0,0,0,0,-0.6);
        else
        	setTwist(0,0,0,0,0,-0.6);
	}
	else
	{
		ROS_INFO("_____________HHHHHHHHHHHHHHHHHHHHHHHHH stop  uturning ______________delta_angle is %f",delta_angle);
		setTwist();
		flag_set=0;
		robot.m_ThetaRobot = 0.0;
		delta_angle=0;
		event.next_route->flag_uturn=0;
		event.next_state=AutoPatrolEvent::autopatrol;
	}
	return;
};

void StateTurnAngle::setObjAngle(float obj_)
{
	obj_angle=obj_;
};
float StateTurnAngle::getObjAngle(){
	return delta_angle;
};


////////////////////////////////////////////////////////////////////////////////////////
bool StateUturn::onInit() {
	std::cout << "___________________________now Robot is  Uturning"
			<< std::endl;
	return true;
}
;
void StateUturn::stateHandle(const cv::Mat& src,AutoPatrolEvent& event,RobotStatus& robot) {
    //To be continue..
	//检测命令是否初次跳转至该状态。如果flag_set==0，初始化起始朝向，目标朝向，夹角

	//如果flag_set=1,计算剩余夹角
		//如果剩余夹角<=0，则停止，flag_set=0 flag_stateswitch=1 跳转至ap态;
		//如果剩余夹角>0,则维持原转速，维持uturn态
	if(flag_set==0)
	{
		setObjAngle();
		ROS_INFO("_____________Start uturning!!!!!!!!!!!!!!!!!!!______________objangle is %f",obj_angle);
		cur_theta=last_theta=fabs(robot.m_ThetaRobot);
		delta_angle=0;
		flag_set=1;
	}

	if(obj_angle-delta_angle>0.1)
	{
	    float tmprobot_angle = robot.m_ThetaRobot;
        cur_theta = fabs(robot.m_ThetaRobot);
        delta_angle += fabs(cur_theta-last_theta);
        last_theta = cur_theta;
		setTwist(0,0,0,0,0,0.6);
	}
	else
	{
		ROS_INFO("_____________HHHHHHHHHHHHHHHHHHHHHHHHH stop  uturning ______________delta_angle is %f",delta_angle);
		setTwist();
		flag_set=0;
		robot.m_ThetaRobot = 0.0;
		delta_angle=0;
		event.next_route->flag_uturn=0;
		event.next_state=AutoPatrolEvent::autopatrol;
	}
	return;

};

void StateUturn::setObjAngle(float obj_)
{
	obj_angle=obj_;
};
float StateUturn::getObjAngle(){
	return obj_angle;
};
float StateUturn::haveTurnAngle(){
	return delta_angle;

};

////////////////////////////////////////////////////////////////////////////////////////

bool StateQrcover::onInit() {
	std::cout << "___________________________now Robot is  Qrcovering"
			<< std::endl;
	return true;
};

void StateQrcover::stateHandle(const cv::Mat& src,AutoPatrolEvent& event,RobotStatus& robot)
{
	//检测命令是否初次跳转至该状态。如果flag_set==0，
			//如果flag_loc==0，report error
			//否则初始化需要前进的距离。根据镜头俯仰角，摄像头高度计算flag_set=1。
	//如果flag_set=1,计算剩余需前进的距离
			//如果剩余距离<=0，则停止，flag_set=0 flag_stateswitch=1 跳转至angleturn态;
			//如果剩余距离>0,则维持原转速，维持qrcover态
	if(flag_set==0)
	{
		if(event.flag_qrloc==0)
		{
			event.next_state=AutoPatrolEvent::ready;
			event.report_=3;
			event.flag_stateswitch=1;
		}
		else
		{
			obj_dist=robot.g_Height*tan(robot.g_Fuyang);
			cur_coor=last_coor=robot.m_PtRobot;
			delta_dist=0;
			flag_set=1;
		}
	}
	else{
		if(obj_dist-delta_dist>0.1)
		{
		    float tmprobot_angle = robot.m_ThetaRobot;
	        cur_coor = robot.m_PtRobot;
	        delta_dist += ( abs(cur_coor.x-last_coor.x)+abs(cur_coor.y-last_coor.y) );
	        last_coor = cur_coor;
			setTwist(0.2,0,0,0,0,0);
			event.next_state=AutoPatrolEvent::qrcover;
		}
		else
		{
			std::cout<<"_____________HHHHHHHHHHHHHHHHHHHHHHHHH stop  uturning ______________delta_angle is "<<delta_dist<<std::endl;
			setTwist();
			flag_set=0;
			delta_dist=0;
			event.flag_stateswitch=1;
			event.next_state=AutoPatrolEvent::angleturn;
		}

	}



};

bool StateQrloc::onInit() {
	std::cout << "___________________________now Robot is  Qrlocing"
			<< std::endl;
	return true;
};

void StateQrloc::stateHandle(const cv::Mat& src,AutoPatrolEvent& event,RobotStatus& robot)
{
	//如果event.flag_qrloc==1;直接report 1；
	//否则调用decodeQR节点服务并等待结果
			//检测失败，循环检测
			//检测成功，更新机器人状态，flag_qrloc=1
	setTwist();
	if(event.flag_qrloc==1){
		event.report_=1;
		event.next_state=AutoPatrolEvent::ready;
		event.flag_stateswitch=1;
		return;
	}

//	if(client_qrdecode.call())
//	{
//		//wait for decode
//		robot.qr_angle=0.0;
//		event.flag_qrloc=1;
//		event.flag_stateswitch=1;
//		event.next_state=AutoPatrolEvent::ready;
//		event.report_=1;
//
//	}

	};
}
;
