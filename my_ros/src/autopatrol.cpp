//==============================================================================
//函数名： StateExecutor类的实现部分以及相关继承类的实现
//作者：王华威
//日期:  2015-7-10
//功能: 用于状态转换时执行相应动作
//修改记录：
//==============================================================================
#include <ros/ros.h>
#include "my_events.hpp"
#include <geometry_msgs/Twist.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <autopatrol.h>
float scanQR(){ return 0.0; };
bool colorThresh(cv::Mat& in, cv::Mat& out, cv::Mat& mask, int high_threshold,
		int low_threshold, bool flag_reverse = 0);
int  aroundCenter(std::vector<cv::Point> ptset,const cv::Point2f& center_,float tolerance_=30);

std::vector<std::vector< cv::Point> > contoursDetect(cv::Mat im);

void calcContourAngle(std::vector<cv::Point> ptset,float& angle,cv::Point2f cen_);

cv::Point2f frameFiled2World(cv::Point2f& field_pt,cv::Point2f PtRobot=cv::Point_<float>(0.0,0.0),
			const float ThetaRobot=0.0 ,const float gama0=1.00,const float height_=32.5) ;

cv::Point2f framePic2Field(const int& width, const int& height,
			cv::Point& pt,const float gama0 = 1.00,const float m_Height =32.5,const float alpha0 = 0.262,
			const float beta0 = 0.34);

float myfunc(float x, const cv::Mat& param) {
	if (param.empty()) {
		std::cout << "paramA has not been initialed" << std::endl;
		return -1;
	}

	//float a0,a1,a2,a3;
	//a0=paramA.at<double>(0,0);
	//a1=paramA.at<double>(1,0);
	//a2=paramA.at<double>(2,0);
	//a3=paramA.at<double>(3,0);
	float y = param.at<double>(0, 0) + param.at<double>(1, 0) * x
			+ param.at<double>(2, 0) * x * x+param.at<double>(3,0)*x*x*x;
	return y;
}
;

float myfunc2(float x, float y,cv::Point2f pt_robot,const float radius=10) {

	float res = (x-pt_robot.x) *( x-pt_robot.x)
			+ (y - pt_robot.y) * (y - pt_robot.y)
			- radius * radius;		//cm
	return res;
}
;
void LeastsquareFit3(std::vector<cv::Point2f> &ptset, cv::Mat& paramA) {
	if (ptset.empty())
		return;
	cv::Mat_<double> F, Y;
	F.create(4, ptset.size());
	Y.create(ptset.size(), 1);
	for (int i = 0; i != ptset.size(); i++) {
		///X和Y置换得到x=f(y);
		double pt_x = ptset[i].y;
		F(0, i) = 1;
		F(1, i) = pt_x;
		F(2, i) = pt_x * pt_x;
		F(3,i)=pt_x*pt_x*pt_x;
		Y(i, 0) = ptset[i].x;

	}			//for1 loop
	cv::Mat F_Ft;
	cv::mulTransposed(F, F_Ft, false);
	cv::Mat temp = (F_Ft.inv());
	//std::cout<<temp.rows<<"  "<<temp.cols<<std::endl<<F.rows<<" "<<F.cols;
	paramA = temp * F * Y;		//a0 a1 a2 a3;opencv中貌似已经包含cv::solve函数用于求解

}			//LeastsquareFit;
namespace autopatrol {

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
		return;

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
		return -1;
#if 1////RGB转HSV
	cv::Mat hsv;
	cv::cvtColor(src, hsv, CV_BGR2HSV_FULL);
	std::vector<cv::Mat> hsv_planes;
	split(hsv, hsv_planes);
	cv::Mat h_ = hsv_planes[0];
	cv::Mat s_ = hsv_planes[1];
	cv::Mat h_QR, h_line;
	cv::GaussianBlur(h_, h_, cv::Size(7, 7), 1);
	colorThresh(h_, h_QR, s_, 165, 185); //进行颜色滤波对蓝色，检查结果Mat h_QR
	colorThresh(h_, h_line, s_, 240, 15, 1);
	; //进行颜色滤波对红色，检查结果Mat h_line
	getPointset(h_QR, ptset_QR);
	getPointset(h_line, ptset_line);
#endif
	return 1;
}

bool StateReady::onInit() {
	ROS_INFO("now Robot is Ready for");
	//setTwist();
	return true;
}
;
void StateReady::stateHandle(const cv::Mat& src, AutoPatrolEvent& state_,RobotStatus& robot_status_) {
//判断图像中是否存在轨迹线或者二维码
	setTwist();
	state_.flag_eventFinish=0;
	if (src.empty()) {
		return;
	}
	processImg(src);
	ROS_INFO("ready: bsize=%d , rsize=%d", ptset_QR.size(), ptset_line.size());
	///图像中二维码与轨迹线都没有发现，小于500时，保持ready状态，置位事件完成标志
	if (ptset_QR.size() < 500 && ptset_line.size() < 500) {
		state_.next_state = AutoPatrolEvent::ready;
		state_.flag_eventFinish = 1;
		setTwist();

	}
	else if(500 < ptset_line.size()&& ptset_QR.size() < 500) { //发现轨迹线色域，未发现QR色域进入ap态
		state_.next_state = AutoPatrolEvent::autopatrol;
	}
	else  //发现QR色域，进入ps态
	{
		state_.next_state = AutoPatrolEvent::pathselect;
	}

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool StateAutoPatrol::onInit() {
	ROS_INFO("now Robot is  autopatrolling");
	//setTwist();
	return true;
}
;
void StateAutoPatrol::stateHandle(const cv::Mat& src, AutoPatrolEvent& state_,RobotStatus& robot_status_) {
//循线移动
	setTwist();
	state_.flag_eventFinish=0;
	if (src.empty()) {
		return;
	}
	processImg(src);
	ROS_INFO("ready: bsize=%d , rsize=%d", ptset_QR.size(), ptset_line.size());
	//a 如果未检测到QR或是Line，转入ready态
	//b 如果检测到QR且当前路径未知（终点QR未检测）则转入ps态
	//c 如果检测到QR且当前QR已扫描
		//c1 state_中有明确next_route	根据state_中next_path,确定连通域角度并提取，获取计算Twist
	    //     连通域内容保持不变直至QR淡出视野
		//c2  state_中无明确next_route 置位flag_eventFinish
	//d 只检测到Line，未检测到QR，则循线走
	if (ptset_QR.size() < 500 && ptset_line.size() < 500) {
		state_.next_state = AutoPatrolEvent::ready;
		this->flag_selfLocate=0;
		return;
	}//if 1

	if(ptset_QR.size() > 500)
	{
		if(0==flag_selfLocate)//还未扫描QR
		{
			state_.next_state = AutoPatrolEvent::pathselect;
		}//if 2.1
		else//QR已经扫描
		{
			if(state_.next_route->id_to==0){   //未设置目标路径，走毛线阿？！
				state_.flag_eventFinish=1;
				setTwist();
			}//if 2.1.1
			else
			{
				std::vector<std::vector<cv::Point> > connect_domain;
				connect_domain.clear();
			    connect_domain=contoursDetect(src);
			    for(std::vector<std::vector<cv::Point> >::iterator it=connect_domain.begin();
			        		   it!=connect_domain.end();it++)
			    {
			    	float angle_c;
			    	calcContourAngle( (*it),angle_c,cv::Point2f(src.cols/2,src.rows/2) );
			    	///angle_c与二维码角度之间还有一个变换关系
			    	angle_c=angle_c-angle_QR;
			    	/////
			    	if(angle_c+0.2>state_.next_route->angle_from||angle_c-0.2<state_.next_route->angle_from)
			    	{
			    		ptset_pic.insert(ptset_pic.begin(),it->begin(),it->end());
			    		break;
			    	}
			    }
			   setTwist( calcObjTwist(ptset_pic,src.cols,src.rows,robot_status_));
			}//else 2.1.1
		}//else 2.1
		return;
	}//if 2

	if(ptset_QR.size() <500 && ptset_line.size() > 500)
	{
		ptset_pic=ptset_line;
		// setTwist( calcObjTwist(ptset_pic,src.cols,src.rows,robot_status_));
		 this->flag_selfLocate=0;
		 return;
	}//if 3
}


geometry_msgs::Twist StateAutoPatrol::calcObjTwist(std::vector<cv::Point> ptset,const int& width,const int& height,RobotStatus& rs)
{
	std::vector<cv::Point2f> ptset_temp;
	ptset_world.clear();
	if (!ptset_pic.empty()) {
				cv::Point2f pt_filed,pt_world;
				for (std::vector<cv::Point>::iterator it = ptset_pic.begin();
						it != ptset_pic.end(); it++) {
					pt_filed = framePic2Field(width,height,*it,rs.g_Fuyang,rs.g_Height,rs.g_Chuizhi,rs.g_Shuiping);

					pt_world=frameFiled2World(pt_filed,rs.m_PtRobot,rs.m_ThetaRobot,rs.g_Fuyang,rs.g_Height);

					ptset_world.push_back(pt_world);
					ptset_temp.push_back(pt_world);
				}

			}
			///合并上一帧目标直线的世界坐标
			if(!ptset_world_pre.empty())
			{
				for(std::vector<cv::Point2f>::iterator it=ptset_world_pre.begin();it!=ptset_world_pre.end();it++)
				{
					ptset_world.push_back(*it);
				}
			}
			if(ptset_temp.size()*2>=ptset_world_pre.size())
			{
				ptset_world_pre.clear();
				ptset_world_pre=ptset_temp;//保存当前帧获得的世界坐标
			}

//////////////////////////////////////////////////////
			cv::Mat paramA_=paramA_=cv::Mat::zeros(4,1,CV_32FC1);
			LeastsquareFit3(ptset_world, paramA_);

	geometry_msgs::Twist t;
    t.angular.x=0;
    t.angular.y=0;
	t.angular.z=0;
	t.linear.x=0;
	t.linear.y=0;
	t.linear.z=0;
	/////getLocation()内容
	if (!paramA_.empty() && ! ptset_world.empty())
	{
		float theta_robot=rs.m_ThetaRobot;
		cv::Point2f pt_robot=rs.m_PtRobot;
		cv::Point2f pt_obj;
		float theta_1;
		float end_y = this->ptset_world[0].y;
				float start_y = end_y;
				for (std::vector<cv::Point2f>::iterator it = ptset_world.begin();
						it != ptset_world.end(); it++) {
					if (start_y > (*it).y) {
						start_y = (*it).y;
					}
					if (end_y < (*it).y) {
						end_y = (*it).y;
					}
				}
				float delta_y = (end_y - start_y) / 600;
				cv::Point2f pt_(myfunc(start_y, paramA_), start_y);
				float err_ = abs( myfunc2(myfunc(start_y, paramA_), start_y,rs.m_PtRobot) );

				for (float i = start_y; i < end_y; i += delta_y) {
					float x_ = myfunc(i, paramA_);
					float err_temp = abs(myfunc2(x_, i,rs.m_PtRobot));
					//ROS_INFO("( %f,%f),;  err %f ",x_,i,err_temp);
					if (err_ > err_temp) {
						pt_.x = x_;
						pt_.y = i;
						err_ = err_temp;
					}
				}			    		//遍历误差err，最小者为交点，输出至pt_；
				pt_obj.x = pt_.x ;	pt_obj.y = pt_.y;

				float deltax=pt_obj.x-pt_robot.x;
				float deltay=pt_obj.y-pt_robot.y;
				if(deltay==0)
				{
					if(deltax>0)
						theta_1=1.57;
					if(deltax<0)
						theta_1=-1.57;
					if(deltax==0)
						theta_1=0;
				}
				else if(deltay<0)
				{
					float theta_=atan( (deltax)/(deltay));
					theta_=(theta_>0?theta_-CV_PI:theta_+CV_PI);
					theta_1 = theta_+theta_robot;
				}
				else
					theta_1= atan( (deltax)/(deltay) )+theta_robot;

				t.angular.z =theta_1;
				t.linear.x=0.1;
	}
return t;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool StatePathSelect::onInit() {
	//setTwist();
	ROS_INFO("now Robot is  pathSelecting");

	return true;
}
;
void StatePathSelect::stateHandle(const cv::Mat& src, AutoPatrolEvent& state_,RobotStatus& robot_status_) {
//提取路径问题
	setTwist();
	state_.flag_eventFinish=0;
	if (src.empty()) {
		return;
	}
	processImg(src);
	ROS_INFO("ready: bsize=%d , rsize=%d", ptset_QR.size(), ptset_line.size());
	//a 如果QR处于图像正中位置，则检测QR，置位state_.flag_eventFinish=1;
	//b 如果QR不处于正中位置，则沿线调整，直至完成跳转至a
	//c 如果未检测到QR，回到ready态
	cv::Point2f cen_(src.cols/2,src.rows/2);
	if( ptset_QR.size()>500  )
	{
		int res_=aroundCenter(ptset_QR,cen_);
		if(1==res_)
		{
			float angleQR_;
			angleQR_=scanQR();
			ap_->haveSelfLocate(angleQR_);
		//	ap_->setSelectPath();
			state_.flag_eventFinish=1;
		}
		else if(0==res_)
		{
			setTwist(0.1);
			ap_->haveNotSelflocate();
		}
		else
		{
			ROS_INFO("wtf?!!!");
		}
	}
	else
	{
		state_.next_state = AutoPatrolEvent::ready;
		ap_->haveNotSelflocate();
	}
	return;
}

}//namespace


