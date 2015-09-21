//==============================================================================
//函数名： frameTrans
//作者：王华威
//日期:  2015
//功能: 坐标系转换，输出在转换坐标系后的坐标值点
//输入: const int& width, const int& height,cv::Point2f& pt，const float gama0 ，const float m_Height ，
//      const float alpha0，const float beta0 
//输出: cv::Point2f
//修改记录：
//==============================================================================
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
cv::Point2f framePic2Field(const int& width, const int& height,
			cv::Point& pt,const float gama0 ,const float m_Height ,const float alpha0,
			const float beta0 ) {
		// ros::Time cur_time=ros::Time::now();
		// ros::Time last_time=ros::Time::now();
		// alpha0=0.262;//m_VerShiYeAng;摄影机垂直视野角；单位为rad
		// gama0= 1.047;//m_FuYangAng; 摄影机俯仰角
		// beta0=0.35;//m_HorShiYeAng;摄影机水平视野角
		// m_Height=32; //摄影机高度
		//opencv坐标转换为图像坐标
		int xp = pt.x - (width - 1) / 2;
		int yp = (height - 1) / 2 - pt.y;

		//图像坐标转换为实际场景坐标
		float tan_alpha1 = 2 * yp * tan(alpha0) / height;
		cv::Point2f pt_;
		//得到的XPYP即为距离镜头中心的实际坐标距离；
		pt_.y =
				m_Height * tan_alpha1
						* ((1 + tan(gama0) * tan(gama0))
								/ (1 - tan(gama0) * tan_alpha1));
		float UG = m_Height * (tan(gama0) - tan(gama0 - alpha0))
				* cos(gama0 - alpha0) / (cos(gama0 - alpha0) - cos(gama0));
		pt_.x = 2 * (UG + pt_.y) * m_Height * xp * tan(beta0)
				/ (UG * cos(gama0) * width);
		//	cur_time=ros::Time::now();
		//double dt=(cur_time-last_time).toSec();
		//ROS_INFO("framePic2Field() time:  %f",dt);

		return pt_;
	}				//framePic2Field();

	////输入现场坐标，输出地图坐标
	cv::Point2f frameFiled2World(cv::Point2f& field_pt,cv::Point2f PtRobot,
			const float ThetaRobot ,const float gama0,const float height_) {
		////theta_map=getYaw()

        float field_y=-height_ * tan(gama0);
		cv::Point2f pt_;
		float dist;
			dist = sqrt(
					(field_pt.x) * (field_pt.x)
							+ (field_y -  field_pt.y )* (field_y - field_pt.y )); //RADIUS HEIGHT*tan(FUYANG) 单位/m
			//ROS_INFO("dist  %f", dist);
			float pt_theta = atan( field_pt.x / ( (field_pt.y)-field_y ) );
			float obj_theta_=ThetaRobot - pt_theta;
			if(obj_theta_>CV_PI)
				obj_theta_=(obj_theta_-2*CV_PI);
			if(obj_theta_<-CV_PI)
				obj_theta_=(obj_theta_+2*CV_PI);
			pt_.x = PtRobot.x - dist * sin(obj_theta_);
			pt_.y = PtRobot.y + dist * cos(obj_theta_);
			return pt_;

	}//frameField2World();
	;


