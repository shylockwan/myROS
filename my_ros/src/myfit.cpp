#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "my_ros/control_param.h"
#include <geometry_msgs/Pose2D.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <fstream>

 static std::ofstream f;
namespace myros {
class myfit {
public:
	ros::NodeHandle node_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Publisher  pub_;
	ros::Subscriber odom_sub;
	cv::Mat_<uchar> kernel_;

	std::vector<cv::Point2f> ptset_pic; //opencv坐标;
	std::vector<cv::Point2f> ptset_world;//当前世界坐标
	std::vector<cv::Point2f> ptset_world_pre; //上一帧
	cv::Point2f pt_obj, pt_robot;
	cv::Mat paramA_;
	cv::Mat_<cv::Vec3b> field_world;
	float dist_;
	float theta_1, theta_2, theta_robot;
	float FUYANG, CHUIZHI, SHUIPING, HEIGHT;
	long int fit_contro_ID;
	myfit(ros::NodeHandle nh_) :
			node_(nh_), it_(nh_), dist_(0.0), theta_1(0.0), theta_2(0.0), theta_robot(
					0.0), FUYANG(1.00), CHUIZHI(0.262), SHUIPING(0.34), HEIGHT(
					32.0) ,fit_contro_ID(0){
		// Subscrive to input video feed and publish output video feed
		paramA_=cv::Mat::zeros(4,1,CV_32FC1);
		field_world=cv::Mat_<cv::Vec3b>::zeros(500,500);
		ptset_world_pre.clear();
		ptset_pic.clear();
		ptset_world.clear();
		pt_obj.x = 0;
		pt_obj.y = 0;
		pt_robot.x = 0;
		pt_robot.y = 0;
		kernel_ =
				(cv::Mat_<uchar>(5, 5) << 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0); //morph kernel
	}
	bool init() {
		ros::param::get("fuyang", FUYANG);
		ros::param::get("chuizhi", CHUIZHI);
		ros::param::get("shuiping", SHUIPING);
		ros::param::get("height", HEIGHT);
		 pub_=node_.advertise<my_ros::control_param>("/control_param",1);
		image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &myfit::imageCb,
				this);
		odom_sub = node_.subscribe<nav_msgs::Odometry>("/odom", 1,
				&myfit::odomCallback, this);
		cv::namedWindow("src", CV_WINDOW_KEEPRATIO | CV_WINDOW_NORMAL);
		cv::namedWindow("dst", CV_WINDOW_KEEPRATIO | CV_WINDOW_NORMAL);
		cv::namedWindow("field", CV_WINDOW_KEEPRATIO | CV_WINDOW_NORMAL);

		return true;
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg) {
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg,
					sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		cv::Mat src(cv_ptr->image);
		// processing on the video stream
		int r = src.rows;
		int c = src.cols;
		cv::Mat dst;
#if 1////RGB转HSV
		cv::Mat hsv;
		cv::cvtColor(src, hsv, CV_BGR2HSV_FULL);
		std::vector<cv::Mat> hsv_planes;
		split(hsv, hsv_planes);
		cv::Mat* src_h = new cv::Mat(hsv_planes[0]);
		cv::Mat src_s = hsv_planes[1];
		cv::GaussianBlur(*src_h, *src_h, cv::Size(7, 7), 1);
		this->ColorThresh(*src_h, *src_h, src_s);
		cv::medianBlur(*src_h, *src_h, 15);
		//
#endif

		cv::morphologyEx(*src_h, *src_h, cv::MORPH_OPEN, kernel_,
				cv::Point(-1, -1), 1);
		cv::Mat dst_h = *src_h;
		cv::cvtColor(dst_h, dst, CV_GRAY2BGR);

#if 1  ////取点记录，并转换为现场坐标进行最小二乘法拟合，得到拟合曲线参数a0,a1,a2,a3
		this->GetPointset(dst_h, ptset_pic);
		std::vector<cv::Point2f> ptset_temp;
		ptset_world.clear();
		if (!ptset_pic.empty()) {
			cv::Point2f pt_filed,pt_world;
			for (std::vector<cv::Point2f>::iterator it = ptset_pic.begin();
					it != ptset_pic.end(); it++) {
				pt_filed = this->framePic2Field(c, r, (*it));
				pt_world=this->frameFiled2World(pt_filed);
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

       /////////////////////////////////////////
		field_world.setTo(0);//顯示圖片

		if(!ptset_world.empty())
		{
			int x_=pt_robot.x+250;
			int y_=250-pt_robot.y;
			uchar* ptr_=field_world.ptr(y_);
			ptr_[x_ * 3] = 0; //第row行的第col个像素点的第一个通道值 Blue
			ptr_[x_ * 3 + 1] = 255; // Green
			ptr_[x_ * 3 + 2] = 0; // Red
			
		for(std::vector<cv::Point2f>::iterator it=ptset_world.begin();it!=ptset_world.end();it++)
		{
			int i=it->y;
			int j=it->x;
			i=250-i;
			j=250+j;
			ptr_=field_world.ptr(i);
			ptr_[j * 3] = 0; //第row行的第col个像素点的第一个通道值 Blue
			ptr_[j * 3 + 1] = 0; // Green
			ptr_[j * 3 + 2] = 255; // Red
		}
		x_=pt_obj.x+250;
		y_=250-pt_obj.y;
			ptr_=field_world.ptr(y_);
			ptr_[x_ * 3] = 255; //第row行的第col个像素点的第一个通道值 Blue
			ptr_[x_ * 3 + 1] = 255; // Green
			ptr_[x_ * 3 + 2] = 255; // Red
			
			
		
		}
		if(ptset_temp.size()*2>=ptset_world_pre.size())
		{
			 ptset_world_pre.clear();
		     ptset_world_pre=ptset_temp;//保存当前帧获得的世界坐标
		}
		//////////////////////////////////////////////
		cv::imshow("field",field_world);
		//ROS_INFO("pt counts:%d",ptset_world.size());
		
		paramA_.setTo(0);
		this->LeastsquareFit(ptset_world, paramA_);
#endif
		float a0, a1, a2, a3;
		a0 = paramA_.at<double>(0, 0);
		a1 = paramA_.at<double>(1, 0);
		a2 = paramA_.at<double>(2, 0);
		a3 = paramA_.at<double>(3, 0);
		cv::rectangle(dst, cv::Point(c / 2 - 10, r / 2 - 10),
				cv::Point(c / 2 + 10, r / 2 + 10), cv::Scalar(0, 255, 0), 5); ///标出现场坐标中心
		this->getLocation(c, r);
		DrawLine(dst, ptset_pic);
		// Update GUI Window
		cv::imshow("dst",dst);
		cv::imshow("src", src);
        cv::waitKey(20);
		//ROS_INFO("x= %f *y3 + %f *y2 + %f *y1 + %f ", a3, a2, a1, a0);
		//ROS_INFO("inobject____theta1: %f , theta2:   %f   dist_: %f ",theta_1,theta_2,dist_);
		///发送数据
	}
	;		//imageCb

	void getLocation(const int& width, const int& height) {
		if (paramA_.empty() || ptset_world.empty())
			return;
		//ros::Time cur_time=ros::Time::now();
		// ros::Time last_time=ros::Time::now();
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
		float err_ = abs( myfunc2(myfunc(start_y, paramA_), start_y) );

		for (float i = start_y; i < end_y; i += delta_y) {
			float x_ = myfunc(i, paramA_);
			float err_temp = abs(myfunc2(x_, i));
			//ROS_INFO("( %f,%f),;  err %f ",x_,i,err_temp);
			if (err_ > err_temp) {
				pt_.x = x_;
				pt_.y = i;
				err_ = err_temp;
			}
		}			    		//遍历误差err，最小者为交点，输出至pt_；
		pt_obj.x = pt_.x ;	pt_obj.y = pt_.y;
		//cv::Point2f pt_n, pt_p;
		//pt_n.y = pt_.y + delta_y;
		//pt_n.x = myfunc(pt_n.y, paramA_);
		//pt_p.y = pt_.y - delta_y;
		//pt_p.x = myfunc(pt_p.y, paramA_);


		theta_2 = 0.0;
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
			theta_1 = atan( (deltax)/(deltay) )+theta_robot;

		dist_=0;

		return;
	}
	;			///getLocation();

	void GetPointset(const cv::Mat& in, std::vector<cv::Point2f>& ptset) {
		if (in.empty() || in.channels() != 1)
			return;

		ptset.clear();
		cv::Point2f pt;
		for (int i = 0; i < in.rows; i++) {
			const uchar* dataI = in.ptr<uchar>(i);
			for (int j = 0; j < in.cols; j++) {
				if (dataI[j] == 0)
					continue;
				else {
					pt.y = i;
					pt.x = j;
					ptset.push_back(pt);
				}			//if

			}			//for2 loop
		}			//for1 loop
	}
	;			///getPointset();待优化：分类记录点或者去除孤立区域

	void ColorThresh(cv::Mat& in, cv::Mat& out, cv::Mat& mask) {
		if (in.empty() || in.channels() != 1)
			return;

		if (out.empty())
			out.create(in.rows, in.cols, CV_8UC1);

		for (int i = 0; i < in.rows; i++) {
			uchar* ptr_in = in.ptr<uchar>((int) i);
			uchar* ptr_mk = mask.ptr<uchar>((int) i);
			uchar* ptr_out = out.ptr<uchar>(i);
			for (int j = 0; j < in.cols; j++) {
				if (ptr_in[j] > 15 && ptr_in[j] < 230 || ptr_mk[j] < 70)//Green&Blue channel
					ptr_out[j] = 0;
				else
					ptr_out[j] = 255;
			}
		}			//for1 loop
	}
	;			//ColorThresh

	void LeastsquareFit(std::vector<cv::Point2f> &ptset, cv::Mat& paramA) {
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

	cv::Point2f framePic2Field(const int& width, const int& height,
			cv::Point2f& pt) {
		// ros::Time cur_time=ros::Time::now();
		// ros::Time last_time=ros::Time::now();
		// alpha0=0.262;//m_VerShiYeAng;摄影机垂直视野角；单位为rad
		// gama0= 1.047;//m_FuYangAng; 摄影机俯仰角
		// beta0=0.35;//m_HorShiYeAng;摄影机水平视野角
		// m_Height=32; //摄影机高度
		const float alpha0 = CHUIZHI;
		const float gama0 = FUYANG;
		const float beta0 = SHUIPING;
		const float m_Height = HEIGHT;
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
	cv::Point2f frameFiled2World(cv::Point2f& field_pt) {
		////theta_map=getYaw()
        float field_y=-HEIGHT * tan(FUYANG);
		cv::Point2f pt_;
		float dist;
			dist = sqrt(
					(field_pt.x) * (field_pt.x)
							+ (field_y -  field_pt.y )* (field_y - field_pt.y )); //RADIUS HEIGHT*tan(FUYANG) 单位/m
			//ROS_INFO("dist  %f", dist);
			float pt_theta = atan( field_pt.x / ( (field_pt.y)-field_y ) );
			float obj_theta_=theta_robot - pt_theta;
			if(obj_theta_>CV_PI)
				obj_theta_=(obj_theta_-2*CV_PI);
			if(obj_theta_<-CV_PI)
				obj_theta_=(obj_theta_+2*CV_PI);
			pt_.x = pt_robot.x - dist * sin(obj_theta_);
			pt_.y = pt_robot.y + dist * cos(obj_theta_);
			return pt_;

	}//frameField2World();
	;

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

	float myfunc2(float x, float y) {

		float res = (x-pt_robot.x) *( x-pt_robot.x)
				+ (y - pt_robot.y) * (y - pt_robot.y)
				- 10 * 10;		//cm
		return res;
	}
	;

	void DrawLine(cv::Mat& res, std::vector<cv::Point2f>& ptset) {
		if(ptset.empty())
			return;
		cv::Mat paramB;
		this->LeastsquareFit(ptset, paramB);
		float start_y = ptset[0].y;
		float end_y = start_y;

		for (std::vector<cv::Point2f>::iterator it = ptset.begin();
				it != ptset.end(); it++) {
			if (start_y > (*it).y) {
				start_y = (*it).y;
			}
			if (end_y < (*it).y) {
				end_y = (*it).y;
			}
		}
		float delta_y = (end_y - start_y) / 800;
		for (float i = start_y; i < end_y; i += delta_y) {
			int x = myfunc(i, paramB);
			if (x <= 0 || x >= res.cols)
				continue;
			uchar* data = res.ptr<uchar>((int) i);
			data[x * 3] = 0; //第row行的第col个像素点的第一个通道值 Blue
			data[x * 3 + 1] = 0; // Green
			data[x * 3 + 2] = 255; // Red
			// std::cout<<"["<<y<<","<<i<<"]"<<std::endl;
		}

		//cv::imshow("result", res);
		//cv::waitKey(30);

	}
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
		//ROS_INFO("(%f ,%f)  ,%f",msg->linear.x,msg->linear.y,msg->angular.z);
		geometry_msgs::Quaternion quat = msg->pose.pose.orientation;
		pt_robot.y = 100*msg->pose.pose.position.x;//单位 m->cm
		pt_robot.x = -100*msg->pose.pose.position.y;
		theta_robot = tf::getYaw(quat);//单位 rad

	}
	void spin()
	{
		ros::spinOnce();
		my_ros::control_param contro_param;
		contro_param.theta1=theta_1;
		contro_param.theta2=theta_2;
		contro_param.dist=dist_;
		contro_param.fit_ID=(++fit_contro_ID);
		pub_.publish(contro_param);
		f<<fit_contro_ID<<" ("<<pt_obj.x<<","<<pt_obj.y<<") ("
				<<pt_robot.x<<","<<pt_robot.y<<")";
		f<<std::endl;

	}
};

class mymap {
public:
	mymap(ros::NodeHandle n):nh_(n) {
		last_pt.x=last_pt.y=300;
		cur_pt=last_pt;
		last_ptobj=last_pt;
		cur_ptobj=last_pt;
		map = cv::Mat_<cv::Vec3b>::zeros(600, 600);
		objmap = cv::Mat_<cv::Vec3b>::zeros(600, 600);
		cv::circle(map, cur_pt, 1, cv::Scalar(0, 0, 255), 1);
		cv::namedWindow("map_robot/cm", CV_WINDOW_KEEPRATIO | CV_WINDOW_NORMAL);
		cv::namedWindow("map_obj/cm", CV_WINDOW_KEEPRATIO | CV_WINDOW_NORMAL);
		//cv::setMouseCallback("map_obj/cm", &(this->mouseCallback), 0);
	};
	/*void mouseCallback(int event, int x, int y, int flags, void* ustc) {
			if (event == CV_EVENT_LBUTTONDOWN) {
				CvPoint pt = cvPoint(x, y);
				char temp[16];
				sprintf(temp, "(%d,%d)", pt.x, pt.y);
				cv::putText(objmap, temp, pt, cv::FONT_HERSHEY_SIMPLEX, 2,
						cv::Scalar(0, 0, 255), 1);
				cv::circle(objmap, pt, 1, cv::Scalar(255, 255, 255), 1);

			}
		}*/
	void updateMap(const cv::Point& pt_robot,const cv::Point& pt_obj ) {

		cur_pt.x = (pt_robot.x ) + 300;
		cur_pt.y = 300 - (pt_robot.y );
		cur_ptobj.x = (pt_obj.x) + 300;
		cur_ptobj.y = 300 - (pt_obj.y );
		cv::circle(map, cur_pt, 1, cv::Scalar(0, 0, 255), 1);
		cv::line(map, last_pt, cur_pt, cv::Scalar(255, 255, 0), 1);
		cv::circle(objmap, cur_ptobj, 1, cv::Scalar(0, 0, 255), 1);
		cv::line(objmap, last_ptobj, cur_ptobj, cv::Scalar(255, 255, 0), 1);
		cv::imshow("map_robot/cm", map);
	    cv::imshow("map_obj/cm", objmap);
		cv::waitKey(30);
		last_pt = cur_pt;
		last_ptobj = cur_ptobj;
		ROS_INFO("pt_robot__x:   %d,   pt_robot__y:  %d", pt_robot.x,pt_robot.y);
		ROS_INFO("pt_obj__x:   %d,   pt_obj__y:  %d", pt_obj.x,pt_obj.y);
	}

private:
	cv::Point last_pt;
	cv::Point cur_pt;
	cv::Point last_ptobj;
	cv::Point cur_ptobj;
	ros::NodeHandle nh_;
	cv::Mat_<cv::Vec3b> map;
	cv::Mat_<cv::Vec3b> objmap;
};
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_fit_application");
  ros::NodeHandle nh;
  myros::myfit mf(nh);
  myros::mymap mm(nh);
  ROS_INFO("my_fit application has been init and wait for arrival of usb_cam image!!!");
  f.open("/home/shylockwang/myfit_out.txt",std::ios_base::out);
 // ros::Publisher pub_ = nh.advertise<my_fit::control_param>("/myfit/control_param", 1);
//  ros::Publisher objpose_pub=nh.advertise<geometry_msgs::Pose2D>("/myfit/objectPose",1);
  ros::Rate loop_rate(5);
  cv::waitKey(3000);
 // my_fit::control_param contro_param;
  if(mf.init() )
  {
  while(ros::ok())
  {
	  mf.spin();
	  mm.updateMap(mf.pt_robot,mf.pt_obj);
	  loop_rate.sleep();
  }
  }

    return 0;
}

