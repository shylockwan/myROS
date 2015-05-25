#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "my_ros/control_param.h"
#include <geometry_msgs/Pose2D.h>
#include <vector>
namespace myros {
class myfit {
public:
	ros::NodeHandle node_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber odom_sub;

	cv::Mat_<uchar> kernel_;
	std::vector<cv::Point2f> ptset_field; //现场坐标
	std::vector<cv::Point2f> ptset_pic; //opencv坐标;
	std::vector<cv::Point2f> ptset_world;
	cv::Point pt_obj, pt_robot;
	cv::Mat paramA_;
	float dist_;
	float theta_1, theta_2, theta_map;
	float FUYANG, CHUIZHI, SHUIPING, HEIGHT;
	myfit(ros::NodeHandle nh_) :
			node_(nh_), it_(nh_), dist_(0.0), theta_1(0.0), theta_2(0.0), theta_map(
					0.0), FUYANG(1.00), CHUIZHI(0.262), SHUIPING(0.34), HEIGHT(
					32.0) {
		// Subscrive to input video feed and publish output video feed

		ptset_field.clear();
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
		image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &myfit::imageCb,
				this);
		odom_sub = node_.subscribe<nav_msgs::Odometry>("/odom", 1,
				&myfit::odomCallback, this);
		cv::namedWindow("src", CV_WINDOW_KEEPRATIO | CV_WINDOW_NORMAL);
		cv::namedWindow("result", CV_WINDOW_KEEPRATIO | CV_WINDOW_NORMAL);
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
		//cv::imshow("mask",*src_h);
#endif

		cv::morphologyEx(*src_h, *src_h, cv::MORPH_OPEN, kernel_,
				cv::Point(-1, -1), 1);
		cv::Mat dst_h = *src_h;
		cv::cvtColor(dst_h, dst, CV_GRAY2BGR);
		ptset_field.clear();

#if 1  ////取点记录，并转换为现场坐标进行最小二乘法拟合，得到拟合曲线参数a0,a1,a2,a3
		this->GetPointset(dst_h, ptset_pic);
		if (!ptset_pic.empty()) {
			ptset_field.clear();
			for (std::vector<cv::Point2f>::iterator it = ptset_pic.begin();
					it != ptset_pic.end(); it++) {

				cv::Point2f pt_ = this->framePic2Field(c, r, (*it));
				ptset_world.push_back(pt_);

			}
		}

		paramA_.setTo(0);
		this->LeastsquareFit(ptset_field, paramA_);
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
		cv::imshow("src", src);
		cv::imshow("result", dst);
		ROS_INFO("x= %f *y3 + %f *y2 + %f *y1 + %f ", a3, a2, a1, a0);
		//ROS_INFO("inobject____theta1: %f , theta2:   %f   dist_: %f ",theta_1,theta_2,dist_);
		///发送数据
		cv::waitKey(30);

	}
	;		//imageCb

	void getLocation(const int& width, const int& height) {
		if (paramA_.empty() || ptset_field.empty())
			return;
		//ros::Time cur_time=ros::Time::now();
		// ros::Time last_time=ros::Time::now();
		cv::Mat field = cv::Mat::zeros(200, 200, CV_8UC3);
		float start_y = -HEIGHT * tan(FUYANG);
		float end_y = this->ptset_field[0].y;

		for (std::vector<cv::Point2f>::iterator it = ptset_field.begin();
				it != ptset_field.end(); it++) {
			if (start_y > (*it).y) {
				start_y = (*it).y;
			}
			if (end_y < (*it).y) {
				end_y = (*it).y;
			}
		}
		float delta_y = (end_y - start_y) / 600;
		std::vector<cv::Point2f> pt_vec;
		cv::Point2f pt_(myfunc(end_y, paramA_), end_y);
		float err_ = abs(myfunc2(myfunc(end_y, paramA_), end_y));

		for (float i = start_y; i < end_y; i += delta_y) {
			float x_ = myfunc(i, paramA_);
			float err_temp = abs(myfunc2(x_, i));
			//ROS_INFO("( %f,%f),;  err %f ",x_,i,err_temp);
			if (err_temp < 20) {
				cv::Point2f pt(myfunc(i, paramA_), i);
				pt_vec.push_back(pt);
			}
			if (err_ > err_temp) {
				pt_.x = x_;
				pt_.y = i;
				err_ = err_temp;
			}
			int x = (int) x_ + 100;
			int y = 100 - (int) i;
			if (x <= 0 || x >= field.cols || y <= 0 || y >= field.rows)
				continue;
			else {
				uchar* data = field.ptr<uchar>(y);
				data[(x) * 3] = 255; //第row行的第col个像素点的第一个通道值 Blue
				data[(x) * 3 + 1] = 0; // Green
				data[(x) * 3 + 2] = 0; // Red
				// std::cout<<"["<<y<<","<<i<<"]"<<std::endl;
			}

		}			    		//遍历误差err，最小者为交点，输出至pt_；
		ROS_INFO("x    %f,;  y %f", pt_.x, pt_.y);
		int x = (int) pt_.x + 100;
		int y = 100 - (int) pt_.y;
		cv::rectangle(field, cv::Point(field.cols / 2 - 1, field.rows / 2 - 1),
				cv::Point(field.cols / 2 + 1, field.rows / 2 + 1),
				cv::Scalar(0, 0, 255), 5);
		if (!(x <= 0 || x >= field.cols || y <= 0 || y >= field.rows)) {
			uchar* data = field.ptr<uchar>(y);
			data[(x) * 3] = 0; //第row行的第col个像素点的第一个通道值 Blue
			data[(x) * 3 + 1] = 255; // Green
			data[(x) * 3 + 2] = 0; // Red
		}

		cv::imshow("field", field);
		cv::Point2f pt_n, pt_p;
		dist_ = pt_.x;
		pt_n.y = pt_.y + delta_y;
		pt_n.x = myfunc(pt_n.y, paramA_);
		pt_p.y = pt_.y - delta_y;
		pt_p.x = myfunc(pt_p.y, paramA_);

		theta_2 = atan((pt_n.x - pt_p.x) / (pt_n.y - pt_p.y + 1e-7));
		if (theta_2 > 1.047)
			theta_2 = 1.047;
		if (theta_2 < -1.047)
			theta_2 = -1.047;
		theta_1 = atan(dist_ / (HEIGHT * tan(FUYANG) + pt_.y));
		//cur_time=ros::Time::now();
		//double dt=(cur_time-last_time).toSec();
		//ROS_INFO("getLocation() time:  %f",dt);
		///现场坐标发送
		pt_obj.x = dist_;
		pt_obj.y = pt_.y;
		//obj_pose.x=dist_;
		//obj_pose.y=pt_.y;
		//obj_pose.theta=theta_1;

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
		F.create(3, ptset.size());
		Y.create(ptset.size(), 1);
		for (int i = 0; i != ptset.size(); i++) {
			///X和Y置换得到x=f(y);
			double pt_x = ptset[i].y;
			F(0, i) = 1;
			F(1, i) = pt_x;
			F(2, i) = pt_x * pt_x;
			//F(3,i)=pt_x*pt_x*pt_x;
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
	void frameFiled2World() {

	}
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
				+ param.at<double>(2, 0) * x * x;
		//+param.at<double>(3,0)*x*x*x;
		return y;
	}
	;

	float myfunc2(float x, float y) {

		float res = x * x
				+ (y + HEIGHT * tan(FUYANG)) * (y + HEIGHT * tan(FUYANG))
				- 12 * 12;		//cm
		return res;
	}
	;

	void DrawLine(cv::Mat& res, std::vector<cv::Point2f>& ptset) {
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

		cv::imshow("result", res);

	}
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
		//ROS_INFO("(%f ,%f)  ,%f",msg->linear.x,msg->linear.y,msg->angular.z);
		geometry_msgs::Quaternion quat = msg->pose.pose.orientation;
		pt_robot.y = msg->pose.pose.position.x;
		pt_robot.x = msg->pose.pose.position.y;
		theta_map = tf::getYaw(quat);

	}
};
}
