#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
using namespace std;
using namespace cv;
//==============================================================================
//函数名： LeastsquareFit1
//作者：王华威
//日期:  2015-7-22
//功能: 一次直线最小二乘拟合，用于检测连通域与QR夹角
//修改记录：
//==============================================================================
void LeastsquareFit1(std::vector<cv::Point2f >& ptset,cv::Mat& A)
{
	if (ptset.empty())
		return;
	cv::Mat_<float> F, Y;
	F.create(2, ptset.size());
	Y.create(ptset.size(), 2);
	for (int i = 0; i < ptset.size(); i++) {
		///X和Y置换得到x=f(y);
		float pt_x = ptset[i].y;
		F(0, i) = 1;
		F(1, i) = pt_x;
		Y(i, 0) = (float)ptset[i].x;

	}			//for1 loop
	cv::Mat F_Ft;
	cv::mulTransposed(F, F_Ft, false);
	cv::Mat temp = (F_Ft.inv());
	//std::cout<<temp.rows<<"  "<<temp.cols<<std::endl<<F.rows<<" "<<F.cols;
	A = temp * F * Y;		//a0 a1 a2 a3;opencv中貌似已经包含cv::solve函数用于求解

}			//LeastsquareFit;;
//左平面为负，右平面为正，从上到下未0到180度，小车的测程信息则相反，左边正，右边负；
//==============================================================================
//函数名： calcAngle
//作者：王华威
//日期:  2015-7-22
//功能: 计算连通域与QR夹角
//修改记录：
//==============================================================================
void calcContourAngle(std::vector<cv::Point2f > ptset,float& angle,cv::Point2f cen_)
{
	if(ptset.empty())
		return;
//std::cout<<"aaaaaaaaaaaaaaaaaaaaaa"<<std::endl;
	cv::Mat paramA=cv::Mat::zeros(2,1,CV_32FC1);
	LeastsquareFit1(ptset,paramA);
//std::cout<<"bbbbbbbbbbbbbbbbbbbbbbbb"<<std::endl;
	float temp_max=ptset[0].y;
	float temp_min=temp_max;
	for(std::vector<cv::Point2f >::iterator it=ptset.begin();it!=ptset.end();it++)
	{
//std::cout<<"x:"<<it->x <<"   y"<<it->y <<std::endl;
		float err_=abs(it->y-cen_.y);
		if(err_>abs(temp_max-cen_.y))
		{
			temp_max=it->y;
		}
		if(err_<abs(temp_min-cen_.y))
		{
			temp_min=it->y;
		}
	}
//std::cout<<"cccccccccccccccc"<<std::endl;
	float a0=paramA.at<float>(0,0);
	float a1=paramA.at<float>(1,0);
	//cv::Point2f pt1(temp_min,a1*temp_min+a0);
	//cv::Point2f pt2(temp_max,a1*temp_max+a0);
	float deltax=a1*(temp_max-temp_min);
	float deltay=(temp_max-temp_min);
std::cout<<"deltax:"<<deltax<<"   deltay"<<deltay<<std::endl;
	if(deltay==0)
		{
			if(deltax>0)
				angle=1.57;
			if(deltax<0)
				angle=-1.57;
			if(deltax==0)
				angle=0;
		}
		else if(deltay<0)
		{
			float theta_=atan( (deltax)/(deltay));
			theta_=(theta_>0?theta_-CV_PI:theta_+CV_PI);
			angle = theta_;
		}
		else
			angle = atan( (deltax)/(deltay) );
std::cout<<"eeeeeeeeeeeeeeeeee"<<std::endl;
	return;

}

//==============================================================================
//函数名： aroundCenter
//作者：王华威
//日期:  2015-7-22
//功能: 判断连通域是否大致处于或接近图像中心
//修改记录：
//==============================================================================
int  aroundCenter(std::vector<cv::Point> ptset,const cv::Point2f& center_,float tolerance_=30)
{
	if(ptset.empty())
		return -1;
	cv::Point2f pt_;
	float x_max=ptset[0].x;
	float x_min=x_max;
	float y_max=ptset[0].y;
	float y_min=y_max;
	cv::Point l_(ptset[0]),d_(ptset[0]),r_(ptset[0]),u_(ptset[0]);//取最上下左右的四个点
	for(std::vector<cv::Point>::iterator it=ptset.begin();it!=ptset.end();it++)
	{
		if(it->x>x_max)
		{
			x_max=it->x;
			r_=(*it);
		}
		if(it->x<x_min)
		{
			x_min=it->x;
			l_=(*it);
		}
		if(it->y>y_max)
		{
			y_max=it->y;
			u_=(*it);
		}
		if(it->y<y_min)
		{
			y_min=it->y;
			d_=(*it);
		}
	}
	pt_.x=float((l_.x+r_.x+u_.x+d_.x))/4;
	pt_.y=float((l_.y+r_.y+u_.y+d_.y))/4;
	float error_=abs(pt_.x-center_.x)+abs(pt_.y-center_.y);
	if(error_<tolerance_)
		return 1;
	else
		return 0;
}

////////////////////////////////////////////
