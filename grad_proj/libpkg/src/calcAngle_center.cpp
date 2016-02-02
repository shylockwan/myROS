#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
using namespace std;
using namespace cv;
	
//左平面为负，右平面为正，从上到下未0到180度，小车的测程信息则相反，左边正，右边负；
//==============================================================================
//函数名： calcAngle
//作者：王华威
//日期:  2015-7-22
//功能: 计算连通域与QR夹角
//修改记录：
//==============================================================================

void LeastsquareFit1(std::vector<cv::Point2f >& ptset,cv::Mat& A)
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
	Vec4f line;
   fitLine(Z,line,CV_DIST_L2,0,0.01,0.01);
   A.at<float>(1,0)=line[1]/line[0];
}


void calcContourAngle(std::vector<cv::Point2f > ptset,float& angle,cv::Point2f cen_)
{
		if(ptset.empty())
			return;
		cv::Mat paramA=cv::Mat::zeros(2,1,CV_32FC1);
		LeastsquareFit1(ptset,paramA);
		float temp_max=ptset[0].y;
		float temp_min=temp_max;
		float temp_max_x=ptset[0].x;
		float temp_min_x=temp_max_x;
		for(std::vector<cv::Point2f >::iterator it=ptset.begin();it!=ptset.end();it++)
		{
			float err_=abs(it->y-cen_.y);
			if(err_>abs(temp_max-cen_.y))
			{
				temp_max=it->y;
			}
		    if(err_<abs(temp_min-cen_.y))
			{
				temp_min=it->y;
		    }
			float err_2=abs(it->x-cen_.x);
			if(err_2>abs(temp_max_x-cen_.x))
			{
				temp_max_x=it->x;
			}
			if(err_2<abs(temp_min_x-cen_.x))
			{
				temp_min_x=it->x;
			}
		}

		//cout<<"cen_x:"<<cen_.x<<"  cen_y:"<<cen_.y<<endl;
		//float a0=paramA.at<float>(0,0);
		float a1=paramA.at<float>(1,0);
	//	cout<<"a1:"<<a1<<"  a0:"<<a0<<endl;
	//	float temp_max_x_y=a1*temp_max_x+a0;
	//	float temp_min_x_y=a1*temp_min_x+a0;
	//	cout<<"temp_max:"<<temp_max<<"   temp_min:"<<temp_min<<endl;
	//	cout<<"temp_max_x:"<<temp_max_x<<"   temp_min_x:"<<temp_min_x<<endl;
	//	cout<<"( "<<temp_max_x<<" , "<<temp_max_x_y<<"   )    ("<<temp_min_x<<" , "<<temp_min_x_y<<" ) "<<endl;
	//	float temp_x1=(temp_max-a0)/a1;
	//	float temp_x2=(temp_min-a0)/a1;
	//	cout<<"( "<<temp_x1<<" , "<<temp_max<<"   )    ("<<temp_x2<<" , "<<temp_min<<" ) "<<endl;
		float deltay=temp_max-temp_min;
	//	cout<<"deltay:"<<deltay<<endl;
		float deltax=(temp_max_x-temp_min_x);
		if(a1<3 && a1>-3)
		{
			if(a1==0)
			{
				if(deltax>0)
					angle=1.57;
				if(deltax<0)
					angle=-1.57;
				if(deltax==0)
					angle=0;
			}
			else if(a1>0)
			{
				if(deltax>=0)
					angle=atan(1/a1);
				else
					angle=atan(1/a1)-CV_PI;
			}
			else
			{
				if(deltax>=0)
							angle=-atan(a1)+CV_PI/2;
						else
							angle=-atan(a1)-CV_PI/2;
			}
		}
		else
		{
			if(deltay>=0)
					angle=atan(1/a1);
			else
				angle=a1>0?atan(1/a1)-CV_PI:atan(1/a1)+CV_PI;
		}


		return;

	}

#if 0
 void calcContourAngle(std::vector<cv::Point2f > ptset,float& angle,cv::Point2f cen_)
 {

 if(ptset.empty())
 return;
 cv::Mat paramA=cv::Mat::zeros(2,1,CV_32FC1);
 LeastsquareFit1(ptset,paramA);
 float temp_max=ptset[0].y;
 float temp_min=temp_max;
 float temp_max_x=ptset[0].x;
 float temp_min_x=temp_max_x;
 for(std::vector<cv::Point2f >::iterator it=ptset.begin();it!=ptset.end();it++)
 {
 float err_=abs(it->y-cen_.y);
 if(err_>abs(temp_max-cen_.y))
 {
 temp_max=it->y;
 }
 if(err_<abs(temp_min-cen_.y))
 {
 temp_min=it->y;
 }
 float err_2=abs(it->x-cen_.x);
 if(err_2>abs(temp_max_x-cen_.x))
 {
 temp_max_x=it->x;
 }
 if(err_2<abs(temp_min_x-cen_.x))
 {
 temp_min_x=it->x;
 }
 }
 cout<<"temp_max_y:"<<temp_max<<"   temp_min_y:"<<temp_min<<endl;
 cout<<"temp_max_x:"<<temp_max_x<<"   temp_min:"<<temp_min_x<<endl;
 float a0=paramA.at<float>(0,0);
 float a1=paramA.at<float>(1,0);
 cout<<"a1:"<<a1<<endl;
 float deltax=a1*(temp_max-temp_min);
 cout<<"deltax:"<<deltax<<"  deltay: "<<temp_max-temp_min<<endl;
 float deltay=(temp_max-temp_min);

 if(a1<10&&a1>-10)//角度远离正负90度的情况
 {
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
	 {
	 	angle = atan( (deltax)/(deltay) );
	 }
}
 else
 {
	 if(temp_max_x<temp_min_x)
	 {
	 float theta_=atan( (deltax)/(deltay));
	 theta_=(theta_>0?theta_-CV_PI:theta_+CV_PI);
	 angle = theta_;
	 }
	 else
	 {
	 angle = atan( (deltax)/(deltay) );
	 }
 }
 return;

 }
#endif

#if 0
void calcContourAngle(std::vector<cv::Point2f> ptset, float& angle,
		cv::Point2f cen_) {
	if (ptset.empty())
		return;
//	cv::Point2f pt_left,pt_right,pt_up,pt_down;
//	pt_left=pt_right=pt_up=pt_down=ptset[0];
//	for (std::vector<cv::Point2f>::iterator it = ptset.begin();
//			it != ptset.end(); it++) {
//		if( it->x < pt_left.x )
//			pt_left= *it;
//		if( it->x > pt_right.x)
//			pt_right= *it;
//		if( it->y > pt_up.y)
//			pt_up= *it;
//		if( it->y < pt_down.y)
//			pt_down= *it;
//	}
//
//	cv::Point2f pt_;
//	pt_.x=(pt_right.x+pt_left.x+pt_up.x+pt_down.x)/4;
//	pt_.y=(pt_right.y+pt_left.y+pt_up.y+pt_down.y)/4;
//	cout << "temp_y:" << pt_.y << "   temp_x:" << pt_.x << endl;
//	float deltax = pt_.x-cen_.x;
//	float deltay =pt_.y-cen_.y;
//
	cv::Moments m=cv::moments(ptset);
	cv::Point2f pt(m.m10/m.m00,m.m01/m.m00);
	//cout << "temp_y:" << pt.y<< "   temp_x:" << pt.x << endl;
	float deltax = pt.x-cen_.x;
	float deltay =pt.y-cen_.y;


	if (deltay == 0) {
		if (deltax > 0)
			angle = 1.57;
		if (deltax < 0)
			angle = -1.57;
		if (deltax == 0)
			angle = 0;
	} else if (deltay < 0) {
		float theta_ = atan((deltax) / (deltay));
		angle = (theta_ > 0 ? theta_ - CV_PI : theta_ + CV_PI);
	} else
		angle = atan((deltax) / (deltay));
	return;
}
#endif

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
