#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "my_fit/control_param.h"

static const std::string OPENCV_WINDOW = "Image window";
static cv::VideoCapture  cap("out.avi");
class myfit
{
public:
	  ros::NodeHandle nh_;
	  image_transport::ImageTransport it_;
	  image_transport::Subscriber image_sub_;
	  ros::Publisher pub_;
	  cv::Mat_<uchar> kernel_;
	  std::vector<cv::Point2f> ptset_;
	  cv::Mat paramA_;
	  float dist_;
	  float theta_;
	  
	  myfit():it_(nh_),dist_(0),theta_(0)
	  {
		  // Subscrive to input video feed and publish output video feed
		      ptset_.clear();
		      kernel_=(cv::Mat_<uchar>(3,3)<<0,1,0,1,1,1,0,1,0); //morph kernel
		     // image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &myfit::imageCb, this);
		      pub_ = nh_.advertise<my_fit::control_param>("/myfit/control_param", 1);
		      cv::namedWindow(OPENCV_WINDOW,CV_WINDOW_KEEPRATIO|CV_WINDOW_NORMAL);
		      cv::namedWindow("result",CV_WINDOW_KEEPRATIO|CV_WINDOW_NORMAL);
		      // cv::namedWindow("edge",CV_WINDOW_KEEPRATIO|CV_WINDOW_NORMAL);
		         cv::namedWindow("mask",CV_WINDOW_KEEPRATIO|CV_WINDOW_NORMAL);
	  }
	  void imageCb()
	  {
	  
		  
             cv::Mat src;
             cap>>src;
		     // processing on the video stream
		     
		     
		     cv::Mat mask,edge;

		     this->ColorThresh(src,mask);
		     cv::morphologyEx(mask,mask,cv::MORPH_OPEN,kernel_,cv::Point(-1,-1),1);
		     cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,kernel_,cv::Point(-1,-1),3);
		     cv::imshow("mask", mask);
		     cv::cvtColor(mask,edge,CV_GRAY2BGR);
		     int r=edge.rows;
		     int c=edge.cols;
		     ptset_.clear();
		     this->GetPointset(mask,ptset_);
		     this->LeastsquareFit(ptset_,paramA_);
		     cv::rectangle(edge,cv::Point(c/2-10,r/2-10), cv::Point(c/2+10,r/2+10),cv::Scalar(0,255,0),5);
		     this->DrawLine(edge,paramA_,ptset_[0],ptset_[ptset_.size()-1]);
		     this->GetLocation(paramA_,&edge);
		     // Update GUI Window
		     cv::imshow(OPENCV_WINDOW, src);
		     my_fit::control_param output_msg;
		     ROS_INFO("dist: %f   theta: %f",dist_,theta_);
		     int tempd=int(dist_/30);
		     int tempt=int(theta_*10);
		     // ROS_INFO("tempd  :  %d  ,tempt  :  %d",tempd,tempt);
		     float w=float(tempd)/10+float(tempt)/20;
		     ROS_INFO("the w is  :  %f",w);
		     output_msg.dist=dist_;
		     output_msg.theta=theta_;
		     
		     pub_.publish(output_msg);
		     cv::waitKey(200);

		     // Output modified video stream
		     // image_pub_.publish(cv_ptr->toImageMsg());
		     
	  };//imageCb

	  void ColorThresh(cv::Mat& in,cv::Mat& out)
	  {
		      	if(in.empty()||in.channels()!=3)
		      		return;

		      	if(out.empty())
		      		out.create(in.rows,in.cols,CV_8UC1);

		      	out.setTo(0);
		      	for(int i=0;i<in.rows;i++)
		      	{
		      		uchar* data = in.ptr<uchar>((int)i);
		      		for(int j=0;j<in.cols;j++)
		      		{
		      			if( data[j * 3 + 0]>100||data[j * 3 + 1]>100)// Red channel
		      				continue;
		      			//
		      			if(data[j * 3] +data[j * 3 + 1]>data[j * 3 + 2])//Green&Blue channel
		      				continue;
		      			out.at<uchar>(i,j)=255;
		      		}
		      	}//for1 loop
	  };//ColorThresh

	  

	   void GetPointset(cv::Mat& in,std::vector<cv::Point2f>& Pt_set)
	    {
	    	if(in.empty())
	    		return;

	    	cv::Point2f pt;
	    	for(int i=0;i<in.rows;i++)
	    		for(int j=0;j<in.cols;j++)
	    		{
	    			if(in.at<uchar>(i,j)==0)
	    				continue;
	    			else
	    			{
	    				pt.x=i;
	    				pt.y=j;
	    				Pt_set.push_back(pt);
	    			}//if

	    		}//for2 loop
	    };

	   void LeastsquareFit(std::vector<cv::Point2f> &pt_set,cv::Mat& paramA)
		{
			if(pt_set.empty())
				return;
			cv::Mat_<double> F,Y;
			F.create(4,pt_set.size());
			Y.create(pt_set.size(),1);
			for(int i=0;i!=pt_set.size();i++)
			{

				double pt_x=pt_set[i].x;
				F(0,i)=1;
				F(1,i)=pt_x;
				F(2,i)=pt_x*pt_x;
				F(3,i)=pt_x*pt_x*pt_x;
				Y(i,0)=pt_set[i].y;


			}//for1 loop
			cv::Mat F_Ft;
			cv::mulTransposed(F,F_Ft,false);
			cv::Mat temp=(F_Ft.inv());
			//std::cout<<temp.rows<<"  "<<temp.cols<<std::endl<<F.rows<<" "<<F.cols;
			paramA=temp*F*Y;//a0 a1 a2 a3;opencv中貌似已经包含cv::solve函数用于求解

		}//LeastsquareFit;

	    void DrawLine(cv::Mat& res,cv::Mat& paramA,cv::Point2f& pt_sta,cv::Point2f& pt_end)
	    {
	    	
	    	if(res.empty()||paramA.empty())
	    		return;
	    	int r=res.rows;
	    	int c=res.cols;
	    	int i,i_stop;
	    	if(pt_sta.x<=pt_end.x)
	    	{
	    	     i=pt_sta.x;
	    	     i_stop=pt_end.x;
	    	}
	    	else
	    	{
	    		 i_stop=pt_sta.x;
	    	     i=pt_end.x;

	    	}
	    	for(;i<i_stop;i++)
	    	{

	    		int y=myfunc(paramA,i);
	    		if(y<=0||y>=res.cols)
	    			continue;
	    		uchar* data = res.ptr<uchar>((int)i);
	    		 data[y * 3] = 0; //第row行的第col个像素点的第一个通道值 Blue
	             data[y * 3 + 1] = 0; // Green
	             data[y * 3 + 2] = 255; // Red
	    		// std::cout<<"["<<y<<","<<i<<"]"<<std::endl;
	    	}
	    	
	    	cv::imshow("result",res);

	    };

	    void GetLocation(cv::Mat& paramA,cv::Mat* img)
	        {
	        	if(paramA.empty())
	        		return;

	        	int r0=(img->rows-1)/2;
	        	//镜头横向近似中点：(img->cols-1)/2;
	        	dist_=(img->cols-1)/2-myfunc(paramA,r0);
	        	//计算拟合直线与y=0直线夹角；
	            theta_=atan((myfunc(paramA,r0+5)-myfunc(paramA,r0-5))/10);

	        	return;

	        };

	     double myfunc(cv::Mat& paramA,double x)
		{
			if(paramA.empty())
			{
				std::cout<<"paramA has not been initialed"<<std::endl;
				return -1;
			}

			//float a0,a1,a2,a3;
			//a0=paramA.at<double>(0,0);
			//a1=paramA.at<double>(1,0);
			//a2=paramA.at<double>(2,0);
			//a3=paramA.at<double>(3,0);
			double y=paramA.at<double>(0,0)
				+paramA.at<double>(1,0)*x
				+paramA.at<double>(2,0)*x*x
				+paramA.at<double>(3,0)*x*x*x;
			return y;
		}	;

};//class myfit

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_fit_application");
  myfit mf;
  ROS_INFO("my_fit application has been init and wait for arrival of usb_cam image!!!");
 

      
	while(ros::ok())
	{
	    mf.imageCb();
       
		//cv::waitKey(30);
	}
  return 0;
}

