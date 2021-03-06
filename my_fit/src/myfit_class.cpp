#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "my_fit/control_param.h"

static const std::string OPENCV_WINDOW = "Image window";

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
		      image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &myfit::imageCb, this);
		      pub_ = nh_.advertise<my_fit::control_param>("/myfit/control_param", 10);
		      cv::namedWindow(OPENCV_WINDOW,CV_WINDOW_KEEPRATIO|CV_WINDOW_NORMAL);
		      cv::namedWindow("result",CV_WINDOW_KEEPRATIO|CV_WINDOW_NORMAL);
	  }
	  void imageCb(const sensor_msgs::ImageConstPtr& msg)
	  {
		  cv_bridge::CvImagePtr cv_ptr;
		     try
		     {
		       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		     }
		     catch (cv_bridge::Exception& e)
		     {
		       ROS_ERROR("cv_bridge exception: %s", e.what());
		       return;
		     }
                     cv::Mat src(cv_ptr->image);
		     // processing on the video stream
		    cv::Mat mask;
		     int r=src.rows;
		     int c=src.cols;
		     
		    
#if 1////RGB转HSV
			cv::Mat hsv;
			cvtColor(src,hsv,CV_BGR2HSV_FULL);
			std::vector<cv::Mat> hsv_planes;
			split(hsv,hsv_planes);
			cv::Mat src_h=hsv_planes[0];
			
	int thre=Otsu(src_h,mask);
	ROS_INFO("the threshold of otsu is： %d  \n",thre);
	cv::threshold(src_h,src_h,thre,255,cv::THRESH_BINARY);
	cv::medianBlur(src_h,src_h,5);
	cv::imshow("mask",src_h);
#endif
		  
		    // cv::morphologyEx(src_h,src_h,cv::MORPH_OPEN,kernel_,cv::Point(-1,-1),1);
		     cv::morphologyEx(src_h,src_h,cv::MORPH_CLOSE,kernel_,cv::Point(-1,-1),3);
		     
		     cv::cvtColor(src_h,edge,CV_GRAY2BGR);
		     
		     ptset_.clear();
		     this->GetPointset(src_h,ptset_);
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
		      ROS_INFO("w   : %f",w);
		     cv::waitKey(60);
		     output_msg.dist=dist_;
		     output_msg.theta=theta_;
		     pub_.publish(output_msg);
	  };//imageCb

	
	  void thinImage(cv::Mat& in,cv::Mat& out,int maxIterations)
	    {
	    	using namespace cv;
	        using namespace std;
	    	//IplImage* src= cvCreateImage(cvSize(in.cols,in.rows),IPL_DEPTH_8U,in.channels());
	    	//IplImage* dst=cvCreateImage(cvSize(out.cols,in.rows),IPL_DEPTH_8U,out.channels());;

	    	IplImage tempdst=out;IplImage tempsrc=in;
	    	IplImage* dst=&tempdst;IplImage* src=&tempsrc;
	    	//src->imageData=(char*)in.data;
	    	//dst->imageData=(char*)out.data;
	    	CvSize size = cvGetSize(src);
	    	cvCopy(src,dst);//将src中的内容拷贝到dst中
	    	//cvThreshold(dst,dst,0,1,cv::THRESH_BINARY);//转换为二值图像
	    	int count = 0;	//记录迭代次数
	    	while (true)
	    	{
	    		count++;
	    		if(maxIterations!=-1 && count > maxIterations) //限制次数并且迭代次数到达
	    			break;
	    		//std::cout << count << ' ';输出迭代次数
	    		vector<pair<int,int> > mFlag; //用于标记需要删除的点
	    		//对点标记
	    		for (int i=0; i<size.height; ++i)
	    		{
	    			for (int j=0; j<size.width; ++j)
	    			{
	    				//如果满足四个条件，进行标记
	    				//  p9 p2 p3
	    				//  p8 p1 p4
	    				//  p7 p6 p5
	    				int p1 = CV_IMAGE_ELEM(dst,uchar,i,j);
	    				int p2 = (i==0)?0:CV_IMAGE_ELEM(dst,uchar,i-1,j);
	    				int p3 = (i==0 || j==size.width-1)?0:CV_IMAGE_ELEM(dst,uchar,i-1,j+1);
	    				int p4 = (j==size.width-1)?0:CV_IMAGE_ELEM(dst,uchar,i,j+1);
	    				int p5 = (i==size.height-1 || j==size.width-1)?0:CV_IMAGE_ELEM(dst,uchar,i+1,j+1);
	    				int p6 = (i==size.height-1)?0:CV_IMAGE_ELEM(dst,uchar,i+1,j);
	    				int p7 = (i==size.height-1 || j==0)?0:CV_IMAGE_ELEM(dst,uchar,i+1,j-1);
	    				int p8 = (j==0)?0:CV_IMAGE_ELEM(dst,uchar,i,j-1);
	    				int p9 = (i==0 || j==0)?0:CV_IMAGE_ELEM(dst,uchar,i-1,j-1);

	    				if ((p2+p3+p4+p5+p6+p7+p8+p9)>=2 && (p2+p3+p4+p5+p6+p7+p8+p9)<=6)
	    				{
	    					int ap=0;
	    					if (p2==0 && p3==1) ++ap;
	    					if (p3==0 && p4==1) ++ap;
	    					if (p4==0 && p5==1) ++ap;
	    					if (p5==0 && p6==1) ++ap;
	    					if (p6==0 && p7==1) ++ap;
	    					if (p7==0 && p8==1) ++ap;
	    					if (p8==0 && p9==1) ++ap;
	    					if (p9==0 && p2==1) ++ap;

	    					if (ap==1)
	    					{
	    						if (p2*p4*p6==0)
	    						{
	    							if (p4*p6*p8==0)
	    							{
	    								//标记
	    								mFlag.push_back(make_pair(i,j));
	    							}
	    						}
	    					}
	    				}
	    			}
	    		}

	    		//将标记的点删除
	    		for (vector<pair<int,int> >::iterator i=mFlag.begin(); i!=mFlag.end(); ++i)
	    		{
	    			CV_IMAGE_ELEM(dst,uchar,i->first,i->second) = 0;
	    		}

	    		//直到没有点满足，算法结束
	    		if (mFlag.size()==0)
	    		{
	    			break;
	    		}
	    		else
	    		{
	    			mFlag.clear();//将mFlag清空
	    		}

	    		//对点标记
	    		for (int i=0; i<size.height; ++i)
	    		{
	    			for (int j=0; j<size.width; ++j)
	    			{
	    				//如果满足四个条件，进行标记
	    				//  p9 p2 p3
	    				//  p8 p1 p4
	    				//  p7 p6 p5
	    				int p1 = CV_IMAGE_ELEM(dst,uchar,i,j);
	    				if(p1!=1) continue;
	    				int p2 = (i==0)?0:CV_IMAGE_ELEM(dst,uchar,i-1,j);
	    				int p3 = (i==0 || j==size.width-1)?0:CV_IMAGE_ELEM(dst,uchar,i-1,j+1);
	    				int p4 = (j==size.width-1)?0:CV_IMAGE_ELEM(dst,uchar,i,j+1);
	    				int p5 = (i==size.height-1 || j==size.width-1)?0:CV_IMAGE_ELEM(dst,uchar,i+1,j+1);
	    				int p6 = (i==size.height-1)?0:CV_IMAGE_ELEM(dst,uchar,i+1,j);
	    				int p7 = (i==size.height-1 || j==0)?0:CV_IMAGE_ELEM(dst,uchar,i+1,j-1);
	    				int p8 = (j==0)?0:CV_IMAGE_ELEM(dst,uchar,i,j-1);
	    				int p9 = (i==0 || j==0)?0:CV_IMAGE_ELEM(dst,uchar,i-1,j-1);

	    				if ((p2+p3+p4+p5+p6+p7+p8+p9)>=2 && (p2+p3+p4+p5+p6+p7+p8+p9)<=6)
	    				{
	    					int ap=0;
	    					if (p2==0 && p3==1) ++ap;
	    					if (p3==0 && p4==1) ++ap;
	    					if (p4==0 && p5==1) ++ap;
	    					if (p5==0 && p6==1) ++ap;
	    					if (p6==0 && p7==1) ++ap;
	    					if (p7==0 && p8==1) ++ap;
	    					if (p8==0 && p9==1) ++ap;
	    					if (p9==0 && p2==1) ++ap;

	    					if (ap==1)
	    					{
	    						if (p2*p4*p8==0)
	    						{
	    							if (p2*p6*p8==0)
	    							{
	    								//标记
	    								mFlag.push_back(make_pair(i,j));
	    							}
	    						}
	    					}
	    				}
	    			}
	    		}
	    		//删除
	    		for (vector<pair<int,int> >::iterator i=mFlag.begin(); i!=mFlag.end(); ++i)
	    		{
	    			CV_IMAGE_ELEM(dst,uchar,i->first,i->second) = 0;
	    		}

	    		//直到没有点满足，算法结束
	    		if (mFlag.size()==0)
	    		{
	    			break;
	    		}
	    		else
	    		{
	    			mFlag.clear();//将mFlag清空
	    		}
	    	}
	    	cv::Mat tempMat(dst);
	    	tempMat.copyTo(out);
	    };

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
			std::cout<<temp.rows<<"  "<<temp.cols<<std::endl<<F.rows<<" "<<F.cols;
			paramA=temp*F*Y;//a0 a1 a2 a3;opencv中貌似已经包含cv::solve函数用于求解

		}//LeastsquareFit;

	    void DrawLine(cv::Mat& res,cv::Mat& paramA,cv::Point2f& pt_sta,cv::Point2f& pt_end)
	    {
	    	//std::cout<<pt_sta.x<<std::endl;
	    	//std::cout<<pt_end.x<<std::endl;
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
  ros::spin();
  return 0;
}

