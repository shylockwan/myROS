#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define CHUIZHI  0.262 //摄影机垂直视野角一半；单位为rad
#define  SHUIPING  0.34   	// beta0=0.35;//m_HorShiYeAng;摄影机水平视野角一半
#define HEIGHT  32  //摄像机height 单位cm
#define DELTA_Y  20 //cm
static const std::string OPENCV_WINDOW = "Image window";
class myCalibra
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  float fy_angle;
  myCalibra()
    : it_(nh_),fy_angle(0.0)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &myCalibra::imageCb, this);
   // image_pub_ = it_.advertise("/image_converter/output_video", 1);
    cv::namedWindow(OPENCV_WINDOW,CV_WINDOW_KEEPRATIO|CV_WINDOW_NORMAL);
    cv::namedWindow("mask",CV_WINDOW_KEEPRATIO|CV_WINDOW_NORMAL);
  }

  ~myCalibra()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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
	int r=src.rows;
	int c=src.cols;
	cv::Mat dst;
#if 1////RGB转HSV
	cv::Mat hsv;

	cv::cvtColor(src,hsv,CV_BGR2HSV_FULL);
	std::vector<cv::Mat> hsv_planes;
	split(hsv,hsv_planes);
	cv::Mat* src_h=new cv::Mat(hsv_planes[0]);
	cv::Mat src_s=hsv_planes[1];
	cv::GaussianBlur(*src_h,*src_h,cv::Size(7,7),1);
	this->ColorThresh(*src_h,*src_h,src_s);
	cv::medianBlur(*src_h,*src_h,15);
	cv::imshow("mask",*src_h);
	float fy_angle=this->FuyangCalibra(*src_h);

#endif
    // Draw an example circle on the video stream
    int y0=(r+1)/2;
    int x0=(c+1)/2;
    if (r > 200 && c > 200)
      {
      cv::circle(src, cv::Point(x0, y0), 5, CV_RGB(255,0,0));
     // cv::rectangle(cv_ptr->image,cv::Rect(x0-60,y0-60,120,120),cv::Scalar(255,255,0),3);
       cv::rectangle(src,cv::Rect(x0-100,y0-100,200,200),cv::Scalar(255,0,0),3);
       cv::line(src,cv::Point(x0,y0-200),cv::Point(x0,y0+200),CV_RGB(255,0,0),3);
       cv::line(src,cv::Point(x0-200,y0),cv::Point(x0+200,y0),CV_RGB(255,0,0),3);
      }


    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, src);
    cv::waitKey(30);

    // Output modified video stream
 //   image_pub_.publish(cv_ptr->toImageMsg());
  }

	void ColorThresh(cv::Mat& in,cv::Mat& out,cv::Mat& mask)
	{
		if(in.empty()||in.channels()!=1)
			return;

		if(out.empty())
			out.create(in.rows,in.cols,CV_8UC1);

		for(int i=0;i<in.rows;i++)
		{
			uchar* ptr_in = in.ptr<uchar>((int)i);
			uchar* ptr_mk = mask.ptr<uchar>((int)i);
			uchar* ptr_out=out.ptr<uchar>(i);
			for(int j=0;j<in.cols;j++)
			{
				if(ptr_in[j]>15&&ptr_in[j]<240||ptr_mk[j]<80)//Green&Blue channel
					ptr_out[j]=0;
				else
					ptr_out[j]=255;
			}
		}//for1 loop
	};//ColorThresh

	float FuyangCalibra(const cv::Mat& in)
		{
			if(in.empty()||in.channels()!=1)
				return 0;
			float r=in.rows;
			float c=in.cols;
			float start_y=c-1;
			float end_y=1;;
			for(int i=0;i<r;i++)
			{
				const uchar* dataI=in.ptr<uchar>(i);
				for(int j=0;j<c;j++)
				{
					if(dataI[j]==0)
						continue;
					else
					{
						if(start_y>i)
						{
							start_y=i;
						}
						if(end_y<i)
						{
							end_y=i;
						}
					}//if

				}//for2 loop
			}//for1 loop

			start_y=(r-1)/2- start_y;
			end_y=(r-1)/2- end_y;
			float A=2*start_y*tan(CHUIZHI)/r;
			float B=2*end_y*tan(CHUIZHI)/r;
			float W=(DELTA_Y/HEIGHT+(1/A)-(1/B))*A*B;
			ROS_INFO("A=  %f  ,B= %f   ",A,B);
			float _a=W*A*B;
			float _b=B*B-W*A-W*B-A*A;
			float _c=W-A*A*B+A*B*B+A-B;
			float x1=(-_b+sqrt(_b*_b-4*_a*_c))/(2*_a);
			float x2=(-_b-sqrt(_b*_b-4*_a*_c))/(2*_a);
			float arcx=atan(x1)*180/CV_PI;
			ROS_INFO("a=  %f  ,b= %f   ,c=   %f,,W=   %f ",_a,_b,_c,W);
			ROS_INFO("start_y=  %f  ,end_y= %f   ,x1=   %f,,x2=   %f ",start_y,end_y,x1,x2);
			ROS_INFO("x=   %f,arcx=   %f ",atan(x1),arcx);
			return arcx;




		};///getPointset();待优化：分类记录点或者去除孤立区域

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_calibra");

  myCalibra ic;

  ROS_INFO("ready to sub image");
  ros::Rate loop_rate(1);
  cv::waitKey(3000);
  while(ros::ok())
  {
	        ros::spinOnce();
	       
		 	//ROS_INFO("outobject____theta1: %f , theta2:   %f   dist_: %f ",mf.theta_1,mf.theta_2,mf.dist_);

	  	    loop_rate.sleep();
  }
//  ros::spin();
  return 0;
}
