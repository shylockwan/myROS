#include <cv.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <iostream>
cv::Mat edge,res;
void fit_Ransac(std::vector<cv::Point2f> pt_set,float prob,cv::Mat & paramA,int max_iter,int dist_thresh);
void CallbackFocus(int events,int x,int y,int flags,void* param);
void LeastsquareFit(std::vector<cv::Point2f>& pt_set,cv::Mat& A);
void thinImage(cv::Mat& in,cv::Mat& out,int maxIterations);
void GetPointset(cv::Mat& in,std::vector<cv::Point2f>& Pt_set);
void DrawLine(cv::Mat& res,cv::Mat& paramA,cv::Point2f pt_sta,cv::Point2f pt_end);
void ColorThresh(cv::Mat& in,cv::Mat& out);
void GetLocation(cv::Mat& paramA,cv::Vec2f& location,cv::Mat* img);
int Otsu(cv::Mat in,cv::Mat& mask);
#if 1
int __main (){

	cv::namedWindow("src",CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );
	cv::namedWindow("hist",CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );
	cv::namedWindow("thin",CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );
	cv::namedWindow("hsv",CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );
	cv::namedWindow("zoom",CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );
	cv::VideoCapture cap;
	cap.open("out.avi");
	cv::Mat src;
	if (!cap.isOpened())
		return 0;
	int counts=0;
	while(1)
	{
		cap>>src;
		
		if(src.empty())
			return -1;
		cv::imshow("src",src);
			

#if 1	//对源图像src进行缩放	
			int m_scale_rate=1;
			//std::cout<<src.rows<<"  "<<src.cols<<std::endl;
			while(src.cols>1600)
			{
				pyrDown(src,src);
				m_scale_rate*=2;
			}
#endif

			cv::Mat mask;
			ColorThresh(src,mask);
			int m_block_sz=5;
			int r=src.rows;
			int c=src.cols;	
			cv::Mat gray;
			cvtColor(src,gray,CV_BGR2GRAY);
			

#if 1////RGB转HSV
			cv::Mat hsv;
			cvtColor(src,hsv,CV_BGR2HSV_FULL);
			std::vector<cv::Mat> hsv_planes;
			split(hsv,hsv_planes);
			cv::Mat src_h=hsv_planes[0];
			
			cv::Mat hist;
			int hbins = 30;
			int channels[]={0};
			float hrange[]={0,255};
			int histsize[]={hbins};
			const float* ranges[] = {hrange};
			cv::calcHist(&src_h,1,channels,cv::Mat(),hist,1,histsize,ranges);
			  int scale = 10;
   cv:: Mat histImg = cv::Mat::zeros(256,hbins*scale,CV_8UC3);
   double maxVal=0;
    minMaxLoc(hist, 0, &maxVal, 0, 0);
    for( int h = 0; h < hbins; h++ )
       { 
		   float binVal = hist.at<float>(h,0);
            int intensity = cvRound(binVal*255/maxVal);
			cv::Rect rect(h*scale,0,scale-1,intensity);
            rectangle( histImg,rect,cv::Scalar(255,0,0),CV_FILLED);
       }
	 cv::flip(histImg, histImg, 0);
	cv::imshow("hist",histImg);
	int thre=Otsu(src_h,cv::Mat());
	//std::cout<<"分割阈值为： "<<thre<<std::endl;
	cv::threshold(src_h,src_h,thre,255,cv::THRESH_BINARY);
	cv::imshow("hsv",src_h);
#endif
#if 0 //adaptive median filter自适应分块中值滤波
			for(int i=0;i<=r-m_block_sz;i+=m_block_sz)
			{
				for(int j=0;j<=c-m_block_sz;j+=m_block_sz)
				{
					cv::Rect rect(j,i,m_block_sz,m_block_sz);
					cv::Mat block_rect;
					block_rect=gray(rect);//B
					cv::Scalar avg,dev;
					cv::meanStdDev(block_rect,avg,dev);
					//std::cout<<avg<<std::endl<<dev<<std::endl;
					int center_val=block_rect.at<uchar>(3,3);
					if(center_val>avg(1)+1.8*dev(1)||center_val<avg(1)-1.8*dev(1))
					{
						cv::medianBlur(block_rect,block_rect,m_block_sz);
					}

				}//for2 loop
			}//for1 loop
#endif
			cv::imshow("mask",mask);
			gray=gray.mul(mask/255);//multiply mask
			cv::Mat_<uchar> kernel=(cv::Mat_<uchar>(3,3)<<0,1,0,1,1,1,0,1,0);
			cv::morphologyEx(gray,gray,cv::MORPH_OPEN,kernel,cv::Point(-1,-1),1);
			cv::morphologyEx(gray,gray,cv::MORPH_CLOSE,kernel,cv::Point(-1,-1),3);
			//cv::imshow("af_median",gray);
			res=gray.clone();

#if 0
			//cv::threshold(gray,gray,0,1,cv::THRESH_BINARY);
			//edge.create(gray.rows,gray.cols,gray.type());
			//thinImage(gray,edge,100);
			//cv::threshold(edge,edge,0,255,cv::THRESH_BINARY);
			//Canny(gray,edge,30,120);

			//cv::setMouseCallback("canny",CallbackFocus,0);
			//cv::imshow("thin",edge);
			std::vector<cv::Point2f> pt_set;
			///
			///去掉孤立点
			///
			///
			GetPointset(gray,pt_set);
			cv::Mat ParamA;
			LeastsquareFit(pt_set,ParamA);
			//fit_Ransac(pt_set,0,ParamA,50,10);
			std::cout<<ParamA;

			///标记拟合直线
			// res.create(src.rows,src.cols,CV_8UC3);
			// res.setTo(0);
			cv::cvtColor(res,res, CV_GRAY2BGR);
			DrawLine(res,ParamA,pt_set[0],pt_set[pt_set.size()-1]);
			cv::Vec2f location(0,0);
			GetLocation(ParamA,location,&res);
			std::cout<<std::endl<<"dist:"<<location[0]<<"  theta:"<<location[1];
#endif
		
			cv::waitKey(10);
		

	}

}

#endif
