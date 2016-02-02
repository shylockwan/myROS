#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "dlRetinex.h"

int main()
{
#if 1
	cv::Mat dst;
	
	cv::VideoCapture cap(0);
	cv::Mat frame;
	cv::waitKey(1000);
	
	//cv::namedWindow("removeLightEffect",CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );
	cv::Mat_<cv::Vec3d> src;
	
	if(cap.isOpened())
	{
		int push_key=0;
		cap>>frame;
		double* w=new double[3];
		w[0]=0.3333;w[1]=0.3333;w[2]=0.3333;
		double * sigma=new double[3];
		sigma[0]=15;sigma[1]=60;;sigma[2]=120;
		/*cv::Mat_<cv::Vec3b> result;
		
		result.create(cv::Size(frame.cols,frame.rows));
		dst.create(cv::Size(frame.cols,frame.rows),CV_64FC3);
		src.create(cv::Size(frame.cols,frame.rows));*/
		cv::imshow("src",frame);
		
		while(cap.read(frame))
		{
			if(frame.empty())
				continue;
			if(push_key==27)
				break;
			IplImage* img_=&IplImage(frame);
			//Retinex(img_,15);
			MultiScaleRetinexCR(img_,3,w,sigma);
			cv::Mat dst(img_);

			
			cv::imshow("Retinx",dst);
			push_key=cv::waitKey(30);
			
		}

	}
	//std::cout<<dst.row(200);
#endif
	



	cv::waitKey();
	system("pause");
	return 0;

}
