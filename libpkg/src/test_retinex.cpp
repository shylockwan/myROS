#include <iostream>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include "dlRetinex.h"
#include<time.h>




int main()
{
	cv::namedWindow("Retinx",CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );
	cv::namedWindow("src",CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );
	double* w=new double[3];
	w[0]=0.3333;w[1]=0.3333;w[2]=0.3333;
	double * sigma=new double[3];
	sigma[0]=15;sigma[1]=60;;sigma[2]=120;
	cv::Mat src=cv::imread("5.jpg");
	/////////////////////////////////
	clock_t start_time=clock();
	cv::Mat temp=src.clone();
	IplImage* img_=&IplImage(temp);
	MultiScaleRetinexCR(img_,3,w,sigma);
	cv::Mat dst(img_);
	clock_t end_time=clock();
	////////////////////////////////////////
	cv::imshow("src",src);
	cv::imshow("Retinx",dst);
	std::cout<< "Running time is: "<<static_cast<double>(end_time-start_time)/CLOCKS_PER_SEC*1000<<"ms"<<std::endl;//输出运行时间
	cv::waitKey();
	system("pause");
}