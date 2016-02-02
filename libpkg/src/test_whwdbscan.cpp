#include <iostream>
#include "whw_dbscan.hpp"

int main()
{
	cv::Mat dst;
	
//	cv::VideoCapture cap(0);
	cv::Mat frame;
	frame=cv::imread("test.jpg");
	cv::waitKey(1000);
	cv::namedWindow("src",CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );

	cv::namedWindow("dst",CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );	
	cv::imshow("src",frame);
	dst.create(frame.rows,frame.cols,CV_8UC3);
	std::vector<cv::Point> ptset;
	whw::processImg(frame,ptset);
	whw::WhwDbscan algr(ptset.begin(),ptset.end());
	algr.getResult(16,100,dst);
	cv::imshow("dst",dst);
	/*if(cap.isOpened())
	{
		int push_key=0;
		cap>>frame;
		dst.create(frame.rows,frame.cols,CV_8UC3);
		while(cap.read(frame))
		{
			if(frame.empty())
				continue;
			if(push_key==27)
				break;
			std::vector<cv::Point> ptset;
			whw::processImg(frame,ptset);
			whw::WhwDbscan algr(ptset.begin(),ptset.end());
			algr.getResult(10,20,dst);

			cv::imshow("src",frame);
			cv::imshow("dst",dst);
			push_key=cv::waitKey(30);			
		}

	}*/
	cv::waitKey();
	system("pause");
}