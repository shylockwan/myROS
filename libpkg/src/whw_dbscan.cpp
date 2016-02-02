#include "whw_dbscan.hpp"
#include <iostream>
namespace whw{
	double WhwDbscan::calcDistance(cv::Point& pt1,cv::Point& pt2){
		double result=0.0;
		double delta_x=abs(pt1.x - pt2.x);
		double delta_y=abs(pt1.y - pt2.y);
		//result=sqrt(delta_x*delta_x+delta_y*delta_y);
		result=std::max(delta_x,delta_y);
		return result;
	}
	void WhwDbscan::getResult(double eps, int minPts,cv::Mat& out){
		if(out.empty())
			return;
		out.setTo(0);
		int result=this->cluster(eps,minPts,pfunc_);
		std::cout<<"共聚类："<<result<<std::endl;
		int mul_=255/(result+2);
		
		
		for ( auto i = 0; i < points.size()-1; ++i )
		{
			cv::Point pt=points[i].P;
			
			if(points[i].cluster!=-1)
			{
				cv::Vec3b color(255,mul_*points[i].cluster,0);
				out.at<cv::Vec3b>(pt)=color;
			}
		}
	};
	bool colorThresh(cv::Mat& in, cv::Mat& out, cv::Mat& mask, int high_threshold, int low_threshold, bool flag_reverse) {
		if (in.empty() || in.channels() != 1)
			return 1;

		if(low_threshold>high_threshold)
		{
			int temp_=high_threshold;
			high_threshold=low_threshold;
			low_threshold=temp_;
		}
		if(low_threshold<0||high_threshold>255)
		{
			return 1;
			std::cout<<"threshold out of range [0 ~255]"<<std::endl;
		}
		if (out.empty())
			out.create(in.rows, in.cols, CV_8UC1);
		for (int i = 0; i < in.rows; i++) {
			uchar* ptr_in = in.ptr<uchar>((int) i);
			uchar* ptr_mk = mask.ptr<uchar>((int) i);
			uchar* ptr_out = out.ptr<uchar>(i);
			for (int j = 0; j < in.cols; j++) {
				if (ptr_mk[j] < 70) {
					ptr_out[j] = 0;
					continue;
				} else {
					if (ptr_in[j] > low_threshold && ptr_in[j] < high_threshold) //Green&Blue channel
						ptr_out[j] = 255 * int(!flag_reverse);
					else
						ptr_out[j] = 255 * int(flag_reverse);
				}
			} //for2 loop

		} //for1 loop
		return 0;
	}	;			//ColorThresh

	bool processImg(const cv::Mat& src,std::vector<cv::Point>& ptset){
		if (src.empty())
		{
			std::cout<<"processImg图片空"<<std::endl;
			return 1;
		}

#if 1////RGB转HSV
		cv::Mat hsv;
		cv::cvtColor(src, hsv, CV_BGR2HSV_FULL);
		std::vector<cv::Mat> hsv_planes;
		split(hsv, hsv_planes);
		cv::Mat h_ = hsv_planes[0];
		cv::Mat s_ = hsv_planes[1];
		cv::Mat h_QR, h_line;
		cv::GaussianBlur(h_, h_, cv::Size(7, 7), 1);
		colorThresh(h_, h_line, s_, 240, 15, 1);
		; //进行颜色滤波对红色，检查结果Mat h_line
		cv::imshow("temp",h_);
		getPointset(h_line,ptset);
#endif
		return 0;

	};
	void getPointset(const cv::Mat& in,std::vector<cv::Point>& ptset){
		if (in.empty() || in.channels() != 1)
		{
			std::cout<<"getPointset图片空"<<std::endl;
			return ;
		}

		ptset.clear();
		cv::Point pt;
		for (int i = 0; i < in.rows; i++) {
			const uchar* dataI = in.ptr<uchar>(i);
			for (int j = 0; j < in.cols; j++) {
				if (dataI[j] == 0)
					continue;
				else {
					pt.y = i;
					pt.x = j;
					ptset.push_back(pt);
				} //if

			} //for2 loop
		} //for1 loop
	} ///getPointset();待优化：分类记录点或者去除孤立区域
};

