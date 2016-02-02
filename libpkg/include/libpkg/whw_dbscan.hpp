#include "dbscan.h"
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <iostream>

namespace whw{
	typedef std::vector<cv::Point>::iterator iter_pt;
	class WhwDbscan:public Clusterer_DBSCAN<iter_pt>
	{
	public:
		WhwDbscan(iter_pt pt_begin,iter_pt pt_end):Clusterer_DBSCAN(pt_begin, pt_end){
			pfunc_=std::bind(&WhwDbscan::calcDistance,this,std::tr1::placeholders::_1,std::tr1::placeholders::_2);	
			
		};
		
		void getResult(double eps, int minPts,cv::Mat& out);
	private:
		double calcDistance(cv::Point& pt1,cv::Point& pt2);
		metric pfunc_;
	};

	bool colorThresh(cv::Mat& in, cv::Mat& out, cv::Mat& mask, int high_threshold, int low_threshold, bool flag_reverse=0);
    bool processImg(const cv::Mat& src,std::vector<cv::Point>& ptset);
	void getPointset(const cv::Mat& in,std::vector<cv::Point>& ptset);
};