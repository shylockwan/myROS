//==============================================================================
//函数名： colorThresh
//作者：王华威
//日期:  2015
//功能: 颜色滤波，得到目标颜色连通域二值图像
//输入: cv::Mat& in, cv::Mat& mask, int high_threshold, int low_threshold, bool flag_reverse=0
//输出: cv::Mat out
//修改记录：
//==============================================================================

#include <opencv2/opencv.hpp>
#include <iostream>
	bool colorThresh(cv::Mat& in, cv::Mat& out, cv::Mat& mask, int high_threshold, int low_threshold, bool flag_reverse) {
		if (in.empty() || in.channels() != 1)
			return -1;

                //Hellen modified 1215
		int Threshold = 50;//for default blue filter
		if(flag_reverse == 1) 
			Threshold = 80; //for red filter  Hellen modified 1128

		if(low_threshold>high_threshold)
		{
		   int temp_=high_threshold;
		   high_threshold=low_threshold;
		   low_threshold=temp_;
		}
		if(low_threshold<0||high_threshold>255)
		{
			return 0;
			std::cout<<"threshold out of range [0 ~255]"<<std::endl;
		}
		if (out.empty())
				out.create(in.rows, in.cols, CV_8UC1);
	for (int i = 0; i < in.rows; i++) {
		uchar* ptr_in = in.ptr<uchar>((int) i);
		uchar* ptr_mk = mask.ptr<uchar>((int) i);
		uchar* ptr_out = out.ptr<uchar>(i);
		for (int j = 0; j < in.cols; j++) {
			if (ptr_mk[j] < Threshold) {
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
	return 1;
	}	;			//ColorThresh


