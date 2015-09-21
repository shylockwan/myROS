#include "contoursDetect.h"

/*Hellen 下面这个函数在两个文件中出现，联编会有问题*/
bool colorThresh(cv::Mat& in, cv::Mat& out, cv::Mat& mask, int high_threshold, int low_threshold, bool flag_reverse=0) {
	if (in.empty() || in.channels() != 1)
		return -1;

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
	return 1;
};//ColorThresh

/*HELLEN 这个文件的功能是什么？*/
void detectContours(cv::Mat img, vector<vector<cv::Point> >& contours, vector<Vec4i>& hierarchy) {
	//erode and dilate
	cv::erode( img, img, cv::Mat(), cv::Point(-1,-1), 6);
	cv::dilate(img, img, cv::Mat(), cv::Point(-1,-1), 6);
	//imshow("result1", img);
    findContours( img, contours, hierarchy,
        CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
	unsigned int Cmin = 10;
	vector< vector< cv::Point> >::iterator itc = contours.begin();
	while(itc != contours.end()) {//eliminate small contour
		if(itc->size() < Cmin) 
			itc = contours.erase(itc);
		else
			++itc;
	}
}//detectContours

//==============================================================================
//函数名： contoursDetect
//作者：louisWei
//日期:  2015-7-13
//功能: 检测连通域，路径分类返回
//输入: cv::Mat
//输出: vector<vector< cv::Point> > 
//修改记录：
//==============================================================================
vector<vector< cv::Point> > contoursDetect(cv::Mat im) {
	cv::Mat hsv,out;
	vector<cv::Mat> HSV;
	cvtColor(im, hsv, CV_BGR2HSV_FULL);
	split(hsv, HSV);
	colorThresh(HSV[0], out, HSV[1], 15, 240, 1);//red
	vector<vector<cv::Point> >  contours;
	vector<cv::Vec4i> hierarchy;
	detectContours(out, contours, hierarchy);

	vector<vector< cv::Point> > subContourPoints;
	vector<Mat> subContours;
	if(contours.size() > 0) {
		int idx = 0;
    	for( ; idx >= 0; idx = hierarchy[idx][0] ) {
			cv::Mat img = cv::Mat::zeros(im.rows, im.cols, CV_8U);
        	drawContours( img, contours, idx, cv::Scalar(255), CV_FILLED, 8, hierarchy );
		
			vector<cv::Point> contoursPoint;
			for(int nr=0; nr<im.rows; ++nr) {
				uchar* data = img.ptr<uchar>(nr);//row first addr
				for(int nc=0; nc<im.cols; ++nc) {
					if(data[nc] == 255)
						contoursPoint.push_back(Point(nc, nr));
				}
			}
			subContourPoints.push_back(contoursPoint);
    	}
	}//else cout<<"no contours!!!"<<endl;
	
	return subContourPoints;
}//contoursDetect

//==============================================================================
//函数名： redOrBlueContoursDetect
//作者：louisWei
//日期:  2015-7-13
//功能: 检测红色或者是蓝色连通域，即代表的是路径或二维码
//输入: cv::Mat， flag=false表示蓝色（默认），flag=true表示红色
//输出: bool true表示检测到路径 
//修改记录：
//==============================================================================
bool redOrBlueContoursDetect(const cv::Mat& im, cv::Point& location, bool flag=false) {
	cv::Mat hsv,out;
	vector<cv::Mat> HSV;
	cvtColor(im, hsv, CV_BGR2HSV_FULL);
	split(hsv, HSV);
	if(flag == true) 
		colorThresh(HSV[0], out, HSV[1], 15, 240, 1);//red
	else
		colorThresh(HSV[0], out, HSV[1], 145, 175, 0);//blue

	vector<vector<cv::Point> >  contoursB;
	vector<cv::Vec4i> hierarchyB;
	detectContours(out, contoursB, hierarchyB);
	if(contoursB.size() == 0) 
		return false;
	else {
		int x = 0;
		int y = 0;
		for(int i=0; i<contoursB[0].size(); ++i) {
			x += contoursB[0][i].x;
			y += contoursB[0][i].y;
		}
		x /= contoursB[0].size();
		y /= contoursB[0].size();
		location = cv::Point(x, y);
		return true;
	}
}//redOrBlueContoursDetect
