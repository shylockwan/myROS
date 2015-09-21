#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
using namespace cv;
bool colorThresh(cv::Mat& in, cv::Mat& out, cv::Mat& mask, int high_threshold, int low_threshold, bool flag_reverse);
void detectContours(cv::Mat img, vector<vector<cv::Point> >& contours, vector<cv::Vec4i>& hierarchy);
vector<vector<cv::Point> > contoursDetect(cv::Mat im);
bool redOrBlueContoursDetect(const cv::Mat& im, cv::Point& location, bool flag);
