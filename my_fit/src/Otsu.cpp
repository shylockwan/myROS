#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
using namespace cv;
int Otsu(cv::Mat in,cv::Mat& mask)
{  
	if(in.empty())
		return -1;
	if (mask.empty())
	{
		mask.create(in.size(),CV_8UC1);
		mask.setTo(255);

	}

	IplImage* src=&in.operator IplImage();
	int height=src->height;  
	int width=src->width;      
	//histogram  
	int sz_mask=0;
	float histogram[256] = {0};  
	for(int i=0; i < height; i++)
	{  
		unsigned char* p=(unsigned char*)src->imageData + src->widthStep * i;  
		for(int j = 0; j < width; j++) 
		{  
			if(mask.at<uchar>(i,j)==0)
			{
				sz_mask++;
				continue;
			}
			histogram[*p++]++;  
		}  
	}  
	//normalize histogram  
	int size = height * width;
	size-=sz_mask;
	for(int i = 0; i < 256; i++)
	{  
		histogram[i] = histogram[i] / size;  
	}  

	//average pixel value  
	float avgValue=0;  
	for(int i=0; i < 256; i++)
	{  
		avgValue += i * histogram[i];  //整幅图像的平均灰度
	}   

	int threshold;    
	float maxVariance=0;  
	float w = 0, u = 0;  
	for(int i = 0; i < 256; i++) 
	{  
		w += histogram[i];  //假设当前灰度i为阈值, 0~i 灰度的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像的比例
		u += i * histogram[i];  // 灰度i 之前的像素(0~i)的平均灰度值： 前景像素的平均灰度值

		float t = avgValue * w - u;  
		float variance = t * t / (w * (1 - w) );  
		if(variance > maxVariance) 
		{  
			maxVariance = variance;  
			threshold = i;  
		}  
	}  

	return threshold;  
} 