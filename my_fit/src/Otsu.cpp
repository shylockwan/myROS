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
		avgValue += i * histogram[i];  //����ͼ���ƽ���Ҷ�
	}   

	int threshold;    
	float maxVariance=0;  
	float w = 0, u = 0;  
	for(int i = 0; i < 256; i++) 
	{  
		w += histogram[i];  //���赱ǰ�Ҷ�iΪ��ֵ, 0~i �Ҷȵ�����(��������ֵ�ڴ˷�Χ�����ؽ���ǰ������) ��ռ����ͼ��ı���
		u += i * histogram[i];  // �Ҷ�i ֮ǰ������(0~i)��ƽ���Ҷ�ֵ�� ǰ�����ص�ƽ���Ҷ�ֵ

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