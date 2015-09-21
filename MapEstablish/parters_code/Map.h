//头文件定义了一个名为Map的类
//一个连接两个整数的函数intcat()
//三个数据结构，分别名为：qrmsg，mapmsg，qrcoormsg
//==============================================================
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/nonfree/features2d.hpp>
//#include <opencv2/features2d/features2d.hpp>
#include<iostream>
#include<fstream>
#include <stdio.h>
#include<vector>
#pragma once
using namespace cv;
using namespace std;
struct qrmsg{
	string fid;//第一个二维码的ID
	string sid;//第二个二维码的ID
	float distance;//两个二维码的间的距离
	int lineid;//线路的ID
	float angle;//从第一个二维码到第二个二维码的转角
};
struct mapmsg{
	float x1,y1;//矢量地图的每个矢量的其实和终止的坐标
	float x2,y2;
};
struct qrcoormsg{
	string id;
	float x;
	float y;
	int count;
};
class Map{
private:
	string qrname;
	string mapname;
	string qrcoordinatename;
	fstream qrfile;
	fstream mapfile;
	fstream qrcoordinatefile;
	vector<qrmsg> qrlist;
	vector<mapmsg> maplist;
	vector<qrcoormsg> coorlist;
	bool qrflag;
	bool mapflag;
	string fid;
	string sid;
	float distance;
	float x11,y11,x22,y22;
	int rencoder1,rencoder2,lencoder1,lencoder2;
	float angle1,angle2;
	int alllineid;
public:
	Map(string name1="map",string name2="qr",string name3="qrcoordinate");
	void addqr(string id,int lencoder ,int rencoder,float angle,vector<float> a);	
	void addmap(float x,float y);	//记录地图坐标信息,函数输入为机器人当前的坐标
	void qrcoordiate(string id,float x,float y,int z);//把二维码及相应的坐标保存在一个文件中
	void keepdate();//保存数据到文件	
	qrmsg qrto(string id1,string id2);//根据两个二维码查询路径，角度等信息	
	qrmsg lineto(int id);//根据路径信息查询其他信息		
	qrcoormsg qrtocoor(string id);//根据二维码id查询它的坐标
	vector<qrmsg> reqrmsg();
	vector<mapmsg> remapmsg();
	vector<qrcoormsg> reqrcoormsg();
};
