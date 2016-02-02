//ͷ�ļ�������һ����ΪMap����
//һ��������������ĺ���intcat()
//�����ݽṹ���ֱ���Ϊ��qrmsg��mapmsg��qrcoormsg
//==============================================================
#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/nonfree/features2d.hpp>
//#include <opencv2/features2d/features2d.hpp>
#include<iostream>
#include<fstream>
#include <stdio.h>
#include<vector>
#include<math.h>
using namespace cv;
using namespace std;
struct qrmsg{
	int fid;//��һ����ά���ID
	int sid;//�ڶ�����ά���ID
	float distance;//������ά��ļ�ľ���
	int lineid;//��·��ID
	float angle;//�ӵ�һ����ά�뵽�ڶ�����ά���ת��
};
struct mapmsg{
	float x1,y1;//ʸ����ͼ��ÿ��ʸ������ʵ����ֹ�����
	float x2,y2;
};
struct qrcoormsg{
	int index;
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
	int fid;
	int sid;
	float distance;
	float x11,y11,x22,y22;
	int rencoder1,rencoder2,lencoder1,lencoder2;
	float angle1,angle2;
	int alllineid;//·�����
	int qrindex;//��ά����
public:
	Map(string name1="/home/baofei/catkin_ws/map",string name2="/home/baofei/catkin_ws/qr",string name3="/home/baofei/catkin_ws/qrcoordinate");
	void addqr(string qrdesc,int lencoder ,int rencoder,float angle,vector<float> a);	
	void addmap(float x,float y);	//��¼��ͼ�����Ϣ,��������Ϊ�����˵�ǰ�����
	void qrcoordiate(string id,float x,float y,int z);//�Ѷ�ά�뼰��Ӧ����걣����һ���ļ���
	int string2index(string id);
	string index2string(int index);
	void keepdate();//������ݵ��ļ�	
	qrmsg qrto(int id1,int id2);//���������ά���ѯ·�����Ƕȵ���Ϣ	
	qrmsg lineto(int id);//���·����Ϣ��ѯ������Ϣ		
	qrcoormsg qrtocoor(string id);//��ݶ�ά��id��ѯ������
	vector<qrmsg> reqrmsg();
	vector<mapmsg> remapmsg();
	vector<qrcoormsg> reqrcoormsg();
	void reset();
};
