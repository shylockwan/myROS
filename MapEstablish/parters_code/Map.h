//ͷ�ļ�������һ����ΪMap����
//һ���������������ĺ���intcat()
//�������ݽṹ���ֱ���Ϊ��qrmsg��mapmsg��qrcoormsg
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
	string fid;//��һ����ά���ID
	string sid;//�ڶ�����ά���ID
	float distance;//������ά��ļ�ľ���
	int lineid;//��·��ID
	float angle;//�ӵ�һ����ά�뵽�ڶ�����ά���ת��
};
struct mapmsg{
	float x1,y1;//ʸ����ͼ��ÿ��ʸ������ʵ����ֹ������
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
	void addmap(float x,float y);	//��¼��ͼ������Ϣ,��������Ϊ�����˵�ǰ������
	void qrcoordiate(string id,float x,float y,int z);//�Ѷ�ά�뼰��Ӧ�����걣����һ���ļ���
	void keepdate();//�������ݵ��ļ�	
	qrmsg qrto(string id1,string id2);//����������ά���ѯ·�����Ƕȵ���Ϣ	
	qrmsg lineto(int id);//����·����Ϣ��ѯ������Ϣ		
	qrcoormsg qrtocoor(string id);//���ݶ�ά��id��ѯ��������
	vector<qrmsg> reqrmsg();
	vector<mapmsg> remapmsg();
	vector<qrcoormsg> reqrcoormsg();
};
