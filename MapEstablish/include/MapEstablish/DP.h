#ifndef _USEFUL_H_     //���û�ж��������
#define _USEFUL_H_     //���������                        
#include <iostream>
#include <vector>
#include <string>
#include <stack>
#include <fstream>
#include <iomanip>
#include <queue>
using namespace std;

extern  int prev1;  //��¼��һ�α����Ķ�ά���ID
extern  int vline[100];  //�����ά��ʣ��δ����·�������±�Ϊ���ڱ����Ķ�ά���ID��ÿ����һ�Σ���Ӧ������ֵ��1��ֱ�������ֵΪ0����ȫ��������
extern  int nextid;  //��һ��Ҫ���ʵĶ�ά���ID 
struct pathline   //����δ����֮ǰ��·����Ϣ����ά���ID��·������
{
	int id;   //��ά���ID
	string string_id;    
	float x;
	float y;
	int ptot;  //·������
};

//����ṹ�����ļ�����Ҫ������
struct path
{
	int p,q;      //��¼�ڵ��   
	int lineid;         //����·����
    float ag,length; //a���������ڵ�ĽǶ�,len����ڵ��ľ���
	bool re;       //��Ҫԭ·����ʱֻ�践��һ����־
	bool stop;
};


void Dijkstra(int , int , float *, int *, float c[100][100]);
vector<path> searchPath(int *,int , int );
vector<path> rtpath(int ,int );
#endif
