#ifndef _USEFUL_H_     //���û�ж��������
#define _USEFUL_H_     //���������                        
#include <iostream>
#include <vector>
#include <stack>
#include <fstream>
#include <iomanip>
using namespace std;
//����ṹ�����ļ�����Ҫ������
struct path
{
	int p,q;      //��¼�ڵ��   
	int lineid;         //����·����
    float ag,length; //a���������ڵ�ĽǶ�,len����ڵ��ľ���        
};

const int maxnum = 100;
void Dijkstra(int , int , float *, int *, float c[maxnum][maxnum]);
vector<path> searchPath(int *,int , int );
vector<path> rtpath(int ,int );
#endif