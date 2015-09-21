#ifndef _USEFUL_H_     //���û�ж��������
#define _USEFUL_H_     //���������                        
#include <iostream>
#include <vector>
#include <stack>
#include <fstream>
#include <iomanip>
#include <queue>
using namespace std;

const int maxnum = 100;
int prev1=0;  //��¼��һ�α����Ķ�ά���ID
int vline[maxnum]={0};  //�����ά��ʣ��δ����·�������±�Ϊ���ڱ����Ķ�ά���ID��ÿ����һ�Σ���Ӧ������ֵ��1��ֱ�������ֵΪ0����ȫ��������
queue<int> unvisited;  //visited�������δ������ά��ID
vector<float> angle;  //�����ά��·���Ƕ�
vector<vector<float> > anglelist; //����ÿ����ά��·���ĽǶȵ�����������
vector<float> outangle;
int nextid=0;  //��һ��Ҫ���ʵĶ�ά���ID
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
};


const int maxint = 999999; 
// �����鶼���±�1��ʼ
float dist[maxnum];     // ��ʾ��ǰ�㵽Դ������·������
float c[maxnum][maxnum];   // ��¼ͼ�������·������
void Dijkstra(int , int , float *, int *, float c[maxnum][maxnum]);
vector<path> searchPath(int *,int , int );
vector<path> rtpath(int ,int );
#endif
