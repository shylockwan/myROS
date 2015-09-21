#ifndef _USEFUL_H_     //如果没有定义这个宏
#define _USEFUL_H_     //定义这个宏                        
#include <iostream>
#include <vector>
#include <stack>
#include <fstream>
#include <iomanip>
#include <queue>
using namespace std;

const int maxnum = 100;
int prev1=0;  //记录上一次遍历的二维码的ID
int vline[maxnum]={0};  //保存二维码剩余未遍历路径数，下标为正在遍历的二维码的ID，每遍历一次，对应的数组值减1，直到数组的值为0，即全部遍历完
queue<int> unvisited;  //visited里面存入未遍历二维码ID
vector<float> angle;  //保存二维码路径角度
vector<vector<float> > anglelist; //保存每个二维码路径的角度的向量的向量
vector<float> outangle;
int nextid=0;  //下一个要访问的二维码的ID
struct pathline   //保存未遍历之前的路径信息：二维码的ID，路径的数
{
	int id;   //二维码的ID
	string string_id;    
	float x;
	float y;
	int ptot;  //路径的数
};

//定义结构保存文件中需要的数据
struct path
{
	int p,q;      //记录节点号   
	int lineid;         //保存路径号
    float ag,length; //a保存两个节点的角度,len保存节点间的距离
	bool re;       //需要原路返回时只需返回一个标志
};


const int maxint = 999999; 
// 各数组都从下标1开始
float dist[maxnum];     // 表示当前点到源点的最短路径长度
float c[maxnum][maxnum];   // 记录图的两点间路径长度
void Dijkstra(int , int , float *, int *, float c[maxnum][maxnum]);
vector<path> searchPath(int *,int , int );
vector<path> rtpath(int ,int );
#endif
