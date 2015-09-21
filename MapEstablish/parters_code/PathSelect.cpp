#include <queue>
#include <string>
#include <algorithm>
#include "DP.h"
#define PI 180

int prev1=0;  //记录上一次遍历的二维码的ID
int vline[maxnum]={0};  //保存二维码剩余未遍历路径数，下标为正在遍历的二维码的ID，每遍历一次，对应的数组值减1，直到数组的值为0，即全部遍历完
queue<int> unvisited;  //visited里面存入未遍历二维码ID
vector<float> angle;  //保存二维码路径角度
vector<vector<float> > anglelist; //保存每个二维码路径的角度的向量的向量
vector<float> outangle;
int nextid=0;  //下一个要访问的二维码的ID
struct pathline   //保存未遍历之前的路径信息：二维码的ID，路径的数
{
	int id;    //二维码的ID
	float x;
	float y;
	int ptot;  //路径的数
};

//===================================================================================
//函数名：  BFS
//作者：    杨旺
//日期：    2015-8-27
//功能：    为遍历提供所需路径
//输入参数：
//          vt   源节点号
//返回值：  类型（path)
//			返回二维码出需要遍历的路径line,包含起始节点和角度值
//====================================================================================

path BFS(int vt)   
{
   pathline readnumber[maxnum]; //读取文件中二维码ID和路径数
   path readnum[maxnum];        //读取文件中二维码相关的五个信息
   path line;     //返回的结构对象
   vector<path> repathlist;  //接收最短路径的返回值
   vector<float> temporary;  //保存临时存放的角度值
   int ntot=0;   
   int aa=0,bb=0,dd=0,ee=0,ff=0,gg=0,hh=0,s=0;
   int ptotal=0;  //路径总数
   int nextid=0;  //下一个要访问的二维码的ID

	if((prev1==0))	
	{  
		unvisited.push(vt);
		ifstream icin1;     //读取保存二维码ID以及路径数目的文件
		icin1.open("qrcoordinate.dat");
		if(!icin1)
			{
				cout<<"文件读取失败"<<endl;
			}
		icin1 >> readnumber[vt].id >> readnumber[vt].x>> readnumber[vt].y>> readnumber[vt].ptot;  //刚开始时文件中只有一条数据，只需读一次
		ptotal=readnumber[vt].ptot;  
		icin1.close();

		vline[vt]=ptotal;   //二维码v未遍历的路径数

		ifstream icin2;    //读取保存路径信息的文件
		icin2.open("qr.dat");
		if(!icin2)
		 {
			cout<<"文件读取失败"<<endl;
		 }
		while(!icin2.eof())
		 {
			icin2 >> readnum[aa].p >> readnum[aa].q >>readnum[aa].length>>readnum[aa].lineid>>readnum[aa].ag;
			if(readnum[aa].p=vt)
			 {
				angle.push_back(readnum[aa].ag);  //将与二维码vt相邻路径的角度值存入到vector中
			 }
			aa++;
		 }
		sort(angle.begin(),angle.end());   //将vector里面的角度值按顺序排列
		anglelist.push_back(angle);
		icin2.close();
		
		prev1=vt;      //保存当前正在遍历的二维码ID
		outangle=anglelist.front();  //得到向量的第一个元素
		anglelist.erase(anglelist.begin());    //删除第一个元素
		line.p=vt;
		line.ag =*outangle.end();   //读取angle最后一个值
		outangle.pop_back();      //读取angle最后一个值后，删除此处的值，更新angle的值
		unvisited.pop();    //删除正在遍历的二维码
		vline[vt]=vline[vt]-1;    //二维码v未遍历的路径数
   }
   if((vt==prev1))
   {
		if(vline[vt]!=0)
         {  
			nextid=0;    //为了保证不再执行最后一个if语句
			line.p=vt;
			line.ag =*outangle.end();  //读取angle最后一个值
			outangle.pop_back();     //读取angle最后一个值后，删除此处的值，更新angle的值
			vline[vt]=vline[vt]-1;   //二维码v未遍历的路径数
		 }
		else        //说明上一个二维码已经遍历完，需要进入下一个二维码进行遍历,需要调用最短路径代码返回到需遍历的二维码
			{
				nextid=unvisited.front();
				unvisited.pop(); //删除未遍历中当前正在遍历的二维码
				outangle=anglelist.front();

				for(int kk=0;kk<=unvisited.size();kk++)    //刚开始遍历的二维码路径不会包含已经遍历过的路径，因而不需要比较是否有重复路径
				{
					ifstream icin6;    //读取保存路径信息的文件
					icin6.open("qr.dat");
					if(!icin6)
					{
						cout<<"文件读取失败"<<endl;
					}
					while(!icin6.eof())
					{
						icin6 >> readnum[hh].p >> readnum[hh].q >>readnum[hh].length>>readnum[hh].lineid>>readnum[hh].ag;
						if((readnum[hh].lineid!=0)&&(readnum[hh].p==nextid))    //表示这条路径已经走过，未走过是ID默认都是0      
						{	
							temporary.push_back(readnum[hh].ag);  //保存已经遍历过路径的角度值
						}
						hh++;
					}
					icin6.close();

					vector<float>::iterator ito = outangle.begin(); 
					vector<float>::iterator itt = temporary.begin(); 
					while (ito != outangle.end()) 
					{ 
						while (itt != temporary.end()) 
						{ 
							if (*ito == *itt)
							{
								outangle.erase(ito);//删除值为temporary的元素 
								vline[nextid]=vline[nextid]-1;//删除已经遍历过的路径
							}
							++itt;
						}
						++ito; 
					} 
					temporary.clear();

					if(vline[nextid]==0)  //如果下一将遍历的二维码没有需遍历的路径，则从unvisited中删除
					{
						nextid=unvisited.front();
						unvisited.pop(); //删除未遍历中当前正在遍历的二维码
						anglelist.erase(anglelist.begin()); 
					}
					else
					{
						prev1=nextid;
						anglelist.erase(anglelist.begin()); 
						break;
					}
				
				}
				if((unvisited.size()==0) && (vline[nextid]==0))
				{
					nextid=0;
					line.p=0; //如果unvisited里面没有未遍历的二维码，说明遍历已经完成，则返回的结构元素的值全部为0
					line.q=0;
					line.ag=0;
					line.lineid=0;
					line.length=0;
				}
			
		}
	 }
	else
     {
	    ifstream icin4;  //读取保存二维码ID以及路径数目的文件
        icin4.open("qrcoordinate.dat");
        if(!icin4)
         {
	       cout<<"文件读取失败"<<endl;
         } 
		while(!icin4.eof())
			{
				icin4 >> readnumber[bb].id >> readnumber[bb].x>> readnumber[bb].y>> readnumber[bb].ptot;
				if(readnumber[bb].id==vt)  
				 {
					s=bb;
					break;
				 }
					bb++;
			}
		icin4.close();

		vline[vt]=readnumber[s].ptot;
		unvisited.push(vt);

		ifstream icin5;    //读取保存路径信息的文件
		icin5.open("qr.dat");
		if(!icin5)
		 {
			cout<<"文件读取失败"<<endl;
		 }
		while(!icin5.eof())
		 {
			icin5 >> readnum[gg].p >> readnum[gg].q >>readnum[gg].length>>readnum[gg].lineid>>readnum[gg].ag;
			if(readnum[gg].p==vt)
			 {
				angle.push_back(readnum[gg].ag);  //将与二维码v相邻路径的角度值存入到vector中
			 }
			gg++;
		 }
		sort(angle.begin(),angle.end());   //将vector里面的角度值按顺序排列
		anglelist.push_back(angle);
		icin5.close();
		
		line.p=vt; //返回到正在遍历的二维码
	    line.ag=PI;
    }
	if((nextid!=vt)&&(nextid!=0))   //如果没有返回到需要遍历的二维码，会不停调用最短路径的程序
	{
		repathlist=rtpath(vt,nextid);  
		line=repathlist.front();
	}
 return  line;
}