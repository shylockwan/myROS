//#include <queue>
#include <string>
#include <algorithm>
#include <sstream>
#include "DP.h"
#include "Map.h"

/*
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
*/

//===================================================================================
//函数名：  BFS
//作者：    杨旺
//日期：    2015-9-4
//功能：    为遍历提供所需路径
//输入参数：
//          vt   源节点号
//返回值：  类型（path)
//			返回二维码出需要遍历的路径line,包含起始节点和角度值
//====================================================================================

path BFS(string string_vt)   
{
	cout<<"hhhhhhhhhhhhhhhhhhhhhhhhh, string_vt="<<string_vt<<endl;
	int vt;
	//将string转换成int
	Map str_to_int;
    vt=str_to_int.string2index(string_vt);
	 cout<<"iiiiiiiiiiiiiiiiiiiiii, vt="<<vt<<endl;
   pathline readnumber[maxnum]; //读取文件中二维码ID和路径数
   path readnum[maxnum];        //读取文件中二维码相关的五个信息
   path line;     //返回的结构对象
   vector<path> repathlist;  //接收最短路径的返回值
   vector<float> temporary;  //保存临时存放的角度值
   int ntot=0;   
   int aa=0,bb=0,dd=0,ee=0,ff=0,gg=0,hh=0,s=0,zz=0;
   int ptotal=0;  //路径总数
   int nextid=0;  //下一个要访问的二维码的ID

    cout<<"iiiiiiiiiiiiiiiiiii222, vt="<<vt<<"prev1="<<prev1<<endl;

	if((prev1==0))	
	{  
		unvisited.push(vt);
		ifstream icin1;     //读取保存二维码ID以及路径数目的文件
		icin1.open("qrcoordinate.dat");
		if(!icin1)
			{
				cout<<"文件读取失败"<<endl;
			}
			while(!icin1.eof())
			{
				icin1 >> readnumber[zz].id >>readnumber[zz].string_id>> readnumber[zz].x>> readnumber[zz].y>> readnumber[zz].ptot; 
				if(readnumber[zz].id==vt)
				{
					ptotal=readnumber[vt].ptot;
					break;
				}
				zz++;
			}
		icin1.close();
                cout<<"jjjjjjjjjjjjjjjjjj, ptotal="<<ptotal<<endl;

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
			if((readnum[aa].p==vt)&&(readnum[aa].lineid==0))
			 {
				angle.push_back(readnum[aa].ag);  //将与二维码vt相邻路径的角度值存入到vector中
			 }
			aa++;
		 }
		sort(angle.begin(),angle.end());   //将vector里面的角度值按顺序排列
               for(vector<float>::const_iterator ci=angle.begin();ci!=angle.end();ci++)//测试容器里面的值是否为二维码的ID号
        	{
              		cout<<"kkkkk"<< *ci<<endl;    //输出依次需要经过的二维码ID
 
        	}
       anglelist.push_back(angle);
       icin2.close();
		prev1=vt;      //保存当前正在遍历的二维码ID
 		cout<<"lllllllll, prev1="<<prev1<<endl;


		outangle=anglelist.front();  //得到向量的第一个元素
		anglelist.erase(anglelist.begin());    //删除第一个元素
		line.p=vt;
		line.ag =*outangle.end();   //读取angle最后一个值
		line.re=0;
		outangle.pop_back();      //读取angle最后一个值后，删除此处的值，更新angle的值
		unvisited.pop();    //删除正在遍历的二维码
		vline[vt]=vline[vt]-1;    //二维码v未遍历的路径数
		cout<<"mmmmmmmmmmmmmm"<<"line.p=" <<line.p<<" "<<"line.ag="<<line.ag<<" "<<"line.re="<<line.re<<" "<<"vline["<<vt<<"]="<<vline[vt]<<endl; 
   }
   if((vt==prev1))
   {
		if(vline[vt]!=0)
         {  
			nextid=0;    //为了保证不再执行最后一个if语句
			line.p=vt;
			line.ag =*outangle.end();  //读取angle最后一个值
			line.re=0;
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
					prev1=0;
					line.re=0;
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
				icin4 >> readnumber[bb].id >>readnumber[bb].string_id>> readnumber[bb].x>> readnumber[bb].y>> readnumber[bb].ptot;
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
	    line.re=1;
    }
	if((nextid!=vt)&&(nextid!=0))   //如果没有返回到需要遍历的二维码，会不停调用最短路径的程序
	{
		repathlist=rtpath(vt,nextid);  
		line=repathlist.front();
	}
 return  line;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*
const int maxint = 999999; 
// 各数组都从下标1开始
float dist[maxnum];     // 表示当前点到源点的最短路径长度
float c[maxnum][maxnum];   // 记录图的两点间路径长度
*/

//===================================================================================
//函数名： Dijkstra
//作者：    杨旺
//日期：    2015-8-24
//功能：    寻找源节点到目标节点最短路径需要经过的节点以及最短距离
//输入参数：n   节点总数
//          v   源节点号
//          dist 第i个节点到目标节点的距离
//          prev2  保存需要经过的中间节点
//          c[][] 保存两个节点间的距离     
//返回值：  类型（void)
//====================================================================================

void Dijkstra(int n, int v, float *dist, int *prev2, float c[maxnum][maxnum])
{
	bool s[maxnum];    // 判断是否已存入该点到S集合中
	for(int i=1; i<=n; ++i)
	{
		dist[i] = c[v][i];
		s[i] = 0;     // 初始都未用过该点
		if(dist[i] == maxint)
			prev2[i] = 0;
		else
			prev2[i] = v;
	}
	dist[v] = 0;
	s[v] = 1;
 
	// 依次将未放入S集合的结点中，取dist[]最小值的结点，放入集合S中
	// 一旦S包含了所有V中顶点，dist就记录了从源点到所有其他顶点之间的最短路径长度
    // 注意是从第二个节点开始，第一个为源点
	for(int i=2; i<=n; ++i)
	{
		float tmp = maxint;
		int u = v;
		// 找出当前未使用的点j的dist[j]最小值
		for(int j=1; j<=n; ++j)
			if((!s[j]) && dist[j]<tmp)
			{
				u = j;              // u保存当前邻接点中距离最小的点的号码
				tmp = dist[j];
			}
		s[u] = 1;    // 表示u点已存入S集合中
 
		// 更新dist
		for(int j=1; j<=n; ++j)
			if((!s[j]) && c[u][j]<maxint)
			{
				float newdist = dist[u] + c[u][j];
				if(newdist < dist[j])
				{
					dist[j] = newdist;
					prev2[j] = u;
				}
			}
	}
}
 
//=============================================================================
//函数名：searchPath
//作者：    杨旺
//日期：    2015-8-27
//功能：    查找从源点v到终点u的路径，并输出路径号及对应的角度值
//输入参数：
//          v   源节点号
//          u   目标节点
//          prev2  保存需要经过的中间节点   
//返回值：  类型（path) 
//			返回需要的路径信息repath
//==============================================================================

vector<path> searchPath(int *prev2,int v, int u)
{
	vector<int> node;
	int que[maxnum];
	int size=0;
	int tot = 1;
	que[tot] = u;
	tot++;
	int tmp = prev2[u];

	while(tmp != v)
	{
		que[tot] = tmp;
		tot++;
		tmp = prev2[tmp];
	}
	que[tot] = v;

	for(int i=tot; i>=1; --i)
		{
			
			//cout << que[i] << endl;
				 node.push_back(que[i]);      //node为需要经过的节点
	     }
	
	
	for(vector<int>::const_iterator citer=node.begin();citer!=node.end();citer++)//测试容器里面的值是否为二维码的ID号
        {
              cout<< *citer<<endl;    //输出依次需要经过的二维码ID
 
        }

	size=node.size();   //需要经过的节点个数
	vector<int> saveline;
	vector<int> savep;
	vector<int> saveq;
	vector<float> saveangle;
	vector<float> savelen;
	vector<bool> savere;
	vector<path> listpath;
	path number[maxnum];
	int i=1;
	//读入文件中的数据
	int n;
	ifstream icin;
	icin.open("qr.dat");
	if(!icin)
	{
		cout<<"文件读取失败";
	}
	 while(!icin.eof())
    {
        icin >> number[i].p >> number[i].q >>number[i].length>>number[i].lineid>>number[i].ag;
        i++;
	 }
	n=i;
	icin.close();
	
	for(int i=0;i<=size-2;i++)
	{
	   for(int j=0;j<=n-1;j++)
	     {

		   if((number[j].p==node[i]) && (number[j].q==node[i+1]))
		      {
				 savep.push_back(node[i]);
				 saveq.push_back(node[i+1]);
				 savelen.push_back(number[j].length);
			     saveline.push_back(number[j].lineid);
			     saveangle.push_back(number[j].ag);
				 savere.push_back(0);
			   
		      }	

	   }
	}
	
	for(int i=0;i<saveline.size();i++)  
	{ 
		path savepath;
		savepath.p=savep[i];
		savepath.q=saveq[i];
		savepath.length=savelen[i];
		savepath.ag=saveangle[i];
		savepath.lineid=saveline[i];
		savepath.re=savere[i];
		listpath.push_back(savepath);
		cout<<savep[i]<<" "<<saveq[i]<<" "<<savelen[i]<<" "<<saveline[i]<<" "<<saveangle[i]<<" "<<savere[i]<<endl; //输出完整的路径信息
	}	
	return listpath;
}

 //=============================================================================
//函数名：rtpath
//作者：    杨旺
//日期：    2015-8-27
//功能：    根据输入的起始节点和目标节点，处理后面函数所需要的参数，得到所需路径信息
//输入参数：
//          st   源节点号
//          ed   目标节点
//      
//返回值：  类型（path) 
//			返回需要的路径信息repath		
//==============================================================================


 vector<path> rtpath(int st,int ed)
{
	int start=st;
	int end=ed;
    int prev[maxnum];     // 记录当前点的前一个结点
    int node_total=0;
	//定义结构体数组
	path num[maxnum];
	int i=1;
	//读入文件中的数据
	ifstream icin;
	icin.open("qr.dat");
	if(!icin)
	{
		cout<<"文件读取失败";
	}
	 while(!icin.eof())
    {
        icin >> num[i].p >> num[i].q >>num[i].length>>num[i].lineid>>num[i].ag;
		
		if(num[i].p>node_total)     //取最大节点号，求出节点总数node_total
		{
			node_total=num[i].p;
		}
		if(num[i].q>node_total)
		{
			node_total=num[i].q;
		}
        i++;
	 }
	icin.close();
	cout<<node_total;

	for(int i=1; i<=node_total; ++i)
		for(int j=1; j<=node_total; ++j)
			c[i][j] =float(maxint);
	
	//读入文件中的数据
	ifstream ifcin;
	ifcin.open("qr.dat");
	if(!ifcin)
	{
		cout<<"文件读取失败";
	}
	
	 while(!ifcin.eof())
    {
        ifcin >> num[i].p >> num[i].q >>num[i].length>>num[i].lineid>>num[i].ag;
		if(num[i].length<maxint)
		{
		c[num[i].p][num[i].q] = num[i].length;  //保存p,q两个节点间的路径长度
		}
        i++;
	 }
	icin.close();

	for(int i=1; i<=node_total; ++i)
	{
		for(int j=1 ;j<=node_total; ++j)
			cout<<setw(8)<<c[i][j];       //输出各节点间的距离
		cout<<endl;

	}
 
	for(int i=1; i<=node_total; ++i)
		dist[i] = maxint;

	Dijkstra(node_total,start,dist,prev,c);
	vector<path> repath;
	repath=searchPath(prev, start, end); //接受返回的结构数组
	system("pause");
 return repath;
}
