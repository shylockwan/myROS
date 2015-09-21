#include "DP.h"
const int maxint = 999999; 
// 各数组都从下标1开始
float dist[maxnum];     // 表示当前点到源点的最短路径长度
float c[maxnum][maxnum];   // 记录图的两点间路径长度

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
		listpath.push_back(savepath);
		cout<<savep[i]<<" "<<saveq[i]<<" "<<savelen[i]<<" "<<saveline[i]<<" "<<saveangle[i]<<endl;    //输出完整的路径信息
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