//#include <queue>
//#include <string>
#include <algorithm>
#include <sstream>
#include "DP.h"
#include "Map.h"


const int maxnum = 100;
int prev1=0;  //��¼��һ�α���Ķ�ά���ID
int vline[100]={0};  //�����ά��ʣ��δ����·�����±�Ϊ���ڱ���Ķ�ά���ID��ÿ����һ�Σ���Ӧ������ֵ��1��ֱ�������ֵΪ0����ȫ��������
int nextid=0;  //��һ��Ҫ���ʵĶ�ά���ID
const int maxint = 999999; 
queue<int> unvisited;  //visited�������δ�����ά��ID
//vector<float> angle;  //�����ά��·���Ƕ�
//vector<vector<float> > anglelist; //����ÿ����ά��·���ĽǶȵ�����������
vector<float> outangle;
float dist[100];     // ��ʾ��ǰ�㵽Դ������·������
float c[100][100];   // ��¼ͼ�������·������
fstream ifcin;


//===================================================================================
//������  BFS
//���ߣ�    ����
//���ڣ�    2015-9-4
//���ܣ�    Ϊ�����ṩ����·��
//�������
//          vt   Դ�ڵ��
//����ֵ��  ���ͣ�path)
//			���ض�ά�����Ҫ�����·��line,����ʼ�ڵ�ͽǶ�ֵ
//====================================================================================

path BFS(string string_vt)   
{
	cout<<"aaaaaaaaaaaaaaaaaaaaaaaaaaaa, string_vt="<<string_vt<<endl;
	int vt;
	//��stringת����int
	Map str_to_int;
    vt=str_to_int.string2index(string_vt);
	cout<<"bbbbbbbbbbbbbbbbbbbbbbbbbbbbb, vt="<<vt<<endl;
    pathline readnumber[maxnum]; //��ȡ�ļ��ж�ά��ID��·����
    path readnum[maxnum];        //��ȡ�ļ��ж�ά����ص������Ϣ
    path line;     //���صĽṹ����
    vector<path> repathlist;  //�������·���ķ���ֵ
    vector<float> temporary;  //������ʱ��ŵĽǶ�ֵ
    int ntot=0;   
    int aa=0,bb=0,dd=0,ee=0,ff=0,gg=0,hh=0,s=0,zz=0;
    int ptotal=0;  //·������
    int nextid=0;  //��һ��Ҫ���ʵĶ�ά���ID
    int size;//����δ����Ľڵ���Ŀ
    cout<<"cccccccccccccccccccccccccccc, vt="<<vt<<"prev1="<<prev1<<endl;

	if((prev1==0))	
	{  
		unvisited.push(vt);
		ifstream icin1;     //��ȡ�����ά��ID�Լ�·����Ŀ���ļ�
		icin1.open("/home/shylockwang/workspace/MyProject/qrcoordinate.dat");
		if(!icin1)
			{
				cout<<"�ļ���ȡʧ��"<<endl;
			}
			while(!icin1.eof())
			{
				icin1 >> readnumber[zz].id >>readnumber[zz].string_id>> readnumber[zz].x>> readnumber[zz].y>> readnumber[zz].ptot; 
				
				if(readnumber[zz].id==vt)
				{
					ptotal=readnumber[zz].ptot;
					break;
				}
				zz++;
			}
		icin1.close();
        cout<<"dddddddddddddddddddddddddddd, ptotal="<<ptotal<<endl;

		vline[vt]=ptotal;   //��ά��vδ�����·����
		cout<<"eeeeeeeeeeeeeeeeeeeeee  total_vline["<<vt<<"]"<<vline[vt]<<endl;

		ifstream icin;    //��ȡ����·����Ϣ���ļ�
		icin.open("/home/shylockwang/workspace/MyProject/qr.dat");
		if(!icin)
		 {
			cout<<"�ļ���ȡʧ��"<<endl;
		 }
		while(!icin.eof())
		 {
			icin >> readnum[aa].p >> readnum[aa].q >>readnum[aa].length>>readnum[aa].lineid>>readnum[aa].ag;
			if((readnum[aa].p==vt))
			 {
				outangle.push_back(readnum[aa].ag);  //�����ά��vt����·���ĽǶ�ֵ���뵽vector��
			 }
			aa++;
		 }
		icin.close();
		sort(outangle.begin(),outangle.end());   //��vector����ĽǶ�ֵ��˳������
        for(vector<float>::const_iterator ci=outangle.begin();ci!=outangle.end();ci++)//�������������ֵ�Ƿ�Ϊ��ά���ID��
        {
              cout<<"fffffffffffffffffffffff"<<" "<< *ci<<endl;    
 
         }
		prev1=vt;      //���浱ǰ���ڱ���Ķ�ά��ID
 		cout<<"gggggggggggggggg, prev1="<<prev1<<endl;
		line.p=vt;
		line.q=0;
		line.ag =outangle.front();   //��ȡangle��һ��ֵ
		if (line.re!=0 || line.re !=1)
			line.re=0;	
		line.lineid=0;
		line.length=0;
		outangle.erase(outangle.begin());      //��ȡangle��һ��ֵ��ɾ��˴���ֵ������angle��ֵ
		unvisited.pop();    //ɾ�����ڱ���Ķ�ά��
		vline[vt]=vline[vt]-1;    //��ά��vδ�����·����
		cout<<"hhhhhhhhhhhhhhhhhhhh"<<"line.p=" <<line.p<<" "<<"line.ag="<<line.ag<<" "<<"line.re="<<line.re<<" "<<"vline["<<vt<<"]="<<vline[vt]<<endl; 
		return  line;
	}
   if((vt==prev1))
   {
		if(vline[vt]!=0)
         {  
			nextid=0;    //Ϊ�˱�֤����ִ�����һ��if���
			line.p=vt;
			line.q=0;
			line.ag =outangle.front();  
		    if (line.re!=0 || line.re !=1)
		        line.re=0;		
			
			line.lineid=0;
			line.length=0;
			outangle.erase(outangle.begin());    //��ȡangle��һ��ֵ��ɾ��˴���ֵ������angle��ֵ
			vline[vt]=vline[vt]-1;   //��ά��vδ�����·����
            cout<<"iiiiiiiiiiiiiiiiiiiii"<<"line.p=" <<line.p<<" "<<"line.ag="<<line.ag<<" "<<"line.re="<<line.re<<" "<<"vline["<<vt<<"]="<<vline[vt]<<endl; 
			return  line;
		}
		else        //˵����һ����ά���Ѿ������꣬��Ҫ������һ����ά����б���,��Ҫ�������·�����뷵�ص������Ķ�ά��
			{
				nextid=unvisited.front();
				cout<<"jjjjjjjjjjjjjjjjjjjjjjj       nextid="<<nextid<<endl;
                cout<<"kkkkkkkkkkkkkkkkkkkkkk       vline["<<nextid<<"]=="<<vline[nextid]<<endl;
				unvisited.pop(); //ɾ��δ�����е�ǰ���ڱ���Ķ�ά��
				cout<<"llllllllllllllllllllll unvisited.size()="<<unvisited.size()<<endl;
				
				size=unvisited.size();
				for(int kk=0;kk<=size;kk++)    //�տ�ʼ����Ķ�ά��·��������Ѿ�������·���������Ҫ�Ƚ��Ƿ����ظ�·��
				{
					cout<<"kk11kk11kk11kk11  kk="<<kk<<endl;
					ifstream icin2;    //��ȡ����·����Ϣ���ļ�
					icin2.open("/home/shylockwang/workspace/MyProject/qr.dat");
					if(!icin2)
					{
						cout<<"�ļ���ȡʧ��"<<endl;
					}
					while(!icin2.eof())
					{
						icin2 >> readnum[hh].p >> readnum[hh].q >>readnum[hh].length>>readnum[hh].lineid>>readnum[hh].ag;
						if(readnum[hh].q==prev1 && readnum[hh].p==nextid)    //��ʾ����·���Ѿ��߹���ʼ���յ�Ķ�ά��ID	��Ϊ0     
						{	
							temporary.push_back(readnum[hh].ag);  //�����Ѿ������·���ĽǶ�ֵ
						}
						if(readnum[hh].p==nextid)
						{
							outangle.push_back(readnum[hh].ag);  //�����ά��vt����·���ĽǶ�ֵ���뵽vector��
						}
						hh++;
					}
					icin2.close();

					for(vector<float>::const_iterator citt=outangle.begin();citt!=outangle.end();citt++)//�������������ֵ�Ƿ�Ϊ��ά���ID��
        			{
              			cout<<"mmmmmmmmmmmmmmmmmmmmmmm"<< *citt<<endl;    
        			 }
					sort(outangle.begin(),outangle.end());   //��vector����ĽǶ�ֵ��˳������
					for(vector<float>::const_iterator cit=temporary.begin();cit!=temporary.end();cit++)
        			{
              			cout<<"nnnnnnnnnnnnnnnnnnnnn"<< *cit<<" "<<endl;    
 
        			}

					for(vector<float>::const_iterator ci=outangle.begin();ci!=outangle.end();ci++)
        			{
              			cout<<"oooooooooooooooooooooo"<< *ci<<" "<<endl;    
 
        			}
					vector<float>::iterator ito = outangle.begin(); 
					vector<float>::iterator itt = temporary.begin();

					cout<<"22222222222222 outangle.size()="<<outangle.size()<<endl;

					while (ito != outangle.end()) 
					{ 
						cout<<"111111111111111111111111"<<endl;
						while (itt != temporary.end()) 
						{ 
							cout<<"2222222222222222222222"<<endl;
							if (*ito == *itt)
							{
								outangle.erase(ito);//ɾ��ֵΪtemporary��Ԫ�� 
								--ito;
								vline[nextid]=vline[nextid]-1;//ɾ���Ѿ�������·��
								cout<<"the visited angle is "<<*itt<<endl;
								break;
							}
								
							++itt;
						}
						++ito; 
					} 
					temporary.clear();
					cout<<"ppppppppppppppppp    vline["<<nextid<<"]=="<<vline[nextid]<<endl;

					if((unvisited.size()==0) && (vline[nextid]==0))
					{
						cout<<"endendendendendendendendend"<<endl;
						nextid=0;
						prev1=0;
						line.re=0;
						line.p=0; //���unvisited����û��δ����Ķ�ά�룬˵�������Ѿ���ɣ��򷵻صĽṹԪ�ص�ֵȫ��Ϊ0
						line.q=0;
						line.ag=0;
						line.lineid=0;
						line.length=0;
						return  line;
				    }
					if(vline[nextid]==0)  //�����һ������Ķ�ά��û��������·�������unvisited��ɾ��
					{
						nextid=unvisited.front();
						cout<<"qqqqqqqqqqqqqqqqq nextid]="<<nextid;
						cout<<"rrrrrrrrrrrrrrrrrrrrrrrr  vline["<<nextid<<"]="<<vline[nextid];
						cout<<"sssssssssssssssssss unvisited.size()="<<unvisited.size()<<endl;
						unvisited.pop(); //ɾ��δ�����е�ǰ���ڱ���Ķ�ά��
						cout<<"ttttttttttttttttttttttt unvisited.size()="<<unvisited.size()<<endl;
					}
					else
					{
						cout<<"cecececececececececececececececece"<<endl;
						prev1=nextid;
						cout<<"v3v3v3v3v3v3v3v3v3v3v3v3v3v3  vline["<<nextid<<"]="<<vline[nextid];
						unvisited.pop(); //ɾ��δ�����е�ǰ���ڱ���Ķ�ά��
						cout<<"u4u4u4u4u4u4u4u4u4u4u4 unvisited.size()="<<unvisited.size()<<endl;
						line.re=0;
						line.p=vt;
						line.q=nextid;
						line.ag=outangle.front(); 
						outangle.erase(outangle.begin()); 
						return line;
						
					}
					/*
				if((unvisited.size()==0) && (vline[nextid]==0))
				{
					cout<<"endendendendendendendendend"<<endl;
					nextid=0;
					prev1=0;
					line.re=0;
					line.p=0; //���unvisited����û��δ����Ķ�ά�룬˵�������Ѿ���ɣ��򷵻صĽṹԪ�ص�ֵȫ��Ϊ0
					line.q=0;
					line.ag=0;
					line.lineid=0;
					line.length=0;
					return  line;
				}
				*/
		}
	 }
}
	else
     {
	    ifstream icin4;  //��ȡ�����ά��ID�Լ�·����Ŀ���ļ�
        icin4.open("/home/shylockwang/workspace/MyProject/qrcoordinate.dat");
        if(!icin4)
         {
	       cout<<"�ļ���ȡʧ��"<<endl;
         } 
		while(!icin4.eof())
			{
				icin4 >> readnumber[bb].id >>readnumber[bb].string_id>> readnumber[bb].x>> readnumber[bb].y>> readnumber[bb].ptot;
				if(readnumber[bb].id==vt)  
				 {
					//s=bb;
					vline[vt]=readnumber[bb].ptot;
					break;
				 }
					bb++;
			}
		icin4.close();

		unvisited.push(vt);

		ifstream icin5;    //��ȡ����·����Ϣ���ļ�
		icin5.open("/home/shylockwang/workspace/MyProject/qr.dat");
		if(!icin5)
		 {
			cout<<"�ļ���ȡʧ��"<<endl;
		 }
		while(!icin5.eof())
		 {
			icin5 >> readnum[gg].p >> readnum[gg].q >>readnum[gg].length>>readnum[gg].lineid>>readnum[gg].ag;
			if(readnum[gg].p==prev1&&readnum[gg].q==vt)
			 {
			    line.ag=-(3.14-readnum[gg].ag);
				 if(line.ag<-3.14)
				 {
					  line.ag=6.28+line.ag;
				 } 
			}		 
			gg++;
		 }
		icin5.close();
		
		line.p=vt; //���ص����ڱ���Ķ�ά��
	    line.re=1;
		line.q=0;
		line.lineid=0;
		line.length=0;
		cout<<"rrrrrrrrrrrrrrrrrrrr"<<"line.p=" <<line.p<<" "<<"line.ag="<<line.ag<<" "<<"line.re="<<line.re<<" "<<"vline["<<vt<<"]="<<vline[vt]<<endl; 
		return  line;
    
	}
	if((nextid!=vt)&&(nextid!=0))   //���û�з��ص���Ҫ����Ķ�ά�룬�᲻ͣ�������·���ĳ���
	{
		repathlist=rtpath(vt,nextid);  
		line=repathlist.front();
		if (line.re!=0 || line.re !=1)
		    line.re=0;		
		
		cout<<"sssssssssssssssssssssss"<<"line.p=" <<line.p<<" "<<"line.ag="<<line.ag<<" "<<"line.re="<<line.re<<" "<<"vline["<<vt<<"]="<<vline[vt]<<endl; 
		return  line;
	}
 
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*
const int maxint = 999999; 
// �����鶼���±�1��ʼ
float dist[maxnum];     // ��ʾ��ǰ�㵽Դ������·������
float c[maxnum][maxnum];   // ��¼ͼ�������·������
*/

//===================================================================================
//������ Dijkstra
//���ߣ�    ����
//���ڣ�    2015-8-24
//���ܣ�    Ѱ��Դ�ڵ㵽Ŀ��ڵ����·����Ҫ����Ľڵ��Լ���̾���
//�������n   �ڵ�����
//          v   Դ�ڵ��
//          dist ��i���ڵ㵽Ŀ��ڵ�ľ���
//          prev2  ������Ҫ������м�ڵ�
//          c[][] ���������ڵ��ľ���     
//����ֵ��  ���ͣ�void)
//====================================================================================

void Dijkstra(int n, int v, float *dist, int *prev2, float c[maxnum][maxnum])
{
	cout<<"gggggggggggggggggggggggggggggggg"<<endl;
	bool s[maxnum];    // �ж��Ƿ��Ѵ���õ㵽S������
	for(int i=1; i<=n; ++i)
	{
		dist[i] = c[v][i];
		s[i] = 0;     // ��ʼ��δ�ù�õ�
		if(dist[i] == maxint)
			prev2[i] = 0;
		else
			prev2[i] = v;
	}
	dist[v] = 0;
	s[v] = 1;
 
	// ���ν�δ����S���ϵĽ���У�ȡdist[]��Сֵ�Ľ�㣬���뼯��S��
	// һ��S��������V�ж��㣬dist�ͼ�¼�˴�Դ�㵽���������֮������·������
    // ע���Ǵӵڶ����ڵ㿪ʼ����һ��ΪԴ��
	for(int i=2; i<=n; ++i)
	{
		float tmp = maxint;
		int u = v;
		// �ҳ���ǰδʹ�õĵ�j��dist[j]��Сֵ
		for(int j=1; j<=n; ++j)
			if((!s[j]) && dist[j]<tmp)
			{
				u = j;              // u���浱ǰ�ڽӵ��о�����С�ĵ�ĺ���
				tmp = dist[j];
			}
		s[u] = 1;    // ��ʾu���Ѵ���S������
 
		// ����dist
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
//������searchPath
//���ߣ�    ����
//���ڣ�    2015-8-27
//���ܣ�    ���Ҵ�Դ��v���յ�u��·���������·���ż���Ӧ�ĽǶ�ֵ
//�������
//          v   Դ�ڵ��
//          u   Ŀ��ڵ�
//          prev2  ������Ҫ������м�ڵ�   
//����ֵ��  ���ͣ�path) 
//			������Ҫ��·����Ϣrepath
//==============================================================================

vector<path> searchPath(int *prev2,int v, int u)
{
	cout<<"hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh"<<endl;
	cout<<" 222222222222   start="<<v<<" "<<"end="<<u<<endl;
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
				 node.push_back(que[i]);      //nodeΪ��Ҫ����Ľڵ�
	     }
	
	/*
	for(vector<int>::const_iterator citer=node.begin();citer!=node.end();citer++)//�������������ֵ�Ƿ�Ϊ��ά���ID��
        {
              cout<< *citer<<endl;    //���������Ҫ����Ķ�ά��ID
 
        }
		*/
	size=node.size();   //��Ҫ����Ľڵ����
	vector<int> saveline;
	vector<int> savep;
	vector<int> saveq;
	vector<float> saveangle;
	vector<float> savelen;
	vector<bool> savere;
	vector<path> listpath;
	path number[maxnum];
	int i=1;
	//�����ļ��е����
	int n;
	ifstream icin;
	icin.open("/home/baofei/catkin_ws/qr.dat");
	if(!icin)
	{
		cout<<"�ļ���ȡʧ��";
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
		cout<<savep[i]<<" "<<saveq[i]<<" "<<savelen[i]<<" "<<saveline[i]<<" "<<saveangle[i]<<" "<<savere[i]<<endl; //��������·����Ϣ
	}	
	return listpath;
}

 //=============================================================================
//������rtpath
//���ߣ�    ����
//���ڣ�    2015-8-27
//���ܣ�    ����������ʼ�ڵ��Ŀ��ڵ㣬������溯������Ҫ�Ĳ���õ�����·����Ϣ
//�������
//          st   Դ�ڵ��
//          ed   Ŀ��ڵ�
//      
//����ֵ��  ���ͣ�path) 
//			������Ҫ��·����Ϣrepath		
//==============================================================================


 vector<path> rtpath(int st,int ed)
{
	fstream ifcin;
	int start=st;
	int end=ed;
    int prev[maxnum];     // ��¼��ǰ���ǰһ�����
    int node_total=0;
	//����ṹ������
	path num[maxnum];
	int i=1;
	
	ifcin.open(("/home/shylockwang/workspace/MyProject/qr.dat"),ios::binary|ifstream::in);
	while(ifcin>>num[i].p)
	{

	ifcin >> num[i].q >>num[i].length>>num[i].lineid>>num[i].ag;
		
		if(num[i].p>node_total)     //ȡ���ڵ�ţ�����ڵ�����node_total
		{
			node_total=num[i].p;
		}
		if(num[i].q>node_total)
		{
			node_total=num[i].q;
		}

	}
	ifcin.close();

	cout<<"node_total="<<node_total<<endl;

	for(int i=1; i<=node_total; ++i)
		for(int j=1; j<=node_total; ++j)
			c[i][j] =float(maxint);
	


	ifcin.open(("/home/shylockwang/workspace/MyProject/qr.dat"),ios::binary|ifstream::in);
	while(ifcin>>num[i].p)
	{
	ifcin >> num[i].q >>num[i].length>>num[i].lineid>>num[i].ag;
		{
		   if(num[i].q!=0)
			{
			c[num[i].p][num[i].q] = num[i].length;  //����p,q�����ڵ���·������
			}
				cout<<num[i].p<<" "<<num[i].q<<" "<<num[i].length<<" "<<num[i].lineid<<" "<<num[i].ag<<endl; 
		}
	}
	ifcin.close();
	
	for(int i=1; i<=node_total; ++i)
	{
		for(int j=1 ;j<=node_total; ++j)
			cout<<setw(8)<<c[i][j];       //������ڵ��ľ���
		cout<<endl;

	}
 	
	for(int i=1; i<=node_total; ++i)
		dist[i] = maxint;
	
	Dijkstra(node_total,start,dist,prev,c);
	vector<path> repath;
	repath=searchPath(prev, start, end); //���ܷ��صĽṹ����
	for(vector<path>::const_iterator citer=repath.begin();citer!=repath.end();citer++)
        {
              cout<< "path"<<citer->p<<" "<<citer->q<<" "<<citer->ag<<endl;    //���������Ҫ����Ķ�ά��ID
 
        }

 return repath;
}
 /*
int main()
{
	vector<path> route;
	route=rtpath(1,3);
	system("pause");
}
*/
