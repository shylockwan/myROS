#include <queue>
#include <string>
#include <algorithm>
#include "DP.h"
#define PI 180

int prev1=0;  //��¼��һ�α����Ķ�ά���ID
int vline[maxnum]={0};  //�����ά��ʣ��δ����·�������±�Ϊ���ڱ����Ķ�ά���ID��ÿ����һ�Σ���Ӧ������ֵ��1��ֱ�������ֵΪ0����ȫ��������
queue<int> unvisited;  //visited�������δ������ά��ID
vector<float> angle;  //�����ά��·���Ƕ�
vector<vector<float> > anglelist; //����ÿ����ά��·���ĽǶȵ�����������
vector<float> outangle;
int nextid=0;  //��һ��Ҫ���ʵĶ�ά���ID
struct pathline   //����δ����֮ǰ��·����Ϣ����ά���ID��·������
{
	int id;    //��ά���ID
	float x;
	float y;
	int ptot;  //·������
};

//===================================================================================
//��������  BFS
//���ߣ�    ����
//���ڣ�    2015-8-27
//���ܣ�    Ϊ�����ṩ����·��
//���������
//          vt   Դ�ڵ��
//����ֵ��  ���ͣ�path)
//			���ض�ά�����Ҫ������·��line,������ʼ�ڵ�ͽǶ�ֵ
//====================================================================================

path BFS(int vt)   
{
   pathline readnumber[maxnum]; //��ȡ�ļ��ж�ά��ID��·����
   path readnum[maxnum];        //��ȡ�ļ��ж�ά����ص������Ϣ
   path line;     //���صĽṹ����
   vector<path> repathlist;  //�������·���ķ���ֵ
   vector<float> temporary;  //������ʱ��ŵĽǶ�ֵ
   int ntot=0;   
   int aa=0,bb=0,dd=0,ee=0,ff=0,gg=0,hh=0,s=0;
   int ptotal=0;  //·������
   int nextid=0;  //��һ��Ҫ���ʵĶ�ά���ID

	if((prev1==0))	
	{  
		unvisited.push(vt);
		ifstream icin1;     //��ȡ�����ά��ID�Լ�·����Ŀ���ļ�
		icin1.open("qrcoordinate.dat");
		if(!icin1)
			{
				cout<<"�ļ���ȡʧ��"<<endl;
			}
		icin1 >> readnumber[vt].id >> readnumber[vt].x>> readnumber[vt].y>> readnumber[vt].ptot;  //�տ�ʼʱ�ļ���ֻ��һ�����ݣ�ֻ���һ��
		ptotal=readnumber[vt].ptot;  
		icin1.close();

		vline[vt]=ptotal;   //��ά��vδ������·����

		ifstream icin2;    //��ȡ����·����Ϣ���ļ�
		icin2.open("qr.dat");
		if(!icin2)
		 {
			cout<<"�ļ���ȡʧ��"<<endl;
		 }
		while(!icin2.eof())
		 {
			icin2 >> readnum[aa].p >> readnum[aa].q >>readnum[aa].length>>readnum[aa].lineid>>readnum[aa].ag;
			if(readnum[aa].p=vt)
			 {
				angle.push_back(readnum[aa].ag);  //�����ά��vt����·���ĽǶ�ֵ���뵽vector��
			 }
			aa++;
		 }
		sort(angle.begin(),angle.end());   //��vector����ĽǶ�ֵ��˳������
		anglelist.push_back(angle);
		icin2.close();
		
		prev1=vt;      //���浱ǰ���ڱ����Ķ�ά��ID
		outangle=anglelist.front();  //�õ������ĵ�һ��Ԫ��
		anglelist.erase(anglelist.begin());    //ɾ����һ��Ԫ��
		line.p=vt;
		line.ag =*outangle.end();   //��ȡangle���һ��ֵ
		outangle.pop_back();      //��ȡangle���һ��ֵ��ɾ���˴���ֵ������angle��ֵ
		unvisited.pop();    //ɾ�����ڱ����Ķ�ά��
		vline[vt]=vline[vt]-1;    //��ά��vδ������·����
   }
   if((vt==prev1))
   {
		if(vline[vt]!=0)
         {  
			nextid=0;    //Ϊ�˱�֤����ִ�����һ��if���
			line.p=vt;
			line.ag =*outangle.end();  //��ȡangle���һ��ֵ
			outangle.pop_back();     //��ȡangle���һ��ֵ��ɾ���˴���ֵ������angle��ֵ
			vline[vt]=vline[vt]-1;   //��ά��vδ������·����
		 }
		else        //˵����һ����ά���Ѿ������꣬��Ҫ������һ����ά����б���,��Ҫ�������·�����뷵�ص�������Ķ�ά��
			{
				nextid=unvisited.front();
				unvisited.pop(); //ɾ��δ�����е�ǰ���ڱ����Ķ�ά��
				outangle=anglelist.front();

				for(int kk=0;kk<=unvisited.size();kk++)    //�տ�ʼ�����Ķ�ά��·����������Ѿ���������·�����������Ҫ�Ƚ��Ƿ����ظ�·��
				{
					ifstream icin6;    //��ȡ����·����Ϣ���ļ�
					icin6.open("qr.dat");
					if(!icin6)
					{
						cout<<"�ļ���ȡʧ��"<<endl;
					}
					while(!icin6.eof())
					{
						icin6 >> readnum[hh].p >> readnum[hh].q >>readnum[hh].length>>readnum[hh].lineid>>readnum[hh].ag;
						if((readnum[hh].lineid!=0)&&(readnum[hh].p==nextid))    //��ʾ����·���Ѿ��߹���δ�߹���IDĬ�϶���0      
						{	
							temporary.push_back(readnum[hh].ag);  //�����Ѿ�������·���ĽǶ�ֵ
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
								outangle.erase(ito);//ɾ��ֵΪtemporary��Ԫ�� 
								vline[nextid]=vline[nextid]-1;//ɾ���Ѿ���������·��
							}
							++itt;
						}
						++ito; 
					} 
					temporary.clear();

					if(vline[nextid]==0)  //�����һ�������Ķ�ά��û���������·�������unvisited��ɾ��
					{
						nextid=unvisited.front();
						unvisited.pop(); //ɾ��δ�����е�ǰ���ڱ����Ķ�ά��
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
					line.p=0; //���unvisited����û��δ�����Ķ�ά�룬˵�������Ѿ���ɣ��򷵻صĽṹԪ�ص�ֵȫ��Ϊ0
					line.q=0;
					line.ag=0;
					line.lineid=0;
					line.length=0;
				}
			
		}
	 }
	else
     {
	    ifstream icin4;  //��ȡ�����ά��ID�Լ�·����Ŀ���ļ�
        icin4.open("qrcoordinate.dat");
        if(!icin4)
         {
	       cout<<"�ļ���ȡʧ��"<<endl;
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

		ifstream icin5;    //��ȡ����·����Ϣ���ļ�
		icin5.open("qr.dat");
		if(!icin5)
		 {
			cout<<"�ļ���ȡʧ��"<<endl;
		 }
		while(!icin5.eof())
		 {
			icin5 >> readnum[gg].p >> readnum[gg].q >>readnum[gg].length>>readnum[gg].lineid>>readnum[gg].ag;
			if(readnum[gg].p==vt)
			 {
				angle.push_back(readnum[gg].ag);  //�����ά��v����·���ĽǶ�ֵ���뵽vector��
			 }
			gg++;
		 }
		sort(angle.begin(),angle.end());   //��vector����ĽǶ�ֵ��˳������
		anglelist.push_back(angle);
		icin5.close();
		
		line.p=vt; //���ص����ڱ����Ķ�ά��
	    line.ag=PI;
    }
	if((nextid!=vt)&&(nextid!=0))   //���û�з��ص���Ҫ�����Ķ�ά�룬�᲻ͣ�������·���ĳ���
	{
		repathlist=rtpath(vt,nextid);  
		line=repathlist.front();
	}
 return  line;
}