//函数名：Map()是Map类的构造函数
//作者：chenchao
//日期：2015-07-13
//功能：初始化Map类，将文件中的数据读入链表，共有三
//      个文件一个文件记录的是两个二维码的ID，他们之间的距离，
//      以及线的id还有从第一个二维码到第二个二维码的转角
//输入参数：三个文件的文件名，不输入文件名则采用默认的文件名
//----------------------------------------------------------------
#include"Map.h"
Map::Map(string name1,string name2,string name3){
		qrflag=false;
		mapflag=false;
		qrname=name2;
		mapname=name1;	
		qrcoordinatename=name3;
		qrfile.open((qrname+".dat").c_str(),ios::binary|ifstream::in);
		if(!qrfile)
		{
		 cerr<<"qrfile didn't exist,creat a new one!"<<endl;
		}
		else{//把数据读到内存中
			string date;
			float date2;
			int date3;
			while(qrfile>>date){
				qrmsg node;
				node.fid=date;
				qrfile>>date;
				node.sid=date;
				qrfile>>date2;
				node.distance=date2;
				qrfile>>date3;
				node.lineid=date3;
				qrfile>>date2;
				node.angle=date2;
				qrlist.push_back(node);//把二值文件中的数据读出来
			}
		}
		alllineid=qrlist.size();
		mapfile.open((mapname+".dat").c_str(),ios::binary|ifstream::in);//把二维码及线路的数据读进内存ios::binary|
		if(!mapfile)
		{
		 cerr<<"map didn't exist,creat a new one!"<<endl;
		}
		else{//把数据读到内存中
			float x1,y1,x2,y2;
			while(mapfile>>x1){
				mapmsg node;
				node.x1=x1;
				mapfile>>y1;
				node.y1=y1;
				mapfile>>x2;
				node.x2=x2;
				mapfile>>y2;
				node.y2=y2;
				maplist.push_back(node);//把二值文件中的数据读出来
			}
		}
		qrcoordinatefile.open((qrcoordinatename+".dat").c_str(),ios::binary|ifstream::in);
		if(!qrcoordinatefile)
		{
			cerr<<"qrcoordinate didn't exist,creat a new one!"<<endl;
		}
		else{
			float x,y;
			string id;
			int z;
			while(qrcoordinatefile>>id)
			{
				qrcoormsg node;
				node.id=id;
				qrcoordinatefile>>x;
				node.x=x;
				qrcoordinatefile>>y;
				node.y=y;
				qrcoordinatefile>>z;
				node.count=z;
				coorlist.push_back(node);
			}
		}
		mapfile.close();
		qrfile.close();
		qrcoordinatefile.close();
	}


//===============================================================================================================
//函数名：addqr()，是Map类的成员函数
//功能：将两个二维码id，线路id，转角信息存到一个链表中
//输入参数:当前扫描到的二维码ID，左右编码器的值，
//小车从前一个二维码到当前二维码的角度,小车在二维码处扫描出的多条路径的对应角度组成的向量
//--------------------------------------------------------------------------
void Map::addqr(string id,int lencoder,int rencoder,float angle, vector<float> a){//记录二维码信息，输入为当前检测到的二维码的ID，		                                 
		float perencoder=2*3.141*0.035*132/6545;//编码器每圈走过的路程
		float ldistance,rdistance; 
		bool idflag=true;//记录是否是第一次遇到某个二维码
		for(vector<qrmsg>::iterator it=qrlist.begin();it!=qrlist.end();++it)//如果第一与第二个二维码的ID都相同，则表示已经记录过不在记录
		{
			if(it->fid==id)
			{
				idflag=false;
				break;
			}
		}
		if(idflag)//如果是第一次遇到某个二维码则记录所有以这个二维码为起点的路径信息
		{
			for(int i=0;i<a.size();i++)
				{
					qrmsg node;
					node.fid=id;
					node.lineid=alllineid;
					node.sid="0";
					node.distance=0;
					++alllineid;
					node.angle=a[i];
					qrlist.push_back(node);
				}
		}

		if(!qrflag)    //第一次调用函数时初始化第一个二维码的信息
		{
			fid=id;
			angle1=0;//第一次遇到某个二维码时的角度没有用
			rencoder1=rencoder;
			lencoder1=lencoder;
			qrflag=true;
			
		}
		else
		{
			sid=id;
			//angle2=angle;
			lencoder2=lencoder;
			rencoder2=rencoder;
			ldistance=lencoder2-lencoder1;
			rdistance=rencoder2-rencoder1;
			distance=perencoder*(rdistance+ldistance)/2;//根据左右编码器的累积值近似两个二维码之间的距离		
			for(vector<qrmsg>::iterator beg=qrlist.begin(),end=qrlist.end();beg!=end;++beg)
			{			
				if((beg->fid==fid)&&(beg->sid==sid))//如果fid与sid都相等的时候，表示当前路径的信息已经记录完整
				{
					fid=sid;
					//angle1=angle;
					rencoder1=rencoder2;
					lencoder1=lencoder2;
					 break;
				}
				else if((beg->fid==fid)&&(angle==beg->angle))//如果只有fid与angle相等，则表示信息不完整需要添加
				{
					beg->distance=distance;
					beg->sid=sid;
					fid=sid;
					//angle1=angle;
					rencoder1=rencoder2;
					lencoder1=lencoder2;
					break;
				}
			}
			
		}
	

		
	}


//===========================================================================================
//函数名：addmap()，是Map类的一个成员函数
//功能：绘制矢量地图，格式为[x1 y1 x2 y2]
//输入：机器人当前的坐标值
//----------------------------------------------------------
void Map::addmap(float x,float y){	//记录地图坐标信息,函数输入为机器人当前的坐标
		if(!mapflag){
			x11=x;
			y11=y;
			mapflag=true;
		}
		else{
			
			x22=x;
			y22=y;
			mapmsg node;
			node.x1=x11;
			node.y1=y11;
			node.x2=x22;
			node.y2=y22;
			maplist.push_back(node);
			x11=x22;
			y11=y22;
		}
	}


//=============================================================
//函数名：qrcoordiate()是Map类的一个成员函数
//功能;把每个二维码id及相对应的坐标，还有与每个二维码连接的路径数记录下来
//输入;二维码Id，响应的坐标，还有路径数 
//------------------------------------------------------------------------
void Map::qrcoordiate(string id,float x,float y,int z){//把二维码及相应的坐标保存在一个文件中
	bool idflag=true;
	for(vector<qrcoormsg>::iterator it=coorlist.begin();it!=coorlist.end();++it)
	{
		if(it->id==id)
		{
			idflag=false;
			break;
		}
		
	}
	if(idflag)
	{
		qrcoormsg node;
		node.id=id;
		node.x=x;
		node.y=y;
		node.count=z;
		coorlist.push_back(node);
	}
	}


//=============================================================
//函数名：keepdate()，是类Map的成员函数
//功能：将数据保存到文件，共有三个文件
//一个是保存二维码的信息，格式为：id x y count(二维码id,坐标x,y,路径数count)
//一个保存矢量地图，格式为：x1 y1 x2 y2（起始点的坐标和终止点的坐标）
//一个保存二维码间的信息，格式为:id1 id2 distance lineid angle
//(两个二维码的id，两个二维码间的距离，线id，角度)
//--------------------------------------------------------------
void Map::keepdate(){//保存数据到文件
		qrfile.open((qrname+".dat").c_str(),ios::out);//,ios::binary|ofstream::out
		for(vector<qrmsg>::iterator it=qrlist.begin();it!=qrlist.end();++it)//二维码地图的存储的形式为[ID1,ID2,distance,lineid,angle]
		{
			qrfile<<it->fid<<" "<<it->sid<<" "<<it->distance<<" "<<it->lineid<<" "<<it->angle<<" "<<endl;
		}
		mapfile.open((mapname+".dat").c_str(),ios::binary|ofstream::out);//
		for(vector<mapmsg>::iterator it=maplist.begin();it!=maplist.end();++it)//矢量地图的存储形式[x1,y1,x2,y2]
		{
			mapfile<<it->x1<<" "<<it->y1<<" "<<it->x2<<" "<<it->y2<<" "<<endl;
			
		}
		qrcoordinatefile.open((qrcoordinatename+".dat").c_str(),ios::binary|ofstream::out);
		for(vector<qrcoormsg>::iterator it=coorlist.begin();it!=coorlist.end();++it)
		{
			qrcoordinatefile<<it->id<<" "<<it->x<<" "<<it->y<<" "<<it->count<<" "<<endl;
		}
		mapfile.close();
		qrfile.close();
		qrcoordinatefile.close();
		
	}


//===========================================================
//函数名：remapmsg()返回地图数据
//功能：返回地图数据
//输出：输出是一个元素为mapmsg的向量
//-----------------------------------------------------
vector<mapmsg> Map::remapmsg(){
	return maplist;
}


//======================================================
//函数名：remapmsg()返回地图数据
//功能：返回地图数据
//输出：输出是一个元素为mapmsg的向量
//--------------------------------------------------------
vector<qrcoormsg> Map::reqrcoormsg(){
	return coorlist;
}


//======================================================
//函数名：reqrmsg()返回地图数据
//功能：返回地图数据
//输出：输出是一个元素为mapmsg的向量
//---------------------------------------------------
vector<qrmsg> Map::reqrmsg(){
	return qrlist;
}


//==================================================
//函数名：qrtocoor(),Map类的一个成员函数
//功能：根据二维码id号，查询二维码的相关信息：坐标和路径数
//输入：二维码的id
//输出：一个包含二维码信息的数据结构:qrcoormsg
//-----------------------------------------------------------
qrcoormsg Map::qrtocoor(string id){//根据二维码id查询它的坐标
		qrcoormsg remsg;
		for(vector<qrcoormsg>::iterator beg=coorlist.begin(),end=coorlist.end();beg!=end;++beg)
		{
			if(beg->id==id)
			{
				remsg=*beg;
				break;
			}
			else
			{
				remsg=*end;
			}
		}
		return remsg;
	}


//==============================================================
//函数名：lineto(),Map类的一个成员函数
//功能：根据线id号，查询跟这条线有关的信息（两头的二维码id号，线的长度，第一个二维码到第二个二维码的的转角）
//输入：线id
//输出：一个包含信息数据结构:qrmsg
//--------------------------------------------------------------------------
qrmsg Map::lineto(int id){//根据路径信息查询其他信息
		qrmsg remsg;
		for(vector<qrmsg>::iterator beg=qrlist.begin(),end=qrlist.end();beg!=end;++beg){
			if(beg->lineid==id)
			{
				remsg=*beg;
				break;
			}
			else
			{
				remsg=*end;
			}
		}
		return remsg;
	}


//==============================================================
//函数名：qrto(),Map类的一个成员函数
//功能：根据两个二维码id号，查询二维码的相关信息：之间的路径ID，转角
//输入：两个二维码的id
//输出：一个包含二维码信息的数据结构:qrmsg
//------------------------------------------------------------------
qrmsg Map::qrto(string id1,string id2){//根据两个二维码查询路径，角度等信息
		qrmsg remsg;
		for(vector<qrmsg>::iterator beg=qrlist.begin(),end=qrlist.end();beg!=end;++beg){
			if((id1==beg->fid)&&(id2==beg->sid))
			{
				remsg=*beg;
				break;
			}
			else
			{
				remsg=*end;
			}
		}
		return remsg;
	}

