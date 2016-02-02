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
		 qrfile.open((qrname+".dat").c_str(),ios::out);
		}
		else{
			int date;
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
				qrlist.push_back(node);
			}
		}
		alllineid=qrlist.size();
		mapfile.open((mapname+".dat").c_str(),ios::binary|ifstream::in);
		if(!mapfile)
		{
		 cerr<<"map didn't exist,creat a new one!"<<endl;
		 mapfile.open((mapname+".dat").c_str(),ios::binary|ofstream::out);
		}
		else{
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
				maplist.push_back(node);
			}
		}
		qrcoordinatefile.open((qrcoordinatename+".dat").c_str(),ios::binary|ifstream::in);
		if(!qrcoordinatefile)
		{
			cerr<<"qrcoordinate didn't exist,creat a new one!"<<endl;
			qrcoordinatefile.open((qrcoordinatename+".dat").c_str(),ios::binary|ofstream::out);
		}
		else{
			float x,y;
			string id;
			int z,index;
			while(qrcoordinatefile>>index)
			{
				qrcoormsg node;
				node.index=index;
				qrcoordinatefile>>id;
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
		qrindex=coorlist.size();
		mapfile.close();
		qrfile.close();
		qrcoordinatefile.close();
	}



void Map::addqr(string qrdesc,int lencoder,int rencoder,float angle, vector<float> a){
		int id;
cout<<"==================map:addqr================================="<<endl;
		for(vector<qrcoormsg>::iterator it=coorlist.begin();it!=coorlist.end();++it)
		{ 
			if(it->id==qrdesc)
			{
			  id=it->index;
			  break;
			}   
		}                         
		float perencoder=2*3.141*0.035*132/6545;
		float ldistance,rdistance; 
		bool idflag=true;
		for(vector<qrmsg>::iterator it=qrlist.begin();it!=qrlist.end();++it)
		{
			if(it->fid==id)
			{
				idflag=false;
				break;
			}
		}    
		if(idflag)
		{
			for(int i=0;i<a.size();i++)
				{
cout<<"add path first id:"<<id<<endl;
					qrmsg node;
					node.fid=id;
					node.lineid=alllineid;
					node.sid=0;
					node.distance=0;
					++alllineid;
					node.angle=a[i];
					qrlist.push_back(node);
				}
		}

		if(!qrflag)    
		{
			fid=id;
			angle1=0;
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
			distance=perencoder*(rdistance+ldistance)/2;
			for(vector<qrmsg>::iterator beg=qrlist.begin(),end=qrlist.end();beg!=end;++beg)
			{			cout<<"map=========map"<<endl;
				if((beg->fid==fid)&&(beg->sid==sid))
				{
					fid=sid;
					//angle1=angle;
					rencoder1=rencoder2;
					lencoder1=lencoder2;
					 break;
				}
				else if((beg->fid==fid)&&((angle<(beg->angle+0.5)&&angle>(beg->angle-0.5))||fabs(fabs(angle)-fabs(beg->angle))<0.5)&&(beg->sid==0)&&(fid!=sid))
				{cout<<"添加完整路径：fid======="<<fid<<"===="<<"sid========="<<sid<<"========="<<endl;
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
		qrfile.open((qrname+".dat").c_str(),ios::out);
		for(vector<qrmsg>::iterator it=qrlist.begin();it!=qrlist.end();++it)
		{
			qrfile<<it->fid<<" "<<it->sid<<" "<<it->distance<<" "<<it->lineid<<" "<<it->angle<<" "<<endl;
		}
		qrfile.close();
	

		
	}



void Map::addmap(float x,float y){	
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
		mapfile.open((mapname+".dat").c_str(),ios::binary|ofstream::out);
		for(vector<mapmsg>::iterator it=maplist.begin();it!=maplist.end();++it)
		{
			mapfile<<it->x1<<" "<<it->y1<<" "<<it->x2<<" "<<it->y2<<" "<<endl;
			
		}
		mapfile.close();
	}



void Map::qrcoordiate(string id,float x,float y,int z){
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
		qrindex++;
		node.index=qrindex;		
		node.id=id;
		node.x=x;
		node.y=y;
		node.count=z;
		coorlist.push_back(node);
cout<<"========map.qrcoor======="<<qrindex<<" "<<id<<" "<<x<<" "<<y<<" "<<z<<" "<<endl;
	}
	qrcoordinatefile.open((qrcoordinatename+".dat").c_str(),ios::binary|ofstream::out);
		for(vector<qrcoormsg>::iterator it=coorlist.begin();it!=coorlist.end();++it)
		{
			qrcoordinatefile<<it->index<<" "<<it->id<<" "<<it->x<<" "<<it->y<<" "<<it->count<<" "<<endl;
		}
		qrcoordinatefile.close();
}

int Map::string2index(string id)
{
	for(vector<qrcoormsg>::iterator it=coorlist.begin();it!=coorlist.end();++it)
		{
			if(it->id==id)
			{
				return it->index;
			}
		}
		return 0;
}

string Map::index2string(int index)
{
	for(vector<qrcoormsg>::iterator it=coorlist.begin();it!=coorlist.end();++it)
		{
			if(it->index==index)
			{
				return it->id;
			}
		}
	return std::string();
}

void Map::keepdate(){
		qrfile.open((qrname+".dat").c_str(),ios::out);
		for(vector<qrmsg>::iterator it=qrlist.begin();it!=qrlist.end();++it)
		{
			qrfile<<it->fid<<" "<<it->sid<<" "<<it->distance<<" "<<it->lineid<<" "<<it->angle<<" "<<endl;
		}
		mapfile.open((mapname+".dat").c_str(),ios::binary|ofstream::out);
		for(vector<mapmsg>::iterator it=maplist.begin();it!=maplist.end();++it)
		{
			mapfile<<it->x1<<" "<<it->y1<<" "<<it->x2<<" "<<it->y2<<" "<<endl;
			
		}
		qrcoordinatefile.open((qrcoordinatename+".dat").c_str(),ios::binary|ofstream::out);
		for(vector<qrcoormsg>::iterator it=coorlist.begin();it!=coorlist.end();++it)
		{
			qrcoordinatefile<<it->index<<" "<<it->id<<" "<<it->x<<" "<<it->y<<" "<<it->count<<" "<<endl;
		}
		mapfile.close();
		qrfile.close();
		qrcoordinatefile.close();
		
	}


vector<mapmsg> Map::remapmsg(){
	return maplist;
}



vector<qrcoormsg> Map::reqrcoormsg(){
	return coorlist;
}



vector<qrmsg> Map::reqrmsg(){
	return qrlist;
}



qrcoormsg Map::qrtocoor(string id){
		qrcoormsg remsg;
		for(vector<qrcoormsg>::iterator beg=coorlist.begin(),end=coorlist.end();beg!=end;++beg)
		{
			if(beg->id==id)
			{
				remsg=*beg;
				break;
			}
			/*else
			{
				remsg=*end;
			}*/
		}
		return remsg;
	}



qrmsg Map::lineto(int id){
		qrmsg remsg;
		for(vector<qrmsg>::iterator beg=qrlist.begin(),end=qrlist.end();beg!=end;++beg){
			if(beg->lineid==id)
			{
				remsg=*beg;
				break;
			}
			/*else
			{
				remsg=*end;
			}*/
		}
		return remsg;
	}



qrmsg Map::qrto(int id1,int id2){
		qrmsg remsg;
		for(vector<qrmsg>::iterator beg=qrlist.begin(),end=qrlist.end();beg!=end;++beg){
			if((id1==beg->fid)&&(id2==beg->sid))
			{
				remsg=*beg;
				break;
			}
			/*else
			{
				remsg=*end;
			}*/
		}
		return remsg;
	}
void Map::reset(){
	qrfile.open((qrname+".dat").c_str(),ios::out);
	mapfile.open((mapname+".dat").c_str(),ios::binary|ofstream::out);
	qrcoordinatefile.open((qrcoordinatename+".dat").c_str(),ios::binary|ofstream::out);
	mapfile.close();
	qrfile.close();
	qrcoordinatefile.close();
	vector<qrmsg> qrlist_temp;
	vector<mapmsg> maplist_temp;
	vector<qrcoormsg> coorlist_temp;
	qrlist=qrlist_temp;
	maplist=maplist_temp;
	coorlist=coorlist_temp;
	alllineid=0;
	qrindex=0;
}
