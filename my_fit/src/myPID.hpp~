#pragma once
class myPID
{
public:
	myPID(const float & _objval,const float& _p=0,const float& _i=0,const float& _d=0)
		:objVal_(_objval),Kp_(_p),Ki_(_i),Kd_(_d),err_(0),err_1(0),err_2(0),sumErr_(0){};
	void updateErr(const float& _val){	err_2=err_1; err_1=err_; err_=objVal_-_val; };
	void setParam(const float& _p,const float& _i,const float& _d){ Kp_=_p; Ki_=_i; Kd_=_d;}
	float processPID(const float& _val,bool flag=0)//flag==0:位置式PID   //flag==1:增量式PID
	{
		updateErr(_val);
		if(0==flag)
		{
			sumErr_+=err_;
			return Kp_*err_+Ki_*sumErr_+Kd_*(err_-err_1);
		}
		if(1==flag)
		{
			return Kp_*(err_-err_1)+Ki_*err_+Kd_*(err_-2*err_1+2*err_2);
		}
		return 0;
			 
	}
protected:
	float err_,err_1,err_2;
	float sumErr_;
	float Kp_;
	float Ki_;
	float Kd_;
	float objVal_;
};
