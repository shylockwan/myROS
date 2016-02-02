#pragma once
namespace myros{
class StateDirector{
public:
	virtual bool onInit()=0;
	virtual ~StateDirector(){};
	virtual void stateUpdate()=0;
	virtual bool getState()=0;
private:

//StateExecutor* se_;
};
}
