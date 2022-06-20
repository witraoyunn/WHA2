#ifndef _CANSENDTASK_H_
#define _CANSENDTASK_H_

#include <string.h>
#include <vector>
#include <thread>
#include "CANSocketStack.h"


class CANSendTask
{
public:
	CANSendTask(){}
	CANSendTask(int can_index);
	~CANSendTask();

	std::thread run();
private:
	void work();

private:
	int m_can_idx;

};


#endif

