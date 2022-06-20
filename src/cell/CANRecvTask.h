#ifndef _CANRECVTASK_H_
#define _CANRECVTASK_H_

#include <string.h>
#include <vector>
#include <thread>
#include "CANSocketStack.h"

class CANRecvTask
{
public:
	CANRecvTask();
	~CANRecvTask();

	std::thread run();
private:
	void work();
};


#endif

