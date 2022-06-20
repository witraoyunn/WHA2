#include "CANSendTask.h"

using namespace std;


CANSendTask::CANSendTask(int can_index)
{
	m_can_idx = can_index;
}

CANSendTask::~CANSendTask()
{

}

thread CANSendTask::run()
{
	return thread(&CANSendTask::work, this);
}

void CANSendTask::work()
{
	//int ret;
	
	while(1)
	{	
		msleep(10);
		
		for(uint8_t i = 0; i < MAX_CELLS_NBR; i++)
		{
			vector<recvDataCAN_t> sendPack;
			
			if(g_can_dev[m_can_idx].SendDataTransfer(i+1,sendPack))
			{
				for(uint32_t j = 0; j < sendPack.size(); j++)
				{
					g_can_dev[m_can_idx].CanSendData(sendPack[j].cellNo,sendPack[j].deviceNo,sendPack[j].identCode,sendPack[j].data,sendPack[j].len);
				}
			}
		}
	}
}


