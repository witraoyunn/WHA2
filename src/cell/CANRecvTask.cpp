#include "CANRecvTask.h"
#include "log4z.h"

using namespace std;


CANRecvTask::CANRecvTask()
{

}

CANRecvTask::~CANRecvTask()
{


}

thread CANRecvTask::run()
{
	return thread(&CANRecvTask::work, this);
}

void CANRecvTask::work()
{
	int ret;
	uint16_t nbytes;    
	struct can_frame rx_frame;
	struct timeval timeout;	
	fd_set readset;
	fd_set exset;
	int fds_max = 0;
	int fds[CAN_PORT_NUM];
	
	for(int i = 0; i < CAN_PORT_NUM; i++)
	{
		fds[i] = g_can_dev[i].return_can_fd();
	}
	
	while(1)
	{
		timeout.tv_sec = 0;
		timeout.tv_usec = 500;
		
		FD_ZERO(&readset);
		FD_ZERO(&exset);

		for(int i = 0; i < CAN_PORT_NUM; i++)
		{
			if(fds[i] > 0)
			{
				FD_SET(fds[i], &readset);			
				FD_SET(fds[i], &exset);

				if(fds[i] > fds_max)
				{
					fds_max = fds[i];
				}
			}
		}

		ret = select(fds_max+1, &readset, NULL, &exset, &timeout);

		if(ret > 0)
		{
			for(int i = 0; i < CAN_PORT_NUM; i++)
			{
				if(FD_ISSET(fds[i], &readset))
				{
					nbytes = read(fds[i], &rx_frame, sizeof(rx_frame));
		            if (nbytes > 0)
		            {
		            	g_can_dev[i].CanRecvDataProcess(rx_frame.data,rx_frame.can_dlc,rx_frame.can_id);
						//LOGFMTD("can_id:%08x can_dlc:%d recv-data: %s",rx_frame.can_id,rx_frame.can_dlc,hexstring(rx_frame.data, rx_frame.can_dlc).c_str());
		            }
				}
				else if(FD_ISSET(fds[i], &exset))
				{
					LOGFMTD("canfd:%d recv error",fds[i]);
				}
			}
		}
	}
}



