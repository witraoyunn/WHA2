//#include <errno.h>
//#include <time.h>

#include "CANSocketStack.h"
#include "log4z.h"

using namespace std;

CANSocketStack g_can_dev[CAN_PORT_NUM];

CANSocketStack::CANSocketStack()
{
	sCanRecvListHandle = NULL;
}

CANSocketStack::~CANSocketStack()
{
	can_close();
}


// ������Ӧ��CAN�ڵ��������е�λ��
// ����ֵ������Ӧ��CAN�ڵ���ڣ��򷵻�CAN�ڵ��������еĽڵ��ַ�����򷵻�NULL
sCanRecvList_t* CANSocketStack::CanNodeSearch(const sCanRecvList_t *pHeadNode, sCanData_t *sCanData)
{
	sCanRecvList_t *pNode = NULL;
 
	if(sCanData == NULL)
	{
		return NULL;
	}
 
	pNode = (sCanRecvList_t *)pHeadNode->pNext; // ͷ�ڵ㲻������ݣ���������CAN�ڵ�ʱ�ӵ�һ���ڵ㿪ʼ
 
	while(pNode != NULL)
	{
		if(pNode->pBuf->deviceIdx == sCanData->deviceIdx && pNode->pBuf->cellIdx == sCanData->cellIdx && 
			pNode->pBuf->identCode == sCanData->identCode && pNode->pBuf->nodeType == sCanData->nodeType)
		{
			return pNode;
		}

		pNode = pNode->pNext;
	}

	return NULL;
}
 
// ��������ͷ�ڵ㣬ͷ�ڵ㲻�������
// ����ֵ�������ɹ��򷵻�ͷ�ڵ��ַ�����򷵻�NULL
sCanRecvList_t* CANSocketStack::ListCreate(void)
{
	sCanRecvList_t *head = NULL;
 
	head = new sCanRecvList_t;
	if(head == NULL)
	{
		return NULL;
	}

	head->pBuf = NULL;
	head->pNext = NULL;
 
	return head;
}
 
// ����һ������ڵ�
// ����ֵ�������ɹ����ؽڵ��ַ�����򷵻�NULL
sCanRecvList_t* CANSocketStack::ListNodeCreate(sCanData_t *sCanData)
{
	sCanRecvList_t *node = NULL;
 
	if(!sCanData)
	{
		return NULL; // �����쳣
	}
 
	if(sCanData->frameSeq != 1)
	{
		return NULL; // ֡��Ų�Ϊ1��˵������ͷ֡
	}
 
	node = new sCanRecvList_t;
	
	if(node == NULL)
	{
		return NULL;
	}
 
	node->pNext = NULL;
	node->pBuf = new sCanRecvData_t;
	if(node->pBuf == NULL)
	{
		delete node;
		node = NULL;
		return NULL;
	}
	 
	node->pBuf->rxLen = sCanData->txLen;
	node->pBuf->deviceIdx = sCanData->deviceIdx;
	node->pBuf->cellIdx = sCanData->cellIdx;
	node->pBuf->identCode = sCanData->identCode;
	node->pBuf->nodeType = sCanData->nodeType;
	
	node->pBuf->rxTotalLen = sCanData->Buf[3];
	node->pBuf->rxTotalLen = (node->pBuf->rxTotalLen << 8) + sCanData->Buf[2];
	node->pBuf->rxTotalLen += 4; //֡ͷ֡β4���ֽ�
	 
	node->pBuf->pBuf = new unsigned char[node->pBuf->rxTotalLen];
	if(node->pBuf->pBuf == NULL)
	{
		delete node->pBuf;
		node->pBuf = NULL;
		delete node;
		node = NULL;
		return NULL;
	}

	memcpy(node->pBuf->pBuf,sCanData->Buf,sCanData->txLen);
	 
	return node;
}
 
//���������е�ָ���ڵ㡣��ָ���ڵ����ΪNULLʱ����ʾ����β�ڵ�
//����ֵ������β�ڵ��ַ������Ϊ�ջ���û���ҵ�ʱ����NULL
sCanRecvList_t* CANSocketStack::ListNodeSearch(const sCanRecvList_t *pHeadNode, const sCanRecvList_t *pSearchNode)
{
	sCanRecvList_t *pNode = (sCanRecvList_t *)pHeadNode;
	 
	if(pNode == NULL)
	{
		return NULL;
	}
	 
	if(pSearchNode == NULL)
	{
		while(pNode->pNext != NULL)
		{
			pNode = pNode->pNext; // ����β�ڵ�
		}
	}
	else
	{
		while(pNode != pSearchNode)
		{
			pNode = pNode->pNext; // ����ָ���ڵ�
			if(pNode == NULL)
			{
				return NULL;
			}
		}
	}
 
	return pNode;
}
 
// �������ĩβ����һ���½ڵ�
void CANSocketStack::ListNodeInsert(const sCanRecvList_t *pHeadNode, sCanRecvList_t * const pNewNode)
{
	sCanRecvList_t *pNode = NULL;
 
	if(pHeadNode == NULL || pNewNode == NULL)
	{
		return;
	}
 
	pNode = ListNodeSearch(pHeadNode, NULL); // ����β�ڵ�
	if(pNode != NULL)
	{
		pNewNode->pNext = NULL;
		pNode->pNext = pNewNode; // ������ĩβ����һ���½ڵ�
	}
}
 
//ɾ��ָ���ڵ�
void CANSocketStack::ListNodeDelete(const sCanRecvList_t *pHeadNode, sCanRecvList_t *pDeleteNode)
{
	sCanRecvList_t *pLastNode = (sCanRecvList_t *)pHeadNode;
 
	if(pHeadNode == NULL || pDeleteNode == NULL)
	{
		return;
	}
 
	//����ɾ���ڵ����һ���ڵ�
	while(pLastNode->pNext != pDeleteNode)
	{
		pLastNode = pLastNode->pNext;
	}

	if(pLastNode != NULL)
	{
		//ɾ���ڵ�
		pLastNode->pNext = pDeleteNode->pNext;

		//�ͷ��ڴ棬ע���ͷ�˳��
		delete[] pDeleteNode->pBuf->pBuf;
		pDeleteNode->pBuf->pBuf = NULL;
 
		delete pDeleteNode->pBuf;
		pDeleteNode->pBuf = NULL;
		pDeleteNode->pNext = NULL;

		delete pDeleteNode;
		pDeleteNode = NULL;
	}
}
 
//���� CAN ����֡���ݣ����Է�֡���ݽ������
// p��RxMsg ����������ָ��(RxMsg.Data)
// len����Ч�ֽ���(RxMsg.DLC)
// extId����չ֡ID(RxMsg.ExtId)
// ����ֵ��0=succ��1=data error��2=memory error
unsigned char CANSocketStack::CanRecvDataProcess(const void *p, unsigned char len, unsigned int extId)
{
	unsigned char i;
	uExtId_t uextId;
	sCanData_t sCanData;
	sCanRecvList_t *pNode = NULL;
	//static sCanRecvList_t *pHeadNode = NULL; // ����һ��˫������
	unsigned char *pBuf = (unsigned char *)p;
 
	if(p == NULL || len < 1)
	{
		return 1; // �����쳣
	}

	//֡��ʶ����ʽת��
	uextId.extId = extId;
	sCanData.deviceIdx = uextId.atr.deviceIdx;
	sCanData.cellIdx = uextId.atr.cellIdx;
	sCanData.frameSeq = uextId.atr.frameSeq;
	sCanData.identCode = uextId.atr.identCode;
	sCanData.nodeType = uextId.atr.nodeType;
	sCanData.txLen = len;
	//for(i = 0; i < sizeof(sCanData.Buf); i++)  //����TODO
	for(i = 0; i < len; i++)  
	{
		sCanData.Buf[i] = pBuf[i];
	}

	//��������Ƿ���ڣ������ھʹ���
	if(sCanRecvListHandle == NULL)
	{
		sCanRecvListHandle = ListCreate();	//����Ϊ�վʹ�������
		if(sCanRecvListHandle == NULL)
		{
			return 2;	//������ʧ�ܵ�ԭ��ֻ���ڴ�����ʧ��
		}
	}
 
	//���ڵ��Ƿ���ڣ������ھʹ���
	pNode = CanNodeSearch(sCanRecvListHandle, &sCanData);
	if(pNode == NULL)
	{
		if(len > 3)		//��1֡��������ҪΪ4�ֽ�
		{
			unsigned short totalLen;
			
			totalLen = pBuf[3];
			totalLen = (totalLen << 8) + pBuf[2];

			if(totalLen <= 4)	//��֡�ܳ�С��8����Ҫ��֡
			{
				CanReadCache(sCanData.cellIdx+1,sCanData.deviceIdx+1,sCanData.identCode,pBuf,len);
				return 0;
			}

			pNode = ListNodeCreate(&sCanData); // ����һ���½ڵ�
			if(pNode == NULL)
			{
				return 2;
			}

			ListNodeInsert(sCanRecvListHandle, pNode); // ����������ӽڵ�
		}
	}
	else
	{
		//֡���ݺϷ�����֤
		unsigned short index = sCanData.frameSeq;
		if((index - 1) * 8 != pNode->pBuf->rxLen)
		{
			return 0; // ֡��Ų���ȷ��ֱ�Ӷ���
		}

		//������ڵ����������
		for(i = 0; i < sCanData.txLen; i++)
		{
			pNode->pBuf->pBuf[pNode->pBuf->rxLen++] = sCanData.Buf[i];
		}
	 
		//������ͨ���ص��������ݸ�Ӧ�ò�
		if(pNode->pBuf->rxLen == pNode->pBuf->rxTotalLen)
		{
			CanReadCache(sCanData.cellIdx+1,sCanData.deviceIdx+1,sCanData.identCode,pNode->pBuf->pBuf, pNode->pBuf->rxLen);
			ListNodeDelete(sCanRecvListHandle, pNode);
		}
	}
	return 0;
}
 

// ��CAN���߷���һ֡����
// ����ֵ��0=succ��1=data error��2=timeout
unsigned char CANSocketStack::CanSendFrame(const void *p, unsigned char len)
{
	uExtId_t uextId;
	sCanData_t *sCanData = (sCanData_t *)p;

	if(sCanData == NULL)
	{
		return 1;
	}

	//֡��ʶ����ʽת��
	uextId.atr.direction = 0;
	uextId.atr.transmode = 1;
	uextId.atr.deviceIdx = sCanData->deviceIdx;
	uextId.atr.cellIdx = sCanData->cellIdx;
	uextId.atr.frameSeq = sCanData->frameSeq;
	uextId.atr.identCode = sCanData->identCode;
	uextId.atr.nodeType = sCanData->nodeType;

 	uextId.extId |= 0x80000000;	//����֡��ʽ��־λ��չ֡
	
	//��������
	return CanWrite(sCanData->Buf, sCanData->txLen, uextId.extId);
}
 
// �� CAN ���߷�������
// dst_addr��Ŀ��ID
// src_addr������ID
// dev_type����������
// priority�����ȼ�
// p����������ָ��
// len�������ֽ���(���Ȳ���)
// ����ֵ��0=succ��1=error
unsigned char CANSocketStack::CanSendData(uint8_t cellNo,uint8_t deviceNo, uint8_t identCode,const void *p, unsigned int len)
{
	sCanData_t sCanData;
	unsigned int i = 0;
	unsigned int FrameCount = 0;
	unsigned char *pBuf = (unsigned char *)p;

	if(p == NULL || len < 1)
	{
		return 1;
	}

	sCanData.deviceIdx = deviceNo-1;
	sCanData.cellIdx = cellNo-1;
	sCanData.identCode = identCode;
	sCanData.nodeType = NODE_TYPE_JN910_SWITCH;

	//����֡
	FrameCount = len / 8;
	for(i = 0; i < FrameCount; i++)
	{
		unsigned char k;

		// ֡���
		sCanData.frameSeq = (unsigned char)(i + 1);

		// ֡����
		for(k = 0; k < 8; k++)
		{
			sCanData.Buf[k] = pBuf[i * 8 + k];
		}

		sCanData.txLen = 8;
		CanSendFrame(&sCanData, sizeof(sCanData));
		usleep(20);
	}

	//������һ֡�Ƿ񱻷���
	if((len % 8) != 0)
	{
		//֡���
		sCanData.frameSeq = (unsigned char)(FrameCount + 1);

		//֡����
		for(i = 0; i < len % 8; i++)
		{
			sCanData.Buf[i] = pBuf[FrameCount * 8 + i];
		}

		sCanData.txLen = i;
		CanSendFrame(&sCanData, sizeof(sCanData));
		usleep(20);
	}
 
	return 0;
}

void CANSocketStack::OSSemPend(pthread_mutex_t *mutex)
{
	pthread_mutex_lock(mutex);
}

void CANSocketStack::OSSemPost(pthread_mutex_t *mutex)
{
	pthread_mutex_unlock(mutex);
}

bool CANSocketStack::RecvDataTransfer(uint8_t cellNo,std::vector<recvDataCAN_t> &recvPack)
{
	if(recv_vector[cellNo-1].size() > 0)
	{
		OSSemPend(&can_recv_mutex[cellNo-1]);
		recvPack.swap(recv_vector[cellNo-1]);
		OSSemPost(&can_recv_mutex[cellNo-1]);

		return true;
	}

	return false;
}

bool CANSocketStack::SendDataTransfer(uint8_t cellNo,std::vector<recvDataCAN_t> &sendPack)
{
	if(send_vector[cellNo-1].size() > 0)
	{
		OSSemPend(&can_send_mutex[cellNo-1]);
		sendPack.swap(send_vector[cellNo-1]);
		OSSemPost(&can_send_mutex[cellNo-1]);

		return true;
	}

	return false;
}

int CANSocketStack::can_socket_init(const char *can_name)
{
    int can_fd = -1;
    struct ifreq ifr;
    struct sockaddr_can addr;
    /*�����׽���*/
    can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_fd < 0)
    {
        return can_fd;
    }
    /*ָ�� can �豸*/
    strcpy(ifr.ifr_name, can_name);
    ioctl(can_fd, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

#if 0 
    /*�رջػ�ģʽ*/
    int loopback = 0;   /* 0 = disabled, 1 = enabled (default) */
    setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
 
    /*�ر������Է�*/
    int recv_own_msgs = 0; /* 0 = disabled (default), 1 = enabled */
    setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));
#endif
    /*���׽����� can0 ��*/
    bind(can_fd, (struct sockaddr *)&addr, sizeof(addr));
    return can_fd;
}

int CANSocketStack::can_init(const char *can_name,int bitrate)
{
	//int ret;
	char command[64];
	
    sprintf(command, "ifconfig %s down", can_name);
    system(command);

	sprintf(command, "/sbin/ip link set %s type can bitrate %d", can_name,bitrate);
    system(command);

	sprintf(command, "ifconfig %s up", can_name);
    system(command);

	sprintf(command, "echo 1024 > /sys/class/net/%s/tx_queue_len", can_name);
    system(command);
	
	usleep(10000);
	
    /*bind can �˿�*/
    m_can_fd = can_socket_init(can_name);
    if (m_can_fd < 0)
    {
        LOGFMTD("can socket create failed!");
        return -1;
    }

 #if 0
    /*����epoll�¼�����*/
    ret = add_epoll_watch(m_can_fd);
    if (ret < 0)
    {
        close(m_can_fd);
        printf("can socket epoll create failed!\r\n");
        return -1;
    }
#endif

	for(int i = 0; i < MAX_CELLS_NBR; i++)
	{
		pthread_mutex_init(&can_recv_mutex[i], NULL);
		pthread_mutex_init(&can_send_mutex[i], NULL);
	}
 
    LOGFMTD("can init success, can_fd: %d !", m_can_fd);
    return 0;
}
 

void CANSocketStack::can_close()
{
	for(int i = 0; i < MAX_CELLS_NBR; i++)
	{
		pthread_mutex_destroy(&can_recv_mutex[i]);
		pthread_mutex_destroy(&can_send_mutex[i]);
	}
#if 0
    if (can_thread > 0)
    {
        void *recycle;
        /*�����߳���Դ  */
        pthread_join(can_thread, &recycle);
        printf("epoll thread destroy!\r\n");
    }

    /*�ر�epoll*/
    if (m_epoll_entity.m_epollfd > 0)
    {
        close(m_epoll_entity.m_epollfd);
        printf("epoll  close!\r\n");
    }
#endif

    /*�ر�CAN����*/
    if (m_can_fd > 0)
    {
        close(m_can_fd);
    }
}


int CANSocketStack::return_can_fd()
{
	if(m_can_fd > 0)
	{
		return m_can_fd;
	}

	return -1;
}

int CANSocketStack::CanWrite(unsigned char * buf,unsigned char len, unsigned int extId)
{
    int tran_count_ret = 0;
    struct can_frame tx_frame;
 
    bzero(&tx_frame, sizeof(tx_frame));
    tx_frame.can_id = extId;
    tx_frame.can_dlc = len;
    memcpy(tx_frame.data, buf, len);
 
    tran_count_ret = write(m_can_fd, &tx_frame, sizeof(tx_frame));
    if (tran_count_ret != sizeof(tx_frame))
    {
        LOGFMTD("canfd:%d send failed!",m_can_fd);
    }

	//LOGFMTD("can_id:%08x can_dlc:%d send-data: %s",extId,len,hexstring(buf, len).c_str());
	
    return tran_count_ret;
}

void CANSocketStack::CanReadCache(uint8_t cellNo,uint8_t deviceNo,uint8_t identCode,unsigned char * buf,unsigned int len)
{
	recvDataCAN_t recv_tmp;

	memcpy(recv_tmp.data,buf,len);
	recv_tmp.len = len;
	recv_tmp.cellNo = cellNo;
	recv_tmp.deviceNo = deviceNo;
	recv_tmp.identCode = identCode;
	
	OSSemPend(&can_recv_mutex[cellNo-1]);
	recv_vector[cellNo-1].push_back(recv_tmp);
	OSSemPost(&can_recv_mutex[cellNo-1]);
}

void CANSocketStack::CanWriteCache(uint8_t cellNo,uint8_t deviceNo,uint8_t identCode,unsigned char * buf,unsigned int len)
{
	recvDataCAN_t send_tmp;

	memcpy(send_tmp.data,buf,len);
	send_tmp.len = len;
	send_tmp.cellNo = cellNo;
	send_tmp.deviceNo = deviceNo;
	send_tmp.identCode = identCode;
	
	OSSemPend(&can_send_mutex[cellNo-1]);
	send_vector[cellNo-1].push_back(send_tmp);
	OSSemPost(&can_send_mutex[cellNo-1]);
}

#if 0
int CANSocketStack::add_epoll_watch(int can_fd)
{
	int ret;

	ret = m_epoll_entity.create(1);
	if(ret < 0)
	{
		return -1;
	}

	ret = m_epoll_entity.addfd(can_fd,EPOLLIN);
	if(ret < 0)
	{
		return -1;
	}

	return ret;
}
#endif

#if 0
void* CANSocketStack::can_recive_task(void* para)
{
    int nfds;
    int timeout = 5;
    uint16_t nbytes;
    struct can_frame rx_frame;

	int fd = 0;
	unsigned int iEvent = 0;

    CANSocketStack* canstack = (CANSocketStack*)para;
    while (1)
    {
		nfds = canstack->m_epoll_entity.waitEvents(timeout);
		
        if (nfds < 0)
        {
			printf("epoll wait error!\r\n");
        }

        for (int i = 0; i < nfds; i++)
        {
			if(canstack->m_epoll_entity.getEvents(fd,iEvent))
			{
				if (iEvent & EPOLLIN)
		        {
		            nbytes = read(fd, &rx_frame, sizeof(rx_frame));
		            if (nbytes > 0)
		            {
		            	canstack->CanRecvDataProcess(rx_frame.data,rx_frame.can_dlc,rx_frame.can_id);
		            }
		        }
			}  
        }
    }
	return NULL;
}
#endif

