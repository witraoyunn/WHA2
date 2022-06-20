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


// 搜索对应的CAN节点在链表中的位置
// 返回值：若相应的CAN节点存在，则返回CAN节点在链表中的节点地址，否则返回NULL
sCanRecvList_t* CANSocketStack::CanNodeSearch(const sCanRecvList_t *pHeadNode, sCanData_t *sCanData)
{
	sCanRecvList_t *pNode = NULL;
 
	if(sCanData == NULL)
	{
		return NULL;
	}
 
	pNode = (sCanRecvList_t *)pHeadNode->pNext; // 头节点不存放数据，所以搜索CAN节点时从第一个节点开始
 
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
 
// 创建链表头节点，头节点不存放数据
// 返回值：创建成功则返回头节点地址，否则返回NULL
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
 
// 创建一个链表节点
// 返回值：创建成功返回节点地址，否则返回NULL
sCanRecvList_t* CANSocketStack::ListNodeCreate(sCanData_t *sCanData)
{
	sCanRecvList_t *node = NULL;
 
	if(!sCanData)
	{
		return NULL; // 数据异常
	}
 
	if(sCanData->frameSeq != 1)
	{
		return NULL; // 帧序号不为1，说明不是头帧
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
	node->pBuf->rxTotalLen += 4; //帧头帧尾4个字节
	 
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
 
//查找链表中的指定节点。当指定节点参数为NULL时，表示搜索尾节点
//返回值：链表尾节点地址，链表为空或者没有找到时返回NULL
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
			pNode = pNode->pNext; // 搜索尾节点
		}
	}
	else
	{
		while(pNode != pSearchNode)
		{
			pNode = pNode->pNext; // 搜索指定节点
			if(pNode == NULL)
			{
				return NULL;
			}
		}
	}
 
	return pNode;
}
 
// 在链表的末尾插入一个新节点
void CANSocketStack::ListNodeInsert(const sCanRecvList_t *pHeadNode, sCanRecvList_t * const pNewNode)
{
	sCanRecvList_t *pNode = NULL;
 
	if(pHeadNode == NULL || pNewNode == NULL)
	{
		return;
	}
 
	pNode = ListNodeSearch(pHeadNode, NULL); // 搜索尾节点
	if(pNode != NULL)
	{
		pNewNode->pNext = NULL;
		pNode->pNext = pNewNode; // 在链表末尾插入一个新节点
	}
}
 
//删除指定节点
void CANSocketStack::ListNodeDelete(const sCanRecvList_t *pHeadNode, sCanRecvList_t *pDeleteNode)
{
	sCanRecvList_t *pLastNode = (sCanRecvList_t *)pHeadNode;
 
	if(pHeadNode == NULL || pDeleteNode == NULL)
	{
		return;
	}
 
	//查找删除节点的上一个节点
	while(pLastNode->pNext != pDeleteNode)
	{
		pLastNode = pLastNode->pNext;
	}

	if(pLastNode != NULL)
	{
		//删除节点
		pLastNode->pNext = pDeleteNode->pNext;

		//释放内存，注意释放顺序
		delete[] pDeleteNode->pBuf->pBuf;
		pDeleteNode->pBuf->pBuf = NULL;
 
		delete pDeleteNode->pBuf;
		pDeleteNode->pBuf = NULL;
		pDeleteNode->pNext = NULL;

		delete pDeleteNode;
		pDeleteNode = NULL;
	}
}
 
//接收 CAN 总线帧数据，并对分帧数据进行组包
// p：RxMsg 数据域数据指针(RxMsg.Data)
// len：有效字节数(RxMsg.DLC)
// extId：扩展帧ID(RxMsg.ExtId)
// 返回值：0=succ，1=data error，2=memory error
unsigned char CANSocketStack::CanRecvDataProcess(const void *p, unsigned char len, unsigned int extId)
{
	unsigned char i;
	uExtId_t uextId;
	sCanData_t sCanData;
	sCanRecvList_t *pNode = NULL;
	//static sCanRecvList_t *pHeadNode = NULL; // 创建一条双向链表
	unsigned char *pBuf = (unsigned char *)p;
 
	if(p == NULL || len < 1)
	{
		return 1; // 数据异常
	}

	//帧标识符格式转换
	uextId.extId = extId;
	sCanData.deviceIdx = uextId.atr.deviceIdx;
	sCanData.cellIdx = uextId.atr.cellIdx;
	sCanData.frameSeq = uextId.atr.frameSeq;
	sCanData.identCode = uextId.atr.identCode;
	sCanData.nodeType = uextId.atr.nodeType;
	sCanData.txLen = len;
	//for(i = 0; i < sizeof(sCanData.Buf); i++)  //长度TODO
	for(i = 0; i < len; i++)  
	{
		sCanData.Buf[i] = pBuf[i];
	}

	//检查链表是否存在，不存在就创建
	if(sCanRecvListHandle == NULL)
	{
		sCanRecvListHandle = ListCreate();	//链表为空就创建链表
		if(sCanRecvListHandle == NULL)
		{
			return 2;	//链表创建失败的原因只有内存申请失败
		}
	}
 
	//检查节点是否存在，不存在就创建
	pNode = CanNodeSearch(sCanRecvListHandle, &sCanData);
	if(pNode == NULL)
	{
		if(len > 3)		//第1帧数据至少要为4字节
		{
			unsigned short totalLen;
			
			totalLen = pBuf[3];
			totalLen = (totalLen << 8) + pBuf[2];

			if(totalLen <= 4)	//单帧总长小于8不需要组帧
			{
				CanReadCache(sCanData.cellIdx+1,sCanData.deviceIdx+1,sCanData.identCode,pBuf,len);
				return 0;
			}

			pNode = ListNodeCreate(&sCanData); // 创建一个新节点
			if(pNode == NULL)
			{
				return 2;
			}

			ListNodeInsert(sCanRecvListHandle, pNode); // 向链表中添加节点
		}
	}
	else
	{
		//帧数据合法性验证
		unsigned short index = sCanData.frameSeq;
		if((index - 1) * 8 != pNode->pBuf->rxLen)
		{
			return 0; // 帧序号不正确，直接丢弃
		}

		//向链表节点中添加数据
		for(i = 0; i < sCanData.txLen; i++)
		{
			pNode->pBuf->pBuf[pNode->pBuf->rxLen++] = sCanData.Buf[i];
		}
	 
		//将数据通过回调函数传递给应用层
		if(pNode->pBuf->rxLen == pNode->pBuf->rxTotalLen)
		{
			CanReadCache(sCanData.cellIdx+1,sCanData.deviceIdx+1,sCanData.identCode,pNode->pBuf->pBuf, pNode->pBuf->rxLen);
			ListNodeDelete(sCanRecvListHandle, pNode);
		}
	}
	return 0;
}
 

// 向CAN总线发送一帧数据
// 返回值：0=succ，1=data error，2=timeout
unsigned char CANSocketStack::CanSendFrame(const void *p, unsigned char len)
{
	uExtId_t uextId;
	sCanData_t *sCanData = (sCanData_t *)p;

	if(sCanData == NULL)
	{
		return 1;
	}

	//帧标识符格式转换
	uextId.atr.direction = 0;
	uextId.atr.transmode = 1;
	uextId.atr.deviceIdx = sCanData->deviceIdx;
	uextId.atr.cellIdx = sCanData->cellIdx;
	uextId.atr.frameSeq = sCanData->frameSeq;
	uextId.atr.identCode = sCanData->identCode;
	uextId.atr.nodeType = sCanData->nodeType;

 	uextId.extId |= 0x80000000;	//设置帧格式标志位扩展帧
	
	//发送数据
	return CanWrite(sCanData->Buf, sCanData->txLen, uextId.extId);
}
 
// 向 CAN 总线发送数据
// dst_addr：目标ID
// src_addr：本地ID
// dev_type：器件类型
// priority：优先级
// p：发送数据指针
// len：发送字节数(长度不限)
// 返回值：0=succ，1=error
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

	//数据帧
	FrameCount = len / 8;
	for(i = 0; i < FrameCount; i++)
	{
		unsigned char k;

		// 帧序号
		sCanData.frameSeq = (unsigned char)(i + 1);

		// 帧数据
		for(k = 0; k < 8; k++)
		{
			sCanData.Buf[k] = pBuf[i * 8 + k];
		}

		sCanData.txLen = 8;
		CanSendFrame(&sCanData, sizeof(sCanData));
		usleep(20);
	}

	//检查最后一帧是否被发送
	if((len % 8) != 0)
	{
		//帧序号
		sCanData.frameSeq = (unsigned char)(FrameCount + 1);

		//帧数据
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
    /*创建套接字*/
    can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_fd < 0)
    {
        return can_fd;
    }
    /*指定 can 设备*/
    strcpy(ifr.ifr_name, can_name);
    ioctl(can_fd, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

#if 0 
    /*关闭回环模式*/
    int loopback = 0;   /* 0 = disabled, 1 = enabled (default) */
    setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
 
    /*关闭自收自发*/
    int recv_own_msgs = 0; /* 0 = disabled (default), 1 = enabled */
    setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));
#endif
    /*将套接字与 can0 绑定*/
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
	
    /*bind can 端口*/
    m_can_fd = can_socket_init(can_name);
    if (m_can_fd < 0)
    {
        LOGFMTD("can socket create failed!");
        return -1;
    }

 #if 0
    /*开启epoll事件监听*/
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
        /*回收线程资源  */
        pthread_join(can_thread, &recycle);
        printf("epoll thread destroy!\r\n");
    }

    /*关闭epoll*/
    if (m_epoll_entity.m_epollfd > 0)
    {
        close(m_epoll_entity.m_epollfd);
        printf("epoll  close!\r\n");
    }
#endif

    /*关闭CAN外设*/
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

