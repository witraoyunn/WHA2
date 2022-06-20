#ifndef _CANSOCKETSTACK_H_
#define _CANSOCKETSTACK_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/socket.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <deque>
#include "Type.h"
#include "EpollMan.h"

// Define NULL pointer value
#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *)0)
#endif
#endif

#define CAN_PORT_NUM		2

#define NODE_TYPE_JN95			1
#define NODE_TYPE_JN9B			2
#define NODE_TYPE_JN910_DCDC	3
#define NODE_TYPE_JN910_SWITCH	4
#define NODE_TYPE_TEMPBOX		5
#define NODE_TYPE_VOLTBOX		6


// 需求：CAN端口可能会收到来自多个节点的分帧数据，在多个节点的分帧数据穿插接收进来后，根据帧标识将多个节点的分帧数据整理合并.
// 思路：建立一条单向链表，链表的一个节点代表CAN总线的一个端点的一个完整数据包，接收到一个分帧数据后将其存储在对应的链表节点中.
 
typedef union
{
	unsigned int extId;
	struct
	{
		unsigned int direction : 1;	//方向
		unsigned int transmode : 1;	//广播0，点播1
		unsigned int deviceIdx : 5;	//设备号
		unsigned int cellIdx   : 3; //库号
		unsigned int frameSeq  : 8; //帧序
		unsigned int identCode : 8;	//特征码
		unsigned int nodeType  : 3; //节点类型
		unsigned int flag_nul  : 3; 
	}atr;
}uExtId_t;
 
#pragma pack(push, 1)
typedef struct _sCanData_t
{
	unsigned char deviceIdx; 	// 设备号
	unsigned char cellIdx; 		// 库号
	unsigned char frameSeq; 	// 帧序
	unsigned char identCode; 	// 特征码
	unsigned char nodeType;		// 节点类型
	unsigned char txLen; 		// 发送字节数
	unsigned char Buf[8]; 		// 发送缓存区
}sCanData_t;
 
typedef struct _sCanRecvData_t
{
	unsigned char deviceIdx; 	// 设备号
	unsigned char cellIdx; 		// 库号
	unsigned char frameSeq; 	// 帧序
	unsigned char identCode; 	// 特征码
	unsigned char nodeType;		// 节点类型

	unsigned char *pBuf; 		// 接收缓存
	unsigned short rxLen; 		// 当前已经接收的字节数
	unsigned short rxTotalLen; 	// 需要接收的总字节数
}sCanRecvData_t;

typedef struct _sCanRecvList_t
{
	struct _sCanRecvData_t *pBuf;
	struct _sCanRecvList_t *pNext;
}sCanRecvList_t;
#pragma pack(pop)

typedef struct _recvDataCAN_t
{
	unsigned char cellNo;		// 库号
	unsigned char deviceNo; 	// 设备号
	unsigned char identCode; 	// 特征码
	unsigned short len;
	unsigned char data[512];
}recvDataCAN_t;

class CANSocketStack
{
public:
	CANSocketStack();
	~CANSocketStack();

	int can_init(const char *can_name,int bitrate);
	void can_close();
	int return_can_fd();

	unsigned char CanRecvDataProcess(const void *p, unsigned char len, unsigned int extId);
	unsigned char CanSendData(uint8_t cellNo,uint8_t deviceNo, uint8_t identCode,const void *p, unsigned int len);
	void CanReadCache(uint8_t cellNo,uint8_t deviceNo,uint8_t identCode,unsigned char * buf,unsigned int len);
	void CanWriteCache(uint8_t cellNo,uint8_t deviceNo,uint8_t identCode,unsigned char * buf,unsigned int len);

	bool RecvDataTransfer(uint8_t cellNo,std::vector<recvDataCAN_t> &recvPack);
	bool SendDataTransfer(uint8_t cellNo,std::vector<recvDataCAN_t> &sendPack);

private:
	sCanRecvList_t *CanNodeSearch(const sCanRecvList_t *pHeadNode, sCanData_t *sCanData);
	sCanRecvList_t *ListCreate(void);
	sCanRecvList_t *ListNodeCreate(sCanData_t *sCanData);
	sCanRecvList_t *ListNodeSearch(const sCanRecvList_t *pHeadNode, const sCanRecvList_t *pSearchNode);
	void ListNodeInsert(const sCanRecvList_t *pHeadNode, sCanRecvList_t * const pNewNode);
	void ListNodeDelete(const sCanRecvList_t *pHeadNode, sCanRecvList_t *pDeleteNode);
	unsigned char CanSendFrame(const void *p, unsigned char len);
	int CanWrite(unsigned char * buf,unsigned char len, unsigned int extId);

	int can_socket_init(const char *can_name);
	
	void OSSemPend(pthread_mutex_t *mutex);
	void OSSemPost(pthread_mutex_t *mutex);
	//int add_epoll_watch(int can_fd);
	//static void* can_recive_task(void* para);
	
public:
	std::vector<recvDataCAN_t> recv_vector[MAX_CELLS_NBR];
	std::vector<recvDataCAN_t> send_vector[MAX_CELLS_NBR];
	
private:
	sCanRecvList_t *sCanRecvListHandle; 
	//EpollMan m_epoll_entity;
	//pthread_t can_thread;
	pthread_mutex_t can_recv_mutex[MAX_CELLS_NBR];
	pthread_mutex_t can_send_mutex[MAX_CELLS_NBR];
	int m_can_fd;
	
};

extern CANSocketStack g_can_dev[CAN_PORT_NUM];

#endif

