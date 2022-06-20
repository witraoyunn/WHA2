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


// ����CAN�˿ڿ��ܻ��յ����Զ���ڵ�ķ�֡���ݣ��ڶ���ڵ�ķ�֡���ݴ�����ս����󣬸���֡��ʶ������ڵ�ķ�֡��������ϲ�.
// ˼·������һ���������������һ���ڵ����CAN���ߵ�һ���˵��һ���������ݰ������յ�һ����֡���ݺ���洢�ڶ�Ӧ������ڵ���.
 
typedef union
{
	unsigned int extId;
	struct
	{
		unsigned int direction : 1;	//����
		unsigned int transmode : 1;	//�㲥0���㲥1
		unsigned int deviceIdx : 5;	//�豸��
		unsigned int cellIdx   : 3; //���
		unsigned int frameSeq  : 8; //֡��
		unsigned int identCode : 8;	//������
		unsigned int nodeType  : 3; //�ڵ�����
		unsigned int flag_nul  : 3; 
	}atr;
}uExtId_t;
 
#pragma pack(push, 1)
typedef struct _sCanData_t
{
	unsigned char deviceIdx; 	// �豸��
	unsigned char cellIdx; 		// ���
	unsigned char frameSeq; 	// ֡��
	unsigned char identCode; 	// ������
	unsigned char nodeType;		// �ڵ�����
	unsigned char txLen; 		// �����ֽ���
	unsigned char Buf[8]; 		// ���ͻ�����
}sCanData_t;
 
typedef struct _sCanRecvData_t
{
	unsigned char deviceIdx; 	// �豸��
	unsigned char cellIdx; 		// ���
	unsigned char frameSeq; 	// ֡��
	unsigned char identCode; 	// ������
	unsigned char nodeType;		// �ڵ�����

	unsigned char *pBuf; 		// ���ջ���
	unsigned short rxLen; 		// ��ǰ�Ѿ����յ��ֽ���
	unsigned short rxTotalLen; 	// ��Ҫ���յ����ֽ���
}sCanRecvData_t;

typedef struct _sCanRecvList_t
{
	struct _sCanRecvData_t *pBuf;
	struct _sCanRecvList_t *pNext;
}sCanRecvList_t;
#pragma pack(pop)

typedef struct _recvDataCAN_t
{
	unsigned char cellNo;		// ���
	unsigned char deviceNo; 	// �豸��
	unsigned char identCode; 	// ������
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

