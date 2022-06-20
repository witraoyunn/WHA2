#ifndef _EPOLLMAN_H_
#define _EPOLLMAN_H_

#include <unistd.h>
#include <string.h>
//#include <errno.h>
#include <sys/epoll.h>
#include <assert.h>
#include <string>
#include "Configuration.h"


/*
EPOLLIN ����ʾ��Ӧ���ļ����������Զ�
EPOLLOUT����ʾ��Ӧ���ļ�����������д
EPOLLPRI����ʾ��Ӧ���ļ��������н��������ݿɶ�
EPOLLERR����ʾ��Ӧ���ļ���������������
EPOLLHUP����ʾ��Ӧ���ļ����������Ҷ�
EPOLLET�� ��ʾ��Ӧ���ļ����������¼�����
*/

class EpollMan
{
public:
	EpollMan();
	~EpollMan();
public:
	int create(int maxfd);
	int addfd(int fd, int flag);
	int deletefd(int fd);
	int modifyfd(int fd, int flag);

	int waitEvents(int timeMs);
	bool getEvents(int &fd, unsigned int &iEvent);

private:
	int ctl(int fd,int epollAction, int flag);

public:
	int m_epollfd;
private:
	struct epoll_event m_epollEvents[MAX_CHANNELS_NBR];
	int m_maxfd;

	int m_iEventNum;
	int m_iCurrEvtIdx;
};

#endif
