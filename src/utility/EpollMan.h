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
EPOLLIN ：表示对应的文件描述符可以读
EPOLLOUT：表示对应的文件描述符可以写
EPOLLPRI：表示对应的文件描述符有紧急的数据可读
EPOLLERR：表示对应的文件描述符发生错误
EPOLLHUP：表示对应的文件描述符被挂断
EPOLLET： 表示对应的文件描述符有事件发生
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
