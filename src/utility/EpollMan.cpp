#include "EpollMan.h"

EpollMan::EpollMan()
{
	m_epollfd = -1;
	m_iEventNum = 0;
	m_iCurrEvtIdx = 0;
	m_maxfd= 0;

	for(int i=0; i<MAX_CHANNELS_NBR; i++)
	{
		m_epollEvents[i].data.fd = -1;
		m_epollEvents[i].events = 0;		
	}
}

EpollMan::~EpollMan()
{
	//if(m_epollEvents)
	//{
	//	delete [] m_epollEvents;
	//}

	if(m_epollfd >= 0)
	{
		close(m_epollfd);
	}
}

int EpollMan::create(int maxfd)
{
	m_maxfd = maxfd;
	m_epollfd = epoll_create(maxfd);
	if(m_epollfd < 0)
	{
		return -1;
	}

	//m_epollEvents = new epoll_event[maxfd];

	return 0;
}

int EpollMan::ctl(int fd,int epollAction, int flag)
{
	struct epoll_event evt;
	evt.events = flag;
	evt.data.fd = fd;

	int ret = epoll_ctl(m_epollfd, epollAction, fd, &evt);
	if(ret < 0)
	{
		return -1;
	}

	return 0;
}

int EpollMan::addfd(int fd, int flag)
{
	return ctl(fd,EPOLL_CTL_ADD,flag);
}

int EpollMan::deletefd(int fd)
{
	return ctl(fd,EPOLL_CTL_DEL,0);
}

int EpollMan::modifyfd(int fd, int flag)
{
	return ctl(fd,EPOLL_CTL_MOD, flag);
}

int EpollMan::waitEvents(int timeMs)
{
	m_iEventNum = epoll_wait(m_epollfd, m_epollEvents, m_maxfd, timeMs);
	m_iCurrEvtIdx = 0;

	return m_iEventNum;
}

bool EpollMan::getEvents(int &fd, unsigned int &iEvent)
{
	if(m_iCurrEvtIdx >= m_iEventNum)
	{
		return false;
	}

	struct epoll_event* curr_event = &m_epollEvents[m_iCurrEvtIdx++];
	fd = curr_event->data.fd;
	iEvent = curr_event->events;

	return true;
}



