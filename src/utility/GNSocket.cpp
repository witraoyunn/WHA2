/**************************************************************
* Copyright (C) 2018, 上海鼎格信息科技有限公司
* All rights reserved.
* 产品号 :
* 所属组件 :GN
* 模块名称 :GNSocket
* 文件名称 :GNSocket.cpp
* 概要描述 :

* 历史记录 :
* 版本      日期         作者    内容
* V1.0    2018-05-18     zhujs     …
***************************************************************/
#include<stdlib.h>
#include<stdio.h>
#include<string.h>
#include <errno.h>
#include "log4z.h"
#include "GNSocket.h"

#ifdef WIN32
#include <windows.h>

GNSocket::InitWSA GNSocket::DummyObject;
#define MSG_DONTWAIT 0
#define EAGAIN WSAEWOULDBLOCK
#define errno WSAGetLastError()
#define close_socket(x) closesocket(x)
#else
#define close_socket(x) close(x)
#endif

#define RECEIVE_BUFFER_MAX_LENGTH   256

GNSocket::GNSocket()
{}

void GNSocket::connect_server(const char *ip_addr, unsigned port)
{
	if ((m_sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		LOGFMTE("Exception:Socket create failed!\n");
		throw - 200;
	}
	struct timeval timeout;
	timeout.tv_sec	= 0;
	timeout.tv_usec = 50000;
	setsockopt(m_sockfd, SOL_SOCKET, SO_RCVTIMEO, (void *)&timeout, sizeof(struct timeval));
	setsockopt(m_sockfd, SOL_SOCKET, SO_SNDTIMEO, (void *)&timeout, sizeof(struct timeval));

	sprintf(m_sock_addr, "%s:%d", ip_addr, port);

	struct sockaddr_in client_addr;
	client_addr.sin_family = AF_INET;
	client_addr.sin_port = htons(port);
	client_addr.sin_addr.s_addr = inet_addr(ip_addr);

	if (connect(m_sockfd, (struct sockaddr*)&client_addr, sizeof(client_addr)) < 0)
	{
		LOGFMTE("Exception:Connect fail! Address[%s]\n", m_sock_addr);
		close_socket(m_sockfd);
		throw - 201;
	}
}

void GNSocket::disconnect()
{
    close_socket(m_sockfd);
}


void GNSocket::set_send_timeout(int millisecond)
{
	int rtn = setsockopt(m_sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&millisecond, sizeof(millisecond));
	if (rtn != 0)
	{
		LOGFMTE("Exception:sent send timeout failed,  Address[%s] return code:%d  errno:%d\n", m_sock_addr, rtn, errno);
		throw - 202;
	}
}

void GNSocket::set_recv_timeout(int millisecond)
{
	int rtn = setsockopt(m_sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&millisecond, sizeof(millisecond));
	if (rtn != 0)
	{
		LOGFMTE("Exception:sent receive timeout failed, Address[%s] return code:%d  errno:%d\n", m_sock_addr, rtn, errno);
		throw - 202;
	}
}

void GNSocket::set_async_mode()
{
    int rtn = 0;
#ifdef WIN32
    unsigned long flag = 1;
    ioctlsocket (m_sockfd, FIONBIO, &flag);
#else
    rtn = fcntl(m_sockfd, F_SETFL, fcntl(m_sockfd, F_GETFL, 0)|O_NONBLOCK);
#endif
    if (rtn < 0)
    {
        LOGFMTE("Exception:Socket fcntl fail! Address[%s]\n", m_sock_addr);
        throw - 203;
    }
}

void GNSocket::send_msg(std::string &req_msg)
{
	int rtn = send(m_sockfd, req_msg.c_str(), req_msg.size(), 0);
	if (rtn != (int)req_msg.size())
	{
		LOGFMTE("Exception:send string failed, Address[%s] req size:%d, return code:%d  errno:%d", m_sock_addr, (int)req_msg.size(), rtn, errno);
		throw - 204;
	}
}

void GNSocket::send_msg(char *msg, int size)
{
	int rtn = send(m_sockfd, msg, size, 0);
	if (rtn != size)
	{
		LOGFMTE("Exception:send char* failed, Address[%s] req size:%d, return code:%d  errno:%d", m_sock_addr, size, rtn, errno);
		throw - 204;
	}
}

std::string GNSocket::recv_msg()
{
	char buf[RECEIVE_BUFFER_MAX_LENGTH] = {'\0'};
	int rtn = recv(m_sockfd, buf, RECEIVE_BUFFER_MAX_LENGTH, 0);
	if (rtn <= 0)
	{
		LOGFMTE("Exception:Receive string failed, Address[%s] return code:%d  errno:%d\n", m_sock_addr, rtn, errno);
		throw - 205;
	}

	return std::string(buf);
}

int GNSocket::recv_msg(char *msg, int length)
{
	int rtn = recv(m_sockfd, msg, length, 0);
	if (rtn <= 0)
	{
		LOGFMTE("Exception:Receive char* failed, Address[%s] return code:%d  errno:%d\n", m_sock_addr, rtn, errno);
		throw - 205;
	}

	return rtn;
}

std::string GNSocket::async_recv_msg()
{
	char buf[RECEIVE_BUFFER_MAX_LENGTH] = {'\0'};
	int rtn = recv(m_sockfd, buf, RECEIVE_BUFFER_MAX_LENGTH, MSG_DONTWAIT);
	if (rtn < 0)
	{
	    if(errno != EAGAIN && errno != 9)
	    {
            LOGFMTE("Exception:Async Receive string failed, Address[%s] return code:%d  errno:%d\n", m_sock_addr, rtn, errno);
            throw - 205;
	    }
	}

	return std::string(buf);
}

int GNSocket::async_recv_msg(char *msg, int length)
{
	int rtn = recv(m_sockfd, msg, length, MSG_DONTWAIT);
	if (rtn < 0)
	{
	    if(errno != EAGAIN && errno != 9)
	    {
            LOGFMTE("Exception:Async Receive char* failed, Address[%s] return code:%d  errno:%d\n", m_sock_addr, rtn, errno);
            throw - 205;
	    }
	}

	return rtn;
}

void GNSocket::send_recv(std::string &req_msg, std::string &rpl_msg)
{
	send_msg(req_msg);
	rpl_msg = recv_msg();
}

