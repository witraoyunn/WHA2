/**************************************************************
* Copyright (C) 2018, 上海鼎格信息科技有限公司
* All rights reserved.
* 产品号 :
* 所属组件 :GN
* 模块名称 :GNSocket
* 文件名称 :GNSocket.h
* 概要描述 :

* 历史记录 :
* 版本      日期         作者    内容
* V1.0    2018-05-18     zhujs     …
***************************************************************/
#ifndef GNSOCKET_H
#define GNSOCKET_H

#include <string>

#ifdef WIN32
#include <winsock2.h>
#else
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#endif

class GNSocket
{
public:
    GNSocket();

    void connect_server(const char *ip_addr, unsigned port);
    void disconnect();

    void set_send_timeout(int millisecond);
    void set_recv_timeout(int millisecond);
    void set_async_mode();

    void send_msg(std::string &req_msg);
	void send_msg(char *msg, int size);

    std::string recv_msg();
    int recv_msg(char *msg, int length);

    std::string async_recv_msg();
    int async_recv_msg(char *msg, int length);

    void send_recv(std::string &req_msg, std::string &rpl_msg);

private:
    char m_sock_addr[32];
    int m_sockfd;

#ifdef WIN32
    static class InitWSA
    {
    public:
        InitWSA(){ WSADATA data; if(WSAStartup(MAKEWORD(2, 2), &data) != 0) throw -206; }
        ~InitWSA(){ WSACleanup(); }
    } DummyObject;
#endif
};

#endif //GNSOCKET_H

