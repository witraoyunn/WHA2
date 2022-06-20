/**************************************************************
* Copyright (C) 2018, 上海鼎格信息科技有限公司
* All rights reserved.
* 产品号 :
* 所属组件 :GN
* 模块名称 :GNMessager
* 文件名称 :GNMessager.h
* 概要描述 :
* 版本      日期         作者    内容
* V1.0    2018-05-18     zhujs     …
***************************************************************/
#ifndef GNMESSAGER_H_
#define GNMESSAGER_H_

#include <string>
#include <string.h>
#include "zmq.hpp"

extern zmq::context_t g_context;

template <typename DATA_type>
class GNMessager
{
private:
    zmq::socket_t *m_socket;
    std::string    m_filter;
	bool           m_isConnected = false;

    typedef struct
    {
        char filter[32];
        DATA_type data;
    } MSG_DATA_STRUCT;

public:
    GNMessager(uint16_t type = ZMQ_REP)
    {
        m_filter.clear();
        m_socket = new zmq::socket_t(g_context, type);
		int nNetTimeout=0;
		//发送时立即返回
		m_socket->setsockopt(ZMQ_SNDTIMEO,(char *)&nNetTimeout,sizeof(int));
		// 开启TCP保活机制，防止网络连接因长时间无数据而被中断
		int tcp_keep_alive = 1;
		m_socket->setsockopt(ZMQ_TCP_KEEPALIVE, &tcp_keep_alive, sizeof(tcp_keep_alive));
		// 网络连接空闲1min即发送保活包
		int tcp_keep_idle = 60;
		m_socket->setsockopt(ZMQ_TCP_KEEPALIVE_IDLE, &tcp_keep_idle, sizeof(tcp_keep_idle));
		int tcp_keep_cnt = 3;
		m_socket->setsockopt(ZMQ_TCP_KEEPALIVE_CNT, &tcp_keep_cnt, sizeof(tcp_keep_cnt));
		int tcp_keep_intvl = 5;
		m_socket->setsockopt(ZMQ_TCP_KEEPALIVE_INTVL, &tcp_keep_intvl, sizeof(tcp_keep_intvl));
    }

    ~GNMessager()
    {
        if (m_socket != 0)
        {
            delete m_socket;
        }
    }

    void set_recv_filter(std::string filter)
    {
        m_filter = filter;
        m_socket->setsockopt(ZMQ_SUBSCRIBE, filter.c_str(), filter.size());
    }

    void bind(std::string address)
    {
        m_socket->bind(address.c_str());
		m_isConnected = true;
    }

    void connect(std::string address)
    {
        m_socket->connect(address.c_str());
		m_isConnected = true;
    }

    void send(DATA_type &data, int flags = 0)
    {
#ifdef OFFLINE_PROTECTION
		if (!m_isConnected)  return;
#endif		
        int size = sizeof(DATA_type);
        zmq::message_t message(size);
        memcpy(message.data(), &data, size);

        m_socket->send(message, flags);
    }

    void send(std::string filter, DATA_type &data, int flags = 0)
    {
#ifdef OFFLINE_PROTECTION
		if (!m_isConnected)	return;
#endif		
		
        MSG_DATA_STRUCT msg_data = MSG_DATA_STRUCT();

        strcpy(msg_data.filter, filter.c_str());
        msg_data.data = data;

        int size = sizeof(MSG_DATA_STRUCT);

        zmq::message_t message(size);
        memcpy(message.data(), &msg_data, size);
        m_socket->send(message, flags);
    }

    void send(std::string &str, int flags = 0)
    {
#ifdef OFFLINE_PROTECTION
		if (!m_isConnected)	return;
#endif		
        m_socket->send(str.c_str(), str.size(), flags);
    }

    bool recevie(DATA_type& data, int flags = 0)
    {
        bool result = false;
#ifdef OFFLINE_PROTECTION
		if (!m_isConnected)	return result;
#endif		
		
        zmq::message_t message;
        if(m_socket->recv(&message, flags))
        {
            if(m_filter.empty())
            {
                int size = sizeof(DATA_type);
                memcpy(&data, message.data(), size);
            }
            else
            {
                MSG_DATA_STRUCT msg_data = MSG_DATA_STRUCT();
                int size = sizeof(msg_data);

                memcpy(&msg_data, message.data(), size);

                data = msg_data.data;

            }
            result = true;
        }

        return result;
    }

    bool recevie(std::string &str, int flags = 0)
    {
        bool result = false;
#ifdef OFFLINE_PROTECTION
		if (!m_isConnected)	return result;
#endif		
		
        zmq::message_t message;
        if(m_socket->recv(&message, flags))
        {
            result = true;
            int size = message.size();
            str = std::string((char*)message.data()).substr(0, size);
        }
        else
        {
            str.clear();
        }

        return result;
    }

	void connectionStatusSet(bool active)
    {
		m_isConnected = active;
	}
};

#endif //GNMESSAGER_H_

