#include <string.h>
#include <sys/types.h>  
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <errno.h>
#include <time.h>

#include "zmq.hpp"
#include "log4z.h"
#include "Cabinet.h"
#include "Type.h"
#include "Dispatcher.h"
#include "MsgFactory.h"
#include "Configuration.h"
#include "FormatConvert.h"


static bool g_is_zmq_created = false;

using namespace std;


// 任务循环中需要做的事情
void Cabinet::run()
{
    try
    {
        int  cab_no = Configuration::instance()->cabinet_no();
        char filter[32] = {'\0'};
        int  valid_idx = sprintf(filter, TOPIC_CAB_FORMAT_STR, cab_no);

		// 创建ZMQ通讯
        try
        {
			// 与上位机的通讯
            MsgFactory::instance()->create_cab_alarm_pusher(Configuration::instance()->ext_alarm_socket());
            MsgFactory::instance()->create_cab_ext_suber(Configuration::instance()->ext_request_socket(), filter);
            MsgFactory::instance()->create_cab_puber(Configuration::instance()->int_cab_puber_socket());

			MsgFactory::instance()->create_cab_transmit_suber(Configuration::instance()->ext_transmit_suber_socket(),filter);
//#ifdef ZHONGYUAN_BTS
//			// 与BTS的通讯
//            MsgFactory::instance()->create_bts_suber(Configuration::instance()->bts_suber_socket(), filter);
//            MsgFactory::instance()->create_bts_pusher(Configuration::instance()->bts_pusher_socket());
//#endif
			g_is_zmq_created = true;
        }
        catch (zmq::error_t &e)
        {
            LOGFMTE("Create ZMQ communication error. code: %d, description:%s", e.num(), e.what());
            exit(-209);
        }
		
        for (;;)
        {
			msleep(80);

            try
            {
				// 接收上位机消息
                string receive_str;
			
                if (MsgFactory::instance()->cab_ext_suber().recevie(receive_str, ZMQ_DONTWAIT) || \
					MsgFactory::instance()->cab_transmit_suber().recevie(receive_str, ZMQ_DONTWAIT))
                {
                    LOGFMTD("Cabinet %d receive message: %s", cab_no, receive_str.c_str());
					// 解析消息
                    string json_str = receive_str.substr(valid_idx);
                    Dispatcher::instance()->publish(json_str);
                }
			}
            catch (int &e)
            {
                LOGFMTE("Cabinet exception code:%d!", e);
                string josn_str = FormatConvert::instance().alarm_to_string(e);
                MsgFactory::instance()->cab_alarm_pusher().send(josn_str);
            }
            catch (zmq::error_t &e)
            {
                LOGFMTE("ZMQ communication error. code: %d, description:%s", e.num(), e.what());
                string josn_str = FormatConvert::instance().alarm_to_string(-209);
                MsgFactory::instance()->cab_alarm_pusher().send(josn_str);
            }
        }

		// 销毁ZMQ通讯
        MsgFactory::instance()->destory_cab_ext_suber();
		MsgFactory::instance()->destory_cab_transmit_suber();
        MsgFactory::instance()->destory_cab_puber();
        MsgFactory::instance()->destory_cab_alarm_pusher();
    }
    catch (zmq::error_t &e)
    {
        LOGFMTE("ZMQ communication error. code: %d, description:%s", e.num(), e.what());
        string josn_str = FormatConvert::instance().alarm_to_string(-209);
        MsgFactory::instance()->cab_alarm_pusher().send(josn_str);
    }

	return;
}


CabinetHeartbeat::CabinetHeartbeat()
{	
    // Initialize the tcp client
    bzero(&mSocketAddr, sizeof(mSocketAddr));
    mSocketAddr.sin_family      = AF_INET; 
    mSocketAddr.sin_addr.s_addr = inet_addr(Configuration::instance()->ext_request_ip().c_str());
    mSocketAddr.sin_port        = htons(5010);
}

CabinetHeartbeat::~CabinetHeartbeat()
{
	closeConnection();
}

std::thread CabinetHeartbeat::run()
{
	return thread(&CabinetHeartbeat::work, this);
}

void CabinetHeartbeat::work()
{
    struct timeval timeout;
    fd_set         readfdSet;
    fd_set         exfdSet;
	time_t         lastTm = 0;
	char           buff[1024];	
	uint8_t        cnt = 0;
	bool enReset = true;

	sleep(2);				// 不延时，MsgFactory调用会导致程序崩溃。
	LOGFMTD("Master heartbeat starting...[%s:%d]", Configuration::instance()->ext_request_ip().c_str(), 5010);

	while (1)
    {
    	msleep(50);
		
    	uint8 timeoutCnt = 0;
		/* 1th: Create a socket */
		mSocketFd = socket(AF_INET, SOCK_STREAM, 0);
		mIsConnected = false;
		/* 2th: Socket configuration */
		/* 设置发送和接收的超时时间 */
		timeout.tv_sec	= 0;
		timeout.tv_usec = 500000;
		setsockopt(mSocketFd, SOL_SOCKET, SO_RCVTIMEO, (void *)&timeout, sizeof(struct timeval));
		setsockopt(mSocketFd, SOL_SOCKET, SO_SNDTIMEO, (void *)&timeout, sizeof(struct timeval));
		/* 设置延迟关闭 */
		struct linger so_linger = {
			.l_onoff  = 0,
			.l_linger = 0
		};
		setsockopt(mSocketFd, SOL_SOCKET, SO_LINGER, (void *)&so_linger, sizeof(so_linger));
		int optval = 1;
		setsockopt(mSocketFd, SOL_SOCKET, SO_REUSEADDR, (void *)&optval, sizeof(optval));
		/* 设置keepAlive */
		int keepAlive = 1;	  
		setsockopt(mSocketFd, SOL_SOCKET, SO_KEEPALIVE, (void *)&keepAlive, sizeof(keepAlive));

		int keepIdle	 = 5;		// 5 second
		int keepInterval = 2;		// 2 second
		int keepCount	 = 2;		// 2 times
		setsockopt(mSocketFd, IPPROTO_TCP, TCP_KEEPIDLE, (void *)&keepIdle, sizeof(keepIdle));
		setsockopt(mSocketFd, IPPROTO_TCP, TCP_KEEPINTVL, (void *)&keepInterval, sizeof(keepInterval));
		setsockopt(mSocketFd, IPPROTO_TCP, TCP_KEEPCNT, (void *)&keepCount, sizeof(keepCount));

        /* 2th: Connection server */        
        while (!mIsConnected)
        {
			LOGFMTD("Master heartbeat connecting...[%s:%d]", Configuration::instance()->ext_request_ip().c_str(), 5010);
            if (connect(mSocketFd, (struct sockaddr*)&mSocketAddr, sizeof(mSocketAddr)) < 0)
            {
				if(enReset)
            	{
					offlineHandle(false);
					enReset = false;
            	}
				sleep(3);
            }
            else
            {			
				recvHandler(EVENT_CONNECTED, NULL, 0);
				enReset = true;
				sleep(1);
            }
        }

        /* 3th: Send heartbeat data */        
        while (mIsConnected)
		{		
			// 3秒心跳间隔
			time_t nowTm = time(NULL);
			if (nowTm - lastTm < 3)
			{
				sleep(3 - (nowTm - lastTm));
				continue;
			}
			lastTm = nowTm;
			
			buff[0] = ++cnt;
			
			if (send(mSocketFd, buff, 1, 0) < 0)
			{
				closeConnection();
				continue;
			}
			
            /* 4th: Receiving data */           
            // 4.1th: Query the socket readability and exception
            FD_ZERO(&readfdSet );
            FD_SET(mSocketFd, &readfdSet);
            FD_ZERO(&exfdSet );
            FD_SET(mSocketFd, &exfdSet);
            
            timeout.tv_sec  = 2;
            timeout.tv_usec = 0;

            int ret = select(mSocketFd + 1, &readfdSet, NULL, &exfdSet, &timeout);
			
			// 4.2th: Read data
            if (ret > 0)
            {
                if (FD_ISSET(mSocketFd, &readfdSet))
                {
                    int recvLen = recv(mSocketFd, buff, sizeof(buff), 0);
                
                    // 4.3th: Excute Callback function
                    if (recvLen > 0)
                    {	
                    	timeoutCnt = 0;
						recvHandler(EVENT_RECEIVE, buff, recvLen);

						if(enReset)
						{
							sys_alarm_report(); 	//未清除告警信息上报
							enReset = false;
						}
                    }
                    else if (recvLen < 0)
                    {
                        if (!(errno == EINTR || errno == EWOULDBLOCK || errno == EAGAIN))
                        {
							closeConnection();
                        }
                    }
                    else
                    {
						closeConnection();
                    }
                }
                // 4.4th: Close socket then it has been exception
                else if (FD_ISSET(mSocketFd, &exfdSet))
                {
					closeConnection();
                }
            }
            else if (ret < 0)
            {
				closeConnection();
            }

			// 无心跳响应，判定为网络异常
			if (!mHeartbeatFlag)
			{
				LOGFMTD("Master heartbeat check fail.");
				timeoutCnt++;
				if(timeoutCnt > 3)
				{
					offlineFlag = true;
					closeConnection();
				}
			}
			// 心跳状态重置
			mHeartbeatFlag = false;
        }
    }
                
	// Thread exit
	closeConnection();
}

void CabinetHeartbeat::recvHandler(TcpEvent_e event, const char *data, int len)
{
	switch (event)
	{
		case EVENT_RECEIVE:
			mHeartbeatFlag = true;
			break;
			
		case EVENT_CONNECTED:
			mIsConnected = true;
			if(offlineFlag)
			{
				offlineFlag = false;
				cab_sub_zmq_reconnect();				
			}
			offlineHandle(mIsConnected);
			LOGFMTD("Master heartbeat connection is establish.");
			break;

		case EVENT_DISCONNECT:			
			mIsConnected = false;
			offlineHandle(mIsConnected);
			LOGFMTD("Master heartbeat connection was be disconnected.");
			break;

		default:
			break;
	}
}

void CabinetHeartbeat::offlineHandle(bool active)
{
	while (!g_is_zmq_created)
	{
		msleep(300);
	}
	
	MsgFactory::instance()->cab_ext_suber().connectionStatusSet(active);
	MsgFactory::instance()->cab_transmit_suber().connectionStatusSet(active);
	MsgFactory::instance()->cab_alarm_pusher().connectionStatusSet(active);
#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
	MsgFactory::instance()->cab_to_ext_reply_pusher().connectionStatusSet(active);
#endif
	for (int i=0; i<MAX_CELLS_NBR; ++i)
	{
		MsgFactory::instance()->cell_to_ext_reply_pusher(i + 1).connectionStatusSet(active);
		MsgFactory::instance()->cell_transmit_pusher(i + 1).connectionStatusSet(active);
		MsgFactory::instance()->cell_alarm_pusher(i + 1).connectionStatusSet(active);
	}
#if defined(ENABLE_PLC)
	MsgFactory::instance()->pins_to_ext_reply_pusher().connectionStatusSet(active);
	MsgFactory::instance()->pins_alarm_pusher().connectionStatusSet(active);
#endif
}

void CabinetHeartbeat::closeConnection()
{
    if (mSocketFd > 0)
    {
        close(mSocketFd);
		recvHandler(EVENT_DISCONNECT, NULL, 0);
    }
}

void CabinetHeartbeat::cab_sub_zmq_reconnect()
{
	int  cab_no = Configuration::instance()->cabinet_no();
	char filter[32] = {'\0'};
	sprintf(filter, TOPIC_CAB_FORMAT_STR, cab_no);

	MsgFactory::instance()->destory_cab_ext_suber();
	MsgFactory::instance()->destory_cab_transmit_suber();

	MsgFactory::instance()->create_cab_ext_suber(Configuration::instance()->ext_request_socket(), filter);
	MsgFactory::instance()->create_cab_transmit_suber(Configuration::instance()->ext_transmit_suber_socket(),filter);
}

void CabinetHeartbeat::sys_alarm_report()
{
	for(int no=1; no<=MAX_CELLS_NBR;no++)
	{
		Dispatcher::instance()->m_send_func_struct(no,SYS_ALARM_CODE_REPORT);
	}
}


