/**************************************************************
* Copyright (C) 2019, 上海鼎格信息科技有限公司
* All rights reserved.
* 产品号   :
* 所属组件 :
* 模块名称 :
* 文件名称 : Cabinet.h
* 概要描述 :
* 历史记录 :
* 版本      日期          作者           内容
* 1.0       20190416     zhangw
***************************************************************/
#ifndef _CABINET_H_
#define _CABINET_H_

#include <string>
#include <thread>
#include <arpa/inet.h>

class Cabinet
{
public:
    void run();
};

class CabinetHeartbeat
{
public:
	typedef enum {
	    EVENT_NONE       = 0x00,
	    EVENT_CONNECTED  = 0x01,
	    EVENT_DISCONNECT = 0x02,
	    EVENT_RECEIVE    = 0x03,
	} TcpEvent_e;

public:
	CabinetHeartbeat();
	~CabinetHeartbeat();
	
    std::thread run();

private:
	void work();
	void recvHandler(TcpEvent_e event, const char *data, int len);
	void closeConnection();
	void offlineHandle(bool active);
	void cab_sub_zmq_reconnect();
	void sys_alarm_report();

private:
	sockaddr_in  mSocketAddr;
	int 		 mSocketFd = 0;
	bool         mIsConnected   = false;
	bool         mHeartbeatFlag = false;
	bool 		 offlineFlag = false;
};

#endif

