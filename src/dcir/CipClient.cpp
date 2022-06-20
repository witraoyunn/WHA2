/**************************************************************
* Copyright (C) 2019, 上海鼎格信息科技有限公司
* All rights reserved.
* 产品号   :
* 所属组件 :
* 模块名称 :
* 文件名称 : CipClient.cpp
* 概要描述 :
* 历史记录 :
* 版本      日期          作者           内容
* 1.0       20190712     zhangw
***************************************************************/
#include "CipClient.h"
#include "log4z.h"

void CipClient::initialize(std::string ip, int port)
{
    LOGFMTT("CipClient::%s IP:[%s], port:[%d] --->>>", __FUNCTION__, ip.c_str(), port);
	try
	{
		m_omron_fin_client.connect_server(ip, port);

		m_omron_fin_client.initialize();
	}
	catch (int ec)
	{
		LOGFMTE("Initialize cip client failed. error code: [%d]", ec);
	}

    LOGFMTT("CipClient::%s ---<<<", __FUNCTION__);
}


CipClient::CipClient()
{
	m_omron_fin_client =  OmronFinsNet();
}

CipClient::~CipClient()
{
	m_omron_fin_client.~OmronFinsNet();
}

bool CipClient::check_battery_enter()
{
    LOGFMTT("CipClient::%s --->>>", __FUNCTION__);

	std::string address = "W400.00";
	bool result = m_omron_fin_client.read_bool(address);

    LOGFMTT("CipClient::%s result:[%s]---<<<", __FUNCTION__, result? "True": "False");

	return result;
}
bool CipClient::check_move_battery_finish(int* station_id)
{
    LOGFMTT("CipClient::%s --->>>", __FUNCTION__);

	std::string inspect_addr = "W400.01";
	bool result = m_omron_fin_client.read_bool(inspect_addr);

	if (result)
	{
		std::string channel_no_addr = "D300";
		*station_id = m_omron_fin_client.read_int(channel_no_addr);

		LOGFMTD("pins channal no :[%d]", *station_id);
	}

    LOGFMTT("CipClient::%s result:[%s]---<<<", __FUNCTION__, result? "True": "False");

	return result;
}
bool CipClient::check_request_exit()
{
    LOGFMTT("CipClient::%s --->>>", __FUNCTION__);

	std::string address = "W400.03";

	bool result =m_omron_fin_client.read_bool(address);

    LOGFMTT("CipClient::%s result:[%s]---<<<", __FUNCTION__, result? "True": "False");

	return result;
}

void CipClient::request_move_battery(bool first)
{
    LOGFMTT("CipClient::%s first:[%d]--->>>", __FUNCTION__, first);

	std::string address = "W450.00";
	if (!first) 
	{
		address = "W450.01";
	}
	m_omron_fin_client.write_bool(address, true);

    LOGFMTT("CipClient::%s---<<<", __FUNCTION__);
}

void CipClient::permit_exit()
{
    LOGFMTT("CipClient::%s --->>>", __FUNCTION__);

	std::string address = "W450.03";
	m_omron_fin_client.write_bool(address, true);

    LOGFMTT("CipClient::%s---<<<", __FUNCTION__);
}