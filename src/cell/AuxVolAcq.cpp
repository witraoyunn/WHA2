#include "AuxVolAcq.h"
#include "utility.h"
#include "log4z.h"
#include "FormatConvert.h"
#include "MsgFactory.h"
#include "utility.h"

#include <string.h>

#ifdef AUX_VOLTAGE_ACQ

#define RECONNECT_INTERVAL_TIME				5
#define DATA_QUERY_INTERVAL_TIME			1
#define QUERY_LOST_TIMES_MAX				6

using namespace std;

AuxVolAcq::AuxVolAcq(int cell_no)
{
	m_cell_no = cell_no;
	memset(m_moduleEn_flag, false, sizeof(m_moduleEn_flag));
	memset(m_isConnected, false, sizeof(m_isConnected));
	memset(m_recv_buff, 0, sizeof(m_recv_buff));
	memset(m_recv_len, 0, sizeof(m_recv_len));
	memset(m_query_lost_tm, 0, sizeof(m_query_lost_tm));
}

AuxVolAcq& AuxVolAcq::operator= (const AuxVolAcq& obj)
{
    if (this != &obj)
    {
        this->m_cell_no = obj.m_cell_no;
    }

    return *this;
}

AuxVolAcq::~AuxVolAcq()
{
	disconnect();
}

thread AuxVolAcq::run()
{
	return thread(&AuxVolAcq::work, this);
}

void AuxVolAcq::work()
{	
//	// debug
//	if (m_cell_no == 1)
//	{
//		m_moduleEn_flag[0] = true;
//	}
	
	// 创建ZMQ通讯
	try
	{
		MsgFactory::instance()->create_aux_param_pusher(m_cell_no);
		MsgFactory::instance()->create_aux_param_puller(m_cell_no);
	}
	catch (zmq::error_t &e)
	{
		LOGFMTE("Create ZMQ communication error. code: %d, description:%s", e.num(), e.what());
		exit(-209);
	}
	
	while (1)
    {
		msleep(100);
		
		// 连接电压采集盒 (心跳超时后，需要重连)
		connect();

		// 接收并处理上位机指令
		request_msg_handle();
		
		// 接收并处理电压采集盒数据
		response_msg_handle();	
		
		// 业务逻辑  
		workLogic();
	}
                
	// Thread exit
	MsgFactory::instance()->destory_aux_param_pusher(m_cell_no);
	MsgFactory::instance()->destory_aux_param_puller(m_cell_no);
}

void AuxVolAcq::workLogic()
{
	time_t nowTm = time(NULL);
	if ((nowTm - m_last_query_tm) < DATA_QUERY_INTERVAL_TIME)  return;
	m_last_query_tm = nowTm;
	
	for (int i=0; i<AUX_VOL_ACQ_CELL_NUM; i++)
	{
		if (!m_isConnected[i])  continue;

		if (m_query_lost_tm[i] < QUERY_LOST_TIMES_MAX)
		{
			// 查询从机数据
			package(i, 0x0F, 0x03);
			m_query_lost_tm[i]++;
		}
		else
		{
			disconnect(i);
			string josn_str = FormatConvert::instance().alarm_to_string(-605);
			MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
		}
	}
}

void AuxVolAcq::connect()
{
	time_t nowTm = time(NULL);

	if ((nowTm - m_reconnect_tm) > RECONNECT_INTERVAL_TIME)
	{
		m_reconnect_tm = nowTm;

		for (int i=0; i<AUX_VOL_ACQ_CELL_NUM; i++)
		{
			connect(i);
		}
	}
}

void AuxVolAcq::connect(int module_no)
{
	if (module_no < AUX_VOL_ACQ_CELL_NUM)
	{
		if (m_isConnected[module_no] || !m_moduleEn_flag[module_no])  return;
		
		char ip[32];
		sprintf(ip, AUX_VOL_ACQ_IP, 200 + (m_cell_no - 1) * AUX_VOL_ACQ_CELL_NUM + module_no);
		
		try
		{	
			m_socket[module_no].connect_server(ip, AUX_VOL_ACQ_PORT);
			m_socket[module_no].set_async_mode();
			m_isConnected[module_no] = true;
			m_query_lost_tm[module_no] = 0;
			m_last_query_tm = time(NULL);
		}
		catch (int &e)
		{
//			LOGFMTD("AuxVolAcq connection fail, code: %d.", e);
		}
	}
}

void AuxVolAcq::disconnect()
{
	for (int i=0; i<AUX_VOL_ACQ_CELL_NUM; i++)
	{
		disconnect(i);
	}
}

void AuxVolAcq::disconnect(int module_no)
{
	if (module_no < AUX_VOL_ACQ_CELL_NUM)
	{
		if (!m_isConnected[module_no])  return;
		
		m_socket[module_no].disconnect();
		m_isConnected[module_no] = false;
		m_query_lost_tm[module_no] = 0;
	}
}

void AuxVolAcq::request_msg_handle()
{
	Cmd_Aux_Param_t recv = Cmd_Aux_Param_t();
	try
	{
		if (MsgFactory::instance()->aux_param_puller(m_cell_no).recevie(recv, ZMQ_DONTWAIT) != true)
		{
			return;
		}
	}
	catch (zmq::error_t &e)
	{
		LOGFMTE("Create ZMQ communication error. code: %d, description:%s", e.num(), e.what());
        string josn_str = FormatConvert::instance().alarm_to_string(-209);
        MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
		exit(-209);
	}

	if (recv.cell_no != m_cell_no)  return;
	if (recv.fuc_code == 0x0F)
	{
		for (int i=0; i<recv.size; ++i)
		{
			switch (recv.data[i].sub_code)
			{
				// 电池电压上限报警值设置
				case 0x42:
				{
					// 一个库有3个电压采集盒
					for (int module_no=0; module_no<AUX_VOL_ACQ_CELL_NUM; ++module_no)
					{
						package(module_no, recv.fuc_code, recv.data[i].sub_code, (uint8_t *)&recv.data[i].value, 4);
					}
					break;
				}
				// 清除告警
				case 0x62:
				{
					for (int module_no=0; module_no<AUX_VOL_ACQ_CELL_NUM; ++module_no)
					{
						package(module_no, recv.fuc_code, recv.data[i].sub_code);
					}
					break;
				}
			}
		}
	}
	else if (recv.fuc_code == SYS_COMMON_CMD)
	{						
		if (recv.status.sub_code == SYS_PROBE_STATUS_SYNC)
		{
			// 刚压合时，延时清除告警。
			if (recv.status.value != 0)
			{
				m_isProbeExtend = true;
				sleep(2);
				for (int module_no=0; module_no<AUX_VOL_ACQ_CELL_NUM; ++module_no)
				{
					package(module_no, 0x0F, 0x62);
				}
			}
		}
		else if (recv.status.sub_code == SYS_CELL_CH_EN_SYNC)
		{
			// 根据上位机连接的通道，使能相应的采集盒
			for (int i=0; i<AUX_VOL_ACQ_CELL_NUM; i++)
			{
				if (((recv.status.value >> (i*AUX_VOL_ACQ_CH_NUM)) & 0xFFFF)  != 0)
				{
					m_moduleEn_flag[i] = true;
				}
				else
				{
					m_moduleEn_flag[i] = false;
					disconnect(i);
				}
			}
		}
	}
}

void AuxVolAcq::response_msg_handle()
{
	int module_no;
	
	try
	{	
		for (int i=0; i<AUX_VOL_ACQ_CELL_NUM; i++)
		{
			if (!m_isConnected[i])  continue;

			module_no = i;
			int size = m_socket[i].async_recv_msg(&m_recv_buff[i][m_recv_len[i]], RECV_BUFFER_SIZE - m_recv_len[i]);
			if (size > 0)
			{
				LOGFMTD("AuxVOlAcq << %s", hexstring((uint8_t *)&m_recv_buff[i][m_recv_len[i]], size).c_str());
				m_recv_len[i] += size;
			}
		}
	}
	catch (int &e)
	{
		disconnect(module_no);
        string josn_str = FormatConvert::instance().alarm_to_string(e);
        MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
	}
	
	for (int i=0; i<AUX_VOL_ACQ_CELL_NUM; i++)
	{
		if (m_recv_len[i] > 0)
		{
			int ret = unpackage(i, (const uint8_t *)&m_recv_buff[i][0], m_recv_len[i]);
			memcpy(&m_recv_buff[i][0], &m_recv_buff[i][ret], m_recv_len[i] - ret);
			m_recv_len[i] -= ret;
		}
	}
}

void AuxVolAcq::package(int module_no, uint8_t funcode, uint8_t subcode, const uint8_t *data, int len)
{
	uint8_t buf[64];
	int     index = 0;
	
	// 帧头
    buf[index++] = 0x5A;
    buf[index++] = 0x5A;
	// 数据长度
	index += 2;
	// 通道标志
    buf[index++] = 0;
	// 功能数据段：命令码
    buf[index++] = funcode;
	// 子命令码和数据
    buf[index++] = subcode;

	if (len > 0)
	{
		memcpy(buf + index, data, len);
		index += len;
	}

	// 长度填充
    buf[2] = (uint8_t) (index & 0xFF);
    buf[3] = (uint8_t) (index >> 8);
	// 校验码（ModbusCRC16）
    uint16_t crcCode = crc16(&buf[2], index-2);
    buf[index++] = (uint8_t) (crcCode & 0xFF);
    buf[index++] = (uint8_t) (crcCode >> 8);
	// 帧尾
    buf[index++] = 0xA5;
    buf[index++] = 0xA5;

	// 发送数据包
	if (m_isConnected[module_no])
	{
		try
		{	
			m_socket[module_no].send_msg((char *)buf, index);
			LOGFMTD("AuxVOlAcq >> %s", hexstring((uint8 *)buf, index).c_str());
		}
		catch (int &e)
		{
			disconnect(module_no);
			string josn_str = FormatConvert::instance().alarm_to_string(e);
			MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
		}
	}
}

int AuxVolAcq::unpackage(int module_no, const uint8_t *data, int len)
{
	int index = 0;
	// 帧头
	if (data[index++] != 0x5A)  return index;
	if (data[index++] != 0x5A)	return index;
	// 数据长度
	int dataLen = data[index++];
	dataLen |= data[index++] << 8;
	if ((dataLen + index) > len)  return 0;
	if ((dataLen + index) > RECV_BUFFER_SIZE)  return index;
	index += dataLen - 2;
	// 帧尾
	if (data[index++] != 0xA5)  return index;
	if (data[index++] != 0xA5)  return index;
	// 校验码 ModbusCRC16
    uint16_t crcCode = crc16((uint8_t *)data + 2, dataLen - 2);
	if ((data[index - 4] != (uint8_t)(crcCode & 0xFF)) || (data[index - 3] != (uint8_t)(crcCode >> 8)))
	{
		return index;
	}

	// 通道标志
	index = 4;
	index++;
	// 功能数据段
    int fun_code = data[index++];				// 功能码
	int sub_code = data[index++];				// 子功能码

	if (fun_code == 0x8F)
	{
		switch (sub_code)
		{
			// 查询回复
			case 0x03:
			{
				m_query_lost_tm[module_no] = 0;
				
				uint16_t ch_status = data[index++];
				ch_status |= data[index++] << 8;
				alarmHandle(module_no, ch_status);
				break;
			}
			// 设置回复
			case 0x42:
			case 0x62:
			{
				string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, -1, ACK_SET_AUX_PARAM, sub_code, true);
				MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
				break;
			}
		}
	}

	return dataLen + 4;
}

void AuxVolAcq::alarmHandle(int module_no, uint16_t devStatus)
{
	if (devStatus == 0)  return;
	if (!m_isProbeExtend)  return;			// 针床未压合

	bool needProtection = false;
	
	for (int i=0; i<AUX_VOL_ACQ_CH_NUM; ++i)
	{
		if ((devStatus & (1 << i)) != 0)
		{
			int ch_no = (module_no * AUX_VOL_ACQ_CH_NUM) + (i + 1);
			string josn_str = FormatConvert::instance().alarm_to_string(-2201, m_cell_no, ch_no);
			MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
			needProtection = true;
		}
	}

	if (needProtection)
	{
		char filter[32] = {'\0'};
		
		Cmd_Cell_Msg_t msg_struct = Cmd_Cell_Msg_t();
		msg_struct.func_code	       = SYS_COMMON_CMD;
		msg_struct.inside_msg.sub_code = SYS_CELL_ALARM_HANDLE;
		msg_struct.inside_msg.value	   = -2201;
		
		sprintf(filter, TOPIC_CELL_FORMAT_STR, m_cell_no);		
		MsgFactory::instance()->cab_puber().send(filter, msg_struct, ZMQ_DONTWAIT);
	}
}
#endif


