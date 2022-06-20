#include <deque>
#include "zmq.hpp"
#include "log4z.h"
#include "Channel.h"
#include "Type.h"
#include "FormatConvert.h"
#include "Serialize.h"
#include "Configuration.h"


using namespace std;

Channel::Channel(int cell_no, int ch_no)
    : m_cell_no(cell_no)
    , m_ch_no(ch_no)
    , m_replyer(cell_no, ch_no)
{
	m_is_connected = false;
	m_cmd_queue.clear();
#ifdef PROTOCOL_5V160A
	//memset(m_module_chEnList, false, sizeof(m_module_chEnList));
#endif
	//printf("Channel constructor cell_no=%d ch_no=%d\n",cell_no,ch_no);
}

Channel& Channel::operator= (const Channel& obj)
{
    if (this != &obj)
    {
        this->m_cell_no      = obj.m_cell_no;
        this->m_ch_no        = obj.m_ch_no;
		this->m_is_connected = obj.m_is_connected;
        this->m_replyer      = obj.m_replyer;
		//this->m_cmd_queue    = obj.m_cmd_queue;
    }
    m_cmd_queue.clear();
    return *this;
}

void Channel::push_command(Cmd_Channel_Msg_t ch_cmd)
{
    //if (m_is_connected)
    {
        m_cmd_queue.push_back(ch_cmd);
    }
}

#ifndef PROTOCOL_5V160A	
bool Channel::is_running()
{
	return m_replyer.m_state.is_running();
}
#endif

bool Channel::is_process_end()
{
	/*
	if (!m_is_connected)
	{		
		// 心跳丢失30S后，电源自动停止工步运行
		if (m_replyer.m_state.is_heartbeat_timeout())
		{
			if (m_replyer.m_state.get_heartbeat_lost_tm() > HEARTBEAT_LOST_ABORT)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		return true;
	}
	*/
	if (m_replyer.m_state.is_step_finish())
	{
		return true;
	}
	return false;
}

void Channel::reset_step_finish_status()
{
	m_replyer.m_state.set_step_finish(false);
}

#if (defined(AWH_GRADING_SYS) || defined(AWH_PRECHARGE_SYS))
bool Channel::is_stepcache_null()
{
	if(m_replyer.m_state.is_stepcache_empty())
	{
		return true;
	}
	
	return false;
}

void Channel::reset_stepcache_status()
{
	m_replyer.m_state.set_stepcache_empty(false);
}
#endif

#if 0
void Channel::set_vac_val(float val)
{
	m_replyer.set_vaccum_value(val);
}

void Channel::set_celltemp_value(float *temper)
{
	m_replyer.set_celltemp_value(temper);
}
#endif

#if 0
bool Channel::is_vac_step()
{
	return m_replyer.m_state.is_step_vac();
}

int8_t Channel::vac_Param()
{
	return m_replyer.m_state.get_vac_value();
}
#endif

void Channel::set_tech_info(int type, const char *src)
{
	if((string(src) != "") && (string(src) != "0"))
	{
		if (type == 0)
		{
			m_replyer.set_battery_type(src);
		}
		else if (type == 1)
		{
			m_replyer.set_stepConf_number(src);
		}
		else if (type == 2)
		{
			m_replyer.set_pallet_barcode(src);
		}
		else if (type == 3)
		{
			m_replyer.set_batch_number(src);
		}
	}
}

void Channel::set_barcode(const char *src)
{
	if((string(src) != "") && (string(src) != "0"))
	{
		m_replyer.set_battery_barcode(src);
	}
}

void Channel::clear_channel_info()
{
	m_replyer.reset_all_info_code();
}


#ifndef PROTOCOL_5V160A	
void ChannelClient::connect()
{
    if (!m_is_connected)
    {
        string ip_addr = "";
        int port = Configuration::instance()->channel_socket(m_cell_no, m_ch_no, ip_addr);
        if (port > 0)
        {
			try
			{	
	            m_device_socket.connect_server(ip_addr.c_str(), port);
	            m_device_socket.set_async_mode();
				m_is_connected = true;
				m_replyer.m_state.reset_heartbeat_tm(); 				// 心跳重置
				// 应答
				string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, m_ch_no, ACK_CONNECTED, -1, -1);
				MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
			}
			catch (int &e)
			{
				string josn_str = FormatConvert::instance().alarm_to_string(-607);
				MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
				LOGFMTW("Channel connection fail, code: %d.", e);
			}	
        }
        else
        {
            LOGFMTE("Cell:%d Channel:%d not config can not connect.", m_cell_no, m_ch_no);
            throw -602;
        }
    }
    else
    {
        string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, m_ch_no, ACK_CONNECTED, -1, -1);
        MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
    }
}

void ChannelClient::disconnect()
{
	if (m_is_connected)
	{
    	m_device_socket.disconnect();
    	m_is_connected = false;
	}
	// 应答
    string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, m_ch_no, ACK_DIS_CONNECTED, -1, -1);
    MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
}

bool ChannelClient::isConnected()
{
	return m_is_connected;
}

void ChannelClient::sync_system_time()
{
    if (m_is_connected)
    {
        Channel_Data_t data = Channel_Data_t();
		
        Serialize data_stream(CMD_SET_SYS_TIME, data);
        send_data(data_stream.bytes_buffer(), data_stream.bytes_size());
    }
}

bool ChannelClient::check_beating()
{
    if (m_is_connected)
    {
        if (m_replyer.m_state.is_heartbeat_timeout())
        {
			m_is_connected = false;
			return false;
        }
		// 下位机没收到心跳指令就会断开连接（对下位机而言，工步记录数据的上传，不能做为网络通信正常的依据）
		Channel_Data_t data = Channel_Data_t();
		
		Serialize data_stream(CMD_BEAT_TRIGGER, data);

		send_data(data_stream.bytes_buffer(), data_stream.bytes_size());
		m_replyer.m_state.set_heartbeat_tm();		
    }

    return true;
}

int ChannelClient::execute()
{
	int ret_cmd_code = -1;
    if (m_cmd_queue.empty())  return ret_cmd_code;

    Cmd_Channel_Msg_t command = m_cmd_queue.front();	
    m_cmd_queue.pop_front();
	ret_cmd_code = command.func_code;

	// 发送命令给下位机
	if (!m_is_connected)  return ret_cmd_code;

    Serialize data_stream(command.func_code, command.data);
    send_data(data_stream.bytes_buffer(), data_stream.bytes_size());

#ifdef CHANNEL_START_DELAY
	// 为抑制启动时BUS电压的变化，每个通道间延时100mS启动。
	if ((command.func_code == CMD_RUN_CONTROL) && (command.data.func_data.func_code == STEP_PROCESS))
	{
		msleep(150);
	}
#endif
	// 清除保存的故障码
#ifndef PROTOCOL_5V160A
	if (command.func_code == CMD_CLEAR_WARNING)
	{
		m_replyer.m_state.clear_error_code();
	}
#endif
	return ret_cmd_code;
}

void ChannelClient::start_process(int step_no)
{
    if (!m_is_connected)  return; 
		
    Channel_Data_t data = Channel_Data_t();
    data.func_data.func_code = STEP_PROCESS;
    data.func_data.value     = step_no;
	
    Serialize data_stream(CMD_RUN_CONTROL, data);
    send_data(data_stream.bytes_buffer(), data_stream.bytes_size());
}

void ChannelClient::stop_process()
{
    if (!m_is_connected)  return;
		
    Channel_Data_t data = Channel_Data_t();
    data.func_data.func_code = STEP_STOP;
    data.func_data.value     = 1;
	
    Serialize data_stream(CMD_RUN_CONTROL, data);
    send_data(data_stream.bytes_buffer(), data_stream.bytes_size());
}

void ChannelClient::pause_process()
{
    if (!m_is_connected)  return;
		
    Channel_Data_t data = Channel_Data_t();
    data.func_data.func_code = STEP_PAUSE;
    data.func_data.value     = 1;
	
    Serialize data_stream(CMD_RUN_CONTROL, data);
    send_data(data_stream.bytes_buffer(), data_stream.bytes_size());
}

void ChannelClient::vac_step_process()
{
    if (!m_is_connected)  return;
		
    Channel_Data_t data = Channel_Data_t();
    data.func_data.func_code = STEP_VACNEXT;
    data.func_data.value     = 1;
	
    Serialize data_stream(CMD_RUN_CONTROL, data);
    send_data(data_stream.bytes_buffer(), data_stream.bytes_size());
}

void ChannelClient::send_data(uint8 *src, int size)
{
	try
	{					
    	m_device_socket.send_msg((char *)src, size);
#ifndef PROTOCOL_5V160A
		if (src[1] != CMD_BEAT_TRIGGER)
#endif
		{
        	LOGFMTD("send-->%d-%2d: %s", m_cell_no, m_ch_no, hexstring(src, size).c_str());
		}
	}
	catch (int &e)
	{
		m_device_socket.disconnect();
		m_is_connected = false;

		e = -603;
		string josn_str = FormatConvert::instance().alarm_to_string(e, m_cell_no, m_ch_no);
		MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
	}
}

std::vector<int> ChannelClient::handle_reply()
{
	std::vector<int> ret;
	
	if (!m_is_connected)  return ret;

	// 接收数据
    char device_reply[MAX_READ_BYTE_NBR] = { 0 };
    int size = 0;
    try
    {
        size = m_device_socket.async_recv_msg(device_reply, MAX_READ_BYTE_NBR);
	}
    catch (int &e)
    {
		m_device_socket.disconnect();
		m_is_connected = false;

		e = -603;
		string josn_str = FormatConvert::instance().alarm_to_string(e, m_cell_no, m_ch_no);
		MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
    }
	// 处理接收的数据
    if (size > 0)
    {
		m_replyer.append((uint8 *)device_reply, size);
		ret = m_replyer.m_state.get_error_code();
		//m_is_en = true;
    }
	
	return ret;
}
#endif


