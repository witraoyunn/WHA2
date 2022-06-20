#include <string>
#include "Type.h"
#include "Configuration.h"
#include "FormatConvert.h"
#include "ChannelState.h"
#include "ReplyHandler.h"
#include "ChannelProto_JN9504.h"


using namespace std;

LoggerId data_id_process;
LoggerId data_id_stop;
LoggerId cell_id_temp;

ReplyHandler::ReplyHandler(int cell_no, int ch_no)
    : m_cell_no(cell_no)
    , m_ch_no(ch_no)
{
#ifndef PROTOCOL_5V160A
	m_vac_val = 0;
	for(int i = 0; i < 6; i++)
	{
		m_temper_val[i] = 0;
	}
	memset((void *)&m_last_stop_data, 0, sizeof(m_last_stop_data));
#endif	
}

ReplyHandler& ReplyHandler::operator= (const ReplyHandler& obj)
{
    if (this != &obj)
    {
        this->m_cell_no       = obj.m_cell_no;
        this->m_ch_no         = obj.m_ch_no;

#ifndef PROTOCOL_5V160A	
		this->m_vac_val       = obj.m_vac_val;
		for(int i = 0; i < 6; i++)
		{
			m_temper_val[i] = 0;
		}
		memset((void *)&m_last_stop_data, 0, sizeof(m_last_stop_data));
#endif		
    }

    return *this;
}

void ReplyHandler::append(uint8 *p_buf, int size)
{
    m_buffer.push(p_buf, size);

	int handle_size = 0;
	
#ifdef PROTOCOL_5V160A
    while (m_buffer.valid_size() > 9)	
#else
	while (m_buffer.valid_size() > 4)
#endif
	{
		if(m_handle_a_frame(&handle_size) > 0)
		{
			m_buffer.remove(handle_size);
		}
		else
		{
			m_buffer.remove(size);
		}
    }
}

void ReplyHandler::set_battery_type(const char *src)
{
	strcpy(m_state.m_battery_type, src); 
}

void ReplyHandler::set_stepConf_number(const char *src)
{
	strcpy(m_state.m_stepConf_number, src); 
}

void ReplyHandler::set_pallet_barcode(const char *src)
{
	strcpy(m_state.m_pallet_barcode, src); 
}

void ReplyHandler::set_batch_number(const char *src)
{
	strcpy(m_state.m_batch_number, src); 
}

void ReplyHandler::set_battery_barcode(const char *src)
{
	strcpy(m_state.m_battery_barcode, src);
//	LOGFMTD("%d-%d barcode: %s", m_cell_no, m_ch_no, m_battery_barcode.str);
}

void ReplyHandler::reset_all_info_code()
{
	m_state.reset_info_code();
}

#if 0
void ReplyHandler::set_vaccum_value(float val){
	m_vac_val = val;
}
void ReplyHandler::set_celltemp_value(float *val)
{
#ifdef PLC_JYT
		for(int i = 0; i < 6; i++)
#else
		for(int i = 0; i < 2; i++)
#endif
		{
			m_temper_val[i] = *(val+i);
		}
}
#endif

#ifdef PROTOCOL_5V160A
int ReplyHandler::m_handle_a_frame(int *ack_size)
{
	return recvReply.recvAckUnPack(m_buffer,m_cell_no,m_ch_no,ack_size,m_state);
}
#else
int ReplyHandler::m_handle_a_frame(int *ack_size)
{
    int func_code = -1;
    uint8 sub_buffer[MAX_ACK_SIZE];

	*ack_size = 0;

	// 包格式解析: 数据长度
	int dataLen = m_buffer[0];
	if (dataLen > MAX_ACK_SIZE)
	{
		*ack_size = 1;
		return func_code;
	}
	if (dataLen > m_buffer.valid_size())  return func_code;

	// 结束符
	if (m_buffer[dataLen - 1] != 0x0D)
	{
		*ack_size = 1;
		return func_code;
	}
	
	memset(sub_buffer, 0x0, MAX_ACK_SIZE);
    m_buffer.buf_copy(sub_buffer, 0, dataLen);
	
	// ModbusCRC16校验
	uint16 crc_1 = crc16(sub_buffer, dataLen-3);
	uint16 crc_2 = ((uint16)sub_buffer[dataLen-3] << 8) | sub_buffer[dataLen-2];
	if (crc_1 != crc_2)
	{
		*ack_size = 1;
		return func_code;
	}

	// 数据段解析
	*ack_size = dataLen;
	func_code = sub_buffer[1];
	
	if ((func_code != 0x27) && (func_code != 0x09))
	{
		LOGFMTD("%d-%2d recv: %s", m_cell_no, m_ch_no, hexstring(sub_buffer, dataLen).c_str());
	}
	
    switch (func_code)
    {
        case ACK_BEAT_TRIGGER:
        {
			m_state.reset_heartbeat_tm();
            break;
        }
		
		case ACK_QUERY_PROCESS_DATA:	  //数据返回
		{			
			Step_Process_Data_t data = Step_Process_Data_t();
			data.cell_no = m_cell_no;
			data.ch_no   = m_ch_no;
			strcpy(data.bat_type, m_state.m_battery_type);
			strcpy(data.stepConf_no, m_state.m_stepConf_number);
			strcpy(data.pallet_barcode, m_state.m_pallet_barcode);
			strcpy(data.bat_barcode, m_state.m_battery_barcode);
			data.run_state   = sub_buffer[3];
			data.run_time    = bytes_to_uint32(sub_buffer+5, 4);
			data.voltage     = (bytes_to_int(sub_buffer+10, 3))/10000.0f;
			data.current     = (bytes_to_uint32(sub_buffer+14, 3))/10000.0f;
			data.capacity    = bytes_to_uint32(sub_buffer+18, 4)/10000.0f;
			data.temperature = bytes_to_uint16(sub_buffer+23)/10.0f;
			data.step_no     = bytes_to_uint16(sub_buffer+26);
			data.step_type   = sub_buffer[29];

			data.loop_no     = bytes_to_uint16(sub_buffer+31);
			data.timestamp   = bytes_to_long(sub_buffer+35);
			data.sum_step    = bytes_to_uint16(sub_buffer+44);
			data.ratio       = bytes_to_uint32(sub_buffer+47, 3)/100.0f;
			data.energy      = bytes_to_uint32(sub_buffer+51, 3)/10000.0f;
			data.pvol        = bytes_to_int(sub_buffer+55, 3)/10000.0f;
			data.t_sink      = bytes_to_uint16(sub_buffer+58)/10.0f;
			data.v_cotact    = (bytes_to_int(sub_buffer+60, 4))/10000.0f;		// 单位 0.1mV/LSB
			data.r_cotact    = (bytes_to_uint16(sub_buffer+64))/1000000.0f;		// 单位 uR/LSB
			data.r_output    = (bytes_to_uint16(sub_buffer+66))/1000000.0f;		// 单位 uR/LSB
	#ifdef PROTOCOL_5V80A
			data.loop_times  = bytes_to_uint16(sub_buffer+68);
			data.error_code  = 0;
			if (sub_buffer[70] != 0)
			{
				if ((sub_buffer[70] & 0x80) != 0)
				{
					data.error_code = -2100 - (sub_buffer[70] & 0x7F);
				}
				else
				{
					data.error_code = -2000 - sub_buffer[70];
				}
			}
			
			int8_t vac_value = sub_buffer[71];
			if (vac_value > 0)
			{
				vac_value = (int8_t)0 - vac_value;
			}
	#endif
			// charge
			if (data.step_type == STEP_CHARGE_CC || 
				data.step_type == STEP_CHARGE_CV || 
				data.step_type == STEP_CHARGE_CC_CV)
			{
				data.energy *= 1.04;
			}
			// discharge
			if (data.step_type == STEP_DISCHARGE_CC || 
				data.step_type == STEP_DISCHARGE_CV || 
				data.step_type == STEP_DISCHARGE_CC_CV || 
				data.step_type == STEP_DISCHARGE_CW)
			{
				data.energy *= 0.94;
			}
			// 针床的负压值
			data.vac_val = m_vac_val;
#if defined(PLC_JYT)
			for(int i = 0; i < 6; i++)
			{
				data.cell_temper[i] = m_temper_val[i];
			}
#endif
			// 记录数据，发送给上位机
			if (Configuration::instance()->need_send_detail_data(m_cell_no))
			{
				string josn_str = FormatConvert::instance().record_to_string(data);
				MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
			}

			// 中航项目需求(接触测试、待机状态 数据不入库)
			if ((data.run_state != STEP_TEST && data.run_state != STEP_STANDBY) &&
				(data.step_no > 0))
			{
				data.is_stop = false;
				string store_josn_str = FormatConvert::instance().process_data_to_store_str(data);
				LOG_INFO(data_id_process, store_josn_str);
#ifdef STOREDATA
				// 工步运行数据入库	
				MsgFactory::instance()->data_to_ext_store_pusher(m_cell_no).send(data);
#endif

				// 工艺配方开始和结束工步入截止数据库
				if ((data.run_state == STEP_BEGIN) || (data.run_state == STEP_FINISH))
				{
					data.is_stop = true;
					store_josn_str = FormatConvert::instance().process_data_to_store_str(data);
					LOG_INFO(data_id_stop, store_josn_str);
#ifdef STOREDATA
					MsgFactory::instance()->data_to_ext_store_pusher(m_cell_no).send(data);
#endif
				}
				// 每个工步的截止数据入库
				if (data.step_no != m_last_stop_data.step_no)
				{
					m_last_stop_data.is_stop = true;
					store_josn_str = FormatConvert::instance().process_data_to_store_str(m_last_stop_data);
					LOG_INFO(data_id_stop, store_josn_str);
#ifdef STOREDATA
					MsgFactory::instance()->data_to_ext_store_pusher(m_cell_no).send(m_last_stop_data);
#endif
				}

				m_last_stop_data = data;
			}
			// 化成工艺状态判断
			if (data.run_state == STEP_BEGIN)
			{
				m_state.set_running(true);
				m_state.set_step_finish(false);
			}
			else if (data.run_state == STEP_PAUSE || data.run_state == STEP_TEST)
			{
				m_state.set_running(true);
			}
			else if (data.run_state == STEP_PROCESS)
			{
				m_state.set_running(true);
			}
			else if (data.run_state == STEP_FINISH)
			{
				m_state.set_running(false);
				m_state.set_step_finish(true);
			}
			else
			{
				m_state.set_running(false);
			}
			// 工步负压处理
			if (data.step_type == STEP_VAC)
			{
				m_state.set_step_vac(true);
				m_state.set_vac_value(vac_value);
			}
			else
			{
				m_state.set_step_vac(false);
			}
			break;
		}
		
		case ACK_SET_PROTECTION:    //保护设置回应
		case ACK_SET_PROTECTION_MERGE:
        case ACK_SET_SAMPLE:        //记录设置回应
        case ACK_SET_WAVE_SHARP:
        case ACK_SET_CALIBRATION:
        {
            int sub_code = sub_buffer[2];
	        int is_ok    = sub_buffer[3];

			// 下位机的全局保护合并指令的功能码已变更，上位机的功能码没有变更，需要做转换。
			if (func_code == ACK_SET_PROTECTION_MERGE)	
			{
				func_code = ACK_SET_PROTECTION;
			}
			string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, m_ch_no, func_code, sub_code, is_ok);
			MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
			break;
        }
		
        case ACK_SET_STEP:     		//工步设置回复
        {
            int sub_code = bytes_to_uint16(sub_buffer+2);
            int is_ok    = sub_buffer[4];

            string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, m_ch_no, func_code, sub_code, is_ok);
            MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
            break;
        }
		
        case ACK_TRIG_PROTECTION:   //保护触发
        {
            int sub_code   = sub_buffer[2];
            int error_code = -2000 - sub_code;
			m_state.push_error_code(sub_code);

            string josn_str = FormatConvert::instance().alarm_to_string(error_code, m_cell_no, m_ch_no, func_code, sub_code);
            MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
//            LOGFMTE("Cell:%d Channel:%d Globle Protection Fire! Function id:0x%x  Protection Code:0x%x", m_cell_no, m_ch_no, func_code, sub_code);
            break;
        }
		
        case ACK_STEP_TRIG_PROTECTION:   //单工步保护触发
        {
            int sub_code   = sub_buffer[2];
            int error_code = -2100 - sub_code;
			m_state.push_error_code(sub_code);

			string josn_str = FormatConvert::instance().alarm_to_string(error_code, m_cell_no, m_ch_no, func_code, sub_code);
            MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
//            LOGFMTE("Cell:%d Channel:%d Step Protection Fire! Function id:0x%x  Protection Code:0x%x", m_cell_no, m_ch_no, func_code, sub_code);
            break;
        }
		
        case ACK_RUN_CONTROL:       //通道控制回复
        {
            int sub_code = sub_buffer[2];
            int is_ok    = sub_buffer[3];

            string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, m_ch_no, func_code, sub_code, is_ok);
            MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
			break;
        }
	
        case ACK_QUERY_CALIBRATION:
		{
			int sub_code = sub_buffer[2];
			int value    = 0;
			if (dataLen == 8)
            {
				value = bytes2_to_int16(sub_buffer+3);
            }
            else if (dataLen == 10)
            {
                value = bytes4_to_int(sub_buffer+3);
            }
			
			string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, m_ch_no, func_code, sub_code, value);
			MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
			break;
		}
		
        case ACK_SET_AUTO_RUN:
        case ACK_SET_PARALLEL:
        case ACK_CLEAR_WARNING:    //清除警告回复
        {
            int sub_code = 0xFF;
            int is_ok    = sub_buffer[2];

            string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, m_ch_no, func_code, sub_code, is_ok);
            MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
            break;
        }

        case ACK_SET_SYS_TIME:  //系统时间设定回复
        {
            break;
        }

        case ACK_QUERY_AUTO_RUN:
		{
            int sub_code = sub_buffer[2];
			int value    = bytes_to_uint16(sub_buffer+3);

            string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, m_ch_no, func_code, sub_code, value);
            MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
			break;
		}
				
        case ACK_QUERY_PROTECTION:  //保护查询回应
        {
            int   sub_code = sub_buffer[2];
            float value    = 0;
			
            if (sub_code == 0x05 || sub_code == 0x08 || sub_code == 0x14)
            {
                value = bytes_to_uint32(sub_buffer+3, 3)/1000.0f;
            }
            else if (sub_code == 0x07 || sub_code == 0x13)
            {
                value = bytes_to_uint32(sub_buffer+3, 3)/10.0f;
            }
			else
            {
                value = bytes_to_uint32(sub_buffer+3, 3)/10000.0f;
            }

            string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, m_ch_no, func_code, sub_code, value);
            MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
            break;
        }
		
        case ACK_CHECK_PROTECTION:
        {
            int sub_code = sub_buffer[2];
            int is_ok    = (sub_buffer[3] == 0x02);

            string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, m_ch_no, func_code, sub_code, is_ok);
            MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
            break;
        }
		
        case ACK_QUERY_VERSION:
        case ACK_QUERY_ADDRESS:
            break;

        default:
            break;
    }

    return func_code;
}

#endif

