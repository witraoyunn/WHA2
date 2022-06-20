#include <string.h>
#include <time.h>
#include "log4z.h"
#include "Type.h"
#include "Serialize.h"
#include "ChannelProto_JN9504.h"


#ifdef PROTOCOL_5V160A				// 5V160A 模拟方案的协议
Serialize::Serialize(int ch_no, int func_code, Channel_Data_t &data,uint8 feat):sendSerialize(feat)
{
	m_size = sendSerialize.sendCmdPack(m_cmd_buffer, ch_no, func_code, data);
}
#else								// 数字方案的协议
Serialize::Serialize(int func_code, Channel_Data_t &data)
{
    memset(m_cmd_buffer, 0x0, MAX_CMD_SIZE);

    uint8 size = 1;
    m_cmd_buffer[size++] = func_code;

    switch (func_code)
    {
        case CMD_SET_PROTECTION:
        {
	#ifdef ZHONGHANG_PRJ_NEW
			m_cmd_buffer[1] = CMD_SET_PROTECTION_MERGE;
	
			uint8 byte[4] = {0};
			// 下位机全局合并指令的参数要按子功能码顺序排列
			for (int i=0; i<MAX_FUCN_LIST_SIZE; i++)
			{
				if (data.subList[i].func_code == 0)  continue;
				
				m_cmd_buffer[size++] = data.subList[i].func_code;
				if (data.subList[i].value < 0)
				{
					int_to_bytes(data.subList[i].value, byte);
					m_cmd_buffer[size++] = byte[0];
					m_cmd_buffer[size++] = byte[2];
					m_cmd_buffer[size++] = byte[3];
				}
				else
				{
					uint32_to_bytes(data.subList[i].value, byte);
					m_cmd_buffer[size++] = byte[1];
					m_cmd_buffer[size++] = byte[2];
					m_cmd_buffer[size++] = byte[3];
				}
			}
	#else
            uint8 sub_func_id = data.func_data.func_code;
            m_cmd_buffer[size++] = sub_func_id;
            uint8 byte[4] = {0};
			if (data.func_data.value < 0)
			{
				int_to_bytes(data.func_data.value, byte);
				m_cmd_buffer[size++] = byte[0];
				m_cmd_buffer[size++] = byte[2];
				m_cmd_buffer[size++] = byte[3];
			}
			else
			{
				uint32_to_bytes(data.func_data.value, byte);
				m_cmd_buffer[size++] = byte[1];
				m_cmd_buffer[size++] = byte[2];
				m_cmd_buffer[size++] = byte[3];
			}
	#endif
            break;
        }

        case CMD_SET_SAMPLE:
        {
            for(int i = 0; i < 3; i++)
            {
                m_cmd_buffer[size++] = data.subList[i].func_code;
                uint8 bytes[4] = {0};
                uint32_to_bytes(data.subList[i].value, bytes);
                m_cmd_buffer[size++] = bytes[0];
                m_cmd_buffer[size++] = bytes[1];
                m_cmd_buffer[size++] = bytes[2];
                m_cmd_buffer[size++] = bytes[3];
            }
            break;
        }
        case CMD_RUN_CONTROL:       //通道控制
        {
            uint8 sub_func_id = data.func_data.func_code;
            m_cmd_buffer[size++] = sub_func_id;
            uint8 byte[4] = {0};
            uint32_to_bytes(data.func_data.value, byte);
            m_cmd_buffer[size++] = byte[2];
            m_cmd_buffer[size++] = byte[3];
            break;
        }
        case CMD_SET_CALIBRATION:
        {
            uint8 sub_func_id = data.func_data.func_code;
            m_cmd_buffer[size++] = sub_func_id;
            uint8 byte[4] = {0};
            int_to_bytes(data.func_data.value, byte);
            m_cmd_buffer[size++] = byte[2];
            m_cmd_buffer[size++] = byte[3];
            break;
        }
        case CMD_SET_WAVE_SHARP:
        {
            uint8 sub_func_id = data.subList[0].func_code;
            m_cmd_buffer[size++] = sub_func_id;
            uint32 value = data.subList[0].value;
            uint8 byte[4] = {0};
            uint32_to_bytes(value, byte);
            m_cmd_buffer[size++] = byte[1];
            m_cmd_buffer[size++] = byte[2];
            m_cmd_buffer[size++] = byte[3];

            m_cmd_buffer[size++] = data.subList[1].func_code;
            value = data.subList[1].value;
            memset(byte, 0x0, 4);
            uint32_to_bytes(value, byte);
            m_cmd_buffer[size++] = byte[1];
            m_cmd_buffer[size++] = byte[2];
            m_cmd_buffer[size++] = byte[3];
            break;
        }
        case CMD_SET_AUTO_RUN:
        {
            uint32 value = data.subList[0].value;
            uint8 byte[4] = {0};
            uint32_to_bytes(value, byte);
            m_cmd_buffer[size++] = byte[3];
            value = data.subList[1].value;
            memset(byte, 0x0, 4);
            uint32_to_bytes(value, byte);
            m_cmd_buffer[size++] = byte[3];
            break;
        }
        case CMD_SET_PARALLEL:
        {
            m_cmd_buffer[size++] = data.func_data.func_code;
            m_cmd_buffer[size++] = data.func_data.value;
            break;
        }
        case CMD_SET_SYS_TIME:      //系统时间设定回复
        {
            uint8 bytes[8] = {0};
            time_t tt = time(NULL);
            long_to_bytes(tt*1000, bytes);

            m_cmd_buffer[size++] = 0;
            m_cmd_buffer[size++] = 0;
            m_cmd_buffer[size++] = 0;
            m_cmd_buffer[size++] = bytes[2];
            m_cmd_buffer[size++] = bytes[3];

            m_cmd_buffer[size++] = bytes[4];
            m_cmd_buffer[size++] = bytes[5];
            m_cmd_buffer[size++] = bytes[6];
            m_cmd_buffer[size++] = bytes[7];
            break;
        }
        case CMD_QUERY_PROTECTION:  //保护查询回应
        case CMD_QUERY_WAVE_SHARP:
        case CMD_QUERY_CALIBRATION:
        {
            m_cmd_buffer[size++] = data.func_data.func_code;
            break;
        }
#ifndef PROTOCOL_5V160A
        case CMD_CLEAR_WARNING:     //清除警告回复
        {
            m_cmd_buffer[size++] = 0x01;
            break;
        }
#endif
        case CMD_QUERY_PROCESS_DATA: //数据返回
        case CMD_QUERY_AUTO_RUN:
        case CMD_BEAT_TRIGGER:
            break;
		
        case CMD_SET_STEP:           //工步设置
        {		
            uint8 bytes[4] = {0};
			memset(bytes, 0x0, 4);
			uint16_to_bytes(data.step_data.step_nbr, bytes);
			m_cmd_buffer[size++] = bytes[0];
			m_cmd_buffer[size++] = bytes[1];
			
            Step_Type_e step_type = data.step_data.step_type;
			m_cmd_buffer[size++] = (uint8)step_type;
			
			if (step_type == STEP_LOOP_START || step_type == STEP_LOOP_STOP)
			{
				// 预留
				m_cmd_buffer[size++] = 0x01;
				// 循环序号
				memset(bytes, 0x0, 4);
				uint32_to_bytes(data.step_data.step_data.loop_data[3].value, bytes);
				m_cmd_buffer[size++] = bytes[2];
				m_cmd_buffer[size++] = bytes[3];
				// 预留
				m_cmd_buffer[size++] = 0xFF;
				m_cmd_buffer[size++] = 0x00;
				// 此循环结束后逻辑
				memset(bytes, 0x0, 4);
				uint32_to_bytes(data.step_data.step_data.loop_data[2].value, bytes);
				m_cmd_buffer[size++] = bytes[3];
				// 循环次数				
				memset(bytes, 0x0, 4);
				uint32_to_bytes(data.step_data.step_data.loop_data[4].value, bytes);
				m_cmd_buffer[size++] = bytes[2];
				m_cmd_buffer[size++] = bytes[3];
				// 此循环起始工步号
				memset(bytes, 0x0, 4);
				uint32_to_bytes(data.step_data.step_data.loop_data[0].value, bytes);
				m_cmd_buffer[size++] = bytes[2];
				m_cmd_buffer[size++] = bytes[3];	
			}
			else
			{				
				//工步额定值设定
				for (int i = 0; i < 3; i++)
				{
					uint8 sub_func_id = data.step_data.step_data.general_data.spec_data[i].func_code;
					m_cmd_buffer[size++] = sub_func_id;
					memset(bytes, 0x0, 4);
					uint32_to_bytes(data.step_data.step_data.general_data.spec_data[i].value, bytes);
					m_cmd_buffer[size++] = bytes[0];
					m_cmd_buffer[size++] = bytes[1];
					m_cmd_buffer[size++] = bytes[2];
					m_cmd_buffer[size++] = bytes[3];
				}

				//记录条件设置
				for (int i = 0; i < 3; i++)
				{
					uint8 sub_func_id = data.step_data.step_data.general_data.record_cond[i].func_code;
					m_cmd_buffer[size++] = sub_func_id;
					memset(bytes, 0x0, 4);
					uint32_to_bytes(data.step_data.step_data.general_data.record_cond[i].value, bytes);
					m_cmd_buffer[size++] = bytes[0];
					m_cmd_buffer[size++] = bytes[1];
					m_cmd_buffer[size++] = bytes[2];
					m_cmd_buffer[size++] = bytes[3];
				}
				//截止数据设置
				for (int i = 0; i < 3; i++)
				{
	#ifdef ZHONGHANG_PRJ
					bool flag = false;
					
					for (int j=0; j<3; ++j)
					{
						if (data.step_data.step_data.general_data.stop1_level[j] == (i + 1))
						{
							uint8 stop_type = data.step_data.step_data.general_data.stop1_type[j];
							m_cmd_buffer[size++] = stop_type;
							m_cmd_buffer[size++] = data.step_data.step_data.general_data.stop1_data[j].func_code;
							memset(bytes, 0x0, 4);
							uint32_to_bytes(data.step_data.step_data.general_data.stop1_data[j].value, bytes);
							m_cmd_buffer[size++] = bytes[0];
							m_cmd_buffer[size++] = bytes[1];
							m_cmd_buffer[size++] = bytes[2];
							m_cmd_buffer[size++] = bytes[3];
							m_cmd_buffer[size++] = data.step_data.step_data.general_data.stop1_result[j].func_code;
							memset(bytes, 0x0, 4);
							uint32_to_bytes(data.step_data.step_data.general_data.stop1_result[j].value, bytes);
							m_cmd_buffer[size++] = bytes[2];
							m_cmd_buffer[size++] = bytes[3];

							flag = true;
						}
					}

					if (!flag)
					{
						memset(&m_cmd_buffer[size], 0xFF, 9);
						size += 9;
					}
	#else
					uint8 stop_type = data.step_data.step_data.general_data.stop1_type[i];
					m_cmd_buffer[size++] = stop_type;
					m_cmd_buffer[size++] = data.step_data.step_data.general_data.stop1_data[i].func_code;
					memset(bytes, 0x0, 4);
					uint32_to_bytes(data.step_data.step_data.general_data.stop1_data[i].value, bytes);
					m_cmd_buffer[size++] = bytes[0];
					m_cmd_buffer[size++] = bytes[1];
					m_cmd_buffer[size++] = bytes[2];
					m_cmd_buffer[size++] = bytes[3];
					m_cmd_buffer[size++] = data.step_data.step_data.general_data.stop1_result[i].func_code;
					memset(bytes, 0x0, 4);
					uint32_to_bytes(data.step_data.step_data.general_data.stop1_result[i].value, bytes);
					m_cmd_buffer[size++] = bytes[2];
					m_cmd_buffer[size++] = bytes[3];
	#endif
				}

				// 单工步保护参数设置
				// 过温保护
				uint32 value = data.step_data.step_data.general_data.protection[0].value;
				if (value == 0xFF)
				{
					m_cmd_buffer[size++] = 0xFF;
					m_cmd_buffer[size++] = 0;
					m_cmd_buffer[size++] = 0;
				}
				else
				{
					m_cmd_buffer[size++] = 0x1;
					memset(bytes, 0x0, 4);
					uint32_to_bytes(value, bytes);
					m_cmd_buffer[size++] = bytes[2];
					m_cmd_buffer[size++] = bytes[3];
				}
				// 充放电保护
				for (int i = 1; i <= 6; i++ )
				{
					uint32 value = data.step_data.step_data.general_data.protection[i].value;
					if (value == 0xFF)
					{
						m_cmd_buffer[size++] = 0xFF;
						m_cmd_buffer[size++] = 0;
						m_cmd_buffer[size++] = 0;
						m_cmd_buffer[size++] = 0;
					}
					else
					{
						m_cmd_buffer[size++] = i+1;
						memset(bytes, 0x0, 4);
						uint32_to_bytes(value, bytes);
						m_cmd_buffer[size++] = bytes[1];
						m_cmd_buffer[size++] = bytes[2];
						m_cmd_buffer[size++] = bytes[3];
					}
				}
				// 跳变保护
				for (int i = 0; i < 4; i++)
				{
					uint32 value = data.step_data.step_data.general_data.protection[7+2*i].value;
					if (value == 0xFF)
					{
						m_cmd_buffer[size++] = 0xFF;
						m_cmd_buffer[size++] = 0;
						m_cmd_buffer[size++] = 0;
						m_cmd_buffer[size++] = 0;
						m_cmd_buffer[size++] = 0;
					}
					else
					{
						m_cmd_buffer[size++] = 0x08+i;
						memset(bytes, 0x0, 4);
						uint32_to_bytes(value, bytes);
						m_cmd_buffer[size++] = bytes[1];
						m_cmd_buffer[size++] = bytes[2];
						m_cmd_buffer[size++] = bytes[3];
						m_cmd_buffer[size++] = data.step_data.step_data.general_data.protection[7+2*i+1].value;
					}
				}
				// 中航项目新增保护参数
	#ifdef ZHONGHANG_PRJ
				for (int i=0; i<3; i++)
				{
					if (data.step_data.step_data.general_data.protection[15 + i*3 + 2].value != 0xFF)
					{
						for (int j=0; j<3; j++)
						{
							int value = data.step_data.step_data.general_data.protection[15 + i*3 + j].value;
							memset(bytes, 0x0, 4);
							int_to_bytes(value, bytes);
							if (j == 2)
							{
								m_cmd_buffer[size++] = 12 + i;			// 使能标志
							}
							else
							{
								m_cmd_buffer[size++] = bytes[0];
							}
							m_cmd_buffer[size++] = bytes[1];
							m_cmd_buffer[size++] = bytes[2];
							m_cmd_buffer[size++] = bytes[3];
						}
					}
					else
					{
						for (int j=0; j<3; j++)
						{
							m_cmd_buffer[size++] = 0xFF;
							m_cmd_buffer[size++] = 0xFF;
							m_cmd_buffer[size++] = 0xFF;
							m_cmd_buffer[size++] = 0xFF;
						}
					}
				}
	#endif
	#ifdef ZHONGHANG_PRJ_NEW
				// 电压电流超差、跨工步保护
				for (int i=0; i<4; i++)
				{
					if (data.step_data.step_data.general_data.protection[24 + i].value != 0xFF)
					{
						int value = data.step_data.step_data.general_data.protection[24 + i].value;
						memset(bytes, 0x0, 4);
						int_to_bytes(value, bytes);

						m_cmd_buffer[size++] = 16 + i;			// 使能标志
						m_cmd_buffer[size++] = bytes[1];
						m_cmd_buffer[size++] = bytes[2];
						m_cmd_buffer[size++] = bytes[3];
					}
					else
					{
						m_cmd_buffer[size++] = 0xFF;			// 禁用标志
						m_cmd_buffer[size++] = 0xFF;
						m_cmd_buffer[size++] = 0xFF;
						m_cmd_buffer[size++] = 0xFF;
					}
				}
				// 温度下限保护
				value = data.step_data.step_data.general_data.protection[28].value;
				if (value == 0xFF)
				{
					m_cmd_buffer[size++] = 0xFF;
					m_cmd_buffer[size++] = 0;
					m_cmd_buffer[size++] = 0;
				}
				else
				{
					m_cmd_buffer[size++] = 20;
					memset(bytes, 0x0, 4);
					uint32_to_bytes(value, bytes);
					m_cmd_buffer[size++] = bytes[2];
					m_cmd_buffer[size++] = bytes[3];
				}
				// 预留
				memset(&m_cmd_buffer[size], 0xFF, 8);
				size += 8;
	#endif
			}

			break;
        }
        case CMD_CHECK_PROTECTION:
        {
            m_cmd_buffer[size++] = data.subList[0].func_code;
            m_cmd_buffer[size++] = 0x01;

            uint8 bytes[2] = {0};
            m_cmd_buffer[size++] = data.subList[1].func_code;
            uint16_to_bytes((uint16)(data.subList[1].value), bytes);
            m_cmd_buffer[size++] = bytes[0];
            m_cmd_buffer[size++] = bytes[1];

            memset(bytes, 0x0, 2);
            m_cmd_buffer[size++] = data.subList[2].func_code;
            uint16_to_bytes((uint16)(data.subList[2].value), bytes);
            m_cmd_buffer[size++] = bytes[0];
            m_cmd_buffer[size++] = bytes[1];
            break;
        }

		case CMD_QUERY_VERSION:
		case CMD_QUERY_ADDRESS:
		    break;

 		default:
            LOGFMTW("Command do nothing!");
            return;
    }

    m_cmd_buffer[0] = size + 3;
    uint16 crc_code = crc16(m_cmd_buffer, size);
    uint16_to_bytes(crc_code, m_cmd_buffer+size);
    m_cmd_buffer[size+2] = 0x0D;

    m_size = m_cmd_buffer[0];
}

#endif

