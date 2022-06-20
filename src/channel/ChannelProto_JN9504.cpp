#include <iostream>
#include <sstream>
#include <string.h>
#include <time.h>
#include <math.h>
#include "log4z.h"
#include "utility.h"

#include "Configuration.h"
#include "ReplyHandler.h"
#include "MsgFactory.h"
#include "FormatConvert.h"
#include "cabinet_db.h"

#include "ChannelProto_JN9504.h"

using namespace std;

ChannelProto_JN9504::ChannelProto_JN9504()
{
#if defined(AWH_FORMATION_SYS)
	m_last_run_state = 0;
#else
	m_last_step_no = 0;
#endif
	feat = 0;
}

ChannelProto_JN9504::ChannelProto_JN9504(uint8_t lastFeature)
{
#if defined(AWH_FORMATION_SYS)
	m_last_run_state = 0;
#else
	m_last_step_no = 0;
#endif
	feat = lastFeature;
}



/********************************************
*	      ��λ������λ������ָ����֡	   	*
*******************************************/
//ȫ�ֱ�������
void ChannelProto_JN9504::globalProtect_set(Channel_Data_t &data,uint8_t *buffer,uint8_t &size)
{
	for (int i=0; i<data.subList_len; ++i)
	{
		//if (data.subList[i].func_code <= GLOBAL_ContactVolPro)
		{
			buffer[size++] = data.subList[i].func_code;
			memcpy(&buffer[size], &data.subList[i].value, 4);
			size += 4;
		}
	}
}

//ȫ�ֱ�����ѯ
void ChannelProto_JN9504::globalProtect_query(uint8_t *buffer,uint8_t &size)
{
	for (int i = 0; i < GLOBAL_ContactVolPro; i++)
	{
		buffer[size++] = i + 1;
	}
}

//��¼���ݲ�ѯ
/*
void ChannelProto_JN9504::processData_query(Channel_Data_t &data,uint8_t *buffer,uint8_t &size)
{
	//buffer[size++] = data.func_data.func_code;
}
*/

//��������
void ChannelProto_JN9504::workstep_set(Channel_Data_t &data,uint8_t *buffer,uint8_t &size)
{
	buffer[size++] = (uint8_t)(data.step_data.step_nbr & 0xFF);
	buffer[size++] = (uint8_t)(data.step_data.step_nbr >> 8);
	buffer[size++] = (uint8_t)data.step_data.step_type;
	
	//�����ֵ�趨
	for (int i = 0; i < 3; i++)
	{
		buffer[size++] = data.step_data.step_data.general_data.spec_data[i].func_code;
		if (data.step_data.step_data.general_data.spec_data[i].func_code != PARAM_TYPE_INVALID)
		{
			memcpy(&buffer[size], &data.step_data.step_data.general_data.spec_data[i].value, 4);
		}
		size += 4;
	}

	//��ֹ��������
	for(int i = 0; i < 4; i++)
	{
		if(data.step_data.step_data.general_data.abort_data[i].func_code != PARAM_TYPE_INVALID)
		{
			uint8 end_level_func = data.step_data.step_data.general_data.abort_level[i];
			end_level_func <<= 4;
			end_level_func |= (0x0F & data.step_data.step_data.general_data.abort_data[i].func_code);
			buffer[size++] = end_level_func;
			
			memcpy(&buffer[size], &data.step_data.step_data.general_data.abort_data[i].value, 4);
		}
		else
		{
			buffer[size++] = PARAM_TYPE_INVALID;
		}
		
		size += 4;
	}

	// ������������������
	for (int i = 0; i < 6; i++)
	{
		buffer[size++] = data.step_data.step_data.general_data.protection[i].func_code;
		if (data.step_data.step_data.general_data.protection[i].func_code != PARAM_TYPE_INVALID)
		{
			memcpy(&buffer[size], &data.step_data.step_data.general_data.protection[i].value, 4);
		}
		size += 4;
	}	
}

//��������״̬����
void ChannelProto_JN9504::workstep_exec_set(Channel_Data_t &data,uint8_t *buffer,uint8_t &size)
{
	buffer[size++] = (uint8_t)((int)data.func_data.value & 0xFF);
	buffer[size++] = (uint8_t)(((int)data.func_data.value >> 8) & 0xFF);
	buffer[size++] = data.func_data.func_code;
}

//����������ѯ
void ChannelProto_JN9504::workstep_query(Channel_Data_t &data,uint8_t *buffer,uint8_t &size)
{
	buffer[size++] = (uint8_t)((int)data.func_data.value & 0xFF);
	buffer[size++] = (uint8_t)(((int)data.func_data.value >> 8) & 0xFF);
}

//У׼��������
void ChannelProto_JN9504::calibrate_param_set(Channel_Data_t &data,uint8_t *buffer,uint8_t &size)
{
	for (int i=0; i<data.subList_len; ++i)
	{
		if (data.subList[i].func_code <= CAL_IlB_D)
		{
			buffer[size++] = data.subList[i].func_code;
			memcpy(&buffer[size], &data.subList[i].value, 4);
			size += 4;
		}
	}
}

//У׼������ѯ
void ChannelProto_JN9504::calibrate_param_query(uint8_t *buffer,uint8_t &size)
{
	for (int i = 0; i < CAL_IlB_D; i++)
	{
		buffer[size++] = i + 1;
	}
}

//AD���ݶ�ȡ
void ChannelProto_JN9504::AD_sample_query(Channel_Data_t &data,uint8_t *buffer,uint8_t &size)
{
	buffer[size++] = data.func_data.func_code;
}

//ϵͳ��Ϣ��ѯ
void ChannelProto_JN9504::system_info_query(Channel_Data_t &data,uint8_t *buffer,uint8_t &size)
{
	buffer[size++] = data.func_data.func_code;
}

//ϵͳָ������
void ChannelProto_JN9504::system_subcmd_set(Channel_Data_t &data,uint8_t *buffer,uint8_t &size)
{
	buffer[size++] = data.func_data.func_code;
	
	switch (data.func_data.func_code)
	{
		// ����澯
		case SUB_CMD_CLEAR_WARNING:
			break;

		// ϵͳʱ���趨
		case SUB_CMD_SET_SYS_TIME:
		{
			time_t tt = time(NULL);
			tt *= 1000;
			memcpy(&buffer[size], &tt, 8);
			size += 8;
			break;
		}
		
		// ������
		case SUB_CMD_BEAT_TRIGGER:
			memcpy(&buffer[size], &data.func_data.value, 4);
			size += 4;
			break;

		// ģʽ����
		case SUB_CMD_WORK_MODE:
			buffer[size++] = (uint8_t)((int)data.func_data.value & 0xFF);
			break;
	
		// ʹ����������
		case SUB_CMD_UPLOAD_ENABLE:
		case SUB_CMD_UPLOAD_DISABLE:
		case SUB_CMD_UPLOAD_CYCLE:
		case SUB_CMD_SYSTEM_INIT:
		case SUB_CMD_SYSTEM_HALT:
		case SUB_CMD_HEARTBEAT_ENABLE:
		case SUB_CMD_HEARTBEAT_DISABLE:
		case SUB_CMD_PROBE_NO_PRESS:
		case SUB_CMD_PROBE_PRESSING:
		case SUB_CMD_PROBE_PRESSED:
			break;
		
		default:
			return;
	}
}

//ϵͳ��������
void ChannelProto_JN9504::system_update_set(Channel_Data_t &data,uint8_t *buffer,uint8_t &size)
{
	buffer[size++] = data.func_data.func_code;
}

//�ڲ���������
void ChannelProto_JN9504::internalProtect_set(Channel_Data_t &data,uint8_t *buffer,uint8_t &size)
{
	for (int i=0; i<data.subList_len; ++i)
	{
		if(data.subList[i].func_code <= INTER_EnvirOverTempPro)
		{
			buffer[size++] = data.subList[i].func_code;
			memcpy(&buffer[size], &data.subList[i].value, 4);
			size += 4;
		}
	}
}

//�ڲ�������ѯ
void ChannelProto_JN9504::internalProtect_query(uint8_t *buffer,uint8_t &size)
{
	for (int i = 0; i < INTER_EnvirOverTempPro; i++)
	{
		buffer[size++] = i + 1;
	}
}

void ChannelProto_JN9504::temperature_query(Channel_Data_t &data,uint8_t *buffer,uint8_t &size)
{
	for (int i=0; i<data.subList_len; ++i)
	{
		buffer[size++] = data.subList[i].func_code;
	}
}

//����֡��֡
uint8_t ChannelProto_JN9504::sendCmdPack(uint8_t *cmd_buffer,int ch_no, int func_code, Channel_Data_t &data)
{
    //memset(cmd_buffer, 0x0, MAX_CMD_SIZE);
    uint8_t size = 0;
	
	// ֡ͷ
    cmd_buffer[size++] = 0x5A;
    cmd_buffer[size++] = 0x5A;
	// ���ݳ���
	size += 2;

	//������
	cmd_buffer[size++] = feat++;
	//״̬��
	cmd_buffer[size++] = 0;
	//ͨ����־(һ��ģ��4��ͨ��)
	if(ch_no > 0)
	{
		cmd_buffer[size++] = 0x01 << ((ch_no-1)%MODULE_CH_NUM);
	}
	else
	{
		cmd_buffer[size++] = 0;
	}
    
	// �������ݶΣ�������
    cmd_buffer[size++] = (uint8_t)func_code;
	
	// �������������
    switch (func_code)
    {
        case CMD_SET_PROTECTION:		//ȫ�ֱ�������        
			globalProtect_set(data,cmd_buffer,size);
            break;
        
		case CMD_QUERY_PROTECTION:  	//ȫ�ֱ�����ѯ
			globalProtect_query(cmd_buffer,size);
			break;

		case CMD_QUERY_PROCESS_DATA: 	//��¼���ݲ�ѯ
			//processData_query(data,cmd_buffer,size);
			break;

		case CMD_SET_STEP:           	// ��������
			workstep_set(data,cmd_buffer,size);
			break;

		case CMD_RUN_CONTROL:			// ��������״̬����
			workstep_exec_set(data,cmd_buffer,size);
            break;

		case CMD_STEP_INQ:
			workstep_query(data,cmd_buffer,size);
			break;	

		case CMD_SET_CALIBRATION:		// �ϻ�У׼����
			calibrate_param_set(data,cmd_buffer,size);
			break;

		case CMD_QUERY_CALIBRATION:		// �ϻ�У׼��ѯ
			calibrate_param_query(cmd_buffer,size);
            break;

		case CMD_AD_DATA_INQ:
			AD_sample_query(data,cmd_buffer,size);
			break;
			
		case CMD_SYSTEM_GET:            //ϵͳ��ѯ����
			system_info_query(data,cmd_buffer,size);
            break;

		case CMD_SYSTEM_SET:     		// ϵͳ��������
        	system_subcmd_set(data,cmd_buffer,size);
            break;
        
		case CMD_SYSTEM_UPDATE:
			break;
			
        case CMD_SET_INTERNAL_PROTECTION:	// �ڲ���������
			internalProtect_set(data,cmd_buffer,size);
            break;

		case CMD_QUERY_INTERNAL_PROTECTION:	// �ڲ�������ѯ
			internalProtect_query(cmd_buffer,size);
		 	break;
			
		case CMD_ACDC_UPDATE:
			break;

		case CMD_QUERY_TEMPERATURE:		//�¶Ȳ�ѯ
			temperature_query(data,cmd_buffer,size);
			break;
			
 		default:
            LOGFMTW("unknown Command 0x%x",func_code);
    }

	// �������
    cmd_buffer[2] = (uint8_t) (size & 0xFF);
    cmd_buffer[3] = (uint8_t) (size >> 8);

	// У���� ModbusCRC16
    uint16_t crcCode = crc16(&cmd_buffer[2], size-2);
    cmd_buffer[size++] = (uint8_t) (crcCode & 0xFF);
    cmd_buffer[size++] = (uint8_t) (crcCode >> 8);
	// ֡β
    cmd_buffer[size++] = 0xA5;
    cmd_buffer[size++] = 0xA5;

    return size;
}


/****************************************************
*	    ��λ����Ӧ������λ��JSON���л�������		*
***************************************************/
void ChannelProto_JN9504::response_reply_send(int value,Response_Data_t &m_data)
{
	// ����ͨ����־�����ͽ������λ��
	//for (int i=0; i<MODULE_CH_NUM; ++i)
	{
		//if ((m_data.ch_mask & (1 << i)) != 0)
		{
            string josn_str = FormatConvert::instance().reply_to_string(m_data.cell_no, m_data.ch_no, m_data.func_code, m_data.sub_code, value);
            MsgFactory::instance()->cell_to_ext_reply_pusher(m_data.cell_no).send(josn_str);
		}
	}
}

void ChannelProto_JN9504::response_reply_send(vector<float> &vals,Response_Data_t &m_data)
{
	// ����ͨ����־�����ͽ������λ��
	//for (int i=0; i<MODULE_CH_NUM; ++i)
	{
		//if ((m_data.ch_mask & (1 << i)) != 0)
		{
            string josn_str = FormatConvert::instance().reply_to_string(m_data.cell_no, m_data.ch_no, m_data.func_code, m_data.sub_code, vals);
            MsgFactory::instance()->cell_to_ext_reply_pusher(m_data.cell_no).send(josn_str);
		}
	}
}


void ChannelProto_JN9504::response_exception_send(int error_code,Response_Data_t &m_data)
{
	// ����ͨ����־ (ÿ��ͨ�����ǵ����ϴ��ĸ澯)
	//for (int i=0; i<MODULE_CH_NUM; ++i)
	{
		//if ((m_data.ch_mask & (1 << i)) != 0)
		{
				string josn_str = FormatConvert::instance().alarm_to_string(error_code, m_data.cell_no, m_data.ch_no, m_data.func_code, m_data.sub_code);
				MsgFactory::instance()->cell_alarm_pusher(m_data.cell_no).send(josn_str);
		}
	}
}


/********************************************
*	      ��λ������λ������ָ���֡	   	*
*******************************************/
//����ָ����Ӧ�Ĵ���
void ChannelProto_JN9504::ack_set_reply(uint8_t *sub_buffer,int &index,int sub_len,Response_Data_t &m_data)
{
	int state = 0;
	// ���������������뷵�صĽ��
	for (int i=0; i<sub_len/2; ++i)
	{
		// �����������ô���
		if (sub_buffer[index + i*2 + 1])
		{
			m_data.sub_code = (sub_buffer[index + i*2] & 0x7F);
			state = sub_buffer[index + i*2 + 1];
			break;
		}
	}

	response_reply_send(state,m_data);
}

//��ѯָ����Ӧ�Ĵ���
void ChannelProto_JN9504::ack_query_reply(uint8_t *sub_buffer,int &index,int sub_len,Response_Data_t &m_data)
{
	vector<float>  vals;
	// ���������������뷵�صĽ��
	for (int i=0; i<sub_len/5; ++i)
	{
		float tmp;
		index += 1;
		memcpy((uint8_t *)&tmp, &sub_buffer[index], 4);
		index += 4;
		vals.push_back(tmp);
	}
	
	response_reply_send(vals,m_data);
}

//��¼������Ӧ�Ĵ���
void ChannelProto_JN9504::ack_process_data_reply(uint8_t *sub_buffer,int &index,int sub_len,Response_Data_t &m_data,ChannelState &m_state)
{
	// ���ݼ���
	Step_Process_Data_t processData = Step_Process_Data_t();			
	memcpy((uint8_t *)&processData.timestamp, &sub_buffer[index], sub_len);

	Step_Process_Addition_Data_t addStatus = m_state.get_addition_status();

	//processData.is_stop = false;
	processData.cell_no = m_data.cell_no;
	processData.ch_no = m_data.ch_no;
	processData.timestamp = millitimestamp();
	
	processData.sum_step = addStatus.sum_step;
	processData.loop_no = addStatus.loop_no;
	processData.batteryTemp = addStatus.batteryTemp;
	processData.vaccum = addStatus.vaccum;
	memcpy((void*)&processData.cellTemp[0],(void*)&addStatus.cellTemp[0],CELL_TEMP_ACQ_NUM*sizeof(float));
	
#if 0
	if(fabs(processData.current) >= 10.0)	//�������ڵ���10A�ż����迹
	{
		resis_calc_flg = true;
	}

	if(fabs(processData.current) < 9.0)	// 1A�Ļ���
	{
		resis_calc_flg = false;
	}

	if(resis_calc_flg)
	{
		processData.r_cotact = fabs((processData.v_cotact - processData.voltage)/processData.current);	
		processData.r_output = fabs((processData.pvol - processData.voltage)/processData.current);
	}
	else
	{
		processData.r_cotact = 0.0;	
		processData.r_output = 0.0;
	}
#else
	if(fabs(processData.current) >= 0.08)		//80mA
	{
		processData.r_cotact = fabs((processData.v_cotact - processData.voltage)/processData.current);
		processData.r_output = fabs((processData.pvol - processData.voltage)/processData.current);
	}
	else
	{
		processData.r_cotact = 0.0;	
		processData.r_output = 0.0;
	}
#endif

#if defined(AWH_FORMATION_SYS)		 //���� �еȴ�״̬
	if(processData.run_state != m_last_run_state)
	{
		if(processData.run_state == STATE_WAITING)		
		{
			m_state.set_step_finish(true);
		}
		m_last_run_state = processData.run_state;
	}
#else								//Ԥ�� ���� �޵ȴ�״̬
	if(processData.run_state != STATE_CONTACT)			//��λ���ᵥ���ۼƽӴ�����ʱ��
	{
		if(processData.run_time < 4.0)					//ֻ��ǰ4��������1�������
		{			
			if(processData.step_no != m_last_step_no)
			{
				/*
				if((processData.run_state == STATE_RUNNING || processData.run_state == STATE_FINISH || processData.run_state == STATE_NO_RUN) 
					&& m_last_step_no > 0 && processData.step_no > 0)
				*/
				if((processData.run_state == STATE_RUNNING || processData.run_state == STATE_START || processData.run_state == STATE_FINISH || processData.run_state == STATE_NO_RUN) 
				  && processData.step_no > 0)
				{
					m_state.set_step_finish(true);
					processData.sum_step += 1;		//�����л��ۼƹ�����1
				}
				
				m_last_step_no = processData.step_no;
			}
		}

		if(processData.sum_step == 65535)			//Ϊ-1 
		{
			processData.sum_step = 0;
		}	

		if((processData.run_state == STATE_WAITING) && (processData.run_time > 1.0))
		{
			m_state.set_stepcache_empty(true);	//Ԥ�� ���� ���ֵȴ�״̬˵����λ���޻��湤��
		}		
	}
#endif

	Step_Process_Record_Data_t step_record;
	step_record.run_state = processData.run_state;
	step_record.voltage = processData.voltage;
	step_record.current = processData.current;
	step_record.pvol = processData.pvol;
	step_record.v_cotact = processData.v_cotact;
	step_record.capacity = processData.capacity;
	step_record.r_cotact = processData.r_cotact;
	step_record.r_output = processData.r_output;
	step_record.run_time = processData.run_time;
#if defined(AWH_FORMATION_SYS)		
	step_record.vaccum = processData.vaccum;
#endif
	step_record.step_no = processData.step_no;
	step_record.CC_CV_flag = processData.CC_CV_flag;
	m_state.update_step_processData(step_record);

	m_state.response_record_hook(processData);
}

////��������ָ����Ӧ�Ĵ���
void ChannelProto_JN9504::ack_step_set_reply(uint8_t *sub_buffer,int &index,Response_Data_t &m_data)
{
	uint16_t stepNo = 0;
	uint8_t resp = 0;
	memcpy((uint8_t *)&stepNo, &sub_buffer[index], 2);

	m_data.sub_code = stepNo;
	index += 2;
	resp = sub_buffer[index++];
	
	response_reply_send((int)resp,m_data);		
}

//��������ָ��Ĵ���
void ChannelProto_JN9504::ack_trig_protection_reply(uint8_t *sub_buffer,int &index,int sub_len,Response_Data_t &m_data,ChannelState &m_state)
{
	int error_code = 0;
	// ����������������
	for (int i=0; i<sub_len; ++i)
	{
		m_data.sub_code = sub_buffer[index++];
		
		switch(m_data.func_code)
		{
			case ACK_TRIG_PROTECTION:
				error_code = -2000;
				break;
				
			case ACK_STEP_TRIG_PROTECTION:
				error_code = -2100;
				break;
				
			case CMD_INTERNAL_TRIG_PROTECT:
				error_code = -2200;
				break;
				
			case CMD_ACDC_TRIG_PROTECT:
				error_code = -2300;
				break;
				
			default:
				error_code = -2500;
				break;
		}
		
		error_code -= m_data.sub_code;
		m_state.push_error_code(error_code);

		response_exception_send(error_code,m_data);
	}
}

//ϵͳ��ѯָ����Ӧ�Ĵ���
void ChannelProto_JN9504::ack_system_get_reply(uint8_t *sub_buffer,int &index,int sub_len,Response_Data_t &m_data,ChannelState &m_state)
{
	if(sub_len > 1)
	{
		m_data.sub_code = sub_buffer[index++];
		
		if (m_data.sub_code == SOFTWARE_VERSION)
		{
			char version_str[64];

			memset(version_str,'\0',sizeof(version_str));
			if(sub_len >= 6)
			{
				float ver_float = 0.0;
				ostringstream v_os;
				
				memcpy((uint8*)&ver_float,&sub_buffer[index],4);
				index += 4;

				v_os.setf(std::ios::fixed);
				v_os.precision(2);	
				v_os << ver_float;

				string tmp = v_os.str();

				sprintf(version_str, "%s%c",tmp.c_str(),sub_buffer[index]);			//ǰ4���ֽ�Ϊfloat ��5�ֽ�ΪASCII��
				index += 1;
			}
			
			m_state.set_version_info(version_str);
		}
	}

}


//ϵͳ����ָ����Ӧ�Ĵ���
void ChannelProto_JN9504::ack_system_set_reply(uint8_t *sub_buffer,int &index,int sub_len,Response_Data_t &m_data,ChannelState &m_state)
{
	m_data.sub_code = sub_buffer[index++];
	int resp = sub_buffer[index++];

	if (m_data.sub_code != SUB_CMD_SET_SYS_TIME && m_data.sub_code != SUB_CMD_BEAT_TRIGGER
		&& m_data.sub_code < SUB_CMD_PROBE_NO_PRESS)
	{			
		response_reply_send(resp,m_data);
	}
	
	if (m_data.sub_code == SUB_CMD_BEAT_TRIGGER)
	{
		m_state.reset_heartbeat_tm();
	}
}


void ChannelProto_JN9504::ack_temperature_reply(uint8_t *sub_buffer,int &index,int sub_len,Response_Data_t &m_data,ChannelState &m_state)
{
	// ���������������뷵�صĽ��
	for (int i=0; i<sub_len/5; ++i)
	{
		Acq_Temperature_Data_t data;
		
		data.chan = sub_buffer[index];
		index += 1;
		memcpy((void *)&data.temp, &sub_buffer[index], 4);
		index += 4;
		
		m_state.push_temperature_data(data);
	}
}

string ChannelProto_JN9504::get_stepname_string(uint8_t stepType)
{
	switch(stepType)
	{
		case STEP_STANDING:
			return "Aging";
		case STEP_CHARGE_CC:
			return "CC";
		case STEP_CHARGE_CV:
			return "CV";
		case STEP_CHARGE_CC_CV:
			return "CCCV";
		case STEP_DISCHARGE_CC:
			return "DCC";
		case STEP_DISCHARGE_CV:
			return "DCV";
		case STEP_DISCHARGE_CC_CV:
			return "DCCCV";
		case STEP_DISCHARGE_CP:
			return "DCW";
		case STEP_CHARGE_CP:
			return "CW";
		case STEP_END:
			return "End";
		case STEP_LOOP:
			return "Loop";
		case STEP_NEG_PRESS:
			return "NegPress";
		case STEP_INVALID:
			return "Invalid";
		default:
			return "None";
	}	
}


bool ChannelProto_JN9504::splicingPacket_resolve(uint8 *data,int dataLen,vector<frame_info_t> &n_frame)
{
	uint16 index = 0;
	
	if(dataLen < 8)
	{
		return false;
	}

	do
	{
		if((data[index] == 0x5B) && (data[index+1] == 0x5B))
		{
			uint16_t len = data[index+3];
			len = (len << 8) + data[index+2];

			if((data[index+2+len] == 0xA5) && (data[index+3+len] == 0xA5))
			{
				frame_info_t frame;
				frame.start_idx = index;
				frame.ch_mask = data[index+6];
				frame.frame_len = len+4;
				index = frame.start_idx + frame.frame_len - 1;
				
				n_frame.push_back(frame);
			}
			else
			{
				return false;
			}
		}
	}while(index++ < dataLen);

	return true;
}


//��Ӧ֡��֡
int ChannelProto_JN9504::recvAckUnPack(RingQueue<uint8> &m_buffer,int m_cell_no,int m_ch_no,int *ack_size,ChannelState &m_state)
{
	uint8_t sub_buffer[MAX_ACK_SIZE];
	int ret = -1;
	int index = 0;
	
	*ack_size = 0;
	// ֡ͷ
	if (m_buffer[index++] != 0x5B)
	{
		*ack_size = index;
		return ret;
	}
	
	if (m_buffer[index++] != 0x5B)
	{
		*ack_size = index;
		return ret;
	}
	
	// ���ݳ���
	int len = m_buffer[index++];
	len |= m_buffer[index++] << 8;
	if ((len + index) > m_buffer.valid_size())  return ret;
	if ((len + index) > MAX_BUFFER_SIZE)
	{
		*ack_size = index;
		return ret;
	}
	
	index += len - 2;
	// ֡β
	if (m_buffer[index++] != 0xA5)
	{
		*ack_size = index;
		return ret;
	}
	if (m_buffer[index++] != 0xA5)
	{
		*ack_size = index;
		return ret;
	}
	// У���� ModbusCRC16	
	memset(sub_buffer, 0x0, MAX_ACK_SIZE);
	m_buffer.buf_copy(sub_buffer, 0, index);
	
    uint16_t crcCode = crc16(sub_buffer + 2, len - 2);
	if ((sub_buffer[index - 4] != (uint8_t)(crcCode & 0xFF)) || (sub_buffer[index - 3] != (uint8_t)(crcCode >> 8)))
	{
		*ack_size = index;
		LOGFMTD("crc err %s %d", __FUNCTION__, __LINE__);
		return ret;
	}
	
	index = 4;
	//������
	//uint8_t feat = sub_buffer[index++];
	index++;
	//״̬��
	//uint8_t state = sub_buffer[index++];
	index++;
	/*
	if(state)
	{
		*ack_size = index;
		return ret;
	}
	*/
	
	Response_Data_t m_data;
	// ͨ����־
	m_data.ch_mask = sub_buffer[index++];
	// �������ݶ�
    m_data.func_code = sub_buffer[index++];		// ������
    m_data.cell_no = m_cell_no;
	m_data.ch_no = m_ch_no;
	m_data.sub_code = -1;
	
	int sub_len = len - 8;						// �����������뼰���ݵ��ܳ���
    switch (m_data.func_code)
    {
		case ACK_SET_PROTECTION:    			// �������û�Ӧ
        case ACK_SET_CALIBRATION:				// У׼���û�Ӧ
        	ack_set_reply(sub_buffer,index,sub_len,m_data);
			break;

	    case ACK_QUERY_PROTECTION:  			// ������ѯ��Ӧ
		case ACK_QUERY_CALIBRATION:				// �ϻ�У׼��ѯ��Ӧ
       		ack_query_reply(sub_buffer,index,sub_len,m_data);
            break;

		case ACK_QUERY_PROCESS_DATA:      		// ��¼���ݷ���
        	ack_process_data_reply(sub_buffer,index,sub_len,m_data,m_state);
        	break;

		case ACK_SET_STEP:	 					// �������û�Ӧ
		case ACK_RUN_CONTROL:					// ��������״̬���û�Ӧ
			ack_step_set_reply(sub_buffer,index,m_data);
			break;
			
		case ACK_STEP_INQ:
			break;

		case ACK_TRIG_PROTECTION:				// ��������
		case ACK_STEP_TRIG_PROTECTION:   		// ��������������
		case CMD_INTERNAL_TRIG_PROTECT:			// �ڲ���������
		case CMD_ACDC_TRIG_PROTECT:				// ACDC��������
			ack_trig_protection_reply(sub_buffer,index,sub_len,m_data,m_state);
	        break;
			
        case CMD_AD_DATA_INQ_REPLY:
			break;

		case ACK_SYSTEM_GET:
			ack_system_get_reply(sub_buffer,index,sub_len,m_data,m_state);
			break;

		case ACK_SYSTEM_SET:					// ϵͳ���û�Ӧ
			ack_system_set_reply(sub_buffer,index,sub_len,m_data,m_state);
			break;
			
		case ACK_QUERY_TEMPERATURE:
			ack_temperature_reply(sub_buffer,index,sub_len,m_data,m_state);
			break;
			
        default:
            break;
    }

	*ack_size = len + 4;		// len + header + tail
    return m_data.func_code;
}




