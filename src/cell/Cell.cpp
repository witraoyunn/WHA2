#include <thread>
#include <sstream>
#include <string>
#include <fstream>
#include <iostream>
#include <sys/types.h>  
#include <sys/stat.h>
#include <errno.h>

#include "log4z.h"
#include "Type.h"
#include "Configuration.h"
#include "FormatConvert.h"
#include "MsgFactory.h"
#include "Cell.h"
#include "Dispatcher.h"
#include "TcpServerMonitor.h"
#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
#include "PinsGroup_AXM.h"
#endif
#include "CANSocketStack.h"


//#define ENV_SAMPLE_TIME   1 		// UNIT:s
#define INTERVAL_BEAT_TIME  2   	// unit:second.
#define INTERVAL_BEAT_TIME_MS	2000   	// unit:millisecond.


using namespace std;

#if defined(AWH_FORMATION_SYS)
Cell::Cell(int cell_no,int can_port) : m_cell_no(cell_no)
#else
Cell::Cell(int cell_no) : m_cell_no(cell_no)
#endif
{
    m_last_beat_time = 0;
	m_last_sync_time = 0;
	m_last_query_time = 0;
	m_last_work_time = 0;
#if defined(AUTO_CONNECT)
	m_chsEnFlag      = 0;
	m_syncTmFlag     = 0;
#endif
	m_state = 0;
	m_last_state = 0;
	m_sync_timeout = 0;
	last_fire_level = 0;
	cell_NG_mask = 0;
	
	m_channel_start_mask.reset();
	m_channel_jump_mask.reset();
	m_channel_wait_mask.reset();
	m_channel_end_mask.reset();
	m_channel_error_mask.reset();
	pass_through_enable = false;
	query_ack_wait = false;
	last_work_flag = false;
	memset((void *)nextStepIdx,0,sizeof(nextStepIdx));
	memset((void *)&pins_cell_status,0,sizeof(Pins_Status_Data_t));
#if defined(FORMATION_TECH_AUTO)	
	memset((void *)&m_bf_sm, 0, sizeof(m_bf_sm));
#endif
#ifdef ZHONGHANG_SYSTEM_PROTECTION		
	memset((void *)m_fireAlarm, 0, sizeof(m_fireAlarm));
#endif
	memset((void *)runStatus,0,sizeof(runStatus));
	memset((void *)channel_temp_alarm,0,sizeof(channel_temp_alarm));
	memset(last_batch_no, 0, sizeof(last_batch_no));

#if defined(AWH_FORMATION_SYS)
	if(can_port > 0)
	{
		m_can_idx = can_port -1;
	}
	else
	{
		m_can_idx = 0;
	}
#endif
#if defined(ENABLE_RUNSTATUS_SAVE)
	char fl_path[128];
	char fl_buff[64];

	sprintf(fl_path,"%s",RUN_STATUS_XML_PATH);
	sprintf(fl_buff,"/SaveStatusCell%d.xml",m_cell_no);
	strcat(fl_path,fl_buff);

	int ret = access(fl_path, 0);			//???????????????????????? ???????????????????????????????????????
	if(ret)
	{
		chanRunStatus.saveRunStatus(fl_path,runStatus,MAX_CHANNELS_NBR,m_cell_no);
	}
#endif
}

Cell& Cell::operator= (const Cell& obj)
{
    if (this != &obj)
    {
        this->m_cell_no        = obj.m_cell_no;
		this->m_last_beat_time = obj.m_last_beat_time;
		this->m_last_sync_time = obj.m_last_sync_time;
		this->m_last_query_time = obj.m_last_query_time;
		this->m_last_work_time = obj.m_last_work_time;
	#if defined(AUTO_CONNECT)	
		this->m_chsEnFlag 	   = obj.m_chsEnFlag;
		this->m_syncTmFlag 	   = obj.m_syncTmFlag;
	#endif	
		this->m_state 			= obj.m_state;
		this->m_last_state 		= obj.m_last_state;
		this->m_sync_timeout 	= obj.m_sync_timeout;
		this->last_fire_level 	= obj.last_fire_level;
		this->cell_NG_mask		= obj.cell_NG_mask;
#if defined(AWH_FORMATION_SYS)		
		this->m_can_idx			= obj.m_can_idx;
#endif
		this->pass_through_enable = obj.pass_through_enable;
		this->query_ack_wait = obj.query_ack_wait;
		this->last_work_flag = obj.last_work_flag;

		m_channel_start_mask.reset();
		m_channel_jump_mask.reset();
		m_channel_wait_mask.reset();
		m_channel_end_mask.reset();
		m_channel_error_mask.reset();
#if defined(FORMATION_TECH_AUTO)		
		memset((void *)&m_bf_sm, 0, sizeof(m_bf_sm));
#endif
#ifdef ZHONGHANG_SYSTEM_PROTECTION		
		memset((void *)m_fireAlarm, 0, sizeof(m_fireAlarm));
#endif
		memset((void *)runStatus,0,sizeof(runStatus));
		memset((void *)nextStepIdx,0,sizeof(nextStepIdx));
		memset((void *)channel_temp_alarm,0,sizeof(channel_temp_alarm));
		memset((void *)&pins_cell_status,0,sizeof(Pins_Status_Data_t));
		memset(last_batch_no, 0, sizeof(last_batch_no));
    }
    return *this;
}

thread Cell::run()
{
    return thread(&Cell::work, this);
}

void Cell::work()
{
    char filter[32] = {'\0'};
    sprintf(filter, TOPIC_CELL_FORMAT_STR, m_cell_no);

    try
    {
		MsgFactory::instance()->create_cell_alarm_pusher(m_cell_no, Configuration::instance()->ext_alarm_socket());
		// ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        MsgFactory::instance()->create_cell_suber(m_cell_no, Configuration::instance()->int_cab_puber_socket(), filter);
		// ??????????????????????????????
		MsgFactory::instance()->create_cell_transmit_pusher(m_cell_no,Configuration::instance()->ext_transmit_pusher_socket());
		// ??????PLC
#if defined(ENABLE_PLC)
        MsgFactory::instance()->create_cell_ctrl_suber(m_cell_no, Configuration::instance()->int_pins_puber_socket(), filter);
        MsgFactory::instance()->create_cell_pusher(m_cell_no, Configuration::instance()->int_pins_puller_socket());
#endif
		// ??????PLC
//#ifdef MITSUBISHI_PLC
//        MsgFactory::instance()->create_cell_ctrl_suber(m_cell_no, Configuration::instance()->int_bgplc_puber_socket(m_cell_no));
//        MsgFactory::instance()->create_cell_pusher(m_cell_no, Configuration::instance()->int_bgplc_puller_socket(m_cell_no));
//#endif
        MsgFactory::instance()->create_cell_to_ext_reply_pusher(m_cell_no, Configuration::instance()->ext_reply_socket());
    }
    catch (zmq::error_t &e)
    {
        LOGFMTE("Create ZMQ communication error. code: %d, description:%s", e.num(), e.what());
        string josn_str = FormatConvert::instance().alarm_to_string(-209);
        MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
        exit(-209);
    }

	// MAX_CHANNELS_NBR???????????????
    try
    {
        for(int i = 0; i < MAX_CHANNELS_NBR; i++)
        {	
        #if defined(AWH_FORMATION_SYS)
            m_channels[i] = ChannelServer(m_cell_no, i + 1,m_can_idx);
		#else
			m_channels[i] = ChannelServer(m_cell_no, i + 1);
		#endif
        }
    }
    catch (int &e)
    {
        LOGFMTE("Construct the Channel failed! Cell NO:%d", m_cell_no);
        exit(e);
    }

	work_logic_init();
	
	// ??????????????????
    while (1)
    {
        msleep(15);
		
#if defined(AUTO_CONNECT)
		net_auto_connect();
#endif
		//????????????????????????
		channel_state_update();

		// ??????????????????????????????
		hearbeat_detect();

		// ??????????????????????????????
        try
        {
			Cmd_Cell_Msg_t receive_data = Cmd_Cell_Msg_t();
#if defined(ENABLE_PLC)
            while(MsgFactory::instance()->cell_ctrl_suber(m_cell_no).recevie(receive_data, ZMQ_DONTWAIT)
                || MsgFactory::instance()->cell_suber(m_cell_no).recevie(receive_data, ZMQ_DONTWAIT))
#else
            while(MsgFactory::instance()->cell_suber(m_cell_no).recevie(receive_data, ZMQ_DONTWAIT))
#endif
            {
				message_handle(receive_data);
			}
		}
		catch (int &e)
		{
			string josn_str = FormatConvert::instance().alarm_to_string(e, m_cell_no);
			MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
		}

		// ????????????????????????
		work_logic_execute();
		// ????????????????????????
		work_logic_recv();
		// ????????????????????????
		work_logic_handle();
	
		long tt_ms = millitimestamp();
		if ((tt_ms - m_last_work_time) > 1000)
		{
			m_last_work_time = tt_ms;
			last_work_flag = true;
			
			//??????????????????
			channel_temperature_update();
			//??????????????????
			pins_status_update();
			//??????????????????
			fireProtection_handle();
		}

		//????????????????????????
		if (query_ack_wait && (device_version.size() >= DEVICE_NUM_ONE_CELL || ((tt_ms - m_last_query_time) > 2000)))
		{
			char mid_version[64];
			
			m_last_query_time = tt_ms;
			if(get_mc_version(mid_version))
			{
				string josn_str = FormatConvert::instance().version_data_to_string(m_cell_no,ACK_QUERY_MC_VERSION,string(mid_version),device_version);
				MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);

				query_ack_wait = false;
			}
		}
    }

    try
    {
        MsgFactory::instance()->destory_cell_suber(m_cell_no);	
		MsgFactory::instance()->destory_cell_transmit_pusher(m_cell_no);
#if defined(ENABLE_PLC)
        MsgFactory::instance()->destory_cell_ctrl_suber(m_cell_no);
        MsgFactory::instance()->destory_cell_pusher(m_cell_no);
#endif
        MsgFactory::instance()->destory_cell_to_ext_reply_pusher(m_cell_no);
    }
    catch (zmq::error_t &e)
    {
        LOGFMTE("Create ZMQ communication error. code: %d, description:%s", e.num(), e.what());
        string josn_str = FormatConvert::instance().alarm_to_string(-209);
        MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
    }
    MsgFactory::instance()->destory_cell_alarm_pusher(m_cell_no);
}

void Cell::channels_linkstate_send(vector<int> &chanArrary,string ip,int state)
{
	int linkstate = state;
	if(cell_NG_mask > 0)
	{
		linkstate |= 0x10;
	}
	string josn_str = FormatConvert::instance().linkstate_to_string(m_cell_no,ACK_CONNECTED,chanArrary,ip,linkstate);
	MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
}

void Cell::cell_alarm_set_error_code(int err_code)
{
	for (int i = 0; i < MAX_CHANNELS_NBR; i++)
	{
		if(m_channel_start_mask[i])
		{
			m_channels[i].set_error_code(err_code);
		}
	}
}

void Cell::set_runstatus_techinfo(int chIdx,int type,char *src)
{
	switch(type)
	{
		//????????????
		case 0:
			strcpy(runStatus[chIdx].BatteryType, src);
			break;
			
		//?????????
		case 1:
			strcpy(runStatus[chIdx].RecipeName, src);
			break;
			
		//?????????
		case 2:
			strcpy(runStatus[chIdx].TrayNo, src);
			break;
			
		//?????????
		case 3:
			strcpy(runStatus[chIdx].BatchNo, src);
			break;
			
		default:
			break;
	}
}

#if defined(ENABLE_RUNSTATUS_SAVE)
bool Cell::channel_saveStatus_active(int chNo,int en)
{
	char xpathRep[128];
	char valueStr[64];

	sprintf(xpathRep, "//runStatus/channel[@No='%d']/Active", chNo);
	sprintf(valueStr, "%d", en);
	chanRunStatus.update_node_value(xpathRep,valueStr);

	return true;
}

bool Cell::update_saveStatus_stepinfo(int chNo)
{
	char xpathRep[128]; 
	char valueStr[128];

	//?????????????????????
	sprintf(xpathRep, "//runStatus/channel[@No='%d']/StepNo", chNo);
	sprintf(valueStr, "%d", runStatus[chNo-1].StepNo);
	chanRunStatus.update_node_value(xpathRep,valueStr);

	//?????????????????????
	sprintf(xpathRep, "//runStatus/channel[@No='%d']/JumpTo", chNo);
	sprintf(valueStr, "%d", runStatus[chNo-1].JumpTo);
	chanRunStatus.update_node_value(xpathRep,valueStr);

	//???????????????
	sprintf(xpathRep, "//runStatus/channel[@No='%d']/LoopNo", chNo);
	sprintf(valueStr, "%d", runStatus[chNo-1].LoopNo);
	chanRunStatus.update_node_value(xpathRep,valueStr);

	//??????????????????
	sprintf(xpathRep, "//runStatus/channel[@No='%d']/StepSum", chNo);
	sprintf(valueStr, "%d", runStatus[chNo-1].StepSum);
	chanRunStatus.update_node_value(xpathRep,valueStr);

	//???????????????????????????
	sprintf(xpathRep, "//runStatus/channel[@No='%d']/TotalTime", chNo);
	sprintf(valueStr, "%d", m_channels[chNo-1].get_total_RunTime());
	chanRunStatus.update_node_value(xpathRep,valueStr);

	//??????????????????????????????
	sprintf(xpathRep, "//runStatus/channel[@No='%d']/Active", chNo);
	sprintf(valueStr, "%d", runStatus[chNo-1].Active);
	chanRunStatus.update_node_value(xpathRep,valueStr);

	return true;
}

bool Cell::update_saveStatus_techinfo(int chNo)
{
	char xpathRep[128]; 
	char valueStr[128];

	//??????????????????
	sprintf(xpathRep, "//runStatus/channel[@No='%d']/BatteryType", chNo);
	if(string(runStatus[chNo-1].BatteryType) != "")
	{
		sprintf(valueStr, "%s", runStatus[chNo-1].BatteryType);
	}
	else
	{
		sprintf(valueStr, "%d", 0);
	}
	chanRunStatus.update_node_value(xpathRep,valueStr);

	//??????????????????
	sprintf(xpathRep, "//runStatus/channel[@No='%d']/RecipeName", chNo);
	if(string(runStatus[chNo-1].RecipeName) != "")
	{
		sprintf(valueStr, "%s", runStatus[chNo-1].RecipeName);
	}
	else
	{
		sprintf(valueStr, "%d", 0);
	}
	chanRunStatus.update_node_value(xpathRep,valueStr);

	//???????????????
	sprintf(xpathRep, "//runStatus/channel[@No='%d']/TrayNo", chNo);
	if(string(runStatus[chNo-1].TrayNo) != "")
	{
		sprintf(valueStr, "%s", runStatus[chNo-1].TrayNo);
	}
	else
	{
		sprintf(valueStr, "%d", 0);
	}
	chanRunStatus.update_node_value(xpathRep,valueStr);

	//???????????????
	sprintf(xpathRep, "//runStatus/channel[@No='%d']/BatchNo", chNo);
	if(string(runStatus[chNo-1].BatchNo) != "")
	{
		sprintf(valueStr, "%s", runStatus[chNo-1].BatchNo);
	}
	else
	{
		sprintf(valueStr, "%d", 0);
	}
	chanRunStatus.update_node_value(xpathRep,valueStr);
	
	//???????????????
	sprintf(xpathRep, "//runStatus/channel[@No='%d']/BatteryNo", chNo);
	if(string(runStatus[chNo-1].BatteryNo) != "")
	{
		sprintf(valueStr, "%s", runStatus[chNo-1].BatteryNo);
	}
	else
	{
		sprintf(valueStr, "%d", 0);
	}
	chanRunStatus.update_node_value(xpathRep,valueStr);

	return true;
}
#endif

void Cell::message_handle(Cmd_Cell_Msg_t &msg)
{
	switch (msg.func_code)
	{
		// ????????????
		case CMD_CONNECT:
		{
#if defined(FORMATION_TECH_AUTO)
			if (m_bf_sm.is_started)
			{
				string josn_str = FormatConvert::instance().alarm_to_string(-2300, m_cell_no);
				MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
				break;
			}
#endif
			for(int i = 1; i <= (MAX_CHANNELS_NBR+MODULE_CH_NUM-1)/MODULE_CH_NUM; i++)
			{
				vector<int> chanArrary;
				
#if defined(AWH_FORMATION_SYS)
				g_TcpServer.get_client_channels(m_cell_no,i,chanArrary);
#else
				string ip = g_TcpServer.get_client_ip(m_cell_no,i); 				
				g_TcpServer.get_client_channels_by_ip(ip,chanArrary);
#endif
				int state = g_TcpServer.get_client_state(m_cell_no,i);
				if(state == Connected) 
				{
#if defined(AWH_FORMATION_SYS)
					channels_linkstate_send(chanArrary,"",NewConnect);
#else
					channels_linkstate_send(chanArrary,ip,NewConnect);
#endif
				}
				else
				{
					channels_linkstate_send(chanArrary,"",NoConnect);
				}	
			}

			break;
		}
		// ??????????????????
		case CMD_DIS_CONNECT:
		{
#if defined(FORMATION_TECH_AUTO)
			if (m_bf_sm.is_started)
			{
				string josn_str = FormatConvert::instance().alarm_to_string(-2301, m_cell_no);
				MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
				break;
			}
#endif
			for (int i = 0; i < MAX_CHANNELS_NBR; i++)
			{
				if (msg.cell_msg.pow.sel_chans[i])
				{
					try
					{
						m_channels[i].disconnect();										
					}
					catch (int &e)
					{
						string josn_str = FormatConvert::instance().alarm_to_string(e, m_cell_no, i+1);
						MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
					}
				}
			}

			break;
		}
		// ????????????????????????Enable????????????????????????
		case ENABLE_RECORD:
		{
			Configuration::instance()->set_send_detail_data_flag(m_cell_no, true);
			break;
		}
		// ????????????????????????Disable????????????????????????
		case DISABLE_RECORD:
		{
			Configuration::instance()->set_send_detail_data_flag(m_cell_no, false);
			break;
		}

		// ??????????????????
		case CMD_SET_TECH_INFO:
		{
			bool is_channel_valid = false;
			for (int i = 0; i < MAX_CHANNELS_NBR; i++)
			{
				if(msg.cell_msg.pow.sel_chans[i])
				{
					is_channel_valid = true;		//???Channels??????
					break;
				}
			}
						
			for (int i=0; i<4; i++)
			{
				if (msg.tech_info[i].str[0] != '\0')
				{
					for (int ch=0; ch<MAX_CHANNELS_NBR; ch++)
					{
						if(is_channel_valid)
						{
							if(msg.cell_msg.pow.sel_chans[ch])
							{
								m_channels[ch].set_tech_info(i, msg.tech_info[i].str);
								set_runstatus_techinfo(ch,i,msg.tech_info[i].str);
							}
						}
						else
						{
							m_channels[ch].set_tech_info(i, msg.tech_info[i].str);
							set_runstatus_techinfo(ch,i,msg.tech_info[i].str);
						}
					}

					if(i == 3)		//?????????
					{
						strcpy(last_batch_no,msg.tech_info[i].str);
					}
				}
			}

			string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, -1, ACK_SET_TECH_INFO, -1, 0);
			MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
			break;
		}
		// ??????????????????
		case CMD_SET_BATTERY_BARCODE:
		{
			for (int i=0; i<MAX_CHANNELS_NBR; ++i)
			{
				//?????????????????????1???????????????????????????
				m_channels[i].clear_channel_info();
				
				if (msg.barcode[i].str[0] != '\0')
				{
					//??????????????????
					m_channels[i].stop_process();
					//???????????????
					m_channels[i].set_barcode(msg.barcode[i].str);
					strcpy(runStatus[i].BatteryNo, msg.barcode[i].str);
				}
				else
				{
					memset(runStatus[i].BatteryNo,0,INFO_SIZE);
				}
			}

			string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, -1, ACK_SET_BATTERY_BARCODE, -1, 0);
			MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
			break;
		}
		//????????????????????????
		case CMD_QUERY_MC_VERSION:
		{
			device_version.clear();
			for(int i = 0; i < (MAX_CHANNELS_NBR+MODULE_CH_NUM-1)/MODULE_CH_NUM; i++)
			{
				m_channels[i*MODULE_CH_NUM].query_version();
			}
			query_ack_wait = true;
			//m_last_query_time = time(NULL);
			m_last_query_time = millitimestamp();		//?????????????????????
			
			break;
		}
		
		//??????????????????
		case CMD_QUERY_ALARM_INFO:		
		//?????????????????????	
		case SYS_ALARM_CODE_REPORT:
		{
			for(int i = 0; i < MAX_CHANNELS_NBR; i++)
			{
				vector<int> err_code;

				err_code = m_channels[i].return_all_error_code();
				for(uint32_t j = 0; j < err_code.size(); j++)
				{
					string josn_str = FormatConvert::instance().alarm_to_string(err_code[j], m_cell_no, i+1, 0, 0);
					MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
				}
			}

			break;
		}
		// ?????????????????????	
		case SYS_COMMON_CMD:
		{
			System_Inside_Msg_t inside_msg = msg.inside_msg;

			switch (inside_msg.sub_code)
			{
#if defined(ZHONGHANG_SYSTEM_PROTECTION)
				case SYS_CELL_ALARM_HANDLE:
					alarmProtection_handle(0, inside_msg.value);
					break;
				case SYS_PINS_ALARM_CLEAR:
					fireAlarmInfo_clear();
					break;
#endif
#if defined(FORMATION_TECH_AUTO)
				case SYS_VAC_FINISH:
					m_bf_sm.is_vac_finish = true;
					break;
				case SYS_PINS_TECH_FINISH:
					m_bf_sm.is_pins_finish = true;
					break;
				case SYS_CELL_VAC_NEXT:
					m_bf_sm.is_vac_next = true;
					break;
#endif
				case SYS_PINS_STATUS_SYNC:
					pins_cell_status = inside_msg.pins_status;
					break;
					
				case SYS_CELL_ALARM_HANDLE:
				{
#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
					if(inside_msg.value != 0)
					{
						cell_alarm_set_error_code(inside_msg.value);
					}
#endif
					m_state = S_PROCESS_ERROR;
					break;
				}

				case SYS_TRAY_STATE_SET:
				{
					for (int i = 0; i < MAX_CHANNELS_NBR; i++)
					{
						m_channels[i].tray_state_set(inside_msg.value);
					}

					break;
				}
					
				case SYS_BLOCK_TEST_ACK:
				{
					if(m_state == S_BLOCK_TEST)
					{
						if(inside_msg.value == 1)
						{
							m_state = S_TRAY_PRESS;
						}
						else
						{
							m_state = S_PROCESS_ERROR;
						}
					}
					
					break;
				}

				case SYS_LEAK_TEST_ACK:
				{
					if(m_state == S_KEEP_VAC_TEST)
					{
						if(inside_msg.value == 1)
						{
							m_state = S_START_STEP; 	//????????????????????????????????????
						}
						else
						{
							m_state = S_PROCESS_ERROR;
						}
					}

					break;
				}

				case SYS_PUMP_VAC_ACK:
					pump_vac_ready = inside_msg.value;
					break;

				case SYS_LEAK_VAC_ACK:
					leak_vac_ready = inside_msg.value;
					break;
			}
			break;
		}

#if defined(FORMATION_TECH_AUTO)
		// ???????????????????????????
		case CMD_AUTO_TECH_CTRL:
		{
			m_bf_sm.tech_type = msg.cell_msg.sub_msg.value;

			switch (msg.cell_msg.sub_msg.code)
			{
				// ????????????
				case 0:
				{
					m_bf_sm.abort_cmd = true;
					string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, -1, ACK_AUTO_TECH_CTRL, msg.cell_msg.sub_msg.code, true);
					MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
					break;
				}
				// ????????????
				case 1:
				{
					if (m_bf_sm.start_cmd)
					{
						string josn_str = FormatConvert::instance().alarm_to_string(-2305, m_cell_no);
						MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
					}
					else
					{
						if (!m_bf_sm.is_pins_auto)
						{
							string josn_str = FormatConvert::instance().alarm_to_string(-2311, m_cell_no);
							MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
							break;
						}
						if (!(m_bf_sm.tech_type >= PINS_TECH_BATTERY && m_bf_sm.tech_type <= PINS_TECH_CALIBRATION))
						{
							string josn_str = FormatConvert::instance().alarm_to_string(-2308, m_cell_no);
							MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
						}
						else
						{
							m_bf_sm.follow_step_no = msg.cell_msg.sub_msg.step_no;
							m_bf_sm.start_cmd = true;
							m_bf_sm.abort_cmd = false;
							string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, -1, ACK_AUTO_TECH_CTRL, msg.cell_msg.sub_msg.code, true);
							MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
						}
					}
					break;
				}
				// ???????????????
				case 2:
				{
					status_sync_pins(SYS_AUTO_TECH_INIT, 0);
					break;
				}
				// ??????????????????
				case 3:
				{
					status_sync_pins(SYS_TECH_ACQ_FINISH, 1);
					break;
				}
				// ?????????
				default:
				{
					string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, -1, ACK_AUTO_TECH_CTRL, msg.cell_msg.sub_msg.code, false);
					MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
					break;
				}
			}
			break;
		}
#endif	
		case CMD_TRANSMIT_ENABLE:
		{
			vector<int> data;
			
			data.push_back(0);

			string josn_str = FormatConvert::instance().transmit_reply_to_string(m_cell_no,-1,"pass_through_start",data);
			MsgFactory::instance()->cell_transmit_pusher(m_cell_no).send(josn_str);
			pass_through_enable = true;
			break;
		}
			
		case CMD_TRANSMIT_DATA:
		{
			if(pass_through_enable)
			{
				uint8 buf[256];
				int data_len = 0;
				
				data_len = msg.cell_msg.pow.data.subList_len;
				for(int i = 0; i < data_len ; i++)
				{
					buf[i] = (uint8)msg.cell_msg.pow.data.subList[i].value;
				}
				
				for (int i = 0; i < MAX_CHANNELS_NBR; i++)
				{
					if (msg.cell_msg.pow.sel_chans[i])
					{
						m_channels[i].transmit_data(buf,data_len);
					}
				}
			}

			break;
		}
		
		case CMD_TRANSMIT_DISABLE:
		{
			vector<int> data;

			data.push_back(0);

			string josn_str = FormatConvert::instance().transmit_reply_to_string(m_cell_no,-1,"pass_through_stop",data);
			MsgFactory::instance()->cell_transmit_pusher(m_cell_no).send(josn_str);
			pass_through_enable = false;

			break;
		}
			
		// ??????????????????????????????
		default:
		{
			bool not_send_flag = false;
#if defined(ENABLE_RUNSTATUS_SAVE)		
			bool status_save_flag = false;
#endif
			Cmd_Channel_Msg_t ch_cmd = Cmd_Channel_Msg_t();
			ch_cmd.func_code = msg.func_code;
			ch_cmd.data 	 = msg.cell_msg.pow.data;

			if(ch_cmd.func_code == CMD_RUN_CONTROL)
			{
#if defined(AWH_FORMATION_SYS)		//??????????????????????????????????????????		
				if(ch_cmd.data.func_data.func_code == EXEC_PROCESS)
				{
					if((m_state > 0) && (m_state < S_PROCESS_ERROR))
					{
						return;
					}
					else
					{
						m_channel_start_mask.reset();
						m_channel_end_mask.reset();
						m_channel_error_mask.reset();
						m_state = 0;

						memset((void*)&m_step_vac,0,sizeof(step_vac_para_t));
						memset((void*)&m_last_step_vac,0,sizeof(step_vac_para_t));
					}
				}
#endif
				m_channel_jump_mask.reset();
			}
			else if((ch_cmd.func_code == CMD_SET_PROTECTION) || (ch_cmd.func_code == CMD_SET_STEP))
			{
				msleep(20);		//???????????????????????????xml??????????????????
			}

			for (int i = 0; i < MAX_CHANNELS_NBR; i++)
			{
				if (msg.cell_msg.pow.sel_chans[i])
				{
					try
					{
						if(ch_cmd.func_code == CMD_SET_PROTECTION)						//??????????????????
						{
							if(m_channels[i].get_globalProtect_para())
							{
								m_channels[i].init_global_exception();
							}
						}
						else if(ch_cmd.func_code == CMD_SET_STEP)						//????????????
						{							
							//????????????
							m_channels[i].get_workstep_recipe();

						#if defined(ENABLE_RUNSTATUS_SAVE)
							status_save_flag = update_saveStatus_techinfo(i+1);			//??????????????????????????????
						#endif
						
							//??????Active
							memset((void*)&runStatus[i],0,sizeof(channel_run_status_t));
							
							not_send_flag = true;
						}
						else if(ch_cmd.func_code == CMD_RUN_CONTROL)					//??????????????????
						{
							if(ch_cmd.data.func_data.func_code == EXEC_PROCESS)			//????????????
							{
								if(m_channels[i].is_protect_alarm_clear())
								{
									m_channel_start_mask[i] = 1;
									m_channel_error_mask[i] = 0;
									nextStepIdx[i] = (uint32)ch_cmd.data.func_data.value - 1;
									
									m_channels[i].set_error_code(0);					//?????????1????????????
#if (defined(AWH_GRADING_SYS) || defined(AWH_PRECHARGE_SYS))									
									m_channels[i].reset_stepcache_status();				//?????????1?????????????????????
#endif
								}
								
								not_send_flag = true;
							}
							else if(ch_cmd.data.func_data.func_code == EXEC_PAUSE)		//????????????
							{
								m_channels[i].set_cmd_pause_flag(true);
							}
							else if(ch_cmd.data.func_data.func_code == EXEC_RESUME)		//????????????
							{
								if(m_channels[i].get_channel_runstate() == STATE_PAUSE)
								{
									if(m_channels[i].get_globalProtect_para())
									{
										m_channels[i].init_global_exception();
									
										global_protect_t global_data;
										Channel_Data_t pow_data;
										m_channels[i].get_globalProtect_info(global_data);
										Dispatcher::instance()->globalProcInfo_resolve(global_data,pow_data);
									
										Cmd_Channel_Msg_t ch_globalPro_cmd;
										ch_globalPro_cmd.ch_no = i+1;
										ch_globalPro_cmd.func_code = CMD_SET_PROTECTION;
										ch_globalPro_cmd.data = pow_data;
									
										m_channels[i].push_command(ch_globalPro_cmd);
									}
									
									runStatus[i].Active = 1;
									m_channel_start_mask[i] = 1;
									m_channel_error_mask[i] = 0;
									m_channels[i].set_cmd_pause_flag(false);
								}
							}
							else if(ch_cmd.data.func_data.func_code == EXEC_STOP)		//????????????
							{
								m_channel_start_mask[i] = 0;
								m_channel_wait_mask[i] = 0;
								m_channel_end_mask[i] = 0;
								m_channel_error_mask[i] = 0;

								//??????Active
								runStatus[i].Active = 0;
								runStatus[i].Finish = 0;
								m_channels[i].set_cmd_pause_flag(false);
							#if defined(ENABLE_RUNSTATUS_SAVE)
								status_save_flag = channel_saveStatus_active(i+1,0);
							#endif
							}
							else if(ch_cmd.data.func_data.func_code == EXEC_JUMP)		//????????????
							{
								if(m_channels[i].is_protect_alarm_clear())
								{
									m_channel_jump_mask[i] = 1;
									nextStepIdx[i] = (uint32)ch_cmd.data.func_data.value - 1;
								}
								
								not_send_flag = true;
							}
						}
						else if(ch_cmd.func_code == CMD_SYSTEM_SET)						//????????????
						{
							if(ch_cmd.data.func_data.func_code == SUB_CMD_CLEAR_WARNING)	//????????????
							{
								m_channels[i].clear_protect_alarm_mask();
							}
						}

						if(!not_send_flag)
						{
					#ifdef PROTOCOL_5V160A
							ch_cmd.ch_no = i + 1;
					#endif
							m_channels[i].push_command(ch_cmd);
						}
					}
					catch (int &e)
					{
						string josn_str = FormatConvert::instance().alarm_to_string(e, m_cell_no, i+1);
						MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
					}
				}
			}

			if(ch_cmd.func_code == CMD_RUN_CONTROL)
			{
				if(ch_cmd.data.func_data.func_code == EXEC_PROCESS)
				{
					if(m_channel_start_mask.count() > 0)	//mask???1?????????
					{
						if((m_state > 0) && (m_state < S_PROCESS_ERROR))	//???????????????
						{
							m_state = S_TRAY_PRESS;			
						}
						else
						{
							m_state = S_PROCESS_START;		//????????????
						}
					}
				}
				else if(ch_cmd.data.func_data.func_code == EXEC_RESUME)
				{
					if(m_channel_start_mask.count() > 0)
					{
						m_state = S_STEP_EXEC;
					}
				}
				else if(ch_cmd.data.func_data.func_code == EXEC_STOP)
				{
#if defined(AWH_FORMATION_SYS)				
					if(Configuration::instance()->is_vacuum_enable())		//????????????
					{
						if((m_state > S_PROCESS_START && m_state < S_PROCESS_END) || pins_cell_status.vacuum < -5.0)
						{
							m_state = S_BREAK_VAC;
							exec_process_status_notify(1);
						}
					}
					else
#endif
					{
						if(m_channel_start_mask.count() == 0)		//????????????????????????????????????????????????
						{
							m_state = S_PROCESS_IDLE;
							exec_process_status_notify(1);
						}
					}
				}
				else if(ch_cmd.data.func_data.func_code == EXEC_JUMP)
				{
					if(m_channel_jump_mask.count() > 0)
					{
						leak_vac_ready = 0;
						pump_vac_ready = 0;
						m_state = S_START_STEP;			//????????????
					}
				}
			}

#if defined(ENABLE_RUNSTATUS_SAVE)
			if(status_save_flag)
			{
				sys_mutex_pend();
				chanRunStatus.endUpdate();
				sys_mutex_post();
			}
#endif			
			// ???????????????????????????
#if defined(ZHONGHANG_SYSTEM_PROTECTION)
			if (msg.func_code == CMD_CLEAR_WARNING)
			{
				fireAlarmInfo_clear();
			}
#endif
			break;
		}
	}
}

#if defined(ENABLE_PLC)
void Cell::send_pins_cmd(Cmd_Pins_Data_t &data_union,uint8 funccode)
{
	Cmd_Pins_Msg_t pin_cmd = Cmd_Pins_Msg_t();
	pin_cmd.cell_no   = m_cell_no;
	pin_cmd.func_code = funccode;
	pin_cmd.pins_msg  = data_union;
	MsgFactory::instance()->cell_pusher(m_cell_no).send(pin_cmd, ZMQ_DONTWAIT);
}
#endif

// ??????????????????????????????
void Cell::exec_process_status_notify(int status_code)
{
	int subcode = 0;

#if defined(AWH_FORMATION_SYS)
	subcode = 1;
#elif defined(AWH_GRADING_SYS)
	subcode = 2;
#elif defined(AWH_PRECHARGE_SYS)
	subcode = 3;
#endif

	string josn_str = FormatConvert::instance().notify_to_string(m_cell_no, CMD_POST_STATUS_INFO, subcode, last_batch_no, status_code,fire_error_code);
	MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
}

bool Cell::loop_workstep_jump(int chIdx, int &vecIdx, uint32 &nextStepIdx, work_step_recipe_t &workstep_info)
{
	//????????????????????????
	bool newItemFlag = true;
	bool is_loop_finish = false;

	for(vecIdx = 0; vecIdx < m_channels[chIdx].get_loop_vector_size(); vecIdx++)
	{
		 if(workstep_info.StepNo == m_channels[chIdx].get_loop_stepNo(vecIdx))
		 {
			newItemFlag = false;
			break;
		 }
	}
	
	if(newItemFlag)
	{
		loopstep_info_t loopInfo;
		loopInfo.StepNo = workstep_info.StepNo;
		loopInfo.LoopCnt = 1;

		if(runStatus[chIdx].LoopNo > 0)			//multiLoopStep????????????????????? ?????????????????????0????????????????????????????????????
		{
			loopInfo.LoopCnt += runStatus[chIdx].LoopNo;
		}
		
		m_channels[chIdx].push_loop_info(loopInfo);
	}
	else
	{
		m_channels[chIdx].increase_loop_count(vecIdx);
	}
	
	//????????????????????????
	if(m_channels[chIdx].get_loop_count(vecIdx) < workstep_info.CycleCount)
	{
		nextStepIdx = workstep_info.CycleFrom - 1;

#if (defined(AWH_GRADING_SYS) || defined(AWH_PRECHARGE_SYS))
		runStatus[chIdx].loop_flag = true;
#else
		runStatus[chIdx].LoopNo += 1;
#endif
		m_channels[chIdx].get_workstep_info(nextStepIdx,workstep_info);
		is_loop_finish = false;
	}
	else
	{
		m_channels[chIdx].set_loop_count(vecIdx,0);		//???????????????????????????
		is_loop_finish = true;
	}

	return is_loop_finish;
}


void Cell::channel_step_startup()
{
	std::bitset<MAX_CHANNELS_NBR> channel_mask;
#if defined(ENABLE_RUNSTATUS_SAVE)
	bool status_save_flag = false;
#endif

	if(m_channel_jump_mask.count() == 0)		//????????????
	{
		channel_mask |= m_channel_start_mask;
	}
	else										//????????????
	{
		channel_mask |= m_channel_jump_mask;
	}

	//?????????????????? ??????????????????
	for (int i = 0; i < MAX_CHANNELS_NBR; i++)
	{
		if(channel_mask[i])
		{
			if(runStatus[i].Active && (m_channel_jump_mask.count() == 0))
			{
				continue;		//????????????????????????
			}
			
			if(m_channels[i].get_workstep_stepsize() > nextStepIdx[i])
			{
				Channel_Data_t pow_data;
				Cmd_Channel_Msg_t ch_cmd;
				work_step_recipe_t m_workstep_info;

#if defined(AWH_FORMATION_SYS)
				if(m_step_vac.step_no != m_last_step_vac.step_no)
				{
					m_last_step_vac = m_step_vac;
				}
#endif
				m_channels[i].get_workstep_info(nextStepIdx[i],m_workstep_info);
				if(strcmp(m_workstep_info.StepName,"Loop") != 0)
				{
#if defined(AWH_FORMATION_SYS)
					if(Configuration::instance()->is_vacuum_enable())	//????????????
					{
						//????????????????????????????????????
						m_step_vac.step_no = m_workstep_info.StepNo;
						m_step_vac.pump_vac = m_workstep_info.VacValue;
						m_step_vac.pump_timeout = m_workstep_info.VacTime;
						m_step_vac.leak_vac = m_workstep_info.LeakValue;
						m_step_vac.leak_timeout = m_workstep_info.LeakTime;
						m_step_vac.in_running = 0;
						
						//????????????
						if(m_step_vac.step_no != m_last_step_vac.step_no)
						{
							if(m_channel_jump_mask.count() == 0)		//????????????
							{
								//??????????????????????????????????????????????????? ??????????????????
								if((m_step_vac.pump_vac - m_last_step_vac.leak_vac) < 0.01)
								{
									pump_vac_ready = 0;
									m_state = S_PUMP_VAC;
									return;
								}
								else
								{
									leak_vac_ready = 0;
									m_state = S_LEAK_VAC;
									return;
								}
							}
							else								//????????????
							{
								//?????????????????????????????????????????????????????????  ??????????????????
								if((m_step_vac.pump_vac - pins_cell_status.vacuum) < 0.01)
								{
									pump_vac_ready = 0;
									m_state = S_PUMP_VAC;
									return;
								}
								else
								{
									leak_vac_ready = 0;
									m_state = S_LEAK_VAC;
									return;
								}
							}
						}
						else
						{
							if(leak_vac_ready)
							{
								leak_vac_ready = 0; 		//?????????????????? ????????????????????????
							}
							m_channel_wait_mask.reset();	//?????????????????? ????????????????????????
						}
					}
#endif
					//??????????????????
					m_channels[i].set_processData_RecTime(m_workstep_info.RecordFrequency);
					//????????????????????????
					m_channels[i].set_stepProtect_Para(m_workstep_info.stepProtect);
					m_channels[i].init_step_exception();
					//????????????????????????????????????
					Dispatcher::instance()->workstepInfo_resolve(m_workstep_info,pow_data);

					//??????????????????
					ch_cmd.ch_no = i+1;
					ch_cmd.func_code = CMD_SET_STEP;
					ch_cmd.data = pow_data;
					m_channels[i].push_command(ch_cmd);

					//??????????????????
					if(m_channel_jump_mask.count() == 0) 		//????????????
					{
					#if defined(AWH_FORMATION_SYS)
						runStatus[i].StepSum = 0;		//???????????????0
					#else
						runStatus[i].StepSum = -1;		//?????????????????????-1
					#endif
						runStatus[i].LoopNo = 0;
						m_channels[i].clear_loop_vector();
						pow_data.func_data.func_code = EXEC_PROCESS;
					}
					else								//????????????
					{
						runStatus[i].Finish = 0;
					#if defined(AWH_FORMATION_SYS)
						runStatus[i].StepSum += 1;
					#endif
						pow_data.func_data.func_code = EXEC_JUMP;
					}
		            pow_data.func_data.value = (float)(nextStepIdx[i]+1);	//??????????????? ?????????
					ch_cmd.func_code = CMD_RUN_CONTROL;
					ch_cmd.data = pow_data;
					m_channels[i].push_command(ch_cmd);

					int nextStepNo = get_next_workstepNo(m_workstep_info);

#if 0
					if(m_channel_jump_mask.count() == 0)	//?????????????????????2????????? ??????????????????????????????
					{
						if(nextStepNo > 0)
						{
							m_channels[i].get_workstep_info(nextStepNo-1,m_workstep_info);
							if(strcmp(m_workstep_info.StepName,"Loop") == 0)
							{
								int loop_vector_idx =0;
								uint32 next_step_idx = 0;

								loop_workstep_jump(i,loop_vector_idx,next_step_idx,m_workstep_info);
							}
							nextStepNo = get_next_workstepNo(m_workstep_info);
						}
						else if(nextStepNo == 0)	//????????????1?????????
						{
							m_workstep_info.StepNo += 1;
							sprintf(m_workstep_info.StepName,"%s","End");
							nextStepNo = -1;
						}

						Dispatcher::instance()->workstepInfo_resolve(m_workstep_info,pow_data);
						
						ch_cmd.func_code = CMD_SET_STEP;
						ch_cmd.data = pow_data;
						m_channels[i].push_command(ch_cmd);
					}
#endif
#if defined(ENABLE_RUNSTATUS_SAVE)
					status_save_flag = channel_saveStatus_active(i+1,1);						
#endif
					channel_step_active(i+1,m_workstep_info.StepNo,nextStepNo);
				}
			}
		}
	}

#if defined(ENABLE_RUNSTATUS_SAVE)
	if(status_save_flag)
	{
		sys_mutex_pend();
		chanRunStatus.endUpdate();
		sys_mutex_post();
	}
#endif

	m_last_state = m_state;
	m_state = S_STEP_EXEC;
}

#if defined(AUTO_CONNECT)
void Cell::net_auto_connect()
{
	m_chsEnFlag  = 0;
	m_syncTmFlag = 0;
	
	for (int i = 0; i < MAX_CHANNELS_NBR; ++i)
	{
		if (!m_channels[i].isConnected())
		{
			try
			{
				if (!((m_bf_sm.tech_type != PINS_TECH_CALIBRATION) && ((i+1)%2 == 0)))
				{
					m_channels[i].connect();
					LOGFMTD("cell-%d CH-%d try connect", m_cell_no, i+1);
				}

				m_chsEnFlag  |= (uint64_t)1 << i;
				m_syncTmFlag |= (uint64_t)1 << i;
			}
			catch (int &e)
			{
				string josn_str = FormatConvert::instance().alarm_to_string(e, m_cell_no, i+1);
				MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
			}	
		}
		else
		{
			m_chsEnFlag |= (uint64_t)1 << i;
		}
	}

	channel_status_sync();
}

// ??????????????????
void Cell::channel_status_sync()
{	
	// ????????????
	if (m_syncTmFlag != 0)
	{
		msleep(300);
		
		for (int i = 0; i < MAX_CHANNELS_NBR; i++)
		{
			if (((m_syncTmFlag >> i) & 1) != 0)
			{
				m_channels[i].sync_system_time();
			}
		}
		m_syncTmFlag = 0;
	}
	// ????????????????????????????????????????????????
#ifdef AUX_VOLTAGE_ACQ
	Cmd_Aux_Param_t msg;
	msg.cell_no         = m_cell_no;
	msg.fuc_code        = SYS_COMMON_CMD;
	msg.status.sub_code = SYS_CELL_CH_EN_SYNC;
	msg.status.value	= m_chsEnFlag;
	MsgFactory::instance()->aux_param_pusher(m_cell_no).send(msg, ZMQ_DONTWAIT);
#endif
}
#endif

void Cell::channel_state_update()
{
	int state;
#ifndef AWH_FORMATION_SYS
	int ch_fd = -1;
	string ip;
#endif	
	
	for(int i = 1; i <= (MAX_CHANNELS_NBR+MODULE_CH_NUM-1)/MODULE_CH_NUM; i++)
	{
		vector<int> chanArrary;
#ifndef AWH_FORMATION_SYS
		ch_fd = g_TcpServer.get_client_connfd(m_cell_no,i);
		if(ch_fd <= 0)
		{
			continue;
		}
#endif
		
		state = g_TcpServer.get_client_state(m_cell_no,i);
		switch(state)
		{
			case NewConnect:
#ifndef AWH_FORMATION_SYS
				m_epoll_cell.addfd(ch_fd,EPOLLIN);
#endif
				g_TcpServer.set_client_state(m_cell_no,i,Connected);

#ifndef AWH_FORMATION_SYS
				ip = g_TcpServer.get_client_ip(m_cell_no,i);					
				if(g_TcpServer.get_client_channels_by_ip(ip,chanArrary))
#else
				if(g_TcpServer.get_client_channels(m_cell_no,i,chanArrary))
#endif
				{
					int chanNo;
					for(uint32 j = 0; j < chanArrary.size(); j++)
					{
						chanNo = chanArrary[j];
						m_channels[chanNo-1].m_is_connected = true;		//????????????
#ifndef AWH_FORMATION_SYS
						m_channels[chanNo-1].set_fd(ch_fd);
#else
						m_channels[chanNo-1].reset_heartbeat();
#endif
					}
				}

#ifndef AWH_FORMATION_SYS
				channels_linkstate_send(chanArrary,ip,NewConnect);
#else
				channels_linkstate_send(chanArrary,"",NewConnect);
#endif
				break;

			case DisConnect:
#ifndef AWH_FORMATION_SYS
				m_epoll_cell.deletefd(ch_fd);
#endif
				g_TcpServer.set_client_state(m_cell_no,i,NoConnect);

#ifndef AWH_FORMATION_SYS
				ip = g_TcpServer.get_client_ip(m_cell_no,i);
				if(g_TcpServer.get_client_channels_by_ip(ip,chanArrary))
#else
				if(g_TcpServer.get_client_channels(m_cell_no,i,chanArrary))
#endif
				{
					int chanNo;
					for(uint32 j = 0; j < chanArrary.size(); j++)
					{
						chanNo = chanArrary[j];
						m_channels[chanNo-1].m_is_connected = false;	//????????????
#ifndef AWH_FORMATION_SYS
						m_channels[chanNo-1].set_fd(-1);
#else
						m_channels[chanNo-1].reset_heartbeat();
#endif
					}
				}

				channels_linkstate_send(chanArrary,"",NoConnect);				
				break;

			default:
				break;
		}		
	}
}

void Cell::work_logic_init()
{
#if defined(ENABLE_RUNSTATUS_SAVE)
	char fl_path[128];
	char fl_buff[64];

	sprintf(fl_path,"%s",RUN_STATUS_XML_PATH);
	sprintf(fl_buff,"/SaveStatusCell%d.xml",m_cell_no);
	strcat(fl_path,fl_buff);
	chanRunStatus.xmlopen(fl_path);
#endif
#if defined(AWH_FORMATION_SYS)
	memset((void*)&m_step_vac,0,sizeof(step_vac_para_t));
	memset((void*)&m_last_step_vac,0,sizeof(step_vac_para_t));
#endif

	for(int i = 0;i < MAX_CHANNELS_NBR;i++)
	{
		m_channels[i].init_globalProtect_para();

#if defined(ENABLE_RUNSTATUS_SAVE)	
		chanRunStatus.getRunStatus(runStatus[i],i+1);
		if(runStatus[i].Active)											//???1??????????????????
		{
			work_step_recipe_t m_workstep_info;
			
			if(m_channels[i].get_globalProtect_para())		
			{
				m_channels[i].init_global_exception();					//????????????????????????
			}
			
			m_channel_start_mask |= (0x00000001 << i);
			m_channels[i].get_workstep_recipe();						//??????????????????
			m_channels[i].set_total_RunTime(runStatus[i].TotalTime);	//?????????????????????
			
			m_channels[i].get_workstep_info(runStatus[i].StepNo-1,m_workstep_info);
			m_channels[i].set_processData_RecTime(m_workstep_info.RecordFrequency);		//??????????????????
			
			m_channels[i].set_tech_info(0, runStatus[i].BatteryType);
			m_channels[i].set_tech_info(1, runStatus[i].RecipeName);
			m_channels[i].set_tech_info(2, runStatus[i].TrayNo);
			m_channels[i].set_tech_info(3, runStatus[i].BatchNo);
			m_channels[i].set_barcode(runStatus[i].BatteryNo);

			strcpy(last_batch_no,runStatus[i].BatchNo);
		}
#endif
		if(Configuration::instance()->is_tempAcq_enable())
		{
			m_channels[i].init_tempAcq_detect_exception();
		}
	}
	
	m_epoll_cell.create(MAX_CHANNELS_NBR);
}

int Cell::get_next_workstepNo(work_step_recipe_t &stepInfo)
{
	int nextStepNo = 0;
	
	if(stepInfo.AbortAction == 1)
	{
		nextStepNo = stepInfo.StepNo+1;
	}
	else if(stepInfo.AbortAction == 2)
	{
		nextStepNo = stepInfo.JumpToStep;
	}
	else
	{
		nextStepNo = 0;
	}

	return nextStepNo;
}

void Cell::channel_step_active(int chanNo,int currentStep,int nextStep)
{
	runStatus[chanNo-1].Active = 1;
	runStatus[chanNo-1].Finish = 0;
	runStatus[chanNo-1].StepNo = currentStep;
	runStatus[chanNo-1].JumpTo = nextStep;
}

void Cell::process_start()
{
	if(Configuration::instance()->is_plc_enable())		//???PLC??????
	{
		Cmd_Pins_Data_t pin_data;
		uint8 pins_cmd;

		if(m_last_state != m_state)
		{
			m_last_state = m_state;
			pins_cmd = CELL_SET_AUTO_MODE;				//?????????????????????
			send_pins_cmd(pin_data,pins_cmd);

#if defined(AWH_FORMATION_SYS)
			if(Configuration::instance()->is_vacuum_enable())	//????????????
			{
				pins_cmd = CELL_TRAY_DOWN;				//????????????
				send_pins_cmd(pin_data,pins_cmd);

				pins_cmd = CELL_VACUUM_TEST_CLEAR;		//??????????????????????????????
				send_pins_cmd(pin_data,pins_cmd);
				
				m_sync_timeout = 0;
				m_last_sync_time = time(NULL);
			}
#endif
		}

#if defined(AWH_FORMATION_SYS)
		if(Configuration::instance()->is_vacuum_enable())		//????????????
		{
			time_t tt = time(NULL);
			if ((tt - m_last_sync_time) >= 1)
			{
				m_last_sync_time = tt;
				
				pins_cmd = PINS_STATUS_QUERY;
				send_pins_cmd(pin_data,pins_cmd);
		
				if(pins_cell_status.pins_unpress)			//?????????????????????
				{
					m_state = S_BLOCK_TEST;		
				}
				else
				{
					if(++m_sync_timeout > 25)
					{			
						string josn_str = FormatConvert::instance().pins_alarm_to_string(-2400-TRAY_UNPRESS_TIMEOUT,m_cell_no);
						MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
						//??????????????????
						pins_cmd = PINS_ALARM_SYNC;
						pin_data.sub_params[0].func_code = TRAY_UNPRESS_TIMEOUT;
						send_pins_cmd(pin_data,pins_cmd);
						
						cell_alarm_set_error_code(-2400-TRAY_UNPRESS_TIMEOUT);
						m_state = S_PROCESS_ERROR;
					}
				}
			}
		}
		else	// ????????? ????????? ??????????????????
#endif
		{
#if defined(AWH_GRADING_SYS)				//??????????????????????????????
			m_state = S_FIREDOOR_CLOSE;
#else
			m_state = S_TRAY_PRESS;
#endif
		}
	}
	else
	{
		m_state = S_START_STEP;
	}
}

void Cell::process_block_test()
{
#if defined(AWH_FORMATION_SYS)
	Cmd_Pins_Data_t pin_data;
	uint8 pins_cmd;

	if(m_last_state != m_state)
	{
		m_last_state = m_state;
		pins_cmd = CELL_BLOCK_TEST_AUTO;				//??????????????????
		send_pins_cmd(pin_data,pins_cmd);
	}
#endif	
}

void Cell::channel_command_push(uint32_t ch_no,uint8_t func_code,Channel_Data_t &pow_data)
{
	Cmd_Channel_Msg_t ch_cmd;
	ch_cmd.ch_no = ch_no;
	ch_cmd.func_code = func_code;
	ch_cmd.data = pow_data;
	
	m_channels[ch_no-1].push_command(ch_cmd);
}

void Cell::channel_step_execute()
{
#if defined(ENABLE_RUNSTATUS_SAVE)		
	bool status_save_flag = false;
#endif

	m_last_state = m_state;
#if defined(AWH_FORMATION_SYS)
	if(Configuration::instance()->is_vacuum_enable())		// ????????????
	{
		for (uint32_t i = 0; i < MAX_CHANNELS_NBR; ++i)
		{
			//?????????????????????????????????
			if((runStatus[i].Finish > 0) && (m_channel_error_mask[i] == 0))
			{
				m_channel_wait_mask[i] = 1;
			}
		}

		if((m_channel_wait_mask == m_channel_start_mask) && (m_channel_start_mask.count() > 0))
		{
			if(leak_vac_ready != 1)		//?????????????????? ?????????????????? ????????????
			{
				m_step_vac.in_running = 1;
				m_state = S_LEAK_VAC;
				return;
			}
		}
		else
		{
			if((m_channel_start_mask.count() == 0) && (m_channel_error_mask.count() > 0))		//???????????????
			{
				m_state = S_BREAK_VAC;
				m_channel_wait_mask.reset();
			}
			return;
		}
	}
#endif

	for (uint32_t i = 0; i < MAX_CHANNELS_NBR; ++i)
	{
		if(runStatus[i].Active)
		{
#if (defined(AWH_GRADING_SYS) || defined(AWH_PRECHARGE_SYS))		
			if(runStatus[i].Finish || m_channels[i].is_stepcache_null())
#else
			if(runStatus[i].Finish)
#endif
			{
				Channel_Data_t pow_data;
#if (defined(AWH_GRADING_SYS) || defined(AWH_PRECHARGE_SYS))
				work_step_recipe_t m_last_workstep_info;

				m_channels[i].get_workstep_info(runStatus[i].StepNo-1,m_last_workstep_info);

				//????????????????????? ??????????????????????????????????????????
				if(m_channels[i].is_stepcache_null() && runStatus[i].Finish == 0)
				{
					uint32_t now_StepNo = m_channels[i].get_channel_step_no();

					if(now_StepNo > 0)
					{
						m_channels[i].get_workstep_info(now_StepNo-1,m_last_workstep_info);
						runStatus[i].StepNo = now_StepNo;
						runStatus[i].JumpTo = get_next_workstepNo(m_last_workstep_info);
					}
					else
					{
						m_channels[i].reset_stepcache_status();
						continue;
					}
				}
#endif
				//???????????????
				if(runStatus[i].JumpTo > 0)
				{
					uint32 nextIndex;
					nextIndex = runStatus[i].JumpTo - 1;
					if(m_channels[i].get_workstep_stepsize() > nextIndex)
					{
						int loop_vec_idx = 0;
#if defined(AWH_FORMATION_SYS)
						bool loop_step_flag = false;
#endif
						bool loopFinish = false;
						work_step_recipe_t m_workstep_info;

#if defined(AWH_FORMATION_SYS)
						if(m_step_vac.step_no != m_last_step_vac.step_no)
						{
							m_last_step_vac = m_step_vac;
						}
#endif

#if (defined(AWH_GRADING_SYS) || defined(AWH_PRECHARGE_SYS))
						if(runStatus[i].Finish > 0)
						{
							if(runStatus[i].loop_flag)
							{
								runStatus[i].loop_flag = false;
								runStatus[i].LoopNo += 1;
							}

							if(runStatus[i].loop_finish)
							{
								if(runStatus[i].loop_clear_delay-- <= 0)
								{
									runStatus[i].loop_finish = false;
									runStatus[i].LoopNo = 0;
								}
							}
						}
#endif
						m_channels[i].get_workstep_info(nextIndex,m_workstep_info);
						//?????????????????????????????????????????????????????????
						if(strcmp(m_workstep_info.StepName,"Loop") == 0)
						{
#if defined(AWH_FORMATION_SYS)
							loop_step_flag = true;
#endif
							loopFinish = loop_workstep_jump(i,loop_vec_idx,nextIndex,m_workstep_info);
						}

						//????????????????????????????????????????????????????????????
						if((strcmp(m_workstep_info.StepName,"Loop") != 0) || loopFinish)
						{
#if defined(AWH_FORMATION_SYS)
							if(Configuration::instance()->is_vacuum_enable())		//????????????
							{
								if(strcmp(m_workstep_info.StepName,"Loop") != 0)
								{										
									//?????????????????????????????????
									m_step_vac.step_no = m_workstep_info.StepNo;
									m_step_vac.pump_vac = m_workstep_info.VacValue;
									m_step_vac.pump_timeout = m_workstep_info.VacTime;
									m_step_vac.leak_vac = m_workstep_info.LeakValue;
									m_step_vac.leak_timeout = m_workstep_info.LeakTime;
									m_step_vac.in_running = 1;

									//??????????????????
									if(m_step_vac.step_no != m_last_step_vac.step_no)
									{
										//???????????????????????? ??????????????????1???????????????
										if(loop_step_flag)
										{
											m_channels[i].decrease_loop_count(loop_vec_idx);
											runStatus[i].LoopNo -= 1;
										}
									
										//??????????????????????????????????????????????????? ??????????????????
										if((m_step_vac.pump_vac - m_last_step_vac.leak_vac) < 0.01)
										{
											leak_vac_ready = 1;		//???1?????????????????????
											pump_vac_ready = 0;
											m_state = S_PUMP_VAC;
											return;
										}
										else
										{
											leak_vac_ready = 0;
											m_state = S_LEAK_VAC;
											return;
										}
									}
									else
									{
										if(leak_vac_ready)
										{
											leak_vac_ready = 0; 		//?????????????????? ????????????????????????
										}
										m_channel_wait_mask.reset();	//?????????????????? ????????????????????????
									}
								}
							}
#endif
							//?????????????????????
							runStatus[i].StepNo = nextIndex+1;

							//?????????????????????
							int jumpTo = get_next_workstepNo(m_workstep_info);
							//?????????????????????
							runStatus[i].JumpTo = jumpTo;
							
							if(loopFinish && jumpTo > 0)		//??????????????? ???????????????
							{
#if (defined(AWH_GRADING_SYS) || defined(AWH_PRECHARGE_SYS))
								runStatus[i].loop_finish = true;
								runStatus[i].loop_clear_delay = 1;
#else
								runStatus[i].LoopNo = 0;
#endif
							}
								
							if(!loopFinish) //?????????????????????????????? Finish?????????1  
							{
								if(runStatus[i].Finish > 0)
								{
									runStatus[i].Finish = 0;
									runStatus[i].StepSum += 1;
								}
#if defined(AWH_FORMATION_SYS)
								//????????????????????????????????????
								m_channels[i].set_processData_RecTime(m_workstep_info.RecordFrequency);
								m_channels[i].set_stepProtect_Para(m_workstep_info.stepProtect);
#elif (defined(AWH_GRADING_SYS) || defined(AWH_PRECHARGE_SYS))
								//????????????????????????????????????
								if(m_channels[i].is_stepcache_null() && runStatus[i].Finish == 0)
								{
									m_channels[i].set_processData_RecTime(m_workstep_info.RecordFrequency);
									m_channels[i].set_stepProtect_Para(m_workstep_info.stepProtect);
								}
								else
								{
									m_channels[i].set_processData_RecTime(m_last_workstep_info.RecordFrequency);
									m_channels[i].set_stepProtect_Para(m_last_workstep_info.stepProtect);
								}
#endif
								m_channels[i].init_step_exception();

								Dispatcher::instance()->workstepInfo_resolve(m_workstep_info,pow_data);
								channel_command_push(i+1,CMD_SET_STEP,pow_data);		//????????????
							}
						}
					}
				}
				else if(runStatus[i].JumpTo == 0)
				{
#if (defined(AWH_GRADING_SYS) || defined(AWH_PRECHARGE_SYS))
					//????????????????????????????????????
					m_channels[i].set_processData_RecTime(m_last_workstep_info.RecordFrequency);
					m_channels[i].set_stepProtect_Para(m_last_workstep_info.stepProtect);
					m_channels[i].init_step_exception();

					runStatus[i].JumpTo = -1;
					runStatus[i].StepSum += 1;
#endif
					//????????????????????????????????????
					work_step_recipe_t endStep;

					endStep.StepNo = runStatus[i].StepNo + 1;
					sprintf(endStep.StepName,"%s","End");

					Dispatcher::instance()->workstepInfo_resolve(endStep,pow_data);
					channel_command_push(i+1,CMD_SET_STEP,pow_data);		//????????????

					//??????Finish
					runStatus[i].Finish = 0;
#if defined(AWH_FORMATION_SYS)
					//??????Active
					runStatus[i].Active = 0;
					//??????????????????
					m_channel_end_mask[i] = 1;
					m_channel_start_mask[i] = 1;
#endif
				}
#if (defined(AWH_GRADING_SYS) || defined(AWH_PRECHARGE_SYS))
				else if(runStatus[i].JumpTo < 0)
				{
					//??????Active
					memset((void*)&runStatus[i],0,sizeof(channel_run_status_t));
					//??????????????????
					m_channel_end_mask[i] = 1;
					m_channel_start_mask[i] = 1;
				}

				if(m_channels[i].is_stepcache_null())
				{
					m_channels[i].reset_stepcache_status();
				}
#endif

#if defined(ENABLE_RUNSTATUS_SAVE)
				if(runStatus[i].Finish == 0)
				{
					status_save_flag = update_saveStatus_stepinfo(i+1);
				}
#endif
			}
		}
	}

#if defined(ENABLE_RUNSTATUS_SAVE)
	if(status_save_flag)
	{
		sys_mutex_pend();
		chanRunStatus.endUpdate();
		sys_mutex_post();
	}
#endif

	if((m_channel_end_mask == m_channel_start_mask) && (m_channel_start_mask.count() > 0))	//??????????????????
	{
		m_state = S_PROCESS_END;
		m_channel_wait_mask.reset();
		m_last_sync_time = time(NULL);
	}

	if((m_channel_start_mask.count() == 0) && (m_channel_error_mask.count() > 0))		//???????????????
	{
		m_state = S_PROCESS_ERROR;
		m_channel_wait_mask.reset();
	}
}

//???????????????????????????
#if defined(AWH_GRADING_SYS)
void Cell::firedoor_close_exec()
{
	if(Configuration::instance()->is_plc_enable())		//???PLC??????
	{
		Cmd_Pins_Data_t pin_data;
		uint8 pins_cmd;

		if(m_last_state != m_state)
		{
			m_last_state = m_state;
			m_sync_timeout = 0;

			pins_cmd = CELL_FIRE_DOOR_UP;			//???????????????
			send_pins_cmd(pin_data,pins_cmd);
			m_last_sync_time = time(NULL);
		}

		time_t tt = time(NULL);
		if ((tt - m_last_sync_time) >= 1)
		{
			m_last_sync_time = tt;
			
			//pins_cmd = PINS_STATUS_QUERY;
			//send_pins_cmd(pin_data,pins_cmd);

			if(!pins_cell_status.fire_door_open)
			{
				m_state = S_TRAY_PRESS; 			//?????????????????????????????????
			}
			else
			{
				if(++m_sync_timeout > 25)
				{
					string josn_str = FormatConvert::instance().pins_alarm_to_string(-2400-FIREDOOR_CLOSE_TIMEOUT,m_cell_no);
					MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);

					//??????????????????
					pins_cmd = PINS_ALARM_SYNC;
					pin_data.sub_params[0].func_code = FIREDOOR_CLOSE_TIMEOUT;
					send_pins_cmd(pin_data,pins_cmd);
					
					cell_alarm_set_error_code(-2400-FIREDOOR_CLOSE_TIMEOUT);
					m_state = S_PROCESS_ERROR;	
				}
			}
		}
	}
	else	
	{
		m_state = S_START_STEP;
	}
}
#endif

void Cell::tray_press_exec()
{	
	if(Configuration::instance()->is_plc_enable())		//???PLC??????
	{
		Cmd_Pins_Data_t pin_data;
		uint8 pins_cmd;

		if(m_last_state != m_state)
		{
			m_last_state = m_state;
			m_sync_timeout = 0;
#if defined(AWH_PRECHARGE_SYS)
			pins_cmd = CELL_PROBE_EXTEND;
#else
			pins_cmd = CELL_TRAY_UP;
#endif
			send_pins_cmd(pin_data,pins_cmd);
		}

		time_t tt = time(NULL);
		if ((tt - m_last_sync_time) >= 1)
		{
			m_last_sync_time = tt;
			
			//pins_cmd = PINS_STATUS_QUERY;
			//send_pins_cmd(pin_data,pins_cmd);
#if defined(AWH_PRECHARGE_SYS)
			if(pins_cell_status.is_tray_ready && pins_cell_status.is_extend)
#else
			if(pins_cell_status.tray_in_place && pins_cell_status.pins_extend)
#endif
			{
#if defined(AWH_FORMATION_SYS)			
				if(Configuration::instance()->is_vacuum_enable())		
				{
					pump_vac_ready = 0;
					leak_vac_ready = 0;
					m_state = S_KEEP_VAC_TEST;	//??????????????? ??????????????????
				}
				else
#endif
				{
					m_state = S_START_STEP;		// ????????? ??????????????????
				}
				msleep(100);
			}
			else
			{
				m_sync_timeout++;
				if(m_sync_timeout > 30)
				{
#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))				
					string josn_str = FormatConvert::instance().pins_alarm_to_string(-2400-TRAY_PRESS_TIMEOUT,m_cell_no);
					MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);

					//??????????????????
					pins_cmd = PINS_ALARM_SYNC;
					pin_data.sub_params[0].func_code = TRAY_PRESS_TIMEOUT;
					send_pins_cmd(pin_data,pins_cmd);
						
					cell_alarm_set_error_code(-2400-TRAY_PRESS_TIMEOUT);
#endif					
					m_state = S_PROCESS_ERROR;
				}
				else if(m_sync_timeout == 15)		//???????????? ??????????????????????????????
				{
#if defined(AWH_PRECHARGE_SYS)
					pins_cmd = CELL_PROBE_EXTEND;
#else
					pins_cmd = CELL_TRAY_UP;
#endif
					send_pins_cmd(pin_data,pins_cmd);
				}
			}
		}
	}
	else	
	{
		m_state = S_START_STEP;
	}
}

void Cell::process_keep_vac_test()
{
#if defined(AWH_FORMATION_SYS)
	Cmd_Pins_Data_t pin_data;
	uint8 pins_cmd;

	if(m_last_state != m_state)
	{
		m_last_state = m_state;
		pins_cmd = CELL_LEAK_RATE_TEST_AUTO;			// ??????????????????
		send_pins_cmd(pin_data,pins_cmd);
	}
#endif
}

void Cell::process_pump_vac_task()
{
#if defined(AWH_FORMATION_SYS)
	if(m_last_state != m_state)
	{
		Cmd_Pins_Data_t pin_data;
		uint8 pins_cmd;
		
		m_last_state = m_state;

		pin_data.sub_params[0].func_code = 0;
		pin_data.sub_params[0].value = m_step_vac.pump_vac;
		pin_data.sub_params[1].func_code = 1;
		pin_data.sub_params[1].value = (float)m_step_vac.pump_timeout;

		pins_cmd = CELL_START_VACUUM_AUTO;			
		send_pins_cmd(pin_data,pins_cmd);
	}
	else
	{
		if(pump_vac_ready == 1)		//???????????????
		{
			if(m_step_vac.in_running)
			{
				m_state = S_STEP_EXEC;   	//????????????
			}
			else
			{
				m_state = S_START_STEP;		//????????????
			}
		}
	}
#endif
}

void Cell::process_leak_vac_task()
{
#if defined(AWH_FORMATION_SYS)
	if(m_last_state != m_state)
	{
		Cmd_Pins_Data_t pin_data;
		uint8 pins_cmd;
		
		m_last_state = m_state;

		pin_data.sub_params[0].func_code = 0;
		pin_data.sub_params[1].func_code = 1;
		//????????????
		if((m_step_vac.step_no != m_last_step_vac.step_no) && (m_last_step_vac.step_no != 0))
		{
			if(m_channel_jump_mask == 0)		//????????????
			{
				//????????????????????????????????????????????????????????????????????????????????????????????????????????????
				if((m_step_vac.pump_vac - m_last_step_vac.leak_vac) < 0.01)
				{
					pin_data.sub_params[0].value = m_last_step_vac.leak_vac;
					pin_data.sub_params[1].value = (float)m_last_step_vac.leak_timeout;
				}
				else
				{
					pin_data.sub_params[0].value = m_step_vac.pump_vac;
					pin_data.sub_params[1].value = (float)m_step_vac.pump_timeout;
				}
			}
			else								//????????????
			{
				pin_data.sub_params[0].value = m_step_vac.pump_vac;
				pin_data.sub_params[1].value = (float)m_step_vac.pump_timeout;
			}
		}
		else
		{
			pin_data.sub_params[0].value = m_step_vac.leak_vac;
			pin_data.sub_params[1].value = (float)m_step_vac.leak_timeout;
		}
		
		pins_cmd = CELL_DRAIN_VACUUM_AUTO;			
		send_pins_cmd(pin_data,pins_cmd);
	}
	else
	{
		if(leak_vac_ready == 1)		//???????????????
		{
			if(m_step_vac.in_running)
			{
				m_state = S_STEP_EXEC;		//????????????
			}
			else
			{
				m_state = S_START_STEP; 	//????????????
			}
			pump_vac_ready = 0;
		}
	}
#endif
}


void Cell::process_break_vac_task()
{
#if defined(AWH_FORMATION_SYS)
	if(m_last_state != m_state)
	{
		Cmd_Pins_Data_t pin_data;
		uint8 pins_cmd;
	
		m_last_state = m_state;
		pin_data.sub_params[0].func_code = 0;
		pin_data.sub_params[0].value = -1.0;		//????????????
		pin_data.sub_params[1].func_code = 1;
		pin_data.sub_params[1].value = 300.0;

		pins_cmd = CELL_OPEN_DRAIN_VACUUM;			
		send_pins_cmd(pin_data,pins_cmd);
	}
	else
	{
		if(leak_vac_ready == 1)			//???????????????
		{
			m_state = S_PROCESS_IDLE;

			if((m_channel_start_mask.count() > 0) || (m_channel_error_mask.count() > 0))
			{
				if((m_channel_start_mask.count() > 0) && (m_channel_start_mask == m_channel_end_mask))
				{
					exec_process_status_notify(0);		//????????????
				}
				else
				{
					exec_process_status_notify(2);		//????????????	??????
				}
			}
			
			for(int i = 0; i < MAX_CHANNELS_NBR; ++i)
			{
				if(m_channel_start_mask[i])
				{
					m_channel_start_mask[i] = 0;
					m_channel_end_mask[i] = 0;
				}
			}
		}
	}
#endif	
}

#if 0
void Cell::tray_unpress_exec()
{
#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS)) 
	if(Configuration::instance()->is_plc_enable())		//???PLC??????
	{
		Cmd_Pins_Data_t pin_data;
		uint8 pins_cmd;

		if(m_last_state != m_state)
		{
			m_last_state = m_state;
			m_sync_timeout = 0;
			pins_cmd = CELL_TRAY_DOWN;
			send_pins_cmd(pin_data,pins_cmd);
		}

		time_t tt = time(NULL);
		if ((tt - m_last_sync_time) >= 1)
		{
			m_last_sync_time = tt;
			
			//pins_cmd = PINS_STATUS_QUERY;
			//send_pins_cmd(pin_data,pins_cmd);
	
			if(pins_cell_status.tray_in_place && pins_cell_status.pins_unpress)
			{
				m_state = S_PROCESS_IDLE;
			}
			else
			{
				m_sync_timeout++;
				if(m_sync_timeout > 20)
				{
					m_state = S_PROCESS_ERROR;
				}
			}
		}
	}
	else
#endif		
	{
		m_state = S_PROCESS_IDLE;
	}
}
#endif

void Cell::process_end()
{
	time_t tt = time(NULL);
	if ((tt - m_last_sync_time) >= 1)
	{
#if defined(AWH_FORMATION_SYS)
		std::bitset<MAX_CHANNELS_NBR> start_mask(m_channel_start_mask);
		std::bitset<MAX_CHANNELS_NBR> end_mask(m_channel_end_mask);
#endif
		m_last_sync_time = tt;

		for(int i = 0; i < MAX_CHANNELS_NBR; ++i)
		{
			if(m_channel_start_mask[i])
			{
				m_channels[i].stop_process();

				m_channel_start_mask[i] = 0;
				m_channel_wait_mask[i] = 0;
				m_channel_end_mask[i] = 0;
			}
		}

#if defined(AWH_FORMATION_SYS)
		if(Configuration::instance()->is_plc_enable())				//???PLC??????
		{
			if(Configuration::instance()->is_vacuum_enable())		//???????????????
			{
				m_channel_start_mask |= start_mask;
				m_channel_end_mask |= end_mask;
				m_state = S_BREAK_VAC;
			}
			else
			{
				//m_state = S_TRAY_UNPRESS;
				exec_process_status_notify(0);
				m_state = S_PROCESS_IDLE;
			}
		}
		else
#endif
		{
			exec_process_status_notify(0);
			m_state = S_PROCESS_IDLE;
		}
	}
}


void Cell::process_error_proc()
{
#if defined(ENABLE_RUNSTATUS_SAVE)
	bool status_save_flag = false;
#endif

	if(m_channel_start_mask.count() > 0 || m_channel_error_mask.count() > 0)
	{
		exec_process_status_notify(2);
	}
		
	for(int i = 0; i < MAX_CHANNELS_NBR; ++i)
	{
		if(m_channel_start_mask[i])
		{
			m_channels[i].stop_process();

			m_channel_start_mask[i] = 0;
			m_channel_end_mask[i] = 0;

			//??????Active
			memset((void*)&runStatus[i],0,sizeof(channel_run_status_t));
			
#if defined(ENABLE_RUNSTATUS_SAVE)
			status_save_flag = channel_saveStatus_active(i+1,0);
#endif
		}
		else
		{
			m_channel_error_mask[i] = 0;
		}
	}

#if defined(ENABLE_RUNSTATUS_SAVE)
	if(status_save_flag)
	{
		sys_mutex_pend();
		chanRunStatus.endUpdate();
		sys_mutex_post();
	}
#endif

	m_state = S_PROCESS_IDLE;	
}


void Cell::work_logic_execute()
{
#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))  //?????? ??????
	switch(m_state)
	{
		case S_PROCESS_START:		//??????
			process_start();			
			break;

		case S_BLOCK_TEST:			//????????????
			process_block_test();
			break;
			
#if defined(AWH_GRADING_SYS)		
		case S_FIREDOOR_CLOSE:		//?????????????????????????????????
			firedoor_close_exec();
			break;
#endif			
		case S_TRAY_PRESS:			//????????????
			tray_press_exec();
			break;
			
		case S_KEEP_VAC_TEST:		//????????????
			process_keep_vac_test();
			break;
			
		case S_PUMP_VAC:			//?????????
			process_pump_vac_task();
			break;
			
		case S_START_STEP:			//????????????
			channel_step_startup();
			break;
			
		case S_LEAK_VAC:			//??????
			process_leak_vac_task();
			break;
			
		case S_STEP_EXEC:			//????????????
			channel_step_execute();
			break;
			
		case S_PROCESS_END:			//????????????
			process_end();
			break;
			
		case S_BREAK_VAC:			//?????????
			process_break_vac_task();
			break;
			
		case S_PROCESS_ERROR:		//??????
			process_error_proc();
			break;
			
		case S_PROCESS_IDLE:		//??????
			break;

		default:
			break;
	}
#else		//??????
	switch(m_state)
	{
		case S_PROCESS_START:		//??????
			m_last_state = m_state;
			m_state = S_TRAY_PRESS;
			break;
			
		case S_TRAY_PRESS:			//????????????
			tray_press_exec();
			break;
		
		case S_START_STEP:			//????????????
			channel_step_startup();
			break;
			
		case S_STEP_EXEC:			//????????????
			channel_step_execute();
			break;
			
		case S_PROCESS_END: 		//????????????
			process_end();
			break;

		//case S_TRAY_UNPRESS:		//????????????
		
		case S_PROCESS_ERROR:		//??????
			process_error_proc();
			break;

		case S_PROCESS_IDLE:		//??????
			break;

		default:
			break;
	}
#endif

	for(int i = 0; i < MAX_CHANNELS_NBR; ++i)
	{
		m_channels[i].execute();
	}
}

void Cell::work_logic_recv()
{
#if (defined(AWH_GRADING_SYS) || defined(AWH_PRECHARGE_SYS))
	int fd = 0;
	unsigned int ch_Event = 0;
	int nfds = 0;

	nfds = m_epoll_cell.waitEvents(2);
	
	for (int i = 0; i < nfds; i++)
	{
		m_epoll_cell.getEvents(fd,ch_Event);
		if (ch_Event & EPOLLIN)
		{	
			int deviceNo;
			vector<int> channels;
			bool found = g_TcpServer.get_client_channels_by_fd(fd,deviceNo,channels);
			//printf("epoll wait get fd %d\n",fd);

			uint8 tmp[1024];
			int tmpLen = 0;

			memset(tmp,0,sizeof(tmp));
			tmpLen = recv(fd,tmp,1024,0);
			//printf("tmpLen=%d ch_Event=%x\n",tmpLen,ch_Event);

			if(tmpLen>0 && found)
			{
				//LOGFMTD("pass_through_enable = %d",pass_through_enable);
				vector<frame_info_t> n_frame;
				if(ChannelProto_JN9504::splicingPacket_resolve(tmp,tmpLen,n_frame))	//????????????
				{
					int chan = 1;
					recvData_t onePack;
					uint16 offset = 0;

					if(!pass_through_enable)
					{
						for(uint32 j=0; j < n_frame.size(); j++)
						{
							if(n_frame[j].ch_mask != 0)		//??????????????????
							{
								for(uint32 k=0; k < channels.size(); k++)
								{
									if(n_frame[j].ch_mask & (1 << k))	
									{
										chan = channels[k];
										offset = n_frame[j].start_idx;
										onePack.len = n_frame[j].frame_len;
										memcpy(onePack.data, &tmp[offset], onePack.len);
										
										m_channels[chan-1].recv_data(onePack);

										break;	//????????????????????????
									}
								}
							}
							else	//???????????????0 ????????????????????????
							{
								offset = n_frame[j].start_idx;
								onePack.len = n_frame[j].frame_len;
								memcpy(onePack.data, &tmp[offset], onePack.len);
								
								for(uint32 k=0; k < channels.size(); k++)
								{
									chan = channels[k];
									m_channels[chan-1].recv_data(onePack);
								}
							}

							if(onePack.data[7] != 0x8D || (onePack.data[8] != 0x03 && onePack.data[8] < 0x0C))
							{
								LOGFMTD("recv<--%d-%2d: %s", m_cell_no, chan, hexstring(onePack.data, onePack.len).c_str());
							}						
						}
					}
					else
					{
						for(uint32 j=0; j < n_frame.size(); j++)
						{
							offset = n_frame[j].start_idx;
							onePack.len = n_frame[j].frame_len;
							memcpy(onePack.data, &tmp[offset], onePack.len);

							//?????????????????????????????????
							if(onePack.data[7] != 0x8D && onePack.data[8] != 0x03)
							{
								int chIdx = (deviceNo-1)*MODULE_CH_NUM;
								m_channels[chIdx].reset_heartbeat();
							}

							// ??????????????????????????????????????????
							if((onePack.data[7] != 0x84) && (onePack.data[7] != 0x8D || onePack.data[8] != 0x03))
							{
								vector<int> data;
								for(int k = 0; k < onePack.len; k++)
								{
									data.push_back(onePack.data[k]);
								}

								string josn_str = FormatConvert::instance().transmit_reply_to_string(m_cell_no,deviceNo,"pass_through_data",data);
								MsgFactory::instance()->cell_transmit_pusher(m_cell_no).send(josn_str);
							}
						}
					}
				}
			}

			//0:?????????????????????  //-1: ???????????? (??????????????????????????????)
			if((tmpLen == 0) || ((tmpLen < 0) && (errno != EINTR) && (errno != EWOULDBLOCK) && (errno != EAGAIN) && (errno != 104)))	
			{
				m_epoll_cell.deletefd(fd);
				g_TcpServer.delete_fd_from_map(fd);
				close(fd);

				if(found)
				{
					g_TcpServer.set_client_state(m_cell_no,deviceNo,DisConnect);
				}			
			}			
		}
		else if((ch_Event & EPOLLHUP) || (ch_Event & EPOLLERR))  
		{
			m_epoll_cell.deletefd(fd);
			g_TcpServer.delete_fd_from_map(fd);
			close(fd);
			//printf("epoll delete err fd %d\n",fd);
		}
	}
#else
	vector<recvDataCAN_t> recvPack;

	if(g_can_dev[m_can_idx].RecvDataTransfer(m_cell_no,recvPack))
	{
		for(uint32_t i = 0; i < recvPack.size(); i++)
		{
			recvData_t onePack;

			if(recvPack[i].deviceNo > MAX_CELLS_NBR)
			{
				continue;
			}
			onePack.len = recvPack[i].len;
			memcpy(onePack.data, recvPack[i].data, recvPack[i].len);

			int state = g_TcpServer.get_client_state(m_cell_no,recvPack[i].deviceNo);
			if(state != Connected)
			{
				g_TcpServer.set_client_state(m_cell_no,recvPack[i].deviceNo,NewConnect);
			}
				
			if(!pass_through_enable)
			{
				uint8 ch_no;
				
				for(uint32 k=0; k < MODULE_CH_NUM; k++)
				{
					if(onePack.data[6] & (1 << k))
					{
						ch_no = (recvPack[i].deviceNo-1)*MODULE_CH_NUM + k + 1;
						break;
					}
				}
				
				if(ch_no <= MAX_CHANNELS_NBR && ch_no >= 1)
				{
					m_channels[ch_no-1].recv_data(onePack);
					if(onePack.data[7] != 0x8D || onePack.data[8] != 0x03)
					{
						LOGFMTD("recv<--%d-%2d: %s", m_cell_no, ch_no, hexstring(onePack.data, onePack.len).c_str());
					}	
				}	
			}
			else
			{
				//?????????????????????????????????
				if(onePack.data[7] != 0x8D && onePack.data[8] != 0x03)
				{
					int chIdx = (recvPack[i].deviceNo-1)*MODULE_CH_NUM;
					m_channels[chIdx].reset_heartbeat();
				}
			
				// ??????????????????????????????????????????
				if((onePack.data[7] != 0x84) && (onePack.data[7] != 0x8D || onePack.data[8] != 0x03))
				{
					vector<int> data;
					for(int k = 0; k < onePack.len; k++)
					{
						data.push_back(onePack.data[k]);
					}
		
					string josn_str = FormatConvert::instance().transmit_reply_to_string(m_cell_no,recvPack[i].deviceNo,"pass_through_data",data);
					MsgFactory::instance()->cell_transmit_pusher(m_cell_no).send(josn_str);
				}
			}
		}
	}
#endif
}

void Cell::work_logic_handle()
{
#if defined(AWH_FORMATION_SYS)
	bool is_vacuum_err = false;
#endif
#if defined(ENABLE_RUNSTATUS_SAVE)
	bool status_save_flag = false;
#endif
	
	for (int i = 0; i < MAX_CHANNELS_NBR; ++i)
	{
		bool err_flag = false;
		char ver_data[64];
		
		//????????????????????????:???????????????????????????
		if(m_channels[i].isConnected())
		{
			Step_Process_Addition_Data_t addStatus;
			AuxTempAcq_Record_Data_t tempDate;

			m_channels[i].get_temperature_Data(tempDate);

			addStatus.loop_no = runStatus[i].LoopNo;
			addStatus.sum_step = runStatus[i].StepSum;
			addStatus.batteryTemp = tempDate.batteryTemp;
			memcpy(addStatus.cellTemp,tempDate.cellTemp,CELL_TEMP_ACQ_NUM*sizeof(float));
#if defined(AWH_FORMATION_SYS)			
			addStatus.vaccum = pins_cell_status.vacuum;
#endif
			m_channels[i].add_addition_status(addStatus);
		}

		m_channels[i].handle_reply(err_flag);
		
		if(err_flag)
		{
#if defined(ENABLE_RUNSTATUS_SAVE)
			if(m_channel_start_mask[i])
			{
				status_save_flag = channel_saveStatus_active(i+1,0);
			}
#endif			
			//????????????????????????????????????????????????
			m_channel_start_mask[i] = 0;
			m_channel_wait_mask[i] = 0;
			m_channel_end_mask[i] = 0;
			m_channel_error_mask[i] = 1;
			
#if defined(AWH_FORMATION_SYS)
			is_vacuum_err = m_channels[i].is_error_in_global_alarm(CELL_VACUUM_FLUC_INDEX);	
#endif
		}

		//????????????????????????
		channel_temp_alarm[i].OTP1 = m_channels[i].is_error_in_global_alarm(CHAN_ULTRA_TEMP_INDEX);
		channel_temp_alarm[i].OTP2 = m_channels[i].is_error_in_global_alarm(CHAN_ULTRAHIGH_TEMP_INDEX);

		//???????????????????????????
		if(m_channels[i].return_version_data(ver_data))
		{
			lower_mc_version_t dev_ver;

			dev_ver.device_no = i/MODULE_CH_NUM +1;
			strcpy(dev_ver.device_ver,ver_data);

			device_version.push_back(dev_ver);
		}

		//????????????????????? ????????????????????????????????????  
		if(m_channels[i].get_channel_runstate() == STATE_PAUSE)
		{
			if((m_state == S_START_STEP) || (m_state == S_STEP_EXEC))
			{
				if((m_channels[i].get_channel_suspend_cnt() < 10) && !m_channels[i].get_cmd_pause_flag())
				{
					if(last_work_flag)
					{
						m_channels[i].resume_process();
						last_work_flag = false;
					}
				}
			}
		}

#if defined(ENABLE_RUNSTATUS_SAVE)
		if(m_state == 0)
		{
			if(m_channels[i].get_channel_runstate() == STATE_RUNNING)
			{
				m_state = S_STEP_EXEC;
			}
		}
#endif
	}

#if defined(AWH_FORMATION_SYS)
	if(is_vacuum_err)
	{
		if(Configuration::instance()->is_vacuum_enable())		//????????????
		{
			if((m_state > S_PROCESS_START) && (m_state < S_PROCESS_END))
			{
				for(int i = 0; i < MAX_CHANNELS_NBR; ++i)
				{
					//????????????????????????????????????
					if(m_channel_error_mask[i] == 0)
					{
						m_channels[i].stop_process();
					}
				}
				m_state = S_BREAK_VAC;
			}
		}
	}
#endif	

#if defined(ENABLE_RUNSTATUS_SAVE)
	if(status_save_flag)
	{
		sys_mutex_pend();
		chanRunStatus.endUpdate();
		sys_mutex_post();
	}
#endif

	// ??????????????????????????????
	charge_discharge_check();
}


void Cell::channel_temperature_update()
{
	channel_temperature_t temp1;
	channel_temperature_t temp2;
	channel_temperature_t temp3;
	channel_temperature_t celltemp;
	AuxTempAcq_Record_Data_t m_temp;
	int chanTemp1ErrCnt = 0;
	int chanTemp2ErrCnt = 0;
	int chanTemp3ErrCnt = 0;
	int cellTempErrCnt = 0;

	g_AuxTempAcq.get_channel_temperature(m_cell_no,temp1,temp2,temp3);
	g_AuxTempAcq.get_cell_temperature(m_cell_no,celltemp);
	
	memcpy(m_temp.cellTemp,(void*)&celltemp.chan1,CELL_TEMP_ACQ_NUM*sizeof(float));
	float *p_flt_celltemp = (float*)&celltemp.chan1;
	for(int i = 0; i < CELL_TEMP_ACQ_NUM; i++)
	{
		if(*(p_flt_celltemp+i) < -99.0)
		{
			cellTempErrCnt++;
		}
	}

	float *p_flt_temp1 = (float*)&temp1.chan1;
	float *p_flt_temp2 = (float*)&temp2.chan1;
	float *p_flt_temp3 = (float*)&temp3.chan1;
	for(int i = 0; i < MAX_CHANNELS_NBR; i++)
	{
		if(i < CHAN_TEMP_ACQ_CHANNELS) 			//???12?????????
		{
			m_temp.batteryTemp = *(p_flt_temp1 + i);
			if(m_temp.batteryTemp < -99.0)
			{
				chanTemp1ErrCnt++;
			}
		}
		else if(i < CHAN_TEMP_ACQ_CHANNELS*2)	//??????12?????????
		{
			m_temp.batteryTemp = *(p_flt_temp2 + (i%CHAN_TEMP_ACQ_CHANNELS));
			if(m_temp.batteryTemp < -99.0)
			{
				chanTemp2ErrCnt++;
			}
		}
		else 									//???12??????
		{
			m_temp.batteryTemp = *(p_flt_temp3 + (i%CHAN_TEMP_ACQ_CHANNELS));
			if(m_temp.batteryTemp < -99.0)
			{
				chanTemp3ErrCnt++;
			}
		}
		m_channels[i].update_temperature_Data(m_temp);
	}

	if(Configuration::instance()->is_tempAcq_enable())
	{
		if((m_state == 0) || (m_state == S_PROCESS_IDLE))	//?????????????????????
		{
			//????????????????????? ????????????????????? ???????????????
			if((chanTemp1ErrCnt == CHAN_TEMP_ACQ_CHANNELS) || (chanTemp2ErrCnt == CHAN_TEMP_ACQ_CHANNELS)
				|| (chanTemp3ErrCnt == CHAN_TEMP_ACQ_CHANNELS) || (cellTempErrCnt == CELL_TEMP_ACQ_NUM))
			{
				if((time(NULL) - get_app_start_time()) > 60)		//??????60s????????????????????????
				{
					cell_NG_mask |= (0x01<<TEMPACQ_ERR_BAN);		//????????????
				}
			}
			else
			{
				cell_NG_mask &= ~(0x01<<TEMPACQ_ERR_BAN);
			}
		}
	}
}

void Cell::pins_status_update()
{
	Cmd_Pins_Data_t pin_data;
	uint8 pins_cmd;

	pins_cmd = PINS_STATUS_QUERY;
	send_pins_cmd(pin_data,pins_cmd);
}

#if defined(FORMATION_TECH_AUTO)
// ????????????????????????
void Cell::power_vac_step_ctrl()
{
	for (int ch=0; ch<MAX_CHANNELS_NBR; ch++)
	{
		m_channels[ch].vac_step_process();
	}
}

// ?????????????????????
void Cell::power_step_ctrl(bool start)
{
	if (start)
	{
		// ??????????????????
		for (int ch=0; ch<MAX_CHANNELS_NBR; ch++)
		{
			m_channels[ch].start_process(m_bf_sm.follow_step_no);
			m_channels[ch].reset_step_finish_status();
		}
	}
	else
	{
		// ???????????????
		for (int ch=0; ch<MAX_CHANNELS_NBR; ch++)
		{
			m_channels[ch].stop_process();				
		}
	}
}


// ?????????????????????
void Cell::status_sync_pins(int sub_code, int val)
{
#ifdef ENABLE_PLC
	Cmd_Pins_Msg_t pin_cmd;
	pin_cmd.cell_no 			= m_cell_no;
	pin_cmd.func_code			= SYS_COMMON_CMD;
	pin_cmd.inside_msg.sub_code = sub_code;
	pin_cmd.inside_msg.value    = val;
	MsgFactory::instance()->cell_pusher(m_cell_no).send(pin_cmd, ZMQ_DONTWAIT);
#endif
}

// ??????????????????????????????
void Cell::batteryFormation_status_notify(int status_code)
{
	string josn_str = FormatConvert::instance().notify_to_string(m_cell_no, CMD_POST_STATUS_INFO, 1, status_code);
	MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
}

void Cell::techTest_status_notify(int status_code)
{
	string josn_str = FormatConvert::instance().notify_to_string(m_cell_no, CMD_POST_STATUS_INFO, 2, status_code);
	MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
}

void Cell::batteryFormation_logic()
{
	// ??????????????????
	if (m_bf_sm.start_cmd)
	{
		if (!m_bf_sm.is_started)
		{
			m_bf_sm.logic_no = 1;
		}
	}
	else
	{
		if (!m_bf_sm.is_started)  return;	
	}
	
	// ??????????????????
	if (m_bf_sm.abort_cmd)
	{
		m_bf_sm.abort_cmd = false;
		m_bf_sm.logic_no  = 6;
	}

	// ??????????????????
	switch (m_bf_sm.logic_no)
	{
		// ???????????????
		case 1:
		{	
			batteryFormation_status_notify(-2400);
			status_sync_pins(SYS_SET_VAC_LEAK_TEST, 1);					// ?????????????????????
			status_sync_pins(SYS_AUTO_TECH_START, m_bf_sm.tech_type);	// ????????????

			if (m_bf_sm.tech_type == PINS_TECH_BATTERY)
			{
				m_bf_sm.is_vac_finish  = false;
				m_bf_sm.is_pins_finish = false;		
				m_bf_sm.logic_no = 2;
			}
			else
			{
				m_bf_sm.is_tech_test_finish = false;
				m_bf_sm.logic_no = 8;
			}
			m_bf_sm.is_started = true;
			break;
		}
		// ??????: ????????????????????????
		case 2:
		{
			if (m_bf_sm.is_vac_finish)
			{
				power_step_ctrl(true);
				m_bf_sm.power_start_tm = time(NULL);
				m_bf_sm.logic_no = 3;
				
				batteryFormation_status_notify(-2416);
			}
			else if (m_bf_sm.is_pins_finish)			// ???????????????????????????
			{
				m_bf_sm.logic_no = 0xFF;
			}
			break;
		}
		// ??????: ????????????????????????????????????
		case 3:
		{
			time_t curr_tm = time(NULL);
			if (curr_tm - m_bf_sm.power_start_tm > 5)
			{
				m_bf_sm.logic_no = 4;
			}
			break;
		}
		// ??????: ????????????????????????
		case 4:
		{
			if (!m_bf_sm.is_charging)
			{
				batteryFormation_status_notify(-2417);				
				status_sync_pins(SYS_POW_STEP_FINISH);
				m_bf_sm.logic_no = 5;
			}
			else
			{
				// ??????????????????????????????????????????????????????????????????????????????
				if (m_bf_sm.is_wait_vac)
				{
					if (m_bf_sm.vac_param == 0xFF)
					{
						string josn_str = FormatConvert::instance().alarm_to_string(-2309, m_cell_no);
						MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
						m_bf_sm.logic_no = 9;
					}
					else if (m_bf_sm.vac_param != 0xFE)
					{
						status_sync_pins(SYS_VAC_SET, m_bf_sm.vac_param);
						m_bf_sm.logic_no = 9;
					}
				}
			}
			break;
		}
		// ??????: ????????????????????????
		case 5:
		{
			if (m_bf_sm.is_pins_finish)
			{
				// ???????????????
				batteryFormation_status_notify(-2421);
				m_bf_sm.logic_no = 0xFF;
			}
			break;
		}
		// ??????: ????????????
		case 6:
		{
			if (m_bf_sm.is_charging)
			{
				power_step_ctrl(false);
				batteryFormation_status_notify(-2422);
			}
			string josn_str = FormatConvert::instance().alarm_to_string(-2304, m_cell_no);
			MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
			m_bf_sm.logic_no = 7;
		}
		// ??????: ????????????
		case 7:
		{
			if (!m_bf_sm.is_charging)
			{
				batteryFormation_status_notify(-2422);
				status_sync_pins(SYS_POW_STEP_FINISH);		// ??????????????????????????????
				status_sync_pins(SYS_AUTO_TECH_ABORT);
				m_bf_sm.logic_no = 0xFF;
			}
			break;
		}
		// ??????: ????????????????????????
		case 8:
		{
			if (m_bf_sm.is_tech_test_finish)
			{
				int status_code = 0;
				if (m_bf_sm.tech_type == PINS_TECH_VAC)
				{
					status_code = -2431;
				}
				else if (m_bf_sm.tech_type == PINS_TECH_PT1000)
				{
					status_code = -2433;
				}
				else if (m_bf_sm.tech_type == PINS_TECH_CALIBRATION)
				{
					status_code = -2435;
				}
				techTest_status_notify(status_code);
				m_bf_sm.logic_no = 0xFF;
			}
			break;
		}
		// ??????: ????????????
		case 9:
		{
			if (m_bf_sm.is_vac_next)
			{
				power_vac_step_ctrl();
				m_bf_sm.is_vac_next = false;
				m_bf_sm.power_start_tm = time(NULL);
				m_bf_sm.logic_no = 3;
				LOGFMTD("cell-%d,is_vac_nexted", m_cell_no);
			}
			break;
		}
		
		default:
		{
			m_bf_sm.start_cmd  = false;
			m_bf_sm.is_started = false;
			break;
		}
	}
}
#endif

void Cell::hearbeat_detect()
{
	long tt = millitimestamp();
	if ((tt - m_last_beat_time) < INTERVAL_BEAT_TIME_MS)  return;
	m_last_beat_time = tt;

	// ?????????????????????????????????
	for(int i = 1; i <= (MAX_CHANNELS_NBR+MODULE_CH_NUM-1)/MODULE_CH_NUM; i++)
	{
		try
		{
#ifdef PROTOCOL_5V160A
			int state = g_TcpServer.get_client_state(m_cell_no,i);

#ifndef AWH_FORMATION_SYS						//CAN????????????????????????????????????
			if(state == Connected)
#endif
			{
				vector<int> chanArrary;
#ifndef AWH_FORMATION_SYS
				string ip = g_TcpServer.get_client_ip(m_cell_no,i);
				
				if(g_TcpServer.get_client_channels_by_ip(ip,chanArrary))
#else
				if(g_TcpServer.get_client_channels(m_cell_no,i,chanArrary))
#endif
				{
					int chNo = chanArrary[0];
					
					if (!m_channels[chNo-1].check_beating(true))
					{
						//LOGFMTE("Cell:%d Channel:%d lose beating.", m_cell_no, chNo);
#ifndef AWH_FORMATION_SYS
						int ch_fd = g_TcpServer.get_client_connfd(m_cell_no,i);
						g_TcpServer.delete_fd_from_map(ch_fd);
#endif
#if defined(AWH_FORMATION_SYS)
						if((state != DisConnect) && (state != NoConnect))
#endif							
						{
							m_channels[chNo-1].disconnect();
							g_TcpServer.set_client_state(m_cell_no,i,DisConnect);
						}
					}
				}
			}
#else
		for (int i = 0; i < MAX_CHANNELS_NBR; i++)
		{
			if (!m_channels[i].check_beating(true))
			{
				LOGFMTE("Cell:%d Channel:%d lose beating.", m_cell_no, i+1);
				string josn_str = FormatConvert::instance().alarm_to_string(-603, m_cell_no, i+1);
				MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
				m_channels[i].disconnect();
			}
		}
#endif
		}
		catch (int &e)
		{
			string josn_str = FormatConvert::instance().alarm_to_string(e, m_cell_no, i);
			MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
		}		
	}
}


// ????????????????????????????????????
void Cell::charge_discharge_check()
{
	for (int ch=0; ch<MAX_CHANNELS_NBR; ch++)
	{
		if (m_channels[ch].is_process_end())
		{
			m_channels[ch].reset_step_finish_status();
		
			//????????????????????????
			if(runStatus[ch].Finish == 0)
			{
				runStatus[ch].Finish = 1;
			}
		}
	}
}

uint8_t Cell::fireProtection_check()
{
	temp_alarm_counter_t channel_temp_trig = {0,0};
	temp_alarm_counter_t cell_temp_trig = {0,0};
	uint8_t continue_OTP1_cnt = 0;

	fire_error_code.clear();

	for(int i = 0;i < MAX_CHANNELS_NBR;i++)
	{
		uint8 counts = 0;
		
		if(channel_temp_alarm[i].OTP1)
		{
			channel_temp_trig.OTP1_cnt++;
			if(channel_temp_alarm[i].OTP1_accum <= OTP1_CONTINUE_THRESHOLD)
			{
				channel_temp_alarm[i].OTP1_accum++;
			}
		}
		else
		{
			channel_temp_alarm[i].OTP1_accum = 0;
		}

		if(channel_temp_alarm[i].OTP2)
		{
			channel_temp_trig.OTP2_cnt++;
		}

		counts = m_channels[i].cell_ultra_temp_counter();
		if(counts > cell_temp_trig.OTP1_cnt)
		{
			cell_temp_trig.OTP1_cnt = counts;
		}

		counts = 0;
		counts = m_channels[i].cell_ultrahigh_temp_counter();
		if(counts > cell_temp_trig.OTP2_cnt)
		{
			cell_temp_trig.OTP2_cnt = counts;
		}
	}

	//???????????????
	if(channel_temp_trig.OTP1_cnt >= 1)
	{
		fire_error_code.push_back(-2059);
	}

	if(channel_temp_trig.OTP2_cnt >= 1)
	{
		fire_error_code.push_back(-2060);
	}
	
	if(cell_temp_trig.OTP1_cnt >= 1)
	{
		fire_error_code.push_back(-2065);
	}

	if(cell_temp_trig.OTP2_cnt >= 1)
	{
		fire_error_code.push_back(-2066);
	}

#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
	if(pins_cell_status.alarm_ahead_smoke)
#elif defined(AWH_PRECHARGE_SYS)
	if(pins_cell_status.err_code.Smoke1Alarm)
#endif
	{
		fire_error_code.push_back(-2423);
	}

#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
	if(pins_cell_status.alarm_rear_smoke)
#elif defined(AWH_PRECHARGE_SYS)
	if(pins_cell_status.err_code.Smoke2Alarm)
#endif
	{
		fire_error_code.push_back(-2424);
	}

	//????????????1????????????
	if((channel_temp_trig.OTP1_cnt >= 3) || (channel_temp_trig.OTP2_cnt >= 2))
	{
		return FIRE_PROTECT_LEVEL1;
	}

	if((channel_temp_trig.OTP1_cnt >= 2) && (channel_temp_trig.OTP2_cnt >= 1))
	{
		return FIRE_PROTECT_LEVEL1;
	}

	if((channel_temp_trig.OTP2_cnt >= 1) && (cell_temp_trig.OTP2_cnt >= 1))
	{
		return FIRE_PROTECT_LEVEL1;
	}

	if((cell_temp_trig.OTP1_cnt >= 2) || (cell_temp_trig.OTP2_cnt >= 2))
	{
		return FIRE_PROTECT_LEVEL1;
	}

#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
	if(pins_cell_status.alarm_ahead_smoke && pins_cell_status.alarm_rear_smoke)
#else
	if(pins_cell_status.err_code.Smoke1Alarm && pins_cell_status.err_code.Smoke2Alarm)
#endif
	{
		return FIRE_PROTECT_LEVEL1;
	}

#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
	if(((channel_temp_trig.OTP2_cnt >= 1) || (cell_temp_trig.OTP2_cnt >= 1)) &&
		(pins_cell_status.alarm_ahead_smoke || pins_cell_status.alarm_rear_smoke))
#else
	if(((channel_temp_trig.OTP2_cnt >= 1) || (cell_temp_trig.OTP2_cnt >= 1)) &&
		(pins_cell_status.err_code.Smoke1Alarm || pins_cell_status.err_code.Smoke2Alarm))
#endif
	{
		return FIRE_PROTECT_LEVEL1;
	}

	//????????????2????????????
	for(int i = 0;i < MAX_CHANNELS_NBR;i++)
	{		
		if(channel_temp_alarm[i].OTP1_accum >= OTP1_CONTINUE_THRESHOLD)
		{
			continue_OTP1_cnt++;
			if(continue_OTP1_cnt >= 2)
			{
				return FIRE_PROTECT_LEVEL1; 	// ??????OPT1??????????????????2????????????1
			}
		}
	}
	
	if((channel_temp_trig.OTP1_cnt >= 2) || (channel_temp_trig.OTP2_cnt >= 1))
	{
		return FIRE_PROTECT_LEVEL2;
	}
	
	if(cell_temp_trig.OTP2_cnt >= 1)
	{
		return FIRE_PROTECT_LEVEL2;
	}

#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
	if(((channel_temp_trig.OTP1_cnt >= 1) || (cell_temp_trig.OTP1_cnt >= 1)) &&
		(pins_cell_status.alarm_ahead_smoke || pins_cell_status.alarm_rear_smoke))
#else
	if(((channel_temp_trig.OTP1_cnt >= 1) || (cell_temp_trig.OTP1_cnt >= 1)) &&
		(pins_cell_status.err_code.Smoke1Alarm || pins_cell_status.err_code.Smoke2Alarm))
#endif
	{
		return FIRE_PROTECT_LEVEL2;
	}

	//????????????3????????????	   ???????????????????????????3
	for(int i = 0;i < MAX_CHANNELS_NBR;i++)
	{		
		if(channel_temp_alarm[i].OTP1_accum >= OTP1_CONTINUE_THRESHOLD)
		{
			return FIRE_PROTECT_LEVEL2;		// ??????OPT1??????????????????1????????????2
		}
	}
	
	if((channel_temp_trig.OTP1_cnt >= 1) || (cell_temp_trig.OTP1_cnt >= 1))
	{
		return FIRE_PROTECT_LEVEL3;
	}

#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
	if(pins_cell_status.alarm_ahead_smoke || pins_cell_status.alarm_rear_smoke)
	{
		return FIRE_PROTECT_LEVEL3;
	}
#elif defined(AWH_PRECHARGE_SYS)
	if(pins_cell_status.err_code.Smoke1Alarm || pins_cell_status.err_code.Smoke2Alarm)
	{
		return FIRE_PROTECT_LEVEL3;
	}
#endif

	return 0;
}

void Cell::fireProtection_execute(uint8_t level)
{
	switch(level)
	{
		case FIRE_PROTECT_LEVEL1:
		{
			if(last_fire_level != level)
			{
				Cmd_Pins_Data_t pin_data;
				uint8 pins_cmd;

				pins_cmd = FIRE_EMERGENCY_BRAKE;
				send_pins_cmd(pin_data,pins_cmd);
				m_state = S_PROCESS_IDLE;
			}

			exec_process_status_notify(FIRE_PROTECT_LEVEL1);
			break;
		}

		case FIRE_PROTECT_LEVEL2:
		{
			if((last_fire_level != level) && (last_fire_level != FIRE_PROTECT_LEVEL1))
			{
				//?????????????????????
				for(int i=0; i<MAX_CHANNELS_NBR; i++)
				{
					if(m_channel_error_mask[i] == 0)
					{
						m_channels[i].stop_process();
					}
				}

				//???????????????
			#if defined(AWH_FORMATION_SYS)
				if(Configuration::instance()->is_vacuum_enable())		//????????????
				{
					m_state = S_BREAK_VAC;
					LOGFMTD("fireAlarm Level2 cell%d vacuum release",m_cell_no); 
				}
				else
			#endif
				{
					m_state = S_PROCESS_IDLE;
				}
			}
			
			exec_process_status_notify(FIRE_PROTECT_LEVEL2);
			break;
		}
		
		case FIRE_PROTECT_LEVEL3:
			exec_process_status_notify(FIRE_PROTECT_LEVEL3);
			break;

		default:
			break;
	}

	last_fire_level = level;
}

void Cell::fireProtection_handle()
{
	uint8_t fire_level = 0;

	fire_level = fireProtection_check();
	fireProtection_execute(fire_level);
}

#if 0
// ???????????????????????????????????????????????????
void Cell::step_vac_check()
{
	bool is_all_vac = true;
	int  vac_index = 0;
	int  vac_cnt = 0;
	int8_t vac_param[MAX_CHANNELS_NBR];
	memset(vac_param, 0xFE, MAX_CHANNELS_NBR);
	
	for (int ch=0; ch<MAX_CHANNELS_NBR; ch++)
	{
		if (!m_channels[ch].is_vac_step())
		{
			if (m_channels[ch].isConnected() && m_channels[ch].is_running())
			{
				is_all_vac = false;
				break;
			}
		}
		else
		{
			vac_param[vac_index++] = m_channels[ch].vac_Param();
		}
	}
	
	if (is_all_vac)
	{
		if (vac_index > 1)
		{
			for (int i=1; i<vac_index; i++)
			{	
				if (vac_param[0] != vac_param[i])
				{
					vac_cnt++;
				}
			}
			if (vac_cnt != 0)
			{
				m_bf_sm.vac_param = 0xff;
			}
			else
			{
				m_bf_sm.vac_param = vac_param[0];
			}
		}
		else
		{
			m_bf_sm.vac_param = vac_param[0];
		}
	}
	
	m_bf_sm.is_wait_vac = is_all_vac;
}

// ??????????????????????????????????????????????????????
void Cell::channel_vaccumVal_update(float val)
{
	for (int ch=0; ch<MAX_CHANNELS_NBR; ch++)
	{
		m_channels[ch].set_vac_val(val);
	}
}

//???????????????????????????
void Cell::channel_celltemper_update(float *temper)
{
	for (int ch=0; ch<MAX_CHANNELS_NBR; ch++)
	{
		m_channels[ch].set_celltemp_value(temper);
	}
}
#endif

#ifdef ZHONGHANG_SYSTEM_PROTECTION
// ??????????????????
void Cell::fireAlarmInfo_clear()
{
	memset((void *)&m_fireAlarm, 0, sizeof(m_fireAlarm));
}

// ?????????????????????
void Cell::power_step_puase()
{
	for (int ch=0; ch<MAX_CHANNELS_NBR; ch++)
	{
		m_channels[ch].pause_process();
	}
}

// ????????????????????????
void Cell::alarmInfo_post(int err_code, int ch_no)
{
	string josn_str = FormatConvert::instance().alarm_to_string(err_code, m_cell_no, ch_no);
	MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
}

// ???????????????????????????
void Cell::actionCmd_to_pins(int sub_code, int val)
{
#ifdef ENABLE_PLC
	Cmd_Pins_Msg_t pin_cmd;
	pin_cmd.cell_no 			= m_cell_no;
	pin_cmd.func_code			= SYS_DO_PROTECT;
	pin_cmd.inside_msg.sub_code = sub_code;
	pin_cmd.inside_msg.value    = val;
	MsgFactory::instance()->cell_pusher(m_cell_no).send(pin_cmd, ZMQ_DONTWAIT);
#endif
}

// ??????????????????
void Cell::protectionAction_execute()
{
	for (int i=0; i<FIRE_ALARM_CLASS_NUM; ++i)
	{
		if (m_fireAlarm[i].is_trigger_flag)
		{
			if (!m_fireAlarm[i].is_executed_flag)
			{
				m_fireAlarm[i].is_executed_flag = true;
				alarmInfo_post(-2510 - i);
				
				/* ????????????2: ??????+???????????? */
				if (i == 2)
				{
					power_step_puase();
				}
				/* ????????????3~5: ??????+???????????? */
				else if (i > 2)
				{
					// ????????????????????????
					power_step_ctrl(false);
					// A???????????????
					actionCmd_to_pins(CELL_OPEN_FIRE_FAN);//fire_protect_A
				}
				/* ????????????5: +?????????????????? */
				if (i == 5)
				{
					// B???????????????
					actionCmd_to_pins(CELL_OPEN_FIRE_SPRAY);//fire_protect_B
				}
			}
		}
	}
}

// ????????????????????????
void Cell::fireProtection_handler()
{
	/* ????????????0 */
	for (int ch=0; ch<MAX_CHANNELS_NBR; ch++)
	{
		/* ???????????? */
		if ((m_fireAlarm[0].chs_temp_warning & ((uint64_t)1 << ch)) != 0)
		{
			// ?????????????????????
			m_channels[ch].stop_process();
			m_fireAlarm[0].is_trigger_flag = true;
		}
	}
	m_fireAlarm[0].chs_temp_warning = 0;	
	
	/* ????????????1 */
	// ??????????????????????????????????????????
	if (m_fireAlarm[1].is_smoke_warning)
	{
		m_fireAlarm[1].is_trigger_flag = true;
	}	

	/* ????????????2 */
	int tempWarning_cnt = 0;
	
	m_fireAlarm[2].is_trigger_flag = false;
	for (int ch=0; ch<MAX_CHANNELS_NBR; ch++)
	{
		// ????????????
		if ((m_fireAlarm[2].chs_temp_warning & ((uint64_t)1 << ch)) != 0)
		{
			tempWarning_cnt++;
		}
		// ????????????
		if ((m_fireAlarm[2].chs_temp_alarm & ((uint64_t)1 << ch)) != 0)
		{
			m_fireAlarm[2].is_trigger_flag = true;
			break;
		}
	}
	// ???????????????
	if (tempWarning_cnt > 1)
	{
		m_fireAlarm[2].is_trigger_flag = true;
	}
	
	/* ????????????3 */
	m_fireAlarm[3].is_trigger_flag = false;
	// ????????????????????????
	for (int ch=0; ch<MAX_CHANNELS_NBR; ch++)
	{
		if ((m_fireAlarm[3].chs_temp_warning & ((uint64_t)1 << ch)) != 0)
		{
			m_fireAlarm[3].is_batTemp_warning = true;
		}
		if ((m_fireAlarm[3].chs_temp_alarm & ((uint64_t)1 << ch)) != 0)
		{
			m_fireAlarm[3].is_batTemp_alarm = true;
		}
		if ((m_fireAlarm[3].chs_temp_emergency & ((uint64_t)1 << ch)) != 0)
		{
			m_fireAlarm[3].is_batTemp_emergency = true;
		}
	}

	// ???????????????/??????/???????????????
	if (m_fireAlarm[3].is_batTemp_emergency || m_fireAlarm[3].is_smoke_alarm || m_fireAlarm[3].is_cellTemp_emergency)
	{
		m_fireAlarm[3].is_trigger_flag = true;
	}
	// ?????? + ????????????/??????
	if (m_fireAlarm[3].is_smoke_warning && (m_fireAlarm[3].is_batTemp_warning || m_fireAlarm[3].is_batTemp_alarm))
	{
		m_fireAlarm[3].is_trigger_flag = true;
	}
	// ???????????? + ????????????
	if (m_fireAlarm[3].is_batTemp_warning && m_fireAlarm[3].is_batTemp_alarm)
	{
		m_fireAlarm[3].is_trigger_flag = true;
	}
	// ??????????????? + ????????????
	if (m_fireAlarm[3].is_cellTemp_emergency && m_fireAlarm[3].is_batTemp_warning)
	{
		m_fireAlarm[3].is_trigger_flag = true;
	}

	/* ????????????4 */
	m_fireAlarm[4].is_trigger_flag = false;
	// ????????????????????????
	for (int ch=0; ch<MAX_CHANNELS_NBR; ch++)
	{
		if ((m_fireAlarm[4].chs_temp_warning & ((uint64_t)1 << ch)) != 0)
		{
			m_fireAlarm[4].is_batTemp_warning = true;
		}
		if ((m_fireAlarm[4].chs_temp_alarm & ((uint64_t)1 << ch)) != 0)
		{
			m_fireAlarm[4].is_batTemp_alarm = true;
		}
		if ((m_fireAlarm[4].chs_temp_emergency & ((uint64_t)1 << ch)) != 0)
		{
			m_fireAlarm[4].is_batTemp_emergency = true;
		}
	}

	// ?????? +	???????????????
	if (m_fireAlarm[4].is_smoke_warning && m_fireAlarm[4].is_batTemp_emergency)
	{
		m_fireAlarm[4].is_trigger_flag = true;
	}
	// ?????? +	????????????
	if (m_fireAlarm[4].is_smoke_alarm && m_fireAlarm[4].is_batTemp_warning)
	{
		m_fireAlarm[4].is_trigger_flag = true;
	}
	// ????????????/??????	+ ???????????????
	if ((m_fireAlarm[4].is_batTemp_warning || m_fireAlarm[4].is_batTemp_alarm) && m_fireAlarm[4].is_batTemp_emergency)
	{
		m_fireAlarm[4].is_trigger_flag = true;
	}
	// ??????????????? +	??????/????????????
	if (m_fireAlarm[4].is_cellTemp_emergency && (m_fireAlarm[4].is_smoke_warning || m_fireAlarm[4].is_batTemp_alarm))
	{
		m_fireAlarm[4].is_trigger_flag = true;
	}

	/* ????????????5 */
	m_fireAlarm[5].is_trigger_flag = false;
	// ????????????????????????
	for (int ch=0; ch<MAX_CHANNELS_NBR; ch++)
	{
		if ((m_fireAlarm[5].chs_temp_warning & ((uint64_t)1 << ch)) != 0)
		{
			m_fireAlarm[5].is_batTemp_warning = true;
		}
		if ((m_fireAlarm[5].chs_temp_alarm & ((uint64_t)1 << ch)) != 0)
		{
			m_fireAlarm[5].is_batTemp_alarm = true;
		}
		if ((m_fireAlarm[5].chs_temp_emergency & ((uint64_t)1 << ch)) != 0)
		{
			m_fireAlarm[5].is_batTemp_emergency = true;
		}
	}

	// ?????? +	????????????/???????????????
	if (m_fireAlarm[5].is_smoke_alarm && (m_fireAlarm[5].is_batTemp_alarm || m_fireAlarm[5].is_batTemp_emergency))
	{
		m_fireAlarm[5].is_trigger_flag = true;
	}
	// ??????????????? +	??????/???????????????
	if (m_fireAlarm[5].is_cellTemp_emergency && (m_fireAlarm[5].is_smoke_alarm || m_fireAlarm[5].is_batTemp_emergency))
	{
		m_fireAlarm[5].is_trigger_flag = true;
	}

	// ??????????????????
	protectionAction_execute();
}

void Cell::alarmProtection_handle(int ch, int code)
{
	switch (code)
	{
#ifdef PLC_JS_CT
		// ??????????????????AC?????????
		case -1040:
		case -1044:
		{
			power_step_ctrl(false);
			m_bf_sm.abort_cmd = true;
			break;
		}
		// ???????????????
		case -1041:
		{
			for (int i = 0; i < MAX_CHANNELS_NBR; i++)
			{
	#ifdef PROTOCOL_5V160A
				if (m_channels[i / MODULE_CH_NUM].isConnected())
				{
					m_channels[i / MODULE_CH_NUM].disconnect();
				}
	#else
				if (m_channels[i].isConnected())
				{			
					m_channels[i].disconnect();
				}
	#endif
				m_bf_sm.abort_cmd = true;
			}
			break;
		}
#endif		
		/* ??????????????? */
		// ????????????????????????????????????
		case -1501:
		{
			for (int i=0; i<FIRE_ALARM_CLASS_NUM; i++)
			{
				m_fireAlarm[i].is_cellTemp_emergency = true;
			}
			break;
		}
		// ??????????????????
		case -1540:
		case -1541:
		{
			for (int i=0; i<FIRE_ALARM_CLASS_NUM; i++)
			{
				m_fireAlarm[i].is_smoke_alarm = true;
			}
			break;
		}
		// ??????????????????
		case -1566:
		case -1567:
		{
			for (int i=0; i<FIRE_ALARM_CLASS_NUM; i++)
			{
				m_fireAlarm[i].is_smoke_warning = true;
			}
			break;
		}
		// ??????????????????
		case -1600:
//		// ??????????????????
//		case -2310:
		{
			power_step_ctrl(false);
			m_bf_sm.abort_cmd = true;
			break;
		}

		// ?????????????????? (???????????????)
		case -2007:
		{
			for (int i=0; i<FIRE_ALARM_CLASS_NUM; i++)
			{
				m_fireAlarm[i].chs_temp_alarm   |= (uint64_t)1 << ch;
			}
			break;
		}
		// ?????????????????? (???????????????)
		case -2043:
		{
			for (int i=0; i<FIRE_ALARM_CLASS_NUM; i++)
			{
				m_fireAlarm[i].chs_temp_warning |= (uint64_t)1 << ch;
			}
			break;
		}
		// ????????????????????????
		case -2044:
		{
			for (int i=0; i<FIRE_ALARM_CLASS_NUM; i++)
			{
				m_fireAlarm[i].chs_temp_emergency |= (uint64_t)1 << ch;
			}
			break;
		}
	}
	
	LOGFMTW("Zhonghang system protection code: %d", code);
}
#endif





