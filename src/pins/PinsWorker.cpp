#include <thread>
#include <sstream>
#include <string>
#include <string.h>
#include <math.h>

#include "log4z.h"
#include "Type.h"
#include "MsgFactory.h"
#include "PinsWorker.h"
#include "S7Client.h"
#include "Configuration.h"
#include "FormatConvert.h"
#include "ReplyHandler.h"
#include "AuxTempAcq.h"
#include "SLMPDriver.h"

#define INTERVAL_ENV_SAMPLE_TIME 		1       					// unit:second.
#define PLC_HEARTBEAT_TIMEOUT_CNT		1       					// 心跳超时次数

#if defined(AWH_PRECHARGE_SYS)
#define INTERVAL_VAC_SAMPLE_TIME 		3             				// unit:second.
#endif

using namespace std;

PinsWorker::PinsWorker()
{
#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
	is_sudden_stop = false;
	
	memset(m_vac_para, 0, PINS_CELLS_NUM*sizeof(plc_vac_para_setting_t));
	memset(m_format_vac, 0, PINS_CELLS_NUM*sizeof(plc_step_vac_t));
	memset(statewait_timeout, 0,PINS_CELLS_NUM*sizeof(uint16_t));
	memset(plc_alarm_mask, 0, PINS_CELLS_NUM*sizeof(uint64_t));
	
	memset(m_block_test_state, 0, sizeof(m_block_test_state));
	memset(m_leak_test_state, 0, sizeof(m_leak_test_state));
	memset(m_vac_state, 0, sizeof(m_vac_state));
	memset(ack_delay_time, 0, sizeof(ack_delay_time));
#if defined(AWH_FORMATION_SYS)
	memset(is_vacuum_error,0,sizeof(is_vacuum_error));
#endif
#else
	memset(is_sudden_stop, 0, sizeof(is_sudden_stop));
#endif
	memset(m_pins_status, 0, PINS_CELLS_NUM*sizeof(Pins_Status_Data_t));
}

#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
void PinsWorker::send_pins_alarm(int err_code,int cell_no)
{
	string josn_str = FormatConvert::instance().pins_alarm_to_string(err_code,cell_no);
	MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
}

void PinsWorker::pre_formation_block_test()
{			
	for(int i = 0; i < PINS_CELLS_NUM; i++)
	{
		switch(m_block_test_state[i])
		{
			case E_WAIT:
				if(m_vac_para[i].blockPumpTime-- <= 0)
				{
					m_pins_work.vac_block_test(i+1,false);		//堵塞测试时间结束
					m_block_test_state[i] = E_JUDGE;
					statewait_timeout[i] = 0;
				}
				m_pins_status[i].blockTestResult = 3;
				//LOGFMTD("m_block_test_state blockPumpTime=%d ",m_vac_para[i].blockPumpTime);
				break;

			case E_JUDGE:
			{
				if(statewait_timeout[i]++ > 2)
				{
					uint16 result = 0;
					if(fabs(m_pins_work.return_vacuum_value(i+1)) < fabs(m_vac_para[i].blockSetVal))
					{
						result = 1;
					}
					else
					{
						result = 2;
					}
					
					m_pins_status[i].blockTestResult = result;
					status_sync_cell(i+1,SYS_BLOCK_TEST_ACK,result);
					m_block_test_state[i] = E_END;
					LOGFMT_DEBUG(cell_id_temp,"库位%d, 堵塞测试结果:%d",i+1,result);
				}
				break;
			}

			default:
				break;
		}
	}
}

void PinsWorker::pre_formation_leak_test()
{			
	for(int i = 0; i < PINS_CELLS_NUM; i++)
	{
		switch(m_leak_test_state[i])
		{
			case E_EXTEND:
			{
				if(m_pins_status[i].pins_extend && m_pins_status[i].tray_in_place)
				{
					m_vac_para[i].KeepVacStableTime += m_vac_para[i].KeepVacTime;
					m_pins_work.leak_ratio_test(i+1,true);		//开始保压测试
					
					m_leak_test_state[i] = E_WAIT;
					m_pins_status[i].keepVacTestResult = 3;
					statewait_timeout[i] = 0;
					LOGFMT_DEBUG(cell_id_temp,"库位%d 下发保压测试开始",i+1);
				}
				else
				{
					if(++statewait_timeout[i] > 20)
					{
						m_leak_test_state[i] = E_END;
						send_pins_alarm(-2400-TRAY_PRESS_TIMEOUT,i+1);
						LOGFMT_DEBUG(cell_id_temp,"库位%d, 保压测试托盘压合超时",i+1);
					}
				}
				break;
			}
				
			case E_WAIT:
				if(m_vac_para[i].KeepVacStableTime > 0)
				{
					m_vac_para[i].KeepVacStableTime--;
					if(m_vac_para[i].KeepVacStableTime <= 0)
					{
						if(m_vac_para[i].KeepVacTestTimeout == 0)
						{
							m_vac_para[i].KeepVacTestTimeout = 1200;		//20mins
						}
						else if(m_vac_para[i].KeepVacTestTimeout < 120)
						{
							m_vac_para[i].KeepVacTestTimeout = 120;			//最小2mins
						}
						
						m_leak_test_state[i] = E_JUDGE;
					}
				}
				break;

			case E_JUDGE:
			{
				uint16 result = 0;
				bool leak_test_ack = false;
				float keepVac_start_val = 0.0;
				float keepVac_end_val = 0.0;
				
				m_pins_work.get_leak_ratio_test_ack(i+1,leak_test_ack);
				if(leak_test_ack)
				{
					m_pins_work.get_cell_keep_vacuum_test_result(i+1,result);
					m_pins_work.get_cell_keep_vacuum_start_value(i+1,keepVac_start_val);
					m_pins_work.get_cell_keep_vacuum_end_value(i+1,keepVac_end_val);
					
					m_pins_status[i].keepVacTestResult = result;
					m_pins_status[i].keepVac_start = keepVac_start_val;
					m_pins_status[i].keepVac_end = keepVac_end_val;

					float delta_vac = fabs(keepVac_start_val - keepVac_end_val);
					m_pins_status[i].leakRatioResult = (delta_vac * 60.0)/(float)m_vac_para[i].KeepVacTime;
				
					m_pins_work.leak_ratio_test(i+1,false);
					status_sync_cell(i+1,SYS_LEAK_TEST_ACK,result);
					m_leak_test_state[i] = E_END;

					LOGFMT_DEBUG(cell_id_temp,"库位%d, 保压测试结果:%d",i+1,result);
				}
				else
				{
					if(++statewait_timeout[i] > m_vac_para[i].KeepVacTestTimeout)		//抽泄负压最小速率 10Kpa/min  -80即为8min
					{
						m_pins_status[i].keepVacTestResult = 2;
						m_pins_status[i].keepVac_start = 0.0;
						m_pins_status[i].keepVac_end = 0.0;
						m_pins_status[i].leakRatioResult = 0.0;
						m_leak_test_state[i] = E_ERROR;

						LOGFMT_DEBUG(cell_id_temp,"库位%d 等待保压测试结果超时",i+1);					
					}
				}

				break;
			}

			case E_ERROR:
				m_pins_work.leak_ratio_test(i+1,false);
				status_sync_cell(i+1,SYS_LEAK_TEST_ACK,0);
				//m_leak_test_state[i] = E_END;
				m_leak_test_state[i] = E_VAC_BREAK;
				break;

			case E_VAC_BREAK:
			{
				m_pins_work.step_para_write_in(i+1,false);

				m_format_vac[i].leak_vac_value = -1;
				m_format_vac[i].leak_vac_timeout= 300;
				m_pins_work.set_cell_high_vacuum_value(i+1,m_format_vac[i].pump_vac_value);
				m_pins_work.set_cell_pump_vacuum_timeout(i+1,m_format_vac[i].pump_vac_timeout);
				m_pins_work.set_cell_leak_vacuum_value(i+1,m_format_vac[i].leak_vac_value);
				m_pins_work.set_cell_leak_vacuum_timeout(i+1,m_format_vac[i].leak_vac_timeout);
				
				m_leak_test_state[i] = E_BREAK_WAIT;
				m_pins_work.step_para_write_in(i+1,true);
				
				m_pins_work.break_vacuum_control(i+1,ENABLE_C);
				statewait_timeout[i] = 0;
				break;
			}

			case E_BREAK_WAIT:
			{
				bool leak_vac_ack = false;
				
				m_pins_work.get_leak_vacuum_ack(i+1,leak_vac_ack);
				if(leak_vac_ack)
				{
					//m_leak_test_state[i] = E_TRAY_RESET;
					m_leak_test_state[i] = E_END;
				}
				else
				{
					if(statewait_timeout[i]++ > (m_format_vac[i].leak_vac_timeout+1))
					{
						m_leak_test_state[i] = E_END;
						if(fabs(m_pins_status[i].vacuum - m_format_vac[i].leak_vac_value) > 6.0)
						{
							send_pins_alarm(-2400-RELEASE_VAC_TIMEOUT,i+1);
						}
					}
				}

				break;
			}
#if 0
			case E_TRAY_RESET:
			{
				if(m_pins_status[i].vacuum > -5.0)
				{
					m_pins_work.set_tray_action(i+1,DOWN_T);
					m_leak_test_state[i] = E_END;
				}
				
				break;
			}
#endif
			default:
				break;
		}
	}
}

void PinsWorker::formation_vac_task()
{
	for(int i = 0; i < PINS_CELLS_NUM; i++)
	{
		switch(m_vac_state[i])
		{
			case E_VAC_WAIT:
			{
				bool para_write_ack = false;
				
				m_pins_work.get_step_para_write_ack(i+1,para_write_ack);
				if(para_write_ack)
				{
					m_vac_state[i] = E_VAC_JUDGE;
					statewait_timeout[i] = 0;
				}
				else
				{
					if(statewait_timeout[i]++ > 20)
					{
						m_vac_state[i] = E_STATE_ERR;
					}
				}
				
				break;
			}
				
			case E_LEAK_WAIT:
				m_vac_state[i] = E_LEAK_JUDGE;
				statewait_timeout[i] = 0;
				ack_delay_time[i] = 0;
				
				break;

			case E_VAC_JUDGE:
			{
				bool pump_vac_ack = false;
				
				m_pins_work.get_pump_vacuum_ack(i+1,pump_vac_ack);
				if(pump_vac_ack)
				{
					status_sync_cell(i+1,SYS_PUMP_VAC_ACK,1);
					m_vac_state[i] = E_STATE_END;
				}
				else
				{
					if(statewait_timeout[i]++ > (m_format_vac[i].pump_vac_timeout+3))
					{
						if(fabs(m_pins_status[i].vacuum - m_format_vac[i].pump_vac_value) > 8.0)
						{
							status_sync_cell(i+1,SYS_PUMP_VAC_ACK,0);
							m_vac_state[i] = E_VAC_RELEASE;
							send_pins_alarm(-2400-PUMPING_VAC_TIMEOUT,i+1);
							//抽真空超时 库位直接NG
							status_sync_cell(i+1,SYS_CELL_ALARM_HANDLE,-2400-PUMPING_VAC_TIMEOUT);
							//同步针床告警掩码
							plc_alarm_mask[i] |= (uint64_t)1 << (PUMPING_VAC_TIMEOUT-1);
						}
						else
						{
							status_sync_cell(i+1,SYS_PUMP_VAC_ACK,1);
							m_vac_state[i] = E_STATE_END;
						}
					}
				}

				break;
			}
			
			case E_LEAK_JUDGE:
			{
				bool leak_vac_ack = false;
				
				m_pins_work.get_leak_vacuum_ack(i+1,leak_vac_ack);
				if(leak_vac_ack)
				{
					if(++ack_delay_time[i] > 4)
					{
						status_sync_cell(i+1,SYS_LEAK_VAC_ACK,1);
						m_vac_state[i] = E_STATE_END;
					}
				}
				else
				{
					statewait_timeout[i]++;
					
					if(statewait_timeout[i] > (m_format_vac[i].leak_vac_timeout+5))
					{
						if(fabs(m_pins_status[i].vacuum - m_format_vac[i].leak_vac_value) > 9.0)
						{
							status_sync_cell(i+1,SYS_LEAK_VAC_ACK,0);
							m_vac_state[i] = E_STATE_END;
							send_pins_alarm(-2400-RELEASE_VAC_TIMEOUT,i+1);
							//泄真空超时 库位直接NG
							status_sync_cell(i+1,SYS_CELL_ALARM_HANDLE,-2400-RELEASE_VAC_TIMEOUT);
						}
						else
						{
							status_sync_cell(i+1,SYS_LEAK_VAC_ACK,1);
							m_vac_state[i] = E_STATE_END;
						}
					}
					else if(statewait_timeout[i] == (m_format_vac[i].leak_vac_timeout/2))
					{
						//泄真空逾时一半时判断是否需要再发一次泄压
						if(fabs(m_pins_status[i].vacuum - m_format_vac[i].leak_vac_value) > 5.0)
						{
							m_pins_work.step_para_write_in(i+1,false);
							m_pins_work.start_vacuum_control(i+1,DISABLE_C);
							
							m_pins_work.set_cell_high_vacuum_value(i+1,m_format_vac[i].pump_vac_value);
							m_pins_work.set_cell_pump_vacuum_timeout(i+1,m_format_vac[i].pump_vac_timeout);
							m_pins_work.set_cell_leak_vacuum_value(i+1,m_format_vac[i].leak_vac_value);
							m_pins_work.set_cell_leak_vacuum_timeout(i+1,m_format_vac[i].leak_vac_timeout);
							
							m_pins_work.step_para_write_in(i+1,true);
							m_pins_work.break_vacuum_control(i+1,ENABLE_C);
						}
						else
						{
							status_sync_cell(i+1,SYS_LEAK_VAC_ACK,1);
							m_vac_state[i] = E_STATE_END;
						}
					}
				}

				break;
			}

			case E_VAC_RELEASE:
			{
#if defined(AWH_FORMATION_SYS) 				
				if(Configuration::instance()->is_vacuum_enable())	//负压系统
				{
					m_pins_work.step_para_write_in(i+1,false);
					
					m_format_vac[i].leak_vac_value = -1;
					m_format_vac[i].leak_vac_timeout= 300;
					m_pins_work.set_cell_high_vacuum_value(i+1,m_format_vac[i].pump_vac_value);
					m_pins_work.set_cell_pump_vacuum_timeout(i+1,m_format_vac[i].pump_vac_timeout);
					m_pins_work.set_cell_leak_vacuum_value(i+1,m_format_vac[i].leak_vac_value);
					m_pins_work.set_cell_leak_vacuum_timeout(i+1,m_format_vac[i].leak_vac_timeout);
					
					m_vac_state[i] = E_RELEASE_WAIT;
					m_pins_work.step_para_write_in(i+1,true);	
					
					m_pins_work.break_vacuum_control(i+1,ENABLE_C);
					statewait_timeout[i] = 0;
				}
				else			//无负压直接弹针床
#endif
				{
					m_vac_state[i] = E_STATE_END;
					//消防保护
					if(fire_emergency_flag[i])
					{
						if(m_pins_status[i].pins_extend)		//针床压合状态才弹针床
						{
							status_sync_cell(i+1,SYS_TRAY_STATE_SET,TRAY_PROCESSING_CMD);
							msleep(100);
						}

						LOGFMTD("fireAlarm Level1 cell%d tray down",i+1);
						m_pins_work.set_tray_action(i+1,DOWN_T);
						m_pins_work.vent_fan_control(i+1,DISABLE_C);

						fire_emergency_flag[i] = 0;
					}
				}

				break;
			}

			case E_RELEASE_WAIT:
			{
				bool leak_vac_ack = false;
								
				m_pins_work.get_leak_vacuum_ack(i+1,leak_vac_ack);
				if(leak_vac_ack)
				{
					m_vac_state[i] = E_STATE_END;

					//消防保护
					if(fire_emergency_flag[i])
					{
						if(m_pins_status[i].vacuum > -7.0)
						{
							if(m_pins_status[i].pins_extend)		//针床压合状态才弹针床
							{
								status_sync_cell(i+1,SYS_TRAY_STATE_SET,TRAY_PROCESSING_CMD);
								msleep(100);
							}
							
							LOGFMTD("fireAlarm Level1 cell%d tray down",i+1); 
							m_pins_work.set_tray_action(i+1,DOWN_T);
							m_pins_work.vent_fan_control(i+1,DISABLE_C);
						}

						fire_emergency_flag[i] = 0;
					}
				}
				else
				{
					if(statewait_timeout[i]++ > (m_format_vac[i].leak_vac_timeout+1))
					{
						m_vac_state[i] = E_STATE_END;
						if(fabs(m_pins_status[i].vacuum - m_format_vac[i].leak_vac_value) > 6.0)
						{
							send_pins_alarm(-2400-RELEASE_VAC_TIMEOUT,i+1);
						}
					}
				}

				break;
			}
			
			case E_STATE_ERR:
				status_sync_cell(i+1,SYS_PUMP_VAC_ACK,0);
				m_vac_state[i] = E_STATE_END;
				break;
				
			default:
				break;
		}
	}
}
#endif

thread PinsWorker::run()
{
	return thread(&PinsWorker::work, this);
}

void PinsWorker::work()
{	
    long curr_tm;
	bool timer_click_flag = false;
#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
	uint8 timeoutCnt = 0;
	uint8 heartbeat_signal = 0;
#endif

#if defined(AWH_PRECHARGE_SYS)
	bool connect_flag[PINS_CELLS_NUM]={false,false,false};
#endif

	// 创建ZMQ消息连接
	try
	{
		MsgFactory::instance()->create_pins_alarm_pusher(Configuration::instance()->ext_alarm_socket());
		MsgFactory::instance()->create_pins_puller(Configuration::instance()->int_pins_puller_socket());
		MsgFactory::instance()->create_pins_to_ext_reply_pusher(Configuration::instance()->ext_reply_socket());
		MsgFactory::instance()->create_pins_puber(Configuration::instance()->int_pins_puber_socket());
	}
	catch (zmq::error_t &e)
	{
		LOGFMTE("Create ZMQ communication error. code: %d, description:%s", e.num(), e.what());
		string josn_str = FormatConvert::instance().alarm_to_string(-209);
		MsgFactory::instance()->pins_alarm_pusher().send(josn_str);

		exit(-209);
	}

#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
	m_pins_work.init_plc_alarm_erc();
#endif

#if defined(AWH_PRECHARGE_SYS)
	for (int i = 0; i < PINS_CELLS_NUM; i++)
	{
		string plc_addr = "";
		unsigned short port = 0;
	
		port = Configuration::instance()->slmp_plc_socket(i+1,plc_addr);

		if((plc_addr != "") && (port != 0))
		{
			if(m_pins_precharge[i].do_plc_connect(plc_addr.c_str(),port))
			{
				connect_flag[i]=true;
				LOGFMTD("PinsWorker Connect PLC %s",plc_addr.c_str());
			}
		}
	}
#endif

	// 针床线程业务处理
	for (;;)
	{
		msleep(100);

		curr_tm = millitimestamp();
		if (curr_tm - m_last_env_time > 1000)
		{
			m_last_env_time = curr_tm;
			timer_click_flag = true;
		}

#if defined(AWH_PRECHARGE_SYS)
		if(Configuration::instance()->is_plc_enable())		//使能PLC
		{
			for (int i = 0; i < PINS_CELLS_NUM; i++)
			{
				if(connect_flag[i])
				{
					if(!m_pins_precharge[i].is_connected())    //PLC未连接
					{
						try
						{
							string plc_addr = "";
							unsigned short port = 0;
							
							port = Configuration::instance()->slmp_plc_socket(i+1,plc_addr);
							if(m_pins_precharge[i].do_plc_connect(plc_addr.c_str(),port))
							{
								LOGFMTD("PinsWorker Connect PLC %s",plc_addr.c_str());
							}
						}
						catch (int &e)
						{
							string josn_str = FormatConvert::instance().alarm_to_string(-2410,i+1);
							MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
						}
					}
					else 
					{
						if(timer_click_flag)
						{
							if(!m_pins_precharge[i].get_heartbeat())		//设置PLC心跳
							{
								m_pins_precharge[i].close_connect();
								string josn_str = FormatConvert::instance().alarm_to_string(-2410,i+1);
								MsgFactory::instance()->pins_alarm_pusher().send(josn_str); 
							}
						}
					}
				}
			}
		 }

		//处理上位机指令
		message_handle();

		// 状态监视和保护
		if(timer_click_flag)
		{
			timer_click_flag = false;
			status_update_handle();
		}
#else
		if(Configuration::instance()->is_plc_enable())		//使能PLC
		{
			if(!m_pins_work.is_connected())				//PLC未连接
			{
				try
				{
					string plc_addr = "";
					unsigned short port;
		
					port = Configuration::instance()->slmp_plc_socket(plc_addr);
					if(m_pins_work.do_plc_connect(plc_addr.c_str(),port))
					{
						timeoutCnt = 0;
						LOGFMTD("PinsWorker Connect PLC %s",plc_addr.c_str());
					}
				}
				catch (int &e)
				{
					string josn_str = FormatConvert::instance().alarm_to_string(e);
					MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
				}
			}
			else	//PLC已连接
			{
				if(timer_click_flag)
				{
					m_pins_work.set_heartbeat_signal((heartbeat_signal++ % 2)+1);
					
					if(m_pins_work.get_plc_status_info())
					{
						timeoutCnt = 0;
						if(m_pins_work.get_plc_alarm_info())
						{
							plc_alarm_handle();
						}
					}
					else
					{
						timeoutCnt++;
						if(timeoutCnt > PLC_HEARTBEAT_TIMEOUT_CNT)
						{
							m_pins_work.close_connect();
						}
					}
				}
			}	
		}

		//处理上位机指令
		message_handle();
				
		// 状态监视
		if(timer_click_flag)
		{
			timer_click_flag = false;
			status_update_handle();
#if defined(AWH_FORMATION_SYS)
			if(Configuration::instance()->is_plc_enable())			//使能PLC
			{
				if(Configuration::instance()->is_vacuum_enable())	//负压系统
				{
					pre_formation_block_test();
					pre_formation_leak_test();
					formation_vac_task();
				}
			}
#else
			if(Configuration::instance()->is_plc_enable())			//使能PLC
			{
				formation_vac_task();								// 定容弹针床判断
			}
#endif
		}
#endif
	}

    try
    {
        MsgFactory::instance()->destory_pins_puller();
        MsgFactory::instance()->destory_pins_to_ext_reply_pusher();
        MsgFactory::instance()->destory_pins_puber();
    }
    catch (zmq::error_t &e)
    {
        LOGFMTE("Create ZMQ communication error. code: %d, description:%s", e.num(), e.what());
        string josn_str = FormatConvert::instance().alarm_to_string(-209);
        MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
    }

    MsgFactory::instance()->destory_pins_alarm_pusher();
}

void PinsWorker::pins_puber_send(int cell_no,Cmd_Cell_Msg_t &msg_struct)
{
	char filter[32] = {'\0'};	
	// 所有库
	if (cell_no == 0xFF)
	{
		for (int i=0; i<PINS_CELLS_NUM; ++i)
		{
			sprintf(filter, TOPIC_CELL_FORMAT_STR, i+1);		
			MsgFactory::instance()->pins_puber().send(filter, msg_struct, ZMQ_DONTWAIT);
		}
	}
	else	// 指定库位
	{
		sprintf(filter, TOPIC_CELL_FORMAT_STR, cell_no);		
		MsgFactory::instance()->pins_puber().send(filter, msg_struct, ZMQ_DONTWAIT);
	}
}

void PinsWorker::status_sync_cell(int cell_no, Pins_Status_Data_t &status)
{
	Cmd_Cell_Msg_t msg_struct = Cmd_Cell_Msg_t();
	
	msg_struct.func_code	       = SYS_COMMON_CMD;
	msg_struct.inside_msg.sub_code = SYS_PINS_STATUS_SYNC;
	msg_struct.inside_msg.pins_status = status;

	pins_puber_send(cell_no,msg_struct);
}

void PinsWorker::status_sync_cell(int cell_no, int sub_code, int value)
{	
	Cmd_Cell_Msg_t msg_struct = Cmd_Cell_Msg_t();
	msg_struct.func_code	       = SYS_COMMON_CMD;
	msg_struct.inside_msg.sub_code = sub_code;
	msg_struct.inside_msg.value	   = value;
		
	pins_puber_send(cell_no,msg_struct);
}


#if defined(AWH_PRECHARGE_SYS)
void PinsWorker::message_handle()
{
	try
	{
		Cmd_Pins_Msg_t recv = Cmd_Pins_Msg_t();

		while (MsgFactory::instance()->pins_puller().recevie(recv, ZMQ_DONTWAIT))
		{
			if (recv.cell_no > PINS_CELLS_NUM)  return;
			if (!m_pins_precharge[recv.cell_no-1].is_connected()) 
			{
				//string josn_str = FormatConvert::instance().alarm_to_string(-2410,recv.cell_no);
				//MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
				return;
			}
#ifdef ENABLE_PLC
			switch (recv.func_code)
			{
				case PINS_STATUS_QUERY:
					status_sync_cell(recv.cell_no, m_pins_status[recv.cell_no-1]);
					break;

				case PINS_ALARM_QUERY:
				{
					Fx_Error_Code_t tmp_err;

					memset((void*)&tmp_err,0,sizeof(Fx_Error_Code_t));
					string alarm_str = FormatConvert::instance().plc_alarm_to_json(recv.cell_no,m_pins_status[recv.cell_no-1].err_code,tmp_err);
					
					MsgFactory::instance()->pins_alarm_pusher().send(alarm_str);
					break;
				}

				// 工装取电开
				case CELL_POWER_ON:
				{
					// 库门防呆
					if (m_pins_precharge[recv.cell_no-1].get_sys_dor_alarm())
					{
						break;
					}
					// 针床压合防呆
					if (!m_pins_precharge[recv.cell_no-1].tray_is_extend())
					{
						break;
					}
					m_pins_precharge[recv.cell_no-1].set_tool_power_on();
					
					LOGFMT_DEBUG(cell_id_temp,"库位%d 工装取电开",recv.cell_no);
					break;
				}
				// 工装取电关
				case CELL_POWER_OFF:
				{
					m_pins_precharge[recv.cell_no-1].set_tool_power_off();

					LOGFMT_DEBUG(cell_id_temp,"库位%d 工装取电关",recv.cell_no);
					break;
				}
				// 托盘上升
				case CELL_PROBE_EXTEND:
				{
					// 库门打开防呆
					if (m_pins_precharge[recv.cell_no-1].get_sys_dor_alarm())
					{
						break;
					}

					m_pins_precharge[recv.cell_no-1].set_clear_alarm();

					if(m_pins_status[recv.cell_no-1].is_retract)		//针床脱开才需压合
					{
						status_sync_cell(recv.cell_no,SYS_TRAY_STATE_SET,TRAY_PROCESSING_CMD);
						msleep(200);
					}

					if(m_pins_status[recv.cell_no-1].is_tray_ready)		//托盘在位才发压合
					{
						m_pins_precharge[recv.cell_no-1].set_tray_moveup();
						LOGFMT_DEBUG(cell_id_temp,"库位%d 托盘上升",recv.cell_no);
					}
					else
					{
						LOGFMT_DEBUG(cell_id_temp,"库位%d 托盘不在位,托盘上升失败",recv.cell_no);
					}

					break;
				}

				case FIRE_EMERGENCY_BRAKE:										//最高消防等级制动
					status_sync_cell(recv.cell_no,SYS_CELL_ALARM_HANDLE,0);		//预充只当前库位停止

				// 托盘下降
				case CELL_PROBE_RETRACT:
				{
					// 库门打开防呆
					if (m_pins_precharge[recv.cell_no-1].get_sys_dor_alarm())
					{
						break;
					}

					if(m_pins_status[recv.cell_no-1].is_extend)			//针床压合才需脱开
					{
						status_sync_cell(recv.cell_no,SYS_TRAY_STATE_SET,TRAY_PROCESSING_CMD);
						msleep(200);
					}
					m_pins_precharge[recv.cell_no-1].set_tray_movedown();

					LOGFMT_DEBUG(cell_id_temp,"库位%d 托盘下降",recv.cell_no);
					break;
				}
				// 冷却风扇开
				case CELL_COOL_FAN_ON:
					m_pins_precharge[recv.cell_no-1].set_cool_fan_on();
					LOGFMT_DEBUG(cell_id_temp,"库位%d 冷风扇开",recv.cell_no);
					break;
				// 冷却风扇关
				case CELL_COOL_FAN_OFF:
					m_pins_precharge[recv.cell_no-1].set_cool_fan_off();
					LOGFMT_DEBUG(cell_id_temp,"库位%d 冷风扇关",recv.cell_no);
					break;
				// 抽风机开
				case CELL_AIR_FAN_ON:
					m_pins_precharge[recv.cell_no-1].set_air_fan_on();
					LOGFMT_DEBUG(cell_id_temp,"库位%d 抽风机开",recv.cell_no);
					break;
				// 抽风机关
				case CELL_AIR_FAN_OFF:
					m_pins_precharge[recv.cell_no-1].set_air_fan_off();
					LOGFMT_DEBUG(cell_id_temp,"库位%d 抽风机关",recv.cell_no);
					break;
				// 清除告警
				case CELL_CLEAR_WARNNING:
					m_pins_precharge[recv.cell_no-1].set_clear_alarm();
					LOGFMT_DEBUG(cell_id_temp,"库位%d 清除告警",recv.cell_no);
					break;
				// 初始化操作
				case CELL_LOAD:
					m_pins_precharge[recv.cell_no-1].set_load();
					LOGFMT_DEBUG(cell_id_temp,"库位%d 初始化操作",recv.cell_no);
					break;
				// 自动启动操作
				case CELL_AUTO_START:
					m_pins_precharge[recv.cell_no-1].set_auto_start();
					LOGFMT_DEBUG(cell_id_temp,"库位%d 自动启动操作",recv.cell_no);
					break;
				// 停止操作
				case CELL_STOP:
					m_pins_precharge[recv.cell_no-1].set_stop();
					LOGFMT_DEBUG(cell_id_temp,"库位%d 停止操作",recv.cell_no);
					break;
				// 设置库位温度告警
				case CELL_TEMP_SET:
					m_pins_precharge[recv.cell_no-1].set_cell_temp_waring(recv.pins_msg.temp);
					LOGFMT_DEBUG(cell_id_temp,"库位%d 设置库位温度告警,温度值:%f",recv.cell_no,recv.pins_msg.temp);
					break;
				// 反馈托盘信息
				case CELL_SET_TARY_STATUS:
					m_pins_precharge[recv.cell_no-1].incell_tray_info(recv.pins_msg.tray_info);
					LOGFMT_DEBUG(cell_id_temp,"库位%d 反馈托盘信息 状态:%d",recv.cell_no,recv.pins_msg.tray_info);
					break;
				// 托盘出库
				case CELL_TARY_OUT:
					m_pins_precharge[recv.cell_no-1].set_outcell_onoff();
					LOGFMT_DEBUG(cell_id_temp,"库位%d 托盘出库",recv.cell_no);
					break;
					
				default:
					//string josn_str = FormatConvert::instance().reply_to_string(recv.cell_no, -1, recv.func_code, -1, false);
					//MsgFactory::instance()->pins_to_ext_reply_pusher().send(josn_str);
					return;
			}
#endif		
#if 0
			if (recv.func_code != SYS_COMMON_CMD && recv.func_code != SYS_DO_PROTECT)
			{
				string josn_str = FormatConvert::instance().reply_to_string(recv.cell_no, -1, recv.func_code, -1, true);
				MsgFactory::instance()->pins_to_ext_reply_pusher().send(josn_str);
			}
#endif
		}
	}
	catch (int &e)
	{
		string josn_str = FormatConvert::instance().alarm_to_string(e);
		MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
		return;
	}
}
#else
void PinsWorker::message_handle()
{
	bool not_ack_flag = false;
	
	try
	{
		Cmd_Pins_Msg_t recv = Cmd_Pins_Msg_t();



		while (MsgFactory::instance()->pins_puller().recevie(recv, ZMQ_DONTWAIT))
		{
#ifdef ENABLE_PLC

			int cellNo = recv.cell_no;
			uint16 data = 0;
			bool status = false;

			if(!m_pins_work.is_connected())
			{
				return;
			}
			
			switch (recv.func_code)
			{
				case PINS_STATUS_QUERY:
					status_sync_cell(cellNo, m_pins_status[cellNo-1]);
					not_ack_flag = true;
					break;
				
				case PINS_ALARM_SYNC:
				{
					uint8 pins_err_code = recv.pins_msg.sub_params[0].func_code;
					if(pins_err_code > 0)
					{
						plc_alarm_mask[cellNo-1] |= (uint64_t)1 << (pins_err_code-1);
					}

					break;
				}

				case PINS_ALARM_QUERY:
				{
					for(uint8 ercIdx=0; ercIdx < MAX_PINS_ERROR_CODE; ercIdx++)
					{
						if((plc_alarm_mask[cellNo-1] & ((uint64_t)1 << ercIdx)) != 0)
						{
							string josn_str = FormatConvert::instance().pins_alarm_to_string(-2401-ercIdx,cellNo);
							MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
						}
					}
					not_ack_flag = true;
					break;
				}

				case CELL_VACUUM_TEST_CLEAR:
				{
					m_pins_status[cellNo-1].blockTestResult = 0;		//清除之前测试结果
					m_pins_status[cellNo-1].keepVacTestResult = 0;		//清除之前测试结果
					break;
				}
					
				// 启动抽真空
				case CELL_START_VACUUM_AUTO:		// 负压流程-抽真空 
					not_ack_flag = true;
				case CELL_START_VACUUM:
				{
					if(recv.pins_msg.sub_params[0].func_code == 0)
					{
						m_format_vac[cellNo-1].pump_vac_value = recv.pins_msg.sub_params[0].value;
					}
				
					if(recv.pins_msg.sub_params[1].func_code == 1)
					{
						m_format_vac[cellNo-1].pump_vac_timeout = (uint16)recv.pins_msg.sub_params[1].value;

						if(m_format_vac[cellNo-1].pump_vac_timeout == 0)		//抽负压时间为0 参数错误
						{
							status_sync_cell(cellNo,SYS_CELL_ALARM_HANDLE,-2400-VACUUM_PARA_ABNORMAL);
							send_pins_alarm(-2400-VACUUM_PARA_ABNORMAL,cellNo);
						}
					}

					if(m_format_vac[cellNo-1].pump_vac_value < 0.01)
					{
						m_pins_work.step_para_write_in(cellNo,false);
						m_pins_work.break_vacuum_control(cellNo,DISABLE_C);	//破真空置0
						
						m_pins_work.set_cell_high_vacuum_value(cellNo,m_format_vac[cellNo-1].pump_vac_value);
						m_pins_work.set_cell_pump_vacuum_timeout(cellNo,m_format_vac[cellNo-1].pump_vac_timeout);
						m_pins_work.set_cell_leak_vacuum_value(cellNo,m_format_vac[cellNo-1].leak_vac_value);
						m_pins_work.set_cell_leak_vacuum_timeout(cellNo,m_format_vac[cellNo-1].leak_vac_timeout);

						m_vac_state[cellNo-1] = E_VAC_WAIT;
						statewait_timeout[cellNo-1] = 0;
						m_pins_work.step_para_write_in(cellNo,true);
						status = m_pins_work.start_vacuum_control(cellNo,ENABLE_C);
					}

					LOGFMT_DEBUG(cell_id_temp,"库位%d 抽真空, 抽真空值:%f, 抽真空逾时:%d",\
						cellNo,m_format_vac[cellNo-1].pump_vac_value,m_format_vac[cellNo-1].pump_vac_timeout);
					
					break;
				}
	
				// 关闭抽真空
				case CELL_STOP_VACUUM:
					status = m_pins_work.start_vacuum_control(cellNo,DISABLE_C);
					LOGFMT_DEBUG(cell_id_temp,"库位%d 停止抽真空",cellNo);
					break;
		
				// 打开泄真空
				case CELL_DRAIN_VACUUM_AUTO:		// 负压流程-泄真空				
					not_ack_flag = true;
				case CELL_OPEN_DRAIN_VACUUM:
				{
					if(m_pins_status[cellNo-1].blockTestResult == 3)	//堵塞测试进行中 要先停止
					{
						m_pins_work.vac_block_test(cellNo,false);
					}
					
					if(recv.pins_msg.sub_params[0].func_code == 0)
					{
						m_format_vac[cellNo-1].leak_vac_value = recv.pins_msg.sub_params[0].value;
					}
				
					if(recv.pins_msg.sub_params[1].func_code == 1)
					{
						m_format_vac[cellNo-1].leak_vac_timeout = (uint16)recv.pins_msg.sub_params[1].value;

						if(m_format_vac[cellNo-1].leak_vac_timeout == 0)		//泄负压时间为0 参数错误
						{
							status_sync_cell(cellNo,SYS_CELL_ALARM_HANDLE,-2400-VACUUM_PARA_ABNORMAL);
							send_pins_alarm(-2400-VACUUM_PARA_ABNORMAL,cellNo);
						}
					}

					m_pins_work.step_para_write_in(cellNo,false);
					m_pins_work.start_vacuum_control(cellNo,DISABLE_C);
					
					m_pins_work.set_cell_high_vacuum_value(cellNo,m_format_vac[cellNo-1].pump_vac_value);
					m_pins_work.set_cell_pump_vacuum_timeout(cellNo,m_format_vac[cellNo-1].pump_vac_timeout);
					m_pins_work.set_cell_leak_vacuum_value(cellNo,m_format_vac[cellNo-1].leak_vac_value);
					m_pins_work.set_cell_leak_vacuum_timeout(cellNo,m_format_vac[cellNo-1].leak_vac_timeout);

					m_vac_state[cellNo-1] = E_LEAK_WAIT;
					m_pins_work.step_para_write_in(cellNo,true);	
					
					status = m_pins_work.break_vacuum_control(cellNo,ENABLE_C);

					LOGFMT_DEBUG(cell_id_temp,"库位%d 泄真空, 泄真空值:%f, 泄真空逾时:%d",\
						cellNo,m_format_vac[cellNo-1].leak_vac_value,m_format_vac[cellNo-1].leak_vac_timeout);
					
					break;
				}
		
				// 关闭泄真空
				case CELL_CLOSE_DRAIN_VACUUM:
					status = m_pins_work.break_vacuum_control(cellNo,DISABLE_C);
					LOGFMT_DEBUG(cell_id_temp,"库位%d 停止泄真空",cellNo);
					break;
				// 工装取电开	
				case CELL_POWER_ON:
					status = m_pins_work.tools_power_control(cellNo,ENABLE_C);
					break;
				// 工装取电关
				case CELL_POWER_OFF:
					status = m_pins_work.tools_power_control(cellNo,DISABLE_C);
					break;
					
				// 托盘上升
				case CELL_TRAY_UP:
				{
					plc_alarm_mask[cellNo-1] = 0;
		#if defined(AWH_FORMATION_SYS)
					is_vacuum_error[cellNo-1] = false;
		#endif
					m_pins_work.clear_once_alarm();
					LOGFMT_DEBUG(cell_id_temp,"库位%d 清除告警",cellNo);
			
					if(m_pins_status[cellNo-1].pins_unpress)				//针床脱开才需压合
					{
						status_sync_cell(cellNo,SYS_TRAY_STATE_SET,TRAY_PROCESSING_CMD);
						msleep(200);
					}
					
					if(m_pins_status[cellNo-1].tray_in_place)				//托盘在位才发送压合
					{
						status = m_pins_work.set_tray_action(cellNo,UP_T);
						m_pins_work.vent_fan_control(cellNo,ENABLE_C);
						LOGFMT_DEBUG(cell_id_temp,"库位%d 托盘上升",cellNo);
					}
					else
					{
						LOGFMT_DEBUG(cell_id_temp,"库位%d 托盘上升失败, 托盘不在位",cellNo);
					}

					break;
				}
		
				// 托盘下降
				case CELL_TRAY_DOWN:
				{
					if(m_pins_status[cellNo-1].pins_extend)				//针床压合才需脱开
					{
						status_sync_cell(cellNo,SYS_TRAY_STATE_SET,TRAY_PROCESSING_CMD);
						msleep(200);
					}
					status = m_pins_work.set_tray_action(cellNo,DOWN_T);
					m_pins_work.vent_fan_control(cellNo,DISABLE_C);
					LOGFMT_DEBUG(cell_id_temp,"库位%d 托盘下降",cellNo);
				}
			
			#if defined(AWH_FORMATION_SYS)
					if(m_pins_status[cellNo-1].vacuum < -7.0)
					{
						LOGFMT_DEBUG(cell_id_temp,"库位%d 负压值: %f",cellNo,m_pins_status[cellNo-1].vacuum);
					}
			#endif
					break;
		
				// 运行模式
				case CELL_SET_AUTO_MODE:
					status = m_pins_work.set_PLC_workMode(cellNo,AUTO_M);
					break;

				// 维护模式
				case CELL_SET_MANUAL_MODE:
					status = m_pins_work.set_PLC_workMode(cellNo,MAINTAIN_M);
					break;
		
				// 校准模式
				case CELL_CALIBRATE_MODE:
					status = m_pins_work.set_calibrate_mode(cellNo);
					break;
		
				// 打开风扇
				case CELL_OPEN_FIRE_FAN:
					status = m_pins_work.vent_fan_control(cellNo,ENABLE_C);
					break;
		
				// 关闭风扇
				case CELL_CLOSE_FIRE_FAN:
					status = m_pins_work.vent_fan_control(cellNo,DISABLE_C);
					break;
				
				// 消防门上升
				case CELL_FIRE_DOOR_UP:
					status = m_pins_work.fireDoor_control(cellNo,UP_T);
					break;

				// 消防门下降
				case CELL_FIRE_DOOR_DOWN:
					status = m_pins_work.fireDoor_control(cellNo,DOWN_T);
					break;
		
				case CELL_BLOCK_TEST_MANUAL:			// 手动模式-堵塞测试						
					if(recv.pins_msg.sub_params[0].func_code == 0)
					{
						m_vac_para[cellNo-1].blockSetVal = recv.pins_msg.sub_params[0].value;
					}

					if(recv.pins_msg.sub_params[1].func_code == 1)
					{
						m_vac_para[cellNo-1].blockPumpTime = (int)recv.pins_msg.sub_params[1].value;
					}
				
				case CELL_BLOCK_TEST_AUTO:				// 自动模式-堵塞测试
				{
					m_pins_status[cellNo-1].blockTestResult = 0;		//清除之前测试结果
					m_pins_status[cellNo-1].keepVacTestResult = 0;		//清除之前测试结果
					if(m_vac_para[cellNo-1].blockPumpTime > 0)
					{
						m_pins_work.start_vacuum_control(cellNo,DISABLE_C);	//抽真空置0
						m_pins_work.break_vacuum_control(cellNo,DISABLE_C);	//破真空置0
						
						status = m_pins_work.vac_block_test(cellNo,true);
						m_block_test_state[cellNo-1] = E_WAIT;
						
						LOGFMT_DEBUG(cell_id_temp,"库位%d, 开始堵塞测试, 堵塞值:%f, 堵塞时间:%d",\
						cellNo,m_vac_para[cellNo-1].blockSetVal,m_vac_para[cellNo-1].blockPumpTime);
					}
					else
					{
						m_pins_status[cellNo-1].blockTestResult = 2;
						status_sync_cell(cellNo,SYS_BLOCK_TEST_ACK,2);
						send_pins_alarm(-2400-VACUUM_PARA_ABNORMAL,cellNo);
					}

					if(recv.func_code == CELL_BLOCK_TEST_AUTO)
					{
						not_ack_flag = true;
					}
					
					break;
				}

				case CELL_LEAK_RATE_TEST_MANUAL:		// 手动模式-保压测试
				{
					if(recv.pins_msg.sub_params[0].func_code == 0)
					{
						m_vac_para[cellNo-1].KeepVacVal = recv.pins_msg.sub_params[0].value;
					}

					if(recv.pins_msg.sub_params[1].func_code == 1)
					{
						m_vac_para[cellNo-1].KeepVacStableTime = (int)recv.pins_msg.sub_params[1].value;
					}

					if(recv.pins_msg.sub_params[2].func_code == 2)
					{
						m_vac_para[cellNo-1].KeepVacLeakRate = recv.pins_msg.sub_params[2].value;
					}

					if(recv.pins_msg.sub_params[3].func_code == 3)
					{
						m_vac_para[cellNo-1].KeepVacTime = (int)recv.pins_msg.sub_params[3].value;
					}

					if(recv.pins_msg.sub_params[4].func_code == 4)
					{
						m_vac_para[cellNo-1].KeepVacTestTimeout = (int)recv.pins_msg.sub_params[4].value;
					}

					if(m_pins_status[cellNo-1].pins_unpress)				//针床脱开才需压合
					{
						status_sync_cell(cellNo,SYS_TRAY_STATE_SET,TRAY_PROCESSING_CMD);
						msleep(200);
					}

					if(m_pins_status[cellNo-1].tray_in_place)					//托盘在位才发送压合
					{
						status = m_pins_work.set_tray_action(cellNo,UP_T);		//只需执行托盘上升
						m_pins_work.vent_fan_control(cellNo,ENABLE_C);
					}
				}
				case CELL_LEAK_RATE_TEST_AUTO:			// 自动模式-保压测试
				{
					m_pins_status[cellNo-1].keepVacTestResult = 0;		//清除之前测试结果
					if(m_vac_para[cellNo-1].KeepVacTime > 0)
					{
						//float keep_vac_contrast;

						m_pins_work.start_vacuum_control(cellNo,DISABLE_C);	//关闭抽真空
						m_pins_work.break_vacuum_control(cellNo,DISABLE_C);	//破真空置0
						//D372 保压破真空值
						m_pins_work.set_keep_vacuum_break_value(-0.5);
						//D5300~D5308 保压目标值
						m_pins_work.set_cell_keep_vacuum_target(cellNo,m_vac_para[cellNo-1].KeepVacVal);
						
						//D373 保压比较值  弃用
						//keep_vac_contrast = m_vac_para[cellNo-1].KeepVacVal;
						//keep_vac_contrast += ((float)m_vac_para[cellNo-1].KeepVacTime / 60.0)*m_vac_para[cellNo-1].KeepVacLeakRate;
						//m_pins_work.set_keep_vacuum_test_value(keep_vac_contrast);
						//D5310~D5318    新增漏率寄存器
						m_pins_work.set_cell_keep_vacuum_leakrate(cellNo,m_vac_para[cellNo-1].KeepVacLeakRate);

						//D5320~D5328 保压稳定时间
						m_pins_work.set_cell_keep_vacuum_stableTime(cellNo,m_vac_para[cellNo-1].KeepVacStableTime);
						//D5340~D5348 保压时间
						m_pins_work.set_cell_keep_vacuum_time(cellNo,m_vac_para[cellNo-1].KeepVacTime);

						m_leak_test_state[cellNo-1] = E_EXTEND;
						statewait_timeout[cellNo-1] = 0;
					}
					else
					{
						m_pins_status[cellNo-1].keepVacTestResult = 2;
						status_sync_cell(cellNo,SYS_LEAK_TEST_ACK,2);
						send_pins_alarm(-2400-VACUUM_PARA_ABNORMAL,cellNo);
					}

					LOGFMT_DEBUG(cell_id_temp,"库位%d, 开始保压测试, 保压值:%f, 稳定时间:%d, 漏率:%f, 保压时间:%d,",\
					cellNo,m_vac_para[cellNo-1].KeepVacVal,m_vac_para[cellNo-1].KeepVacStableTime,m_vac_para[cellNo-1].KeepVacLeakRate,m_vac_para[cellNo-1].KeepVacTime);

					if(recv.func_code == CELL_LEAK_RATE_TEST_AUTO)
					{
						not_ack_flag = true;
					}
					
					break;
				}

				// 系统红灯
				case CELL_RED_LIGHT:
					status = m_pins_work.set_system_lamp_buzzer(RED_I);
					break;
		
				// 系统黄灯
				case CELL_YELLOW_LIGHT:
					status = m_pins_work.set_system_lamp_buzzer(YELLOW_I);
					break;
		
				// 系统绿灯
				case CELL_GREEN_LIGHT:
					status = m_pins_work.set_system_lamp_buzzer(GREEN_I);
					break;
		
				// 蜂鸣器
				case CELL_BUZZER:
					status = m_pins_work.set_system_lamp_buzzer(BUZZER_I);
					break;
		
				// 库位OPT1
				case CELL_OPT1:
					status = m_pins_work.set_cell_OTP1(cellNo);
					break;
					
				// 库位OPT2
				case CELL_OPT2:
					status = m_pins_work.set_cell_OTP2(cellNo);
					break;

				// 针床告警清除
				case CELL_CLEAR_WARNNING:
				{
					plc_alarm_mask[cellNo-1] = 0;
			#if defined(AWH_FORMATION_SYS)		
					is_vacuum_error[cellNo-1] = false;
			#endif
					status = m_pins_work.clear_once_alarm();
					break;
				}

				case CELL_VAC_PARA_SET:
				{
					if(recv.pins_msg.sub_params[0].func_code == 0)
					{
						m_vac_para[cellNo-1].blockSetVal = recv.pins_msg.sub_params[0].value;
					}

					if(recv.pins_msg.sub_params[1].func_code == 1)
					{
						m_vac_para[cellNo-1].blockPumpTime = (int)recv.pins_msg.sub_params[1].value;
					}

					if(recv.pins_msg.sub_params[2].func_code == 2)
					{
						m_vac_para[cellNo-1].KeepVacVal = recv.pins_msg.sub_params[2].value;
					}

					if(recv.pins_msg.sub_params[3].func_code == 3)
					{
						m_vac_para[cellNo-1].KeepVacStableTime = (int)recv.pins_msg.sub_params[3].value;
					}

					if(recv.pins_msg.sub_params[4].func_code == 4)
					{
						m_vac_para[cellNo-1].KeepVacLeakRate = recv.pins_msg.sub_params[4].value;
					}

					if(recv.pins_msg.sub_params[5].func_code == 5)
					{
						m_vac_para[cellNo-1].KeepVacTime = (int)recv.pins_msg.sub_params[5].value;
					}

					if(recv.pins_msg.sub_params[6].func_code == 6)
					{
						m_vac_para[cellNo-1].KeepVacTestTimeout = (int)recv.pins_msg.sub_params[6].value;
					}

					break;
				}
				
				case PLC_REG_READ:
				{
					int addr = 0;
					int offset = 0;

					if(recv.pins_msg.sub_params[0].func_code == 0)
					{
						addr = (int)recv.pins_msg.sub_params[0].value;
					}
					
					if(recv.pins_msg.sub_params[1].func_code == 1)
					{
						offset = (int)recv.pins_msg.sub_params[1].value;
					}

					if(addr > 0)
					{
						status = m_pins_work.plc_register_read(addr,offset,data);
					}

					break;
				}
					
				case PLC_REG_WRITE:
				{
					int addr = 0;
					int offset = 0;
					uint16 val= 0;

					if(recv.pins_msg.sub_params[0].func_code == 0)
					{
						addr = (int)recv.pins_msg.sub_params[0].value;
					}
					
					if(recv.pins_msg.sub_params[1].func_code == 1)
					{
						offset = (int)recv.pins_msg.sub_params[1].value;
					}

					if(recv.pins_msg.sub_params[2].func_code == 2)
					{
						val = (uint16)recv.pins_msg.sub_params[2].value;
					}

					if(addr > 0)
					{
						status = m_pins_work.plc_register_write(addr,offset,val);
					}
					
					break;
				}
				
				case PLC_SW_VERSION:
					status = m_pins_work.get_plc_software_version(data);
					break;

				case FIRE_EMERGENCY_BRAKE:										//最高消防等级制动
				{
					for(int i = 0; i < PINS_CELLS_NUM; i++)
					{
						if(m_vac_state[i] != E_VAC_RELEASE && m_vac_state[i] != E_RELEASE_WAIT)
						{
							status_sync_cell(i+1,SYS_CELL_ALARM_HANDLE,0);		//所有库位停止
							m_pins_work.fireDoor_control(i+1,UP_T);				//所有库消防门关闭
							m_vac_state[i] = E_VAC_RELEASE;						//所有库位泄压
							fire_emergency_flag[i] = 1;
						#if defined(AWH_FORMATION_SYS)
							LOGFMTD("fireAlarm Level1 cell%d vacuum release",i+1);
						#endif
						}
					}
					
					break;
				}
				
				default:
					//string josn_str = FormatConvert::instance().reply_to_string(recv.cell_no, -1, recv.func_code, -1, false);
					//MsgFactory::instance()->pins_to_ext_reply_pusher().send(josn_str);
					return;
			}
#endif
			if(!not_ack_flag)
			{
				int ret = 0;
				if(!status)
				{
					ret = -1;
				}
				
				pins_ack_reply(cellNo,recv.func_code,ret,data);
			}
		}
	}
	catch (int &e)
	{
		string josn_str = FormatConvert::instance().alarm_to_string(e);
		MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
		return;
	}
}
#endif

#if 0
#if defined(PLC_JYT)
void PinsWorker::status_update_handle()
{
#ifdef ZHONGHANG_SYSTEM_PROTECTION
	// 全局告警
	uint16_t read_err[2];
	PinsGroup::query_global_alarm(read_err);	

	for (uint8_t i=0; i<2; ++i)
	{
		bitset<16> error_code(read_err[i]);
		bitset<16> last_error(m_global_alarm[i]);
		
		for (uint8_t j=0; j<16; ++j)
		{
			if ((error_code[j] != 0) && (error_code[j] != last_error[j]))
			{
				// 推送告警信息
				int err_code = -1600 - (i * 16) - j;
				string josn_str = FormatConvert::instance().alarm_to_string(err_code);
				MsgFactory::instance()->pins_alarm_pusher().send(josn_str);

				// 整柜急停
				if (i == 0 && j == 0)
				{
					status_sync_cell(0xFF, SYS_CELL_ALARM_HANDLE, err_code);
				}
			}
		}
	}
	memcpy((void *)m_global_alarm, (void *)read_err, sizeof(read_err));
	
	// 电源柜温度告警判断
	float powerTemp[16];
	PinsGroup::query_power_cabinet_temp(powerTemp);
	for (uint8_t i=0; i<sizeof(powerTemp)/sizeof(powerTemp[0]); ++i)
	{
		if (powerTemp[i] > m_power_TUL)
		{
			string josn_str = FormatConvert::instance().alarm_to_string(-1055);
			MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
			break;
		}
	}	
#endif

	// 更新所有库位状态
	for (int cell=0; cell<PINS_CELLS_NUM; cell++)
	{
		int cell_no = cell + 1;
		Pins_Status_Data_t status = m_pins[cell].get_env_datas();
		status.cell = cell_no;

		// 数据同步给库位线程
		status_sync_cell(status.cell, status);
		
		// 状态上传（上位机控制数据是否上传）
		if (Configuration::instance()->need_send_detail_data(cell_no))
		{
			string josn_str = FormatConvert::instance().pins_data_to_json(cell_no, status);
			MsgFactory::instance()->pins_to_ext_reply_pusher().send(josn_str);
		}
		// 写入log(lbfs/data/目录)
		string store_josn_str = FormatConvert::instance().pins_data_to_store_str(cell_no, status);
		LOG_INFO(cell_id_temp, store_josn_str);

		// 故障码上报
		for (uint8_t i=0; i<5; ++i)
		{
			bitset<16> error_code(status.err_code[i]);
			bitset<16> last_error(m_pins_status[cell].err_code[i]);
			
			for (uint8_t j=0; j<16; ++j)
			{
				if ((error_code[j] != 0) && (error_code[j] != last_error[j]))
				{
					// 告警处理
					int err_code = -1500 - (i * 16) - j;
					cell_alarm_handler(cell_no, err_code);
				}
			}
			// 更新恒温告警状态
			if (i == 2)
			{
				m_bf_sm[cell].c_temp_alarm = error_code[2];
			}
		}
		
		// 针床压合状态同步给辅助电压采集盒
		status_sync_auxModule(cell_no, status);
		// 电池温度告警（库温）
		cellCabinet_tempDetect(cell_no, status);
		// 保存状态值
		memcpy((void *)&m_pins_status[cell], (void *)&status, sizeof(status));
		
		// 检测抽/泄负压是否完成
		if (m_handOp_Info[cell].is_vac_started || m_handOp_Info[cell].is_vac_stopped)
		{
			int step_no = m_pins[cell].get_step_no();

			if (m_handOp_Info[cell].is_vac_started)
			{
				if ((step_no == 22) && (step_no != m_handOp_Info[cell].last_step_no))
				{
					m_handOp_Info[cell].is_vac_started = false;
					batteryFormation_status_notify(cell_no, -2424);
				}
			}
			else
			{
				if ((step_no >= 24) && (step_no != m_handOp_Info[cell].last_step_no))
				{
					m_handOp_Info[cell].is_vac_stopped = false;
					batteryFormation_status_notify(cell_no, -2425);
				}
			}
			m_handOp_Info[cell].last_step_no = step_no;
		}
	}
}

#elif defined(PLC_JS_CT)
void PinsWorker::status_update_handle()
{
	// 全局告警
#ifdef ZHONGHANG_SYSTEM_PROTECTION
	bitset<8> status = PinsGroup::query_cabinet_signal();	
	
	// bit依次为：电源柜急停、电源柜断电、AC侧过流、AC侧过压、AC侧欠压
	for (int i=0; i<5; ++i)
	{
		if ((status[i] != 0) && (status[i] != m_power_cabinet_status[i]))
		{
			// 推送告警
			int error_code = -1040 - i;
			string josn_str = FormatConvert::instance().alarm_to_string(error_code);
			MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
			
			// 解决上位机通道显示告警问题（急停、断电）
			if (i == 0 || i == 1 || i == 4)
			{
				status_sync_cell(0xFF, SYS_CELL_ALARM_HANDLE, error_code);
			}
			// 电源柜急停
			if (i == 0)
			{
				m_ac_halt_alarm = true;
			}
		}
	}
	
	// 电源柜温度
	float cabinetTemp[4];
	PinsGroup::query_power_cabinet_temp(cabinetTemp);
	for (uint8_t i=0; i<sizeof(cabinetTemp)/sizeof(cabinetTemp[0]); ++i)
	{
		if (cabinetTemp[i] > m_power_TUL)
		{
			string josn_str = FormatConvert::instance().alarm_to_string(-1055);
			MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
			break;
		}
	}
	m_power_cabinet_status = status;
#endif

	// 更新所有库位数据 (FAT恒温PLC只有2库)
	for (int cell=1; cell<2; cell++)
	{
		int cell_no = cell + 1;
		Pins_Status_Data_t status = m_pins[cell].get_env_datas();
		status.cell = cell_no;
		// 状态上传（上位机控制数据是否上传）
		if (Configuration::instance()->need_send_detail_data(cell_no))
		{
			string josn_str = FormatConvert::instance().pins_data_to_json(cell_no, status);
			MsgFactory::instance()->pins_to_ext_reply_pusher().send(josn_str);
		}
		// 写入log(lbfs/data/目录)
		string store_josn_str = FormatConvert::instance().pins_data_to_store_str(cell_no, status);
		LOG_INFO(cell_id_temp, store_josn_str);

		// 烟感
		bitset<2> smokeStatus(status.smokeStatus);
		for (uint8_t j=0; j<smokeStatus.size(); ++j)
		{
			if (smokeStatus[j] != 0)
			{
				int ex_code = -1052 - j;
				system_protection(cell_no, true);

				string josn_str = FormatConvert::instance().alarm_to_string(ex_code, cell_no);
				MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
			}
		}

		// 故障码上报
		bitset<32> error_code(status.err_code);
		bitset<32> last_error(m_pins_status[cell].err_code);
		bool smokeFlag = false;
		
		// 忽略bit0
		for (uint8_t j=1; j<error_code.size(); ++j)
		{
			if ((error_code[j] != 0) && (error_code[j] != last_error[j]))
			{
#ifdef ZHONGHANG_SYSTEM_PROTECTION
				// 保护功能: 烟雾报警
				if (j == 7)
				{
					system_protection(cell_no, true);
				}
				// 保护功能：压床急停
				if (j == 21)
				{		
					system_protection(cell_no, false);
				}
#endif
				// 推送告警信息
				int ex_code = -1000 - j;
				string josn_str = FormatConvert::instance().alarm_to_string(ex_code, cell_no);
				MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
			}
		}
//		// 同步烟感状态给库位线程
//		status_sync_cell(cell_no, SYS_SMOKE_STATUS_SYNC, smokeFlag);

		// 针床压合状态同步给辅助电压采集盒
		status_sync_auxModule(cell_no, status);
		// 电池温度告警
		cellCabinet_tempDetect(cell_no, status);
		// 保存状态值
		memcpy((void *)&m_pins_status[cell], (void *)&status, sizeof(status));
	}
}

#elif defined(PLC_JS_VF)
void PinsWorker::status_update_handle()
{
	// 全局告警
#ifdef ZHONGHANG_SYSTEM_PROTECTION
	bitset<8> status = PinsGroup::query_cabinet_signal();	

	// bit依次为：电箱急停、面板急停
	for (int i=0; i<2; ++i)
	{
		if ((status[i] != 0) && (status[i] != m_power_cabinet_status[i]))
		{
			string josn_str = FormatConvert::instance().alarm_to_string(-1364 - i);
			MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
		}
	}
	m_power_cabinet_status = status;
#endif

	// 更新所有库位状态
	for (int cell=0; cell<PINS_CELLS_NUM; cell++)
	{
		int cell_no = cell + 1;
		Pins_Status_Data_t status = m_pins[cell].get_env_datas();
		status.cell = cell_no;
		// 状态上传（上位机控制数据是否上传）
		if (Configuration::instance()->need_send_detail_data(cell_no))
		{
			string josn_str = FormatConvert::instance().pins_data_to_json(cell_no, status);
			MsgFactory::instance()->pins_to_ext_reply_pusher().send(josn_str);
		}
		// 写入log(lbfs/data/目录)
		string store_josn_str = FormatConvert::instance().pins_data_to_store_str(cell_no, status);
		LOG_INFO(cell_id_temp, store_josn_str);

		// 故障码上报
		bitset<9> error_code(status.err_code);
		bitset<9> last_error(m_pins_status[cell].err_code);
		
		for (uint8_t j=0; j<error_code.size(); ++j)
		{
			if ((error_code[j] != 0) && (error_code[j] != last_error[j]))
			{
				// 推送告警信息
				int ex_code = -1300 - j;
				string josn_str = FormatConvert::instance().alarm_to_string(ex_code, cell_no);
				MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
				LOGFMTD("cell=%d,  cell_no=%d", cell, cell_no);
			}
		}

		// 针床压合状态同步给辅助电压采集盒
		status_sync_auxModule(cell_no, status);
		// 电池温度告警
		cellCabinet_tempDetect(cell_no, status);
		// 保存状态值
		memcpy((void *)&m_pins_status[cell], (void *)&status, sizeof(status));
	}
}
#endif
#endif

#if defined(AWH_PRECHARGE_SYS)
void PinsWorker::get_cell_temp_data(int cellNo,Pins_Status_Data_t &status)
{
	channel_temperature_t temp_data;
	
	g_AuxTempAcq.get_cell_temperature(cellNo,temp_data);
	memcpy((void*)&status.temp[0],(void*)&temp_data.chan1,CELL_TEMP_ACQ_NUM*sizeof(float));
}
bool PinsWorker::error_cmp(const Fx_Error_Code &a ,const Fx_Error_Code &b)
{
		return ( a.PressureError == b.PressureError) && (a.EStop == b.EStop)  \
			&&( a.PLCHeartBeat == b.PLCHeartBeat) && (a.DoorOpen == b.DoorOpen)  \
			&&( a.MoveUpTimeout == b.MoveUpTimeout) && (a.MoveDownTimeout == b.MoveDownTimeout)  \
			&&( a.TrayReverse == b.TrayReverse) && (a.NoTray == b.NoTray)  \
			&&( a.TrayNotReady == b.TrayNotReady) && (a.FanOverload == b.FanOverload)  \
			&&( a.RollerOverload == b.RollerOverload) && (a.RollerRunTimeout == b.RollerRunTimeout)  \
			&&( a.RollerReverseRunTimeout == b.RollerReverseRunTimeout) && (a.LeftFanError == b.LeftFanError)  \
			&&( a.TopFanError == b.TopFanError) && (a.RightFanError == b.RightFanError)  \
			&&( a.Smoke1Alarm == b.Smoke1Alarm) && (a.Smoke2Alarm == b.Smoke2Alarm)  \
			&&( a.CabSmokeAlarm == b.CabSmokeAlarm) && (a.PinSmokeAlarm == b.PinSmokeAlarm)  \
			&&( a.PinSmokeHeavyAlarm == b.PinSmokeHeavyAlarm) && (a.HighTempAlarm == b.HighTempAlarm);
}

void PinsWorker::status_update_handle()
{
	// 更新所有库位状态
	for (int cell=0; cell<PINS_CELLS_NUM; cell++)
	{
		int cell_no = cell + 1;
		
		Pins_Status_Data_t status;
		memset((void *)&status, 0, sizeof(status));

		status.cell = cell_no;
		get_cell_temp_data(cell_no,status);

		if(m_pins_precharge[cell].is_connected())
		{
			if(m_pins_precharge[cell].set_cell_temp(status.temp))
			{
				status.temp_warning = (m_pins_precharge[cell].get_cell_temp_waring())/10.0;
				m_pins_precharge[cell].get_error_code(status.err_code);
				m_pins_precharge[cell].get_status_data(status);
			}

			if(status.err_code.EStop)			//库位急停
			{
				if(!is_sudden_stop[cell])
				{
					is_sudden_stop[cell] = true;
					status_sync_cell(cell_no,SYS_CELL_ALARM_HANDLE,0);
					LOGFMT_DEBUG(cell_id_temp,"针床库位%d 紧急停止",cell_no);
				}
			}
			else
			{
				is_sudden_stop[cell] = false;
			}

			if(last_pins_state[cell] != m_pins_status[cell].is_extend)
			{
				int cmd_value;
				
				if(m_pins_status[cell].is_extend)		//托盘已压合
				{
					cmd_value = TART_PRESSED_CMD;
				}
				else
				{
					cmd_value = TARY_UNPRESS_CMD;
				}
				msleep(50);
				status_sync_cell(cell+1,SYS_TRAY_STATE_SET,cmd_value);
				last_pins_state[cell] = m_pins_status[cell].is_extend;
			}
		}
		else
		{
			status.is_plc_connect = false;
		}
	
		// 状态上传（上位机控制数据是否上传）
//		if (Configuration::instance()->need_send_detail_data(cell_no))
		{
			string josn_str = FormatConvert::instance().pins_data_to_json(cell_no, status);
			MsgFactory::instance()->pins_to_ext_reply_pusher().send(josn_str);
		}
	
		// 推送告警信息
		if(!error_cmp(status.err_code,m_pins_status[cell].err_code))
		{
			// 推送告警信息
			string alarm_str = FormatConvert::instance().plc_alarm_to_json(cell_no,status.err_code,m_pins_status[cell].err_code);
			MsgFactory::instance()->pins_alarm_pusher().send(alarm_str);
		}
		// 保存状态值
		memcpy((void *)&m_pins_status[cell], (void *)&status, sizeof(status));
	}
}
#else

string PinsWorker::get_funccode_string(int funccode)
{
	switch(funccode)
	{
		case CELL_START_VACUUM:
			return "start_pump_vacuum";
		case CELL_STOP_VACUUM:
			return "stop_pump_vacuum";
		case CELL_OPEN_DRAIN_VACUUM:
			return "start_release_vacuum";
		case CELL_CLOSE_DRAIN_VACUUM:
			return "stop_release_vacuum";
		case CELL_POWER_ON:
			return "24V_on";
		case CELL_POWER_OFF:
			return "24V_off";
		case CELL_TRAY_UP:
			return "tray_up";
		case CELL_TRAY_DOWN:
			return "tray_down";
		case CELL_SET_AUTO_MODE:
			return "auto_mode";
		case CELL_SET_MANUAL_MODE:
			return "maintenance_mode";
		case CELL_CALIBRATE_MODE:
			return "calibration_mode";
		case CELL_OPEN_FIRE_FAN:
			return "start_fan";
		case CELL_CLOSE_FIRE_FAN:
			return "stop_fan";
		case CELL_FIRE_DOOR_UP:
			return "fire_door_up";
		case CELL_FIRE_DOOR_DOWN:
			return "fire_door_down";
		case CELL_BLOCK_TEST_MANUAL:
			return "start_block_test";
		case CELL_LEAK_RATE_TEST_MANUAL:
			return "leak_rate_test";	
		case CELL_RED_LIGHT:
			return "red_light";
		case CELL_YELLOW_LIGHT:
			return "yellow_light";
		case CELL_GREEN_LIGHT:
			return "green_light";
		case CELL_BUZZER:
			return "buzzer";
		case CELL_OPT1:
			return "otp1";
		case CELL_OPT2:
			return "otp2";
		case CELL_CLEAR_WARNNING:
			return "plc_alarm_reset";
		case CELL_VAC_PARA_SET:
			return "vacuum_set";
		case PLC_REG_READ:
			return "register_read";
		case PLC_REG_WRITE:
			return "register_write";
		case PLC_SW_VERSION:
			return "plc_sw_version";
			
		default:
			return "Invalid";
	}	
}


void PinsWorker::pins_ack_reply(int cell_no,int funcode,int status,uint16 data)
{
	string func_code_str = get_funccode_string(funcode);
	if(func_code_str != "Invalid")
	{
		string josn_str = FormatConvert::instance().pinscell_reply_to_string(cell_no, func_code_str, status, data);
		MsgFactory::instance()->pins_to_ext_reply_pusher().send(josn_str);
	}
}


void PinsWorker::get_cell_temp_data(int cellNo,Pins_Status_Data_t &status)
{
	channel_temperature_t temp_data;
	
	g_AuxTempAcq.get_cell_temperature(cellNo,temp_data);
	memcpy((void*)&status.temp[0],(void*)&temp_data.chan1,CELL_TEMP_ACQ_NUM*sizeof(float));
}

void PinsWorker::get_cell_pins_data(int cellNo,Pins_Status_Data_t &status)
{
	status.vacuum = m_pins_work.return_vacuum_value(cellNo);
	
	status.warn_poweroff_delay = m_pins_work.return_warn_poweroff_delay();
	status.emstop_poweroff_delay = m_pins_work.return_emstop_poweroff_delay();
	
	status.pins_extend = m_pins_work.return_pins_is_press_bitVal(cellNo);
	status.pins_unpress = m_pins_work.return_pins_is_unpress_bitVal(cellNo);
	
	status.tray_in_place = m_pins_work.return_tray_in_place_bitVal(cellNo);
	status.fire_door_open = m_pins_work.return_fireDoor_is_open_bitVal(cellNo);

	status.is_auto_mode = m_pins_work.return_auto_mode(cellNo);
	status.is_plc_connect = m_pins_work.is_connected();
}

void PinsWorker::status_update_handle()
{
	// 更新所有库位状态
	for (int i=0; i<PINS_CELLS_NUM; i++)
	{
		int cell_no = i + 1;
		
		get_cell_temp_data(cell_no,m_pins_status[i]);
		if(m_pins_work.is_connected())	
		{
			get_cell_pins_data(cell_no,m_pins_status[i]);

			if(last_pins_state[i] != m_pins_status[i].pins_extend)
			{
				int cmd_value;
				
				if(m_pins_status[i].pins_extend)		//托盘已压合
				{
					cmd_value = TART_PRESSED_CMD;
				}
				else
				{
					cmd_value = TARY_UNPRESS_CMD;
				}
				msleep(50);
				status_sync_cell(i+1,SYS_TRAY_STATE_SET,cmd_value);
				last_pins_state[i] = m_pins_status[i].pins_extend;
			}

#if defined(AWH_FORMATION_SYS)
			if(!m_pins_status[i].tray_in_place)					//托盘不在位清除之前的测试结果
			{
				if(m_pins_status[i].blockTestResult != 0)
				{
					m_pins_status[i].blockTestResult = 0;		//清除之前测试结果
				}

				if(m_pins_status[i].keepVacTestResult != 0)
				{
					m_pins_status[i].keepVacTestResult = 0;		//清除之前测试结果
				}
			}

			if(m_pins_status[i].vacuum > 3.0)		//负压值为正值表示抽负压异常
			{
				if(!is_vacuum_error[i])
				{
					is_vacuum_error[i] = true;
					status_sync_cell(i+1,SYS_CELL_ALARM_HANDLE,-2400-VACUUM_VALUE_POSITIVE);
					send_pins_alarm(-2400-VACUUM_VALUE_POSITIVE,i+1);
					//同步针床告警掩码
					plc_alarm_mask[i] |= (uint64_t)1 << (VACUUM_VALUE_POSITIVE-1);
				}
			}
			else
			{
				is_vacuum_error[i] = false;
			}
#endif
		}
		else
		{
			m_pins_status[i].is_plc_connect = false;
		}
		m_pins_status[i].is_cell_forbid = false;
		
		// 状态上传（上位机控制数据是否上传）
		//if (Configuration::instance()->need_send_detail_data(cell_no))
		{
			string josn_str = FormatConvert::instance().pins_data_to_json(cell_no, m_pins_status[i]);
			MsgFactory::instance()->pins_to_ext_reply_pusher().send(josn_str);
		}
		
		// 写入log(lbfs/data/目录)
		//string store_josn_str = FormatConvert::instance().pins_data_to_store_str(cell_no, status);
		//LOG_INFO(cell_id_temp, store_josn_str);

#if 0
		// 故障码上报
		bitset<9> error_code(status.err_code);
		bitset<9> last_error(m_pins_status[cell].err_code);
		
		for (uint8_t j=0; j<error_code.size(); ++j)
		{
			if ((error_code[j] != 0) && (error_code[j] != last_error[j]))
			{
				// 推送告警信息
				int ex_code = -1300 - j;
				string josn_str = FormatConvert::instance().alarm_to_string(ex_code, cell_no);
				MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
				LOGFMTD("cell=%d,  cell_no=%d", cell, cell_no);
			}
		}
#endif
	}
}


void PinsWorker::plc_alarm_mask_proc(int cellNo,std::vector<alarm_erc_t> &alarm_erc_vector)
{
	for(uint32 i = 0; i < alarm_erc_vector.size(); i++)
	{
		int ercIdx = alarm_erc_vector[i].alarm_erc - 1;
		
		if(alarm_erc_vector[i].is_happen)				//告警发生
		{
			if((plc_alarm_mask[cellNo-1] & ((uint64_t)1 << ercIdx)) == 0)
			{
				plc_alarm_mask[cellNo-1] |= (uint64_t)1 << ercIdx;
				send_pins_alarm(-2401-ercIdx,cellNo);
			}
		}
		else
		{
			if((plc_alarm_mask[cellNo-1] & ((uint64_t)1 << ercIdx)) != 0)
			{
				plc_alarm_mask[cellNo-1] &= ~((uint64_t)1 << ercIdx);
			}
		}
	}
}

void PinsWorker::plc_alarm_handle()
{
	alarm_erc_t m_alarm_erc;
	vector<alarm_erc_t> cab_alarm_erc;
	
	//急停告警处理
	if(m_pins_work.sudden_stop_alarm.face_panel_ctl_stop.is_happen \
	|| m_pins_work.sudden_stop_alarm.HMI_ctl_stop.is_happen)
	{
		if(!is_sudden_stop)
		{
			is_sudden_stop = true;
			status_sync_cell(0xFF,SYS_CELL_ALARM_HANDLE,-2400-FACE_PANEL_STOP);
			LOGFMT_DEBUG(cell_id_temp,"针床紧急停止");	
		}
	}
	else
	{
		is_sudden_stop = false;
	}

	//PLC整柜告警项
	m_alarm_erc = m_pins_work.sudden_stop_alarm.face_panel_ctl_stop;	//面板按键急停
	cab_alarm_erc.push_back(m_alarm_erc);

	m_alarm_erc = m_pins_work.sudden_stop_alarm.HMI_ctl_stop;			//手持设备急停
	cab_alarm_erc.push_back(m_alarm_erc);

	//m_alarm_erc = m_pins_work.cab_smoke_alarm;						//电源柜烟雾
	//cab_alarm_erc.push_back(m_alarm_erc);

	m_alarm_erc = m_pins_work.sys_abnormal_alarm.AC1_over_voltage;		//AC1过压
	cab_alarm_erc.push_back(m_alarm_erc);

	m_alarm_erc = m_pins_work.sys_abnormal_alarm.AC1_low_voltage;		//AC1欠压
	cab_alarm_erc.push_back(m_alarm_erc);

	m_alarm_erc = m_pins_work.sys_abnormal_alarm.AC1_over_current;		//AC1过流
	cab_alarm_erc.push_back(m_alarm_erc);

	m_alarm_erc = m_pins_work.sys_abnormal_alarm.AC2_over_voltage;		//AC2过压
	cab_alarm_erc.push_back(m_alarm_erc);

	m_alarm_erc = m_pins_work.sys_abnormal_alarm.AC2_low_voltage;		//AC2欠压
	cab_alarm_erc.push_back(m_alarm_erc);

	m_alarm_erc = m_pins_work.sys_abnormal_alarm.AC2_over_current;		//AC2过流
	cab_alarm_erc.push_back(m_alarm_erc);

	plc_alarm_mask_proc(1,cab_alarm_erc);
	
	//PLC分库位告警项
	for(int i = 0; i < PINS_CELLS_NUM; i++)
	{
		vector<alarm_erc_t> cell_alarm_erc;
		
		m_alarm_erc = m_pins_work.cell_smoke_alarm[i].cell_ahead_smoke;		//库位前方烟雾告警
		cell_alarm_erc.push_back(m_alarm_erc);
		m_pins_status[i].alarm_ahead_smoke = m_alarm_erc.is_happen;

		m_alarm_erc = m_pins_work.cell_smoke_alarm[i].cell_rear_smoke;		//库位后方烟雾告警
		cell_alarm_erc.push_back(m_alarm_erc);
		m_pins_status[i].alarm_rear_smoke = m_alarm_erc.is_happen;

		m_alarm_erc = m_pins_work.cell_smoke_alarm[i].cell_ahead_and_rear_smoke;	//库位前方和后方烟雾告警
		cell_alarm_erc.push_back(m_alarm_erc);
		if(m_alarm_erc.is_happen)
		{
			m_pins_status[i].alarm_ahead_smoke = true;
			m_pins_status[i].alarm_rear_smoke = true;
		}

		m_alarm_erc = m_pins_work.cell_smoke_alarm[i].cell_24V_abnormal;		//库位前方烟雾告警
		cell_alarm_erc.push_back(m_alarm_erc);

		m_alarm_erc = m_pins_work.cell_action_alarm[i].tray_extend_timeout;		//针床压合超时
		cell_alarm_erc.push_back(m_alarm_erc);
		m_alarm_erc = m_pins_work.cell_action_alarm[i].tray_retract_timeout;	//针床脱开超时
		cell_alarm_erc.push_back(m_alarm_erc);
		m_alarm_erc = m_pins_work.cell_action_alarm[i].firedoor_off_timeout;	//消防门关闭超时
		cell_alarm_erc.push_back(m_alarm_erc);
		m_alarm_erc = m_pins_work.cell_action_alarm[i].firedoor_on_timeout;		//消防门开启超时
		cell_alarm_erc.push_back(m_alarm_erc);
		m_alarm_erc = m_pins_work.cell_action_alarm[i].fanspeed_abnormal;		//库位风扇转速异常
		cell_alarm_erc.push_back(m_alarm_erc);

		m_alarm_erc = m_pins_work.cell_action_alarm[i].tray_reverse;			//托盘反向
		cell_alarm_erc.push_back(m_alarm_erc);

		plc_alarm_mask_proc(i+1,cell_alarm_erc);
	}
}
#endif

#if 0
#if defined(PLC_JYT)
void PinsWorker::batteryFormation_logic(int cell)
{
	// 开始化成工艺
	if (m_bf_sm[cell].start_cmd)
	{
		if (!m_bf_sm[cell].is_started)
		{		
			m_bf_sm[cell].is_started = true;
			m_bf_sm[cell].logic_no   = 0;
		}
	}
	else
	{
		if (!m_bf_sm[cell].is_started)  return;
	}
	
	// 终止化成工艺
	if (m_bf_sm[cell].abort_cmd)
	{
		m_bf_sm[cell].abort_cmd = false;
		batteryFormation_status_notify(cell+1, -2428);
		m_bf_sm[cell].logic_no  = 15;
	}
	
	// 执行化成工艺
	int step_no = m_pins[cell].get_step_no();
	m_pins_status[cell].step_no = step_no;
	
	switch (m_bf_sm[cell].logic_no)
	{
		// 工艺等待
		case 0:
		{
			if (m_bf_sm[cell].c_temp_alarm)
			{
				string josn_str = FormatConvert::instance().alarm_to_string(-2303, cell+1);
				MsgFactory::instance()->pins_alarm_pusher().send(josn_str);

				status_sync_cell(cell+1, SYS_PINS_TECH_FINISH); 	// 通知库位线程工艺完成
				m_bf_sm[cell].logic_no = 0xFF;
			}
			else
			{
				if (step_no > 2)
				{
					string josn_str = FormatConvert::instance().alarm_to_string(-2307, cell+1);
					MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
					
					status_sync_cell(cell+1, SYS_PINS_TECH_FINISH); 	// 通知库位线程工艺完成
					m_bf_sm[cell].logic_no = 0xFF;
				}
				else
				{
					m_bf_sm[cell].logic_no = 1;
				}
			}
			break;
		}
		// 工装参数写入
		case 1:
		{
			if (step_no > 0)
			{	
				m_pins[cell].set_device_type(PINS_TECH_CLEAN);	
				m_pins[cell].set_plc_init(true);
				msleep(500);
				m_pins[cell].set_plc_init(false);
			}
			if (step_no == 1)
			{
				if (m_pins[cell].is_cell_empty())
				{
					status_sync_cell(cell+1, SYS_PINS_TECH_FINISH); 	// 通知库位线程工艺终止
					m_bf_sm[cell].logic_no = 0xFF;
					
					string josn_str = FormatConvert::instance().alarm_to_string(-2306, cell+1);
					MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
				}
				else
				{
					m_pins[cell].set_device_type(m_bf_sm[cell].tech_type);
					m_bf_sm[cell].logic_no = 2;
					batteryFormation_status_notify(cell+1, -2401);
				}
			}
			else if (step_no == 2)
			{
				m_pins[cell].set_device_type(1);
				m_bf_sm[cell].logic_no = 2;
				batteryFormation_status_notify(cell+1, -2401);
			}
			break;
		}
		// 托盘上升
		case 2:
		{
			// 先下发泄漏率测试使能
			if (m_bf_sm[cell].tech_type == PINS_TECH_BATTERY || m_bf_sm[cell].tech_type == PINS_TECH_VAC)
			{
				m_pins[cell].enable_vac_leak(m_bf_sm[cell].vac_leak_test_en);
			}
			// 不同工装，工艺流程不同
			if (m_bf_sm[cell].tech_type == PINS_TECH_BATTERY)
			{
				// 配方第一个工步为负压工步
				if (m_bf_sm[cell].multi_vac_flag)
				{
					m_bf_sm[cell].multi_vac_flag = false;
					m_pins[cell].set_vac_value(m_bf_sm[cell].vac);				
				}
				m_bf_sm[cell].power_step_finish = false;
				m_bf_sm[cell].logic_no = 3;
			}
			else
			{
				m_bf_sm[cell].tech_acq_finish = false;
				m_bf_sm[cell].logic_no = 11;
			}
			// 再控制托盘上升
			m_pins[cell].pins_extend();
			batteryFormation_status_notify(cell+1, -2410);
			break;
		}
		// 状态提示：泄漏率测试
		case 3:
		{
			if (step_no > 10)
			{
				// 关闭泄漏率测试，防止泄漏率测试失败后重复测试
				if (m_bf_sm[cell].vac_leak_test_en)
				{
					m_pins[cell].enable_vac_leak(false);
					batteryFormation_status_notify(cell+1, -2411);
				}
				m_bf_sm[cell].logic_no = 4;
			}
			break;
		}
		// 化成工艺：启动化成
		case 4:
		{
			if (step_no >= 17)
			{
				if (m_bf_sm[cell].vac_leak_test_en)
				{
					batteryFormation_status_notify(cell+1, -2412);
				}
				m_pins[cell].start_vac_seq();
				m_bf_sm[cell].logic_no = 5;
				batteryFormation_status_notify(cell+1, -2413);
			}
			break;
		}
		// 化成工艺：等待保压完成
		case 5:
		{
			if (step_no == 22)
			{
				batteryFormation_status_notify(cell+1, -2415);
				m_bf_sm[cell].logic_no = 7;
			}
			else if (step_no == 23)									// 泄漏率测试失败，结束工艺
			{
				batteryFormation_status_notify(cell+1, -2423);
				m_bf_sm[cell].logic_no = 9;
			}
			break;
		}
//		// 化成工艺：异常终止
//		case 6:
//		{
//			m_pins[cell].stop_vac_seq();
//			m_bf_sm[cell].logic_no = 9;
//			batteryFormation_status_notify(cell+1, -2414);
//			break;
//		}
		// 化成工艺：保压和恒温正常
		case 7:
		{
			status_sync_cell(cell+1, SYS_VAC_FINISH);				// 通知库位线程开始运行工步
			m_bf_sm[cell].logic_no = 8;
			break;
		}
		// 化成工艺：等待工步运行完成
		case 8:
		{
			if (m_bf_sm[cell].power_step_finish)
			{
				m_pins[cell].stop_vac_seq();
				m_bf_sm[cell].logic_no = 9;
				batteryFormation_status_notify(cell+1, -2418);
			}
			else
			{
				// 多段负压工艺
				if (m_bf_sm[cell].multi_vac_flag)
				{
					m_pins[cell].set_vac_value(m_bf_sm[cell].vac);					
					m_bf_sm[cell].is_vac_finish  = false;
					m_bf_sm[cell].multi_vac_flag = false;
					m_bf_sm[cell].logic_no = 14;
					batteryFormation_status_notify(cell+1, -2426);
				}
			}
			break;
		}
		// 化成工艺：等待泄压完成
		case 9:
		{
			if (((step_no >= 24) && (step_no <= 29)) || ((step_no >= 80) && (step_no <= 89)))
			{
				batteryFormation_status_notify(cell+1, -2419);
				m_pins[cell].pins_retract();						// 托盘下降
				m_bf_sm[cell].logic_no = 10;
			}
			break;
		}
		// 化成工艺：结束
		case 10:
		{
			if (m_pins_status[cell].is_retract)
			{
				batteryFormation_status_notify(cell+1, -2420);
				status_sync_cell(cell+1, SYS_PINS_TECH_FINISH); 	// 通知库位线程工艺完成
				m_bf_sm[cell].logic_no = 0xFF;
			}
			break;
		}
		// 工装：等待PLC工步状态
		case 11:
		{
			int wait_step_no = 0;
			int status_code  = 0;
			
			if (m_bf_sm[cell].tech_type == PINS_TECH_VAC)
			{
				wait_step_no = 36;
				status_code  = -2430;
			}
			else if (m_bf_sm[cell].tech_type == PINS_TECH_PT1000)
			{
				wait_step_no = 64;
				status_code  = -2432;
			}
			else if (m_bf_sm[cell].tech_type == PINS_TECH_CALIBRATION)
			{
				wait_step_no = 90;
				status_code  = -2434;
			}
			
			if (step_no == wait_step_no)
			{
				techTest_status_notify(cell+1, status_code);
				m_bf_sm[cell].logic_no = 12;
			}
			break;
		}
		// 工装：等待上位机通知采集完成
		case 12:
		{
			if (m_bf_sm[cell].tech_acq_finish)
			{
				m_pins[cell].pins_retract();						// 托盘下降
				m_bf_sm[cell].logic_no = 13;
			}
			break;
		}
		// 工装：结束
		case 13:
		{
			if (m_pins_status[cell].is_retract)
			{				
				status_sync_cell(cell+1, SYS_TECH_TEST_FINISH); 	// 通知库位线程工装测试完成
				m_bf_sm[cell].logic_no = 0xFF;
			}
			break;
		}
		// 化成工艺：多段负压
		case 14:
		{
			if (m_pins[cell].is_keep_vac() && (fabs(m_pins_status[cell].vacuum - m_bf_sm[cell].vac) <= PINS_VACCUM_DV))
			{
				status_sync_cell(cell+1, SYS_CELL_VAC_NEXT); 	// 通知库位負壓工步切換
				batteryFormation_status_notify(cell+1, -2427);
				m_bf_sm[cell].logic_no = 8;
			}
			break;
		}
		// 化成工艺：终止
		case 15:
		{
			if (m_bf_sm[cell].power_step_finish)
			{
				if (step_no >= 18)
				{
					m_pins[cell].stop_vac_seq();
					batteryFormation_status_notify(cell+1, -2418);
					m_bf_sm[cell].logic_no = 16;
				}
			}
			break;
		}
		// 化成工艺：终止(等待泄压)
		case 16:
		{
			if (step_no >= 80)
			{
				m_pins[cell].pins_retract();
				batteryFormation_status_notify(cell+1, -2429);
				m_bf_sm[cell].logic_no = 0xFF;
			}
			break;
		}
		
		default:
		{
			m_bf_sm[cell].start_cmd  = false;
			m_bf_sm[cell].is_started = false;
			break;
		}
	}
}
#elif defined(PLC_JS_VF)
void PinsWorker::batteryFormation_logic(int cell)
{
	// 开始化成工艺
	if (m_bf_sm[cell].start_cmd)
	{
		if (!m_bf_sm[cell].is_started)
		{		
			m_bf_sm[cell].is_started = true;
			m_bf_sm[cell].logic_no   = 0;
		}
	}
	else
	{
		if (!m_bf_sm[cell].is_started)  return;
	}
	
	// 终止化成工艺
	if (m_bf_sm[cell].abort_cmd)
	{
		m_bf_sm[cell].abort_cmd = false;
		batteryFormation_status_notify(cell+1, -2428);
		m_bf_sm[cell].logic_no  = 10;
	}
	
	// 执行化成工艺	
	switch (m_bf_sm[cell].logic_no)
	{
		// 针床状态检测
		case 0:
		{
			bool is_check_ok = true;
			
			if (m_pins_status[cell].err_code != 0)
			{
				is_check_ok = false;

				string josn_str = FormatConvert::instance().alarm_to_string(-2312, cell+1);
				MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
			}
			else
			{
				if (!m_pins_status[cell].is_auto_mode)
				{
					is_check_ok = false;
					
					string josn_str = FormatConvert::instance().alarm_to_string(-2311, cell+1);
					MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
				}
				else
				{
					if (m_pins_status[cell].is_emergency_stop)
					{
						is_check_ok = false;
						
						string josn_str = FormatConvert::instance().alarm_to_string(-2313, cell+1);
						MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
					}					
					if (m_pins_status[cell].is_palletizer_entry)
					{
						is_check_ok = false;
						
						string josn_str = FormatConvert::instance().alarm_to_string(-2314, cell+1);
						MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
					}
					if (m_pins_status[cell].is_cell_empty)
					{
						is_check_ok = false;
						
						string josn_str = FormatConvert::instance().alarm_to_string(-2315, cell+1);
						MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
					}
				}
			}
			
			if (!is_check_ok)
			{
				status_sync_cell(cell+1, SYS_PINS_TECH_FINISH); 	// 通知库位线程工艺完成
				m_bf_sm[cell].logic_no = 0xFF;
			}
			else
			{
				m_bf_sm[cell].logic_no = 1;
			}
			break;
		}
		// 托盘压合
		case 1:
		{
			// 再控制托盘上升
			m_pins[cell].pins_extend();
			batteryFormation_status_notify(cell+1, -2410);			
			m_bf_sm[cell].logic_no = 2;
			break;
		}
		// 泄漏率测试
		case 2:
		{
			// 泄漏率测试
			if (m_bf_sm[cell].vac_leak_test_en)
			{
				batteryFormation_status_notify(cell+1, -2411);
				m_bf_sm[cell].logic_no = 3;
			}
			else
			{
				m_bf_sm[cell].logic_no = 4;
			}
			break;
		}
		// 化成工艺：启动化成
		case 4:
		{
			m_pins[cell].close_minus_vac();
			if (m_bf_sm[cell].vac < -70.0)							// 开高真空
			{
				m_pins[cell].close_low_vac();
				m_pins[cell].open_high_vac();
			}
			else
			{
				m_pins[cell].close_high_vac();						// 开低真空
				m_pins[cell].open_low_vac();
			}
			batteryFormation_status_notify(cell+1, -2413);
			status_sync_cell(cell+1, SYS_VAC_FINISH);				// 通知库位线程开始运行工步
			m_bf_sm[cell].logic_no = 5;
			break;
		}
		// 化成工艺：等待保压完成
		case 5:
		{
			if (m_pins_status[cell].vacuum < -30.0)
			{
				status_sync_cell(cell+1, SYS_VAC_FINISH);			// 通知库位线程开始运行工步
				m_bf_sm[cell].logic_no = 6;
			}
			break;
		}
		// 化成工艺：等待工步运行完成
		case 6:
		{
			if (m_bf_sm[cell].power_step_finish)
			{
				m_pins[cell].close_high_vac();						// 泄负压
				m_pins[cell].close_low_vac();
				m_pins[cell].open_minus_vac();
				batteryFormation_status_notify(cell+1, -2418);
				m_bf_sm[cell].logic_no = 8;
			}
			else
			{
				// 多段负压工艺
				if (m_bf_sm[cell].multi_vac_flag)
				{
					if (m_bf_sm[cell].vac < -70.0)					// 开高真空
					{
						m_pins[cell].close_low_vac();
						m_pins[cell].open_high_vac();
					}
					else
					{
						m_pins[cell].close_high_vac();				// 开低真空
						m_pins[cell].open_low_vac();
					}
					m_bf_sm[cell].is_vac_finish  = false;
					m_bf_sm[cell].multi_vac_flag = false;
					m_bf_sm[cell].logic_no = 7;
					batteryFormation_status_notify(cell+1, -2426);
				}
			}
			break;
		}
		// 化成工艺：多段负压
		case 7:
		{
			if ((m_bf_sm[cell].vac < -30.0) && (fabs(m_pins_status[cell].vacuum - m_bf_sm[cell].vac) <= PINS_VACCUM_DV))
			{
				status_sync_cell(cell+1, SYS_CELL_VAC_NEXT); 		// 通知库位負壓工步切換
				batteryFormation_status_notify(cell+1, -2427);
				m_bf_sm[cell].logic_no = 6;
			}
			break;
		}
		// 化成工艺：等待泄压完成
		case 8:
		{
			if (m_pins_status[cell].vacuum > -5.0)
			{
				batteryFormation_status_notify(cell+1, -2419);
				m_pins[cell].pins_retract();						// 托盘下降
				m_bf_sm[cell].logic_no = 9;
			}
			break;
		}
		// 化成工艺：结束
		case 9:
		{
			if (m_pins_status[cell].is_retract)
			{
				batteryFormation_status_notify(cell+1, -2420);
				status_sync_cell(cell+1, SYS_PINS_TECH_FINISH); 	// 通知库位线程工艺完成
				m_bf_sm[cell].logic_no = 0xFF;
			}
			break;
		}
		// 化成工艺：终止
		case 10:
		{
			if (m_bf_sm[cell].power_step_finish)
			{
				m_pins[cell].close_high_vac();
				m_pins[cell].close_low_vac();
				m_pins[cell].open_minus_vac();
				batteryFormation_status_notify(cell+1, -2418);
				m_bf_sm[cell].logic_no = 8;
			}
			break;
		}
		
		default:
		{
			m_bf_sm[cell].start_cmd  = false;
			m_bf_sm[cell].is_started = false;
			break;
		}
	}
}
#endif
#endif

#if 0
void PinsWorker::cellCabinet_tempDetect(int cell_no, Pins_Status_Data_t &status)
{
#ifdef ZHONGHANG_SYSTEM_PROTECTION
	static int cellTempAlarmCnt[7] = { 0, 0, 0, 0, 0, 0, 0 };				// 库位环境温度告警计数

	// 电池环境温度预警
	int cellTempAlarmNum = 0;

	for (int j=0; j<7; ++j)
	{
		if (status.temp[j] >= m_cell_BTUL)
		{
			cellTempAlarmNum++;
			cellTempAlarmCnt[j]++;
			string josn_str = FormatConvert::instance().alarm_to_string(-2503, cell_no);
			MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
			break;
		}
	}
	// 同时出现2个及以上温度告警, 执行保护操作
	if (cellTempAlarmNum >= 2)
	{
		system_protection(cell_no, true);
	}
	else
	{
		// 任一温度告警持续>=3秒, 执行保护操作
		for (int j=0; j<7; ++j)
		{
			if (cellTempAlarmCnt[j] >= 3)
			{
				system_protection(cell_no, true);
				memset(cellTempAlarmCnt, 0, sizeof(cellTempAlarmCnt));			// 复位
				break;
			}
		}
	}
#endif
}

// 库位告警处理
void PinsWorker::cell_alarm_handler(int cell_no, int err_code)
{
	// 推送告警信息给上位机
	string josn_str = FormatConvert::instance().alarm_to_string(err_code, cell_no);
	MsgFactory::instance()->pins_alarm_pusher().send(josn_str);

	// 告警码同步给库位线程
	switch (err_code)
	{
		case -1501:
		case -1540:
		case -1541:
		case -1566:
		case -1567:
			status_sync_cell(cell_no, SYS_CELL_ALARM_HANDLE, err_code);
			break;
	}
}

void PinsWorker::system_protection(int cell_no, bool retract)
{	
    char filter[32] = {'\0'};
	
	// 先停止通道输入（工步停止）
	Cmd_Cell_Msg_t msg_struct = Cmd_Cell_Msg_t();
	msg_struct.func_code = CMD_RUN_CONTROL;
	for (int i = 0; i < MAX_CHANNELS_NBR; i++)
	{
		msg_struct.cell_msg.pow.sel_chans[i] = 1;
	}
	msg_struct.cell_msg.pow.data.func_data.func_code = STEP_STOP;
	msg_struct.cell_msg.pow.data.func_data.value = 1;
	
	// 所有库
	if (cell_no == 0xFF)
	{
		for (int i=0; i<MAX_CELLS_NBR; ++i)
		{
			sprintf(filter, TOPIC_CELL_FORMAT_STR, i+1);		
			MsgFactory::instance()->pins_puber().send(filter, msg_struct, ZMQ_DONTWAIT);
		}
	}
	// 指定库位
	else
	{
		sprintf(filter, TOPIC_CELL_FORMAT_STR, cell_no);		
		MsgFactory::instance()->pins_puber().send(filter, msg_struct, ZMQ_DONTWAIT);
	}
}

#if 0
void PinsWorker::vaccum_wave_protect()
{
	for (int cell=0; cell<PINS_CELLS_NUM; cell++)
	{
		// 波动保护条件检测
		if (m_bf_sm[cell].vac < -30.0)
		{
			if (fabs(m_pins_status[cell].vacuum - m_bf_sm[cell].vac) >= m_bf_sm[cell].vac_wp_dv)
			{
				if (!m_bf_sm[cell].is_vac_wp_alarm)
				{
					m_bf_sm[cell].vac_wp_triger_tm = time(NULL);
				}
				m_bf_sm[cell].is_vac_wp_alarm = true;
			}
			else
			{
				m_bf_sm[cell].is_vac_wp_alarm = false;
			}
		}
		// 执行波动保护动作
		if (m_bf_sm[cell].is_vac_wp_alarm)
		{
			if (((time(NULL) - m_bf_sm[cell].vac_wp_triger_tm) >= m_bf_sm[cell].vac_wp_tm)
				&& !m_bf_sm[cell].is_vac_wp_handle)
			{
				cell_alarm_handler(cell+1, -2310);
				m_bf_sm[cell].is_vac_wp_handle = true;
			}
		}
	}
}
#endif

void PinsWorker::status_sync_auxModule(int cell_no, Pins_Status_Data_t &status)
{
#ifdef AUX_VOLTAGE_ACQ
	// 针床压合状态同步给辅助电压采集盒
	if (status.is_extend || status.is_retract)
	{
		if ((status.is_extend  != m_pins_status[cell_no-1].is_extend) ||
			(status.is_retract != m_pins_status[cell_no-1].is_retract))
		{
			Cmd_Aux_Param_t msg;
			msg.cell_no 		= cell_no;
			msg.fuc_code		= SYS_COMMON_CMD;
			msg.status.sub_code = SYS_PROBE_STATUS_SYNC;
			msg.status.value	= status.is_extend ? 1 : 0;
			MsgFactory::instance()->aux_param_pusher(cell_no).send(msg, ZMQ_DONTWAIT);
		}
	}
#endif
}


// 发送通知消息给上位机
void PinsWorker::batteryFormation_status_notify(int cell_no, int status_code)
{
	string josn_str = FormatConvert::instance().notify_to_string(cell_no, CMD_POST_STATUS_INFO, 1, status_code);
	MsgFactory::instance()->cell_to_ext_reply_pusher(cell_no).send(josn_str);
}

void PinsWorker::techTest_status_notify(int cell_no, int status_code)
{
	string josn_str = FormatConvert::instance().notify_to_string(cell_no, CMD_POST_STATUS_INFO, 2, status_code);
	MsgFactory::instance()->cell_to_ext_reply_pusher(cell_no).send(josn_str);
}
#endif

