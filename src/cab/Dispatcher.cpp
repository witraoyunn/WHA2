#include <map>
#include <cstring>
#include <assert.h>
#include <fstream>
#include <iostream>

#include "zmq.h"
#include "json/json.h"
#include "log4z.h"
#include "utility.h"
#include "Dispatcher.h"
#include "GNMessager.h"
#include "FormatConvert.h"
#include "Configuration.h"

using namespace std;

Dispatcher::Dispatcher()
{
}


Dispatcher::~Dispatcher()
{
}

Dispatcher* Dispatcher::instance()
{
    static Dispatcher obj;
    return &obj;
}


void Dispatcher::str_funccode_dispatch(Json::Value &msg_root)
{
#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS)) 
	Cmd_Pins_Data_t pin_data = Cmd_Pins_Data_t();
#endif
	Channel_Data_t pow_data = Channel_Data_t();

	string str_func = msg_root["FuncCode"].asString();
	m_func_code = 0;
	
#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
	if(str_func == "vacuum_set")
	{
		m_func_code = CELL_VAC_PARA_SET;
		
		pin_data.sub_params[0].func_code = 0;
		pin_data.sub_params[0].value = msg_root["BlockSetValue"].asDouble();

		pin_data.sub_params[1].func_code = 1;
		pin_data.sub_params[1].value = msg_root["BlockPumpTime"].asInt();

		pin_data.sub_params[2].func_code = 2;
		pin_data.sub_params[2].value = msg_root["HoldPressureValue"].asDouble();
		
		pin_data.sub_params[3].func_code = 3;
		pin_data.sub_params[3].value = msg_root["HoldPressureStableTime"].asInt();

		pin_data.sub_params[4].func_code = 4;
		pin_data.sub_params[4].value = msg_root["HoldPressureLeakRate"].asDouble();
		
		pin_data.sub_params[5].func_code = 5;
		pin_data.sub_params[5].value = msg_root["HoldPressureKeepTime"].asInt();

		pin_data.sub_params[6].func_code = 6;
		pin_data.sub_params[6].value = msg_root["HoldPressureTimeout"].asInt();
	}
	else if(str_func == "start_pump_vacuum")
	{
		m_func_code = CELL_START_VACUUM;

		pin_data.sub_params[0].func_code = 0;
		pin_data.sub_params[0].value = msg_root["PumpVac"].asDouble();

		pin_data.sub_params[1].func_code = 1;
		pin_data.sub_params[1].value = msg_root["PumpTimeout"].asInt();
	}
	else if(str_func == "stop_pump_vacuum")
	{
		m_func_code = CELL_STOP_VACUUM;
	}
	else if(str_func == "start_release_vacuum")
	{
		m_func_code = CELL_OPEN_DRAIN_VACUUM;

		pin_data.sub_params[0].func_code = 0;
		pin_data.sub_params[0].value = msg_root["LeakVac"].asDouble();

		pin_data.sub_params[1].func_code = 1;
		pin_data.sub_params[1].value = msg_root["LeakTimeout"].asInt();
	}
	else if(str_func == "stop_release_vacuum")
	{
		m_func_code = CELL_CLOSE_DRAIN_VACUUM;
	}
	else if(str_func == "tray_up")
	{
		m_func_code = CELL_TRAY_UP;
	}
	else if(str_func == "tray_down")
	{
		m_func_code = CELL_TRAY_DOWN;
	}
	else if(str_func == "leak_rate_test")
	{
		m_func_code = CELL_LEAK_RATE_TEST_MANUAL;

		pin_data.sub_params[0].func_code = 0;
		pin_data.sub_params[0].value = msg_root["HoldPressureValue"].asDouble();

		pin_data.sub_params[1].func_code = 1;
		pin_data.sub_params[1].value = msg_root["HoldPressureStableTime"].asInt();

		pin_data.sub_params[2].func_code = 2;
		pin_data.sub_params[2].value = msg_root["HoldPressureLeakRate"].asDouble();
		
		pin_data.sub_params[3].func_code = 3;
		pin_data.sub_params[3].value = msg_root["HoldPressureKeepTime"].asInt();

		pin_data.sub_params[4].func_code = 4;
		pin_data.sub_params[4].value = msg_root["HoldPressureTimeout"].asInt();
	}
	else if(str_func == "auto_mode")
	{
		m_func_code = CELL_SET_AUTO_MODE;
	}
	else if(str_func == "maintenance_mode")
	{
		m_func_code = CELL_SET_MANUAL_MODE;
	}
	else if(str_func == "calibration_mode")
	{
		m_func_code = CELL_CALIBRATE_MODE;
	}
	else if(str_func == "start_fan")
	{
		m_func_code = CELL_OPEN_FIRE_FAN;
	}
	else if(str_func == "stop_fan")
	{
		m_func_code = CELL_CLOSE_FIRE_FAN;
	}
	else if(str_func == "fire_door_up")
	{
		m_func_code = CELL_FIRE_DOOR_UP;
	}
	else if(str_func == "fire_door_down")
	{
		m_func_code = CELL_FIRE_DOOR_DOWN;
	}
	else if(str_func == "start_block_test")
	{
		m_func_code = CELL_BLOCK_TEST_MANUAL;
		
		pin_data.sub_params[0].func_code = 0;
		pin_data.sub_params[0].value = msg_root["BlockSetValue"].asDouble();

		pin_data.sub_params[1].func_code = 1;
		pin_data.sub_params[1].value = msg_root["PumpTime"].asInt();
		//pin_data.sub_params[1].value = msg_root["PumpTime"].asDouble();
	}
	else if(str_func == "red_light")
	{
		m_func_code = CELL_RED_LIGHT;
	}
	else if(str_func == "yellow_light")
	{
		m_func_code = CELL_YELLOW_LIGHT;
	}
	else if(str_func == "green_light")
	{
		m_func_code = CELL_GREEN_LIGHT;
	}
	else if(str_func == "buzzer")
	{
		m_func_code = CELL_BUZZER;
	}
	else if(str_func == "otp1")
	{
		m_func_code = CELL_OPT1;
	}
	else if(str_func == "otp2")
	{
		m_func_code = CELL_OPT2;
	}
	else if(str_func == "plc_alarm_reset")
	{
		m_func_code = CELL_CLEAR_WARNNING;
	}
	else if(str_func == "24V_on")
	{
		m_func_code = CELL_POWER_ON;
	}
	else if(str_func == "24V_off")
	{
		m_func_code = CELL_POWER_OFF;
	}
	else if(str_func == "register_read")
	{
		m_func_code = PLC_REG_READ;

		pin_data.sub_params[0].func_code = 0;
		pin_data.sub_params[0].value = msg_root["Addr"].asInt();

		pin_data.sub_params[1].func_code = 1;
		pin_data.sub_params[1].value = msg_root["OffsetBit"].asInt();
	}
	else if(str_func == "register_write")
	{
		m_func_code = PLC_REG_WRITE;
		pin_data.sub_params[0].func_code = 0;
		pin_data.sub_params[0].value = msg_root["Addr"].asInt();

		pin_data.sub_params[1].func_code = 1;
		pin_data.sub_params[1].value = msg_root["OffsetBit"].asInt();

		pin_data.sub_params[2].func_code = 2;
		pin_data.sub_params[2].value = msg_root["Date"].asInt();
	}
	else if(str_func == "plc_sw_version")
	{
		m_func_code = PLC_SW_VERSION;
	}
	
	if(m_func_code != 0)
	{
		m_send_struct(pin_data);
		return;
	}
#endif

	if(str_func == "pass_through_start")
	{
		m_func_code = CMD_TRANSMIT_ENABLE;
	}
	else if(str_func == "pass_through_data")
	{
		m_func_code = CMD_TRANSMIT_DATA;

		memset(m_sel_chans,0,sizeof(m_sel_chans));
		Json::Value json_chs = msg_root["Channels"];
		for (uint8_t i = 0; i < json_chs.size(); i++)
		{
		    int chan_no = json_chs[i].asInt();
		    assert(chan_no >= 1 && chan_no <= MAX_CHANNELS_NBR);
		    m_sel_chans[chan_no-1] = 1;
		}

		Json::Value data = msg_root["Data"];
		pow_data.subList_len = data.size();
		for (int i = 0; i < pow_data.subList_len; i++)
		{
			pow_data.subList[i].func_code = i+1;
			pow_data.subList[i].value	  = data[i].asInt();
		}
	}
	else if(str_func == "pass_through_stop")
	{
		m_func_code = CMD_TRANSMIT_DISABLE;
	}

	if(m_func_code != 0)
	{
		m_send_struct(pow_data);
	}
}


#define CMD_CLEAR_WARNING  0x14
void Dispatcher::publish(std::string json_msg)
{
	// JSON协议解析
	Json::Reader reader;
	Json::Value v_msg_root;

	bool re = reader.parse(json_msg, v_msg_root, false);
	if (!re)
	{
        LOGFMTE("The string is not json format. string: %s", json_msg.c_str());
        throw -300;
	}
	
#if defined(AWH_PRECHARGE_SYS)	
	//新增协议解析格式
	if(v_msg_root["DataType"].type() != Json::nullValue )
	{
		m_cell_no = v_msg_root["CellNo"].asInt();
		string datatype = v_msg_root["DataType"].asString();
		LOGFMTD("Cell:%d datatype:%s", m_cell_no, datatype.c_str());
		
		if(! strcasecmp(datatype.c_str(),"Write") )
		{
			Json::Value data = v_msg_root["Data"];
			Cmd_Pins_Data_t pin_data = Cmd_Pins_Data_t();

			//托盘上升
			if(data["TrayMoveUp"].asInt())
			{
				m_send_struct_PLC(pin_data,CELL_PROBE_EXTEND);
				LOGFMTD("TrayMoveUp:%d", data["TrayMoveUp"].asInt());
			}
			//托盘下降
			if(data["TrayMoveDown"].asInt())
			{
				m_send_struct_PLC(pin_data,CELL_PROBE_RETRACT);
				LOGFMTD("TrayMoveDown:%d", data["TrayMoveDown"].asInt());
			}
			//工装取电开
			if(data["ToolPowerOn"].asInt())
			{
				m_send_struct_PLC(pin_data,CELL_POWER_ON);
				LOGFMTD("ToolPowerOn:%d", data["ToolPowerOn"].asInt());
			//工装取电关
			}
			if((data["ToolPowerOn"].asInt()==0) && data["ToolPowerOn"].type() != Json::nullValue )
			{
				m_send_struct_PLC(pin_data,CELL_POWER_OFF);
				LOGFMTD("ToolPowerOn:%d", data["ToolPowerOn"].asInt());
			}
			//冷风扇开
			if(data["CoolFanRun"].asInt())
			{
				m_send_struct_PLC(pin_data,CELL_COOL_FAN_ON);
				LOGFMTD("CoolFanRun:%d", data["CoolFanRun"].asInt());
			//冷风扇关
			}
			if((data["CoolFanRun"].asInt()==0) && data["CoolFanRun"].type() != Json::nullValue)
			{
				m_send_struct_PLC(pin_data,CELL_COOL_FAN_OFF);
				LOGFMTD("CoolFanRun:%d", data["CoolFanRun"].asInt());
			}
			//抽风机开
			if(data["AirFanRun"].asInt())
			{
				m_send_struct_PLC(pin_data,CELL_AIR_FAN_ON);
				LOGFMTD("AirFanRun:%d", data["AirFanRun"].asInt());
			//抽风机关
			}
			if( (data["AirFanRun"].asInt() == 0) && data["AirFanRun"].type() != Json::nullValue)
			{
				m_send_struct_PLC(pin_data,CELL_AIR_FAN_OFF);
				LOGFMTD("AirFanRun:%d", data["AirFanRun"].asInt());
			}
			//清除操作
			if(data["ClearAlarm"].asInt())
			{
				m_send_struct_PLC(pin_data,CELL_CLEAR_WARNNING);
				LOGFMTD("ClearAlarm:%d", data["ClearAlarm"].asInt());
			}
			//初始化操作
			if(data["Load"].asInt())
			{
				m_send_struct_PLC(pin_data,CELL_LOAD);
				LOGFMTD("Load:%d", data["Load"].asInt());
			}
			//自启动操作
			if(data["AutoStart"].asInt())
			{
				m_send_struct_PLC(pin_data,CELL_AUTO_START);
				LOGFMTD("AutoStart:%d", data["AutoStart"].asInt());
			}
			//停止操作
			if(data["Stop"].asInt())
			{
				m_send_struct_PLC(pin_data,CELL_STOP);
				LOGFMTD("Stop:%d", data["Stop"].asInt());
			}
			//设置库位温度告警
			if(data["TempAlarm"].asDouble())
			{
				pin_data.temp = data["TempAlarm"].asDouble();
				m_send_struct_PLC(pin_data,CELL_TEMP_SET);
				LOGFMTD("TempAlarm:%f", data["TempAlarm"].asDouble());
			}
			//反馈托盘信息
			if(data["SetTrayStatus"].asInt())
			{
				pin_data.tray_info = data["SetTrayStatus"].asInt();
				m_send_struct_PLC(pin_data,CELL_SET_TARY_STATUS);
				LOGFMTD("SetTrayStatus:%d", data["SetTrayStatus"].asInt());
			}
			//托盘出库
			if(data["TrayOut"].asInt())
			{
				m_send_struct_PLC(pin_data,CELL_TARY_OUT);
				LOGFMTD("TrayOut:%d", data["TrayOut"].asInt());
			}
		}
	}
	else
#endif
	{
	    m_cell_no = v_msg_root["CellNo"].asInt();
		if(m_cell_no < 1 || m_cell_no > MAX_CELLS_NBR)
		{
			LOGFMTD("CellNo exceed limit : %d", m_cell_no);
		}

		if(v_msg_root["FuncCode"].isInt())		//FuncCode为整形
		{
			m_func_code = v_msg_root["FuncCode"].asInt();

			for (int i = 0; i < MAX_CHANNELS_NBR; i++)
			{
			    m_sel_chans[i] = 0;
			}
			
			Json::Value json_chs = v_msg_root["Channels"];
			for (uint8_t i = 0; i < json_chs.size(); i++)
			{
			    int chan_no = json_chs[i].asInt();
			    if(chan_no >= 1 && chan_no <= MAX_CHANNELS_NBR)
			    {
					m_sel_chans[chan_no-1] = 1;
				}
			}

			Json::Value data = v_msg_root["Data"];

			switch(m_func_code)
			{
				case 10:
					m_func_code = CMD_SET_STEP;		//工步设置
					break;
				case 12:
					m_func_code = CMD_RUN_CONTROL;	//运行状态设置
					break;
			}
			
			// 命令处理
			if (m_func_code < CELL_PROBE_EXTEND)
			{
				switch (m_func_code)
				{
			        case CMD_CONNECT:
			        case CMD_DIS_CONNECT:
			        case CMD_QUERY_PROTECTION:
			        case CMD_QUERY_PROCESS_DATA:
			        case CMD_QUERY_CALIBRATION:
#ifdef PROTOCOL_5V160A
					case CMD_SYSTEM_GET:
					case CMD_SYSTEM_SET:
					case CMD_QUERY_INTERNAL_PROTECTION:
#else
			        case CMD_QUERY_VERSION:
			        case CMD_QUERY_ADDRESS:
			        case CMD_QUERY_AUTO_RUN:
			        case CMD_QUERY_WAVE_SHARP:
			        case CMD_SET_SYS_TIME:
			        case CMD_SET_PARALLEL:
			        case CMD_CLEAR_WARNING:
#endif
			        case CMD_RUN_CONTROL:
			        case ENABLE_RECORD:
			        case DISABLE_RECORD:
			        {
			            Channel_Data_t pow_data = Channel_Data_t();
			            if (data.size() > 1)
			            {
			                //超出异常，只允许有一组参数
			                string josn_str = FormatConvert::instance().alarm_to_string(-604, m_cell_no, -1, m_func_code, -1);
		                    MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
			            }
			            else
			            {
							int i = 0;
			                pow_data.func_data.func_code = data[i].get("SubFuncCode", 0xFF).asInt();
			                pow_data.func_data.value     = data[i].get("Value", 0).asDouble();

							m_send_struct(pow_data);
			            }
#ifdef AUX_VOLTAGE_ACQ
						// 辅助电压采集盒告警清除
						if (m_func_code == CMD_CLEAR_WARNING)
						{		
							Cmd_Aux_Param_t msg;
							msg.cell_no  = m_cell_no;
							msg.fuc_code = 0x0F;					// 对应下位机协议
							msg.size	 = 1;
							msg.data[0].sub_code = 0x62;
							msg.data[0].value	 = 0;
							m_send_struct(msg);
						}
#endif
			            break;
			        }
#ifdef PROTOCOL_5V160A
					case CMD_CLEAR_WARNING:
					{
						Channel_Data_t pow_data = Channel_Data_t();

						m_func_code = CMD_SYSTEM_SET;
						pow_data.func_data.func_code = SUB_CMD_CLEAR_WARNING;
						pow_data.func_data.value = 0;
						
						m_send_struct(pow_data);

						break;
					}
					case CMD_SET_PROTECTION:				// 全局保护设置
			        case CMD_SET_CALIBRATION:				// 老化校准设置
			        case CMD_SET_INTERNAL_PROTECTION:
					{
						data = v_msg_root["GlobalProtect"];
						m_parse_globalProc_JN95(data);
						
						break;
			        }
#else
					case CMD_SET_PROTECTION:				// 全局保护设置
					{
			#ifdef ZHONGHANG_PRJ_NEW
						Channel_Data_t pow_data = Channel_Data_t();
						memset((void *)&pow_data, 0, sizeof(pow_data));
						// 下位机全局合并指令的参数要按子功能码顺序排列
						pow_data.subList_len = data.size();
						for (int i = 0; i < pow_data.subList_len; i++)
						{
							int sub_code = data[i].get("SubFuncCode", 0xFF).asInt();
							int order = sub_code & 0x7F;
							if ((order == 0) || (order > MAX_FUCN_LIST_SIZE))  continue;
							pow_data.subList[order-1].func_code = sub_code;
							pow_data.subList[order-1].value     = m_procet_factor(sub_code, data[i].get("Value", 0xFF).asDouble());
						}
						m_send_struct(pow_data);
			#else
						for (uint8_t i = 0; i < data.size(); i++)
						{
							Channel_Data_t pow_data = Channel_Data_t();
							pow_data.func_data.func_code = data[i].get("SubFuncCode", 0xFF).asInt();
							pow_data.func_data.value = m_procet_factor(pow_data.func_data.func_code, data[i].get("Value", 0xFF).asDouble());
							m_send_struct(pow_data);
						}
			#endif
						break;
					}

					case CMD_SET_CALIBRATION:				// 老化校准设置
			        {
			            for (uint8_t i = 0; i < data.size(); i++)
			            {
			                Channel_Data_t pow_data = Channel_Data_t();
			                pow_data.func_data.func_code = data[i].get("SubFuncCode", 0xFF).asInt();
			                pow_data.func_data.value = m_cal_factor(pow_data.func_data.func_code, data[i].get("Value", 0xFF).asDouble());
			                m_send_struct(pow_data);
			            }
			            break;
			        }
#endif
			        case CMD_SET_STEP:
			        {
						data = v_msg_root["StepData"];
						m_parse_process_cmd_JN95(data);

			            break;
			        }
					
#ifndef PROTOCOL_5V160A
			        case CMD_SET_AUTO_RUN:
			        {
			            Channel_Data_t pow_data = Channel_Data_t();

			            int i = 0;
			            pow_data.subList[i].func_code = data[i].get("SubFuncCode", 0xFF).asInt();
			            pow_data.subList[i].value = data[i].get("Value", 0xFF).asInt();

			            i = 1;
			            pow_data.subList[i].func_code = data[i].get("SubFuncCode", 0xFF).asInt();
			            pow_data.subList[i].value = data[i].get("Value", 0xFF).asInt();

			            m_send_struct(pow_data);
			            break;
			        }
					
			        case CMD_SET_WAVE_SHARP:
			        {
			            Channel_Data_t pow_data = Channel_Data_t();
						for (int i=0; i<2; ++i)
						{
				            pow_data.subList[i].func_code = data[i].get("SubFuncCode", 0xFF).asInt();
							if ((pow_data.subList[i].func_code & 0x80) == 0)
							{
								if (pow_data.subList[i].func_code == 5)
								{
					            	pow_data.subList[i].value = data[i].get("Value", 0xFF).asDouble();
								}
								else
								{
					            	pow_data.subList[i].value = data[i].get("Value", 0xFF).asDouble() * 10000;
								}
							}
						}
			            m_send_struct(pow_data);
			            break;
			        }
					
			        case CMD_SET_SAMPLE:
			        {
			            Channel_Data_t pow_data;
			            //时间条件 unit:S to ms
			            int i = 0;
			            pow_data.subList[i].func_code = (uint8)data[i].get("SubFuncCode", 0xFF).asInt();
			            pow_data.subList[i].value = (uint32)(data[i].get("Value", 0xFF).asInt()*1000);

			            //电压条件 unit:V to 10mV
			            i = 1;
			            pow_data.subList[i].func_code = (uint8)data[i].get("SubFuncCode", 0xFF).asInt();
			            pow_data.subList[i].value = (uint32)(data[i].get("Value", 0xFF).asDouble() * 10000);

			            //电流条件 unit:A to 10mA
			            i = 2;
			            pow_data.subList[i].func_code = (uint8)data[i].get("SubFuncCode", 0xFF).asInt();
			            pow_data.subList[i].value = (uint32)(data[i].get("Value", 0xFF).asDouble() * 10000);

			            m_send_struct(pow_data);
			            break;
			        }

			        case CMD_CHECK_PROTECTION:
			        {
			            Channel_Data_t pow_data;

			            //开启状态
			            int i = 0;
			            pow_data.subList[i].func_code = 0x1;
			            pow_data.subList[i].value = 0x1;


			            //预充电电流值 unit:A to mA
			            i = 1;
			            pow_data.subList[i].func_code = (uint8)data[i-1].get("SubFuncCode", 0xFF).asInt();
			            pow_data.subList[i].value = (uint32)(data[i-1].get("Value", 0xFF).asDouble()*1000);

			            //预充电时间值 unit:s to ms
			            i = 2;
			            pow_data.subList[i].func_code = (uint8)data[i-1].get("SubFuncCode", 0xFF).asInt();
			            pow_data.subList[i].value = (uint32)(data[i-1].get("Value", 0xFF).asDouble()*1000);

			            m_send_struct(pow_data);
			            break;
			        }
#endif

					// 工艺信息绑定
					case CMD_SET_TECH_INFO:
					{
						Json::Value fin_data = v_msg_root["FINData"];
						
						Cmd_Cell_Msg_t msg = Cmd_Cell_Msg_t();
						msg.func_code = m_func_code;
						memset((void *)&msg.tech_info, '\0', sizeof(msg.tech_info));	

						for (int i = 0; i < MAX_CHANNELS_NBR; i++)
						{
						    msg.cell_msg.pow.sel_chans[i] = m_sel_chans[i];
						}
						
						for (uint32_t i=0; i<fin_data.size(); i++)
						{
							int id = fin_data[i].get("SubFuncCode", 0xFF).asInt();
							if ((id >= 1) && (id <= 4))
							{
								strcpy(msg.tech_info[id-1].str, fin_data[i].get("Value", "").asCString());
							}
						}
						
						char topic[32] = {'\0'};
						sprintf(topic, TOPIC_CELL_FORMAT_STR, m_cell_no);
						MsgFactory::instance()->cab_puber().send(topic, msg, ZMQ_DONTWAIT);
						break;
					}
					
					// 电池条码绑定
					case CMD_SET_BATTERY_BARCODE:
					{		
						Cmd_Cell_Msg_t msg = Cmd_Cell_Msg_t();
						msg.func_code = m_func_code;
						memset((void *)&msg.barcode, '\0', sizeof(msg.barcode));
						
						Json::Value snList = v_msg_root["SNData"];
						for (uint8_t i=0; i<snList.size(); ++i)
						{
							int ch_no = snList[i].get("ChannelNo", 0xFF).asInt();
							if (ch_no < 1 || ch_no > MAX_CHANNELS_NBR)  continue;
							
							strcpy(msg.barcode[ch_no - 1].str, snList[i].get("Barcode", "").asCString());
						}
						
						char topic[32] = {'\0'};
						sprintf(topic, TOPIC_CELL_FORMAT_STR, m_cell_no);
						MsgFactory::instance()->cab_puber().send(topic, msg, ZMQ_DONTWAIT);
						break;
					}

					case CMD_QUERY_MC_VERSION:
					{
						Channel_Data_t pow_data = Channel_Data_t();

						m_send_struct(pow_data);
						break;
					}

					case CMD_QUERY_ALARM_INFO:
					{
						Channel_Data_t pow_data = Channel_Data_t();

						Cmd_Pins_Data_t pin_data = Cmd_Pins_Data_t();

						m_func_code = PINS_ALARM_QUERY;				//针床告警查询
			    	    m_send_struct(pin_data);

						m_func_code = CMD_QUERY_ALARM_INFO;
						m_send_struct(pow_data);
						break;
					}

#ifdef AUX_VOLTAGE_ACQ
					// 辅助类参数设置
					case CMD_SET_AUX_PARAM:
					{
						Cmd_Aux_Param_t msg;
						msg.cell_no  = m_cell_no;
						msg.fuc_code = 0x0F;					// 对应下位机协议
						msg.size     = data.size();
						for (int i=0; i<msg.size; ++i)
						{
							msg.data[i].sub_code = data[i].get("SubFuncCode", 0xFF).asInt();
							msg.data[i].value    = data[i].get("Value", 0xFF).asDouble();
						}
			            m_send_struct(msg);
						break;
					}
#endif
					case CMD_AUTO_TECH_CTRL:
					{
						if (data.size() > 0)
						{
							int i = 0;
							Sub_Cmd_Msg_t sub_msg;
							sub_msg.code    = data[i].get("SubFuncCode", 0xFF).asInt();
							sub_msg.value   = data[i].get("Value", 0xFF).asInt();
							sub_msg.step_no = data[i].get("StepNo", 1).asInt();
				            m_send_struct(sub_msg);
						}
						break;
					}

#if 0
					case CMD_FIRE_PROC_SET:
					{
						// 保存配置参数到文件
						ofstream outfile;
						outfile.open(Configuration::instance()->conf_dir_get() + "/fireLogic_" + to_string(m_cell_no) + ".conf", 
							ios::out | ios::trunc);
						outfile.close();
						// 发送给库位线程
						Sub_Cmd_Msg_t sub_msg;
						m_send_struct(sub_msg);
						break;
					}
#endif						
					default:
						//LOGFMTW("Power Command[%d] Warning, Nothing done!", m_func_code);
						break;
				}
			}
			else if (m_func_code < SYS_COMMON_CMD)
			{
				switch (m_func_code)
				{
					case CELL_CLEAR_WARNNING:
				    {
			    	    Cmd_Pins_Data_t pin_data = Cmd_Pins_Data_t();
			    	    m_send_struct(pin_data);
			    	    break;
			    	}

					default:
						break;
				}
#if 0
				switch (m_func_code)
				{
					// PLC相关的指令
			        case CELL_PROBE_EXTEND:
			        case CELL_PROBE_RETRACT:
			        case CELL_POWER_ON:
			        case CELL_POWER_OFF:
			        case CELL_OPEN_HIGH_VACUUM:
			        case CELL_CLOSE_HIGHT_VACUUM:
			        case CELL_OPEN_LOW_VACUUM:
			        case CELL_CLOSE_LOW_VACUUM:
			        case CELL_CLOSE_DRAIN_VACUUM:
			        //case CELL_START_VACUUM:
			        case CELL_OPEN_DRAIN_VACUUM:
			        case CELL_STOP_VACUUM:
			        case CELL_CLEAR_WARNNING:
			#ifdef PLC_JS_VF
					case CELL_SET_AUTO_MODE:
					case CELL_SET_MANUAL_MODE:
					case CELL_OPEN_FIRE_FAN:
					case CELL_CLOSE_FIRE_FAN:
					case CELL_OPEN_FIRE_DOR:
					case CELL_CLOSE_FIRE_DOR:
					case CELL_CLEAN_SMOKE_ALARM:
			#endif
			        case DCR_CMD_PREPARE_INSPECT:
			        case DCR_CMD_ALLOW_EXIT:
			        case DCR_CMD_RESET_STATE:
					case CELL_OPEN_FIRE_SPRAY:
					case CELL_CLOSE_FIRE_SPRAY:
			    	{
			    	    Cmd_Pins_Data_t pin_data = Cmd_Pins_Data_t();
			    	    m_send_struct(pin_data);
			    	    break;
			    	}
					
					case CELL_START_VACUUM:
			    	{
			    	    Cmd_Pins_Data_t pin_data = Cmd_Pins_Data_t();
			#ifdef PLC_JS_VF
						m_send_struct(pin_data);
			#else
					
						m_send_struct(pin_data);
			#endif
			            for (uint8_t i = 0; i < data.size(); i++)
			            {
			                int subfunc = data[i]["SubFuncCode"].asInt();
			                pin_data.vac_params[i] = subfunc;           //通过真空参数传精实PLC高低真空选择和模式
			            }
			    	    m_send_struct(pin_data);
			    	    break;
			    	}
					case CELL_SET_TEMP_THRESHOLD:
					case CELL_CONSTANT_TEMP_SET:
			    	{
			    	    Cmd_Pins_Data_t pin_data = Cmd_Pins_Data_t();
			            for (uint8_t i = 0; i < data.size(); i++)
			            {
			                int subfunc = data[i]["SubFuncCode"].asInt();
			                pin_data.vac_params[subfunc] = data[i]["Value"].asDouble();
			            }
			    	    m_send_struct(pin_data);
			    	    break;
			    	}

			        case CELL_SET_VACUUM_PARAM:
			    	{
			            Cmd_Pins_Data_t pin_data = Cmd_Pins_Data_t();		
			            for (uint8_t i = 0; i < PINS_PARAM_MAX; i++)
			            {
							pin_data.vac_params[i] = -1.0;					// 通过真空类型参数判断下发的是抽真空参数，还是泄真空参数
			            }
			#ifndef PLC_JS_CT
						pin_data.vac_params[PINS_VAC_TAP] = 2.0;			// 未选择阀门
			#endif
			            for (uint8_t i = 0; i < data.size(); i++)
			            {
			                int subfunc = data[i]["SubFuncCode"].asInt();
			                pin_data.vac_params[subfunc] = data[i]["Value"].asDouble();
			            }
			    	    m_send_struct(pin_data);
			    	    break;
			    	}
					
					default:
						LOGFMTW("Power Command[%d] Warning, Nothing done!", m_func_code);
						break;
				}
#endif
			}
		}	
		else if(v_msg_root["FuncCode"].isString())	//FuncCode为字符串
		{
			str_funccode_dispatch(v_msg_root);
		}	
	}
}

/*
void Dispatcher::publish_bts(std::string json_msg)
{
//	// JSON协议解析
//	Json::Reader reader;
//	Json::Value v_msg_root;
//
//	bool re = reader.parse(json_msg, v_msg_root, false);
//	if (!re)
//	{
//        LOGFMTE("The string is not json format. string: %s", json_msg.c_str());
//        throw -300;
//	}
//	
//	BTS_Info_t bts_info;
//
//	string cmd_code = v_msg_root["Code"].asString();
//	strcpy(bts_info.station, v_msg_root["Station"].asCString());
//	strcpy(bts_info.operate, v_msg_root["Operate"].asCString());
//	strcpy(bts_info.date, v_msg_root["Date"].asCString());
//	
//	strcpy(bts_info.traySn, v_msg_root["TraySN"].asCString());
//	strcpy(bts_info.cabinetNo, v_msg_root["CabinetNO"].asCString());
//	strcpy(bts_info.stockSn, v_msg_root["StockSN"].asCString());
//	strcpy(bts_info.stockNo, v_msg_root["StockNO"].asCString());
//
//	if (cmd_code == "107")
//	{
//		Json::Value cells = v_msg_root["Cells"];
//		for (int i=0; i<cells.size(); i++)
//		{
//			strcpy(bts_info.cells[cells[i].], v_msg_root["TraySN"].asInt());
//		}
//	}
}
*/

JN9504_Step_Type_e Dispatcher::get_stepname_value(string stepName)
{
	if(stepName == "Aging")
	{
		return STEP_STANDING;
	}
	else if(stepName == "CC")
	{
		return STEP_CHARGE_CC;
	}
	else if(stepName == "CV")
	{
		return STEP_CHARGE_CV;
	}
	else if(stepName == "CCCV")
	{
		return STEP_CHARGE_CC_CV;
	}
	else if(stepName == "DCC")
	{
		return STEP_DISCHARGE_CC;
	}
	else if(stepName == "DCV")
	{
		return STEP_DISCHARGE_CV;
	}
	else if(stepName == "DCCCV")
	{
		return STEP_DISCHARGE_CC_CV;
	}
	else if(stepName == "DCW")
	{
		return STEP_DISCHARGE_CP;	
	}
	else if(stepName == "CW")
	{
		return STEP_CHARGE_CP;
	}
	else if(stepName == "End")
	{
		return STEP_END;
	}
	else if(stepName == "Loop")
	{
		return STEP_LOOP;
	}
	else if(stepName == "NegPress")
	{
		return STEP_NEG_PRESS;
	}
	else
	{
		return STEP_INVALID;
	}		
}

#ifdef PROTOCOL_5V160A
void Dispatcher::globalProcInfo_resolve(global_protect_t &globalInfo,Channel_Data_t &pow_data)
{
	pow_data.subList_len = 10;	//全局保护共10项
		
	if(globalInfo.OverVoltage > 0.0)
	{
		pow_data.subList[0].func_code  = 1;
		pow_data.subList[0].value = globalInfo.OverVoltage;
	}
	else
	{
		pow_data.subList[0].func_code = 0x81;		//最高位为1表示未生效
		pow_data.subList[0].value = 0.0;
	}

	if(globalInfo.UnderVoltage > 0.0)
	{
		pow_data.subList[1].func_code  = 2;
		pow_data.subList[1].value = globalInfo.UnderVoltage;
	}
	else
	{
		pow_data.subList[1].func_code = 0x82;		//最高位为1表示未生效
		pow_data.subList[1].value = 0.0;
	}

	if(globalInfo.ChargeOverCurrent > 0.0)
	{
		pow_data.subList[2].func_code  = 3;
		pow_data.subList[2].value = globalInfo.ChargeOverCurrent;
	}
	else
	{
		pow_data.subList[2].func_code = 0x83;		//最高位为1表示未生效
		pow_data.subList[2].value = 0.0;
	}

	if(globalInfo.DischargeOverCurrent > 0.0)
	{
		pow_data.subList[3].func_code  = 4;
		pow_data.subList[3].value = globalInfo.DischargeOverCurrent;
	}
	else
	{
		pow_data.subList[3].func_code = 0x84;		//最高位为1表示未生效
		pow_data.subList[3].value = 0.0;
	}

	if(globalInfo.OverCapacity > 0.0)
	{
		pow_data.subList[4].func_code  = 5;
		pow_data.subList[4].value = globalInfo.OverCapacity;
	}
	else
	{
		pow_data.subList[4].func_code = 0x85;		//最高位为1表示未生效
		pow_data.subList[4].value = 0.0;
	}

	if(globalInfo.InitUpperVoltage > 0.0)
	{
		pow_data.subList[5].func_code  = 6;
		pow_data.subList[5].value = globalInfo.InitUpperVoltage;
	}
	else
	{
		pow_data.subList[5].func_code = 0x86;		//最高位为1表示未生效
		pow_data.subList[5].value = 0.0;
	}

	if(globalInfo.InitLowerVoltage > 0.0)
	{
		pow_data.subList[6].func_code  = 7;
		pow_data.subList[6].value = globalInfo.InitLowerVoltage;
	}
	else
	{
		pow_data.subList[6].func_code = 0x87;		//最高位为1表示未生效
		pow_data.subList[6].value = 0.0;
	}

	if(globalInfo.ContactTestTime > 0)
	{
		pow_data.subList[7].func_code  = 8;
		pow_data.subList[7].value = globalInfo.ContactTestTime;
	}
	else
	{
		pow_data.subList[7].func_code = 0x88;		//最高位为1表示未生效
		pow_data.subList[7].value = 0.0;
	}

	if(globalInfo.ContactTestCurrent > 0.0)
	{
		pow_data.subList[8].func_code  = 9;
		pow_data.subList[8].value = globalInfo.ContactTestCurrent;
	}
	else
	{
		pow_data.subList[8].func_code = 0x89;		//最高位为1表示未生效
		pow_data.subList[8].value = 0.0;
	}

	if(globalInfo.ContactTestDeltaVoltage > 0.0)
	{
		pow_data.subList[9].func_code  = 0x0A;
		pow_data.subList[9].value = globalInfo.ContactTestDeltaVoltage;
	}
	else
	{
		pow_data.subList[9].func_code = 0x8A;		//最高位为1表示未生效
		pow_data.subList[9].value = 0.0;
	}
}

void Dispatcher::workstepInfo_resolve(work_step_recipe_t &stepInfo,Channel_Data_t &pow_data)
{	
	Ch_Step_Data_t step_data_struct;
	Step_Data_t    step_data;
	Step_Param_t   general_data;

	memset((void*)&general_data,0,sizeof(Step_Param_t));
	step_data_struct.step_nbr  = stepInfo.StepNo;				// 工步序号

	string stepNameStr = stepInfo.StepName;
	JN9504_Step_Type_e process_name = get_stepname_value(stepNameStr);
	step_data_struct.step_type = process_name;					// 工步名称
	//if(stepNameStr == "Loop")
	//{
	//	step_data_struct.step_type = STEP_END;
	//}
	
	// 取额定参数， 额定参数类型与工步类型匹配
	for(int i = 0; i < 3; i++)
	{
		general_data.spec_data[i].func_code = PARAM_INVALID;
	}

	if(stepInfo.RegularVoltage > 0.0)
	{
		general_data.spec_data[0].func_code = PARAM_V;
		general_data.spec_data[0].value 	= stepInfo.RegularVoltage;
	}

	if(stepInfo.RegularCurrent > 0.0)
	{
		general_data.spec_data[1].func_code = PARAM_I;
		general_data.spec_data[1].value 	= stepInfo.RegularCurrent;
	}

	for(int i = 0; i < 4; i++)
	{
		general_data.abort_data[i].func_code = PARAM_INVALID;
    }

	int level = stepInfo.TimeAbortPrio;
	if(level < 5 && level > 0)
	{
		general_data.abort_data[0].func_code = PARAM_T;
		double timeMin = stepInfo.AbortMinute;
		double timeSec = stepInfo.AbortSecond;
		general_data.abort_data[0].value	 = timeMin*60.0 + timeSec;
		general_data.abort_level[0] 		 = level;
	}
	
	level = stepInfo.VoltageAbortPrio;
	if(level < 5 && level > 0)
	{
		general_data.abort_data[1].func_code = PARAM_V;
		general_data.abort_data[1].value	 = stepInfo.AbortVoltage;
		general_data.abort_level[1] 		 = level;
	}
	
	level = stepInfo.CurrentAbortPrio;
	if(level < 5 && level > 0)
	{
		general_data.abort_data[2].func_code = PARAM_I;
		general_data.abort_data[2].value	 = stepInfo.AbortCurrent;
		general_data.abort_level[2]		 	 = level;
	}

	level = stepInfo.CapacityAbortPrio;
	if(level < 5 && level > 0)
	{
		general_data.abort_data[3].func_code = PARAM_C;
		general_data.abort_data[3].value	 = stepInfo.AbortCapacity;
		general_data.abort_level[3] 		 = level;
	}

	for(int i = 0; i < 6; i++)
	{
		general_data.protection[i].func_code = PARAM_INVALID;
	}

	if(stepInfo.stepProtect.UpperVoltage > 0.0)
	{
		general_data.protection[0].func_code = 1;
		general_data.protection[0].value = stepInfo.stepProtect.UpperVoltage;
	}
	if(stepInfo.stepProtect.LowerVoltage > 0.0)
	{
		general_data.protection[1].func_code = 2;
		general_data.protection[1].value = stepInfo.stepProtect.LowerVoltage;
	}
	if(stepInfo.stepProtect.OverCurrent > 0.0)
	{
		general_data.protection[2].func_code = 3;
		general_data.protection[2].value = stepInfo.stepProtect.OverCurrent;
	}
	if(stepInfo.stepProtect.OverCapacity > 0.0)
	{
		general_data.protection[3].func_code = 4;
		general_data.protection[3].value = stepInfo.stepProtect.OverCapacity;
	}
	if(stepInfo.stepProtect.VoltageOverDiff > 0.0)
	{
		general_data.protection[4].func_code = 5;
		general_data.protection[4].value = stepInfo.stepProtect.VoltageOverDiff;
	}
	if(stepInfo.stepProtect.CurrentOverDiff > 0.0)
	{
		general_data.protection[5].func_code = 6;
		general_data.protection[5].value = stepInfo.stepProtect.CurrentOverDiff;
	}
	
	step_data.general_data     = general_data;
	step_data_struct.step_data = step_data;
	pow_data.step_data         = step_data_struct;

}

void Dispatcher::do_save_globalProtect(Json::Value &data)
{
	XmlProcAPI globalProfile; 
	global_protect_t newGlobalPro;
	
	//要与global_protect_t成员顺序一致
	string float_items[] = {"OverVoltage", "UnderVoltage","VoltageFluctuateRange","DischargeOverCurrent","ChargeOverCurrent",
	"CurrentFluctuateRange","ContactResistance","LoopResistance","BatteryLowerTemp","BatteryOverTemp",
	"BatteryUltraTemp","BatteryUltraHighTemp","BatteryTempSmallFluctuateRange","BatteryTempLargeFluctuate","BatteryTempSuddenRise",
	"CellLowerTemp","CellUltraTemp","CellUltraHighTemp","OverCapacity", "TotalCapacityProtect",
	"InitUpperVoltage","InitLowerVoltage","ContactTestCurrent","ContactTestDeltaVoltage","AuxVoltage",
	"EndTotalCapacityUpper","EndTotalCapacityLower","VacuumFluctuate"};
	
	string int_items[] = {"VoltageFluctuateCount","CurrentFluctuateCount","BatteryTempSmallFluctuateCount","ContactTestTime"};

	char fl_path[128];
	char fl_buff[64];

	float *p_flt_global = (float*)&newGlobalPro.OverVoltage;
	for(int i = 0; i < 28; i++)
	{
		if (data.isMember(float_items[i]))
		{
			*(p_flt_global+i) = data[float_items[i].c_str()].asDouble();
			if(i == 19 || i == 25 || i == 27)
			{
				if(*(p_flt_global+i) == 0.0)	//总容量限值 结束总量上限 负压波动值为0.0时
				{
					*(p_flt_global+i) = 0.001;
				}
			}
		}
		else
		{
			if(i == 19 || i == 25 || i == 26) 	// 19 25 26为总容量值  可以为负，所以特殊处理  
			{
				*(p_flt_global+i) = 0.0;		// 表无效
			}
			else
			{
				*(p_flt_global+i) = -99.9;		// 表无效
			}
		}
	}

	int *p_int_global = (int*)&newGlobalPro.VoltageFluctuateCount;
	for(int i = 0; i < 4; i++)
	{
		if (data.isMember(int_items[i]))
		{
			*(p_int_global+i) = data[int_items[i].c_str()].asInt();
		}
		else
		{
			*(p_int_global+i) = 0;
		}
	}

	for(int i = 0; i < MAX_CHANNELS_NBR; i++)
	{
		if(m_sel_chans[i])
		{
			sprintf(fl_path,"%s",GLOBAL_PRO_XML_PATH);
			sprintf(fl_buff,"/GlobalProtect%d-%d.xml",m_cell_no,i+1);
			strcat(fl_path,fl_buff);		

			globalProfile.saveGlobalProtect(fl_path,newGlobalPro,i+1,m_cell_no);
		}
	}
}

void Dispatcher::m_parse_globalProc_JN95(Json::Value &data)
{
	Channel_Data_t pow_data = Channel_Data_t();
	pow_data.subList_len = 10;	//全局保护共10项

	string items_pro[] = {"OverVoltage", "UnderVoltage", "ChargeOverCurrent",
		"DischargeOverCurrent", "OverCapacity", "InitUpperVoltage", "InitLowerVoltage", 
		"ContactTestTime","ContactTestCurrent","ContactTestDeltaVoltage"};

	try
	{
		//存储全局保护参数
		do_save_globalProtect(data);
		
		for (int i = 0; i < pow_data.subList_len; i++)
		{
			if (data.isMember(items_pro[i]))
			{
				pow_data.subList[i].func_code  = i + 1;
				pow_data.subList[i].value = data[items_pro[i].c_str()].asDouble();
			}
			else
			{
				pow_data.subList[i].func_code = (i + 1)|0x80;		//最高位为1表示未生效
				pow_data.subList[i].value = 0.0;
			}
		}

		m_send_struct(pow_data);
	}
	catch (int &e)
	{
	    LOGFMTE("set GlobalProtect exception code:%d!", e);
	    string josn_str = FormatConvert::instance().alarm_to_string(e);
	    MsgFactory::instance()->cab_alarm_pusher().send(josn_str);
	}
}

void Dispatcher::m_parse_process_cmd_JN95(Json::Value &data)
{
	char fl_path[128];
	char fl_buff[64];
		
	int TotalStep = data["TotalStep"].asInt();

	XmlProcAPI recipefile; 
	work_step_recipe_t *newStepRecp = new work_step_recipe_t[TotalStep];
	if(newStepRecp == NULL)
	{
		return;
	}
	memset(newStepRecp,0,sizeof(work_step_recipe_t) * TotalStep);

	try
	{	
		for (int j = 0; j < TotalStep; j++) 			// 遍历各个工步
		{
			Json::Value v_process = data["StepItems"][j];

			int process_step = v_process["StepNo"].asInt();
			newStepRecp[j].StepNo = process_step;
			
			string stepNameStr = v_process["StepName"].asString();
			sprintf(newStepRecp[j].StepName,"%s",stepNameStr.c_str());

#if 0		
			JN9504_Step_Type_e process_name = get_stepname_value(stepNameStr);
			Channel_Data_t pow_data;
			
			Ch_Step_Data_t step_data_struct;
			Step_Data_t    step_data;
			Step_Param_t   general_data;

			memset((uint8*)&general_data,0,sizeof(Step_Param_t));
			step_data_struct.step_nbr  = process_step;					// 工步序号
			step_data_struct.step_type = process_name;					// 工步名称

			// 取额定参数， 额定参数类型与工步类型匹配
			for (int i = 0; i < 3; i++)
			{
				general_data.spec_data[i].func_code = PARAM_INVALID;
            }
			if (v_process.isMember("RegularVoltage"))
			{
				general_data.spec_data[0].func_code = PARAM_V;
				general_data.spec_data[0].value 	= v_process["RegularVoltage"].asDouble();
			}
			if (v_process.isMember("RegularCurrent"))
			{
				general_data.spec_data[1].func_code = PARAM_I;
				general_data.spec_data[1].value 	= v_process["RegularCurrent"].asDouble();
			}
			/*
			if (v_process.isMember("RatedPower"))
			{
				general_data.spec_data[2].func_code = PARAM_P;
				general_data.spec_data[2].value 	= v_process["RatedPower"].asDouble();
			}
			*/
#endif

#if 0
			// 取截止类型:电流、电压、时间、容量
			for (int i = 0; i < 4; i++)
			{
				general_data.abort_data[i].func_code = PARAM_INVALID;
            }

			if (v_process.isMember("TimeAbortPriority"))
			{
				int level = v_process["TimeAbortPriority"].asInt();
				if(level < 5 && level > 0)
				{
					general_data.abort_data[level-1].func_code = PARAM_T;
					
					double timeMin = v_process["AbortMinute"].asDouble();
					double timeSec = v_process["AbortSecond"].asDouble();
					general_data.abort_data[level-1].value	 = timeMin*60.0 + timeSec;
					general_data.abort_level[level-1] 		 = level;
				}
			} 

			if (v_process.isMember("VoltageAbortPriority"))
			{
				int level = v_process["VoltageAbortPriority"].asInt();
				if(level < 5 && level > 0)
				{
					general_data.abort_data[level-1].func_code = PARAM_V;
					general_data.abort_data[level-1].value	 =  v_process["AbortVoltage"].asDouble();
					general_data.abort_level[level-1] 		 = level;
				}
			} 

			if (v_process.isMember("CurrentAbortPriority"))
			{
				int level = v_process["CurrentAbortPriority"].asInt();
				if(level < 5 && level > 0)
				{
					general_data.abort_data[level-1].func_code = PARAM_I;
					general_data.abort_data[level-1].value	 =  v_process["AbortCurrent"].asDouble();
					general_data.abort_level[level-1]		 = level;
				}
			}

			if (v_process.isMember("CapacityAbortPriority"))
			{
				int level = v_process["CapacityAbortPriority"].asInt();
				if(level < 5 && level > 0)
				{
					general_data.abort_data[level-1].func_code = PARAM_C;
					general_data.abort_data[level-1].value	 =  v_process["AbortCapacity"].asDouble();
					general_data.abort_level[level-1] 		 = level;
				}
			}
#endif			
			
			newStepRecp[j].RegularCurrent = v_process["RegularCurrent"].asDouble();
			newStepRecp[j].RegularVoltage = v_process["RegularVoltage"].asDouble();
			newStepRecp[j].RecordFrequency = v_process["RecordFrequency"].asInt();
			newStepRecp[j].AbortCurrent = v_process["AbortCurrent"].asDouble();
			newStepRecp[j].CurrentAbortPrio = v_process["CurrentAbortPriority"].asInt();
			newStepRecp[j].AbortVoltage = v_process["AbortVoltage"].asDouble();
			newStepRecp[j].VoltageAbortPrio = v_process["VoltageAbortPriority"].asInt();
			newStepRecp[j].AbortCapacity = v_process["AbortCapacity"].asDouble();
			newStepRecp[j].CapacityAbortPrio = v_process["CapacityAbortPriority"].asInt();
			newStepRecp[j].AbortMinute = v_process["AbortMinute"].asInt();
			newStepRecp[j].AbortSecond = v_process["AbortSecond"].asInt();
			newStepRecp[j].TimeAbortPrio = v_process["TimeAbortPriority"].asInt();
			newStepRecp[j].AbortAction = v_process["AbortAction"].asInt();
			newStepRecp[j].JumpToStep = v_process["JumpToStep"].asInt();

			newStepRecp[j].CycleFrom = v_process["CycleFrom"].asInt();
			newStepRecp[j].CycleCount = v_process["CycleCount"].asInt();
			newStepRecp[j].VacValue = v_process["VacuumValue"].asDouble();
			newStepRecp[j].VacTime = v_process["VacuumTime"].asInt();
			newStepRecp[j].LeakValue = v_process["ReleaseValue"].asDouble();
			newStepRecp[j].LeakTime = v_process["ReleaseTime"].asInt();
			
			Json::Value v_protect_data = v_process["StepProtect"];
#if 0
			// 单工步保护
			string items_pro[] = {"UpperVoltage", "LowerVoltage", "OverCurrent", "OverCapacity",
				"VoltageOverDiff", "CurrentOverDiff"};

			for (int i = 0; i < 6; i++)
			{
				if (v_protect_data.isMember(items_pro[i]))
				{
					general_data.protection[i].func_code = i + 1;
					general_data.protection[i].value = v_protect_data[items_pro[i].c_str()].asDouble();
				}
				else
				{
                	general_data.protection[i].func_code = PARAM_INVALID;
				}
            }
#endif

			if(v_protect_data.isMember("UpperVoltage"))
				newStepRecp[j].stepProtect.UpperVoltage = v_protect_data["UpperVoltage"].asDouble();
			else
				newStepRecp[j].stepProtect.UpperVoltage = 0.0;

			if(v_protect_data.isMember("LowerVoltage"))
				newStepRecp[j].stepProtect.LowerVoltage = v_protect_data["LowerVoltage"].asDouble();
			else
				newStepRecp[j].stepProtect.LowerVoltage = 0.0;

			if(v_protect_data.isMember("VoltageOverDiff"))
				newStepRecp[j].stepProtect.VoltageOverDiff = v_protect_data["VoltageOverDiff"].asDouble();
			else
				newStepRecp[j].stepProtect.VoltageOverDiff = 0.0;
				
			newStepRecp[j].stepProtect.PositiveDeltaVoltage = v_protect_data["PositiveDeltaVoltage"].asDouble();
			newStepRecp[j].stepProtect.PositiveDeltaVoltageCount = v_protect_data["PositiveDeltaVoltageCount"].asInt();	
			newStepRecp[j].stepProtect.NegativeDeltaVoltage = v_protect_data["NegativeDeltaVoltage"].asDouble();
			newStepRecp[j].stepProtect.NegativeDeltaVoltageCount = v_protect_data["NegativeDeltaVoltageCount"].asInt();
			newStepRecp[j].stepProtect.CrossStepPositiveDeltaVoltage = v_protect_data["CrossStepPositiveDeltaVoltage"].asDouble();
			newStepRecp[j].stepProtect.CrossStepNegativeDeltaVoltage = v_protect_data["CrossStepNegativeDeltaVoltage"].asDouble();

			newStepRecp[j].stepProtect.UpperTimeVoltage = v_protect_data["UpperTimeVoltage"].asDouble();
			newStepRecp[j].stepProtect.LowerTimeVoltage = v_protect_data["LowerTimeVoltage"].asDouble();
			newStepRecp[j].stepProtect.TimeVoltageCheckTime = v_protect_data["TimeVoltageCheckTime"].asInt();
			newStepRecp[j].stepProtect.RangeVoltageCheckTime1 = v_protect_data["RangeVoltageCheckTime1"].asInt();
			newStepRecp[j].stepProtect.RangeVoltageCheckTime2 = v_protect_data["RangeVoltageCheckTime2"].asInt();
			//newStepRecp[j].stepProtect.DeltaTimeVoltage = v_protect_data["DeltaTimeVoltage"].asDouble();
			newStepRecp[j].stepProtect.UpperDeltaRangeVoltage = v_protect_data["UpperDeltaRangeVoltage"].asDouble();
			newStepRecp[j].stepProtect.LowerDeltaRangeVoltage = v_protect_data["LowerDeltaRangeVoltage"].asDouble();

			newStepRecp[j].stepProtect.VoltageCheckCycleTime = v_protect_data["VoltageCheckCycleTime"].asInt();
			newStepRecp[j].stepProtect.UpperDeltaCycleVoltage = v_protect_data["UpperDeltaCycleVoltage"].asDouble();
			newStepRecp[j].stepProtect.LowerDeltaCycleVoltage = v_protect_data["LowerDeltaCycleVoltage"].asDouble();
			
			if(v_protect_data.isMember("OverCurrent"))
				newStepRecp[j].stepProtect.OverCurrent = v_protect_data["OverCurrent"].asDouble();
			else
				newStepRecp[j].stepProtect.OverCurrent = 0.0;

			if(v_protect_data.isMember("CurrentOverDiff"))
				newStepRecp[j].stepProtect.CurrentOverDiff = v_protect_data["CurrentOverDiff"].asDouble();
			else
				newStepRecp[j].stepProtect.CurrentOverDiff = 0.0;		
			
			newStepRecp[j].stepProtect.PositiveDeltaCurrent = v_protect_data["PositiveDeltaCurrent"].asDouble();
			newStepRecp[j].stepProtect.PositiveDeltaCurrentCount = v_protect_data["PositiveDeltaCurrentCount"].asInt();
			newStepRecp[j].stepProtect.NegativeDeltaCurrent = v_protect_data["NegativeDeltaCurrent"].asDouble();
			newStepRecp[j].stepProtect.NegativeDeltaCurrentCount = v_protect_data["NegativeDeltaCurrentCount"].asInt();
			if(v_protect_data.isMember("OverCapacity"))
				newStepRecp[j].stepProtect.OverCapacity = v_protect_data["OverCapacity"].asDouble();
			else
				newStepRecp[j].stepProtect.OverCapacity = 0.0;

			if(v_protect_data.isMember("BatteryUpperTemp"))
				newStepRecp[j].stepProtect.BatteryUpperTemp = v_protect_data["BatteryUpperTemp"].asDouble();
			else
				newStepRecp[j].stepProtect.BatteryUpperTemp = -99.9;

			if(v_protect_data.isMember("BatteryLowerTemp"))
				newStepRecp[j].stepProtect.BatteryLowerTemp = v_protect_data["BatteryLowerTemp"].asDouble();
			else
				newStepRecp[j].stepProtect.BatteryLowerTemp = -99.9;

			newStepRecp[j].stepProtect.LowerStepStopVoltage = v_protect_data["LowerStepStopVoltage"].asDouble();
			newStepRecp[j].stepProtect.LowerStepStopCapacity = v_protect_data["LowerStepStopCapacity"].asDouble();
			newStepRecp[j].stepProtect.LowerStepStopTime = v_protect_data["LowerStepStopTime"].asInt();
#if 0
			step_data.general_data     = general_data;
			step_data_struct.step_data = step_data;
			pow_data.step_data         = step_data_struct;

			if(j == 0)
			{
				m_send_struct(pow_data);
			}
#endif
		}

		for(int i = 0; i < MAX_CHANNELS_NBR; i++)
		{
			if(m_sel_chans[i])
			{
				sprintf(fl_path,"%s",WORK_STEP_XML_PATH);
				sprintf(fl_buff,"/WorkstepRecipe%d-%d.xml",m_cell_no,i+1);
				strcat(fl_path,fl_buff);		
		
				recipefile.saveStepRecipeData(fl_path,newStepRecp,TotalStep);
			}
		}
		
		Channel_Data_t pow_data;
		m_send_struct(pow_data);
	}
	catch (int &e)
	{
	    LOGFMTE("set Step exception code:%d!", e);
	    string josn_str = FormatConvert::instance().alarm_to_string(e);
	    MsgFactory::instance()->cab_alarm_pusher().send(josn_str);
	}

	if(newStepRecp)
	{
		delete[] newStepRecp;
	}
}

void Dispatcher::m_parse_process_cmd(Json::Value &data)
{
	for (size_t j = 1; j <= data.size(); j++) 									// 遍历各个工步
	{
		Json::Value v_process = data[j - 1];
		int process_name = v_process["ProcessStepName"].asInt();
		int process_step = v_process["ProcessStepNo"].asInt();

		Channel_Data_t pow_data;
		
		Ch_Step_Data_t step_data_struct;
		Step_Data_t    step_data;
		Step_Param_t   general_data;

		memset((uint8*)&general_data,0,sizeof(Step_Param_t));
		step_data_struct.step_nbr  = process_step;						// 工步序号
		step_data_struct.step_type = (JN9504_Step_Type_e)process_name;	// 工步名称
		
		if (process_name == STEP_LOOP_START || process_name == STEP_LOOP_STOP)		// 循环开始/结束
		{
			//if (process_name == EXEC_STOP)
            {
                Json::Value v_loop_step_setting = v_process["LoopStepSetting"];
        		step_data.loop_data[0].value = v_loop_step_setting["BeginStepNo"].asDouble();
        		step_data.loop_data[1].value = v_loop_step_setting["EndStepNo"].asDouble();
        		step_data.loop_data[2].value = v_loop_step_setting["StepAbortType"].asDouble();
        		step_data.loop_data[3].value = v_loop_step_setting["LoopType"].asDouble();
        		step_data.loop_data[4].value = v_loop_step_setting["LoopTime"].asDouble();
            }

            step_data_struct.step_data = step_data;
			pow_data.step_data = step_data_struct;
		}
		else 																	// 工步
		{
			// 取额定参数， 额定参数类型与工步类型匹配
			// 电流、电压、功率
			Json::Value v_process_step_setting = v_process["ProcessStepSetting"];

			for (int i = 0; i < 3; i++)
			{
				general_data.spec_data[i].func_code = PARAM_INVALID;
            }
			if (v_process_step_setting.isMember("Voltage"))
			{
				general_data.spec_data[0].func_code = PARAM_V;
				general_data.spec_data[0].value 	= v_process_step_setting["Voltage"].asDouble();
			}
			if (v_process_step_setting.isMember("Current"))
			{
				general_data.spec_data[1].func_code = PARAM_I;
				general_data.spec_data[1].value 	= v_process_step_setting["Current"].asDouble();
			}
			/*
			if (v_process_step_setting.isMember("Power"))
			{
				general_data.spec_data[2].func_code = PARAM_P;
				general_data.spec_data[2].value 	= v_process_step_setting["Power"].asDouble();
			}
			*/

			// 取截止类型:电流、电压、时间、容量
			Json::Value v_abort_setting = v_process["AbortSetting"];
			
			for (int i = 0; i < 4; i++)
			{
				general_data.abort_data[i].func_code = PARAM_INVALID;
            }
			for (size_t j = 0; j < v_abort_setting.size(); j++)
			{
				if (v_abort_setting[j].isMember("Time"))
				{
					general_data.abort_data[0].func_code = PARAM_T;
					general_data.abort_data[0].value 	 = v_abort_setting[j]["Time"].asDouble();
					general_data.abort_level[0]          = v_abort_setting[j]["Level"].asInt();
				} 
				else if (v_abort_setting[j].isMember("Voltage"))
				{
					general_data.abort_data[1].func_code = PARAM_V;
					general_data.abort_data[1].value 	 = v_abort_setting[j]["Voltage"].asDouble();
					general_data.abort_level[1]          = v_abort_setting[j]["Level"].asInt();
				} 
				else if (v_abort_setting[j].isMember("Current"))
				{
					general_data.abort_data[2].func_code = PARAM_I;
					general_data.abort_data[2].value 	 = v_abort_setting[j]["Current"].asDouble();
					general_data.abort_level[2]          = v_abort_setting[j]["Level"].asInt();
				}
				else if (v_abort_setting[j].isMember("Capacity"))
				{
					general_data.abort_data[3].func_code = PARAM_C;
					general_data.abort_data[3].value 	 = v_abort_setting[j]["Capacity"].asDouble();
					general_data.abort_level[3]          = v_abort_setting[j]["Level"].asInt();
				}
			}

#ifndef PROTOCOL_5V160A
			// 取记录条件   : 电流、电压、时间
			Json::Value v_record_setting = v_process["RecordSetting"];

			for (int i = 0; i < 3; i++)
			{
				general_data.record_cond[i].func_code = PARAM_INVALID;
            }
			if (v_record_setting.isMember("Time"))
			{
				general_data.record_cond[0].func_code = PARAM_T;
				general_data.record_cond[0].value 	  = v_record_setting["Time"].asDouble();
			}
			if (v_record_setting.isMember("Voltage"))
			{
				general_data.record_cond[1].func_code = PARAM_V;
				general_data.record_cond[1].value 	= v_record_setting["Voltage"].asDouble();
			}
			if (v_record_setting.isMember("Current"))
			{
				general_data.record_cond[2].func_code = PARAM_I;
				general_data.record_cond[2].value 	  = v_record_setting["Current"].asDouble();
			}
#endif

			// 单工步保护
			Json::Value v_protect_data = v_process["ProtectSetting"];
			string items_4[] = {"OTP", "OCAP", "UCAP", "OPV", "UVP", "CURRENTH", "CURRENTL", "CJU", "CJUDATA", "CJL", "CJLDATA",
				"VJU", "VJUDATA", "VJL", "VJLDATA", "IODV", "VODV", "UTP"};

#ifndef PROTOCOL_5V160A
			for (int i = 0; i < 18; i++)
#else
			for (int i = 0; i < 6; i++)
#endif
			{
				if (v_protect_data.isMember(items_4[i]))
				{
					general_data.protection[i].func_code = i + 1;
					general_data.protection[i].value	 = v_protect_data[items_4[i].c_str()].asDouble();
				}
				else
				{
                	general_data.protection[i].func_code = PARAM_INVALID;
				}
            }

#ifndef PROTOCOL_5V160A
			Json::Value v_tvp = v_process["TimerVolProtectSetting"];
			if (v_tvp.isMember("time"))
			{
				general_data.protection[18].func_code = 19;
				general_data.protection[18].value	  = v_tvp["time"].asDouble();
			}
			if (v_tvp.isMember("VH"))
			{
				general_data.protection[19].func_code = 20;
				general_data.protection[19].value	  = v_tvp["VH"].asDouble();
			}
			if (v_tvp.isMember("VL"))
			{
				general_data.protection[20].func_code = 21;
				general_data.protection[20].value	  = v_tvp["VL"].asDouble();
			}

			Json::Value v_tvdp = v_process["TimerVolDeltaProtectSetting"];
			if (v_tvdp.isMember("t1"))
			{
				general_data.protection[21].func_code = 22;
				general_data.protection[21].value	  = v_tvdp["t1"].asDouble();
			}
			if (v_tvdp.isMember("t2"))
			{
				general_data.protection[22].func_code = 23;
				general_data.protection[22].value	  = v_tvdp["t2"].asDouble();
			}
			if (v_tvdp.isMember("DV"))
			{
				general_data.protection[23].func_code = 24;
				general_data.protection[23].value	  = v_tvdp["DV"].asDouble();
			}

			Json::Value v_bsp = v_process["BatShortProtectSetting"];
			if (v_bsp.isMember("time"))
			{
				general_data.protection[24].func_code = 25;
				general_data.protection[24].value	  = v_bsp["time"].asDouble();
			}
			if (v_bsp.isMember("VH"))
			{
				general_data.protection[25].func_code = 26;
				general_data.protection[25].value	  = v_bsp["VH"].asDouble();
			}
			if (v_bsp.isMember("VL"))
			{
				general_data.protection[26].func_code = 27;
				general_data.protection[26].value	  = v_bsp["VL"].asDouble();
			}
#endif
			step_data.general_data     = general_data;
			step_data_struct.step_data = step_data;
			pow_data.step_data         = step_data_struct;
		}
		
		m_send_struct(pow_data);
	}
}
#else
void Dispatcher::m_parse_process_cmd(Json::Value &data)
{
	for (size_t j = 0; j < data.size(); j++) 									// 遍历各个工步
	{
		Channel_Data_t pow_data;
		Ch_Step_Data_t step_data_struct;
		Step_Data_t    step_data;
		Step_Param_t   general_data;
		
		Json::Value v_process = data[j];
		int process_name = v_process["ProcessStepName"].asInt();				// 工步名称
		int process_step = v_process["ProcessStepNo"].asInt();					// 工步序号

		if (process_name == STEP_LOOP_START || process_name == STEP_LOOP_STOP) 	// 循环开始/循环结束
		{
			step_data_struct.step_nbr  = process_step;
			step_data_struct.step_type = (Step_Type_e)process_name;

			Json::Value v_loop_step_setting = v_process["LoopStepSetting"];
			step_data.loop_data[0].value = v_loop_step_setting.get("BeginStepNo", 0).asInt();
			step_data.loop_data[1].value = v_loop_step_setting.get("EndStepNo", 0).asInt();
			step_data.loop_data[2].value = v_loop_step_setting.get("StepAbortType", 0).asInt();
			step_data.loop_data[3].value = v_loop_step_setting.get("LoopType", 0).asInt();
			step_data.loop_data[4].value = v_loop_step_setting.get("LoopTime", 0).asInt();

			step_data_struct.step_data = step_data;
			pow_data.step_data = step_data_struct;
		}
		else // 工步
		{
			step_data_struct.step_nbr  = process_step;
			step_data_struct.step_type = (Step_Type_e)process_name;

			// 取额定参数， 额定参数类型与工步类型匹配
			Json::Value v_process_step_setting = v_process["ProcessStepSetting"];

			for (int i = 0; i < 3; i++)
			{
                general_data.spec_data[i].func_code = PARAM_INVALID;  			// 记录数据
            }

			// 时间 电阻
			if (v_process_step_setting.isMember("Time"))
			{
                general_data.spec_data[0].func_code = PARAM_T;
                general_data.spec_data[0].value = v_process_step_setting["Time"].asDouble()*1000;
			}
			else if (v_process_step_setting.isMember("Resistance"))
			{
                general_data.spec_data[0].func_code = PARAM_R;
				if (process_name == STEP_VAC)
				{
					general_data.spec_data[0].func_code = PARAM_VAC;
                	general_data.spec_data[0].value = v_process_step_setting["Resistance"].asDouble();
					// 第一个工步为负压工步
					if (process_step == 1)
					{			
						m_send_pins(SYS_VAC_SET, general_data.spec_data[0].value);
					}
				}
				else
				{
                	general_data.spec_data[0].value = v_process_step_setting["Resistance"].asDouble()*1000;
				}
			}

			if (v_process_step_setting.isMember("Voltage"))
			{
                general_data.spec_data[1].func_code = PARAM_V;
                general_data.spec_data[1].value = v_process_step_setting["Voltage"].asDouble()*10000;
			}
			else if (v_process_step_setting.isMember("Power"))
			{
                general_data.spec_data[1].func_code = PARAM_P;
                general_data.spec_data[1].value = v_process_step_setting["Power"].asDouble()*1000;
			}

			// 电流
			if (v_process_step_setting.isMember("Current"))
			{
                general_data.spec_data[2].func_code = PARAM_I;
                general_data.spec_data[2].value = v_process_step_setting["Current"].asDouble()*10000;
			}

			// 取记录条件   时间 电压  电流
			for (int i = 0; i < 3; i++)
			{
                general_data.record_cond[i].func_code = 0xFF;  					// 记录数据
            }

			Json::Value v_record_setting = v_process["RecordSetting"];
			if (v_record_setting.isMember("Time"))
			{
			    general_data.record_cond[0].func_code = PARAM_T;
				general_data.record_cond[0].value = v_record_setting["Time"].asDouble()*1000;
			}

			if (v_record_setting.isMember("Voltage"))
			{
                general_data.record_cond[1].func_code = PARAM_V;
                general_data.record_cond[1].value = v_record_setting["Voltage"].asDouble()*10000;
			}

			if (v_record_setting.isMember("Current"))
			{
                general_data.record_cond[2].func_code = PARAM_I;
                general_data.record_cond[2].value = v_record_setting["Current"].asDouble()*10000;
			}

			// 取截止类型
			Json::Value v_abort_setting = v_process["AbortSetting"];

			step_data.general_data = general_data;
			step_data_struct.step_data = step_data;
			pow_data.step_data = step_data_struct;

			for (int i = 0; i < 3; i++)
			{
                pow_data.step_data.step_data.general_data.stop1_type[i] = 0xFF;                       // 截止量
                pow_data.step_data.step_data.general_data.stop1_data[i].func_code = 0xFF;             // 工步截止逻辑类型， 默认 大于等于
                pow_data.step_data.step_data.general_data.stop1_result[i].func_code = 0xFF;           // 工步截止结果类型， 默认为 下一步
                pow_data.step_data.step_data.general_data.stop1_level[i] = 0xFF;                      // 优先级
            }

			for (size_t i = 0; i < v_abort_setting.size(); i++)
			{
	#ifdef ZHONGHANG_PRJ
				// 截止条件类型
				uint8_t stop_type = PARAM_INVALID;
				const char *name = "";

				if (v_abort_setting[i].isMember("Time"))
				{
				    name = "Time";
					stop_type = PARAM_T;
				}
				else if (v_abort_setting[i].isMember("Resistance"))
				{
				    name = "Resistance";
					stop_type = PARAM_R;
				}
				else if (v_abort_setting[i].isMember("Voltage"))
				{
				    name = "Voltage";
					stop_type = PARAM_V;
				}
				else if (v_abort_setting[i].isMember("Power"))
				{
				    name = "Power";
					stop_type = PARAM_P;
				}
				else if (v_abort_setting[i].isMember("Current"))
				{
				    name = "Current";
					stop_type = PARAM_I;
				}
				else if (v_abort_setting[i].isMember("Capacity"))
				{
				    name = "Capacity";
					stop_type = PARAM_C;
				}
				pow_data.step_data.step_data.general_data.stop1_type[i] = stop_type;
				// 工步截止逻辑类型， 默认 大于等于
				pow_data.step_data.step_data.general_data.stop1_data[i].func_code = v_abort_setting[i]["LogicNo"].asInt();
				if (stop_type == PARAM_V || stop_type == PARAM_I)
				{
					pow_data.step_data.step_data.general_data.stop1_data[i].value = v_abort_setting[i][name].asDouble()*10000;
				}
				else
				{
					pow_data.step_data.step_data.general_data.stop1_data[i].value = v_abort_setting[i][name].asDouble()*1000;
				}
				// 工步截止结果类型， 默认为 下一步
				pow_data.step_data.step_data.general_data.stop1_result[i].func_code = v_abort_setting[i]["StepAbortType"].asInt();
				pow_data.step_data.step_data.general_data.stop1_result[i].value = v_abort_setting[i]["GotoStep"].asInt();
				// 截止条件优先级
				pow_data.step_data.step_data.general_data.stop1_level[i] = v_abort_setting[i]["Level"].asInt();				
	#else
				// 老版本
				if (v_abort_setting[i].isMember("Time"))
				{
				    const char *name = "Time";
					pow_data.step_data.step_data.general_data.stop1_type[0] = PARAM_T;         											// 截止条件类型
					pow_data.step_data.step_data.general_data.stop1_data[0].func_code = v_abort_setting[i]["LogicNo"].asInt();			// 工步截止逻辑类型， 默认 大于等于
					pow_data.step_data.step_data.general_data.stop1_data[0].value = v_abort_setting[i][name].asDouble()*1000;
					pow_data.step_data.step_data.general_data.stop1_result[0].func_code = v_abort_setting[i]["StepAbortType"].asInt();	// 工步截止结果类型， 默认为 下一步
					pow_data.step_data.step_data.general_data.stop1_result[0].value = v_abort_setting[i]["GotoStep"].asInt();			// 不跳转
				}
				else if (v_abort_setting[i].isMember("Resistance"))
				{
				    const char *name = "Resistance";
					pow_data.step_data.step_data.general_data.stop1_type[0] = PARAM_R;         											// 截止条件类型
					pow_data.step_data.step_data.general_data.stop1_data[0].func_code = v_abort_setting[i]["LogicNo"].asInt();			// 工步截止逻辑类型， 默认 大于等于
					pow_data.step_data.step_data.general_data.stop1_data[0].value = v_abort_setting[i][name].asDouble()*1000;
					pow_data.step_data.step_data.general_data.stop1_result[0].func_code = v_abort_setting[i]["StepAbortType"].asInt();	// 工步截止结果类型， 默认为 下一步
					pow_data.step_data.step_data.general_data.stop1_result[0].value = v_abort_setting[i]["GotoStep"].asInt();			// 不跳转

				}
				else if (v_abort_setting[i].isMember("Voltage"))
				{
				    const char *name = "Voltage";
					pow_data.step_data.step_data.general_data.stop1_type[1] = PARAM_V;         											// 截止条件类型
					pow_data.step_data.step_data.general_data.stop1_data[1].func_code = v_abort_setting[i]["LogicNo"].asInt();			// 工步截止逻辑类型， 默认 大于等于
					pow_data.step_data.step_data.general_data.stop1_data[1].value = v_abort_setting[i][name].asDouble()*10000;
					pow_data.step_data.step_data.general_data.stop1_result[1].func_code = v_abort_setting[i]["StepAbortType"].asInt();	// 工步截止结果类型， 默认为 下一步
					pow_data.step_data.step_data.general_data.stop1_result[1].value = v_abort_setting[i]["GotoStep"].asInt();			// 不跳转
				}
				else if (v_abort_setting[i].isMember("Power"))
				{
				    const char *name = "Power";
					pow_data.step_data.step_data.general_data.stop1_type[1] = PARAM_P;         											// 截止条件类型
					pow_data.step_data.step_data.general_data.stop1_data[1].func_code = v_abort_setting[i]["LogicNo"].asInt();			// 工步截止逻辑类型， 默认 大于等于
					pow_data.step_data.step_data.general_data.stop1_data[1].value = v_abort_setting[i][name].asDouble()*1000;
					pow_data.step_data.step_data.general_data.stop1_result[1].func_code = v_abort_setting[i]["StepAbortType"].asInt();	// 工步截止结果类型， 默认为 下一步
					pow_data.step_data.step_data.general_data.stop1_result[1].value = v_abort_setting[i]["GotoStep"].asInt();			// 不跳转
				}
				else if (v_abort_setting[i].isMember("Current"))
				{
				    const char *name = "Current";
					pow_data.step_data.step_data.general_data.stop1_type[2] = PARAM_I;         											// 截止条件类型
					pow_data.step_data.step_data.general_data.stop1_data[2].func_code = v_abort_setting[i]["LogicNo"].asInt();			// 工步截止逻辑类型， 默认 大于等于
					pow_data.step_data.step_data.general_data.stop1_data[2].value = v_abort_setting[i][name].asDouble()*10000;
					pow_data.step_data.step_data.general_data.stop1_result[2].func_code = v_abort_setting[i]["StepAbortType"].asInt();	// 工步截止结果类型， 默认为 下一步
					pow_data.step_data.step_data.general_data.stop1_result[2].value = v_abort_setting[i]["GotoStep"].asInt();			// 不跳转
				}
				else if (v_abort_setting[i].isMember("Capacity"))
				{
				    const char *name = "Capacity";
					pow_data.step_data.step_data.general_data.stop1_type[2] = PARAM_C;         											// 截止条件类型
					pow_data.step_data.step_data.general_data.stop1_data[2].func_code = v_abort_setting[i]["LogicNo"].asInt();			// 工步截止逻辑类型， 默认 大于等于
					pow_data.step_data.step_data.general_data.stop1_data[2].value = v_abort_setting[i][name].asDouble()*1000;
					pow_data.step_data.step_data.general_data.stop1_result[2].func_code = v_abort_setting[i]["StepAbortType"].asInt();	// 工步截止结果类型， 默认为 下一步
					pow_data.step_data.step_data.general_data.stop1_result[2].value = v_abort_setting[i]["GotoStep"].asInt();			// 不跳转
				}
	#endif
			}
			
			// 工步保护	
			memset((void *)pow_data.step_data.step_data.general_data.protection, 0xFF, sizeof(pow_data.step_data.step_data.general_data.protection));
			Json::Value v_protect_data = v_process["ProtectSetting"];
			pow_data.step_data.step_data.general_data.protection[0].value  = v_protect_data.isMember("OTP") ? (v_protect_data.get("OTP", 0xFF).asDouble())*10.0 : 0xFF;
			pow_data.step_data.step_data.general_data.protection[1].value  = v_protect_data.isMember("OCAP") ? (v_protect_data.get("OCAP", 0xFF).asDouble())*1000.0 : 0xFF;
			pow_data.step_data.step_data.general_data.protection[2].value  = v_protect_data.isMember("UCAP") ? (v_protect_data.get("UCAP", 0xFF).asDouble())*1000.0 : 0xFF;
			pow_data.step_data.step_data.general_data.protection[3].value  = v_protect_data.isMember("OPV") ? (v_protect_data.get("OPV", 0xFF).asDouble())*10000.0 : 0xFF;
			pow_data.step_data.step_data.general_data.protection[4].value  = v_protect_data.isMember("UVP") ? (v_protect_data.get("UVP", 0xFF).asDouble())*10000.0 : 0xFF;
			pow_data.step_data.step_data.general_data.protection[5].value  = v_protect_data.isMember("CURRENTH") ? (v_protect_data.get("CURRENTH", 0xFF).asDouble())*10000.0 : 0xFF;
			pow_data.step_data.step_data.general_data.protection[6].value  = v_protect_data.isMember("CURRENTL") ? (v_protect_data.get("CURRENTL", 0xFF).asDouble())*10000.0 : 0xFF;
			pow_data.step_data.step_data.general_data.protection[7].value  = v_protect_data.isMember("CJU") ? (v_protect_data.get("CJU", 0xFF).asDouble())*10000.0 : 0xFF;
			pow_data.step_data.step_data.general_data.protection[8].value  = v_protect_data.isMember("CJUDATA") ? v_protect_data.get("CJUDATA", 0xFF).asInt() : 0xFF;
			pow_data.step_data.step_data.general_data.protection[9].value  = v_protect_data.isMember("CJL") ? (v_protect_data.get("CJL", 0xFF).asDouble())*10000.0 : 0xFF;
			pow_data.step_data.step_data.general_data.protection[10].value = v_protect_data.isMember("CJLDATA") ? v_protect_data.get("CJLDATA", 0xFF).asInt() : 0xFF;
			pow_data.step_data.step_data.general_data.protection[11].value = v_protect_data.isMember("VJU") ? (v_protect_data.get("VJU", 0xFF).asDouble())*10000.0 : 0xFF;
			pow_data.step_data.step_data.general_data.protection[12].value = v_protect_data.isMember("VJUDATA") ? v_protect_data.get("VJUDATA", 0xFF).asInt() : 0xFF;
			pow_data.step_data.step_data.general_data.protection[13].value = v_protect_data.isMember("VJL") ? (v_protect_data.get("VJL", 0xFF).asDouble())*10000.0 : 0xFF;
			pow_data.step_data.step_data.general_data.protection[14].value = v_protect_data.isMember("VJLDATA") ? v_protect_data.get("VJLDATA", 0xFF).asInt() : 0xFF;
			// 新增保护参数
	#ifdef ZHONGHANG_PRJ
			if (v_process.isMember("TimerVolProtectSetting"))
			{
				Json::Value v_tvp = v_process["TimerVolProtectSetting"];
				pow_data.step_data.step_data.general_data.protection[15].value = v_tvp.isMember("time") ? v_tvp.get("time", 0xFF).asDouble() * 1000.0 : 0xFF;
				pow_data.step_data.step_data.general_data.protection[16].value = v_tvp.isMember("VH") ? v_tvp.get("VH", 0xFF).asDouble() * 10000.0 : 0xFF;
				pow_data.step_data.step_data.general_data.protection[17].value = v_tvp.isMember("VL") ? v_tvp.get("VL", 0xFF).asDouble() * 10000.0 : 0xFF;
			}
			if (v_process.isMember("TimerVolDeltaProtectSetting"))
			{
				Json::Value v_tvdp = v_process["TimerVolDeltaProtectSetting"];
				pow_data.step_data.step_data.general_data.protection[18].value = v_tvdp.isMember("t1") ? v_tvdp.get("t1", 0xFF).asDouble() * 1000.0 : 0xFF;
				pow_data.step_data.step_data.general_data.protection[19].value = v_tvdp.isMember("t2") ? v_tvdp.get("t2", 0xFF).asDouble() * 1000.0 : 0xFF;
				pow_data.step_data.step_data.general_data.protection[20].value = v_tvdp.isMember("DV") ? v_tvdp.get("DV", 0xFF).asDouble() * 10000.0 : 0xFF;
			}
			if (v_process.isMember("BatShortProtectSetting"))
			{
				Json::Value v_bsp = v_process["BatShortProtectSetting"];
				pow_data.step_data.step_data.general_data.protection[21].value = v_bsp.isMember("time") ? v_bsp.get("time", 0xFF).asDouble() * 1000.0 : 0xFF;
				pow_data.step_data.step_data.general_data.protection[22].value = v_bsp.isMember("VH") ? v_bsp.get("VH", 0xFF).asDouble() * 10000.0 : 0xFF;
				pow_data.step_data.step_data.general_data.protection[23].value = v_bsp.isMember("VL") ? v_bsp.get("VL", 0xFF).asDouble() * 10000.0 : 0xFF;
			}
	#endif
	#ifdef ZHONGHANG_PRJ_NEW
			pow_data.step_data.step_data.general_data.protection[24].value = v_protect_data.isMember("VODV") ? v_protect_data.get("VODV", 0xFF).asDouble()*10000.0 : 0xFF;
			pow_data.step_data.step_data.general_data.protection[25].value = v_protect_data.isMember("IODV") ? v_protect_data.get("IODV", 0xFF).asDouble()*10000.0 : 0xFF;
			pow_data.step_data.step_data.general_data.protection[26].value = v_protect_data.isMember("NDVP") ? v_protect_data.get("NDVP", 0xFF).asDouble()*10000.0 : 0xFF;
			pow_data.step_data.step_data.general_data.protection[27].value = v_protect_data.isMember("NDVN") ? v_protect_data.get("NDVN", 0xFF).asDouble()*10000.0 : 0xFF;
			pow_data.step_data.step_data.general_data.protection[28].value = v_protect_data.isMember("UTP") ? v_protect_data.get("UTP", 0xFF).asDouble()*10.0 : 0xFF;
	#endif
		}
		
		m_send_struct(pow_data);
	}
}

int Dispatcher::m_procet_factor(uint8 subfunc, double value)
{
    int result = 0;
    switch(subfunc)
    {
        case GLOBAL_OCAP:
        case GLOBAL_UCAP:
		case 0x25:							// 总荷电容量上限
		case 0x28:							// 接触测试时间
            result = (int)(value*1000);
            break;
		
        case GLOBAL_OTP:
        case GLOBAL_UTP:
        case GLOBAL_POW_OTP:
		case GLOBAL_BAT_OTW:
		case 44:							// 电池超高温
            result = (int)(value*10);
            break;
		
        case GLOBAL_REVERSE:
            result = (int)(value*10000);
            break;

		// 次数
		case 15:
		case 17:
            result = (int)value;
			break;
			
		// 电压、电流 *10000
        default:
            result = (int)(value*10000);
            break;
    }

    return result;
}

int Dispatcher::m_cal_factor(uint8 subfunc, double value)
{
    int result = 0;
    switch(subfunc)
    {
        case CAL_C_I_OFFSET:
        case CAL_D_I_OFFSET:
            result = (int)(value*10000);
            break;
        case CAL_T_CAL:
        case CAL_T_ENV:
            result = (int)(value*10);
            break;
        default:
            result = (int)(value);
            break;
    }

    return result;
}

#endif

void Dispatcher::m_send_func_struct(int cell_no,int func_code)
{
    char topic[32] = {'\0'};
    sprintf(topic, TOPIC_CELL_FORMAT_STR, cell_no);
	
    Cmd_Cell_Msg_t msg_struct = Cmd_Cell_Msg_t();
	msg_struct.func_code = func_code;
	
	MsgFactory::instance()->cab_puber().send(topic, msg_struct, ZMQ_DONTWAIT);
}

void Dispatcher::m_send_struct(Channel_Data_t &data_union)
{
    char topic[32] = {'\0'};
    sprintf(topic, TOPIC_CELL_FORMAT_STR, m_cell_no);

    Cmd_Cell_Msg_t msg_struct = Cmd_Cell_Msg_t();
    msg_struct.func_code = m_func_code;

	for (int i = 0; i < MAX_CHANNELS_NBR; i++)
	{
	    msg_struct.cell_msg.pow.sel_chans[i] = m_sel_chans[i];
	}
    msg_struct.cell_msg.pow.data = data_union;

    MsgFactory::instance()->cab_puber().send(topic, msg_struct, ZMQ_DONTWAIT);
}

void Dispatcher::m_send_struct(Cmd_Pins_Data_t &data_union)
{
#if defined(ENABLE_PLC)
	Cmd_Pins_Msg_t pin_cmd = Cmd_Pins_Msg_t();
	pin_cmd.cell_no   = m_cell_no;
	pin_cmd.func_code = m_func_code;
	pin_cmd.pins_msg  = data_union;
	MsgFactory::instance()->cell_pusher(m_cell_no).send(pin_cmd, ZMQ_DONTWAIT);
#endif
}

#if defined(AWH_PRECHARGE_SYS)
void Dispatcher::m_send_struct_PLC(Cmd_Pins_Data_t &data_union , int func_code)
{
#if defined(ENABLE_PLC)
	Cmd_Pins_Msg_t pin_cmd = Cmd_Pins_Msg_t();
	pin_cmd.cell_no   = m_cell_no;
	pin_cmd.func_code = func_code;
	pin_cmd.pins_msg  = data_union;
	MsgFactory::instance()->cell_pusher(m_cell_no).send(pin_cmd, ZMQ_DONTWAIT);
#endif
}
#endif
void Dispatcher::m_send_struct(Sub_Cmd_Msg_t &data_union)
{
    char topic[32] = {'\0'};
    sprintf(topic, TOPIC_CELL_FORMAT_STR, m_cell_no);

    Cmd_Cell_Msg_t msg_struct = Cmd_Cell_Msg_t();
    msg_struct.func_code        = m_func_code;
    msg_struct.cell_msg.sub_msg = data_union;
    MsgFactory::instance()->cab_puber().send(topic, msg_struct, ZMQ_DONTWAIT);
}

#ifdef AUX_VOLTAGE_ACQ
void Dispatcher::m_send_struct(Cmd_Aux_Param_t &data)
{
	MsgFactory::instance()->aux_param_pusher(m_cell_no).send(data, ZMQ_DONTWAIT);
}
#endif

void Dispatcher::m_send_pins(int sub_code, int val)
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


