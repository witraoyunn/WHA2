#include "BatteryGradingPLC.h"
#include "Configuration.h"
#include "MsgFactory.h"
#include "FormatConvert.h"
#include "log4z.h"
#include "Type.h"

#include <stdint.h>

#ifdef MITSUBISHI_PLC 

#define D_REG_ADDR(x)			(x + FX5U_D_REG_OFFSET)
#define M_REG_ADDR(x)			(x + FX5U_M_REG_OFFSET)

using namespace std;


BatteryGradingPLC::BatteryGradingPLC(int first_cell_no, int second_cell_no)
    : m_1th_cell_no(first_cell_no), m_2th_cell_no(second_cell_no)
{
}

BatteryGradingPLC& BatteryGradingPLC::operator= (const BatteryGradingPLC& obj)
{
    if (this != &obj)
    {
        this->m_1th_cell_no = obj.m_1th_cell_no;
        this->m_2th_cell_no = obj.m_2th_cell_no;
    }

    return *this;
}

thread BatteryGradingPLC::run()
{
	return thread(&BatteryGradingPLC::work, this);
}

void BatteryGradingPLC::work()
{
	// 库位线程 消息通讯 (一个PLC控制2个库）
	try
	{
		MsgFactory::instance()->create_bg_plc_puber(m_1th_cell_no, Configuration::instance()->int_bgplc_puber_socket(m_1th_cell_no));
		MsgFactory::instance()->create_bg_plc_puber(m_2th_cell_no, Configuration::instance()->int_bgplc_puber_socket(m_2th_cell_no));
		MsgFactory::instance()->create_bg_plc_puller(m_1th_cell_no, Configuration::instance()->int_bgplc_puller_socket(m_1th_cell_no));
		MsgFactory::instance()->create_bg_plc_puller(m_2th_cell_no, Configuration::instance()->int_bgplc_puller_socket(m_2th_cell_no));
	}
	catch (zmq::error_t &e)
	{
		LOGFMTE("Create ZMQ communication error. code: %d, description:%s", e.num(), e.what());
		string josn_str = FormatConvert::instance().alarm_to_string(-210, m_1th_cell_no);
		MsgFactory::instance()->cell_alarm_pusher(m_1th_cell_no).send(josn_str);
		exit(-210);
	}

	// 创建客户端
	try
	{
		string plc_ip = Configuration::instance()->bg_plc_ip(m_1th_cell_no);

		if (plc_ip != "")
		{
			m_modbus_client = new ModbusTcpClient(plc_ip.c_str(), MODBUS_TCP_DEFAULT_PORT);
		}
		else
		{
			LOGFMTW("BatteryGradingPLC::%s, Didn't get ip address of PLC from cell %d.", __FUNCTION__, m_1th_cell_no);
			string josn_str = FormatConvert::instance().alarm_to_string(-1100, m_1th_cell_no);
			MsgFactory::instance()->cell_alarm_pusher(m_1th_cell_no).send(josn_str);
			exit(-1100);
		}
	}
	catch (int &e)
	{
		LOGFMTE("BatteryGradingPLC::%s, error code %d.", __FUNCTION__, e);
		exit(e);
	}

	for (;;)
	{
		// 连接PLC服务端
		try
		{		
			m_modbus_client->connect();
			LOGFMTT("Battery grading plc connect success.");
		}
		catch (int &e)
		{
			LOGFMTT("BatteryGradingPLC::%s, error code %d.", __FUNCTION__, e);
			string josn_str = FormatConvert::instance().alarm_to_string(-1101, m_1th_cell_no);
			MsgFactory::instance()->cell_alarm_pusher(m_1th_cell_no).send(josn_str);

			sleep(5);
			continue;
		}

		// PLC业务逻辑
		for (;;)
		{
			// 更新PLC的状态
			if (statusUpdate() < 0)  break;
			// 上位机命令处理
			messageHandle();
			// 库一业务流程处理
			workFlowHandle_1th();
			// 库二业务流程处理
			workFlowHandle_2th();
			// 告警检测和处理
			alarmDetectHandle();

			msleep(100);		
		}
	}

	// 销毁zmq通讯
	MsgFactory::instance()->destory_bg_plc_puber(m_1th_cell_no);
	MsgFactory::instance()->destory_bg_plc_puber(m_2th_cell_no);
	MsgFactory::instance()->destory_bg_plc_puller(m_1th_cell_no);
	MsgFactory::instance()->destory_bg_plc_puller(m_2th_cell_no);
}

// 读取PLC所有的R属性寄存器
int BatteryGradingPLC::statusUpdate()
{
	try
	{
		m_modbus_client->read(M_REG_ADDR(511), m_M511_520, sizeof(m_M511_520)/sizeof(m_M511_520[0]));
		m_modbus_client->read(D_REG_ADDR(500), &m_D500,    1);
		m_modbus_client->read(D_REG_ADDR(501), m_D501_509, sizeof(m_D501_509)/sizeof(m_D501_509[0]));
		m_modbus_client->read(D_REG_ADDR(516), m_D516_524, sizeof(m_D516_524)/sizeof(m_D516_524[0]));
		m_modbus_client->read(M_REG_ADDR(551), m_M551_560, sizeof(m_M551_560)/sizeof(m_M551_560[0]));
		m_modbus_client->read(D_REG_ADDR(531), m_D531_539, sizeof(m_D531_539)/sizeof(m_D531_539[0]));
		m_modbus_client->read(D_REG_ADDR(546), m_D546_554, sizeof(m_D546_554)/sizeof(m_D546_554[0]));
	}
	catch (int &e)
	{
		LOGFMTW("BatteryGradingPLC::%s, error code %d.", __FUNCTION__, e);
		// 推送告警：PLC网络连接异常
		string josn_str = FormatConvert::instance().alarm_to_string(-1102, m_1th_cell_no);
		MsgFactory::instance()->cell_alarm_pusher(m_1th_cell_no).send(josn_str);
		return -1;
	}

	return 0;
}

// 处理上位机下发的指令
void BatteryGradingPLC::messageHandle()
{
	// 一库
	CMD_BFG_PLC_DATA_STRUCT recv_data_1th;

	if (MsgFactory::instance()->bg_plc_puller(m_1th_cell_no).recevie(recv_data_1th, ZMQ_DONTWAIT))
	{
		switch (recv_data_1th.func_code)
		{
			// 分容开始命令
			case BG_PLC_START_CMD:
				m_1th_wfsm = BG_START;
				break;
			
			// 物流系统库位入料通知
			case BG_WMS_PUT_INTO_NOTIFY:
				m_1th_wfsm = WMS_PUTINTO_COMPLETED;
				break;
			
			// 夹极耳命令
			case BG_PLC_PRESS_ETAB_CMD:
				m_1th_wfsm = EXECUTE_PRESS_ETAB;
				break;

			// 松开极耳命令
			case BG_PLC_RELEASE_ETAB_CMD:
				m_1th_wfsm = EXECUTE_RELEASE_ETAB;
				break;
					
			// 物流系统库位托盘取走通知
			case BG_WMS_TAKE_OUT_NOTIFY:
				m_1th_wfsm = WMS_TAKE_OUT_COMPLETED;
				break;
			
			default:
				break;
		}

		// 应答
		string josn_str = FormatConvert::instance().reply_to_string(recv_data_1th.cell_no, -1, recv_data_1th.func_code, -1, true);
		MsgFactory::instance()->cell_to_ext_reply_pusher(m_1th_cell_no).send(josn_str);
	}

	// 二库
	CMD_BFG_PLC_DATA_STRUCT recv_data_2th;
	
	if (MsgFactory::instance()->bg_plc_puller(m_2th_cell_no).recevie(recv_data_2th, ZMQ_DONTWAIT))
	{
		switch (recv_data_2th.func_code)
		{
			// 分容开始命令
			case BG_PLC_START_CMD:
				m_2th_wfsm = BG_START;
				break;
			
			// 物流系统库位入料通知
			case BG_WMS_PUT_INTO_NOTIFY:
				m_2th_wfsm = WMS_PUTINTO_COMPLETED;
				break;
			
			// 夹极耳命令
			case BG_PLC_PRESS_ETAB_CMD:
				m_2th_wfsm = EXECUTE_PRESS_ETAB;
				break;

			// 松开极耳命令
			case BG_PLC_RELEASE_ETAB_CMD:
				m_2th_wfsm = EXECUTE_RELEASE_ETAB;
				break;
					
			// 物流系统库位托盘取走通知
			case BG_WMS_TAKE_OUT_NOTIFY:
				m_2th_wfsm = WMS_TAKE_OUT_COMPLETED;
				break;
			
			default:
				break;
		}

		// 应答
		string josn_str = FormatConvert::instance().reply_to_string(recv_data_2th.cell_no, -1, recv_data_2th.func_code, -1, true);
		MsgFactory::instance()->cell_to_ext_reply_pusher(m_2th_cell_no).send(josn_str);
	}
}

// 库一处理业务流程
void BatteryGradingPLC::workFlowHandle_1th()
{
	switch (m_1th_wfsm)
	{
		// 分容开始，M507=1 初始化PLC
		case BG_START:
			m_modbus_client->write(M_REG_ADDR(507), true);
			m_1th_wfsm = WAIT_PLC_PUT_INTO_READY;
			break;

		// 等待PLC的入库信号
		case WAIT_PLC_PUT_INTO_READY:
		{
			bool status = false;
			
			// 入料准备就绪 M517=1
			if (m_M511_520[6] == 1)
			{
				status = true;
				// 通知MES系统
				string josn_str = FormatConvert::instance().notify_to_string(m_1th_cell_no, BG_PLC_NOTIFY, BG_PLC_PUT_INTO_NOTIFY, status);
				MsgFactory::instance()->cell_to_ext_reply_pusher(m_1th_cell_no).send(josn_str);
				// 货叉正在放入托盘 M506=1 
				m_modbus_client->write(M_REG_ADDR(506), true);
				m_1th_wfsm = WAIT_EVENT_MSG;
			}
			else
			{
				// 通知MES系统
				string josn_str = FormatConvert::instance().notify_to_string(m_1th_cell_no, BG_PLC_NOTIFY, BG_PLC_PUT_INTO_NOTIFY, status);
				MsgFactory::instance()->cell_to_ext_reply_pusher(m_1th_cell_no).send(josn_str);
			}
			break;
		}
		
		// WMS入库完成
		case WMS_PUTINTO_COMPLETED:
			// 货叉放入托盘动作完成 M506=0 
			m_modbus_client->write(M_REG_ADDR(506), false);
			m_1th_wfsm = WAIT_PALLET_IN_PLACE;
			break;

		// 检测托盘到位和货叉到位
		case WAIT_PALLET_IN_PLACE:
			// 判断PLC是否为自动模式 M511=0
			if (m_M511_520[0] == 0)
			{
				// 启动就绪 M518=1
				if (m_M511_520[7] == 1)
				{
					// 通知MES系统
					string josn_str = FormatConvert::instance().notify_to_string(m_1th_cell_no, BG_PLC_NOTIFY, BG_PLC_CELL_PALLET_READY, 1);
					MsgFactory::instance()->cell_to_ext_reply_pusher(m_1th_cell_no).send(josn_str);
					m_1th_wfsm = WAIT_EVENT_MSG;
				}
			}
			// 推送告警: PLC处于手动模式
			else
			{
				string josn_str = FormatConvert::instance().alarm_to_string(-1106, m_1th_cell_no);
				MsgFactory::instance()->cell_alarm_pusher(m_1th_cell_no).send(josn_str);
				m_1th_wfsm = WAIT_EVENT_MSG;
			}
			break;

		// 执行夹极耳, M503=1
		case EXECUTE_PRESS_ETAB:
			m_modbus_client->write(M_REG_ADDR(503), true);
			m_1th_wfsm = WAIT_PRESSING_ETAB;
			break;

		// 等待夹极耳动作完成
		case WAIT_PRESSING_ETAB:		
			// M515=1 && D502=0
			if ((m_M511_520[4] == 1) && (m_D501_509[1] == 0))
			{
				// 通知MES系统
				string josn_str = FormatConvert::instance().notify_to_string(m_1th_cell_no, BG_PLC_NOTIFY, BG_PLC_CELL_START_READY, 1);
				MsgFactory::instance()->cell_to_ext_reply_pusher(m_1th_cell_no).send(josn_str);
				m_1th_wfsm = WAIT_EVENT_MSG;
			}
			break;

		// 松开极耳   M504=1
		case EXECUTE_RELEASE_ETAB:
			m_modbus_client->write(M_REG_ADDR(504), true);
			m_1th_wfsm = WAIT_RELEASING_ETAB;
			break;

		// 等待极耳松开 M516=1
		case WAIT_RELEASING_ETAB:
			if (m_M511_520[5] == 1)
			{				
				// 通知MES系统
				string josn_str = FormatConvert::instance().notify_to_string(m_1th_cell_no, BG_PLC_NOTIFY, BG_PLC_ETAB_STATUS_NOTIFY, 1);
				MsgFactory::instance()->cell_to_ext_reply_pusher(m_1th_cell_no).send(josn_str);
				// 货叉正在取走托盘 M508=1
				m_modbus_client->write(M_REG_ADDR(508), true);
				m_1th_wfsm = WAIT_EVENT_MSG;
			}
			break;

		// 货叉已将托盘取走 M508=0
		case WMS_TAKE_OUT_COMPLETED:
			m_modbus_client->write(M_REG_ADDR(508), false);
			m_1th_wfsm = WAIT_EVENT_MSG;
			break;

		// 启动消防系统
		case FIRE_EXTINGUISHER_SYSTEM_START:
			// 启动消防 M505=1
			m_modbus_client->write(M_REG_ADDR(505), true);
			// 通知1230消防系统
			
			break;
			
		// 等待事件或消息
		case WAIT_EVENT_MSG:
		// 初始或未知状态
		default:
			break;
	}
}

// 库二处理业务流程
void BatteryGradingPLC::workFlowHandle_2th()
{
	switch (m_2th_wfsm)
	{
		// 分容开始，M547=1 初始化PLC
		case BG_START:
			m_modbus_client->write(M_REG_ADDR(547), true);
			m_2th_wfsm = WAIT_PLC_PUT_INTO_READY;
			break;

		// 等待PLC的入库信号
		case WAIT_PLC_PUT_INTO_READY:
		{
			bool status = false;
			
			// 入料准备就绪 M557=1
			if (m_M551_560[6] == 1)
			{
				status = true;
				// 通知MES系统
				string josn_str = FormatConvert::instance().notify_to_string(m_2th_cell_no, BG_PLC_NOTIFY, BG_PLC_PUT_INTO_NOTIFY, status);
				MsgFactory::instance()->cell_to_ext_reply_pusher(m_2th_cell_no).send(josn_str);
				// 货叉正在放入托盘 M546=1 
				m_modbus_client->write(M_REG_ADDR(546), true);
				m_2th_wfsm = WAIT_EVENT_MSG;
			}
			else
			{
				// 通知MES系统
				string josn_str = FormatConvert::instance().notify_to_string(m_2th_cell_no, BG_PLC_NOTIFY, BG_PLC_PUT_INTO_NOTIFY, status);
				MsgFactory::instance()->cell_to_ext_reply_pusher(m_2th_cell_no).send(josn_str);
			}
			break;
		}
		
		// WMS入库完成
		case WMS_PUTINTO_COMPLETED:
			// 货叉放入托盘动作完成 M546=0 
			m_modbus_client->write(M_REG_ADDR(546), false);
			m_2th_wfsm = WAIT_PALLET_IN_PLACE;
			break;

		// 检测托盘到位和货叉到位
		case WAIT_PALLET_IN_PLACE:
			// 判断PLC是否为自动模式 M551=0
			if (m_M551_560[0] == 0)
			{
				// 启动就绪 M558=1
				if (m_M551_560[7] == 1)
				{
					// 通知MES系统
					string josn_str = FormatConvert::instance().notify_to_string(m_2th_cell_no, BG_PLC_NOTIFY, BG_PLC_CELL_PALLET_READY, 1);
					MsgFactory::instance()->cell_to_ext_reply_pusher(m_2th_cell_no).send(josn_str);
					m_2th_wfsm = WAIT_EVENT_MSG;
				}
			}
			// 推送告警: PLC处于手动模式
			else
			{
				string josn_str = FormatConvert::instance().alarm_to_string(-1106, m_2th_cell_no);
				MsgFactory::instance()->cell_alarm_pusher(m_2th_cell_no).send(josn_str);
				m_2th_wfsm = WAIT_EVENT_MSG;
			}
			break;

		// 执行夹极耳, M543=1
		case EXECUTE_PRESS_ETAB:
			m_modbus_client->write(M_REG_ADDR(543), true);
			m_2th_wfsm = WAIT_PRESSING_ETAB;
			break;

		// 等待夹极耳动作完成
		case WAIT_PRESSING_ETAB:		
			// M555=1 && D532=0
			if ((m_M551_560[4] == 1) && (m_D531_539[1] == 0))
			{
				// 通知MES系统
				string josn_str = FormatConvert::instance().notify_to_string(m_2th_cell_no, BG_PLC_NOTIFY, BG_PLC_CELL_START_READY, 1);
				MsgFactory::instance()->cell_to_ext_reply_pusher(m_2th_cell_no).send(josn_str);
				m_2th_wfsm = WAIT_EVENT_MSG;
			}
			break;

		// 松开极耳   M544=1
		case EXECUTE_RELEASE_ETAB:
			m_modbus_client->write(M_REG_ADDR(544), true);
			m_2th_wfsm = WAIT_RELEASING_ETAB;
			break;

		// 等待极耳松开 M556=1
		case WAIT_RELEASING_ETAB:
			if (m_M551_560[5] == 1)
			{				
				// 通知MES系统
				string josn_str = FormatConvert::instance().notify_to_string(m_2th_cell_no, BG_PLC_NOTIFY, BG_PLC_ETAB_STATUS_NOTIFY, 1);
				MsgFactory::instance()->cell_to_ext_reply_pusher(m_2th_cell_no).send(josn_str);
				// 货叉正在取走托盘 M548=1
				m_modbus_client->write(M_REG_ADDR(548), true);
				m_2th_wfsm = WAIT_EVENT_MSG;
			}
			break;

		// 货叉已将托盘取走 M548=0
		case WMS_TAKE_OUT_COMPLETED:
			m_modbus_client->write(M_REG_ADDR(548), false);
			m_2th_wfsm = WAIT_EVENT_MSG;
			break;

		// 启动消防系统
		case FIRE_EXTINGUISHER_SYSTEM_START:
			// 启动消防 M545=1
			m_modbus_client->write(M_REG_ADDR(545), true);
			// 通知1230消防系统
			
			break;
			
		// 等待事件或消息
		case WAIT_EVENT_MSG:
		// 初始或未知状态
		default:
			break;
	}
}

// 告警检测和处理
void BatteryGradingPLC::alarmDetectHandle()
{
	static uint16_t d500=0;
	static uint16_t d501=0, d502=0, d503=0;
	static uint16_t d531=0, d532=0, d533=0;

	// 整机报警
	if (m_D500 != 0)
	{
		uint16_t now = m_D500;
		uint16_t pre = d500;
		
		// 有3个报警类型
		for (int i=0; i<3; i++)
		{
			// 仅推送一次告警
			if (((now & 1) != 0) && ((pre & 1) == 0))
			{
				string josn_str = FormatConvert::instance().alarm_to_string(-1103 - i, m_1th_cell_no);
				MsgFactory::instance()->cell_alarm_pusher(m_1th_cell_no).send(josn_str);
			}
			now >>= 1;
			pre >>= 1;
		}
	}
	d500 = m_D500;
	
	// 一库一类报警 D501
	if (m_D501_509[0] != 0)
	{
		uint16_t now = m_D501_509[0];
		uint16_t pre = d501;
		
		// 有6个报警类型
		for (int i=0; i<6; i++)
		{
			// 仅推送一次告警
			if (((now & 1) != 0) && ((pre & 1) == 0))
			{
				string josn_str = FormatConvert::instance().alarm_to_string(-1120 - i, m_1th_cell_no);
				MsgFactory::instance()->cell_alarm_pusher(m_1th_cell_no).send(josn_str);
			}
			now >>= 1;
			pre >>= 1;
		}
	}
	d501 = m_D501_509[0];
	
	// 一库二类报警 D502
	if (m_D501_509[1] != 0)
	{
		uint16_t now = m_D501_509[1];
		uint16_t pre = d502;
		
		// 有12个报警类型
		for (int i=0; i<12; i++)
		{
			// 仅推送一次告警
			if (((now & 1) != 0) && ((pre & 1) == 0))
			{
				string josn_str = FormatConvert::instance().alarm_to_string(-1140 - i, m_1th_cell_no);
				MsgFactory::instance()->cell_alarm_pusher(m_1th_cell_no).send(josn_str);
			}
			now >>= 1;
			pre >>= 1;
		}
	}
	d502 = m_D501_509[1];
	
	// 一库三类报警 D503
	if (m_D501_509[2] != 0)
	{
		uint16_t now = m_D501_509[2];
		uint16_t pre = d503;
		
		// 有4个报警类型
		for (int i=0; i<4; i++)
		{
			// 仅推送一次告警
			if (((now & 1) != 0) && ((pre & 1) == 0))
			{
				string josn_str = FormatConvert::instance().alarm_to_string(-1160 - i, m_1th_cell_no);
				MsgFactory::instance()->cell_alarm_pusher(m_1th_cell_no).send(josn_str);
			}
			// 温度+烟雾告警保护
			if (((now & 1) != 0) && ((m_D501_509[0] & 0x0020) != 0))
			{
				// 启动消防
				m_1th_wfsm = FIRE_EXTINGUISHER_SYSTEM_START;
			}
			now >>= 1;
			pre >>= 1;
		}
	}
	d503 = m_D501_509[2];

	// 二库一类报警 D531
	if (m_D531_539[0] != 0)
	{
		uint16_t now = m_D531_539[0];
		uint16_t pre = d531;
		
		// 有6个报警类型
		for (int i=0; i<6; i++)
		{
			// 仅推送一次告警
			if (((now & 1) != 0) && ((pre & 1) == 0))
			{
				string josn_str = FormatConvert::instance().alarm_to_string(-1120 - i, m_2th_cell_no);
				MsgFactory::instance()->cell_alarm_pusher(m_2th_cell_no).send(josn_str);
			}
			now >>= 1;
			pre >>= 1;
		}
	}
	d531 = m_D531_539[0];
	
	// 二库二类报警 D532
	if (m_D531_539[1] != 0)
	{
		uint16_t now = m_D531_539[1];
		uint16_t pre = d532;
		
		// 有12个报警类型
		for (int i=0; i<12; i++)
		{
			// 仅推送一次告警
			if (((now & 1) != 0) && ((pre & 1) == 0))
			{
				string josn_str = FormatConvert::instance().alarm_to_string(-1140 - i, m_2th_cell_no);
				MsgFactory::instance()->cell_alarm_pusher(m_2th_cell_no).send(josn_str);
			}
			now >>= 1;
			pre >>= 1;
		}
	}
	d532 = m_D531_539[1];
	
	// 二库三类报警 D533
	if (m_D531_539[2] != 0)
	{
		uint16_t now = m_D531_539[2];
		uint16_t pre = d533;
		
		// 有4个报警类型
		for (int i=0; i<4; i++)
		{
			// 仅推送一次告警
			if (((now & 1) != 0) && ((pre & 1) == 0))
			{
				string josn_str = FormatConvert::instance().alarm_to_string(-1160 - i, m_2th_cell_no);
				MsgFactory::instance()->cell_alarm_pusher(m_2th_cell_no).send(josn_str);
			}
			// 温度+烟雾告警保护
			if (((now & 1) != 0) && ((m_D531_539[0] & 0x0020) != 0))
			{
				// 启动消防
				m_1th_wfsm = FIRE_EXTINGUISHER_SYSTEM_START;
			}
			now >>= 1;
			pre >>= 1;
		}
	}
	d533 = m_D531_539[2];
}
#endif
