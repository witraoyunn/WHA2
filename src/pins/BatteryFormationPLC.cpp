#include "BatteryFormationPLC.h"
#include "Configuration.h"
#include "MsgFactory.h"
#include "FormatConvert.h"
#include "log4z.h"
#include "Type.h"

#include <stdint.h>

#ifdef MITSUBISHI_PLC 

#define D_REG_ADDR(x)			(x + Q03_D_REG_OFFSET)
#define M_REG_ADDR(x)			(x + Q03_M_REG_OFFSET)


using namespace std;

BatteryFormationPLC::BatteryFormationPLC(int first_cell_no, int second_cell_no)
    : m_1th_cell_no(first_cell_no), m_2th_cell_no(second_cell_no)
{
}

BatteryFormationPLC& BatteryFormationPLC::operator= (const BatteryFormationPLC& obj)
{
    if (this != &obj)
    {
        this->m_1th_cell_no = obj.m_1th_cell_no;
        this->m_2th_cell_no = obj.m_2th_cell_no;
    }

    return *this;
}

thread BatteryFormationPLC::run()
{
	return thread(&BatteryFormationPLC::work, this);
}

void BatteryFormationPLC::work()
{
	// 库位线程 消息通讯
	try
	{
		MsgFactory::instance()->create_bf_plc_puber(m_1th_cell_no, Configuration::instance()->int_bfplc_puber_socket(m_1th_cell_no));
		MsgFactory::instance()->create_bf_plc_puber(m_2th_cell_no, Configuration::instance()->int_bfplc_puber_socket(m_2th_cell_no));
		MsgFactory::instance()->create_bf_plc_puller(m_1th_cell_no, Configuration::instance()->int_bfplc_puller_socket(m_1th_cell_no));
		MsgFactory::instance()->create_bf_plc_puller(m_2th_cell_no, Configuration::instance()->int_bfplc_puller_socket(m_2th_cell_no));
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
		string plc_ip = Configuration::instance()->bf_plc_ip(m_1th_cell_no);

		if (plc_ip != "")
		{
			m_modbus_client = new ModbusTcpClient(plc_ip.c_str(), MODBUS_TCP_DEFAULT_PORT);
		}
		else
		{
			LOGFMTW("BatteryFormationPLC::%s, Didn't get ip address of BG_PLC.", __FUNCTION__);
			string josn_str = FormatConvert::instance().alarm_to_string(-1200, m_1th_cell_no);
			MsgFactory::instance()->cell_alarm_pusher(m_1th_cell_no).send(josn_str);
			exit(-1200);
		}
	}
	catch (int &e)
	{
		LOGFMTE("BatteryFormationPLC::%s, error code %d.", __FUNCTION__, e);
		exit(e);
	}
	
	for (;;)
	{
		// 连接PLC服务端
		try
		{		
			m_modbus_client->connect();
			LOGFMTT("Battery formation plc connect success.");
		}
		catch (int &e)
		{
			LOGFMTE("BatteryFormationPLC::%s, error code %d.", __FUNCTION__, e);

			sleep(1);
			continue;
		}
			
		// PLC业务逻辑
		for (;;)
		{
			// 更新PLC的状态
			if (statusUpdate() < 0)  break;
			// 上位机命令处理
			messageHandle();
			// 业务流程处理
			workFlowHandle();
			// 告警检测和处理
			alarmDetectHandle();

			msleep(100);		
		}
	}
	
	// 销毁zmq通讯
	MsgFactory::instance()->destory_bf_plc_puber(m_1th_cell_no);
	MsgFactory::instance()->destory_bf_plc_puber(m_2th_cell_no);
	MsgFactory::instance()->destory_bf_plc_puller(m_1th_cell_no);
	MsgFactory::instance()->destory_bf_plc_puller(m_2th_cell_no);
}

bool BatteryFormationPLC::read_MReg(int addr)
{
	bool ret = false;
	m_modbus_client->read(M_REG_ADDR(608), &ret, 1);
	return ret;
}

// 读取PLC所有的R属性寄存器
int BatteryFormationPLC::statusUpdate()
{
	try
	{
		m_modbus_client->read(M_REG_ADDR(900), m_M900_905, sizeof(m_M900_905)/sizeof(m_M900_905[0]));
		m_modbus_client->read(M_REG_ADDR(910), m_M910_915, sizeof(m_M910_915)/sizeof(m_M910_915[0]));
		m_modbus_client->read(M_REG_ADDR(920), m_M920_921, sizeof(m_M920_921)/sizeof(m_M920_921[0]));
		m_modbus_client->read(D_REG_ADDR(400), m_D400_401, sizeof(m_D400_401)/sizeof(m_D400_401[0]));
		m_modbus_client->read(D_REG_ADDR(400), m_D404_405, sizeof(m_D404_405)/sizeof(m_D404_405[0]));
		m_modbus_client->read(D_REG_ADDR(400), m_D408_410, sizeof(m_D408_410)/sizeof(m_D408_410[0]));
		m_modbus_client->read(D_REG_ADDR(400), m_D413_430, sizeof(m_D413_430)/sizeof(m_D413_430[0]));
	}
	catch (int &e)
	{
		LOGFMTE("BatteryGradingPLC::%s, error code %d.", __FUNCTION__, e);
		// 推送告警：PLC网络连接异常
		string josn_str = FormatConvert::instance().alarm_to_string(-1202, m_1th_cell_no);
		MsgFactory::instance()->cell_alarm_pusher(m_1th_cell_no).send(josn_str);
		return -1;
	}

	return 0;
}

// 处理上位机下发的指令
void BatteryFormationPLC::messageHandle()
{
	// 一库
	CMD_BFG_PLC_DATA_STRUCT recv_data_1th;

	if (MsgFactory::instance()->bf_plc_puller(m_1th_cell_no).recevie(recv_data_1th, ZMQ_DONTWAIT))
	{
		switch (recv_data_1th.func_code)
		{
			// 化成开始命令
			case BF_PLC_START_CMD:
				m_1th_wfsm = BF_START;
				break;
			
			// 物流系统库位入料通知
			case BF_WMS_PUT_INTO_NOTIFY:
				m_1th_wfsm = WMS_READY_NOTIFY;
				break;

			// 物流系统空托盘到达
			case BF_WMS_PALLET_READY:
				m_1th_wfsm = WMS_EMPTY_PEALLET_READY;
				break;
						
			default:
				break;
		}

		// 应答
		string josn_str = FormatConvert::instance().reply_to_string(recv_data_1th.cell_no, -1, recv_data_1th.func_code, -1, true);
		MsgFactory::instance()->cell_to_ext_reply_pusher(m_1th_cell_no).send(josn_str);
	}
}

// 处理业务流程
void BatteryFormationPLC::workFlowHandle()
{
	switch (m_1th_wfsm)
	{
		// 化成开始，M608=1 M610=1
		case BF_START:
			if (read_MReg(608) && read_MReg(610))
			{
				// 下发托盘扫码指令 M619=1
				m_modbus_client->write(M_REG_ADDR(619), true);
				m_1th_wfsm = WAIT_PALLET_SCAN_CODE;
			}
			break;

		// 等待托盘扫码完成, M618=1
		case WAIT_PALLET_SCAN_CODE:
			if (read_MReg(618))
			{
				// 读取PLC的托盘扫码信息, D574~583
				uint16_t buf[10];
				m_modbus_client->read(D_REG_ADDR(574), buf, sizeof(buf)/sizeof(buf[0]));
				// 与WMS的托盘扫码信息进行比对
				
				// 通知MES系统
				string josn_str = FormatConvert::instance().notify_to_string(m_1th_cell_no, BF_PLC_NOTIFY, BF_SCAN_CODE_NOTIFY, 1);
				MsgFactory::instance()->cell_to_ext_reply_pusher(m_1th_cell_no).send(josn_str);
				m_1th_wfsm = CHECK_PLC_RUN_MODE;
			}
			break;
		
		// PLC模式检测
		case CHECK_PLC_RUN_MODE:
			// 判断PLC是否为自动模式 M901=1
			if (m_M900_905[1] != 1)
			{
				// 推送告警: PLC处于手动模式
				string josn_str = FormatConvert::instance().alarm_to_string(-1206, m_1th_cell_no);
				MsgFactory::instance()->cell_alarm_pusher(m_1th_cell_no).send(josn_str);
			}
			m_1th_wfsm = WAIT_EVENT_MSG;
			break;

		// WMS上料准备就绪
		case WMS_READY_NOTIFY:
			// 上位机ready M609=1
			m_modbus_client->write(M_REG_ADDR(609), true);
			// 开始电芯扫码 D490=1
			m_modbus_client->write(D_REG_ADDR(490), (uint16_t)1);
			m_1th_wfsm = WAIT_CELL_SCAN_CODE;
			break;

		// 等待电芯扫码完成 M620=1
		case WAIT_CELL_SCAN_CODE:
			if (read_MReg(620))
			{
				// 读取PLC的电芯扫码信息, D604~613
				uint16_t buf[10];
				m_modbus_client->read(D_REG_ADDR(604), buf, sizeof(buf)/sizeof(buf[0]));
				// 电芯所在夹具的序号
				uint16_t order;
				m_modbus_client->read(D_REG_ADDR(622), &order, 1);
				// 信息比对
				if (true)
				{
					// 电芯码有效 D621=3
					m_modbus_client->write(D_REG_ADDR(621), (uint16_t)3);
					// 绑定单个电芯信息，记忆该盘电芯信息
					// 绑定完成
				}
				else
				{
					// 电芯码无效 D621=2
					m_modbus_client->write(D_REG_ADDR(621), (uint16_t)2);
					m_1th_wfsm = WAIT_EVENT_MSG;
				}
			}
			break;

		// 单个电芯绑定完成
		case BOND_SINGLE_CELL_INFO:		
			// 通知PLC绑定完成 M623=1 D490=2
			m_modbus_client->write(M_REG_ADDR(623), true);
			m_modbus_client->write(D_REG_ADDR(490), (uint16_t)2);
			m_1th_wfsm = WAIT_EVENT_MSG;
			break;

		// 整盘电芯绑定完成
		case BOND_ALL_CELL_INFO:
		{
			// 通知PLC整个夹具上料完成，整盘电芯绑定完成 M625=1 D490=3
			m_modbus_client->write(M_REG_ADDR(625), true);
			m_modbus_client->write(D_REG_ADDR(490), (uint16_t)3);
			// 通知WMS电池抓取完成
			string josn_str = FormatConvert::instance().notify_to_string(m_1th_cell_no, BF_PLC_NOTIFY, BF_WMS_TAKE_OUT_PALLET, 1);
			MsgFactory::instance()->cell_to_ext_reply_pusher(m_1th_cell_no).send(josn_str);
			
			m_1th_wfsm = WAIT_HOT_COMP_READY;
			break;
		}

		// 等待PLC热压准备就绪
		case WAIT_HOT_COMP_READY:
			// 热压1准备就绪 M670=1
			if (read_MReg(670))
			{
				// 通知MES准备就绪
				
				m_1th_wfsm = WAIT_EVENT_MSG;
			}
			break;

		// 启动热压
		case START_HOT_COMPRESSING:
			// 上位机热压1准备就绪 M671=1
			m_modbus_client->write(M_REG_ADDR(671), true);
			m_1th_wfsm = WAIT_HOT_COMP_READY;
			break;

		// 等待热压1压力和温度达到条件
		case WAIT_PRES_TEMP_READY:
			// 热压1准备就绪 M672=1
			if (read_MReg(672))
			{
				// 通知MES
				
				m_1th_wfsm = WAIT_EVENT_MSG;
			}
			break;

		// 等待热压化成工艺完成
		case WAIT_BF_HOT_COMP_COMPLETE:
			//
			if (true)
			{
				// 上位机化成完成 M673=1
				m_modbus_client->write(M_REG_ADDR(673), true);
				m_1th_wfsm = WAIT_COLD_COMP_STARTING;
			}
			break;

		// 等待冷压启动
		case WAIT_COLD_COMP_STARTING:
			// 冷压中 M704=1
			if (read_MReg(704))
			{
				// 通知WMS
				string josn_str = FormatConvert::instance().notify_to_string(m_1th_cell_no, BF_PLC_NOTIFY, BF_COLD_COMP_STARTING, 1);
				MsgFactory::instance()->cell_to_ext_reply_pusher(m_1th_cell_no).send(josn_str);
				m_1th_wfsm = WAIT_COLD_COMP_COMPLETE;
			}
			break;
			
		// 等待冷压结束
		case WAIT_COLD_COMP_COMPLETE:
			// 冷压结束 M706=1
			if (read_MReg(706))
			{
				// 通知MES
				string josn_str = FormatConvert::instance().notify_to_string(m_1th_cell_no, BF_PLC_NOTIFY, BF_COLD_COMP_CMPLETED, 1);
				MsgFactory::instance()->cell_to_ext_reply_pusher(m_1th_cell_no).send(josn_str);
				m_1th_wfsm = WAIT_TAKE_OUT_READY;
			}
			break;

		// 化成热压异常
		case BF_HOT_COMP_EX:
			m_1th_wfsm = WAIT_EVENT_MSG;
			break;

		// 等待化成下料准备就绪
		case WAIT_TAKE_OUT_READY:
			// 下料准备就绪 M638=1
			if (read_MReg(638))
			{
				// 通知WMS
				string josn_str = FormatConvert::instance().notify_to_string(m_1th_cell_no, BF_PLC_NOTIFY, BF_WMS_TAKE_OUT_READY, 1);
				MsgFactory::instance()->cell_to_ext_reply_pusher(m_1th_cell_no).send(josn_str);
				m_1th_wfsm = WAIT_EVENT_MSG;
			}
			break;

		// WMS空托盘到达
		case WMS_EMPTY_PEALLET_READY:
			// 托盘信息
			// 上位机下料准备就绪 M639=1
			m_modbus_client->write(M_REG_ADDR(639), true);
			
			break;

		// 下料电芯扫码
		case LAYING_OFF_CELL_SCAN_CODE:
			if (true)
			{
				// PLC扫码 D500=1
				m_modbus_client->write(D_REG_ADDR(500), (uint16_t)1);
				m_1th_wfsm = WAIT_LAYING_OFF_SCAN_CODE;
			}
			break;

		// 等待下料扫码完成 M650=1
		case WAIT_LAYING_OFF_SCAN_CODE:
			if (read_MReg(650))
			{
				// 读取PLC的电芯扫码信息, D634~643
				uint16_t buf[10];
				m_modbus_client->read(D_REG_ADDR(634), buf, sizeof(buf)/sizeof(buf[0]));
				// 电芯所在夹具的序号
				uint16_t order;
				m_modbus_client->read(D_REG_ADDR(652), &order, 1);
				// 判断电芯化成是否正常
				if (true)
				{
					// 通知PLC D651=3
					m_modbus_client->write(D_REG_ADDR(651), (uint16_t)3);
				}
				else
				{
					m_modbus_client->write(D_REG_ADDR(651), (uint16_t)2);
				}
				
				m_1th_wfsm = BOND_SINGLE_CELL_SN;
			}
			break;

		// 单个电芯所在位置和SN号绑定
		case BOND_SINGLE_CELL_SN:
			// 通知PLC绑定完成 M653=1 D500=2
			m_modbus_client->write(M_REG_ADDR(653), true);
			m_modbus_client->write(D_REG_ADDR(500), (uint16_t)2);
			// 判断所有电芯绑定是否完成
			if (true)
			{
				m_1th_wfsm = BOND_ALL_CELL_SN_COMPLETE;
			}
			else
			{
				// 待讨论
				m_1th_wfsm = WAIT_EVENT_MSG;
			}

		// 所有电芯所在位置与SN号绑定完成
		case BOND_ALL_CELL_SN_COMPLETE:
			// 通知PLC绑定完成 M655=1 D500=2
			m_modbus_client->write(M_REG_ADDR(655), true);
			m_modbus_client->write(D_REG_ADDR(500), (uint16_t)3);
			m_1th_wfsm = WMS_TAKE_OUT_PEALLET;
			break;

		// 通知WMS取走化成后的托盘
		case WMS_TAKE_OUT_PEALLET:
			// 需明确通知信息
			m_1th_wfsm = WAIT_EVENT_MSG;
			break;
			
		// 等待事件或消息
		case WAIT_EVENT_MSG:
		// 初始或未知状态
		default:
			break;
	}
}

// 告警检测和处理
void BatteryFormationPLC::alarmDetectHandle()
{
	static bool     m900_905[6]  = {false, false, false, false, false, false};
	static uint16_t d400_401[2]  = {0, 0};
	static uint16_t d404_405[2]  = {0, 0};
	static uint16_t d408_410[3]  = {0, 0, 0};
	static uint16_t d413_430[18] = {0};

	// 整机报警
	for (uint32_t i=0; i<sizeof(m_M900_905); ++i)
	{
		// 仅推送一次告警
		if ((m_M900_905[i] != 0) && (m900_905[i] == 0))
		{
			if (i == 2)  continue;
			string josn_str = FormatConvert::instance().alarm_to_string(-1203 - i, m_1th_cell_no);
			MsgFactory::instance()->cell_alarm_pusher(m_1th_cell_no).send(josn_str);
		}
		
		// M904 火灾报警处理
		if (i == 4)
		{
			m_1th_wfsm = BF_HOT_COMP_EX;
		}
		
		m900_905[i] = m_M900_905[i];
	}
	
	// 上料故障 D400 D401
	for (uint32_t i=0; i<sizeof(m_D400_401)/sizeof(m_D400_401[0]); ++i)
	{
		if (m_D400_401[i] != 0)
		{
			uint16_t now = m_D400_401[i];
			uint16_t pre = d400_401[i];
			
			// 有16个报警类型
			for (int j=0; j<16; ++j)
			{
				// 仅推送一次告警
				if (((now & 1) != 0) && ((pre & 1) == 0))
				{
					string josn_str = FormatConvert::instance().alarm_to_string(-1210 - (i*16 + j), m_1th_cell_no);
					MsgFactory::instance()->cell_alarm_pusher(m_1th_cell_no).send(josn_str);
				}
				now >>= 1;
				pre >>= 1;
			}
		}
		d400_401[i] = m_D400_401[i];
	}
	
	// 下料故障 D404 D405
	for (uint32_t i=0; i<sizeof(m_D404_405)/sizeof(m_D404_405[0]); ++i)
	{
		if (m_D404_405[i] != 0)
		{
			uint16_t now = m_D404_405[i];
			uint16_t pre = d404_405[i];
			
			// 有16个报警类型
			for (int j=0; j<16; ++j)
			{
				// 仅推送一次告警
				if (((now & 1) != 0) && ((pre & 1) == 0))
				{
					string josn_str = FormatConvert::instance().alarm_to_string(-1240 - (i*16 + j), m_1th_cell_no);
					MsgFactory::instance()->cell_alarm_pusher(m_1th_cell_no).send(josn_str);
				}
				now >>= 1;
				pre >>= 1;
			}
		}
		d404_405[i] = m_D404_405[i];
	}

	// 行车及冷热压故障
	for (uint32_t i=0; i<sizeof(m_D408_410)/sizeof(m_D408_410[0]); ++i)
	{
		if (m_D408_410[i] != 0)
		{
			uint16_t now = m_D408_410[i];
			uint16_t pre = d408_410[i];
			
			// 有16个报警类型
			for (int j=0; j<16; ++j)
			{
				// 仅推送一次告警
				if (((now & 1) != 0) && ((pre & 1) == 0))
				{
					string josn_str = FormatConvert::instance().alarm_to_string(-1270 - (i*16 + j), m_1th_cell_no);
					MsgFactory::instance()->cell_alarm_pusher(m_1th_cell_no).send(josn_str);
				}
				now >>= 1;
				pre >>= 1;
			}

			// 故障处理
			m_1th_wfsm = BF_HOT_COMP_EX;
		}
		d408_410[i] = m_D408_410[i];
	}
	
	// 温度异常
	for (uint32_t i=0; i<sizeof(m_D413_430)/sizeof(m_D413_430[0]); ++i)
	{
		if (m_D413_430[i] != 0)
		{
			uint16_t now = m_D413_430[i];
			uint16_t pre = d413_430[i];
			
			// 有16个报警类型
			for (int j=0; j<16; ++j)
			{
				// 仅推送一次告警
				if (((now & 1) != 0) && ((pre & 1) == 0))
				{
					string josn_str = FormatConvert::instance().alarm_to_string(-1318 - (i*16 + j), m_1th_cell_no);
					MsgFactory::instance()->cell_alarm_pusher(m_1th_cell_no).send(josn_str);
				}
				now >>= 1;
				pre >>= 1;
			}
		}
		d413_430[i] = m_D413_430[i];
	}
}
#endif
