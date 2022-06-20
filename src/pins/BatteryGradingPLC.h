#ifndef BATTERYGRADINGPLC_H
#define BATTERYGRADINGPLC_H

#include <thread>
#include <string>

#include "ModbusTcpClient.h"

#ifdef MITSUBISHI_PLC 

#define FX5U_D_REG_OFFSET		0			// D寄存器内存偏移地址
#define FX5U_M_REG_OFFSET		8192		// M寄存器内存偏移地址


class BatteryGradingPLC {

private:
	enum WorkFlowStateMachine {				// 业务流程状态枚举
		WAIT_EVENT_MSG = 0,					// 等待消息或事件
		BG_START,							// 分容开始
		WAIT_PLC_PUT_INTO_READY,			// 等待PLC入库信号
		WMS_PUTINTO_COMPLETED,				// WMS入库完成
		WAIT_PALLET_IN_PLACE,				// 等待PLC托盘到位信号
		EXECUTE_PRESS_ETAB,					// 夹极耳
		WAIT_PRESSING_ETAB,					// 等待夹极耳完成
		EXECUTE_RELEASE_ETAB,				// 松开极耳
		WAIT_RELEASING_ETAB,				// 等待极耳松开
		WMS_TAKE_OUT_COMPLETED,				// WMS出库完成
		FIRE_EXTINGUISHER_SYSTEM_START,		// 消防系统启动
	};

public:
	BatteryGradingPLC() {}
    BatteryGradingPLC(int first_cell_no, int second_cell_no);
    BatteryGradingPLC& operator= (const BatteryGradingPLC& obj);

    std::thread run();

private:
	ModbusTcpClient *m_modbus_client;
	bool             m_M511_520[20];		// R属性寄存器 M511~520
	uint16_t         m_D500;				// R属性寄存器 D500
	uint16_t         m_D501_509[9];			// R属性寄存器 D501~509
	float            m_D516_524[5];			// R属性寄存器 D516~524，float型D寄存地址不是连续，但内存地址是连续的(float占4个字节)
	bool             m_M551_560[10];		// R属性寄存器 M551~560
	uint16_t         m_D531_539[9];			// R属性寄存器 D531~539
	float            m_D546_554[5];			// R属性寄存器 D516~524

    int  m_1th_cell_no;
    int  m_2th_cell_no;
	WorkFlowStateMachine m_1th_wfsm = WAIT_EVENT_MSG;	// 一库业务流程状态机
	WorkFlowStateMachine m_2th_wfsm = WAIT_EVENT_MSG;	// 二库业务流程状态机

private:
	void work();
	int  statusUpdate();
	void messageHandle();
	void workFlowHandle_1th();
	void workFlowHandle_2th();
	void alarmDetectHandle();
};

#endif
#endif

