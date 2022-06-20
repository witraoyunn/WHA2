#include "log4z.h"
#include "PinsGroup_AXM.h"
#include "ReplyHandler.h"


using namespace std;

#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
/******************************write*******************************/
//库位设置
#define XM_PLC_CELL_SETTING_ADDR(x)		(100 + ((x>4)? x:(x-1))) 	//库位动作设置寄存器, x为库位号			
#define	CELL_TRAY_DOWN_OFFSET			0		//托盘下降
#define	CELL_TRAY_UP_OFFSET				1		//托盘上升
#define AUTO_OPT_MODE_OFFSET			2		//化成模式
#define MAINTENANCE_MODE_OFFSET			3		//维护模式
#define CRANE_FORK_BUSY_OFFSET			4		//Crane Fork Busy
#define	HIGH_VAC_ON_OFFSET				5		//启动高真空
#define BREAK_VAC_ON_OFFSET				6		//破真空启动
#define FIRE_DOOR_UP_OFFSET				7		//消防门上升
#define FIRE_DOOR_DOWN_OFFSET			8		//消防门下降
#define LEAK_RATIO_TEST_OFFSET			10		//漏率测试
#define CELL_FAN_ON_OFFSET				11		//启动风扇
#define VAC_BLOCK_TEST_OFFSET			12		//启动堵塞测试
#define STEP_PARA_WRITE_IN_OFFSET		13		//工步参数写入系统完成
#define TOOLS_POWER_ON_OFFSET			14		//工装取电控制
//#define ALARM_RESET_OFFSET			15		//PLC异常重置信号

//PLC心跳
#define PLC_HEARTBEAT_CYCLE_ADDR		295

//系统三色灯
#define XM_PLC_TRICOLOR_LAMP_ADDR		296
#define SYS_RED_LAMP_OFFSET				0
#define SYS_YELLOW_LAMP_OFFSET			1
#define SYS_GREEN_LAMP_OFFSET			2
#define SYS_BUZZER_OFFSET				3

//库位OTP2
#define XM_PLC_CELL_OTP2_ADDR			298
#define CELL1_OPT2_OFFSET				0
#define CELL2_OPT2_OFFSET				1
#define CELL3_OPT2_OFFSET				2
#define CELL4_OPT2_OFFSET				3
#define CELL5_OPT2_OFFSET				5
#define CELL6_OPT2_OFFSET				6
#define CELL7_OPT2_OFFSET				7
#define CELL8_OPT2_OFFSET				8

//库位OTP1
#define XM_PLC_CELL_OTP1_ADDR			300
#define CELL1_OPT1_OFFSET				0
#define CELL2_OPT1_OFFSET				1
#define CELL3_OPT1_OFFSET				2
#define CELL4_OPT1_OFFSET				3
#define CELL5_OPT1_OFFSET				5
#define CELL6_OPT1_OFFSET				6
#define CELL7_OPT1_OFFSET				7
#define CELL8_OPT1_OFFSET				8

//库位校准模式
#define XM_PLC_CELL_CALIB_ADDR			302
#define CELL1_CALIB_OFFSET				0
#define CELL2_CALIB_OFFSET				1
#define CELL3_CALIB_OFFSET				2
#define CELL4_CALIB_OFFSET				3
#define CELL5_CALIB_OFFSET				5
#define CELL6_CALIB_OFFSET				6
#define CELL7_CALIB_OFFSET				7
#define CELL8_CALIB_OFFSET				8
#define HMI_ALARM_RESET_OFFSET			15

//库位启动消防
#define XM_PLC_CELL_FIRE_CTL_ADDR		304
#define CELL1_FIRE_CTL_OFFSET			0
#define CELL2_FIRE_CTL_OFFSET			1
#define CELL3_FIRE_CTL_OFFSET			2
#define CELL4_FIRE_CTL_OFFSET			3
#define CELL5_FIRE_CTL_OFFSET			5
#define CELL6_FIRE_CTL_OFFSET			6
#define CELL7_FIRE_CTL_OFFSET			7
#define CELL8_FIRE_CTL_OFFSET			8

//负压控制1
//#define XM_PLC_LOW_VAC_SET_ADDR			364		//无
//#define XM_PLC_HIGH_VAC_SET_ADDR			365		//无
//#define XM_PLC_LOW_VAC_ERR_VAL_ADDR		366		//无
//真空误差值设定
#define XM_PLC_VAC_ERR_SET_ADDR				367
//#define XM_PLC_LOW_VAC_ALARM_ADDR			368
//#define XM_PLC_HIGH_VAC_ALARM_ADDR		369
//#define XM_PLC_LOW_VAC_ALARM_TIME_ADDR	370
#define XM_PLC_BREAK_VAC_SET_ADDR			372
#define XM_PLC_KEEP_VAC_TEST_SET_ADDR		373

//负压控制2
/*
#define XM_PLC_CELL1_VAC_SET_ADDR			380
#define XM_PLC_CELL1_BREAK_VAC_SET_ADDR		381
#define XM_PLC_CELL2_VAC_SET_ADDR			382
#define XM_PLC_CELL2_BREAK_VAC_SET_ADDR		383
#define XM_PLC_CELL3_VAC_SET_ADDR			384
#define XM_PLC_CELL3_BREAK_VAC_SET_ADDR		385
#define XM_PLC_CELL4_VAC_SET_ADDR			386
#define XM_PLC_CELL4_BREAK_VAC_SET_ADDR		387
#define XM_PLC_CELL5_VAC_SET_ADDR			390
#define XM_PLC_CELL5_BREAK_VAC_SET_ADDR		391
#define XM_PLC_CELL6_VAC_SET_ADDR			392
#define XM_PLC_CELL6_BREAK_VAC_SET_ADDR		393
#define XM_PLC_CELL7_VAC_SET_ADDR			394
#define XM_PLC_CELL7_BREAK_VAC_SET_ADDR		395
#define XM_PLC_CELL8_VAC_SET_ADDR			396
#define XM_PLC_CELL8_BREAK_VAC_SET_ADDR		397
*/

//库位轻烟持续时间
//#define XM_PLC_LIGHT_SMOKE_TIME_ADDR		388

//库位浓烟持续时间
//#define XM_PLC_THICK_SMOKE_TIME_ADDR		389

//库位当前工步号
#define XM_PLC_CELL1_STEP_NO_ADDR			5200
#define XM_PLC_CELL2_STEP_NO_ADDR			5201
#define XM_PLC_CELL3_STEP_NO_ADDR			5202
#define XM_PLC_CELL4_STEP_NO_ADDR			5203
#define XM_PLC_CELL5_STEP_NO_ADDR			5205
#define XM_PLC_CELL6_STEP_NO_ADDR			5206
#define XM_PLC_CELL7_STEP_NO_ADDR			5207
#define XM_PLC_CELL8_STEP_NO_ADDR			5208

//库位保压设定值
#define XM_PLC_CELL1_KEEP_VAC_SET_ADDR		5300
#define XM_PLC_CELL2_KEEP_VAC_SET_ADDR		5301
#define XM_PLC_CELL3_KEEP_VAC_SET_ADDR		5302
#define XM_PLC_CELL4_KEEP_VAC_SET_ADDR		5303
#define XM_PLC_CELL5_KEEP_VAC_SET_ADDR		5305
#define XM_PLC_CELL6_KEEP_VAC_SET_ADDR		5306
#define XM_PLC_CELL7_KEEP_VAC_SET_ADDR		5307
#define XM_PLC_CELL8_KEEP_VAC_SET_ADDR		5308

//库位保压泄露率设定值
#define XM_PLC_CELL1_LEAKRATE_SET_ADDR		5310
#define XM_PLC_CELL2_LEAKRATE_SET_ADDR		5311
#define XM_PLC_CELL3_LEAKRATE_SET_ADDR		5312
#define XM_PLC_CELL4_LEAKRATE_SET_ADDR		5313
#define XM_PLC_CELL5_LEAKRATE_SET_ADDR		5315
#define XM_PLC_CELL6_LEAKRATE_SET_ADDR		5316
#define XM_PLC_CELL7_LEAKRATE_SET_ADDR		5317
#define XM_PLC_CELL8_LEAKRATE_SET_ADDR		5318


//库位保压稳定时间
#define XM_PLC_CELL1_KEEP_VAC_STABLE_TIME_ADDR		5320
#define XM_PLC_CELL2_KEEP_VAC_STABLE_TIME_ADDR		5321
#define XM_PLC_CELL3_KEEP_VAC_STABLE_TIME_ADDR		5322
#define XM_PLC_CELL4_KEEP_VAC_STABLE_TIME_ADDR		5323
#define XM_PLC_CELL5_KEEP_VAC_STABLE_TIME_ADDR		5325
#define XM_PLC_CELL6_KEEP_VAC_STABLE_TIME_ADDR		5326
#define XM_PLC_CELL7_KEEP_VAC_STABLE_TIME_ADDR		5327
#define XM_PLC_CELL8_KEEP_VAC_STABLE_TIME_ADDR		5328

//库位保压时间
#define XM_PLC_CELL1_KEEP_VAC_TIME_ADDR		5340
#define XM_PLC_CELL2_KEEP_VAC_TIME_ADDR		5341
#define XM_PLC_CELL3_KEEP_VAC_TIME_ADDR		5342
#define XM_PLC_CELL4_KEEP_VAC_TIME_ADDR		5343
#define XM_PLC_CELL5_KEEP_VAC_TIME_ADDR		5345
#define XM_PLC_CELL6_KEEP_VAC_TIME_ADDR		5346
#define XM_PLC_CELL7_KEEP_VAC_TIME_ADDR		5347
#define XM_PLC_CELL8_KEEP_VAC_TIME_ADDR		5348

//库位高真空值
#define XM_PLC_CELL1_HIGH_VAC_SET_ADDR		5380
#define XM_PLC_CELL2_HIGH_VAC_SET_ADDR		5381
#define XM_PLC_CELL3_HIGH_VAC_SET_ADDR		5382
#define XM_PLC_CELL4_HIGH_VAC_SET_ADDR		5383
#define XM_PLC_CELL5_HIGH_VAC_SET_ADDR		5385
#define XM_PLC_CELL6_HIGH_VAC_SET_ADDR		5386
#define XM_PLC_CELL7_HIGH_VAC_SET_ADDR		5387
#define XM_PLC_CELL8_HIGH_VAC_SET_ADDR		5388

//库位抽真空超时时间
#define XM_PLC_CELL1_PUMP_VAC_TIMEOUT_ADDR		5400
#define XM_PLC_CELL2_PUMP_VAC_TIMEOUT_ADDR		5401
#define XM_PLC_CELL3_PUMP_VAC_TIMEOUT_ADDR		5402
#define XM_PLC_CELL4_PUMP_VAC_TIMEOUT_ADDR		5403
#define XM_PLC_CELL5_PUMP_VAC_TIMEOUT_ADDR		5405
#define XM_PLC_CELL6_PUMP_VAC_TIMEOUT_ADDR		5406
#define XM_PLC_CELL7_PUMP_VAC_TIMEOUT_ADDR		5407
#define XM_PLC_CELL8_PUMP_VAC_TIMEOUT_ADDR		5408

//库位泄真空超时时间
#define XM_PLC_CELL1_LEAK_VAC_TIMEOUT_ADDR		5420
#define XM_PLC_CELL2_LEAK_VAC_TIMEOUT_ADDR		5421
#define XM_PLC_CELL3_LEAK_VAC_TIMEOUT_ADDR		5422
#define XM_PLC_CELL4_LEAK_VAC_TIMEOUT_ADDR		5423
#define XM_PLC_CELL5_LEAK_VAC_TIMEOUT_ADDR		5425
#define XM_PLC_CELL6_LEAK_VAC_TIMEOUT_ADDR		5426
#define XM_PLC_CELL7_LEAK_VAC_TIMEOUT_ADDR		5427
#define XM_PLC_CELL8_LEAK_VAC_TIMEOUT_ADDR		5428

//库位泄真空目标值
#define XM_PLC_CELL1_LEAK_VAC_SET_ADDR		5440
#define XM_PLC_CELL2_LEAK_VAC_SET_ADDR		5441
#define XM_PLC_CELL3_LEAK_VAC_SET_ADDR		5442
#define XM_PLC_CELL4_LEAK_VAC_SET_ADDR		5443
#define XM_PLC_CELL5_LEAK_VAC_SET_ADDR		5445
#define XM_PLC_CELL6_LEAK_VAC_SET_ADDR		5446
#define XM_PLC_CELL7_LEAK_VAC_SET_ADDR		5447
#define XM_PLC_CELL8_LEAK_VAC_SET_ADDR		5448


/*******************************************read*******************************************/
//针床库位动作查询
#define XM_PLC_CELL_SETTING_GET_ADDR(x) 	(130 + ((x>4)? x:(x-1))) 	//库位动作查询寄存器, x为库位号			

//系统异常告警
#define XM_PLC_SYS_ABNORMAL_ALARM_ADDR		160
#define AC1_OVER_VOLTAGE_OFFSET				0
#define AC1_LOW_VOLTAGE_OFFSET				1
#define AC1_OVER_CURRENT_OFFSET				2
#define LOW_PRESSURE_OFFSET					3
#define CAB_TEMP_ABNORMAL_OFFSET			8
#define AC2_OVER_VOLTAGE_OFFSET				10
#define AC2_LOW_VOLTAGE_OFFSET				11
#define AC2_OVER_CURRENT_OFFSET				12
#define HEART_BEAT_FAILED_OFFSET			15

//烟雾告警
#define XM_PLC_CAB_SMOKE_ALARM_ADDR			161
#define CAB_SMOKE_ALARM_OFFSET				10

//风扇告警
#define XM_PLC_CAB_FAN_ALARM_ADDR			162
#define CAB_FAN1_OVERLOAD_OFFSET			5
#define CAB_FAN2_OVERLOAD_OFFSET			6
#define CAB_FAN3_OVERLOAD_OFFSET			7
#define CAB_FAN4_OVERLOAD_OFFSET			8
#define CAB_FAN5_OVERLOAD_OFFSET			10
#define CAB_FAN6_OVERLOAD_OFFSET			11
#define CAB_FAN7_OVERLOAD_OFFSET			12
#define CAB_FAN8_OVERLOAD_OFFSET			13

//急停告警
#define XM_PLC_SUDDEN_STOP_ALARM_ADDR		163
#define FACE_PANEL_STOP_ALARM_OFFSET		0
#define HMI_STOP_ALARM_OFFSET				1
#define AC1_STOP_NO_RESET_OFFSET			6
#define AC2_STOP_NO_RESET_OFFSET			7
#define CAB_EMERGENCY_STOP_OFFSET			8

//针床库位烟雾告警
#define XM_PLC_CELL_SMOKE_ALARM_ADDR(x)		(165 + ((x>4)? x:(x-1)))
#define CELL_AHEAD_SMOKE_ALARM_OFFSET		0
#define CELL_REAR_SMOKE_ALARM_OFFSET		1
//#define CELL_AHEAD_OR_REAR_SMOKE_OFFSET	2	//已弃用
#define CELL_24V_ABNORMAL_OFFSET			3
#define CELL_AHEAD_AND_REAR_SMOKE_OFFSET	7

//针床库位动作异常
#define XM_PLC_CELL_ACTION_ALARM_ADDR(x)	(200 + ((x>4)? x:(x-1)))
#define CELL_RELIEVE_VAC_TIMEOUT_OFFSET		0
#define CELL_TRAY_EXTEND_TIMEOUT_OFFSET		1
#define CELL_TRAY_RETRACT_TIMEOUT_OFFSET	2
#define CELL_PUMP_VAC_TIMEOUT_OFFSET		4
#define CELL_FIREDOOR_CLOSE_TIMEOUT_OFFSET	5
#define CELL_FIREDOOR_OPEN_TIMEOUT_OFFSET	6
#define CELL_FAN_SPEED_ABNORMAL_OFFSET		7
#define CELL_PANEL_DOOR_ABNORMAL_OFFSET		10
#define CELL_TRAY_REVERSE_OFFSET			11
#define CELL_MAINTAIN_DOOR_DET_OFFSET		15

//库位针床维修按钮
#define XM_PLC_CELL_PINS_STATUS_ADDR		232
#define CELL1_REPAIR_BUTTON_OFFSET			0
#define CELL2_REPAIR_BUTTON_OFFSET			1
#define CELL3_REPAIR_BUTTON_OFFSET			2
#define CELL4_REPAIR_BUTTON_OFFSET			3
#define CELL5_REPAIR_BUTTON_OFFSET			5
#define CELL6_REPAIR_BUTTON_OFFSET			6
#define CELL7_REPAIR_BUTTON_OFFSET			7
#define CELL8_REPAIR_BUTTON_OFFSET			8

//库位托盘1-4状态
#define XM_PLC_CELL1_4_TRAY_STATUS_ADDR		234
#define CELL1_TRAY_IN_PLACE_OFFSET			0
#define CELL2_TRAY_IN_PLACE_OFFSET			1
#define CELL3_TRAY_IN_PLACE_OFFSET			2
#define CELL4_TRAY_IN_PLACE_OFFSET			3
#define CELL1_TRAY_RISE_TO_PLACE_OFFSET		5
#define CELL2_TRAY_RISE_TO_PLACE_OFFSET		6
#define CELL3_TRAY_RISE_TO_PLACE_OFFSET		7
#define CELL4_TRAY_RISE_TO_PLACE_OFFSET		8

//库位托盘5-8状态
#define XM_PLC_CELL5_8_TRAY_STATUS_ADDR		235
#define CELL5_TRAY_IN_PLACE_OFFSET			0
#define CELL6_TRAY_IN_PLACE_OFFSET			1
#define CELL7_TRAY_IN_PLACE_OFFSET			2
#define CELL8_TRAY_IN_PLACE_OFFSET			3
#define CELL5_TRAY_RISE_TO_PLACE_OFFSET		5
#define CELL6_TRAY_RISE_TO_PLACE_OFFSET		6
#define CELL7_TRAY_RISE_TO_PLACE_OFFSET		7
#define CELL8_TRAY_RISE_TO_PLACE_OFFSET		8

//库位消防门是否开启
#define XM_PLC_CELL_FIREDOOR_IS_OPEN_ADDR	240
#define CELL1_FIREDOOR_IS_OPEN_OFFSET		0
#define CELL2_FIREDOOR_IS_OPEN_OFFSET		1
#define CELL3_FIREDOOR_IS_OPEN_OFFSET		2
#define CELL4_FIREDOOR_IS_OPEN_OFFSET		3
#define CELL5_FIREDOOR_IS_OPEN_OFFSET		5
#define CELL6_FIREDOOR_IS_OPEN_OFFSET		6
#define CELL7_FIREDOOR_IS_OPEN_OFFSET		7
#define CELL8_FIREDOOR_IS_OPEN_OFFSET		8

//库位1-4压合状态
#define XM_PLC_CELL1_4_PINS_IS_PRESS_ADDR	242
#define CELL1_PINS_IS_PRESS_OFFSET			0
#define CELL1_PINS_IS_UNPRESS_OFFSET		1
#define CELL2_PINS_IS_PRESS_OFFSET			2
#define CELL2_PINS_IS_UNPRESS_OFFSET		3
#define CELL3_PINS_IS_PRESS_OFFSET			4
#define CELL3_PINS_IS_UNPRESS_OFFSET		5
#define CELL4_PINS_IS_PRESS_OFFSET			6
#define CELL4_PINS_IS_UNPRESS_OFFSET		7

//库位5-8压合状态
#define XM_PLC_CELL5_8_PINS_IS_PRESS_ADDR	243
#define CELL5_PINS_IS_PRESS_OFFSET			0
#define CELL5_PINS_IS_UNPRESS_OFFSET		1
#define CELL6_PINS_IS_PRESS_OFFSET			2
#define CELL6_PINS_IS_UNPRESS_OFFSET		3
#define CELL7_PINS_IS_PRESS_OFFSET			4
#define CELL7_PINS_IS_UNPRESS_OFFSET		5
#define CELL8_PINS_IS_PRESS_OFFSET			6
#define CELL8_PINS_IS_UNPRESS_OFFSET		7

//库位强制弹开状态
#define XM_PLC_CELL_PINS_FORCE_UNPRESS_ADDR		248
#define CELL1_PINS_FORCE_UNPRESS_OFFSET			0
#define CELL2_PINS_FORCE_UNPRESS_OFFSET			1
#define CELL3_PINS_FORCE_UNPRESS_OFFSET			2
#define CELL4_PINS_FORCE_UNPRESS_OFFSET			3
#define CELL5_PINS_FORCE_UNPRESS_OFFSET			5
#define CELL6_PINS_FORCE_UNPRESS_OFFSET			6
#define CELL7_PINS_FORCE_UNPRESS_OFFSET			7
#define CELL8_PINS_FORCE_UNPRESS_OFFSET			8

//库位真空值
#define XM_PLC_CELL_VAC_VALUE_ADDR(x)			(253 + ((x>4)? x:(x-1))*2)

//重大告警断电延迟秒数
#define XM_PLC_ALARM_POWERDOWN_DELAY_ADDR		421

//紧急停止断电延迟秒数
#define XM_PLC_EMERGENCY_POWERDOWN_DELAY_ADDR	422

//库位保压测试结果
#define XM_PLC_CELL_KEEP_VAC_RESULT_ADDR(x)		(5360 + ((x>4)? x:(x-1)))

//PLC软件版本
#define XM_PLC_SOFTWARE_VERSION_ADDR			5460

//保压开始压力值
#define XM_PLC_CELL_KEEP_VAC_START_VAL_ADDR(x)	(5470 + ((x>4)? x:(x-1)))

//保压结束压力值
#define XM_PLC_CELL_KEEP_VAC_END_VAL_ADDR(x)	(5500 + ((x>4)? x:(x-1)))


PinsGroup_AXM::PinsGroup_AXM()
{
	is_plc_connected = false;
	warn_poweroff_delaysec = 0;
	emstop_poweroff_delaysec = 0;
	
	memset((void*)&sys_abnormal_alarm,0,sizeof(plc_sys_abnormal_alarm_t));
	memset((void*)&cab_smoke_alarm,0,sizeof(alarm_erc_t));
	memset((void*)&cabfan_overload_alarm,0,sizeof(plc_cab_fan_alarm_t));
	memset((void*)&sudden_stop_alarm,0,sizeof(plc_sudden_stop_alarm_t));
	
	memset((void*)cell_smoke_alarm,0,MAX_CELLS_NBR*sizeof(plc_cell_smoke_alarm_t));
	memset((void*)cell_action_alarm,0,MAX_CELLS_NBR*sizeof(plc_cell_action_alarm_t));
	memset((void*)vac_value,0,MAX_CELLS_NBR*sizeof(float));
}

PinsGroup_AXM::~PinsGroup_AXM()
{

}

bool PinsGroup_AXM::do_plc_connect(const char *ip,int port)
{
	bool status = false;
	status = m_slmp_dev.SLMPconnect(ip,port);
	if(status)
	{
		is_plc_connected = true;
	}
	else
	{
		is_plc_connected = false;
	}

	return status;
}

bool PinsGroup_AXM::is_connected()
{
	return is_plc_connected;
}

void PinsGroup_AXM::close_connect()
{
	is_plc_connected = false;
	m_slmp_dev.SLMPclose();
}
	
bool PinsGroup_AXM::set_tray_action(int cellNo,uint8 action)
{
	bool status = false;
	uint16 out = 0;

	status = m_slmp_dev.read_register(XM_PLC_CELL_SETTING_ADDR(cellNo),out);
	if(status)
	{
		if(action == UP_T)
		{
			if(out & (0x0001 << CELL_TRAY_UP_OFFSET))
			{
				out &= (~(0x0001 << CELL_TRAY_UP_OFFSET));		//如果bit为1需要先清掉
				status = m_slmp_dev.write_register(XM_PLC_CELL_SETTING_ADDR(cellNo),out);
			}
		
			out |= (0x0001 << CELL_TRAY_UP_OFFSET);
			out &= (~(0x0001 << CELL_TRAY_DOWN_OFFSET));
		}
		else
		{
			//弹开针床前 抽真空bit清0
			if(out & (0x0001 << HIGH_VAC_ON_OFFSET))
			{
				out &= (~(0x0001 << HIGH_VAC_ON_OFFSET));
			}

			if(out & (0x0001 << CELL_TRAY_DOWN_OFFSET))
			{
				out &= (~(0x0001 << CELL_TRAY_DOWN_OFFSET));	//如果bit为1需要先清掉
				status = m_slmp_dev.write_register(XM_PLC_CELL_SETTING_ADDR(cellNo),out);
			}
					
			out |= (0x0001 << CELL_TRAY_DOWN_OFFSET);
			out &= (~(0x0001 << CELL_TRAY_UP_OFFSET));
		}

		status = m_slmp_dev.write_register(XM_PLC_CELL_SETTING_ADDR(cellNo),out);
	}

	return status;
}

bool PinsGroup_AXM::set_PLC_workMode(int cellNo,uint8 mode)
{
	bool status = false;
	uint16 out = 0;

	status = m_slmp_dev.read_register(XM_PLC_CELL_SETTING_ADDR(cellNo),out);
	if(status)
	{
		if(mode == AUTO_M)
		{
			out |= (0x0001 << AUTO_OPT_MODE_OFFSET);
			out &= (~(0x0001 << MAINTENANCE_MODE_OFFSET));
		}
		else
		{
			out |= (0x0001 << MAINTENANCE_MODE_OFFSET);
			out &= (~(0x0001 << AUTO_OPT_MODE_OFFSET));
		}

		status = m_slmp_dev.write_register(XM_PLC_CELL_SETTING_ADDR(cellNo),out);
	}

	return status;
}

bool PinsGroup_AXM::start_vacuum_control(int cellNo,bool en)
{
	bool status = false;

	status = m_slmp_dev.write_bit(XM_PLC_CELL_SETTING_ADDR(cellNo),HIGH_VAC_ON_OFFSET,en);
	
	return status;
}

bool PinsGroup_AXM::break_vacuum_control(int cellNo,bool en)
{
	bool status = false;

	status = m_slmp_dev.write_bit(XM_PLC_CELL_SETTING_ADDR(cellNo),BREAK_VAC_ON_OFFSET,en);
	
	return status;
}

bool PinsGroup_AXM::fireDoor_control(int cellNo,uint8 action)
{
	bool status = false;
	uint16 out = 0;
	
	status = m_slmp_dev.read_register(XM_PLC_CELL_SETTING_ADDR(cellNo),out);
	if(status)
	{
		if(action == UP_T)	//消防门关闭
		{
#if defined(AWH_FORMATION_SYS)		
			out |= (0x0001 << FIRE_DOOR_UP_OFFSET);
			out &= (~(0x0001 << FIRE_DOOR_DOWN_OFFSET));
#else
			out &= (~(0x0001 << CRANE_FORK_BUSY_OFFSET));
#endif
		}
		else		//消防门打开
		{
#if defined(AWH_FORMATION_SYS)	
			out |= (0x0001 << FIRE_DOOR_DOWN_OFFSET);
			out &= (~(0x0001 << FIRE_DOOR_UP_OFFSET));
#else
			out |= (0x0001 << CRANE_FORK_BUSY_OFFSET);
#endif
		}

		status = m_slmp_dev.write_register(XM_PLC_CELL_SETTING_ADDR(cellNo),out);
	}
	//LOGFMTD("fireDoor_control action:%d  status:%d ",action, status);

	return status;
}


bool PinsGroup_AXM::leak_ratio_test(int cellNo,bool en)
{
	bool status = false;

	status = m_slmp_dev.write_bit(XM_PLC_CELL_SETTING_ADDR(cellNo),LEAK_RATIO_TEST_OFFSET,en);
	
	return status;
}

bool PinsGroup_AXM::vent_fan_control(int cellNo,bool en)
{
	bool status = false;

	status = m_slmp_dev.write_bit(XM_PLC_CELL_SETTING_ADDR(cellNo),CELL_FAN_ON_OFFSET,en);
	//LOGFMTD("fan_control en:%d  status:%d ",en, status);
	if(en)
	{
		LOGFMT_DEBUG(cell_id_temp,"库位%d 风扇打开",cellNo);
	}
	else
	{
		LOGFMT_DEBUG(cell_id_temp,"库位%d 风扇关闭",cellNo);
	}
	
	return status;
}

bool PinsGroup_AXM::vac_block_test(int cellNo,bool en)
{
	bool status = false;

	status = m_slmp_dev.write_bit(XM_PLC_CELL_SETTING_ADDR(cellNo),VAC_BLOCK_TEST_OFFSET,en);
	return status;
}

bool PinsGroup_AXM::step_para_write_in(int cellNo,bool en)
{
	bool status = false;

	status = m_slmp_dev.write_bit(XM_PLC_CELL_SETTING_ADDR(cellNo),STEP_PARA_WRITE_IN_OFFSET,en);
		
	return status;
}

bool PinsGroup_AXM::tools_power_control(int cellNo,bool en)
{
	bool status = false;

	status = m_slmp_dev.write_bit(XM_PLC_CELL_SETTING_ADDR(cellNo),TOOLS_POWER_ON_OFFSET,en);
		
	return status;
}


bool PinsGroup_AXM::set_heartbeat_signal(uint16 value)
{
	bool status = false;
	
	status = m_slmp_dev.write_register(PLC_HEARTBEAT_CYCLE_ADDR,value);
	return status;
}

bool PinsGroup_AXM::set_system_lamp_buzzer(uint8 item)
{
	bool status = false;

	if(item == RED_I)
	{
		status = m_slmp_dev.write_bit(XM_PLC_TRICOLOR_LAMP_ADDR,SYS_RED_LAMP_OFFSET,true);
	}
	else if(item == YELLOW_I)
	{
		status = m_slmp_dev.write_bit(XM_PLC_TRICOLOR_LAMP_ADDR,SYS_YELLOW_LAMP_OFFSET,true);
	}
	else if(item == GREEN_I)
	{
		status = m_slmp_dev.write_bit(XM_PLC_TRICOLOR_LAMP_ADDR,SYS_GREEN_LAMP_OFFSET,true);
	}
	else if(item == BUZZER_I)
	{
		status = m_slmp_dev.write_bit(XM_PLC_TRICOLOR_LAMP_ADDR,SYS_BUZZER_OFFSET,true);
	}

	//LOGFMTD("lamp_buzzer item:%d  status:%d ",item, status);
	
	return status;
}


bool PinsGroup_AXM::set_cell_OTP2(int cellNo)
{
	bool status = false;
	int bit_offset;
	
	switch(cellNo)
	{
		case 1:
			bit_offset = CELL1_OPT2_OFFSET;
			break;
		case 2:
			bit_offset = CELL2_OPT2_OFFSET;
			break;
		case 3:
			bit_offset = CELL3_OPT2_OFFSET;
			break;
		case 4:
			bit_offset = CELL4_OPT2_OFFSET;
			break;
		case 5:
			bit_offset = CELL5_OPT2_OFFSET;
			break;
		case 6:
			bit_offset = CELL6_OPT2_OFFSET;
			break;
		case 7:
			bit_offset = CELL7_OPT2_OFFSET;
			break;
		case 8:
			bit_offset = CELL8_OPT2_OFFSET;
			break;
		default:
			return false;
	}
	
	status = m_slmp_dev.write_bit(XM_PLC_CELL_OTP2_ADDR,bit_offset,true);

	return status;
}

bool PinsGroup_AXM::set_cell_OTP1(int cellNo)
{
	bool status = false;
	int bit_offset;
	
	switch(cellNo)
	{
		case 1:
			bit_offset = CELL1_OPT1_OFFSET;
			break;
		case 2:
			bit_offset = CELL2_OPT1_OFFSET;
			break;
		case 3:
			bit_offset = CELL3_OPT1_OFFSET;
			break;
		case 4:
			bit_offset = CELL4_OPT1_OFFSET;
			break;
		case 5:
			bit_offset = CELL5_OPT1_OFFSET;
			break;
		case 6:
			bit_offset = CELL6_OPT1_OFFSET;
			break;
		case 7:
			bit_offset = CELL7_OPT1_OFFSET;
			break;
		case 8:
			bit_offset = CELL8_OPT1_OFFSET;
			break;	
		default:
			return false;
	}
	
	status = m_slmp_dev.write_bit(XM_PLC_CELL_OTP1_ADDR,bit_offset,true);

	return status;
}


bool PinsGroup_AXM::set_calibrate_mode(int cellNo)
{
	bool status = false;
	int bit_offset;
	
	switch(cellNo)
	{
		case 1:
			bit_offset = CELL1_CALIB_OFFSET;
			break;
		case 2:
			bit_offset = CELL2_CALIB_OFFSET;
			break;
		case 3:
			bit_offset = CELL3_CALIB_OFFSET;
			break;
		case 4:
			bit_offset = CELL4_CALIB_OFFSET;
			break;
		case 5:
			bit_offset = CELL5_CALIB_OFFSET;
			break;
		case 6:
			bit_offset = CELL6_CALIB_OFFSET;
			break;
		case 7:
			bit_offset = CELL7_CALIB_OFFSET;
			break;
		case 8:
			bit_offset = CELL8_CALIB_OFFSET;
			break;	
		default:
			return false;
	}
	
	status = m_slmp_dev.write_bit(XM_PLC_CELL_CALIB_ADDR,bit_offset,true);

	return status;
}


bool PinsGroup_AXM::clear_once_alarm()
{
	bool status = false;
	uint16 out = 0;
	
	status = m_slmp_dev.write_bit(XM_PLC_CELL_CALIB_ADDR,HMI_ALARM_RESET_OFFSET,true);
	
	if(status)	
	{
		//清除告警灯
		status = m_slmp_dev.read_register(XM_PLC_TRICOLOR_LAMP_ADDR,out);
		if(status)
		{
			out &= (~(0x0001 << SYS_RED_LAMP_OFFSET));
			out &= (~(0x0001 << SYS_YELLOW_LAMP_OFFSET));
			out &= (~(0x0001 << SYS_GREEN_LAMP_OFFSET));
			out &= (~(0x0001 << SYS_BUZZER_OFFSET));
		
			status = m_slmp_dev.write_register(XM_PLC_TRICOLOR_LAMP_ADDR,out);
		}
	}
	
	return status;
}

bool PinsGroup_AXM::set_fire_fighting(uint8 cellNo)
{
	bool status = false;
	int offset;

	if(cellNo < 5)
	{
		offset = cellNo-1;
	}
	else
	{
		offset = cellNo;
	}
	
	status = m_slmp_dev.write_bit(XM_PLC_CELL_FIRE_CTL_ADDR,offset,true);

	return status;
}

bool PinsGroup_AXM::set_vacuum_error_value(short err_val)
{
	bool status = false;
	uint16 value;
	
	value = 100*err_val;
	status = m_slmp_dev.write_register(XM_PLC_VAC_ERR_SET_ADDR,value);

	return status;
}

bool PinsGroup_AXM::set_keep_vacuum_break_value(float val)
{
	bool status = false;
	uint16 value;
	
	value = (uint16)(100*val);
	status = m_slmp_dev.write_register(XM_PLC_BREAK_VAC_SET_ADDR,value);

	return status;
}

bool PinsGroup_AXM::set_keep_vacuum_test_value(float val)
{
	bool status = false;
	uint16 value;
	
	value = (uint16)(100*val);
	status = m_slmp_dev.write_register(XM_PLC_KEEP_VAC_TEST_SET_ADDR,value);

	return status;
}

bool PinsGroup_AXM::set_cell_stepNo(int cellNo,uint16 stepNo)
{
	bool status = false;
	int address;
	
	switch(cellNo)
	{
		case 1:
			address = XM_PLC_CELL1_STEP_NO_ADDR;
			break;
		case 2:
			address = XM_PLC_CELL2_STEP_NO_ADDR;
			break;
		case 3:
			address = XM_PLC_CELL3_STEP_NO_ADDR;
			break;
		case 4:
			address = XM_PLC_CELL4_STEP_NO_ADDR;
			break;
		case 5:
			address = XM_PLC_CELL5_STEP_NO_ADDR;
			break;
		case 6:
			address = XM_PLC_CELL6_STEP_NO_ADDR;
			break;
		case 7:
			address = XM_PLC_CELL7_STEP_NO_ADDR;
			break;
		case 8:
			address = XM_PLC_CELL8_STEP_NO_ADDR;
			break;
			
		default:
			return false;
	}

	status = m_slmp_dev.write_register(address,stepNo);

	return status;
}

bool PinsGroup_AXM::set_cell_keep_vacuum_target(int cellNo,float val)
{
	bool status = false;
	int address;
	uint16 value;
	
	switch(cellNo)
	{
		case 1:
			address = XM_PLC_CELL1_KEEP_VAC_SET_ADDR;
			break;
		case 2:
			address = XM_PLC_CELL2_KEEP_VAC_SET_ADDR;
			break;
		case 3:
			address = XM_PLC_CELL3_KEEP_VAC_SET_ADDR;
			break;
		case 4:
			address = XM_PLC_CELL4_KEEP_VAC_SET_ADDR;
			break;
		case 5:
			address = XM_PLC_CELL5_KEEP_VAC_SET_ADDR;
			break;
		case 6:
			address = XM_PLC_CELL6_KEEP_VAC_SET_ADDR;
			break;
		case 7:
			address = XM_PLC_CELL7_KEEP_VAC_SET_ADDR;
			break;
		case 8:
			address = XM_PLC_CELL8_KEEP_VAC_SET_ADDR;
			break;
			
		default:
			return false;
	}

	value = (uint16)(100*val);
	status = m_slmp_dev.write_register(address,value);

	return status;
}

bool PinsGroup_AXM::set_cell_keep_vacuum_leakrate(int cellNo,float val)
{
	bool status = false;
	int address;
	uint16 value;
	
	switch(cellNo)
	{
		case 1:
			address = XM_PLC_CELL1_LEAKRATE_SET_ADDR;
			break;
		case 2:
			address = XM_PLC_CELL2_LEAKRATE_SET_ADDR;
			break;
		case 3:
			address = XM_PLC_CELL3_LEAKRATE_SET_ADDR;
			break;
		case 4:
			address = XM_PLC_CELL4_LEAKRATE_SET_ADDR;
			break;
		case 5:
			address = XM_PLC_CELL5_LEAKRATE_SET_ADDR;
			break;
		case 6:
			address = XM_PLC_CELL6_LEAKRATE_SET_ADDR;
			break;
		case 7:
			address = XM_PLC_CELL7_LEAKRATE_SET_ADDR;
			break;
		case 8:
			address = XM_PLC_CELL8_LEAKRATE_SET_ADDR;
			break;
			
		default:
			return false;
	}

	value = (uint16)(100*val);
	status = m_slmp_dev.write_register(address,value);

	return status;
}


bool PinsGroup_AXM::set_cell_keep_vacuum_stableTime(int cellNo,uint16 t)
{
	bool status = false;
	int address;
	uint16 value;
	
	switch(cellNo)
	{
		case 1:
			address = XM_PLC_CELL1_KEEP_VAC_STABLE_TIME_ADDR;
			break;
		case 2:
			address = XM_PLC_CELL2_KEEP_VAC_STABLE_TIME_ADDR;
			break;
		case 3:
			address = XM_PLC_CELL3_KEEP_VAC_STABLE_TIME_ADDR;
			break;
		case 4:
			address = XM_PLC_CELL4_KEEP_VAC_STABLE_TIME_ADDR;
			break;
		case 5:
			address = XM_PLC_CELL5_KEEP_VAC_STABLE_TIME_ADDR;
			break;
		case 6:
			address = XM_PLC_CELL6_KEEP_VAC_STABLE_TIME_ADDR;
			break;
		case 7:
			address = XM_PLC_CELL7_KEEP_VAC_STABLE_TIME_ADDR;
			break;
		case 8:
			address = XM_PLC_CELL8_KEEP_VAC_STABLE_TIME_ADDR;
			break;
			
		default:
			return false;
	}

	value = t;
	status = m_slmp_dev.write_register(address,value);

	return status;
}

bool PinsGroup_AXM::set_cell_keep_vacuum_time(int cellNo,uint16 t)
{
	bool status = false;
	int address;
	uint16 value;
	
	switch(cellNo)
	{
		case 1:
			address = XM_PLC_CELL1_KEEP_VAC_TIME_ADDR;
			break;
		case 2:
			address = XM_PLC_CELL2_KEEP_VAC_TIME_ADDR;
			break;
		case 3:
			address = XM_PLC_CELL3_KEEP_VAC_TIME_ADDR;
			break;
		case 4:
			address = XM_PLC_CELL4_KEEP_VAC_TIME_ADDR;
			break;
		case 5:
			address = XM_PLC_CELL5_KEEP_VAC_TIME_ADDR;
			break;
		case 6:
			address = XM_PLC_CELL6_KEEP_VAC_TIME_ADDR;
			break;
		case 7:
			address = XM_PLC_CELL7_KEEP_VAC_TIME_ADDR;
			break;
		case 8:
			address = XM_PLC_CELL8_KEEP_VAC_TIME_ADDR;
			break;
			
		default:
			return false;
	}

	value = t;
	status = m_slmp_dev.write_register(address,value);

	return status;
}


bool PinsGroup_AXM::set_cell_high_vacuum_value(int cellNo,float val)
{
	bool status = false;
	int address;
	uint16 value;
	
	switch(cellNo)
	{
		case 1:
			address = XM_PLC_CELL1_HIGH_VAC_SET_ADDR;
			break;
		case 2:
			address = XM_PLC_CELL2_HIGH_VAC_SET_ADDR;
			break;
		case 3:
			address = XM_PLC_CELL3_HIGH_VAC_SET_ADDR;
			break;
		case 4:
			address = XM_PLC_CELL4_HIGH_VAC_SET_ADDR;
			break;
		case 5:
			address = XM_PLC_CELL5_HIGH_VAC_SET_ADDR;
			break;
		case 6:
			address = XM_PLC_CELL6_HIGH_VAC_SET_ADDR;
			break;
		case 7:
			address = XM_PLC_CELL7_HIGH_VAC_SET_ADDR;
			break;
		case 8:
			address = XM_PLC_CELL8_HIGH_VAC_SET_ADDR;
			break;
			
		default:
			return false;
	}

	value = (uint16)(100*val);
	status = m_slmp_dev.write_register(address,value);

	return status;	
}


bool PinsGroup_AXM::set_cell_pump_vacuum_timeout(int cellNo,uint16 t)
{
	bool status = false;
	int address;
	uint16 value;
	
	switch(cellNo)
	{
		case 1:
			address = XM_PLC_CELL1_PUMP_VAC_TIMEOUT_ADDR;
			break;
		case 2:
			address = XM_PLC_CELL2_PUMP_VAC_TIMEOUT_ADDR;
			break;
		case 3:
			address = XM_PLC_CELL3_PUMP_VAC_TIMEOUT_ADDR;
			break;
		case 4:
			address = XM_PLC_CELL4_PUMP_VAC_TIMEOUT_ADDR;
			break;
		case 5:
			address = XM_PLC_CELL5_PUMP_VAC_TIMEOUT_ADDR;
			break;
		case 6:
			address = XM_PLC_CELL6_PUMP_VAC_TIMEOUT_ADDR;
			break;
		case 7:
			address = XM_PLC_CELL7_PUMP_VAC_TIMEOUT_ADDR;
			break;
		case 8:
			address = XM_PLC_CELL8_PUMP_VAC_TIMEOUT_ADDR;
			break;
			
		default:
			return false;
	}

	value = 10*t;
	status = m_slmp_dev.write_register(address,value);

	return status;	
}

bool PinsGroup_AXM::set_cell_leak_vacuum_timeout(int cellNo,uint16 t)
{
	bool status = false;
	int address;
	uint16 value;
	
	switch(cellNo)
	{
		case 1:
			address = XM_PLC_CELL1_LEAK_VAC_TIMEOUT_ADDR;
			break;
		case 2:
			address = XM_PLC_CELL2_LEAK_VAC_TIMEOUT_ADDR;
			break;
		case 3:
			address = XM_PLC_CELL3_LEAK_VAC_TIMEOUT_ADDR;
			break;
		case 4:
			address = XM_PLC_CELL4_LEAK_VAC_TIMEOUT_ADDR;
			break;
		case 5:
			address = XM_PLC_CELL5_LEAK_VAC_TIMEOUT_ADDR;
			break;
		case 6:
			address = XM_PLC_CELL6_LEAK_VAC_TIMEOUT_ADDR;
			break;
		case 7:
			address = XM_PLC_CELL7_LEAK_VAC_TIMEOUT_ADDR;
			break;
		case 8:
			address = XM_PLC_CELL8_LEAK_VAC_TIMEOUT_ADDR;
			break;
			
		default:
			return false;
	}

	value = 10*t;
	status = m_slmp_dev.write_register(address,value);

	return status;	
}

bool PinsGroup_AXM::set_cell_leak_vacuum_value(int cellNo,float val)
{
	bool status = false;
	int address;
	uint16 value;
	
	switch(cellNo)
	{
		case 1:
			address = XM_PLC_CELL1_LEAK_VAC_SET_ADDR;
			break;
		case 2:
			address = XM_PLC_CELL2_LEAK_VAC_SET_ADDR;
			break;
		case 3:
			address = XM_PLC_CELL3_LEAK_VAC_SET_ADDR;
			break;
		case 4:
			address = XM_PLC_CELL4_LEAK_VAC_SET_ADDR;
			break;
		case 5:
			address = XM_PLC_CELL5_LEAK_VAC_SET_ADDR;
			break;
		case 6:
			address = XM_PLC_CELL6_LEAK_VAC_SET_ADDR;
			break;
		case 7:
			address = XM_PLC_CELL7_LEAK_VAC_SET_ADDR;
			break;
		case 8:
			address = XM_PLC_CELL8_LEAK_VAC_SET_ADDR;
			break;
			
		default:
			return false;
	}

	value = (uint16)(100*val);
	status = m_slmp_dev.write_register(address,value);

	return status;	
}


void PinsGroup_AXM::init_system_abnormal_alarm()
{
	sys_abnormal_alarm.AC1_over_voltage.alarm_erc = 1;
	sys_abnormal_alarm.AC1_low_voltage.alarm_erc = 2;
	sys_abnormal_alarm.AC1_over_current.alarm_erc = 3;
	sys_abnormal_alarm.AC2_over_voltage.alarm_erc = 4;
	sys_abnormal_alarm.AC2_low_voltage.alarm_erc = 5;
	sys_abnormal_alarm.AC2_over_current.alarm_erc = 6;
	sys_abnormal_alarm.low_pressure.alarm_erc = 7;
	sys_abnormal_alarm.cab_temp_err.alarm_erc = 8;
	sys_abnormal_alarm.heartbeat_stop.alarm_erc = 10;
}

void PinsGroup_AXM::init_cab_smoke_alarm()
{
	cab_smoke_alarm.alarm_erc = 9;
}

void PinsGroup_AXM::init_cab_fan_alarm()
{
	cabfan_overload_alarm.fan1_overload.alarm_erc = 11;
	cabfan_overload_alarm.fan2_overload.alarm_erc = 12;
	cabfan_overload_alarm.fan3_overload.alarm_erc = 13;
	cabfan_overload_alarm.fan4_overload.alarm_erc = 14;
	cabfan_overload_alarm.fan5_overload.alarm_erc = 15;
	cabfan_overload_alarm.fan6_overload.alarm_erc = 16;
	cabfan_overload_alarm.fan7_overload.alarm_erc = 17;
	cabfan_overload_alarm.fan8_overload.alarm_erc = 18;
}

void PinsGroup_AXM::init_sudden_stop_alarm()
{
	sudden_stop_alarm.face_panel_ctl_stop.alarm_erc = 19;
	sudden_stop_alarm.HMI_ctl_stop.alarm_erc = 20;
	sudden_stop_alarm.AC1_stop_unreset.alarm_erc = 21;
	sudden_stop_alarm.AC2_stop_unreset.alarm_erc = 22;
	sudden_stop_alarm.cab_ctl_sudden_stop.alarm_erc = 20;
}

void PinsGroup_AXM::init_cell_smoke_alarm()
{
	for(int i = 0; i < MAX_CELLS_NBR; i++)
	{
		cell_smoke_alarm[i].cell_ahead_smoke.alarm_erc = 23;
		cell_smoke_alarm[i].cell_rear_smoke.alarm_erc = 24;
		cell_smoke_alarm[i].cell_ahead_and_rear_smoke.alarm_erc = 25;
		cell_smoke_alarm[i].cell_24V_abnormal.alarm_erc = 26;
	}
}

void PinsGroup_AXM::init_cell_action_abnormal_alarm()
{
	for(int i = 0; i < MAX_CELLS_NBR; i++)
	{
		cell_action_alarm[i].relieve_vac_timeout.alarm_erc = 27;
		cell_action_alarm[i].tray_extend_timeout.alarm_erc = 28;
		cell_action_alarm[i].tray_retract_timeout.alarm_erc = 29;	
		cell_action_alarm[i].pump_vac_timeout.alarm_erc = 30;
		cell_action_alarm[i].firedoor_off_timeout.alarm_erc = 31;
		cell_action_alarm[i].firedoor_on_timeout.alarm_erc = 32;
		cell_action_alarm[i].fanspeed_abnormal.alarm_erc = 33;
		cell_action_alarm[i].paneldoor_abnormal.alarm_erc = 34;
		cell_action_alarm[i].tray_reverse.alarm_erc = 35;
		cell_action_alarm[i].maintain_door_abnormal.alarm_erc = 39;
	}
}

void PinsGroup_AXM::init_plc_alarm_erc()
{
	init_system_abnormal_alarm();
	init_cab_smoke_alarm();
	init_cab_fan_alarm();
	init_sudden_stop_alarm();
	init_cell_smoke_alarm();
	init_cell_action_abnormal_alarm();
}

bool PinsGroup_AXM::get_system_abnormal_alarm()
{
	bool status = false;
	uint16_t out = 0;
	
	status = m_slmp_dev.read_register(XM_PLC_SYS_ABNORMAL_ALARM_ADDR,out);
	if(status)
	{
		if(out & (0x0001 << AC1_OVER_VOLTAGE_OFFSET))
		{
			sys_abnormal_alarm.AC1_over_voltage.is_happen = true;
		}
		else
		{
			sys_abnormal_alarm.AC1_over_voltage.is_happen = false;
		}

		if(out & (0x0001 << AC1_LOW_VOLTAGE_OFFSET))
		{
			sys_abnormal_alarm.AC1_low_voltage.is_happen = true;
		}
		else
		{
			sys_abnormal_alarm.AC1_low_voltage.is_happen = false;
		}

		if(out & (0x0001 << AC1_OVER_CURRENT_OFFSET))
		{
			sys_abnormal_alarm.AC1_over_current.is_happen = true;
		}
		else
		{
			sys_abnormal_alarm.AC1_over_current.is_happen = false;
		}

		if(out & (0x0001 << AC2_OVER_VOLTAGE_OFFSET))
		{
			sys_abnormal_alarm.AC2_over_voltage.is_happen = true;
		}
		else
		{
			sys_abnormal_alarm.AC2_over_voltage.is_happen = false;
		}

		if(out & (0x0001 << AC2_LOW_VOLTAGE_OFFSET))
		{
			sys_abnormal_alarm.AC2_low_voltage.is_happen = true;
		}
		else
		{
			sys_abnormal_alarm.AC2_low_voltage.is_happen = false;
		}
		
		if(out & (0x0001 << AC2_OVER_CURRENT_OFFSET))
		{
			sys_abnormal_alarm.AC2_over_current.is_happen = true;
		}
		else
		{
			sys_abnormal_alarm.AC2_over_current.is_happen = false;
		}

		if(out & (0x0001 << LOW_PRESSURE_OFFSET))
		{
			sys_abnormal_alarm.low_pressure.is_happen = true;
		}
		else
		{
			sys_abnormal_alarm.low_pressure.is_happen = false;
		}

		if(out & (0x0001 << CAB_TEMP_ABNORMAL_OFFSET))
		{
			sys_abnormal_alarm.cab_temp_err.is_happen = true;
		}
		else
		{
			sys_abnormal_alarm.cab_temp_err.is_happen = false;
		}

		if(out & (0x0001 << HEART_BEAT_FAILED_OFFSET))
		{
			sys_abnormal_alarm.heartbeat_stop.is_happen = true;
		}
		else
		{
			sys_abnormal_alarm.heartbeat_stop.is_happen = false;
		}
	}

	return status;
}

bool PinsGroup_AXM::get_cab_smoke_alarm()
{
	bool status = false;
	uint16_t out = 0;
	
	status = m_slmp_dev.read_register(XM_PLC_CAB_SMOKE_ALARM_ADDR,out);
	if(status)
	{
		if(out & (0x0001 << CAB_SMOKE_ALARM_OFFSET))
		{
			cab_smoke_alarm.is_happen = true;
		}
		else
		{
			cab_smoke_alarm.is_happen = false;
		}
	}

	return status;
}

bool PinsGroup_AXM::get_cab_fan_alarm()
{
	bool status = false;
	uint16_t out = 0;
	
	status = m_slmp_dev.read_register(XM_PLC_CAB_FAN_ALARM_ADDR,out);
	if(status)
	{
		if(out & (0x0001 << CAB_FAN1_OVERLOAD_OFFSET))
		{
			cabfan_overload_alarm.fan1_overload.is_happen = true;
		}
		else
		{
			cabfan_overload_alarm.fan1_overload.is_happen = false;
		}

		if(out & (0x0001 << CAB_FAN2_OVERLOAD_OFFSET))
		{
			cabfan_overload_alarm.fan2_overload.is_happen = true;
		}
		else
		{
			cabfan_overload_alarm.fan2_overload.is_happen = false;
		}

		if(out & (0x0001 << CAB_FAN3_OVERLOAD_OFFSET))
		{
			cabfan_overload_alarm.fan3_overload.is_happen = true;
		}
		else
		{
			cabfan_overload_alarm.fan3_overload.is_happen = false;
		}

		if(out & (0x0001 << CAB_FAN4_OVERLOAD_OFFSET))
		{
			cabfan_overload_alarm.fan4_overload.is_happen = true;
		}
		else
		{
			cabfan_overload_alarm.fan4_overload.is_happen = false;
		}

		if(out & (0x0001 << CAB_FAN5_OVERLOAD_OFFSET))
		{
			cabfan_overload_alarm.fan5_overload.is_happen = true;
		}
		else
		{
			cabfan_overload_alarm.fan5_overload.is_happen = false;
		}

		if(out & (0x0001 << CAB_FAN6_OVERLOAD_OFFSET))
		{
			cabfan_overload_alarm.fan6_overload.is_happen = true;
		}
		else
		{
			cabfan_overload_alarm.fan6_overload.is_happen = false;
		}

		if(out & (0x0001 << CAB_FAN7_OVERLOAD_OFFSET))
		{
			cabfan_overload_alarm.fan7_overload.is_happen = true;
		}
		else
		{
			cabfan_overload_alarm.fan7_overload.is_happen = false;
		}

		if(out & (0x0001 << CAB_FAN8_OVERLOAD_OFFSET))
		{
			cabfan_overload_alarm.fan8_overload.is_happen = true;
		}
		else
		{
			cabfan_overload_alarm.fan8_overload.is_happen = false;
		}
	}

	return status;
}

bool PinsGroup_AXM::get_sudden_stop_alarm()
{
	bool status = false;
	uint16_t out = 0;
	
	status = m_slmp_dev.read_register(XM_PLC_SUDDEN_STOP_ALARM_ADDR,out);
	if(status)
	{
		if(out & (0x0001 << FACE_PANEL_STOP_ALARM_OFFSET))
		{
			sudden_stop_alarm.face_panel_ctl_stop.is_happen = true;
		}
		else
		{
			sudden_stop_alarm.face_panel_ctl_stop.is_happen = false;
		}

		if(out & (0x0001 << HMI_STOP_ALARM_OFFSET))
		{
			sudden_stop_alarm.HMI_ctl_stop.is_happen = true;
		}
		else
		{
			sudden_stop_alarm.HMI_ctl_stop.is_happen = false;
		}

		if(out & (0x0001 << AC1_STOP_NO_RESET_OFFSET))
		{
			sudden_stop_alarm.AC1_stop_unreset.is_happen = true;
		}
		else
		{
			sudden_stop_alarm.AC1_stop_unreset.is_happen = false;
		}

		if(out & (0x0001 << AC2_STOP_NO_RESET_OFFSET))
		{
			sudden_stop_alarm.AC2_stop_unreset.is_happen = true;
		}
		else
		{
			sudden_stop_alarm.AC2_stop_unreset.is_happen = false;
		}

		if(out & (0x0001 << CAB_EMERGENCY_STOP_OFFSET))
		{
			sudden_stop_alarm.cab_ctl_sudden_stop.is_happen = true;
		}
		else
		{
			sudden_stop_alarm.cab_ctl_sudden_stop.is_happen = false;
		}
	}

	return status;
}

bool PinsGroup_AXM::get_cell_smoke_alarm(int cellNo)
{
	bool status = false;
	uint16_t out = 0;
	
	status = m_slmp_dev.read_register(XM_PLC_CELL_SMOKE_ALARM_ADDR(cellNo),out);
	if(status)
	{
		if(out & (0x0001 << CELL_AHEAD_SMOKE_ALARM_OFFSET))
		{
			cell_smoke_alarm[cellNo-1].cell_ahead_smoke.is_happen = true;
		}
		else
		{
			cell_smoke_alarm[cellNo-1].cell_ahead_smoke.is_happen = false;
		}

		if(out & (0x0001 << CELL_REAR_SMOKE_ALARM_OFFSET))
		{
			cell_smoke_alarm[cellNo-1].cell_rear_smoke.is_happen = true;
		}
		else
		{
			cell_smoke_alarm[cellNo-1].cell_rear_smoke.is_happen = false;
		}
	
		if(out & (0x0001 << CELL_AHEAD_AND_REAR_SMOKE_OFFSET))
		{
			cell_smoke_alarm[cellNo-1].cell_ahead_and_rear_smoke.is_happen = true;
		}
		else
		{
			cell_smoke_alarm[cellNo-1].cell_ahead_and_rear_smoke.is_happen = false;
		}
		
		if(out & (0x0001 << CELL_24V_ABNORMAL_OFFSET))
		{
			cell_smoke_alarm[cellNo-1].cell_24V_abnormal.is_happen = true;
		}
		else
		{
			cell_smoke_alarm[cellNo-1].cell_24V_abnormal.is_happen = false;
		}
	}

	return status;
}

bool PinsGroup_AXM::get_cell_action_abnormal_alarm(int cellNo)
{
	bool status = false;
	uint16_t out = 0;
	
	status = m_slmp_dev.read_register(XM_PLC_CELL_ACTION_ALARM_ADDR(cellNo),out);
	if(status)
	{
		if(out & (0x0001 << CELL_RELIEVE_VAC_TIMEOUT_OFFSET))
		{
			cell_action_alarm[cellNo-1].relieve_vac_timeout.is_happen = true;
		}
		else
		{
			cell_action_alarm[cellNo-1].relieve_vac_timeout.is_happen = false;
		}

		if(out & (0x0001 << CELL_TRAY_EXTEND_TIMEOUT_OFFSET))
		{
			cell_action_alarm[cellNo-1].tray_extend_timeout.is_happen = true;
		}
		else
		{
			cell_action_alarm[cellNo-1].tray_extend_timeout.is_happen = false;
		}

		if(out & (0x0001 << CELL_TRAY_RETRACT_TIMEOUT_OFFSET))
		{
			cell_action_alarm[cellNo-1].tray_retract_timeout.is_happen = true;
		}
		else
		{
			cell_action_alarm[cellNo-1].tray_retract_timeout.is_happen = false;
		}

		if(out & (0x0001 << CELL_PUMP_VAC_TIMEOUT_OFFSET))
		{
			cell_action_alarm[cellNo-1].pump_vac_timeout.is_happen = true;
		}
		else
		{
			cell_action_alarm[cellNo-1].pump_vac_timeout.is_happen = false;
		}

		if(out & (0x0001 << CELL_FIREDOOR_CLOSE_TIMEOUT_OFFSET))
		{
			cell_action_alarm[cellNo-1].firedoor_off_timeout.is_happen = true;
		}
		else
		{
			cell_action_alarm[cellNo-1].firedoor_off_timeout.is_happen = false;
		}

		if(out & (0x0001 << CELL_FIREDOOR_OPEN_TIMEOUT_OFFSET))
		{
			cell_action_alarm[cellNo-1].firedoor_on_timeout.is_happen = true;
		}
		else
		{
			cell_action_alarm[cellNo-1].firedoor_on_timeout.is_happen = false;
		}

		if(out & (0x0001 << CELL_FAN_SPEED_ABNORMAL_OFFSET))
		{
			cell_action_alarm[cellNo-1].fanspeed_abnormal.is_happen = true;
		}
		else
		{
			cell_action_alarm[cellNo-1].fanspeed_abnormal.is_happen = false;
		}

		if(out & (0x0001 << CELL_PANEL_DOOR_ABNORMAL_OFFSET))
		{
			cell_action_alarm[cellNo-1].paneldoor_abnormal.is_happen = true;
		}
		else
		{
			cell_action_alarm[cellNo-1].paneldoor_abnormal.is_happen = false;
		}

		if(out & (0x0001 << CELL_TRAY_REVERSE_OFFSET))
		{
			cell_action_alarm[cellNo-1].tray_reverse.is_happen = true;
		}
		else
		{
			cell_action_alarm[cellNo-1].tray_reverse.is_happen = false;
		}
		
		if(out & (0x0001 << CELL_MAINTAIN_DOOR_DET_OFFSET))
		{
			cell_action_alarm[cellNo-1].maintain_door_abnormal.is_happen = true;
		}
		else
		{
			cell_action_alarm[cellNo-1].maintain_door_abnormal.is_happen = false;
		}
	}

	return status;
}


bool PinsGroup_AXM::get_cell_pins_repair_button_status()
{
	bool status = false;
	uint16_t out = 0;
	
	status = m_slmp_dev.read_register(XM_PLC_CELL_PINS_STATUS_ADDR,out);
	if(status)
	{
		for(int i = 0; i < MAX_CELLS_NBR+1; i++)
		{
			if(i<4)
			{
				if(out & (0x0001 << i))
				{
					pinsRepairButton[i] = 1;
				}
				else
				{
					pinsRepairButton[i] = 0;
				}
			}
			else if(i>4)
			{
				if(out & (0x0001 << i))
				{
					pinsRepairButton[i-1] = 1;
				}
				else
				{
					pinsRepairButton[i-1] = 0;
				}
			}
		}
	}

	return status;
}

bool PinsGroup_AXM::get_cell_tray_in_place_status()
{
	bool status1 = false;
	bool status2 = false;
	uint16_t out1 = 0;
	uint16_t out2 = 0;
	
	status1 = m_slmp_dev.read_register(XM_PLC_CELL1_4_TRAY_STATUS_ADDR,out1);
	status2 = m_slmp_dev.read_register(XM_PLC_CELL5_8_TRAY_STATUS_ADDR,out2);
	
	if(status1 && status2)
	{
		for(int i = 0; i < MAX_CELLS_NBR+1; i++)
		{
			if(i<4)
			{
				if(out1 & (0x0001 << i))
				{
					trayInPlace[i] = 1;
				}
				else
				{
					trayInPlace[i] = 0;
				}
				
				if(out2 & (0x0001 << i))
				{
					trayInPlace[4+i] = 1;
				}
				else
				{
					trayInPlace[4+i] = 0;
				}
			}
			else if(i>4)
			{				
				if(out1 & (0x0001 << i))
				{
					trayRiseToPlace[i-5] = 1;
				}
				else
				{
					trayRiseToPlace[i-5] = 0;
				}
				
				if(out2 & (0x0001 << i))
				{
					trayRiseToPlace[i-1] = 1;
				}
				else
				{
					trayRiseToPlace[i-1] = 0;
				}
			}
		}
	}
	else
	{
		return false;
	}
	
	return true;
}


bool PinsGroup_AXM::get_cell_fireDoor_is_open_status()
{
	bool status = false;
	uint16_t out = 0;
	
	status = m_slmp_dev.read_register(XM_PLC_CELL_FIREDOOR_IS_OPEN_ADDR,out);
	if(status)
	{
		for(int i = 0; i < MAX_CELLS_NBR+1; i++)
		{
			if(i<4)
			{
				if(out & (0x0001 << i))
				{
					fireDoorOpen[i] = 1;
				}
				else
				{
					fireDoorOpen[i] = 0;
				}
			}
			else if(i>4)
			{
				if(out & (0x0001 << i))
				{
					fireDoorOpen[i-1] = 1;
				}
				else
				{
					fireDoorOpen[i-1] = 0;
				}
			}
		}
	}

	return status;
}

bool PinsGroup_AXM::get_cell_pins_is_press_status()
{
	bool status1 = false;
	bool status2 = false;
	uint16_t out1 = 0;
	uint16_t out2 = 0;
	
	status1 = m_slmp_dev.read_register(XM_PLC_CELL1_4_PINS_IS_PRESS_ADDR,out1);
	status2 = m_slmp_dev.read_register(XM_PLC_CELL5_8_PINS_IS_PRESS_ADDR,out2);
	
	if(status1 && status2)
	{
		for(int i = 0; i < MAX_CELLS_NBR; i++)
		{
			if(i%2 == 0)	// 0 2 4 6 
			{
				if(out1 & (0x0001 << i))
				{
					pinsIsPress[i/2] = 1;
				}
				else
				{
					pinsIsPress[i/2] = 0;
				}

				if(out2 & (0x0001 << i))
				{
					pinsIsPress[i/2 + 4] = 1;
				}
				else
				{
					pinsIsPress[i/2 + 4] = 0;
				}
			}
			else		// 1 3 5 7
			{
				if(out1 & (0x0001 << i))
				{
					pinsIsUnPress[(i-1)/2] = 1;
				}
				else
				{
					pinsIsUnPress[(i-1)/2] = 0;
				}
				
				if(out2 & (0x0001 << i))
				{
					pinsIsUnPress[(i-1)/2 + 4] = 1;
				}
				else
				{
					pinsIsUnPress[(i-1)/2 + 4] = 0;
				}
			}
		}
	}
	else
	{
		return false;
	}
	
	return true;
}

bool PinsGroup_AXM::get_cell_pins_force_unpress_status()
{
	bool status = false;
	uint16_t out = 0;
	
	status = m_slmp_dev.read_register(XM_PLC_CELL_PINS_FORCE_UNPRESS_ADDR,out);
	if(status)
	{
		for(int i = 0; i < MAX_CELLS_NBR+1; i++)
		{
			if(i<4)
			{
				if(out & (0x0001 << i))
				{
					pinsForceUnPress[i] = 1;
				}
				else
				{
					pinsForceUnPress[i] = 0;
				}
			}
			else if(i>4)
			{
				if(out & (0x0001 << i))
				{
					pinsForceUnPress[i-1] = 1;
				}
				else
				{
					pinsForceUnPress[i-1] = 0;
				}
			}
		}
	}

	return status;
}


bool PinsGroup_AXM::get_cell_vacuum_value()
{
	bool status = false;
	uint16_t out = 0;

	for(int i = 0; i < MAX_CELLS_NBR; i++)
	{
		status = m_slmp_dev.read_register(XM_PLC_CELL_VAC_VALUE_ADDR(i+1),out);
		if(status)
		{
			vac_value[i] = (((short)out) / 100.0);
		}
		else
		{
			vac_value[i] = 0.0;
		}
	}

	return status;
}


bool PinsGroup_AXM::get_cell_auto_mode()
{
	bool status = false;
	uint16_t out = 0;

	for(int i = 0; i < MAX_CELLS_NBR; i++)
	{
		status = m_slmp_dev.read_register(XM_PLC_CELL_SETTING_GET_ADDR(i+1),out);
		if(status)
		{
			if(out & (0x0001 << AUTO_OPT_MODE_OFFSET))
			{
				autoMode[i] = 1;
			}
			else
			{
				autoMode[i] = 0;
			}
		}
	}

	return status;
}

bool PinsGroup_AXM::get_leak_ratio_test_ack(int cellNo,bool &ack)
{
	bool status = false;
	uint16_t out = 0;

	status = m_slmp_dev.read_register(XM_PLC_CELL_SETTING_GET_ADDR(cellNo),out);
	if(status)
	{
		if(out & (0x0001 << LEAK_RATIO_TEST_OFFSET))
		{
			ack = true;
		}
		else
		{
			ack = false;
		}
	}

	return status;
}

bool PinsGroup_AXM::get_pump_vacuum_ack(int cellNo,bool &ack)
{
	bool status = false;
	uint16_t out = 0;

	status = m_slmp_dev.read_register(XM_PLC_CELL_SETTING_GET_ADDR(cellNo),out);
	if(status)
	{
		if(out & (0x0001 << HIGH_VAC_ON_OFFSET))
		{
			ack = true;
		}
		else
		{
			ack = false;
		}
	}

	return status;
}

bool PinsGroup_AXM::get_leak_vacuum_ack(int cellNo,bool &ack)
{
	bool status = false;
	uint16_t out = 0;

	status = m_slmp_dev.read_register(XM_PLC_CELL_SETTING_GET_ADDR(cellNo),out);
	if(status)
	{
		if(out & (0x0001 << BREAK_VAC_ON_OFFSET))
		{
			ack = true;
		}
		else
		{
			ack = false;
		}
	}

	return status;
}

bool PinsGroup_AXM::get_step_para_write_ack(int cellNo,bool &ack)
{
	bool status = false;
	uint16_t out = 0;

	status = m_slmp_dev.read_register(XM_PLC_CELL_SETTING_GET_ADDR(cellNo),out);
	if(status)
	{
		if(out & (0x0001 << STEP_PARA_WRITE_IN_OFFSET))
		{
			ack = true;
		}
		else
		{
			ack = false;
		}
	}

	return status;
}


bool PinsGroup_AXM::get_plc_software_version(uint16 &out)
{
	bool status = false;
	
	status = m_slmp_dev.read_register(XM_PLC_SOFTWARE_VERSION_ADDR,out);
	return status;
}

bool PinsGroup_AXM::get_cell_keep_vacuum_test_result(int cellNo,uint16 &result)
{
	bool status = false;

	status = m_slmp_dev.read_register(XM_PLC_CELL_KEEP_VAC_RESULT_ADDR(cellNo),result);
	return status;
}

bool PinsGroup_AXM::get_cell_keep_vacuum_start_value(int cellNo,float &val)
{
	bool status = false;
	uint16_t out = 0;
	
	status = m_slmp_dev.read_register(XM_PLC_CELL_KEEP_VAC_START_VAL_ADDR(cellNo),out);
	val = (short)out/100.0;
	
	return status;
}

bool PinsGroup_AXM::get_cell_keep_vacuum_end_value(int cellNo,float &val)
{
	bool status = false;
	uint16_t out = 0;

	status = m_slmp_dev.read_register(XM_PLC_CELL_KEEP_VAC_END_VAL_ADDR(cellNo),out);
	val = (short)out/100.0;
	
	return status;
}

bool PinsGroup_AXM::get_warn_powerdown_delaytime()
{
	bool status = false;
	uint16_t out = 0;

	status = m_slmp_dev.read_register(XM_PLC_ALARM_POWERDOWN_DELAY_ADDR,out);

	if(status)
	{
		warn_poweroff_delaysec = out/10;
	}

	return status;
}

bool PinsGroup_AXM::get_emergency_powerdown_delaytime()
{
	bool status = false;
	uint16_t out = 0;

	status = m_slmp_dev.read_register(XM_PLC_EMERGENCY_POWERDOWN_DELAY_ADDR,out);

	if(status)
	{
		emstop_poweroff_delaysec = out/10;
	}

	return status;
}


bool PinsGroup_AXM::get_plc_alarm_info()
{
	bool status = false;
	
	status = get_system_abnormal_alarm();
	if(!status)
	{
		return false;
	}

/*	
	status = get_cab_smoke_alarm();
	if(!status)
	{
		return false;
	}
*/	
	//get_cab_fan_alarm();
	
	status = get_sudden_stop_alarm();
	if(!status)
	{
		return false;
	}

	for(int i = 0; i < MAX_CELLS_NBR; i++)
	{
		status = get_cell_smoke_alarm(i+1);
		if(!status)
		{
			return false;
		}
		
		status = get_cell_action_abnormal_alarm(i+1);
		if(!status)
		{
			return false;
		}
	}

	return status;
}

bool PinsGroup_AXM::get_plc_status_info()
{
	bool status = false;

#if 0
	status = get_cell_pins_repair_button_status();
	if(!status)
	{
		return false;
	}
#endif
	
	status = get_cell_tray_in_place_status();
	if(!status)
	{
		return false;
	}
	
	status = get_cell_fireDoor_is_open_status();
	if(!status)
	{
		return false;
	}
		
	status = get_cell_pins_is_press_status();
	if(!status)
	{
		return false;
	}
	
#if 0
	status = get_cell_pins_force_unpress_status();
	if(!status)
	{
		return false;
	}
#endif

	status = get_cell_vacuum_value();
	if(!status)
	{
		return false;
	}

	status = get_cell_auto_mode();
	if(!status)
	{
		return false;
	}

	status = get_warn_powerdown_delaytime();
	if(!status)
	{
		return false;
	}
	
	status = get_emergency_powerdown_delaytime();
	if(!status)
	{
		return false;
	}

	uint16 data = 0;
	status = plc_register_read(PLC_HEARTBEAT_CYCLE_ADDR,16,data);
	if(data == 0)		//心跳失败
	{
		return false;
	}

	return status;
}

bool PinsGroup_AXM::plc_register_read(int addr,int offset,uint16 &out)
{
	bool status = false;
	bool bitVal = false;

	//LOGFMTD("plc_register_read addr:%d offset:%d ",addr, offset);
	if((offset >= 0) && (offset < 16))	//读bit
	{
		status = m_slmp_dev.read_bit(addr,offset,bitVal);
		out = bitVal;
	}
	else	//读字
	{
		status = m_slmp_dev.read_register(addr,out);
	}

	return status;
}

bool PinsGroup_AXM::plc_register_write(int addr,int offset,uint16 data)
{
	bool status = false;
	
	//LOGFMTD("plc_register_write addr:%d offset:%d data:0x%04x ",addr, offset, data);
	if((offset >= 0) && (offset < 16))	//读bit
	{
		status = m_slmp_dev.write_bit(addr,offset,data);
	}
	else	//读字
	{
		status = m_slmp_dev.write_register(addr,data);
	}

	return status;
}

bool PinsGroup_AXM::return_pins_repair_button_bitVal(int cellNo)
{
	return pinsRepairButton[cellNo-1];
}

bool PinsGroup_AXM::return_tray_in_place_bitVal(int cellNo)
{
	return trayInPlace[cellNo-1];
}

bool PinsGroup_AXM::return_tray_rise_to_place_bitVal(int cellNo)
{
	return trayRiseToPlace[cellNo-1];
}

bool PinsGroup_AXM::return_fireDoor_is_open_bitVal(int cellNo)
{
	return fireDoorOpen[cellNo-1];
}

bool PinsGroup_AXM::return_pins_is_press_bitVal(int cellNo)
{
	return pinsIsPress[cellNo-1];
}

bool PinsGroup_AXM::return_pins_is_unpress_bitVal(int cellNo)
{
	return pinsIsUnPress[cellNo-1];
}

bool PinsGroup_AXM::return_pins_force_unpress_bitVal(int cellNo)
{
	return pinsForceUnPress[cellNo-1];
}

bool PinsGroup_AXM::return_auto_mode(int cellNo)
{
	return autoMode[cellNo-1];
}

float PinsGroup_AXM::return_vacuum_value(int cellNo)
{
	return vac_value[cellNo-1];
}

int PinsGroup_AXM::return_warn_poweroff_delay()
{
	return warn_poweroff_delaysec;
}

int PinsGroup_AXM::return_emstop_poweroff_delay()
{
	return emstop_poweroff_delaysec;
}

#endif

