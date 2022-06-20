#ifndef PINSGROUP_AXM_H_
#define PINSGROUP_AXM_H_
#include <bitset>
#include "Type.h"
#include "SLMPDriver.h"

#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))

#define MAX_PINS_ERROR_CODE	41

typedef enum
{
	FACE_PANEL_STOP		 = 19,
	RELEASE_VAC_TIMEOUT	 = 27,
	TRAY_PRESS_TIMEOUT 	 = 28,
	TRAY_UNPRESS_TIMEOUT = 29,
	PUMPING_VAC_TIMEOUT	 = 30,
	FIREDOOR_CLOSE_TIMEOUT = 31,
	FIREDOOR_OPEN_TIMEOUT  = 32,
	VACUUM_PARA_ABNORMAL  = 40,
	VACUUM_VALUE_POSITIVE  = 41
}plc_alarm_type_e;

typedef enum
{
	UP_T	= 0x00,			// 上升
	DOWN_T 	= 0x01,   		// 下降
} plc_action_type_e;

typedef enum
{
	AUTO_M		= 0x00,		// 自动模式
	MAINTAIN_M 	= 0x01,   	// 维护模式
} plc_opt_mode_e;

typedef enum
{
	ENABLE_C	= true,		// 启动
	DISABLE_C	= false,   	// 停止
} plc_en_control_e;

typedef enum
{
	RED_I		= 0x00,		// 红灯
	YELLOW_I	= 0x01,   	// 黄灯
	GREEN_I		= 0x02,		// 绿灯
	BUZZER_I	= 0x03,		// 蜂鸣器
} plc_indicator_type_e;

typedef enum
{
	E_EXTEND		= 1,
	E_WAIT 			= 2,
	E_JUDGE 		= 3,
	E_END 			= 4,
	E_ERROR 		= 5,
	E_VAC_BREAK 	= 6,
	E_BREAK_WAIT 	= 7,
	//E_TRAY_RESET	= 8,
}plc_test_state_e;

typedef enum
{
	E_VAC_WAIT 	= 1,
	E_VAC_JUDGE = 2,
	E_LEAK_WAIT	= 3,
	E_LEAK_JUDGE= 4,
	E_STATE_END = 5,
	E_STATE_ERR	= 6,
	E_VAC_RELEASE  = 7,
	E_RELEASE_WAIT = 8,
}plc_vac_state_e;

typedef struct
{
	float blockSetVal;
	int blockPumpTime;
	float KeepVacVal;
	int KeepVacStableTime;
	float KeepVacLeakRate;
	int KeepVacTime;
	int KeepVacTestTimeout;
}plc_vac_para_setting_t;

typedef struct
{
	float pump_vac_value;			// 抽真空值
	float leak_vac_value;			// 破真空值
	uint16 pump_vac_timeout;		// 抽真空超时时间
	uint16 leak_vac_timeout;		// 破真空超时时间
}plc_step_vac_t;

#if 0
typedef struct
{
	bool tray_down;
	bool tray_up;
	bool autoMode;
	bool maintainMode;
	bool crane_fork_busy;
	bool highVac_on;
	bool leakVac_on;
	bool firedoor_up;
	bool firedoor_down;
	bool leak_ratio_test;
	bool fan_on;
	bool block_test;
	bool stepPara_write;
}plc_cell_setting_t;
#endif

typedef struct
{
	bool is_happen;
	int alarm_erc;
}alarm_erc_t;

typedef struct
{
	alarm_erc_t AC1_over_voltage;
	alarm_erc_t AC1_low_voltage;
	alarm_erc_t AC1_over_current;
	alarm_erc_t AC2_over_voltage;
	alarm_erc_t AC2_low_voltage;
	alarm_erc_t AC2_over_current;

	alarm_erc_t low_pressure;
	alarm_erc_t cab_temp_err;
	alarm_erc_t heartbeat_stop;
}plc_sys_abnormal_alarm_t;

typedef struct
{
	alarm_erc_t face_panel_ctl_stop;
	alarm_erc_t HMI_ctl_stop;
	alarm_erc_t AC1_stop_unreset;
	alarm_erc_t AC2_stop_unreset;
	alarm_erc_t cab_ctl_sudden_stop;
}plc_sudden_stop_alarm_t;

typedef struct
{
	alarm_erc_t fan1_overload;
	alarm_erc_t fan2_overload;
	alarm_erc_t fan3_overload;
	alarm_erc_t fan4_overload;
	alarm_erc_t fan5_overload;
	alarm_erc_t fan6_overload;
	alarm_erc_t fan7_overload;
	alarm_erc_t fan8_overload;
}plc_cab_fan_alarm_t;

typedef struct
{
	alarm_erc_t cell_ahead_smoke;
	alarm_erc_t cell_rear_smoke;
	alarm_erc_t cell_ahead_and_rear_smoke;
	alarm_erc_t cell_24V_abnormal;
}plc_cell_smoke_alarm_t;

typedef struct
{
	alarm_erc_t relieve_vac_timeout;
	alarm_erc_t tray_extend_timeout;
	alarm_erc_t tray_retract_timeout;
	alarm_erc_t pump_vac_timeout;
	alarm_erc_t firedoor_off_timeout;
	alarm_erc_t firedoor_on_timeout;
	alarm_erc_t fanspeed_abnormal;
	alarm_erc_t paneldoor_abnormal;
	alarm_erc_t tray_reverse;
	alarm_erc_t maintain_door_abnormal;
}plc_cell_action_alarm_t;


class PinsGroup_AXM
{
public:
	PinsGroup_AXM();
	~PinsGroup_AXM();

	bool do_plc_connect(const char *ip,int port);
	bool is_connected();
	void close_connect();

	void init_plc_alarm_erc();
//PLC库位设置
	bool set_tray_action(int cellNo,uint8 action);
	bool set_PLC_workMode(int cellNo,uint8 mode);

	bool start_vacuum_control(int cellNo,bool en);
	bool break_vacuum_control(int cellNo,bool en);

	bool fireDoor_control(int cellNo,uint8 action);
	bool leak_ratio_test(int cellNo,bool en);
	bool vent_fan_control(int cellNo,bool en);
	bool vac_block_test(int cellNo,bool en);
	bool step_para_write_in(int cellNo,bool en);
	bool tools_power_control(int cellNo,bool en);
//心跳信号设置	
	bool set_heartbeat_signal(uint16 value);

//系统灯
	bool set_system_lamp_buzzer(uint8 item);

//库位OPT设置
	bool set_cell_OTP2(int cellNo);
	bool set_cell_OTP1(int cellNo);

//设置校准模式
	bool set_calibrate_mode(int cellNo);

//PLC 异常重置
	bool clear_once_alarm();

//启动消防
	bool set_fire_fighting(uint8 cellNo);

//真空误差值设定
	bool set_vacuum_error_value(short err_val);

//保压破真空维持值
	bool set_keep_vacuum_break_value(float val);

//保压设定值
	bool set_keep_vacuum_test_value(float val);

//库位真空设定值  D380~D397不使用了
	//bool set_cell_vacuum_value(int cellNo,short val);
//库位破真空设定值
	//bool set_cell_break_vacuum_value(int cellNo,short val);

//设置库位工步号
	bool set_cell_stepNo(int cellNo,uint16 stepNo);

//设置库位保压值
	bool set_cell_keep_vacuum_target(int cellNo,float val);

//设置库位保压漏率
	bool set_cell_keep_vacuum_leakrate(int cellNo,float val);

//设置库位保压稳定时间
	bool set_cell_keep_vacuum_stableTime(int cellNo,uint16 t);

//设置库位保压时间
	bool set_cell_keep_vacuum_time(int cellNo,uint16 t);

//设置高真空值
	bool set_cell_high_vacuum_value(int cellNo,float val);

//设置库位抽真空超时时间
	bool set_cell_pump_vacuum_timeout(int cellNo,uint16 t);

//设置库位泄真空超时时间
	bool set_cell_leak_vacuum_timeout(int cellNo,uint16 t);

//设置库位泄真空值
	bool set_cell_leak_vacuum_value(int cellNo,float val);

#if 0
//获取库位执行结果
	bool get_cell_setting_result(int cellNo,plc_cell_setting_t result);
#endif

//获取漏率测试应答
	bool get_leak_ratio_test_ack(int cellNo,bool &ack);
//获取抽真空应答
	bool get_pump_vacuum_ack(int cellNo,bool &ack);
//获取泄真空应答
	bool get_leak_vacuum_ack(int cellNo,bool &ack);
//获取参数写入状态
	bool get_step_para_write_ack(int cellNo,bool &ack);
//获得PLC软件版本
	bool get_plc_software_version(uint16 &out);

//获取保压测试结果
	bool get_cell_keep_vacuum_test_result(int cellNo,uint16 &result);
//保压开始压力值
	bool get_cell_keep_vacuum_start_value(int cellNo,float &val);
//保压完成压力值
	bool get_cell_keep_vacuum_end_value(int cellNo,float &val);

	bool plc_register_read(int addr,int offset,uint16 &out);
	bool plc_register_write(int addr,int offset,uint16 data);

	bool get_plc_alarm_info();
	bool get_plc_status_info();

	bool return_pins_repair_button_bitVal(int cellNo);
	bool return_tray_in_place_bitVal(int cellNo);
	bool return_tray_rise_to_place_bitVal(int cellNo);
	bool return_fireDoor_is_open_bitVal(int cellNo);
	bool return_pins_is_press_bitVal(int cellNo);
	bool return_pins_is_unpress_bitVal(int cellNo);
	bool return_pins_force_unpress_bitVal(int cellNo);
	bool return_auto_mode(int cellNo);
	float return_vacuum_value(int cellNo);
	int return_warn_poweroff_delay();
	int return_emstop_poweroff_delay();

private:
	void init_system_abnormal_alarm();
	void init_cab_smoke_alarm();
	void init_cab_fan_alarm();
	void init_sudden_stop_alarm();
	void init_cell_smoke_alarm();
	void init_cell_action_abnormal_alarm();
//获取系统异常告警
	bool get_system_abnormal_alarm();
//获取烟雾告警
	bool get_cab_smoke_alarm();
//获取电源柜风扇告警
	bool get_cab_fan_alarm();
//获取急停告警
	bool get_sudden_stop_alarm();
//获取库位烟雾告警
	bool get_cell_smoke_alarm(int cellNo);
//获取库位动作异常告警
	bool get_cell_action_abnormal_alarm(int cellNo);

//获取库位真空值
	bool get_cell_vacuum_value();
//获取库位脱开状态
	bool get_cell_pins_repair_button_status();
//获取库位托盘是否在位状态
	bool get_cell_tray_in_place_status();
//获取库位托盘是否上升到位状态
	//bool get_cell_tray_rise_to_place_status();	//合并
//获取库位消防门是否开启状态
	bool get_cell_fireDoor_is_open_status();
//获取库位是否压合状态
	bool get_cell_pins_is_press_status();
//获取库位是否脱开状态
	//bool get_cell_pins_is_unpress_status();		//合并
//获取库位强制脱开状态
	bool get_cell_pins_force_unpress_status();

//获取告警断电延时时间
	bool get_warn_powerdown_delaytime();
//获取紧急断电延时时间
	bool get_emergency_powerdown_delaytime();
// 获取库位是否自动模式
	bool get_cell_auto_mode();

public:
	plc_sys_abnormal_alarm_t sys_abnormal_alarm;
	alarm_erc_t cab_smoke_alarm;
	plc_cab_fan_alarm_t cabfan_overload_alarm;				// 电源柜排风扇过载
	plc_sudden_stop_alarm_t sudden_stop_alarm;
	plc_cell_smoke_alarm_t cell_smoke_alarm[MAX_CELLS_NBR];
	plc_cell_action_alarm_t cell_action_alarm[MAX_CELLS_NBR];
	
private:
	SLMPDriver m_slmp_dev;         		// 三菱SLMP
	bool is_plc_connected;

	int warn_poweroff_delaysec;			// 告警断电延迟时间
	int emstop_poweroff_delaysec;		// 急停断电延迟时间
	std::bitset<8> pinsRepairButton;	// 库位维修按钮状态
	std::bitset<8> trayInPlace;			// 托盘在位状态
	std::bitset<8> trayRiseToPlace;		// 托盘上升到位状态
	std::bitset<8> fireDoorOpen; 		// 消防门是否开启
	std::bitset<8> pinsIsPress; 		// 针床压合状态
	std::bitset<8> pinsIsUnPress; 		// 针床脱开状态
	std::bitset<8> pinsForceUnPress; 	// 针床强制弹开状态
	std::bitset<8> autoMode; 			// 是否自动模式

	float vac_value[MAX_CELLS_NBR]; 	// 库位真空值
};
#endif
#endif

