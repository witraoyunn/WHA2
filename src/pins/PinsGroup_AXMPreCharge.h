#ifndef PINSGROUP_AXMPRECHARGE_H_
#define PINSGROUP_AXMPRECHARGE_H_
#include <bitset>
#include "Type.h"
#include "SLMPDriver.h"

#if defined(AWH_PRECHARGE_SYS)

class PinsGroup_AXMPreCharge
{
public:
	PinsGroup_AXMPreCharge();
	~PinsGroup_AXMPreCharge();
	
	bool do_plc_connect(const char *ip,int port);
	bool is_connected();
	void close_connect();

	//气压异常状态 
	bool get_sys_pressur_alarm();
	//系统异常状态
	bool get_sys_stop_alarm();
	//PLC心跳异常状态
	bool get_sys_heart_alarm();
	// 库门异常状态
	bool get_sys_dor_alarm();
	// 顶升气缸上行超时异常
	bool get_move_uptimeout_alarm();
	// 顶升气缸下行超时异常
	bool get_move_downtimeout_alarm();
	// 托盘防反异常
	bool get_tray_reverse_alarm();
	// 无托盘异常
	bool get_no_tray_alarm();
	// 托盘未定位异常
	bool get_tray_not_ready_alarm();
	// 排风机过载异常
	bool get_fan_overload_alarm();
	// 滚筒过载异常
	bool get_roller_overload_alarm();
	// 滚筒入库超时异常
	bool get_roller_run_timeout_alarm();
	// 滚筒出库超时
	bool get_roller_reverse_run_timeout_alarm();
	// 左侧排风扇异常
	bool get_left_fan_alarm();
	// 右侧排风扇异常
	bool get_right_fan_alarm();
	// 顶侧排风扇异常
	bool get_top_fan_alarm();
	// 烟感告警1
	bool get_smake1_alarm();
	// 烟感告警2
	bool get_smake2_alarm();
	// 电柜烟感告警
	bool get_cab_smake_alarm();
	// 针床轻感告警
	bool get_pin_smake_alarm();
	// 针床浓感告警
	bool get_pin_smake_heavy_alarm();
	// 超温告警
	bool get_high_temp_alarm();
	// 心跳数据
	bool get_heartbeat();
	// 托盘压合到位状态
	bool tray_is_extend();
	// 托盘脱开到位状态
	bool tray_is_retract();
	// 工装取电状态
	bool tool_power_onoff();
	// 冷却风扇状态
	bool cool_fan_onoff();
	// 抽风机状态
	bool air_fan_onoff();
	// 手自动模式
	bool plc_mode();
	// 初始化完成
	bool plc_is_loaded();
	// 托盘在位状态
	bool tray_is_ready();
	// 自动启动状态
	bool is_auto_start();
	// 托盘上升
	bool set_tray_moveup();
	// 托盘下降
	bool set_tray_movedown();
	// 工装取电开
	bool set_tool_power_on();
	// 工装取电关
	bool set_tool_power_off();
	// 冷却风扇开
	bool set_cool_fan_on();
	// 冷却风扇关
	bool set_cool_fan_off();
	// 抽风机开
	bool set_air_fan_on();
	// 抽风机关
	bool set_air_fan_off();
	// 清楚告警
	bool set_clear_alarm();
	// 初始化操作
	bool set_load();
	// 自动启动操作
	bool set_auto_start();
	// 停止操作
	bool set_stop();
	// 库位温度
	bool set_cell_temp(float temp[CELL_TEMP_ACQ_NUM]);
	// 设置库位温度告警阈值
	bool set_cell_temp_waring(float temp);
	// 读取库位温度告警阈值
	uint16_t get_cell_temp_waring();
	//允许入库状态
	uint16_t get_incell_onoff();
	// 入库到位通知
	uint16_t is_incell_status();
	// 托盘状态信息
	bool incell_tray_info(uint16_t val);
	// 通知托盘出库
	bool set_outcell_onoff();
	// 托盘出库完成
	uint16_t is_outcell_status();
	//获取警告状态
	bool get_error_code(Fx_Error_Code_t &err_code);
	//获取状态数据
	bool get_status_data(Pins_Status_Data_t &status);

private:
	SLMPDriver m_slmp_dev;
	int heart_timeout_cnt;
	bool is_plc_connected;
};
#endif
#endif


