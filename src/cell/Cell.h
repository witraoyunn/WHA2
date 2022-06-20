#ifndef CELL_H_
#define CELL_H_

#include <thread>
#include <bitset>

#include "Channel.h"
#include "ChannelServer.h"
#include "EpollMan.h"
#include "Type.h"
#include "XmlProcAPI.h"

#ifdef ZHONGHANG_SYSTEM_PROTECTION	
#define FIRE_ALARM_CLASS_NUM			6		// 消防告警类别数量
#endif

#define OTP1_CONTINUE_THRESHOLD			5		// 通道OPT1持续阈值

typedef enum
{
	S_PROCESS_START	= 0x01, 	// 工艺启动
	S_BLOCK_TEST	= 0x02, 	// 堵塞测试
	S_FIREDOOR_CLOSE= 0x03, 	// 消防门关闭
	S_TRAY_PRESS	= 0x04, 	// 托盘压合
	S_KEEP_VAC_TEST	= 0x05,		// 保压测试
	S_PUMP_VAC		= 0x06,		// 抽真空
	S_START_STEP	= 0x07,		// 启动工步
	S_LEAK_VAC		= 0x08,		// 泄压
	S_STEP_EXEC		= 0x09,		// 工步执行中
	S_PROCESS_END	= 0x0A,		// 工步完成
	S_BREAK_VAC		= 0x0B,		// 破真空
	S_TRAY_UNPRESS	= 0x0C, 	// 托盘脱开
	S_PROCESS_ERROR	= 0x0D, 	// 异常
	S_PROCESS_IDLE	= 0x0E		// 空闲
} Process_work_state_e;

typedef enum
{
	LEAK_TEST_NG    = 1,
	VAC_FLUC_NG     = 2,
	CONTACT_TEST_NG = 3,
	NG_CHAN_EXCEED  = 4,
	PINS_ERR_BAN    = 5,
	CONTACT_V_OVER  = 6,
	TEMPACQ_ERR_BAN = 7
}Cell_forbid_type_e;

typedef enum
{
	FIRE_PROTECT_LEVEL1 = 11,
	FIRE_PROTECT_LEVEL2 = 12,
	FIRE_PROTECT_LEVEL3 = 13
}Fire_Protect_level_e;

typedef struct
{
	int step_no;
	float pump_vac;
	int pump_timeout;
	float leak_vac;
	int leak_timeout;
	int in_running;
}step_vac_para_t;

typedef struct
{
	bool OTP1;		//超温告警
	bool OTP2;		//超高温告警
	int OTP1_accum;	//连续次数
}temp_alarm_t;

typedef struct
{
	uint8_t OTP1_cnt;		//超温计数
	uint8_t OTP2_cnt;		//超高温计数
}temp_alarm_counter_t;

#if defined(FORMATION_TECH_AUTO)
struct FormationTechParam_t
{
	int   tech_type     = 0;			// 工装类型/编号
	bool  start_cmd  	= false;		// 化成启动命令标志
	bool  abort_cmd  	= false;		// 化成终止命令标志
	bool  is_started 	= false;		// 化成工艺启动的标志
	int   logic_no   	= 0xFF;			// 化成工艺状态机
	bool  is_pins_auto  = false;		// 针床自动模式标志
	bool  is_vac_finish	= false;		// 保压完成
	bool  is_pins_finish = false;		// 化成工艺完成
	bool  is_charging   = false;		// 充放电中状态标志
	bool  is_wait_vac   = false;        // 等待負壓標誌
	int   vac_param     = 0;            // 負壓值
	bool  is_vac_next   = false;		// 负压工步跳转标志
	bool  is_tech_test_finish = false;	// 工装测试完成标志
	time_t power_start_tm = 0;			// 电源模块工步启动时间
	int   follow_step_no  = 0;			// 电源模块接续工步号
};
#endif

#ifdef ZHONGHANG_SYSTEM_PROTECTION		
	//#pragma pack(1)
	#pragma pack(push, 1)
	struct FireAlarmInfo_t
	{
		uint64_t chs_temp_warning   = 0;
		uint64_t chs_temp_alarm     = 0;
		uint64_t chs_temp_emergency = 0;
			
		bool is_batTemp_warning    = false;	// 电池过温预警
		bool is_batTemp_alarm	   = false;	// 电池超温告警
		bool is_batTemp_emergency  = false;	// 电池超高温告警
		bool is_smoke_warning      = false;	// 轻烟预警
		bool is_smoke_alarm        = false;	// 浓烟告警
		bool is_cellTemp_emergency = false;	// 库位超高温告警

		bool is_trigger_flag  = false;		// 告警触发标志
		bool is_executed_flag = false;		// 保护执行标志
	};
	#pragma pack(pop)
#endif

class Cell
{
public:
    Cell(){}
#if defined(AWH_FORMATION_SYS)	
    Cell(int cell_no,int can_port);
#else
	Cell(int cell_no);
#endif
    Cell& operator= (const Cell& obj);
    std::thread run();

private:
    void work();
	void message_handle(Cmd_Cell_Msg_t &msg);
#if defined(AUTO_CONNECT)
	void net_auto_connect();
	void channel_status_sync();
#endif
	void channels_linkstate_send(std::vector<int> &chanArrary,std::string ip,int state);
	void channel_state_update();
	void channel_clear_active(int chanNo);
	void channel_step_active(int chanNo,int currentStep,int nextStep);
	void set_runstatus_techinfo(int chIdx,int type,char *src);
#if defined(ENABLE_RUNSTATUS_SAVE)
	bool channel_saveStatus_active(int chNo,int en);
	bool update_saveStatus_stepinfo(int chNo);			//存储运行状态工步信息
	bool update_saveStatus_techinfo(int chNo);			//存储运行状态工艺信息
#endif
	void work_logic_init();
	void work_logic_execute();
	void work_logic_recv();
	void work_logic_handle();
	void channel_temperature_update();
	void pins_status_update();
	void cell_alarm_set_error_code(int err);

	void hearbeat_detect();
	void charge_discharge_check();
	uint8_t fireProtection_check();
	void fireProtection_execute(uint8_t level);
	void fireProtection_handle();
	
#if defined(ENABLE_PLC)
	void send_pins_cmd(Cmd_Pins_Data_t &data_union,uint8 funccode);
#endif
	void exec_process_status_notify(int status_code);
	int get_next_workstepNo(work_step_recipe_t &stepInfo);
	bool loop_workstep_jump(int chIdx, int &vecIdx, uint32 &nextStepIdx, work_step_recipe_t &workstep_info);
	void channel_command_push(uint32_t ch_no,uint8_t func_code,Channel_Data_t &pow_data);
	void process_start();
	void process_block_test();
#if defined(AWH_GRADING_SYS)
	void firedoor_close_exec();
#endif
	void tray_press_exec();
	void process_keep_vac_test();
	void process_pump_vac_task();
	void channel_step_startup();
	void process_leak_vac_task();
	void channel_step_execute();
	void process_break_vac_task();
	//void tray_unpress_exec();
	void process_end();
	void process_error_proc();

	//void step_vac_check();
	//void channel_vaccumVal_update(float val);
	//void channel_celltemper_update(float *temper);
#if defined(FORMATION_TECH_AUTO)
	void power_vac_step_ctrl();
	void power_step_ctrl(bool start);
	void status_sync_pins(int sub_code, int val=-1);
	void batteryFormation_status_notify(int status_code);
	void techTest_status_notify(int status_code);
	void batteryFormation_logic();
#endif

#ifdef ZHONGHANG_SYSTEM_PROTECTION
	void fireAlarmInfo_clear();
	void power_step_puase();
	void alarmInfo_post(int err_code, int ch_no=-1);
	void actionCmd_to_pins(int sub_code, int val=-1);
	void protectionAction_execute();
	void fireProtection_handler();
	void alarmProtection_handle(int ch, int code);
#endif

private:
	ChannelServer m_channels[MAX_CHANNELS_NBR];
	EpollMan m_epoll_cell;
#if defined(ENABLE_RUNSTATUS_SAVE)
	XmlProcAPI chanRunStatus;
#endif

    int    m_cell_no;
	time_t m_last_sync_time;
	long   m_last_query_time;
	long   m_last_beat_time;
	long   m_last_work_time;
#if defined(AUTO_CONNECT)	
	uint64_t m_chsEnFlag;
	uint64_t m_syncTmFlag;
#endif
	std::bitset<MAX_CHANNELS_NBR> m_channel_start_mask;
	std::bitset<MAX_CHANNELS_NBR> m_channel_jump_mask;
	std::bitset<MAX_CHANNELS_NBR> m_channel_wait_mask;
	std::bitset<MAX_CHANNELS_NBR> m_channel_end_mask;
	std::bitset<MAX_CHANNELS_NBR> m_channel_error_mask;
	uint32_t nextStepIdx[MAX_CHANNELS_NBR];
	channel_run_status_t runStatus[MAX_CHANNELS_NBR];
	temp_alarm_t channel_temp_alarm[MAX_CHANNELS_NBR];

	Pins_Status_Data_t pins_cell_status;
#if defined(AWH_FORMATION_SYS)	
	step_vac_para_t	m_step_vac;
	step_vac_para_t	m_last_step_vac;
#endif
	uint8_t m_state;
	uint8_t m_last_state;
	uint8_t m_sync_timeout;
	uint8_t cell_NG_mask;
	int pump_vac_ready;
	int leak_vac_ready;
#if defined(AWH_FORMATION_SYS)	
	int m_can_idx;							//can接口索引号
#endif
	uint8_t last_fire_level;
	bool pass_through_enable;
	bool query_ack_wait;
	bool last_work_flag;
	char last_batch_no[INFO_SIZE];					//库位批次号记录
	std::vector<int> fire_error_code;				//消防事件告警码
	std::vector<lower_mc_version_t> device_version;	//下位机模块版本

#if defined(FORMATION_TECH_AUTO)		
	FormationTechParam_t  m_bf_sm;			// 化成工艺状态机信息
#endif
#ifdef ZHONGHANG_SYSTEM_PROTECTION	
	FireAlarmInfo_t m_fireAlarm[FIRE_ALARM_CLASS_NUM];	// 消防级别0~5告警信息
#endif
};

#endif //CELL_H_


