#ifndef PINSWORKER_H_
#define PINSWORKER_H_

#include <bitset>
#include <array>

#include "Type.h"
#include "PinsGroup.h"
#include "PinsGroup_AXM.h"
#include "PinsGroup_AXMPreCharge.h"

#if defined(AWH_PRECHARGE_SYS)
#define PINS_CELLS_NUM							3		// 针床库位数量
//#define PINS_VACCUM_DV						5.0		// 允许的负压误差范围(PLC固定该值为+/-5)
#else
#define PINS_CELLS_NUM							8		// 一个PLC控制的库位数量
#endif

typedef enum
{
	TARY_UNPRESS_CMD	= 0,		// 托盘未压合
	TRAY_PROCESSING_CMD = 1,   		// 托盘压合中
	TART_PRESSED_CMD 	= 2,		// 托盘已压合
} tray_state_commond_e;


#if defined(AWH_PRECHARGE_SYS)
class PinsWorker
{
public:
    PinsWorker();
    std::thread run();

private:
    void work();
	void message_handle();
	void status_update_handle();
	void status_sync_cell(int cell_no, int sub_code, int value=0);
	void status_sync_cell(int cell_no, Pins_Status_Data_t &status);
	void pins_puber_send(int cell_no,Cmd_Cell_Msg_t &msg_struct);
	void get_cell_temp_data(int cellNo,Pins_Status_Data_t &status);
	bool error_cmp(const Fx_Error_Code &a ,const Fx_Error_Code &b);
#if 0
private:
	struct FormationTechParam_t
	{
		int   tech_type         = 0;			// 工装类型/编号
		bool  start_cmd    		= false;		// 化成启动命令标志
		bool  abort_cmd  	    = false;		// 化成终止命令标志
		bool  is_started   		= false;		// 化成工艺启的标志
		int   logic_no     		= 0xFF;			// 化成工艺状态机
		bool  c_temp_alarm 		= false;		// 恒温失败告警
		bool  power_step_finish = false;		// 电源模块工步运行完成标志
		bool  is_charging       = false;		// 充放电中状态标志
		bool  vac_leak_test_en  = false;		// 泄漏率测试使能标志
		bool  tech_acq_finish 	= false;		// 工装数据采集完成标志
		bool  multi_vac_flag    = false;        // 多段负压工艺标志
		bool  is_vac_finish     = false;		// 负压保压完成（负压达到设定值）
		float vac				= 0.0;			// 负压设定值
		float vac_wp_dv			= 0.0;			// 负压波动保护值
		int   vac_wp_tm			= 0;			// 负压波动保护时间（检测窗口）
		bool  is_vac_wp_alarm   = false;		// 负压波动保护触发标志
		time_t vac_wp_triger_tm = 0;			// 负压波动保护触发时间
		bool  is_vac_wp_handle  = false;		// 负压波动保护处理标志
	};

	struct HandOperationInfo_t
	{
		int   last_step_no	 = 0;				// 最后读取的工步号
		bool  is_vac_started = false;			// 抽负压启动标志
		bool  is_vac_stopped = false;			// 泄负压启动标志
	};
#endif
private:
	//PinsGroup           	m_pins[PINS_CELLS_NUM];			// 针床库位读写
	Pins_Status_Data_t  	m_pins_status[PINS_CELLS_NUM];	// 针床库位状态信息
	//HandOperationInfo_t 	m_handOp_Info[PINS_CELLS_NUM];	// 手动操作的相关信息
	//FormationTechParam_t  m_bf_sm[PINS_CELLS_NUM];		// 化成工艺状态机信息
	PinsGroup_AXMPreCharge	m_pins_precharge[PINS_CELLS_NUM];	// 预充针床
	long m_last_env_time = 0;		//上1次状态采集时间
	bool last_pins_state[PINS_CELLS_NUM];						//上1次针床状态
	bool is_sudden_stop[PINS_CELLS_NUM];						//针床急停标志
};
#else
class PinsWorker
{
public:
    PinsWorker();
    std::thread run();

private:
    void work();
	void message_handle();
	void pre_formation_block_test();
	void pre_formation_leak_test();
	void formation_vac_task();
	std::string get_funccode_string(int funccode);
	void pins_ack_reply(int cell_no,int funcode,int status,uint16 data);
	void send_pins_alarm(int err_code,int cell_no);
	void status_update_handle();
	void plc_alarm_mask_proc(int cellNo,std::vector<alarm_erc_t> &alarm_erc_vector);
	void plc_alarm_handle();

private:
	void get_cell_temp_data(int cellNo,Pins_Status_Data_t &status);
	void get_cell_pins_data(int cellNo,Pins_Status_Data_t &status);
	void status_sync_cell(int cell_no, int sub_code, int value=0);
	void status_sync_cell(int cell_no, Pins_Status_Data_t &status);
	void pins_puber_send(int cell_no,Cmd_Cell_Msg_t &msg_struct);

private:
	PinsGroup_AXM           m_pins_work;						// 针床库位读写
	Pins_Status_Data_t  	m_pins_status[PINS_CELLS_NUM];		// 针床库位状态信息
	plc_vac_para_setting_t  m_vac_para[PINS_CELLS_NUM];			// 化成前负压参数		
	plc_step_vac_t			m_format_vac[PINS_CELLS_NUM];		// 化成负压参数

	plc_test_state_e 		m_block_test_state[PINS_CELLS_NUM];	// 堵塞测试流程状态
	plc_test_state_e 		m_leak_test_state[PINS_CELLS_NUM];	// 保压测试流程状态
	plc_vac_state_e 		m_vac_state[PINS_CELLS_NUM];		// 负压流程状态

	uint16_t statewait_timeout[PINS_CELLS_NUM];					// 等待超时时间
	uint64_t plc_alarm_mask[PINS_CELLS_NUM];					// PLC告警项掩码
	std::bitset<PINS_CELLS_NUM> fire_emergency_flag;			// 消防保护发生标志
	uint8_t ack_delay_time[PINS_CELLS_NUM];						// 延迟应答时间
	
	long m_last_env_time = 0;									//上1次状态采集时间
    bool is_sudden_stop;										//针床急停标志
#if defined(AWH_FORMATION_SYS)    
    bool is_vacuum_error[PINS_CELLS_NUM];						//负压值异常
#endif    
    bool last_pins_state[PINS_CELLS_NUM];						//上1次针床状态
};
#endif
#endif //PINSWORKER_H_


