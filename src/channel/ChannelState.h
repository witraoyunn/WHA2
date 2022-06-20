#ifndef CHANNELSTATE_H_
#define CHANNELSTATE_H_

#include "utility.h"
//#include "Type.h"
#include <vector>
#include "Configuration.h"

#define HEARTBEAT_TIMEOUT_MAX		3		// 心跳超时次数
#define INFO_SIZE					128

typedef enum
{
	STATE_NO_RUN	= 0x00, 	// 未运行
	STATE_PAUSE		= 0x01, 	// 暂停
	STATE_RUNNING	= 0x02, 	// 运行中
	STATE_FINISH	= 0x03, 	// 完成
	STATE_START		= 0x04, 	// 流程开始
	STATE_WAITING	= 0x05,		// 等待
	STATE_CONTACT 	= 0x06,		// 接触测试
} JN9504_Step_Record_e;

//上报扩展记录数据
typedef struct
{
	uint16 sum_step;						// 累加工步数
	uint16 loop_no; 						// 循环序号
	float batteryTemp;						// 电池温度
	float vaccum;							// 负压值
	float cellTemp[CELL_TEMP_ACQ_NUM];		// 库位温度
}Step_Process_Addition_Data_t;

typedef struct
{
	float cc_capacity;
	int cc_time;
	float cv_capacity;
	int cv_time;
}Step_Capacity_Record_t;

//监测记录数据
typedef struct
{
	int run_state;				// 运行状态
	float voltage;				// 电池电压
    float current;				// 输出电流
    float pvol;					// 输出电压
    float v_cotact;				// 探针电压
    float capacity;				// 电池容量
    float r_cotact;				// 接触电阻
	float r_output;				// 输出回路电阻
	float run_time;				// 运行时间
#if defined(AWH_FORMATION_SYS)	
	float vaccum;				// 负压值
	float vac_set;				// 负压设定值
#endif
	uint16 step_no;				// 工步号
	uint16 CC_CV_flag;			// 恒流恒压标志  0:恒流  1:恒压
}Step_Process_Record_Data_t;

//温度采集记录数据
typedef struct
{
	float batteryTemp;					// 电池温度
	float cellTemp[CELL_TEMP_ACQ_NUM];	// 库位温度
}AuxTempAcq_Record_Data_t;

//通道运行状态记录数据
typedef struct
{
	Step_Process_Record_Data_t proceData;
	AuxTempAcq_Record_Data_t tempData;
}Channel_Record_Data_t;


//采集温度
typedef struct
{
	uint8 chan;			//通道号
	float temp;			//采集温度
}Acq_Temperature_Data_t;

#pragma pack(1)
typedef struct
{
	bool   is_stop;					// 截止数据标志
	bool   undefine1;
	bool   undefine2;
	bool   undefine3;
	int    cell_no;
	int    ch_no;
	
    long long timestamp;			// 时间戳
    uint8  run_state;				// 运行状态
    uint8  step_type;				// 工步类型
    uint16 step_no;					// 工步号
    
    float  run_time;				// 运行时间
    float  voltage;					// 电池电压
    float  current;					// 输出电流
    float  pvol;					// 输出电压
    float  v_cotact;				// 探针电压
    float  capacity;				// 电池容量
	float  energy;					// 电池能量

	uint16 CC_CV_flag;				//CC CV标志
	uint8 err_mask;					//故障类型
	uint8 end_type;					//截止类型

	//扩展记录数据
	float r_cotact;					//接触电阻
	float r_output;					//输出回路电阻
	uint16 sum_step;				// 累加工步数
	uint16 loop_no; 				// 循环序号
	float batteryTemp;				// 电池温度
	float vaccum;					// 负压值
	float cellTemp[CELL_TEMP_ACQ_NUM];	// 库位温度
		
	char bat_type[INFO_SIZE];		// 电芯类型
	char stepConf_no[INFO_SIZE];	// 工步配方的编号
	char pallet_barcode[INFO_SIZE];	// 托盘码
	char batch_no[INFO_SIZE];		// 批次号
	char bat_barcode[INFO_SIZE];	// 电芯码
	int totalTime;
	int	run_result;					// 运行完成结果
	int ng_reason;					// NG原因
	float cc_capacity;				// 恒流容量
	int cc_second;					// 恒流时间
	float cv_capacity;				// 恒压容量
	int cv_second;					// 恒压时间
} Step_Process_Data_t;
#pragma pack()

class ChannelState
{
public:
    ChannelState();

	ChannelState& operator= (const ChannelState& obj);
	void reset_heartbeat_tm();
	void set_heartbeat_tm();
	bool is_heartbeat_timeout();
#ifndef PROTOCOL_5V160A		
	int  get_heartbeat_lost_tm();
    void set_running(bool flag);
    bool is_running();
#endif	
#if 0	
	void set_step_vac(bool flag);
	bool is_step_vac();
	void set_vac_value(int8_t value);
	int8_t get_vac_value();
#endif	
	void set_step_finish(bool flag);
	bool is_step_finish();
#if (defined(AWH_GRADING_SYS) || defined(AWH_PRECHARGE_SYS))	
	void set_stepcache_empty(bool flag);
	bool is_stepcache_empty();
#endif	
	void push_error_code(int code);
	void clear_error_code();
	int get_lastest_error_code();
	std::vector<int> get_error_code();

	void set_version_info(char *str);
	bool get_version_info(char *str);

	void set_addition_status(Step_Process_Addition_Data_t &data);
	Step_Process_Addition_Data_t get_addition_status();
	bool is_processData_update();
	void update_step_processData(Step_Process_Record_Data_t &rec);
	void get_step_processData(Step_Process_Record_Data_t &rec);
	void set_processData_info_code();
	void reset_info_code();

	void cache_step_processRecord(Step_Process_Data_t &rec,bool has_record);
	void report_processRecord_cache(int cellNo);

	void push_temperature_data(Acq_Temperature_Data_t data);
	void clear_temperature_data();
	void get_temperature_data(std::vector<Acq_Temperature_Data_t> &tempData);

	void set_record_frequency(int recTime);
	void set_total_runtime(int time);
	int get_total_runtime();
	void do_runtime_counter();
	void response_record_hook(Step_Process_Data_t &processData);
	void push_data_to_CabDBServer(int cellNo,Step_Process_Data_t &data);
	void saveCutOffData(int cellNo,Step_Process_Data_t &data);
	void saveProcessData(int cellNo,Step_Process_Data_t &data);
	int return_suspend_cnt();
	void response_record_send(int cell_no,uint8 errorType,int &errorCode,bool &err_flag);

public:
	char m_battery_type[INFO_SIZE];
	char m_stepConf_number[INFO_SIZE];
	char m_pallet_barcode[INFO_SIZE];
	char m_batch_number[INFO_SIZE];
	char m_battery_barcode[INFO_SIZE];

	char lower_version[64];
	
private:
	int	m_heartbeat_cnt;
#ifndef PROTOCOL_5V160A	
	time_t m_last_hb_tm;
	bool m_is_running;
#endif	
    bool m_is_step_finish;
#if (defined(AWH_GRADING_SYS) || defined(AWH_PRECHARGE_SYS))
	bool m_is_stepcache_empty;
#endif
	bool m_rec_update_flag;
	
	int recFreq;
	int reCounter;
	int totalRunTime;	//总运行时间
	int re_suspend_cnt;	//暂停次数
	Step_Capacity_Record_t m_capacity_rec;
	Step_Process_Addition_Data_t addStatus;
	Step_Process_Record_Data_t m_record;
	Step_Process_Data_t m_processData;
	Step_Process_Data_t m_last_processData;
	
	std::vector<int> m_err_code;
	std::vector<int> m_prev_err_code;
	std::vector<Acq_Temperature_Data_t> m_temperature_data;
	std::deque<Step_Process_Data_t> m_record_deque;
};

#endif //CHANNELSTATE_H_


