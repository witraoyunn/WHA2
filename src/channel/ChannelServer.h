#ifndef _CHANNELSERVER_H_
#define _CHANNELSERVER_H_
#include <deque>
#include <vector>

#include "Channel.h"
#include "Type.h"
#include "XmlProcAPI.h"
#include "GlobalException.h"
#include "StepException.h"

#define GLOBAL_EXC_GENERAL_NUM	9
#define GLOBAL_EXC_TEMP_NUM		13

#define CELL_VACUUM_FLUC_INDEX		8

#define CHAN_TEMPACQ_DETECT_INDEX	9
#define CHAN_ULTRA_TEMP_INDEX		12
#define CHAN_ULTRAHIGH_TEMP_INDEX	13

#define CELL_TEMPACQ_DETECT_INDEX	17
#define CELL_ULTRA_TEMP_INDEX		19
#define CELL_ULTRAHIGH_TEMP_INDEX	20

#define GLOBAL_EXC_NUM			(GLOBAL_EXC_GENERAL_NUM+GLOBAL_EXC_TEMP_NUM)

#define STEP_EXC_NUM	17

enum global_protect_type_e
{
	PROTECT_TYPE_GENERAL = 0,
	PROTECT_TYPE_TEMP = 1,
};

class ChannelServer:public Channel
{
public:
	ChannelServer(){}
#if defined(AWH_FORMATION_SYS)
	ChannelServer(int cell_no, int ch_no, int can_idx);
#else
	ChannelServer(int cell_no, int ch_no);
#endif
	~ChannelServer();

	void set_fd(int fd);
	int get_fd();
	void send_data(uint8 *src, int size);
	void recv_data(recvData_t &recv);
	//void connect(){}
	bool isConnected();
	void disconnect();
	void add_addition_status(Step_Process_Addition_Data_t &data);
	int handle_reply(bool &err_flag);

	int  execute();
#if defined(AUTO_CONNECT)	
	void sync_system_time();
#endif
	void reset_heartbeat();
	bool check_beating(bool en);
	void transmit_data(uint8 *data,int len);
#if defined(FORMATION_TECH_AUTO)	
    void start_process(int step_no);		// 开始运行工步
#endif 
	void stop_process();					// 停止运行工步
	void pause_process();				// 暂停运行工步
	void resume_process();
#if defined(FORMATION_TECH_AUTO)	
	void vac_step_process();			// 多段负压工步
#endif
	void tray_state_set(int state);
	void query_version();
	bool return_version_data(char *data);

	int get_loop_vector_size();
	void clear_loop_vector();
	void push_loop_info(loopstep_info_t data);
	int get_loop_stepNo(int idx);
	void set_loop_count(int idx,int count);
	void increase_loop_count(int idx);
	void decrease_loop_count(int idx);
	int get_loop_count(int idx);

	void set_processData_RecTime(int recTime);
	void set_total_RunTime(int time);
	int get_total_RunTime();

	void set_cmd_pause_flag(bool flag);
	bool get_cmd_pause_flag();
	void update_temperature_Data(AuxTempAcq_Record_Data_t &temp);
	void get_temperature_Data(AuxTempAcq_Record_Data_t &temp);
	int get_channel_runstate();
	uint16_t get_channel_step_no();
	int get_channel_suspend_cnt();

	void init_globalProtect_para();
	void get_globalProtect_info(global_protect_t &data);
	bool get_globalProtect_para();
	void get_workstep_recipe();
	void get_workstep_info(uint32 index,work_step_recipe_t &step_info);
	uint32 get_workstep_stepsize();
	void set_stepProtect_Para(step_protect_t &para);

	void push_error_code(int code);
	void clear_error_code();
	std::vector<int> return_all_error_code();

	void init_tempAcq_detect_exception();
	void init_global_exception();
	void init_step_exception();
	void do_global_exception_task(uint8 proType);
	void do_step_exception_task();
	void do_record_send_task(bool &err_flag);
	void do_exception_extra_task();
	void set_error_code(int error);
	void check_lower_error_alarm();
	bool is_protect_alarm_clear();
	void clear_protect_alarm_mask();
	bool is_error_in_global_alarm(uint8 errIdx);
	uint8 cell_ultra_temp_counter();
	uint8 cell_ultrahigh_temp_counter();

public:
	uint32 global_alarm_mask;
	uint32 step_alarm_mask;
	
private:
	int ch_fd;
	int m_error;
	int m_last_error;
#if defined(AWH_FORMATION_SYS)	
	int m_can_idx;
#endif
	bool global_ERC_happen;
	bool step_ERC_happen;
	bool internal_ERC_happen;
	bool ACDC_ERC_happen;
	
	bool m_temp_update_flg;
	bool m_pause_cmd_flg;

	static uint8 cellfeat[MAX_CELLS_NBR];
	std::deque<recvData_t> recv_queue;
	std::vector<loopstep_info_t> multiLoopStep;
	std::vector<work_step_recipe_t> stepArrary;
	std::vector<int> mid_error_code;
	
	global_protect_t m_global_protect;
	step_protect_t m_step_protect;
	Channel_Record_Data_t m_record;
	Channel_Record_Data_t m_last_record;
	GlobalException* p_global_ExcTask[GLOBAL_EXC_NUM];
	StepException* p_step_ExcTask[STEP_EXC_NUM];
};

#endif

