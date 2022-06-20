#ifndef PINSWORKER_H_
#define PINSWORKER_H_

#include <bitset>
#include <array>

#include "Type.h"
#include "PinsGroup.h"
#include "PinsGroup_AXM.h"
#include "PinsGroup_AXMPreCharge.h"

#if defined(AWH_PRECHARGE_SYS)
#define PINS_CELLS_NUM							3		// �봲��λ����
//#define PINS_VACCUM_DV						5.0		// ����ĸ�ѹ��Χ(PLC�̶���ֵΪ+/-5)
#else
#define PINS_CELLS_NUM							8		// һ��PLC���ƵĿ�λ����
#endif

typedef enum
{
	TARY_UNPRESS_CMD	= 0,		// ����δѹ��
	TRAY_PROCESSING_CMD = 1,   		// ����ѹ����
	TART_PRESSED_CMD 	= 2,		// ������ѹ��
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
		int   tech_type         = 0;			// ��װ����/���
		bool  start_cmd    		= false;		// �������������־
		bool  abort_cmd  	    = false;		// ������ֹ�����־
		bool  is_started   		= false;		// ���ɹ������ı�־
		int   logic_no     		= 0xFF;			// ���ɹ���״̬��
		bool  c_temp_alarm 		= false;		// ����ʧ�ܸ澯
		bool  power_step_finish = false;		// ��Դģ�鹤��������ɱ�־
		bool  is_charging       = false;		// ��ŵ���״̬��־
		bool  vac_leak_test_en  = false;		// й©�ʲ���ʹ�ܱ�־
		bool  tech_acq_finish 	= false;		// ��װ���ݲɼ���ɱ�־
		bool  multi_vac_flag    = false;        // ��θ�ѹ���ձ�־
		bool  is_vac_finish     = false;		// ��ѹ��ѹ��ɣ���ѹ�ﵽ�趨ֵ��
		float vac				= 0.0;			// ��ѹ�趨ֵ
		float vac_wp_dv			= 0.0;			// ��ѹ��������ֵ
		int   vac_wp_tm			= 0;			// ��ѹ��������ʱ�䣨��ⴰ�ڣ�
		bool  is_vac_wp_alarm   = false;		// ��ѹ��������������־
		time_t vac_wp_triger_tm = 0;			// ��ѹ������������ʱ��
		bool  is_vac_wp_handle  = false;		// ��ѹ�������������־
	};

	struct HandOperationInfo_t
	{
		int   last_step_no	 = 0;				// ����ȡ�Ĺ�����
		bool  is_vac_started = false;			// �鸺ѹ������־
		bool  is_vac_stopped = false;			// й��ѹ������־
	};
#endif
private:
	//PinsGroup           	m_pins[PINS_CELLS_NUM];			// �봲��λ��д
	Pins_Status_Data_t  	m_pins_status[PINS_CELLS_NUM];	// �봲��λ״̬��Ϣ
	//HandOperationInfo_t 	m_handOp_Info[PINS_CELLS_NUM];	// �ֶ������������Ϣ
	//FormationTechParam_t  m_bf_sm[PINS_CELLS_NUM];		// ���ɹ���״̬����Ϣ
	PinsGroup_AXMPreCharge	m_pins_precharge[PINS_CELLS_NUM];	// Ԥ���봲
	long m_last_env_time = 0;		//��1��״̬�ɼ�ʱ��
	bool last_pins_state[PINS_CELLS_NUM];						//��1���봲״̬
	bool is_sudden_stop[PINS_CELLS_NUM];						//�봲��ͣ��־
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
	PinsGroup_AXM           m_pins_work;						// �봲��λ��д
	Pins_Status_Data_t  	m_pins_status[PINS_CELLS_NUM];		// �봲��λ״̬��Ϣ
	plc_vac_para_setting_t  m_vac_para[PINS_CELLS_NUM];			// ����ǰ��ѹ����		
	plc_step_vac_t			m_format_vac[PINS_CELLS_NUM];		// ���ɸ�ѹ����

	plc_test_state_e 		m_block_test_state[PINS_CELLS_NUM];	// ������������״̬
	plc_test_state_e 		m_leak_test_state[PINS_CELLS_NUM];	// ��ѹ��������״̬
	plc_vac_state_e 		m_vac_state[PINS_CELLS_NUM];		// ��ѹ����״̬

	uint16_t statewait_timeout[PINS_CELLS_NUM];					// �ȴ���ʱʱ��
	uint64_t plc_alarm_mask[PINS_CELLS_NUM];					// PLC�澯������
	std::bitset<PINS_CELLS_NUM> fire_emergency_flag;			// ��������������־
	uint8_t ack_delay_time[PINS_CELLS_NUM];						// �ӳ�Ӧ��ʱ��
	
	long m_last_env_time = 0;									//��1��״̬�ɼ�ʱ��
    bool is_sudden_stop;										//�봲��ͣ��־
#if defined(AWH_FORMATION_SYS)    
    bool is_vacuum_error[PINS_CELLS_NUM];						//��ѹֵ�쳣
#endif    
    bool last_pins_state[PINS_CELLS_NUM];						//��1���봲״̬
};
#endif
#endif //PINSWORKER_H_


