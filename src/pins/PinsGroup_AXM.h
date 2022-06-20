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
	UP_T	= 0x00,			// ����
	DOWN_T 	= 0x01,   		// �½�
} plc_action_type_e;

typedef enum
{
	AUTO_M		= 0x00,		// �Զ�ģʽ
	MAINTAIN_M 	= 0x01,   	// ά��ģʽ
} plc_opt_mode_e;

typedef enum
{
	ENABLE_C	= true,		// ����
	DISABLE_C	= false,   	// ֹͣ
} plc_en_control_e;

typedef enum
{
	RED_I		= 0x00,		// ���
	YELLOW_I	= 0x01,   	// �Ƶ�
	GREEN_I		= 0x02,		// �̵�
	BUZZER_I	= 0x03,		// ������
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
	float pump_vac_value;			// �����ֵ
	float leak_vac_value;			// �����ֵ
	uint16 pump_vac_timeout;		// ����ճ�ʱʱ��
	uint16 leak_vac_timeout;		// ����ճ�ʱʱ��
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
//PLC��λ����
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
//�����ź�����	
	bool set_heartbeat_signal(uint16 value);

//ϵͳ��
	bool set_system_lamp_buzzer(uint8 item);

//��λOPT����
	bool set_cell_OTP2(int cellNo);
	bool set_cell_OTP1(int cellNo);

//����У׼ģʽ
	bool set_calibrate_mode(int cellNo);

//PLC �쳣����
	bool clear_once_alarm();

//��������
	bool set_fire_fighting(uint8 cellNo);

//������ֵ�趨
	bool set_vacuum_error_value(short err_val);

//��ѹ�����ά��ֵ
	bool set_keep_vacuum_break_value(float val);

//��ѹ�趨ֵ
	bool set_keep_vacuum_test_value(float val);

//��λ����趨ֵ  D380~D397��ʹ����
	//bool set_cell_vacuum_value(int cellNo,short val);
//��λ������趨ֵ
	//bool set_cell_break_vacuum_value(int cellNo,short val);

//���ÿ�λ������
	bool set_cell_stepNo(int cellNo,uint16 stepNo);

//���ÿ�λ��ѹֵ
	bool set_cell_keep_vacuum_target(int cellNo,float val);

//���ÿ�λ��ѹ©��
	bool set_cell_keep_vacuum_leakrate(int cellNo,float val);

//���ÿ�λ��ѹ�ȶ�ʱ��
	bool set_cell_keep_vacuum_stableTime(int cellNo,uint16 t);

//���ÿ�λ��ѹʱ��
	bool set_cell_keep_vacuum_time(int cellNo,uint16 t);

//���ø����ֵ
	bool set_cell_high_vacuum_value(int cellNo,float val);

//���ÿ�λ����ճ�ʱʱ��
	bool set_cell_pump_vacuum_timeout(int cellNo,uint16 t);

//���ÿ�λй��ճ�ʱʱ��
	bool set_cell_leak_vacuum_timeout(int cellNo,uint16 t);

//���ÿ�λй���ֵ
	bool set_cell_leak_vacuum_value(int cellNo,float val);

#if 0
//��ȡ��λִ�н��
	bool get_cell_setting_result(int cellNo,plc_cell_setting_t result);
#endif

//��ȡ©�ʲ���Ӧ��
	bool get_leak_ratio_test_ack(int cellNo,bool &ack);
//��ȡ�����Ӧ��
	bool get_pump_vacuum_ack(int cellNo,bool &ack);
//��ȡй���Ӧ��
	bool get_leak_vacuum_ack(int cellNo,bool &ack);
//��ȡ����д��״̬
	bool get_step_para_write_ack(int cellNo,bool &ack);
//���PLC����汾
	bool get_plc_software_version(uint16 &out);

//��ȡ��ѹ���Խ��
	bool get_cell_keep_vacuum_test_result(int cellNo,uint16 &result);
//��ѹ��ʼѹ��ֵ
	bool get_cell_keep_vacuum_start_value(int cellNo,float &val);
//��ѹ���ѹ��ֵ
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
//��ȡϵͳ�쳣�澯
	bool get_system_abnormal_alarm();
//��ȡ����澯
	bool get_cab_smoke_alarm();
//��ȡ��Դ����ȸ澯
	bool get_cab_fan_alarm();
//��ȡ��ͣ�澯
	bool get_sudden_stop_alarm();
//��ȡ��λ����澯
	bool get_cell_smoke_alarm(int cellNo);
//��ȡ��λ�����쳣�澯
	bool get_cell_action_abnormal_alarm(int cellNo);

//��ȡ��λ���ֵ
	bool get_cell_vacuum_value();
//��ȡ��λ�ѿ�״̬
	bool get_cell_pins_repair_button_status();
//��ȡ��λ�����Ƿ���λ״̬
	bool get_cell_tray_in_place_status();
//��ȡ��λ�����Ƿ�������λ״̬
	//bool get_cell_tray_rise_to_place_status();	//�ϲ�
//��ȡ��λ�������Ƿ���״̬
	bool get_cell_fireDoor_is_open_status();
//��ȡ��λ�Ƿ�ѹ��״̬
	bool get_cell_pins_is_press_status();
//��ȡ��λ�Ƿ��ѿ�״̬
	//bool get_cell_pins_is_unpress_status();		//�ϲ�
//��ȡ��λǿ���ѿ�״̬
	bool get_cell_pins_force_unpress_status();

//��ȡ�澯�ϵ���ʱʱ��
	bool get_warn_powerdown_delaytime();
//��ȡ�����ϵ���ʱʱ��
	bool get_emergency_powerdown_delaytime();
// ��ȡ��λ�Ƿ��Զ�ģʽ
	bool get_cell_auto_mode();

public:
	plc_sys_abnormal_alarm_t sys_abnormal_alarm;
	alarm_erc_t cab_smoke_alarm;
	plc_cab_fan_alarm_t cabfan_overload_alarm;				// ��Դ���ŷ��ȹ���
	plc_sudden_stop_alarm_t sudden_stop_alarm;
	plc_cell_smoke_alarm_t cell_smoke_alarm[MAX_CELLS_NBR];
	plc_cell_action_alarm_t cell_action_alarm[MAX_CELLS_NBR];
	
private:
	SLMPDriver m_slmp_dev;         		// ����SLMP
	bool is_plc_connected;

	int warn_poweroff_delaysec;			// �澯�ϵ��ӳ�ʱ��
	int emstop_poweroff_delaysec;		// ��ͣ�ϵ��ӳ�ʱ��
	std::bitset<8> pinsRepairButton;	// ��λά�ް�ť״̬
	std::bitset<8> trayInPlace;			// ������λ״̬
	std::bitset<8> trayRiseToPlace;		// ����������λ״̬
	std::bitset<8> fireDoorOpen; 		// �������Ƿ���
	std::bitset<8> pinsIsPress; 		// �봲ѹ��״̬
	std::bitset<8> pinsIsUnPress; 		// �봲�ѿ�״̬
	std::bitset<8> pinsForceUnPress; 	// �봲ǿ�Ƶ���״̬
	std::bitset<8> autoMode; 			// �Ƿ��Զ�ģʽ

	float vac_value[MAX_CELLS_NBR]; 	// ��λ���ֵ
};
#endif
#endif

