#ifndef PINSGROUP_H_
#define PINSGROUP_H_

#include <string>
#include <bitset>
#include <vector>
#include "Type.h"
#if defined(AWH_PRECHARGE_SYS)
#include "SLMPDriver.h"
#include "Channel.h"
#include "ChannelServer.h"
#include "AuxTempAcq.h"
#endif

class PinsGroup
{
public:
    PinsGroup(){}
    PinsGroup(int group_no);
    PinsGroup& operator= (const PinsGroup& obj);
    ~PinsGroup();

    static bool query_heartbeat_signal();
    static std::bitset<8> query_cabinet_signal();
#if defined(PLC_JYT)
	static void query_power_cabinet_temp(float temp[16]);
	static void query_global_alarm(uint16_t err[2]);
#elif defined(PLC_JS_CT)
	static void query_power_cabinet_temp(float temp[4]);
#endif

#ifdef PLC_JYT
	void set_action_bits(int bit_no, bool val);
	bool get_status_bits(int bit_no);
	void set_plc_init(bool flag);
	void set_device_type(int type);
	int  get_step_no();
	void enable_vac_leak(bool en);
	void fire_protect_A(bool on);
	void fire_protect_B(bool on);
#endif
	void set_chargeDischarge_status(bool flag);

    void power_ON();
    void power_OFF();
    void pins_extend();
    void pins_retract();
    void open_high_vac();
    void close_high_vac();
    void open_low_vac();
    void close_low_vac();
    void open_minus_vac();
    void close_minus_vac();
    void start_vac_seq();
    void stop_vac_seq();
	void set_vac_value(float value);
    void set_vac_params(float *vac_param);
	void set_constant_temp(float temp, float tempDiv);
	void set_fireProtection_temp(float temp);
    void clear_alarm();
#ifdef PLC_JS_VF
	void set_work_mode(bool auto_mode);
	void fire_fan_on_off(bool on);
	void fire_dor_on_off(bool on);
    void clear_smoke_alarm();
	void set_hvac_work_mode(uint16_t mode);
	void set_lvac_work_mode(uint16_t mode);
#endif

#if (defined(PLC_JYT) || defined(PLC_JS_VF) || defined(PLC_JS_CT))
	Pins_Status_Data_t get_env_datas();
#endif
    bool is_auto_mode();
	bool is_maint_mode();
    bool is_cell_empty();
    bool is_pins_extend();
    bool is_pins_retract();
    bool is_dor_open();
	bool is_keep_vac();

#ifdef PLC_JYT
	void get_error_code(uint16_t err[5]);
#else
	int  get_error_code();
#endif

#ifdef PLC_JS_VF
    void get_temperature(float temp[2]);
#elif defined(PLC_JYT)
    void get_temperature(float temp[8]);
#else
    void get_temperature(float temp[7]);
#endif
    float get_vac_value();
	bool  get_vac_status();

    void  start_1230_sys();
    bool  state_1230_sys();
    void  start_1230_fans();

private:
    uint32 m_group;
	
#ifdef PLC_JYT
	//#pragma pack(1)
	struct Readable_Reg_t
	{
		uint16_t actWord;
		uint32_t lampWord;
		uint16_t step;
		int16_t  vacPV;
		uint16_t errA;
		uint16_t errB1;
		uint16_t errB2;
		uint16_t errC1;
		uint16_t errC2;
		uint16_t pins_temp[8];				// 针床库温
	};
	
	Readable_Reg_t m_read_reg;				// 连续的可读点位
#endif
};

#endif //PINSGROUP_H_

