#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
#ifndef _CABINETMETER_H_
#define _CABINETMETER_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <vector>
#include <thread>

#include "modbus.h"


#define METER_CNT			2
#define METER_NO_1			1
#define METER_NO_2			2

#define IrAt_ADDR			0x0006
#define UrAt_ADDR			0x0007

#define FORWARD_ENERGY_ADDR	0x101E
#define REVERSE_ENERGY_ADDR	0x1028

#define PHASE_UA_ADDR		0x2006
#define PHASE_UB_ADDR		0x2008
#define PHASE_UC_ADDR		0x200A

#define CURRENT_IA_ADDR		0x200C
#define CURRENT_IB_ADDR		0x200E
#define CURRENT_IC_ADDR		0x2010

#define POWER_A_ADDR		0x2014
#define POWER_B_ADDR		0x2016
#define POWER_C_ADDR		0x2018

//#pragma pack(2)
typedef struct
{
	uint16_t IrAt;
	uint16_t UrAt;
}meter_UI_attr_t;
//#pragma pack()

typedef struct
{
	long timestamp;			// 时间戳
	float forward_energy;	//正向有功功率
	float reverse_energy;	//反向有功功率
#if 0
	float wU[3];			//三相电压
	float wI[3];			//三相电流
	float wP[3];			//三相功率
#endif	
}meter_data_t;

typedef struct
{
	int addr;
	int len;
	float *out;
}meter_item_t;

class CabinetMeter
{
public:
	CabinetMeter();
	~CabinetMeter();

	std::thread run();
private:
	void work();
	
	void serial_port_detect();									//多串口检测
	void read_meter_item();										//抄读电表
	void convert_meter_data(int meterNo,meter_data_t &val);		//获取电表数据

	void set_meter_item(int meterNo);								//配置电表抄表项
	void add_item(int addr,int len,float *out);
	bool read_register(int addr, uint16_t *out, int len);
	bool read_register(int addr,float *out,int len);

private:
	modbus_t *m_rtu_master;
	meter_data_t m_mtr_data[METER_CNT];
	std::vector<meter_item_t> m_mtr_items;

	int m_meterNo;
	int m_port_No;
	unsigned int itm_idx;
	unsigned int itm_read_mask[METER_CNT];
	meter_UI_attr_t u_i_ratio[METER_CNT];			//电压电流互感器倍率
	
	bool m_finish_flag[METER_CNT];
	bool m_valid_flag;
};
#endif
#endif
