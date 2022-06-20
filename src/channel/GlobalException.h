#ifndef _GLOBALEXCEPTION_H_
#define _GLOBALEXCEPTION_H_

#include "XmlProcAPI.h"
#include "ChannelState.h"


class GlobalException
{
public:
	GlobalException();
	virtual ~GlobalException();
	
	void init_task(int bERC);
	int return_erc_code();
	void exception_alarm_send(int cellNo,int chanNo);
	virtual bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec){return false;}
	//virtual bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec) = 0;
private:
	int m_ERC_Code;
};

//电压波动保护
class VoltageFluc : public GlobalException
{
public:
	VoltageFluc();
	~VoltageFluc();
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int flucCount;
};

//辅助电压异常保护
class AuxtVoltageErr : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//电流波动保护
class CurrntFluc : public GlobalException
{
public:
	CurrntFluc();
	~CurrntFluc();
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int flucCount;
};

//总电荷容量保护
class CapacityExceed : public GlobalException
{
public:
	CapacityExceed();
	~CapacityExceed();
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	float negCapacity;
	float posCapacity;
};


//结束总电荷容量下限
class EndCapacityLower : public GlobalException
{
public:
	EndCapacityLower();
	~EndCapacityLower();
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	float negCapacity;
	float posCapacity;
	float tmpCapacity;
};

//结束总电荷容量上限
class EndCapacityUpper : public GlobalException
{
public:
	EndCapacityUpper();
	~EndCapacityUpper();
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	float negCapacity;
	float posCapacity;
	float tmpCapacity;
};


//接触阻抗保护
class ContactResExceed : public GlobalException
{
public:
	ContactResExceed();
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	bool resis_calc_flg;
};

//回路阻抗保护
class LoopResExceed : public GlobalException
{
public:
	LoopResExceed();
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	bool resis_calc_flg;
};

#if defined(AWH_FORMATION_SYS)	
//负压波动保护
class VacuumFluc : public GlobalException
{
public:
	VacuumFluc();
	~VacuumFluc();
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	float vac_base;
	int timetick;
	int fluc_limit;
};
#endif

//电池温度下限保护
class BatteryTempLowerLimit : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//电池温度过温保护
class BatteryTempOverLimit : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//电池温度超温保护
class BatteryTempUltra : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//电池温度超高温保护
class BatteryTempUltraOver : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//电池温度小波动
class BatteryTempSmallFluc : public GlobalException
{
public:
	BatteryTempSmallFluc();
	~BatteryTempSmallFluc();
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int flucCount;
};

//电池温度大波动
class BatteryTempLargeFluc : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};


//电池温度突升
class BatteryTempSuddenRise : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//库位温度下限保护
class CellTempLowerLimit : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//库位温度超温保护
class CellTempOver : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//库位温度超高温保护
class CellTempUltraOver : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//库位温度极差保护
class CellTempContrast : public GlobalException
{
public:
	CellTempContrast();
	~CellTempContrast();
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	float max_contrast;
};


//通道温度采集异常
class ChanTempAcqBoxDetect : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//库位温度采集异常
class CellTempAcqBoxDetect : public GlobalException
{
public:
	CellTempAcqBoxDetect();
	~CellTempAcqBoxDetect();
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	float range_upper;
	float range_lower;
};


#endif
