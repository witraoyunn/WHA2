#ifndef _STEPEXCEPTION_H_
#define _STEPEXCEPTION_H_

#include "XmlProcAPI.h"
#include "ChannelState.h"

class StepException
{
public:
	StepException();
	virtual ~StepException();
	
	void init_task(int bERC);
	int return_erc_code();
	void exception_alarm_send(int cellNo,int chanNo);
	virtual bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec){return false;}
	//virtual bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec) = 0;
private:
	int m_ERC_Code;
};

//定时电压超上限
class TimingVoltageUpper : public StepException
{
public:
	TimingVoltageUpper();
	~TimingVoltageUpper();
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int timetick;
};

//定时电压超下限
class TimingVoltageLower : public StepException
{
public:
	TimingVoltageLower();
	~TimingVoltageLower();
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int timetick;

};

//定时电压变化量上限
class TimingVoltageUpperDetect : public StepException
{
public:
	TimingVoltageUpperDetect();
	~TimingVoltageUpperDetect();
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int timetick;
	float timeVoltage1;
	float timeVoltage2;
};

//定时电压变化量下限
class TimingVoltageLowerDetect : public StepException
{
public:
	TimingVoltageLowerDetect();
	~TimingVoltageLowerDetect();
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int timetick;
	float timeVoltage1;
	float timeVoltage2;
};


//周期电压变化量上限
class CycleVoltageUpperDetect : public StepException
{
public:
	CycleVoltageUpperDetect();
	~CycleVoltageUpperDetect();
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int timetick;
	float cycleVoltage;
	float last_cycleVoltage;
};

//周期电压变化量下限
class CycleVoltageLowerDetect : public StepException
{
public:
	CycleVoltageLowerDetect();
	~CycleVoltageLowerDetect();
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int timetick;
	float cycleVoltage;
	float last_cycleVoltage;
};


//电压正跳变
class VoltagePosJump : public StepException
{
public:
	VoltagePosJump();
	~VoltagePosJump();
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int happenCount;
};

//电压负跳变
class VoltageNegJump : public StepException
{
public:
	VoltageNegJump();
	~VoltageNegJump();
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int happenCount;
};

//跨工步正间隔电压变化
class CrossStepVoltageUpper : public StepException
{
public:
	CrossStepVoltageUpper();
	~CrossStepVoltageUpper();
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//跨工步负间隔电压变化
class CrossStepVoltageLower : public StepException
{
public:
	CrossStepVoltageLower();
	~CrossStepVoltageLower();
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//电流正跳变
class CurrentPosJump : public StepException
{
public:
	CurrentPosJump();
	~CurrentPosJump();
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int happenCount;
};

//电流负跳变
class CurrentNegJump : public StepException
{
public:
	CurrentNegJump();
	~CurrentNegJump();
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int happenCount;
};

//电池温度过温保护
class StepCellTempUpper : public StepException
{
public:
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//电池温度下限保护
class StepCellTempLower : public StepException
{
public:
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//工步结束电压超下限
class StepEndVoltageLower : public StepException
{
public:
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//工步结束容量超下限
class StepEndCapacityLower : public StepException
{
public:
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//工步结束时间超下限
class StepEndTimeLower : public StepException
{
public:
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

#endif

