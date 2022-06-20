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

//��ʱ��ѹ������
class TimingVoltageUpper : public StepException
{
public:
	TimingVoltageUpper();
	~TimingVoltageUpper();
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int timetick;
};

//��ʱ��ѹ������
class TimingVoltageLower : public StepException
{
public:
	TimingVoltageLower();
	~TimingVoltageLower();
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int timetick;

};

//��ʱ��ѹ�仯������
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

//��ʱ��ѹ�仯������
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


//���ڵ�ѹ�仯������
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

//���ڵ�ѹ�仯������
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


//��ѹ������
class VoltagePosJump : public StepException
{
public:
	VoltagePosJump();
	~VoltagePosJump();
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int happenCount;
};

//��ѹ������
class VoltageNegJump : public StepException
{
public:
	VoltageNegJump();
	~VoltageNegJump();
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int happenCount;
};

//�繤���������ѹ�仯
class CrossStepVoltageUpper : public StepException
{
public:
	CrossStepVoltageUpper();
	~CrossStepVoltageUpper();
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//�繤���������ѹ�仯
class CrossStepVoltageLower : public StepException
{
public:
	CrossStepVoltageLower();
	~CrossStepVoltageLower();
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//����������
class CurrentPosJump : public StepException
{
public:
	CurrentPosJump();
	~CurrentPosJump();
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int happenCount;
};

//����������
class CurrentNegJump : public StepException
{
public:
	CurrentNegJump();
	~CurrentNegJump();
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int happenCount;
};

//����¶ȹ��±���
class StepCellTempUpper : public StepException
{
public:
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//����¶����ޱ���
class StepCellTempLower : public StepException
{
public:
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//����������ѹ������
class StepEndVoltageLower : public StepException
{
public:
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//������������������
class StepEndCapacityLower : public StepException
{
public:
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//��������ʱ�䳬����
class StepEndTimeLower : public StepException
{
public:
	bool run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

#endif

