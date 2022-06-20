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

//��ѹ��������
class VoltageFluc : public GlobalException
{
public:
	VoltageFluc();
	~VoltageFluc();
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int flucCount;
};

//������ѹ�쳣����
class AuxtVoltageErr : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//������������
class CurrntFluc : public GlobalException
{
public:
	CurrntFluc();
	~CurrntFluc();
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int flucCount;
};

//�ܵ����������
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


//�����ܵ����������
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

//�����ܵ����������
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


//�Ӵ��迹����
class ContactResExceed : public GlobalException
{
public:
	ContactResExceed();
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	bool resis_calc_flg;
};

//��·�迹����
class LoopResExceed : public GlobalException
{
public:
	LoopResExceed();
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	bool resis_calc_flg;
};

#if defined(AWH_FORMATION_SYS)	
//��ѹ��������
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

//����¶����ޱ���
class BatteryTempLowerLimit : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//����¶ȹ��±���
class BatteryTempOverLimit : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//����¶ȳ��±���
class BatteryTempUltra : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//����¶ȳ����±���
class BatteryTempUltraOver : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//����¶�С����
class BatteryTempSmallFluc : public GlobalException
{
public:
	BatteryTempSmallFluc();
	~BatteryTempSmallFluc();
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	int flucCount;
};

//����¶ȴ󲨶�
class BatteryTempLargeFluc : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};


//����¶�ͻ��
class BatteryTempSuddenRise : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//��λ�¶����ޱ���
class CellTempLowerLimit : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//��λ�¶ȳ��±���
class CellTempOver : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//��λ�¶ȳ����±���
class CellTempUltraOver : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//��λ�¶ȼ����
class CellTempContrast : public GlobalException
{
public:
	CellTempContrast();
	~CellTempContrast();
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
private:
	float max_contrast;
};


//ͨ���¶Ȳɼ��쳣
class ChanTempAcqBoxDetect : public GlobalException
{
public:
	bool run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec);
};

//��λ�¶Ȳɼ��쳣
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
