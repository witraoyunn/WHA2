#include <math.h>
#include <string>
#include "MsgFactory.h"
#include "FormatConvert.h"
#include "Configuration.h"

#include "GlobalException.h"
#include "ChannelProto_JN9504.h"


GlobalException::GlobalException()
{
	m_ERC_Code = 0;
}

GlobalException::~GlobalException()
{

}

void GlobalException::init_task(int bERC)
{
	if(bERC)
	{
		m_ERC_Code = bERC;
	}
}

int GlobalException::return_erc_code()
{
	return m_ERC_Code;
}

void GlobalException::exception_alarm_send(int cellNo,int chanNo)
{
	int item_code = -2000;
	
	item_code -= m_ERC_Code;
	std::string josn_str = FormatConvert::instance().alarm_to_string(item_code, cellNo, chanNo, 0x83, m_ERC_Code);
	MsgFactory::instance()->cell_alarm_pusher(cellNo).send(josn_str);	
}

VoltageFluc::VoltageFluc()
{
	flucCount = 0;
}

VoltageFluc::~VoltageFluc()
{

}

bool VoltageFluc::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if((rec.proceData.run_state > STATE_NO_RUN) && (rec.proceData.run_state < STATE_START) )
	{
		if(rec.proceData.step_no == last_rec.proceData.step_no)
		{
			if(fabs(rec.proceData.voltage - last_rec.proceData.voltage) > pro.VoltageFluctuateRange)
			{
				flucCount++;
			}

			if(flucCount >= pro.VoltageFluctuateCount)
			{
				return true;
			}
		}
		else
		{
			flucCount = 0;
		}
	}

	return false;
}

bool AuxtVoltageErr::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if(rec.proceData.voltage > 0.0)
	{
		 if(rec.proceData.v_cotact > pro.AuxVoltage)
		 {
			 return true;
		 }
	}

	return false;
}

CurrntFluc::CurrntFluc()
{
	flucCount = 0;
}

CurrntFluc::~CurrntFluc()
{

}

bool CurrntFluc::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if(rec.proceData.run_state == STATE_RUNNING)
	{
		if(rec.proceData.step_no == last_rec.proceData.step_no)
		{
			if((last_rec.proceData.CC_CV_flag == CONSTANT_CURRENT_FLG) && (rec.proceData.CC_CV_flag == CONSTANT_CURRENT_FLG))
			{
				if(fabs(rec.proceData.current - last_rec.proceData.current) > pro.CurrentFluctuateRange)
				{
					flucCount++;
				}
				
				if(flucCount >= pro.CurrentFluctuateCount)
				{
					return true;
				}
			}
		}
		else
		{
			flucCount = 0;
		}
	}

	return  false;
}

CapacityExceed::CapacityExceed()
{
	negCapacity = 0.0;
	posCapacity = 0.0;
}

CapacityExceed::~CapacityExceed()
{

}

bool CapacityExceed::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	float tmpCapacity = 0.0;

#if defined(AWH_FORMATION_SYS)
	if((rec.proceData.run_state == STATE_RUNNING) || (rec.proceData.run_state == STATE_WAITING))
#else
	if((rec.proceData.run_state == STATE_RUNNING) || (rec.proceData.run_state == STATE_FINISH))
#endif
	{
	#if defined(AWH_FORMATION_SYS)		 //化成 有等待状态
		if(rec.proceData.run_state == STATE_WAITING)		
	#else								//预充 分容 无等待状态
		if(rec.proceData.step_no != last_rec.proceData.step_no)
	#endif
		{
			if(last_rec.proceData.run_state == STATE_RUNNING)
			{
				if(last_rec.proceData.current < 0.0)
				{
					negCapacity -= last_rec.proceData.capacity;
				}
				else
				{
					posCapacity += last_rec.proceData.capacity;
				}	
			}
		}
		else
		{
			if(rec.proceData.current < 0.0)
			{
				tmpCapacity = -rec.proceData.capacity;
			}
			else
			{
				tmpCapacity = rec.proceData.capacity;
			}
		}

		if(pro.TotalCapacityProtect < 0)
		{
			if((posCapacity + negCapacity + tmpCapacity) < pro.TotalCapacityProtect)
			{
				return true;
			}	
		}
		else
		{
			if((posCapacity + negCapacity + tmpCapacity) > pro.TotalCapacityProtect)
			{
				return true;
			}	
		}
	}
	
	return false;
}


EndCapacityLower::EndCapacityLower()
{
	negCapacity = 0.0;
	posCapacity = 0.0;
	tmpCapacity = 0.0;
}

EndCapacityLower::~EndCapacityLower()
{

}

bool EndCapacityLower::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
#if defined(AWH_FORMATION_SYS)
	if((rec.proceData.run_state == STATE_RUNNING) || (rec.proceData.run_state == STATE_WAITING))
#else
	if((rec.proceData.run_state == STATE_RUNNING) || (rec.proceData.run_state == STATE_FINISH))
#endif
	{					
	#if defined(AWH_FORMATION_SYS)		 //化成 有等待状态
		if(rec.proceData.run_state == STATE_WAITING)		
	#else								//预充 分容 无等待状态
		if(rec.proceData.step_no != last_rec.proceData.step_no)
	#endif
		{
			if(last_rec.proceData.run_state == STATE_RUNNING)
			{
				if(last_rec.proceData.current < 0.0)
				{
					negCapacity -= last_rec.proceData.capacity;
				}
				else
				{
					posCapacity += last_rec.proceData.capacity;
				}

				tmpCapacity = 0.0;
			}
		}
		else
		{
			if(rec.proceData.current < 0.0)
			{
				tmpCapacity = -rec.proceData.capacity;
			}
			else
			{
				tmpCapacity = rec.proceData.capacity;
			}
		}
	}


#if defined(AWH_FORMATION_SYS)		 //化成 有等待状态
	if((rec.proceData.run_state == STATE_FINISH && last_rec.proceData.run_state == STATE_WAITING) || 
		(rec.proceData.run_state == STATE_NO_RUN && last_rec.proceData.run_state == STATE_RUNNING))
#else								//预充 分容 无等待状态
	if((rec.proceData.run_state == STATE_FINISH && rec.proceData.step_no != last_rec.proceData.step_no) || 
		(rec.proceData.run_state == STATE_NO_RUN && last_rec.proceData.run_state == STATE_RUNNING))
#endif
	{
		if(pro.EndTotalCapacityLower < 0)
		{
			if((posCapacity + negCapacity + tmpCapacity) > pro.EndTotalCapacityLower)
			{
				return true;
			}	
		}
		else
		{
			if((posCapacity + negCapacity + tmpCapacity) < pro.EndTotalCapacityLower)
			{
				return true;
			}	
		}
	}

	return false;
}


EndCapacityUpper::EndCapacityUpper()
{
	negCapacity = 0.0;
	posCapacity = 0.0;
	tmpCapacity = 0.0;
}

EndCapacityUpper::~EndCapacityUpper()
{

}

bool EndCapacityUpper::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
#if defined(AWH_FORMATION_SYS)
	if((rec.proceData.run_state == STATE_RUNNING) || (rec.proceData.run_state == STATE_WAITING))
#else
	if((rec.proceData.run_state == STATE_RUNNING) || (rec.proceData.run_state == STATE_FINISH))
#endif
	{					
	#if defined(AWH_FORMATION_SYS)		 //化成 有等待状态
		if(rec.proceData.run_state == STATE_WAITING)		
	#else								//预充 分容 无等待状态
		if(rec.proceData.step_no != last_rec.proceData.step_no)
	#endif
		{
			if(last_rec.proceData.run_state == STATE_RUNNING)
			{
				if(last_rec.proceData.current < 0.0)
				{
					negCapacity -= last_rec.proceData.capacity;
				}
				else
				{
					posCapacity += last_rec.proceData.capacity;
				}

				tmpCapacity = 0.0;
			}
		}
		else
		{
			if(rec.proceData.current < 0.0)
			{
				tmpCapacity = -rec.proceData.capacity;
			}
			else
			{
				tmpCapacity = rec.proceData.capacity;
			}
		}
	}

#if defined(AWH_FORMATION_SYS)		 //化成 有等待状态
	if((rec.proceData.run_state == STATE_FINISH && last_rec.proceData.run_state == STATE_WAITING) || 
		(rec.proceData.run_state == STATE_NO_RUN && last_rec.proceData.run_state == STATE_RUNNING))
#else								//预充 分容 无等待状态
	if((rec.proceData.run_state == STATE_FINISH && rec.proceData.step_no != last_rec.proceData.step_no) || 
		(rec.proceData.run_state == STATE_NO_RUN && last_rec.proceData.run_state == STATE_RUNNING))
#endif	
	{
		if(pro.EndTotalCapacityUpper < 0)
		{
			if((posCapacity + negCapacity + tmpCapacity) < pro.EndTotalCapacityUpper)
			{
				return true;
			}	
		}
		else
		{
			if((posCapacity + negCapacity + tmpCapacity) > pro.EndTotalCapacityUpper)
			{
				return true;
			}	
		}
	}

	return false;
}


ContactResExceed::ContactResExceed()
{
	resis_calc_flg = false;
}

bool ContactResExceed::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if(rec.proceData.run_state == STATE_RUNNING)
	{
		if(fabs(rec.proceData.current) >= 10.0)	//电流大于等于10A才计算阻抗
		{
			resis_calc_flg = true;
		}

		if(fabs(rec.proceData.current) < 9.0)	// 1A的回滞
		{
			resis_calc_flg = false;
		}

		if(resis_calc_flg)
		{
			if(rec.proceData.r_cotact > pro.ContactResistance)
			{
				return true;
			}
		}
	}
	
	return false;
}

LoopResExceed::LoopResExceed()
{
	resis_calc_flg = false;
}

bool LoopResExceed::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if(rec.proceData.run_state == STATE_RUNNING)
	{
		if(fabs(rec.proceData.current) >= 10.0)	//电流大于等于10A才计算阻抗
		{
			resis_calc_flg = true;
		}

		if(fabs(rec.proceData.current) < 9.0)	// 1A的回滞
		{
			resis_calc_flg = false;
		}

		if(resis_calc_flg)
		{
			if(rec.proceData.r_output > pro.LoopResistance)
			{
				return true;
			}
		}
	}

	return false;
}


#if defined(AWH_FORMATION_SYS)	
VacuumFluc::VacuumFluc()
{
	vac_base = 0.0;
	timetick = 0;
	fluc_limit = Configuration::instance()->vacuum_fluc_time_config();
	if(fluc_limit <= 0)
	{
		fluc_limit = 60;
	}
}

VacuumFluc::~VacuumFluc()
{

}

bool VacuumFluc::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	bool is_vaccum_fluc = false;
	
	if(rec.proceData.run_state == STATE_RUNNING)
	{
		if(last_rec.proceData.run_state == STATE_RUNNING)
		{
			if(rec.proceData.step_no == last_rec.proceData.step_no)
			{
				if(rec.proceData.run_time == 10)
				{
					vac_base = rec.proceData.vac_set;
					timetick = 0;
				}
				else if(rec.proceData.run_time > 10)
				{
					if((rec.proceData.vaccum < (vac_base - pro.VacuumFluctuate)) ||
						(rec.proceData.vaccum > (vac_base + pro.VacuumFluctuate)))
					{
						if(timetick == 0)
						{
							timetick = 1;			//第1次触发波动限值开始计时
						}
						is_vaccum_fluc = true;
					}
					else
					{
						timetick = 0;
					}

					if(timetick > 0)
					{
						if((timetick++ > fluc_limit) && is_vaccum_fluc)
						{
							return true;
						}
					}
				}
			}
			else
			{
				timetick = 0;
			}
		}
		else
		{
			timetick = 0;
		}
	}

	return false;
}
#endif

bool BatteryTempLowerLimit::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if((rec.proceData.run_state > STATE_NO_RUN) && (rec.proceData.run_state != STATE_START))
	{
		if((rec.tempData.batteryTemp < pro.BatteryLowerTemp) && (rec.tempData.batteryTemp > -50.0))
		{
			return true;
		}
	}
	
	return false;
}

bool BatteryTempOverLimit::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if(rec.tempData.batteryTemp > pro.BatteryOverTemp)
	{
		return true;
	}
	
	return false;
}

bool BatteryTempUltra::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if(rec.tempData.batteryTemp > pro.BatteryUltraTemp)
	{
		return true;
	}

	return false;
}

bool BatteryTempUltraOver::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if(rec.tempData.batteryTemp > pro.BatteryUltraHighTemp)
	{
		return true;
	}

	return false;
}

BatteryTempSmallFluc::BatteryTempSmallFluc()
{
	flucCount = 0;
}

BatteryTempSmallFluc::~BatteryTempSmallFluc()
{

}

bool BatteryTempSmallFluc::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if((rec.proceData.run_state > STATE_NO_RUN) && (rec.proceData.run_state != STATE_START))
	{		
		if(fabs(rec.tempData.batteryTemp - last_rec.tempData.batteryTemp) > pro.BatteryTempSmallFluctuateRange)
		{
			flucCount++;
		}

		if(flucCount >= pro.BatteryTempSmallFluctuateCount)
		{
			flucCount = 0;
			return true;
		}
	}

	return false;
}

bool BatteryTempLargeFluc::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if((rec.proceData.run_state > STATE_NO_RUN) && (rec.proceData.run_state != STATE_START))
	{
		if(fabs(rec.tempData.batteryTemp - last_rec.tempData.batteryTemp) > pro.BatteryTempLargeFluctuate)
		{
			return true;
		}
	}

	return false;
}

bool BatteryTempSuddenRise::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if((rec.proceData.run_state > STATE_NO_RUN) && (rec.proceData.run_state != STATE_START))
	{
		if((rec.tempData.batteryTemp - last_rec.tempData.batteryTemp) > pro.BatteryTempSuddenRise)
		{
			return true;
		}
	}

	return false;
}

bool CellTempLowerLimit::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if((rec.proceData.run_state > STATE_NO_RUN) && (rec.proceData.run_state != STATE_START))
	{
		for(int i = 0; i < CELL_TEMP_ACQ_NUM; i++)
		{
			if((rec.tempData.cellTemp[i] < pro.CellLowerTemp) && (rec.tempData.cellTemp[i] > -50.0))
			{
				return true;
			}
		}
	}

	return false;
}

bool CellTempOver::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	for(int i = 0; i < CELL_TEMP_ACQ_NUM; i++)
	{
		if((rec.tempData.cellTemp[i] > pro.CellUltraTemp) && (rec.tempData.cellTemp[i] > -50.0))
		{
			return true;
		}
	}
	
	return false;
}

bool CellTempUltraOver::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	for(int i = 0; i < CELL_TEMP_ACQ_NUM; i++)
	{
		if((rec.tempData.cellTemp[i] > pro.CellUltraHighTemp) && (rec.tempData.cellTemp[i] > -50.0))
		{
			return true;
		}
	}

	return false;
}

CellTempContrast::CellTempContrast()
{
	Configuration::instance()->get_cellTemp_contrast(max_contrast);
	if(max_contrast < 0.0)
	{
		max_contrast = fabs(max_contrast);
	}
}

CellTempContrast::~CellTempContrast()
{

}

bool CellTempContrast::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	float temp_min = 1000.0;
	float temp_max = -90.0;
	uint8_t invalid_count = 0;
	
	for(int i = 0; i < CELL_TEMP_ACQ_NUM; i++)
	{
		if(rec.tempData.cellTemp[i] < -50.0)
		{
			invalid_count++;
			continue;
		}
		
		if(rec.tempData.cellTemp[i] < temp_min)
		{
			temp_min = rec.tempData.cellTemp[i];
		}

		if(rec.tempData.cellTemp[i] >= temp_max)
		{
			temp_max = rec.tempData.cellTemp[i];
		}
	}
	
	if((invalid_count <= (CELL_TEMP_ACQ_NUM-2)) && (fabs(temp_max - temp_min) > max_contrast))
	{
		return true;
	}

	return false;
}


bool ChanTempAcqBoxDetect::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if(rec.tempData.batteryTemp < -50.0)
	{
		return true;
	}
	
	return false;	
}

CellTempAcqBoxDetect::CellTempAcqBoxDetect()
{
	Configuration::instance()->get_cellTemp_range(range_upper,range_lower);
}

CellTempAcqBoxDetect::~CellTempAcqBoxDetect()
{

}

bool CellTempAcqBoxDetect::run_task(global_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	for(int i = 0; i < CELL_TEMP_ACQ_NUM; i++)
	{
		if((rec.tempData.cellTemp[i] < range_lower) || (rec.tempData.cellTemp[i] > range_upper))
		{
			return true;
		}
	}

	return false;
}


