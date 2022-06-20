#include <math.h>
#include <string>
#include "MsgFactory.h"
#include "FormatConvert.h"

#include "StepException.h"
#include "ChannelProto_JN9504.h"

StepException::StepException()
{
	m_ERC_Code = 0;
}

StepException::~StepException()
{

}

void StepException::init_task(int bERC)
{
	if(bERC)
	{
		m_ERC_Code = bERC;
	}
}

int StepException::return_erc_code()
{
	return m_ERC_Code;
}

void StepException::exception_alarm_send(int cellNo,int chanNo)
{
	int item_code = -2100;
	
	item_code -= m_ERC_Code;
	std::string josn_str = FormatConvert::instance().alarm_to_string(item_code, cellNo, chanNo, 0x88, m_ERC_Code);
	MsgFactory::instance()->cell_alarm_pusher(cellNo).send(josn_str);	
}

TimingVoltageUpper::TimingVoltageUpper()
{
	timetick = 0;
}

TimingVoltageUpper::~TimingVoltageUpper()
{

}

bool TimingVoltageUpper::run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if(rec.proceData.run_state == STATE_RUNNING)
	{
		if(rec.proceData.step_no == last_rec.proceData.step_no)
		{
			if(++timetick >= pro.TimeVoltageCheckTime)
			{
				if(rec.proceData.voltage > pro.UpperTimeVoltage)
				{
					return true;
				}
			}
		}
		else
		{
			timetick = 0;
		}
	}

	return false;
}

TimingVoltageLower::TimingVoltageLower()
{
	timetick = 0;
}

TimingVoltageLower::~TimingVoltageLower()
{

}

bool TimingVoltageLower::run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if(rec.proceData.run_state == STATE_RUNNING)
	{
		if(rec.proceData.step_no == last_rec.proceData.step_no)
		{
			if(++timetick >= pro.TimeVoltageCheckTime)
			{
				if(rec.proceData.voltage < pro.LowerTimeVoltage)
				{
					return true;
				}
			}
		}
		else
		{
			timetick = 0;
		}
	}

	return false;
}


TimingVoltageUpperDetect::TimingVoltageUpperDetect()
{
	timetick = 0;
	timeVoltage1 = 0.0;
	timeVoltage2 = 0.0;	
}

TimingVoltageUpperDetect::~TimingVoltageUpperDetect()
{

}

bool TimingVoltageUpperDetect::run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if(rec.proceData.run_state == STATE_RUNNING)
	{
		if(rec.proceData.step_no == last_rec.proceData.step_no)
		{
			timetick++;
			if(timetick >= pro.RangeVoltageCheckTime1)
			{
				if(timeVoltage1 == 0.0)
				{
					timeVoltage1 = rec.proceData.voltage;
				}
			}

			if(timetick >= pro.RangeVoltageCheckTime2)
			{
				if(timeVoltage2 == 0.0)
				{
					timeVoltage2 = rec.proceData.voltage;
				}
			}
			
			if(timeVoltage1 > 0.0 && timeVoltage2 > 0.0)
			{
				if(fabs(timeVoltage1 - timeVoltage2) > pro.UpperDeltaRangeVoltage)
				{
					return true;
				}
			}
		}
		else
		{
			timetick = 0;
			timeVoltage1 = 0.0;
			timeVoltage2 = 0.0;
		}
	}

	return false;
}


TimingVoltageLowerDetect::TimingVoltageLowerDetect()
{
	timetick = 0;
	timeVoltage1 = 0.0;
	timeVoltage2 = 0.0;	
}

TimingVoltageLowerDetect::~TimingVoltageLowerDetect()
{

}

bool TimingVoltageLowerDetect::run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if(rec.proceData.run_state == STATE_RUNNING)
	{
		if(rec.proceData.step_no == last_rec.proceData.step_no)
		{
			timetick++;
			if(timetick >= pro.RangeVoltageCheckTime1)
			{
				if(timeVoltage1 == 0.0)
				{
					timeVoltage1 = rec.proceData.voltage;
				}
			}

			if(timetick >= pro.RangeVoltageCheckTime2)
			{
				if(timeVoltage2 == 0.0)
				{
					timeVoltage2 = rec.proceData.voltage;
				}
			}
			
			if(timeVoltage1 > 0.0 && timeVoltage2 > 0.0)
			{
				if(fabs(timeVoltage1 - timeVoltage2) < pro.LowerDeltaRangeVoltage)
				{
					return true;
				}
			}
		}
		else
		{
			timetick = 0;
			timeVoltage1 = 0.0;
			timeVoltage2 = 0.0;
		}
	}

	return false;
}

CycleVoltageUpperDetect::CycleVoltageUpperDetect()
{
	timetick = 0;
	cycleVoltage = 0.0;
	last_cycleVoltage = 0.0;
}

CycleVoltageUpperDetect::~CycleVoltageUpperDetect()
{

}

bool CycleVoltageUpperDetect::run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if(rec.proceData.run_state == STATE_RUNNING)
	{
		if(rec.proceData.step_no == last_rec.proceData.step_no)
		{
			timetick++;
			if((timetick % pro.VoltageCheckCycleTime) == 0)
			{
				last_cycleVoltage = cycleVoltage;
				cycleVoltage = rec.proceData.voltage;

				if(cycleVoltage > 0.0 && last_cycleVoltage > 0.0)
				{
					if(fabs(cycleVoltage - last_cycleVoltage) > pro.UpperDeltaCycleVoltage)
					{
						return true;
					}
				}
			}
		}
		else
		{
			timetick = 0;
			cycleVoltage = 0.0;
			last_cycleVoltage = 0.0;
		}
	}

	return false;
}


CycleVoltageLowerDetect::CycleVoltageLowerDetect()
{
	timetick = 0;
	cycleVoltage = 0.0;
	last_cycleVoltage = 0.0;
}

CycleVoltageLowerDetect::~CycleVoltageLowerDetect()
{

}

bool CycleVoltageLowerDetect::run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if(rec.proceData.run_state == STATE_RUNNING)
	{
		if(rec.proceData.step_no == last_rec.proceData.step_no)
		{
			timetick++;
			if((timetick % pro.VoltageCheckCycleTime) == 0)
			{
				last_cycleVoltage = cycleVoltage;
				cycleVoltage = rec.proceData.voltage;

				if(cycleVoltage > 0.0 && last_cycleVoltage > 0.0)
				{
					if(fabs(cycleVoltage - last_cycleVoltage) < pro.LowerDeltaCycleVoltage)
					{
						return true;
					}
				}
			}
		}
		else
		{
			timetick = 0;
			cycleVoltage = 0.0;
			last_cycleVoltage = 0.0;
		}
	}

	return false;
}


VoltagePosJump::VoltagePosJump()
{
	happenCount = 0;
}

VoltagePosJump::~VoltagePosJump()
{

}

bool VoltagePosJump::run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if(rec.proceData.run_state == STATE_RUNNING)
	{
		if(rec.proceData.step_no == last_rec.proceData.step_no)
		{
			if((last_rec.proceData.CC_CV_flag == CONSTANT_CURRENT_FLG) && (rec.proceData.CC_CV_flag == CONSTANT_CURRENT_FLG))
			{
				if(rec.proceData.voltage > last_rec.proceData.voltage)
				{
					if((rec.proceData.voltage - last_rec.proceData.voltage) > pro.PositiveDeltaVoltage)
					{
						happenCount++;
					}

					if(happenCount >= pro.PositiveDeltaVoltageCount)
					{
						return true;
					}
				}
			}
		}
		else
		{
			happenCount = 0;
		}
	}

	return false;
}

VoltageNegJump::VoltageNegJump()
{
	happenCount = 0;
}

VoltageNegJump::~VoltageNegJump()
{

}

bool VoltageNegJump::run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if(rec.proceData.run_state == STATE_RUNNING)
	{
		if(rec.proceData.step_no == last_rec.proceData.step_no)
		{
			if(rec.proceData.voltage < last_rec.proceData.voltage)
			{
				if((last_rec.proceData.voltage - rec.proceData.voltage) > pro.NegativeDeltaVoltage)
				{
					happenCount++;
				}

				if(happenCount >= pro.NegativeDeltaVoltageCount)
				{
					return true;
				}
			}
		}
		else
		{
			happenCount = 0;
		}
	}

	return false;
}

CrossStepVoltageUpper::CrossStepVoltageUpper()
{

}

CrossStepVoltageUpper::~CrossStepVoltageUpper()
{

}

bool CrossStepVoltageUpper::run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if(rec.proceData.run_state > STATE_NO_RUN)
	{
		if((rec.proceData.step_no != last_rec.proceData.step_no) && (last_rec.proceData.step_no > 0))
		{
			if(fabs(rec.proceData.voltage-last_rec.proceData.voltage) > pro.CrossStepPositiveDeltaVoltage)
			{
				return true;
			}
		}
	}

	return false;
}

CrossStepVoltageLower::CrossStepVoltageLower()
{

}

CrossStepVoltageLower::~CrossStepVoltageLower()
{

}

bool CrossStepVoltageLower::run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if(rec.proceData.run_state > STATE_NO_RUN)
	{
		if((rec.proceData.step_no != last_rec.proceData.step_no) && (last_rec.proceData.step_no > 0))
		{
			if(fabs(rec.proceData.voltage-last_rec.proceData.voltage) < pro.CrossStepNegativeDeltaVoltage)
			{
				return true;
			}
		}
	}

	return false;
}

CurrentPosJump::CurrentPosJump()
{
	happenCount = 0;
}

CurrentPosJump::~CurrentPosJump()
{

}

bool CurrentPosJump::run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if(rec.proceData.run_state == STATE_RUNNING)
	{
		if(rec.proceData.step_no == last_rec.proceData.step_no)
		{
			if((last_rec.proceData.CC_CV_flag == CONSTANT_VOLTAGE_FLG) && (rec.proceData.CC_CV_flag == CONSTANT_VOLTAGE_FLG))
			{
				if(fabs(rec.proceData.current) > fabs(last_rec.proceData.current))
				{
					if((fabs(rec.proceData.current) - fabs(last_rec.proceData.current)) > pro.PositiveDeltaCurrent)
					{
						happenCount++;
					}
				
					if(happenCount >= pro.PositiveDeltaCurrentCount)
					{
						return true;
					}
				}
			}
		}
		else
		{
			happenCount = 0;
		}
	}

	return false;
}

CurrentNegJump::CurrentNegJump()
{
	happenCount = 0;
}

CurrentNegJump::~CurrentNegJump()
{

}

bool CurrentNegJump::run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if(rec.proceData.run_state == STATE_RUNNING)
	{
		if(rec.proceData.step_no == last_rec.proceData.step_no)
		{
			if((last_rec.proceData.CC_CV_flag == CONSTANT_VOLTAGE_FLG) && (rec.proceData.CC_CV_flag == CONSTANT_VOLTAGE_FLG))
			{
				if(fabs(rec.proceData.current) < fabs(last_rec.proceData.current))
				{
					if((fabs(last_rec.proceData.current) - fabs(rec.proceData.current)) > pro.NegativeDeltaCurrent)
					{
						happenCount++;
					}
				
					if(happenCount >= pro.NegativeDeltaCurrentCount)
					{
						return true;
					}
				}
			}
		}
		else
		{
			happenCount = 0;
		}
	}

	return false;
}

bool StepCellTempUpper::run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if((rec.proceData.run_state != STATE_NO_RUN) && (rec.proceData.run_state != STATE_START))
	{
		if((rec.tempData.batteryTemp > pro.BatteryUpperTemp) && (rec.tempData.batteryTemp > -50.0))
		{
			return true;
		}
	}

	return false;
}

bool StepCellTempLower::run_task(step_protect_t &pro,Channel_Record_Data_t &rec,Channel_Record_Data_t &last_rec)
{
	if((rec.proceData.run_state != STATE_NO_RUN) && (rec.proceData.run_state != STATE_START))
	{
		if((rec.tempData.batteryTemp < pro.BatteryLowerTemp) && (rec.tempData.batteryTemp > -50.0))
		{
			return true;
		}
	}

	return false;
}

bool StepEndVoltageLower::run_task(step_protect_t & pro,Channel_Record_Data_t & rec,Channel_Record_Data_t & last_rec)
{
	if(rec.proceData.step_no > 0)
	{
	#if defined(AWH_FORMATION_SYS)		 //化成 有等待状态
		if(rec.proceData.run_state == STATE_WAITING)		
	#else								//预充 分容 无等待状态
		if(rec.proceData.step_no != last_rec.proceData.step_no)
	#endif
		{
			if(last_rec.proceData.run_state == STATE_RUNNING)
			{
				if(last_rec.proceData.voltage < pro.LowerStepStopVoltage)
				{
					return true;
				}
			}
		}
	}
	
	return false;
}


bool StepEndCapacityLower::run_task(step_protect_t & pro,Channel_Record_Data_t & rec,Channel_Record_Data_t & last_rec)
{
	if(rec.proceData.step_no > 0)
	{	
	#if defined(AWH_FORMATION_SYS)		 //化成 有等待状态
		if(rec.proceData.run_state == STATE_WAITING)		
	#else								//预充 分容 无等待状态
		if(rec.proceData.step_no != last_rec.proceData.step_no)
	#endif
		{
			if(last_rec.proceData.run_state == STATE_RUNNING)
			{
				if(last_rec.proceData.capacity < pro.LowerStepStopCapacity)
				{
					return true;
				}
			}
		}
	}
	
	return false;
}


bool StepEndTimeLower::run_task(step_protect_t & pro,Channel_Record_Data_t & rec,Channel_Record_Data_t & last_rec)
{
	if(rec.proceData.step_no > 0)
	{
	#if defined(AWH_FORMATION_SYS)		 //化成 有等待状态
		if(rec.proceData.run_state == STATE_WAITING)		
	#else								//预充 分容 无等待状态
		if(rec.proceData.step_no != last_rec.proceData.step_no)
	#endif
		{
			if(last_rec.proceData.run_state == STATE_RUNNING)
			{
				if((int)last_rec.proceData.run_time < pro.LowerStepStopTime)
				{
					return true;
				}
			}
		}
	}
	
	return false;
}


