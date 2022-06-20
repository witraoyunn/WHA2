#include <iostream>
#include <sstream>
#include <time.h>
#include <math.h>
#include <iomanip>
#include "log4z.h"
#include "FormatConvert.h"
#include "ChannelProto_JN9504.h"

using namespace std;

FormatConvert& FormatConvert::instance()
{
    static FormatConvert obj;

    return obj;
}

FormatConvert::FormatConvert()
{
	m_line_no = 0;
	m_cabinet_no = 0;
}

FormatConvert::~FormatConvert()
{}


string FormatConvert::timestamp_to_string(long tt)
{
    time_t rawtime = (time_t)(tt/1000);
    struct tm *timeinfo = localtime(&rawtime);

    char time_buffer[64];
    strftime(time_buffer, 64, "%F %T",timeinfo);

	ostringstream os;
	os << time_buffer << ".";
	os << (int)(tt%1000);

    string result = os.str();

	return result;
}

string FormatConvert::alarm_to_string(int value, int cell, int ch, int func_code, int sub_func_code)
{
	ostringstream os;
    os << RESPONSE_EXCEPTION << "{"
            << "\"CabNo\":" << m_cabinet_no <<","
        	<< "\"CellNo\":" << cell << ","
        	<< "\"ChannelNo\":" << ch << ","
            << "\"FuncCode\":" << func_code << ","
            << "\"SubFuncCode\":" << sub_func_code << ","
            << "\"Data\":" << value
       << "}";

    string result = os.str();
    LOGFMTD("Alarm msg: %s", result.c_str());

    return result;
}

string FormatConvert::pins_alarm_to_string(int error, int cell)
{
	ostringstream os;
    os << RESPONSE_CELLALARM << "{"
            << "\"CabNo\":" << m_cabinet_no <<","
        	<< "\"CellNo\":" << cell << ","
            << "\"AlarmCode\":" << error
       << "}";

    string result = os.str();
    LOGFMTD("Pins Alarm msg: %s", result.c_str());

    return result;
}


#if defined(AWH_PRECHARGE_SYS)
string FormatConvert::plc_alarm_to_json(int cell, Fx_Error_Code_t &data, Fx_Error_Code_t &last_data)
{
	ostringstream os;
    os  <<"{"
         << "\"CabNo\":" << m_cabinet_no <<","
    	 << "\"CellNo\":" << cell << ","
         << "\"DataType\":" << "\"Warning\"" << ","
         << "\"TimeStamp\":" << time(NULL) << ","
         << "\"Data\":"
         << "{"; 
		//告警信息
		if ( data.PressureError != last_data.PressureError)
		{
			os << "\"PressureError\":" << data.PressureError << ",";
		}
		if ( data.EStop != last_data.EStop)
		{
			os << "\"EStop\":" << data.EStop << ",";
		}
		if ( data.PLCHeartBeat != last_data.PLCHeartBeat)
		{
			os << "\"PLCHeartBeat\":" << data.PLCHeartBeat << ",";
		}
		if ( data.DoorOpen != last_data.DoorOpen)
		{
			os << "\"DoorOpen\":" << data.DoorOpen << ",";
		}
		if ( data.MoveUpTimeout != last_data.MoveUpTimeout)
		{
			os << "\"MoveUpTimeout\":" << data.MoveUpTimeout << ",";
		}
		if ( data.MoveDownTimeout != last_data.MoveDownTimeout)
		{
			os << "\"MoveDownTimeout\":" << data.MoveDownTimeout << ",";
		}
		if ( data.TrayReverse != last_data.TrayReverse)
		{
			os << "\"TrayReverse\":" << data.TrayReverse << ",";
		}
		if ( data.NoTray != last_data.NoTray)
		{
			os << "\"NoTray\":" << data.NoTray << ",";
		}
		if ( data.TrayNotReady != last_data.TrayNotReady)
		{
			os << "\"TrayNotReady\":" << data.TrayNotReady << ",";
		}
		if ( data.FanOverload != last_data.FanOverload)
		{
			os << "\"FanOverload\":" << data.FanOverload << ",";
		}
		if ( data.RollerOverload != last_data.RollerOverload)
		{
			os << "\"RollerOverload\":" << data.RollerOverload << ",";
		}
		if ( data.RollerRunTimeout != last_data.RollerRunTimeout)
		{
			os << "\"RollerRunTimeout\":" << data.RollerRunTimeout << ",";
		}
		if ( data.RollerReverseRunTimeout != last_data.RollerReverseRunTimeout)
		{
			os << "\"RollerReverseRunTimeout\":" << data.RollerReverseRunTimeout << ",";
		}
		if ( data.LeftFanError != last_data.LeftFanError)
		{
			os << "\"LeftFanError\":" << data.LeftFanError << ",";
		}
		if ( data.RightFanError != last_data.RightFanError)
		{
			os << "\"RightFanError\":" << data.RightFanError << ",";
		}
		if ( data.TopFanError != last_data.TopFanError)
		{
			os << "\"TopFanError\":" << data.TopFanError << ",";
		}
		if ( data.Smoke1Alarm != last_data.Smoke1Alarm)
		{
			os << "\"Smoke1Alarm\":" << data.Smoke1Alarm << ",";
		}
		if ( data.Smoke2Alarm != last_data.Smoke2Alarm)
		{
			os << "\"Smoke2Alarm\":" << data.Smoke2Alarm << ",";
		}
		if ( data.CabSmokeAlarm != last_data.CabSmokeAlarm)
		{
			os << "\"CabSmokeAlarm\":" << data.CabSmokeAlarm << ",";
		}
		if ( data.PinSmokeAlarm != last_data.PinSmokeAlarm)
		{
			os << "\"PinSmokeAlarm\":" << data.PinSmokeAlarm << ",";
		}
		if ( data.PinSmokeHeavyAlarm != last_data.PinSmokeHeavyAlarm)
		{
			os << "\"PinSmokeHeavyAlarm\":" << data.PinSmokeHeavyAlarm << ",";
		}
		if ( data.HighTempAlarm != last_data.HighTempAlarm)
		{
			os << "\"HighTempAlarm\":" << data.HighTempAlarm << ",";
		}
	//清理最后一个字符","
	string result = os.str();
	result.erase(result.size()-1);
	os.str("");
	os << result;
    os << "}"
       << "}";

    result = os.str();
    LOGFMTD("Pins data: %s", result.c_str());
    return result;
}
#endif

string FormatConvert::version_data_to_string(int cell, int func_code, string mid_ver, vector<lower_mc_version_t> &low_ver)
{
	ostringstream valArrary;
	valArrary.setf(std::ios::fixed);

	valArrary << "[";
	for (uint32_t i=0; i<low_ver.size(); ++i)
	{
		if(i != 0)
		{
			valArrary << ",";
		}

		valArrary << "{"
   				  << "\"DeviceNo\":" << low_ver[i].device_no << ","
				  << "\"LowerVersion\":\"" << string(low_ver[i].device_ver) << "\""
				  << "}";
	}
	valArrary << "]";

	ostringstream os;
    os << RESPONSE_VERSION << "{"
            << "\"CabNo\":" << m_cabinet_no << ","
            << "\"CellNo\":" << cell << ","
            << "\"FuncCode\":" << func_code << ","
            << "\"MiddleVersion\":\"" << mid_ver << "\","
            
			<< "\"LowerMachine\":"<< valArrary.str()
       << "}";

    string result = os.str();
    //LOGFMTD("Version data: %s", result.c_str());

    return result;
}

string FormatConvert::reply_to_string(int cell, int ch, int func_code, int sub_func_code, int value)
{
	ostringstream os;
    os << RESPONSE_REPLY << "{"
            << "\"CabNo\":" << m_cabinet_no <<","
        	<< "\"CellNo\":" << cell << ","
        	<< "\"ChannelNo\":" << ch << ","
            << "\"FuncCode\":" << func_code << ","
            << "\"SubFuncCode\":" << sub_func_code << ","
            << "\"Data\":" << value
       << "}";

    string result = os.str();
    LOGFMTD("Reply msg: %s", result.c_str());
    return result;
}

string FormatConvert::reply_to_string(int cell, int ch, int func_code, int sub_func_code, float value)
{
	ostringstream os;
    os << RESPONSE_REPLY << "{"
            << "\"CabNo\":" << m_cabinet_no <<","
            << "\"CellNo\":" << cell << ","
            << "\"ChannelNo\":" << ch << ","
            << "\"FuncCode\":" << func_code << ","
            << "\"SubFuncCode\":" << sub_func_code << ","
            << "\"Data\":" << value
       << "}";

    string result = os.str();
    LOGFMTD("Reply msg: %s", result.c_str());
    return result;
}

string FormatConvert::reply_to_string(int cell, int ch, int func_code, int sub_func_code, vector<float> value)
{
	ostringstream valArrary;
	valArrary << "[";
	for (uint32_t i=0; i<value.size(); ++i)
	{
		valArrary << value[i] << ",";
	}
	valArrary << "]";
	
	ostringstream os;
    os << RESPONSE_REPLY << "{"
            << "\"CabNo\":" << m_cabinet_no <<","
            << "\"CellNo\":" << cell << ","
            << "\"ChannelNo\":" << ch << ","
            << "\"FuncCode\":" << func_code << ","
            << "\"SubFuncCode\":" << sub_func_code << ","
            << "\"Data\":" << valArrary.str()
       << "}";

    string result = os.str();
    LOGFMTD("Reply msg: %s", result.c_str());
    return result;
}

string FormatConvert::reply_to_string(int cell, int ch, int func_code, int sub_func_code, string value)
{
	ostringstream os;
    os << RESPONSE_REPLY << "{"
            << "\"CabNo\":" << m_cabinet_no <<","
            << "\"CellNo\":" << cell << ","
            << "\"ChannelNo\":" << ch << ","
            << "\"FuncCode\":" << func_code << ","
            << "\"SubFuncCode\":" << sub_func_code << ","
            << "\"Message\":\"" << value << "\""
       << "}";

    string result = os.str();
    LOGFMTD("Reply msg: %s", result.c_str());
    return result;
}

string FormatConvert::pinscell_reply_to_string(int cell, string func_code, int status, uint16 data)
{
	ostringstream os;
    os << RESPONSE_CELLREPLY << "{"
            << "\"CabNo\":" << m_cabinet_no <<","
            << "\"CellNo\":" << cell << ","
            << "\"FuncCode\":\"" << func_code << "\","
            << "\"Status\":" << status;
		if(func_code == "register_read" || func_code == "plc_sw_version")
		{
			os << ",\"Data\":" << (int)data;
		}

    os << "}";

    string result = os.str();
    LOGFMTD("PinsCell Reply msg: %s", result.c_str());
    return result;
}



string FormatConvert::record_to_string(Step_Process_Data_t& data)
{
	ostringstream os;
	os.setf(std::ios::fixed);

	string stepTypeName = ChannelProto_JN9504::get_stepname_string(data.step_type);
	
    os << RESPONSE_RECORD << "{"
         << "\"CabNo\":" << m_cabinet_no <<","
    	 << "\"CellNo\":" << data.cell_no << ","
    	 << "\"ChannelNo\":" << data.ch_no << ","
    	 << "\"FuncCode\":" << 0 << ","
    	 << "\"Data\":"
    	 << "{";
	
			if(string(data.stepConf_no) != "")
			{
				os << "\"RecipeName\":" << "\"" << string(data.stepConf_no) << "\",";
			}

			if(string(data.batch_no) != "")
			{
				os << "\"BatchNo\":" << "\"" << string(data.batch_no) << "\",";
			}

			if(string(data.pallet_barcode) != "")
			{
				os << "\"TrayNo\":" << "\"" << string(data.pallet_barcode) << "\",";
			}

          os << "\"RunState\":" << (int)(data.run_state) << ","
        	 << "\"RunTime\":" << (int)(data.run_time) << ",";

	      os.precision(4);
          os << "\"Voltage\":" << data.voltage << ","
        	 << "\"Current\":" << data.current << ","
        	 << "\"Capacity\":" << data.capacity << ",";
		  os.precision(1);

          os << "\"BatteryTemperature\":" << data.batteryTemp << ","
          	 << "\"StepNo\":" << data.step_no << ","
        	 << "\"StepType\":" << "\"" << stepTypeName << "\","
        	 << "\"LoopNo\":" << data.loop_no << ","
        	 //<< "\"CurrentTime\":" << "\"" << timestamp_to_string(data.timestamp) << "\","
        	 << "\"CurrentTime\":" << data.timestamp << ","
        	 << "\"SumStep\":" << data.sum_step << ",";
#ifndef PROTOCOL_5V160A
		  os.precision(2);
          os << "\"Ratio\":" << data.ratio << ",";
#endif
		  os.precision(4);
          os << "\"Energy\":" << data.energy << ","
        	 << "\"Povl\":" << data.pvol << ","
#ifndef PROTOCOL_5V160A
             << "\"T-Sink\":" << data.t_sink << ","
		 	 << "\"CotactVol\":" << data.v_cotact << ","
		 	 << "\"CotactRes\":" << data.r_cotact << ","
		 	 << "\"OutputRes\":" << data.r_output << ","	
		     << "\"LoopTimes\":" << data.loop_times << ","
		     << "\"ErrorCode\":" << data.error_code << ",";
		  os.precision(1);
		  os << "\"Vaccum\":" << data.vac_val
#else
			<< "\"ContactVol\":" << data.v_cotact << ","
			<< "\"ContactRes\":" << data.r_cotact << ","
		 	<< "\"OutputRes\":" << data.r_output << ","	
			<< "\"CC_CV_Flag\":" << (int)data.CC_CV_flag << ","
			<< "\"ErrorCode\":" << (int)data.err_mask << ","
#if defined(AWH_FORMATION_SYS)
			<< "\"Vaccum\":" << data.vaccum << ","
#endif
			<< "\"TotalTime\":" << data.totalTime
#endif
//		  #ifdef CHANNEL_STORE_CELL_TEMPER
//		  os  << "\"Vaccum\":" << data.vac_val << ","
//		  	  << "\"CellT1\":" << data.cell_temper[0] << ","
//		  	  << "\"CellT2\":" << data.cell_temper[1] << ","
//		      << "\"CellT3\":" << data.cell_temper[2] << ","
//		      << "\"CellT4\":" << data.cell_temper[3] << ","
//		      << "\"CellT5\":" << data.cell_temper[4] << ","
//		      << "\"CellT6\":" << data.cell_temper[5]
//		 #else
//		 os  << "\"Vaccum\":" << data.vac_val
//		 #endif
         << "}"
	  << "}";

    string result = os.str();
    LOGFMTD("Record msg: %s", result.c_str());

    return result;
}

std::string FormatConvert::process_data_to_store_str(Step_Process_Data_t& data)
{
    ostringstream os;
	os.setf(std::ios::fixed);

	string stepTypeName = ChannelProto_JN9504::get_stepname_string(data.step_type);
	
    os << "{"
#ifdef ZHONGYUAN_DB
		 << "\"id\":" << "\"" << m_line_no << "-" << m_cabinet_no << "-" << data.cell_no << "-" << data.ch_no << "-"
		 << string(data.bat_barcode) << "-" << data.timestamp << "\","
#endif
#ifndef PROTOCOL_5V160A
		 << "\"BatteryType\":" << "\"" << string(data.bat_type) << "\","
		 << "\"StepConfNo\":" << "\"" << string(data.stepConf_no) << "\","
		 << "\"PalletCode\":" << "\"" << string(data.pallet_barcode) << "\","
		 << "\"BATTERY_NUMBER\":" << "\"" << string(data.bat_barcode) << "\","
#endif		 
         << "\"IsStop\":" << data.is_stop <<","
         << "\"LineNo\":" << m_line_no <<","
         << "\"CabNo\":" << m_cabinet_no <<","
         << "\"CellNo\":" << data.cell_no << ","
         << "\"ChannelNo\":" << data.ch_no << ","
         << "\"FuncCode\":" << 0 << ","
         << "\"StepNo\":" << data.step_no << ","
         //<< "\"StepType\":" << (int)(data.step_type) << ","
         << "\"StepType\":" << "\"" << stepTypeName << "\","
         << "\"RunState\":" << (int)(data.run_state) << ","
		 << "\"RunTime\":" << (int)(data.run_time) << ",";

	  os.precision(4);
      os << "\"Volt\":" << data.voltage << ","
         << "\"Curr\":" << data.current << ","
         << "\"Capacity\":" << data.capacity << ",";
	  os.precision(1);
#ifndef PROTOCOL_5V160A
      os << "\"BatteryTemperature\":" << data.temperature << ",";
#endif
      os << "\"LoopNo\":" << data.loop_no << ","
         << "\"CurrentTime\":" << data.timestamp << ","
         << "\"SumStep\":" << data.sum_step << ",";
#ifndef PROTOCOL_5V160A
	  os.precision(2);
      os << "\"Ratio\":" << data.ratio << ",";
#endif
	  os.precision(4);
      os << "\"Energy\":" << data.energy << ","
         << "\"Povl\":" << data.pvol << ","

#ifndef PROTOCOL_5V160A
		 << "\"CotactRes\":" << data.r_cotact << ","
		 << "\"LoopTimes\":" << data.loop_times << ","
		 << "\"ErrorCode\":" << data.error_code << ","
#endif
	 ;
	 os.precision(1);
     
#ifdef CHANNEL_STORE_CELL_TEMPER
	 os << "\"Vaccum\":" << data.vac_val << ","
	 	<< "\"CellT1\":" << data.cell_temper[0] << ","
	 	<< "\"CellT2\":" << data.cell_temper[1] << ","
	 	<< "\"CellT3\":" << data.cell_temper[2] << ","
	 	<< "\"CellT4\":" << data.cell_temper[3] << ","
	 	<< "\"CellT5\":" << data.cell_temper[4] << ","
	 	<< "\"CellT6\":" << data.cell_temper[5]
#else
	  
#ifndef PROTOCOL_5V160A
	 os << "\"Vaccum\":" << data.vac_val
#else
	 os << "\"CC_CV_Flag\":" << (int)data.CC_CV_flag << ","
		<< "\"err_mask\":" << (int)data.err_mask
#endif
	 #endif
      << "}";

    string result = os.str();
//    LOGFMTD("d2k: %s", result.c_str());
    return result;
}


std::string FormatConvert::process_data_to_store_str_new(Step_Process_Data_t& data)
{
    ostringstream os;
	os.setf(std::ios::fixed);

	string stepTypeName = ChannelProto_JN9504::get_stepname_string(data.step_type);
	
    os << "{"
		   << "\"currentTime\":" << data.timestamp << ","
		   << "\"lineNo\":" << m_line_no << ","
		   << "\"cabNo\":" << m_cabinet_no <<","
		   << "\"cellNo\":" << data.cell_no << ","
		   << "\"channelNo\":" << data.ch_no << ","
		   << "\"stepNo\":" << data.step_no << ","
		   << "\"loopNo\":" << data.loop_no << ","
		   << "\"sumStep\":" << data.sum_step << ","
		   << "\"runState\":" << (int)(data.run_state) << ","
		   << "\"runTime\":" << (int)(data.run_time) << ","
		   << "\"totalTime\":" << data.totalTime << ","
		   << "\"stepType\":" << "\"" << stepTypeName << "\",";

		   if(string(data.bat_barcode) != "")
		   {
				os << "\"batteryNo\":" << "\"" << string(data.bat_barcode) << "\",";
		   }
		   
		   if(string(data.bat_type) != "")
		   {
				os << "\"batteryType\":" << "\"" << string(data.bat_type) << "\",";
		   }

		   if(string(data.pallet_barcode) != "")
		   {
				os << "\"trayNo\":" << "\"" << string(data.pallet_barcode) << "\",";
		   }

		   if(string(data.batch_no) != "")
		   {
				os << "\"batchNo\":" << "\"" << string(data.batch_no) << "\",";
		   }

		   if(string(data.stepConf_no) != "")
		   {
			   	os << "\"recipeName\":" << "\"" << string(data.stepConf_no) << "\",";
		   }
		   
		os.precision(4);
		os << "\"voltage\":" << data.voltage << ","
			<< "\"current\":" << data.current << ","
			<< "\"capacity\":" << data.capacity << ","
			<< "\"energy\":" << data.energy << ","
			<< "\"povl\":" << data.pvol << ","
			<< "\"contactVol\":" << data.v_cotact << ","
			<< "\"contactRes\":" << data.r_cotact << ","
		 	<< "\"outputRes\":" << data.r_output << ",";
		os.precision(1);
		os << "\"batteryTemp\":" << data.batteryTemp << ","
#if defined(AWH_FORMATION_SYS)
			<< "\"vacuum\":" << data.vaccum << ","
#endif
			<< "\"cellT1\":" << data.cellTemp[0] << ","
			<< "\"cellT2\":" << data.cellTemp[1] << ","
			<< "\"cellT3\":" << data.cellTemp[2] << ","
			<< "\"cellT4\":" << data.cellTemp[3] << ","
			<< "\"cellT5\":" << data.cellTemp[4] << ","
			<< "\"cellT6\":" << data.cellTemp[5] << ","
#if defined(AWH_PRECHARGE_SYS)
			<< "\"cellT7\":" << data.cellTemp[6] << ","
#elif defined(AWH_GRADING_SYS)
			<< "\"cellT7\":" << data.cellTemp[6] << ","
			<< "\"cellT8\":" << data.cellTemp[7] << ","
			<< "\"cellT9\":" << data.cellTemp[8] << ","
#endif
			<< "\"cvFlag\":" << data.CC_CV_flag << ",";
			if(data.end_type > 0)
			{
				os << "\"actionInfo\":" << "\"" <<	"01" << setw(2) << setfill('0') << to_string(data.end_type) << "\",";
			}
			
			if(data.run_result > 0)
			{	
				int res;
				if(data.run_result == 1)	//OK
				{
					res = 1;
					os << "\"result\":" << res << ",";
				}
				else						//NG
				{
					res = 0;
					os << "\"result\":" << res << ","
					   << "\"ngReason\":" << data.ng_reason << ",";
				}
			}

			if(data.cc_capacity > 0.0 || data.cv_capacity > 0.0)
			{
				os.precision(4);
				os << "\"cc_capacity\":" << data.cc_capacity << ","
				   << "\"cc_second\":" << data.cc_second << ","
				   << "\"cv_capacity\":" << data.cv_capacity << ","
				   << "\"cv_second\":" << data.cv_second << ",";
			}
			os << "\"stepStop\":" << data.is_stop 
	<< "}";

    string result = os.str();
    //LOGFMTD("d2k: %s", result.c_str());
    return result;
}


#ifdef PROTOCOL_5V160A
std::string FormatConvert::process_data_to_sql_str(Step_Process_Data_t& data, const char *tableName)
{
	ostringstream os;
	os.setf(std::ios::fixed);
	
	os << "INSERT INTO " << string(tableName) 
		 << "(ID,BatteryType,StepConfNo,PalletCode,BATTERY_NUMBER,"
		 << "IsStop,LineNo,CabNo,CellNo,ChannelNo,"
		 << "StepNo,StepType,RunState,RunTime,Volt,Curr,Capacity,BatteryTemperature,"
		 << "LoopNo,CurrentTime,SumStep,Ratio,Energy,Povl,CotactRes,LoopTimes,ErrorCode,Vaccum)"
		 << " VALUES("
		 << "'" << m_line_no << "-" << m_cabinet_no << "-" << data.cell_no << "-" << data.ch_no << "-"

		 << "" << "-" << data.timestamp << "',"
		 << "'" << "" << "',"
		 << "'" << "" << "',"
		 << "'" << "" << "',"
		 << "'" << "" << "',"

		 << data.is_stop << ","
		 << m_line_no << ","
		 << m_cabinet_no << ","
		 << data.cell_no << ","
		 << data.ch_no << ","
		 << data.step_no << ","
		 << (int)(data.step_type) << ","
		 << (int)(data.run_state) << ","
		 << (int)(data.run_time * 1000.0) << ",";

	  os.precision(4);
	  os << data.voltage << ","
		 << data.current << ","
		 << data.capacity << ",";
	  os.precision(1);
	  os << "" << ",";
	  os << data.loop_no << ","
		 << data.timestamp << ","
		 << data.sum_step << ",";
	  os.precision(2);
	  os << "" << ",";
	  os.precision(4);
	  os << data.energy << ","
		 << data.pvol << ","
//		 << data.r_cotact << ","

//		 << data.loop_times << ","
//		 << data.error_code << ","
//		 ;
//	  os.precision(1);
//	  os << data.vac_val
	  << ")";

	string result = os.str();
	return result;
}
#else
std::string FormatConvert::process_data_to_sql_str(Step_Process_Data_t& data, const char *tableName)
{
	ostringstream os;
	os.setf(std::ios::fixed);
	
	os << "INSERT INTO " << string(tableName) 
		 << "(ID,BatteryType,StepConfNo,PalletCode,BATTERY_NUMBER,"
		 << "IsStop,LineNo,CabNo,CellNo,ChannelNo,"
		 << "StepNo,StepType,RunState,RunTime,Volt,Curr,Capacity,BatteryTemperature,"
		 << "LoopNo,CurrentTime,SumStep,Ratio,Energy,Povl,CotactRes,LoopTimes,ErrorCode,Vaccum)"
		 << " VALUES("
		 << "'" << m_line_no << "-" << m_cabinet_no << "-" << data.cell_no << "-" << data.ch_no << "-"
		 	<< string(data.bat_barcode) << "-" << data.timestamp << "',"
		 << "'" << string(data.bat_type) << "',"
		 << "'" << string(data.stepConf_no) << "',"
		 << "'" << string(data.pallet_barcode) << "',"
		 << "'" << string(data.bat_barcode) << "',"

		 << data.is_stop << ","
		 << m_line_no << ","
		 << m_cabinet_no << ","
		 << data.cell_no << ","
		 << data.ch_no << ","
		 << data.step_no << ","
		 << (int)(data.step_type) << ","
		 << (int)(data.run_state) << ","
		 << (int)(data.run_time) << ",";

	  os.precision(4);
	  os << data.voltage << ","
		 << data.current << ","
		 << data.capacity << ",";
	  os.precision(1);
	  os << data.temperature << ",";
	  os << data.loop_no << ","
		 << data.timestamp << ","
		 << data.sum_step << ",";
	  os.precision(2);
	  os << data.ratio << ",";
	  os.precision(4);
	  os << data.energy << ","
		 << data.pvol << ","
		 << data.r_cotact << ","

		 << data.loop_times << ","
		 << data.error_code << ",";
	  os.precision(1);
	  os << data.vac_val
	  << ")";

	string result = os.str();
	return result;
}
#endif

#if defined(AWH_PRECHARGE_SYS)
string FormatConvert::pins_data_to_json(int cell, Pins_Status_Data_t &data)
{
	ostringstream os;
	os.setf(std::ios::fixed);
    os <<"{"
         << "\"LineNo\":" << m_line_no <<","
    	 << "\"CabNo\":" << m_cabinet_no <<","
    	 << "\"CellNo\":" << cell << ","
    	 << "\"DataType\":" << "\"Status\"" <<","
    	 << "\"TimeStamp\":" << time(NULL) <<","
         << "\"Data\":"
         << "{"
		 << "\"TrayIsExtend\":" << data.is_extend << ","
		 << "\"TrayIsRetract\":" << data.is_retract << ","
		 << "\"ToolPowerIsOn\":" << data.is_tool_power << ","
		 << "\"CoolFanIsRun\":" << data.is_cool_fan << ","
		 << "\"AirFanIsRun\":" << data.is_air_fan << ","
		 << "\"IsAutoMode\":" << data.is_auto_mode << ","
		 << "\"IsLoaded\":" << data.is_loaded << ","
		 << "\"TrayIsReady\":" << data.is_tray_ready << ","
		 << "\"IsAutoStart\":" << data.is_auto_start << ",";
		os.precision(1);
		  for(int i=0 ; i < CELL_TEMP_ACQ_NUM ; i++)
		  {
          		os << "\"Temp"<< i+1 << "\":" << data.temp[i] << ",";
		  }

		  os << "\"TempWarning\":" << data.temp_warning << ",";
	      os << "\"CanInTray\":" << data.is_can_intray << ","
		 << "\"TrayInReady\":" << data.is_tray_inready << ","
		 << "\"IsPLCconnect\":" << data.is_plc_connect << ","
		 << "\"TrayOutReady\":" << data.is_tray_outready
	      << "}"
       << "}";

    string result = os.str();
    //LOGFMTD("Pins data: %s", result.c_str());
    return result;
}
#else
string FormatConvert::pins_data_to_json(int cell, Pins_Status_Data_t &data)
{
	ostringstream os;
	os.setf(std::ios::fixed);

    os << RESPONSE_CELLDATA <<"{"
         << "\"CabNo\":" << m_cabinet_no <<","
    	 << "\"CellNo\":" << cell << ","
         << "\"FuncCode\":" << CELL_PINS_STATUS << ","
         << "\"SubFuncCode\":" << -1 << ","
         << "\"Data\":"
         << "{";
		  os.precision(1);
		  for(int i=0 ; i < CELL_TEMP_ACQ_NUM ; i++)
		  {
          		os << "\"Temp"<< i+1 << "\":" << data.temp[i] << ",";
		  }
		  os.precision(4);
          os << "\"Vacuum\":" << data.vacuum << ","
		     << "\"WarnPowerOffDelay\":" << data.warn_poweroff_delay << ","
		     << "\"EmStopPowerOffDelay\":" << data.emstop_poweroff_delay << ","
#if defined(AWH_FORMATION_SYS)
			 << "\"BlockTestResult\":" << data.blockTestResult << ","
			 << "\"KeepVacStart\":" << data.keepVac_start << ","
			 << "\"KeepVacEnd\":" << data.keepVac_end << ","
		  	 << "\"LeakRatioResult\":" << data.leakRatioResult << ","
		  	 << "\"KeepVacTestResult\":" << data.keepVacTestResult << ","
#endif
			 << "\"PinsPress\":" << data.pins_extend << ","
			 << "\"PinsUnPress\":" << data.pins_unpress << ","
			 << "\"TrayInPlace\":" << data.tray_in_place << ","
			 << "\"IsFireDoorOpen\":" << data.fire_door_open << ","
			 << "\"IsAutoMode\":" << data.is_auto_mode << ","
			 << "\"IsPLCconnect\":" << data.is_plc_connect << ","
			 << "\"IsForbidden\":" << data.is_cell_forbid
    	 << "}"
       << "}";

    string result = os.str();
    //LOGFMTD("Pins data: %s", result.c_str());
    return result;
}
#endif

string FormatConvert::pins_vac_to_json(Pins_Vac_ReportForms_t data)
{
	ostringstream valArrary;
	valArrary.precision(4);
	valArrary.setf(std::ios::fixed);
	
	valArrary << "[";
	for (uint8_t i=0; i<sizeof(data.vacs)/sizeof(data.vacs[0]); ++i)
	{
		valArrary << data.vacs[i] << ",";
	}
	valArrary << "]";
	
    char tm_buf[64];
    strftime(tm_buf, 64, "%F %T", &data.dt);
	
	ostringstream os;
    os <<"{"
         << "\"LineNo\":" << m_line_no <<","
         << "\"CabNo\":" << m_cabinet_no <<","
    	 << "\"CellNo\":" << data.cell << ","
    	 << "\"date_time\":" << tm_buf << ","
         << "vac_list:" << valArrary.str()
       << "}";

    string result = os.str();
    LOGFMTD("Pins data: %s", result.c_str());
    return result;
}

string FormatConvert::notify_to_string(int cell, int func_code, int sub_code, char *batch_no, int value,vector<int> &errCode)
{
	ostringstream valArrary;
	
	valArrary << "[";
	for (uint32_t i=0; i<errCode.size(); ++i)
	{
		if(i != 0)
		{
			valArrary << ",";
		}
		valArrary << errCode[i];
	}
	valArrary << "]";

	ostringstream os;
    os << RESPONSE_NOTIFY << "{"
            << "\"CabNo\":" << m_cabinet_no <<","
        	<< "\"CellNo\":" << cell << ","
        	<< "\"ChannelNo\":" << -1 << ","
            << "\"FuncCode\":" << func_code << ","
            << "\"SubFuncCode\":" << sub_code << ",";

			if(string(batch_no) != "")
			{
				os << "\"BatchNo\":" << "\"" << string(batch_no) << "\",";
			}

			if(value >= 11 && errCode.size() > 0)
			{
				os << "\"FireAlarmcode\":" << valArrary.str()<< ",";
			}
			
	os << "\"Data\":" << value
       << "}";

    string result = os.str();
    LOGFMTD("Notify: %s", result.c_str());

	return result;
}

string FormatConvert::linkstate_to_string(int cell, int func_code, vector<int> chan, string ip, int state)
{
	ostringstream valArrary;
	
	valArrary << "[";
	for (uint32_t i=0; i<chan.size(); ++i)
	{
		if(i != 0)
		{
			valArrary << ",";
		}
		valArrary << chan[i];
	}
	valArrary << "]";

	ostringstream os;
    os << RESPONSE_CHAN_LINK << "{"
            << "\"CabNo\":" << m_cabinet_no <<","
        	<< "\"CellNo\":" << cell << ","
			<< "\"Channels\":"<< valArrary.str()<<","
            << "\"FuncCode\":" << func_code << ","
            << "\"Data\":" 
    	 	<< "{"
        		<< "\"LinkState\":" << state << ","
				<< "\"IP\":" << "\"" << ip << "\""
			<< "}"
       << "}";

    string result = os.str();
    LOGFMTD("Channel_Link: %s", result.c_str());

    return result;
}


#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS)) 	//化成柜和定容柜才有电表
string FormatConvert::cab_meter_to_string(vector<meter_data_t> &mtr_data)
{
	ostringstream valArrary;
	valArrary.setf(std::ios::fixed);

	valArrary << "[";
	for (uint32_t i=0; i<mtr_data.size(); ++i)
	{
		if(i != 0)
		{
			valArrary << ",";
		}

		valArrary << "{"
   				  << "\"MeterNo\":" << i+1 << ","
				  << "\"Time\":" << mtr_data[i].timestamp << ",";
   		#if 0	
		valArrary.precision(1);
		valArrary << "\"UA\":" << mtr_data[i].wU[0] << ","
 				<< "\"UB\":" << mtr_data[i].wU[1] << ","
 				<< "\"UC\":" << mtr_data[i].wU[2] << ",";
		valArrary.precision(3);
 		valArrary << "\"IA\":" << mtr_data[i].wI[0] << ","
 				<< "\"IB\":" << mtr_data[i].wI[1] << ","
 				<< "\"IC\":" << mtr_data[i].wI[2] << ",";
 		valArrary.precision(1);		
 		valArrary << "\"PA\":" << mtr_data[i].wP[0] << ","
 				<< "\"PB\":" << mtr_data[i].wP[1] << ","
 				<< "\"PC\":" << mtr_data[i].wP[2] << ",";
		#endif
		valArrary.precision(2);	
   		valArrary << "\"ForwardEnergy\":" << mtr_data[i].forward_energy << ","
   				  << "\"ReverseEnergy\":" << mtr_data[i].reverse_energy
				  << "}";
	}
	valArrary << "]";

	ostringstream os;
    os << RESPONSE_CAB_METER << "{"
            << "\"CabNo\":" << m_cabinet_no << ","
			<< "\"MeterItems\":"<< valArrary.str()
       << "}";

    string result = os.str();
    //LOGFMTD("Cab Meter: %s", result.c_str());

    return result;
}
#endif

string FormatConvert::transmit_reply_to_string(int cell, int device, string func_code, vector<int> &value)
{
	ostringstream valArrary;
	
	valArrary << "[";
	for (uint32_t i=0; i<value.size(); ++i)
	{
		if(i != 0)
		{
			valArrary << ",";
		}
		valArrary << value[i];
	}
	valArrary << "]";
	
	ostringstream os;
    os << RESPONSE_TRANSMIT_REPLY << "{"
            << "\"CabNo\":" << m_cabinet_no <<","
            << "\"CellNo\":" << cell << ","
            << "\"DeviceNo\":" << device << ","
            << "\"FuncCode\":" << "\"" << func_code << "\","
            << "\"Data\":" << valArrary.str()
       << "}";

    string result = os.str();
    LOGFMTD("Transmit Reply msg: %s", result.c_str());
    return result;
}



