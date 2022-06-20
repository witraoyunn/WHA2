#ifndef FORMATCONVERT_H_
#define FORMATCONVERT_H_

#include <string>
#include <vector>
#include "Type.h"
#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
#include "CabinetMeter.h"
#endif

class FormatConvert
{
public:
    static FormatConvert& instance();

    void set_line_no(int no) { m_line_no = no; }
    void set_cabinet_no(int no) { m_cabinet_no = no; }

	std::string timestamp_to_string(long tt);
#if defined(AWH_PRECHARGE_SYS)
	std::string plc_alarm_to_json(int cell, Fx_Error_Code_t &data, Fx_Error_Code_t &last_data);
#endif
    std::string alarm_to_string(int value, int cell = -1, int ch = -1, int func_code = -1, int sub_func_code = -1);
	std::string pins_alarm_to_string(int error, int cell);
	std::string version_data_to_string(int cell, int func_code, std::string mid_ver, std::vector<lower_mc_version_t> &low_ver);
    std::string reply_to_string(int cell, int ch, int func_code, int sub_func_code, int value);
    std::string reply_to_string(int cell, int ch, int func_code, int sub_func_code, float value);
	std::string reply_to_string(int cell, int ch, int func_code, int sub_func_code, std::vector<float> value);
    std::string reply_to_string(int cell, int ch, int func_code, int sub_func_code, std::string value);
	std::string pinscell_reply_to_string(int cell, std::string func_code, int status, uint16 data);
	std::string transmit_reply_to_string(int cell, int device, std::string func_code, std::vector<int> &value);

	std::string record_to_string(Step_Process_Data_t& data);
    std::string process_data_to_store_str(Step_Process_Data_t& data);
	std::string process_data_to_store_str_new(Step_Process_Data_t& data);
	std::string process_data_to_sql_str(Step_Process_Data_t& data, const char *tableName);

	// 老版本，针床消息
    std::string pins_data_to_json(int cell, Pins_Status_Data_t &data);
#if defined(AWH_PRECHARGE_SYS)	
	std::string pins_data_to_store_str(int cell, Pins_Status_Data_t &data);
#endif
	std::string pins_vac_to_json(Pins_Vac_ReportForms_t data);
	//电源柜电表消息
#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
	std::string cab_meter_to_string(std::vector<meter_data_t> &mtr_data);
#endif
	// 化成分容相关的通知消息
	std::string notify_to_string(int cell, int func_code, int sub_code, char *batch_no, int value, std::vector<int> &errCode);
	std::string linkstate_to_string(int cell, int func_code, std::vector<int> chan, std::string ip, int state);
	
private:
    FormatConvert();
    FormatConvert(const FormatConvert &);
    FormatConvert &operator=(const FormatConvert &);
    ~FormatConvert();

    int m_line_no;
    int m_cabinet_no;
};

#endif // FORMATCONVERT_H_

