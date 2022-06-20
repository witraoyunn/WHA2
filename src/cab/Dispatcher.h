/**************************************************************
* Copyright (C) 2019, 上海鼎格信息科技有限公司
* All rights reserved.
* 产品号   :
* 所属组件 :
* 模块名称 :
* 文件名称 : Dispatcher.h
* 概要描述 :
* 历史记录 :
* 版本      日期          作者           内容
* 1.0       20190416     zhangw
***************************************************************/
#ifndef DISPATCHER_H_
#define DISPATCHER_H_

#include <map>
#include "zmq.hpp"
#include "json/json.h"
#include "Type.h"
#include "MsgFactory.h"
#include "XmlProcAPI.h"

class Dispatcher
{
public:
	~Dispatcher();
    static Dispatcher* instance();
	
	void publish(std::string json_msg);
	//void publish_bts(std::string json_msg);
	void globalProcInfo_resolve(global_protect_t &globalInfo,Channel_Data_t &pow_data);
	void workstepInfo_resolve(work_step_recipe_t &stepInfo,Channel_Data_t &pow_data);
	void m_send_func_struct(int cell_no,int func_code);

private:
    Dispatcher();
    Dispatcher(const Dispatcher&) {}
	
	void str_funccode_dispatch(Json::Value &data);
#ifndef PROTOCOL_5V160A
	int  m_procet_factor(uint8 subfunc, double value);
	int  m_cal_factor(uint8 subfunc, double value);
#endif
	JN9504_Step_Type_e get_stepname_value(std::string stepName);
	void do_save_globalProtect(Json::Value &data);
	void m_parse_globalProc_JN95(Json::Value &data);
	void m_parse_process_cmd_JN95(Json::Value &data);
	void m_parse_process_cmd(Json::Value  &data);

    void m_send_struct(Channel_Data_t &data_union);
    void m_send_struct(Cmd_Pins_Data_t &data_union);
#if defined(AWH_PRECHARGE_SYS)
	void m_send_struct_PLC(Cmd_Pins_Data_t &data_union , int func_code);
#endif
	void m_send_struct(Sub_Cmd_Msg_t &data_union);
#ifdef AUX_VOLTAGE_ACQ
	void m_send_struct(Cmd_Aux_Param_t &data);
#endif
	void m_send_pins(int sub_code, int val);
	
private:
	uint8 m_sel_chans[MAX_CHANNELS_NBR];
	int	m_cell_no;
	int	m_func_code;
};

#endif //DISPATCHER_H_


