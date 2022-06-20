/**************************************************************
* Copyright (C) 2018, 上海鼎格信息科技有限公司
* All rights reserved.
* 产品号 :
* 所属组件 :GN
* 模块名称 :MsgFactory
* 文件名称 :MsgFactory.h
* 概要描述 :
* 版本      日期         作者    内容
* V1.0    2018-05-18     zhujs     …
***************************************************************/
#include <string>
#include "log4z.h"
#include "GNMessager.h"
#include "MsgFactory.h"

using namespace std;

MsgFactory* MsgFactory::instance()
{
    static MsgFactory obj;
    return &obj;
}

MsgFactory::MsgFactory()
{
    m_ext_suber = 0;
    m_cab_puber = 0;
    m_cab_alarm_pusher = 0;
    for (int i = 0; i < MAX_CELLS_NBR; i++)
    {
        m_cell_suber[i] = 0;
        m_ext_reply_pusher[i] = 0;
        m_cell_alarm_pusher[i] = 0;
    }
}

MsgFactory::~MsgFactory()
{}

//void MsgFactory::create_bts_suber(string address, string filter)
//{
//    m_bts_suber = new GNMessager<int>(ZMQ_SUB);
//    m_bts_suber->connect(address);
//    m_bts_suber->set_recv_filter(filter);
//}
//
//void MsgFactory::destory_bts_suber()
//{
//    if(m_bts_suber)
//    {
//        delete m_bts_suber;
//    }
//}
//
//void MsgFactory::create_bts_pusher(string address)
//{
//    m_bts_pusher = new GNMessager<int>(ZMQ_PUSH);
//    m_bts_pusher->connect(address);
//}
//
//void MsgFactory::destory_bts_pusher()
//{
//    if(m_bts_pusher)
//    {
//        delete m_bts_pusher;
//    }
//}

void MsgFactory::create_cab_ext_suber(string address, string filter)
{
    m_ext_suber = new GNMessager<int>(ZMQ_SUB);
    m_ext_suber->connect(address);
    m_ext_suber->set_recv_filter(filter);
}

void MsgFactory::destory_cab_ext_suber()
{
    if(m_ext_suber)
    {
        delete m_ext_suber;
    }
}

void MsgFactory::create_cab_alarm_pusher(string address)
{
    m_cab_alarm_pusher = new GNMessager<int>(ZMQ_PUSH);
    m_cab_alarm_pusher->connect(address);
}

void MsgFactory::destory_cab_alarm_pusher()
{
    if(m_cab_alarm_pusher)
    {
        delete m_cab_alarm_pusher;
    }
}

void MsgFactory::create_cab_transmit_suber(string address,string filter)
{
	m_cab_transmit_suber = new GNMessager<int>(ZMQ_SUB);
    m_cab_transmit_suber->connect(address);
	m_cab_transmit_suber->set_recv_filter(filter);
}

void MsgFactory::destory_cab_transmit_suber()
{
    if(m_cab_transmit_suber)
    {
        delete m_cab_transmit_suber;
    }
}

GNMessager<int>& MsgFactory::cab_transmit_suber()
{ 
	return *m_cab_transmit_suber; 
}

void MsgFactory::create_cell_transmit_pusher(int cell_no, string address)
{
    int cell_idx = cell_no-1;
    m_cell_transmit_pusher[cell_idx] = new GNMessager<int>(ZMQ_PUSH);
    m_cell_transmit_pusher[cell_idx]->connect(address);
}

void MsgFactory::destory_cell_transmit_pusher(int cell_no)
{
    int cell_idx = cell_no-1;
    if(m_cell_transmit_pusher[cell_idx])
    {
        delete m_cell_transmit_pusher[cell_idx];
    }
}

GNMessager<int>& MsgFactory::cell_transmit_pusher(int cell_no)
{
	return *m_cell_transmit_pusher[cell_no-1];
}


void MsgFactory::create_cell_alarm_pusher(int cell_no, string address)
{
    int cell_idx = cell_no-1;
    m_cell_alarm_pusher[cell_idx] = new GNMessager<int>(ZMQ_PUSH);
    m_cell_alarm_pusher[cell_idx]->connect(address);
}

void MsgFactory::destory_cell_alarm_pusher(int cell_no)
{
    int cell_idx = cell_no-1;
    if(m_cell_alarm_pusher[cell_idx])
    {
        delete m_cell_alarm_pusher[cell_idx];
    }
}

void MsgFactory::create_pins_alarm_pusher(std::string address)
{
    m_pins_alarm_pusher = new GNMessager<int>(ZMQ_PUSH);
    m_pins_alarm_pusher->connect(address);
}

void MsgFactory::destory_pins_alarm_pusher()
{
    if(m_pins_alarm_pusher)
    {
        delete m_pins_alarm_pusher;
    }
}

void MsgFactory::create_cab_puber(string address)
{
    m_cab_puber = new GNMessager<Cmd_Cell_Msg_t>(ZMQ_PUB);
    m_cab_puber->bind(address);
}

void MsgFactory::destory_cab_puber()
{
    if(m_cab_puber)
    {
        delete m_cab_puber;
    }
}

void MsgFactory::create_cell_suber(int cell_no, string address, string filter)
{
    int cell_idx = cell_no-1;
    m_cell_suber[cell_idx] = new GNMessager<Cmd_Cell_Msg_t>(ZMQ_SUB);

    m_cell_suber[cell_idx]->connect(address);
    m_cell_suber[cell_idx]->set_recv_filter(filter);
}

void MsgFactory::destory_cell_suber(int cell_no)
{
    int cell_idx = cell_no-1;
    if(m_cell_suber[cell_idx])
    {
        delete m_cell_suber[cell_idx];
    }
}

void MsgFactory::create_cell_ctrl_suber(int cell_no, string address, string filter)
{
    int cell_idx = cell_no-1;
    m_cell_ctrl_suber[cell_idx] = new GNMessager<Cmd_Cell_Msg_t>(ZMQ_SUB);

    m_cell_ctrl_suber[cell_idx]->connect(address);
    m_cell_ctrl_suber[cell_idx]->set_recv_filter(filter);
}

void MsgFactory::destory_cell_ctrl_suber(int cell_no)
{
    int cell_idx = cell_no-1;
    if(m_cell_ctrl_suber[cell_idx])
    {
        delete m_cell_ctrl_suber[cell_idx];
    }
}

void MsgFactory::create_cell_to_ext_reply_pusher(int cell_no, string address)
{
    int cell_idx = cell_no-1;
    m_ext_reply_pusher[cell_idx] = new GNMessager<int>(ZMQ_PUSH);
	
    m_ext_reply_pusher[cell_idx]->connect(address);
}

void MsgFactory::destory_cell_to_ext_reply_pusher(int cell_no)
{
    int cell_idx = cell_no-1;
    if(m_ext_reply_pusher[cell_idx])
    {
        delete m_ext_reply_pusher[cell_idx];
    }
}

void MsgFactory::create_data_to_ext_store_pusher(string address)
{
    m_ext_store_pusher = new GNMessager<int>(ZMQ_PUSH);
    m_ext_store_pusher->connect(address);
}

/*
void MsgFactory::create_data_to_int_store_pusher(int cell_no, string address)
{
    int cell_idx = cell_no-1;

    m_int_store_pusher[cell_idx] = new GNMessager<Step_Process_Data_t>(ZMQ_PUSH);
    m_int_store_pusher[cell_idx]->connect(address);
}



void MsgFactory::destory_data_to_int_store_pusher(int cell_no)
{
    int cell_idx = cell_no-1;
    if(m_int_store_pusher[cell_idx])
    {
        delete m_int_store_pusher[cell_idx];
    }
}
*/

void MsgFactory::destory_data_to_ext_store_pusher()
{
	if(m_ext_store_pusher)
	{
		delete m_ext_store_pusher;
	}
}

/*
void MsgFactory::create_data_to_int_store_puller(int cell_no, string address)
{
    int cell_idx = cell_no-1;
    //m_int_store_puller[cell_idx] = new GNMessager<Step_Process_Data_t>(ZMQ_PULL);
    m_int_store_puller[cell_idx] = new GNMessager<int>(ZMQ_PULL);
    m_int_store_puller[cell_idx]->bind(address);
}

void MsgFactory::destory_data_to_int_store_puller(int cell_no)
{
    int cell_idx = cell_no-1;
    if(m_int_store_puller[cell_idx])
    {
        delete m_int_store_puller[cell_idx];
    }
}
*/

void MsgFactory::create_cell_pusher(int cell_no, std::string address)
{
    int cell_idx = cell_no-1;
    m_cell_pusher[cell_idx] = new GNMessager<Cmd_Pins_Msg_t>(ZMQ_PUSH);
    m_cell_pusher[cell_idx]->connect(address);
}

void MsgFactory::destory_cell_pusher(int cell_no)
{
    int cell_idx = cell_no-1;
    if(m_cell_pusher[cell_idx])
    {
        delete m_cell_pusher[cell_idx];
    }
}

void MsgFactory::create_pins_puller(std::string address)
{
    m_pins_puller = new GNMessager<Cmd_Pins_Msg_t>(ZMQ_PULL);
    m_pins_puller->bind(address);
}

void MsgFactory::destory_pins_puller()
{
    if(m_pins_puller)
    {
        delete m_pins_puller;
    }
}

void MsgFactory::create_pins_puber(std::string address)
{
    m_pins_puber = new GNMessager<Cmd_Cell_Msg_t>(ZMQ_PUB);
    m_pins_puber->bind(address);
}

void MsgFactory::destory_pins_puber()
{
    if(m_pins_puber)
    {
        delete m_pins_puber;
    }
}

void MsgFactory::create_pins_to_ext_reply_pusher(std::string address)
{
    m_pins_ext_reply_pusher = new GNMessager<int>(ZMQ_PUSH);
    m_pins_ext_reply_pusher->connect(address);
}

void MsgFactory::destory_pins_to_ext_reply_pusher()
{
    if(m_pins_ext_reply_pusher)
    {
        delete m_pins_ext_reply_pusher;
    }
}

#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
void MsgFactory::create_cab_to_ext_reply_pusher(std::string address)
{
    m_cab_ext_reply_pusher = new GNMessager<int>(ZMQ_PUSH);
    m_cab_ext_reply_pusher->connect(address);
}

void MsgFactory::destory_cab_to_ext_reply_pusher()
{
    if(m_cab_ext_reply_pusher)
    {
        delete m_cab_ext_reply_pusher;
    }
}
#endif

// 辅助类参数
#ifdef AUX_VOLTAGE_ACQ
void MsgFactory::create_aux_param_pusher(int cell_no, std::string address)
{
    int cell_idx = cell_no - 1;
    m_aux_param_pusher[cell_idx] = new GNMessager<Cmd_Aux_Param_t>(ZMQ_PUSH);
    m_aux_param_pusher[cell_idx]->bind(address + to_string(cell_no));
}

void MsgFactory::destory_aux_param_pusher(int cell_no)
{
    int cell_idx = cell_no - 1;
    if (m_aux_param_pusher[cell_idx])
    {
        delete m_aux_param_pusher[cell_idx];
    }
}
void MsgFactory::create_aux_param_puller(int cell_no, std::string address)
{
    int cell_idx = cell_no - 1;
    m_aux_param_puller[cell_idx] = new GNMessager<Cmd_Aux_Param_t>(ZMQ_PULL);
    m_aux_param_puller[cell_idx]->connect(address + to_string(cell_no));
}

void MsgFactory::destory_aux_param_puller(int cell_no)
{
    int cell_idx = cell_no - 1;
    if (m_aux_param_puller[cell_idx])
    {
        delete m_aux_param_puller[cell_idx];
    }
}
#endif

#ifdef MITSUBISHI_PLC
// 化成PLC
void MsgFactory::create_bf_plc_puller(int cell_no, std::string address)
{
    int cell_idx = cell_no - 1;
    m_bf_plc_puller[cell_idx] = new GNMessager<CMD_BFG_PLC_DATA_STRUCT>(ZMQ_PULL);
    m_bf_plc_puller[cell_idx]->bind(address);
}

void MsgFactory::destory_bf_plc_puller(int cell_no)
{
    int cell_idx = cell_no - 1;
    if (m_bf_plc_puller[cell_idx])
    {
        delete m_bf_plc_puller[cell_idx];
    }
}

void MsgFactory::create_bf_plc_puber(int cell_no, std::string address)
{
    int cell_idx = cell_no - 1;
    m_bf_plc_puber[cell_idx] = new GNMessager<Cmd_Cell_Msg_t>(ZMQ_PUB);
    m_bf_plc_puber[cell_idx]->bind(address);
}

void MsgFactory::destory_bf_plc_puber(int cell_no)
{
    int cell_idx = cell_no - 1;
    if (m_bf_plc_puber[cell_idx])
    {
        delete m_bf_plc_puber[cell_idx];
    }
}

// 分容PLC
void MsgFactory::create_bg_plc_puller(int cell_no, std::string address)
{
    int cell_idx = cell_no - 1;
    m_bg_plc_puller[cell_idx] = new GNMessager<CMD_BFG_PLC_DATA_STRUCT>(ZMQ_PULL);
    m_bg_plc_puller[cell_idx]->bind(address);
}

void MsgFactory::destory_bg_plc_puller(int cell_no)
{
    int cell_idx = cell_no - 1;
    if (m_bg_plc_puller[cell_idx])
    {
        delete m_bg_plc_puller[cell_idx];
    }
}

void MsgFactory::create_bg_plc_puber(int cell_no, std::string address)
{
    int cell_idx = cell_no - 1;
    m_bg_plc_puber[cell_idx] = new GNMessager<Cmd_Cell_Msg_t>(ZMQ_PUB);
    m_bg_plc_puber[cell_idx]->bind(address);
}

void MsgFactory::destory_bg_plc_puber(int cell_no)
{
    int cell_idx = cell_no - 1;
    if (m_bg_plc_puber[cell_idx])
    {
        delete m_bg_plc_puber[cell_idx];
    }
}
#endif

// 物流系统WMS
#if 0
void MsgFactory::create_wms_suber(int cell_no, std::string address)
{
    int cell_idx = cell_no - 1;
    m_wms_suber[cell_idx] = new GNMessager<CMD_WMS_DATA_STRUCT>(ZMQ_SUB);
    m_wms_suber[cell_idx]->connect(address);
}

void MsgFactory::destory_wms_suber(int cell_no)
{
    int cell_idx = cell_no - 1;
    if (m_wms_suber[cell_idx])
    {
        delete m_wms_suber[cell_idx];
    }
}

void MsgFactory::create_wms_pusher(int cell_no, std::string address)
{
    int cell_idx = cell_no - 1;
    m_wms_pusher[cell_idx] = new GNMessager<CMD_WMS_DATA_STRUCT>(ZMQ_PUSH);
    m_wms_pusher[cell_idx]->connect(address);
}

void MsgFactory::destory_wms_pusher(int cell_no)
{
    int cell_idx = cell_no - 1;
    if (m_wms_pusher[cell_idx])
    {
        delete m_wms_pusher[cell_idx];
    }
}

void MsgFactory::create_wms_puber(std::string address)
{
    m_wms_puber = new GNMessager<CMD_WMS_DATA_STRUCT>(ZMQ_PUB);
    m_wms_puber->bind(address);
}

void MsgFactory::destory_wms_puber()
{
    if (m_wms_puber)
    {
        delete m_wms_puber;
    }
}

void MsgFactory::create_wms_puller(std::string address)
{
    m_wms_puller = new GNMessager<CMD_WMS_DATA_STRUCT>(ZMQ_PULL);
    m_wms_puller->bind(address);
}

void MsgFactory::destory_wms_puller()
{
    if (m_wms_puller)
    {
        delete m_wms_puller;
    }
}
#endif

