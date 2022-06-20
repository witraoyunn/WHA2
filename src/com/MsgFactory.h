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
#ifndef MSGFACTORY_H_
#define MSGFACTORY_H_

#include "Type.h"
#include "GNMessager.h"

#define TOPIC_CAB_FORMAT_STR    "Cab%d# "
#define TOPIC_CELL_FORMAT_STR   "CELL_%d_# "

class MsgFactory
{
public:
    static MsgFactory* instance();

//	// BTS：接收命令和回复
//    void create_bts_suber(std::string address, std::string filter);
//    void destory_bts_suber();
//    GNMessager<int>& bts_suber() { return *m_bts_suber; }
//	
//    void create_bts_pusher(std::string address);
//    void destory_bts_pusher();
//    GNMessager<int>& bts_pusher() { return *m_bts_pusher; }
	// 上位机：接收命令
    void create_cab_ext_suber(std::string address, std::string filter);
    void destory_cab_ext_suber();
    GNMessager<int>& cab_ext_suber() { return *m_ext_suber; }

	void create_cab_transmit_suber(std::string address,std::string filter);
	void destory_cab_transmit_suber();
	GNMessager<int>& cab_transmit_suber();

	void create_cell_transmit_pusher(int cell_no, std::string address);
	void destory_cell_transmit_pusher(int cell_no);
	GNMessager<int>& cell_transmit_pusher(int cell_no);
	
	// 上位机：命令执行结果返回
    void create_cell_to_ext_reply_pusher(int cell_no, std::string address);
    void destory_cell_to_ext_reply_pusher(int cell_no);
    GNMessager<int>& cell_to_ext_reply_pusher(int cell_no) { return *m_ext_reply_pusher[cell_no-1]; }
	// 工步数据上传
    void create_data_to_ext_store_pusher(std::string address);
    void destory_data_to_ext_store_pusher();
	GNMessager<int>& data_to_ext_store_pusher() { return *m_ext_store_pusher; }

	//void create_data_to_int_store_pusher(int cell_no, std::string address);
	//void destory_data_to_int_store_pusher(int cell_no);
    //GNMessager<Step_Process_Data_t>& data_to_int_store_pusher(int cell_no) { return *m_int_store_pusher[cell_no-1]; }
	//void create_data_to_int_store_puller(int cell_no, std::string address);
    //void destory_data_to_int_store_puller(int cell_no);
    //GNMessager<Step_Process_Data_t>& data_to_int_store_puller(int cell_no) { return *m_int_store_puller[cell_no-1]; }
    //GNMessager<int>& data_to_int_store_puller(int cell_no) { return *m_int_store_puller[cell_no-1]; }

	// 上位机：针床命令执行结果返回
    void create_pins_to_ext_reply_pusher(std::string address);
    void destory_pins_to_ext_reply_pusher();
    GNMessager<int>& pins_to_ext_reply_pusher() { return *m_pins_ext_reply_pusher; }

	// 上位机：电源柜消息上报
#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))	
	void create_cab_to_ext_reply_pusher(std::string address);
    void destory_cab_to_ext_reply_pusher();
    GNMessager<int>& cab_to_ext_reply_pusher() { return *m_cab_ext_reply_pusher; }
#endif

	// 告警信息推送
    void create_cab_alarm_pusher(std::string address);
    void destory_cab_alarm_pusher();
    GNMessager<int>& cab_alarm_pusher() { return *m_cab_alarm_pusher; }

    void create_cell_alarm_pusher(int cell_no, std::string address);
    void destory_cell_alarm_pusher(int cell_no);
    GNMessager<int>& cell_alarm_pusher(int cell_no) { return *m_cell_alarm_pusher[cell_no-1]; }
	// 针床告警信息推送
    void create_pins_alarm_pusher(std::string address);
    void destory_pins_alarm_pusher();
    GNMessager<int>& pins_alarm_pusher() { return *m_pins_alarm_pusher; }

	// 机柜消息发布
    void create_cab_puber(std::string address);
    void destory_cab_puber();
    GNMessager<Cmd_Cell_Msg_t>& cab_puber() { return *m_cab_puber; }
	// 机柜消息订阅
    void create_cell_suber(int cell_no, std::string address, std::string filter);
    void destory_cell_suber(int cell_no);
    GNMessager<Cmd_Cell_Msg_t>& cell_suber(int cell_no) { return *m_cell_suber[cell_no-1]; }

    void create_cell_ctrl_suber(int cell_no, std::string address, std::string filter = "ALL");
    void destory_cell_ctrl_suber(int cell_no);
    GNMessager<Cmd_Cell_Msg_t>& cell_ctrl_suber(int cell_no) { return *m_cell_ctrl_suber[cell_no-1]; }

    void create_cell_pusher(int cell_no, std::string address);
    void destory_cell_pusher(int cell_no);
    GNMessager<Cmd_Pins_Msg_t>& cell_pusher(int cell_no) { return *m_cell_pusher[cell_no-1]; }

    void create_pins_puller(std::string address);
    void destory_pins_puller();
    GNMessager<Cmd_Pins_Msg_t>& pins_puller() { return *m_pins_puller; }

    void create_pins_puber(std::string address);
    void destory_pins_puber();
    GNMessager<Cmd_Cell_Msg_t>& pins_puber() { return *m_pins_puber; }
	
	// 辅助类参数
#ifdef AUX_VOLTAGE_ACQ
    void create_aux_param_pusher(int cell_no, std::string address="inproc://aux_param-puller-pipe");
    void destory_aux_param_pusher(int cell_no);
    GNMessager<Cmd_Aux_Param_t>& aux_param_pusher(int cell_no) { return *m_aux_param_pusher[cell_no-1]; }
	
    void create_aux_param_puller(int cell_no, std::string address="inproc://aux_param-puller-pipe");
    void destory_aux_param_puller(int cell_no);
    GNMessager<Cmd_Aux_Param_t>& aux_param_puller(int cell_no) { return *m_aux_param_puller[cell_no-1]; }
#endif

#ifdef MITSUBISHI_PLC
	// 化成PLC
	void create_bf_plc_puller(int cell_no, std::string address);
	void destory_bf_plc_puller(int cell_no);
	GNMessager<CMD_BFG_PLC_DATA_STRUCT>& bf_plc_puller(int cell_no) { return *m_bf_plc_puller[cell_no - 1]; }

	void create_bf_plc_puber(int cell_no, std::string address);
	void destory_bf_plc_puber(int cell_no);
	GNMessager<Cmd_Cell_Msg_t>& bf_plc_puber(int cell_no) { return *m_bf_plc_puber[cell_no - 1]; }
	
	// 分容PLC
	void create_bg_plc_puller(int cell_no, std::string address);
	void destory_bg_plc_puller(int cell_no);
	GNMessager<CMD_BFG_PLC_DATA_STRUCT>& bg_plc_puller(int cell_no) { return *m_bg_plc_puller[cell_no - 1]; }

	void create_bg_plc_puber(int cell_no, std::string address);
	void destory_bg_plc_puber(int cell_no);
	GNMessager<Cmd_Cell_Msg_t>& bg_plc_puber(int cell_no) { return *m_bg_plc_puber[cell_no - 1]; }
#endif

	// 物流系统WMS，暂时用不着，后面改成消防系统
#if 0	
	void create_wms_suber(int cell_no, std::string address);
	void destory_wms_suber(int cell_no);
	GNMessager<CMD_WMS_DATA_STRUCT>& wms_suber(int cell_no) { return *m_wms_suber[cell_no - 1]; }
	
	void create_wms_pusher(int cell_no, std::string address);
	void destory_wms_pusher(int cell_no);
	GNMessager<CMD_WMS_DATA_STRUCT>& wms_pusher(int cell_no) { return *m_wms_pusher[cell_no - 1]; }

	void create_wms_puber(std::string address);
	void destory_wms_puber();
	GNMessager<CMD_WMS_DATA_STRUCT>& wms_puber() { return *m_wms_puber; }

	void create_wms_puller(std::string address);
	void destory_wms_puller();
	GNMessager<CMD_WMS_DATA_STRUCT>& wms_puller() { return *m_wms_puller; }
#endif

private:
    MsgFactory();
    MsgFactory(const MsgFactory&);
    MsgFactory& operator=(const MsgFactory&);
    ~MsgFactory();

//	// 与中原BTS通讯	
//    GNMessager<int> *m_bts_suber;
//    GNMessager<int> *m_bts_pusher;

	// 与老上位机通讯
    GNMessager<int> *m_ext_suber;
    GNMessager<int> *m_ext_reply_pusher[MAX_CELLS_NBR];
#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))	
	GNMessager<int> *m_cab_ext_reply_pusher;
#endif

    //GNMessager<Step_Process_Data_t> *m_int_store_pusher[MAX_CELLS_NBR];
    //GNMessager<Step_Process_Data_t> *m_int_store_puller[MAX_CELLS_NBR];
    //GNMessager<int> *m_int_store_puller[MAX_CELLS_NBR];
    GNMessager<int> *m_ext_store_pusher;

    GNMessager<int> *m_cab_alarm_pusher;
    GNMessager<int> *m_cell_alarm_pusher[MAX_CELLS_NBR];
    GNMessager<int> *m_pins_alarm_pusher;

	GNMessager<int> *m_cab_transmit_suber;
	GNMessager<int> *m_cell_transmit_pusher[MAX_CELLS_NBR];

    GNMessager<Cmd_Cell_Msg_t> *m_cab_puber;
    GNMessager<Cmd_Cell_Msg_t> *m_cell_suber[MAX_CELLS_NBR];
    GNMessager<Cmd_Cell_Msg_t> *m_cell_ctrl_suber[MAX_CELLS_NBR];

    GNMessager<Cmd_Pins_Msg_t> *m_cell_pusher[MAX_CELLS_NBR];

    GNMessager<Cmd_Pins_Msg_t> *m_pins_puller;
    GNMessager<Cmd_Cell_Msg_t> *m_pins_puber;
    GNMessager<int>            *m_pins_ext_reply_pusher;

#ifdef AUX_VOLTAGE_ACQ
    GNMessager<Cmd_Aux_Param_t> *m_aux_param_pusher[MAX_CELLS_NBR];
    GNMessager<Cmd_Aux_Param_t> *m_aux_param_puller[MAX_CELLS_NBR];
#endif	

#ifdef MITSUBISHI_PLC
	// 化成PLC
    GNMessager<CMD_BFG_PLC_DATA_STRUCT> *m_bf_plc_puller[MAX_CELLS_NBR];
    GNMessager<Cmd_Cell_Msg_t>          *m_bf_plc_puber[MAX_CELLS_NBR];
	// 分容PLC
    GNMessager<CMD_BFG_PLC_DATA_STRUCT> *m_bg_plc_puller[MAX_CELLS_NBR];
    GNMessager<Cmd_Cell_Msg_t>          *m_bg_plc_puber[MAX_CELLS_NBR];
#endif
	// 物流系统WMS
#if 0	
    GNMessager<CMD_WMS_DATA_STRUCT>  *m_wms_suber[MAX_CELLS_NBR];
    GNMessager<CMD_WMS_DATA_STRUCT>  *m_wms_pusher[MAX_CELLS_NBR];
    GNMessager<CMD_WMS_DATA_STRUCT>  *m_wms_puber;
    GNMessager<CMD_WMS_DATA_STRUCT>  *m_wms_puller;
#endif	
};

#endif //MSGFACTORY_H_

