/**************************************************************
* Copyright (C) 2018, 上海鼎格信息科技有限公司
* All rights reserved.
* 产品号 :
* 所属组件 :GN
* 模块名称 :Configuration
* 文件名称 :Configuration.h
* 概要描述 :
* 版本      日期         作者    内容
* V1.0    2018-05-18     zhujs     …
***************************************************************/
#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include "json/json.h"
#include <string>
#include <vector>


#define MAX_CELLS_NBR     				8
#ifdef CABTOOL_64CH
	#define MAX_CHANNELS_NBR			64
#else
	#define MAX_CHANNELS_NBR			36
#endif

#if defined(AWH_PRECHARGE_SYS)	
#define CELL_TEMP_ACQ_NUM		7		//预充7路
#elif defined(AWH_GRADING_SYS)
#define CELL_TEMP_ACQ_NUM		9		//分容9路
#else
#define CELL_TEMP_ACQ_NUM		6		//化成6路
#endif
#if (defined(AWH_PRECHARGE_SYS) || defined(AWH_FORMATION_SYS))
#define DEVICE_NUM_ONE_CELL		5		//每库5个电源模块
#else
#define DEVICE_NUM_ONE_CELL		9		//每库9个电源模块
#endif

//#define ENABLE_RUNSTATUS_SAVE	1

class Configuration
{
public:
    static Configuration* instance();

	void conf_dir_set(std::string path);
	std::string conf_dir_get();
	
    void read_config_file(std::string file_name);

    int line_no();
    int cabinet_no();

    std::string plc_ip_address();
	int slmp_plc_socket(std::string &ip_addr);
	int slmp_plc_socket(int cell_no,std::string &ip_addr);
#if defined(AWH_FORMATION_SYS)
	bool can_bitrate_config(int canNo,int &bitrate);
	bool can_cell_config(int cellNo,int &can_port);
	int vacuum_fluc_time_config();
#endif
	void plc_enable_config();
	void vacuum_enable_config();
	void tempAcq_enable_config();
	void cellTemp_contrast_config();
	void cellTemp_range_config();
	void get_cellTemp_contrast(float &contrast);
	void get_cellTemp_range(float &upper,float &lower);
	bool is_plc_enable();
	bool is_vacuum_enable();
	bool is_tempAcq_enable();
	std::string scanner_ip_address();

	std::string ext_request_ip();
    std::string ext_request_socket();   //请求数据端口
    std::string ext_reply_socket();     //应答和记录数据
    std::string ext_alarm_socket();     //异常数据端口
	std::string ext_transmit_suber_socket();
	std::string ext_transmit_pusher_socket();
	
    std::string bts_suber_socket();
    std::string bts_pusher_socket();

    std::string ext_data_store_socket();
	//std::string int_data_store_socket(int cell_no);
	
    std::string int_cab_puber_socket();

    std::string int_pins_puller_socket();
    std::string int_pins_puber_socket();

	std::string bf_plc_ip(int cell_no);
	std::string bg_plc_ip(int cell_no);
	std::string wms_ip();

#ifdef MITSUBISHI_PLC
	std::string int_bfplc_puller_socket(int cell_no);	// 化成PLC
	std::string int_bfplc_puber_socket(int cell_no);
	std::string int_bgplc_puller_socket(int cell_no);	// 分容PLC
	std::string int_bgplc_puber_socket(int cell_no);
#endif	
	std::string int_wms_puber_socket();					// 物流WMS
	std::string int_wms_pusher_socket();				

    std::vector<int> cells();

    int channel_socket(int cell_no, int chan_no, std::string &ip_addr);
#if defined(AWH_FORMATION_SYS)	
	void device_socket(int cell_no, int device,std::vector<int> &channels);
#else
	int device_socket(int cell_no, int device, std::string &ip_addr,std::vector<int> &channels);
#endif
	int channel_server_socket(std::string &ip_addr);
#if 0	
	void channel_temperature_acq_socket(int device, std::string &ip_addr,int &cell_no,std::vector<int> &channels);
	void cell_temperature_acq_socket(int device, std::string &ip_addr,int &cell_no,std::vector<int> &channels);
#else
	void temperature_acqBox_socket(int device, std::string &ip_addr,int &cell_no,std::vector<int> &channelTempNo,std::vector<int> &cellTempNo);
#endif
    void set_send_detail_data_flag(int cell_no, bool send){m_cell_send_data[cell_no-1] = send;}
    bool need_send_detail_data(int cell_no){return m_cell_send_data[cell_no-1];}

private:
    Configuration();
    Configuration(const Configuration&);
    Configuration& operator=(const Configuration&);
    ~Configuration();

	Json::Value m_cfg_root;
	float m_cellTemp_contrast;
	float m_cellTemp_range_upper;
	float m_cellTemp_range_lower;
	bool m_enable_plc;
	bool m_enable_vacuum;
	bool m_enable_tempacq;
	bool m_cell_send_data[MAX_CELLS_NBR];

	std::string m_conf_dir;								// 配置文件所在的目录（绝对路径）
};

#endif //CONFIGURATION_H_


