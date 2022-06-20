/**************************************************************
* Copyright (C) 2018, 上海鼎格信息科技有限公司
* All rights reserved.
* 产品号 :
* 所属组件 :GN
* 模块名称 :Configuration
* 文件名称 :Configuration.cpp
* 概要描述 :
* 版本      日期         作者    内容
* V1.0    2018-05-18     zhujs     …
***************************************************************/
#include <sstream>
#include <fstream>
#include "log4z.h"
#include "Configuration.h"
#include "Type.h"

using namespace std;

Configuration* Configuration::instance()
{
    static Configuration obj;
    return &obj;
}

Configuration::Configuration()
{
    for (int i = 0; i < MAX_CELLS_NBR; i++)
    {
        m_cell_send_data[i] = false;
    }
	m_enable_plc = false;
	m_enable_vacuum = false;
	m_enable_tempacq = false;
	m_cellTemp_contrast = 1000.0;
	m_cellTemp_range_upper = 1000.0;
	m_cellTemp_range_lower = -90.0;
}

Configuration::~Configuration()
{}

void Configuration::read_config_file(string file_name)
{
	Json::Reader reader;

	ifstream is(file_name);
	if (!is)
	{
		is.close();
        LOGFMTE("The %s Config File not exist!", file_name.c_str());
        throw -301;
	}

	bool re = reader.parse(is, m_cfg_root, false);
	if (!re)
	{
		is.close();
        LOGFMTE("The %s Config File is not json format.", file_name.c_str());
        throw -300;
	}
}

void Configuration::conf_dir_set(std::string path)
{
	m_conf_dir = path;
}

std::string Configuration::conf_dir_get()
{
	return m_conf_dir;
}

int Configuration::line_no()
{
    return m_cfg_root["LineNo"].asInt();
}

int Configuration::cabinet_no()
{
    return m_cfg_root["CabinetNo"].asInt();
}

string Configuration::plc_ip_address()
{
    return m_cfg_root["PLCAddress"].asString();
}

string Configuration::scanner_ip_address()
{
    return "192.168.1.3";
//    return m_cfg_root["ScannerAddress"].asString();
}

string Configuration::ext_request_ip()
{
    return string(m_cfg_root["Master"].asString().erase(0, 6));
}

string Configuration::ext_request_socket()
{
    return string(m_cfg_root["Master"].asString() + ":5000");
}

string Configuration::ext_reply_socket()
{
    return string(m_cfg_root["Master"].asString() + ":5002");
}

string Configuration::ext_alarm_socket()
{
    return string(m_cfg_root["Master"].asString() + ":5001");
}

string Configuration::ext_transmit_suber_socket()
{
    return string(m_cfg_root["Master"].asString() + ":5003");
}

string Configuration::ext_transmit_pusher_socket()
{
    return string(m_cfg_root["Master"].asString() + ":5004");
}



string Configuration::bts_suber_socket()
{
    return string(m_cfg_root["Master"].asString() + ":5006");
}

string Configuration::bts_pusher_socket()
{
    return string(m_cfg_root["Master"].asString() + ":5007");
}

/*
string Configuration::int_data_store_socket(int cell_no)
{
    return string("inproc:///tmp/lbfs-data-" + to_string(cell_no));
	//return string("ipc:///tmp/lbfs-data");
}
*/

string Configuration::ext_data_store_socket()
{
    //return string("ipc:///tmp/lbfs-data");
    //return string("tcp:///tmp/lbfs-data");
    return string("tcp://127.0.0.1:5557");
 }

string Configuration::int_cab_puber_socket()
{
    return string("inproc://cab-puber-pipe");
}

string Configuration::int_pins_puller_socket()
{
    return string("inproc://pins-puller-pipe");
}

string Configuration::int_pins_puber_socket()
{
    return string("inproc://pins-puber-pipe");
}

// 化成PLC的IP地址
string Configuration::bf_plc_ip(int cell_no)
{
	Json::Value cell_elems = m_cfg_root["Cells"];
    int cell_size = cell_elems.size();

	for (int i=0; i<cell_size; i++)
	{
		if (cell_elems[i]["CellNo"].asInt() == cell_no)
		{
			return cell_elems[i]["BF_PLC_IP"].asString();
		}
	}

	return "";
}

// 分容PLC的IP地址
string Configuration::bg_plc_ip(int cell_no)
{
	Json::Value cell_elems = m_cfg_root["Cells"];
    int cell_size = cell_elems.size();

	for (int i=0; i<cell_size; i++)
	{
		if (cell_elems[i]["CellNo"].asInt() == cell_no)
		{
			return cell_elems[i]["BG_PLC_IP"].asString();
		}
	}

	return "";
}

// WMS物流系统IP
string Configuration::wms_ip()
{
    return m_cfg_root["WMS_IP"].asString();
}

#ifdef MITSUBISHI_PLC 
// 化成PLC pull of zeromq
string Configuration::int_bfplc_puller_socket(int cell_no)
{
    return string("inproc://bf-plc-puller-" + to_string(cell_no));
}

// 化成PLC pub of zeromq
string Configuration::int_bfplc_puber_socket(int cell_no)
{
    return string("inproc://bf-plc-puber-" + to_string(cell_no));
}

// 分容PLC pull of zeromq
string Configuration::int_bgplc_puller_socket(int cell_no)
{
    return string("inproc://bg-plc-puller-" + to_string(cell_no));
}

// 分容PLC pub of zeromq
string Configuration::int_bgplc_puber_socket(int cell_no)
{
    return string("inproc://bg-plc-puber-" + to_string(cell_no));
}
#endif

// 物流系统 WMS
string Configuration::int_wms_puber_socket()
{
    return string("inproc://wms-puber");
}

string Configuration::int_wms_pusher_socket()
{
    return string("inproc://wms-pusher");
}

vector<int> Configuration::cells()
{
	Json::Value cell_elems = m_cfg_root["Cells"];
    int cell_size = cell_elems.size();

    vector<int> result_vec;
	for (int i = 0; i < cell_size; i++)
	{
		result_vec.push_back(cell_elems[i]["CellNo"].asInt());
	}

	return result_vec;
}

int Configuration::channel_socket(int cell_no, int chan_no, std::string &ip_addr)
{
    int port = -1;
    string chan_socket = "";
	Json::Value cell_elems = m_cfg_root["Cells"];
    int cell_size = cell_elems.size();
	
	ip_addr = "";
	for (int i = 0; i < cell_size; i++)
	{
		if (cell_elems[i]["CellNo"].asInt() == cell_no)
		{
            Json::Value channel_elems = cell_elems[i]["Channels"];
            int chan_size = channel_elems.size();
            for (int j = 0; j < chan_size; j++)
            {
            	Json::Value chanNo_elems = channel_elems[j]["ChannelNo"];
				int chanNo_size = chanNo_elems.size();

				for(int k = 0; k < chanNo_size; k++)
				{
					if(chanNo_elems[k].asInt() == chan_no)
					{
	                    chan_socket = channel_elems[j]["DeviceAddress"].asString();

	                    std::string::size_type idx = chan_socket.find(':');
	                    ip_addr = chan_socket.substr(0, idx);

	                    std::istringstream is(chan_socket.substr(idx + 1));
	                    is >> port;
	                    break;
                	}
				}

				if(ip_addr != "")
				{
					break;
				}
            }
		}
	}

	return port;
}


#if defined(AWH_FORMATION_SYS)
void Configuration::device_socket(int cell_no, int device,vector<int> &channels)
{
	Json::Value cell_elems = m_cfg_root["Cells"];
    int cell_size = cell_elems.size();
	
	for (int i = 0; i < cell_size; i++)
	{
		if (cell_elems[i]["CellNo"].asInt() == cell_no)
		{
            Json::Value channel_elems = cell_elems[i]["Channels"];
            int chan_size = channel_elems.size();
            for (int j = 0; j < chan_size; j++)
            {
				if(channel_elems[j]["Device"].asInt() == device)
				{
				    Json::Value chanNo_elems = channel_elems[j]["ChannelNo"];
					int chanNo_size = chanNo_elems.size();

					for(int k = 0; k < chanNo_size; k++)
					{
						channels.push_back(chanNo_elems[k].asInt());
					}
					
                    break;
            	}
            }
		}
	}
}
#else
int Configuration::device_socket(int cell_no, int device, std::string &ip_addr,vector<int> &channels)
{
    int port = -1;
    string chan_socket = "";
	Json::Value cell_elems = m_cfg_root["Cells"];
    int cell_size = cell_elems.size();
	
	ip_addr = "";
	for (int i = 0; i < cell_size; i++)
	{
		if (cell_elems[i]["CellNo"].asInt() == cell_no)
		{
            Json::Value channel_elems = cell_elems[i]["Channels"];
            int chan_size = channel_elems.size();
            for (int j = 0; j < chan_size; j++)
            {
				if(channel_elems[j]["Device"].asInt() == device)
				{
				    Json::Value chanNo_elems = channel_elems[j]["ChannelNo"];
					int chanNo_size = chanNo_elems.size();

					for(int k = 0; k < chanNo_size; k++)
					{
						channels.push_back(chanNo_elems[k].asInt());
					}
					
                    chan_socket = channel_elems[j]["DeviceAddress"].asString();

                    std::string::size_type idx = chan_socket.find(':');
                    ip_addr = chan_socket.substr(0, idx);

                    std::istringstream is(chan_socket.substr(idx + 1));
                    is >> port;
                    break;
            	}
            }
		}
	}

	return port;
}
#endif

int Configuration::channel_server_socket(std::string &ip_addr)
{
	int port = -1;
	string server_socket = "";
	
	ip_addr = "";
	server_socket = m_cfg_root["ChannelServer"].asString();

	std::string::size_type idx = server_socket.find(':');
	ip_addr = server_socket.substr(0, idx);

	std::istringstream is(server_socket.substr(idx + 1));
	is >> port; 

	if(ip_addr == "")
	{
		ip_addr = "192.168.1.8";
		port = 5000;
	}

	return port;
}

#if 0
void Configuration::channel_temperature_acq_socket(int device, string &ip_addr,int &cell_no,vector<int> &channels)
{
    string acq_socket = "";
	Json::Value acq_elems = m_cfg_root["ChannelTempAcq"];
    int elemsize = acq_elems.size();
	
	ip_addr = "";
	for (int i = 0; i < elemsize; i++)
	{
		Json::Value device_elems = acq_elems[i];
		if (device_elems["Device"].asInt() == device)
		{
			cell_no = device_elems["CellNo"].asInt();
            
	       	Json::Value chanNo_elems = device_elems["ChannelNo"];
			int chanNo_size = chanNo_elems.size();
			for(int j = 0; j < chanNo_size; j++)
			{
				channels.push_back(chanNo_elems[j].asInt());
			}
			
	        acq_socket = device_elems["TempAcqAddress"].asString();

			if(acq_socket != "")
			{
				ip_addr = acq_socket;
			}

	        break;
		}
	}
}


void Configuration::cell_temperature_acq_socket(int device, string &ip_addr,int &cell_no,vector<int> &channels)
{
    string acq_socket = "";
	Json::Value acq_elems = m_cfg_root["CellTempAcq"];
    int elemsize = acq_elems.size();
	
	ip_addr = "";
	for (int i = 0; i < elemsize; i++)
	{
		Json::Value device_elems = acq_elems[i];
		if (device_elems["Device"].asInt() == device)
		{
			cell_no = device_elems["CellNo"].asInt();
			
	       	Json::Value chanNo_elems = device_elems["ChannelNo"];
			int chanNo_size = chanNo_elems.size();
			for(int j = 0; j < chanNo_size; j++)
			{
				channels.push_back(chanNo_elems[j].asInt());
			}
			
	        acq_socket = device_elems["TempAcqAddress"].asString();

			if(acq_socket != "")
			{
				ip_addr = acq_socket;
			}

	        break;
		}
	}
}
#else
void Configuration::temperature_acqBox_socket(int device, string &ip_addr,int &cell_no,vector<int> &channelTempNo,vector<int> &cellTempNo)
{
    string acq_socket = "";
	Json::Value acq_elems = m_cfg_root["TempAcqBox"];
    int elemsize = acq_elems.size();
	
	ip_addr = "";
	for (int i = 0; i < elemsize; i++)
	{
		Json::Value device_elems = acq_elems[i];
		if (device_elems["Device"].asInt() == device)
		{
			int j = 0;
			cell_no = device_elems["CellNo"].asInt();
            
	       	Json::Value chanTemp_elems = device_elems["ChannelTempAcq"];
			int chanTemp_size = chanTemp_elems.size();
			for(j = 0; j < chanTemp_size; j++)
			{
				channelTempNo.push_back(chanTemp_elems[j].asInt());
			}
			
			Json::Value cellTemp_elems = device_elems["CellTempAcq"];
			int cellTemp_size = cellTemp_elems.size();
			for(j = 0; j < cellTemp_size; j++)
			{
				cellTempNo.push_back(cellTemp_elems[j].asInt());
			}
			
	        acq_socket = device_elems["TempAcqAddress"].asString();

			if(acq_socket != "")
			{
				ip_addr = acq_socket;
			}

	        break;
		}
	}
}
#endif


int Configuration::slmp_plc_socket(std::string &ip_addr)
{
	int port = -1;
	string plc_socket = "";
	
	ip_addr = "";
	plc_socket = m_cfg_root["PLCAddress"].asString();

	std::string::size_type idx = plc_socket.find(':');
	ip_addr = plc_socket.substr(0, idx);

	std::istringstream is(plc_socket.substr(idx + 1));
	is >> port; 

	return port;
}

int Configuration::slmp_plc_socket(int cell_no,std::string &ip_addr)
{
	int port = -1;
	string plc_socket = "";
	
	ip_addr = "";

	Json::Value cell_elems = m_cfg_root["Cells"];
    int cell_size = cell_elems.size();

	for (int i=0; i<cell_size; i++)
	{
		if (cell_elems[i]["CellNo"].asInt() == cell_no)
		{ 
			plc_socket = cell_elems[i]["PLCAddress"].asString();
			std::string::size_type idx = plc_socket.find(':');
			ip_addr = plc_socket.substr(0, idx);
			std::istringstream is(plc_socket.substr(idx + 1));
			is >> port; 
			return port;
		}
	}
	return port;
}

#if defined(AWH_FORMATION_SYS)
bool Configuration::can_bitrate_config(int canNo,int &bitrate)
{
	Json::Value can_elems = m_cfg_root["CAN_CellNo"];

	if(can_elems.type() != Json::nullValue)
	{
		int elemsize = can_elems.size();

		for(int i = 0; i < elemsize; i++)
		{
			if(canNo == can_elems[i]["CAN_Port"].asInt())
			{
				bitrate = can_elems[i]["Bitrate"].asInt() * 1000;
				return true;
			}
		}
	}

	return false;
}

bool Configuration::can_cell_config(int cellNo,int &can_port)
{
	Json::Value can_elems = m_cfg_root["CAN_CellNo"];

	if(can_elems.type() != Json::nullValue)
	{
		int elemsize = can_elems.size();

		for(int i = 0; i < elemsize; i++)
		{
			Json::Value cell_elems = can_elems[i]["Cells"];
			
			for(uint32_t j = 0; j < cell_elems.size(); j++)
			{
				if(cellNo == cell_elems[j].asInt())
				{
					can_port = can_elems[i]["CAN_Port"].asInt();
					return true;
				}
			}
		}
	}

	return false;
}

int Configuration::vacuum_fluc_time_config()
{
	int count = 60;
	
	if(m_cfg_root["Vacuum_Fluc_Time"].isInt())
	{
		count = m_cfg_root["Vacuum_Fluc_Time"].asInt();
		if(count <= 0)
		{
			count = 60;
		}
	}

	return count;
}
#endif

void Configuration::plc_enable_config()
{
	if(m_cfg_root["Enable_PLC"].isInt())
	{
		if(m_cfg_root["Enable_PLC"].asInt())
		{
			m_enable_plc = true;
		}
		else
		{
			m_enable_plc = false;
		}		
	}
	else
	{
		m_enable_plc = false;	
	}
}

void Configuration::vacuum_enable_config()
{
	if(m_cfg_root["Enable_Vacuum"].isInt())
	{
		if(m_cfg_root["Enable_Vacuum"].asInt())
		{
			m_enable_vacuum = true;
		}
		else
		{
			m_enable_vacuum = false;
		}		
	}
	else
	{
		m_enable_vacuum = false;	
	}
}

void Configuration::tempAcq_enable_config()
{
	if(m_cfg_root["Enable_TempAcq"].isInt())
	{
		if(m_cfg_root["Enable_TempAcq"].asInt())
		{
			m_enable_tempacq = true;
		}
		else
		{
			m_enable_tempacq = false;
		}		
	}
	else
	{
		m_enable_tempacq = false;	
	}
}

void Configuration::cellTemp_contrast_config()
{
	if(m_cfg_root["CellTemp_Contrast"].type() != Json::nullValue)
	{
		m_cellTemp_contrast =  m_cfg_root["CellTemp_Contrast"].asDouble();
	}
	else
	{
		m_cellTemp_contrast = 1000.0;
	}
}

void Configuration::cellTemp_range_config()
{
	if(m_cfg_root["CellTemp_Range_Upper"].type() != Json::nullValue)
	{
		m_cellTemp_range_upper =  m_cfg_root["CellTemp_Range_Upper"].asDouble();
	}
	else
	{
		m_cellTemp_range_upper = 1000.0;
	}

	if(m_cfg_root["CellTemp_Range_Lower"].type() != Json::nullValue)
	{
		m_cellTemp_range_lower =  m_cfg_root["CellTemp_Range_Lower"].asDouble();
	}
	else
	{
		m_cellTemp_range_lower = -90.0;
	}
}

void Configuration::get_cellTemp_contrast(float &contrast)
{
	contrast = m_cellTemp_contrast;
}

void Configuration::get_cellTemp_range(float &upper,float &lower)
{
	upper = m_cellTemp_range_upper;
	lower = m_cellTemp_range_lower;
}

bool Configuration::is_plc_enable()
{
	return m_enable_plc;
}

bool Configuration::is_vacuum_enable()
{
	return m_enable_vacuum;
}

bool Configuration::is_tempAcq_enable()
{
	return m_enable_tempacq;
}

