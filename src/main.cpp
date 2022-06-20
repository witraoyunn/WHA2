/**************************************************************
* Copyright (C) 2019, 上海鼎格信息科技有限公司
* All rights reserved.
* 产品号   :
* 所属组件 :
* 模块名称 :
* 文件名称 : main.cpp
* 概要描述 :
* 历史记录 :
* 版本      日期          作者           内容
* 1.0     20190416    zhangw
* 2.0     20200424    朱海峰          增加
***************************************************************/

#include <string>
#include <vector>
#include <thread>
#include <exception>
#include <sys/types.h>  
#include <sys/stat.h>

#include "zmq.h"
#include "log4z.h"
#include "Configuration.h"

#include "Cabinet.h"
#include "Cell.h"
#include "Channel.h"
#include "FormatConvert.h"

#include "DCRWorker.h"
#include "PinsWorker.h"
//#include "BatteryFormationPLC.h"
//#include "BatteryGradingPLC.h"
#include "AuxVolAcq.h"
#include "AuxTempAcq.h"
#include "cabinet_db.h"

#include "TcpServerMonitor.h"
#include "XmlProcAPI.h"
#if defined(AWH_FORMATION_SYS)
#include "CANRecvTask.h"
#include "CANSendTask.h"
#endif

using namespace std;
using namespace zsummer::log4z;

#define CONFIG_FILE_NAME 					"Power.conf"
#define LOG_FILE_NAME 						"config.cfg"

#define BUILD_NOMBER						1
static int SW_VER_MAJOR  = 0;
static int SW_VER_MINOR  = 0;
static int SW_VER_NUMBER = 0;

int main(int argc, char * args[])
{
	// 项目类别
#if defined(ZHONGYUAN_PRJ)
	SW_VER_MAJOR = 0x30;
#elif defined(ZHONGHANG_PRJ)
	SW_VER_MAJOR = 0x20;
#else
	SW_VER_MAJOR = 0x20;
#endif

#if defined(AWH_GRADING_SYS)		//分容系统
	SW_VER_MAJOR |= 0x02;
#elif defined(AWH_FORMATION_SYS)	//化成系统
	SW_VER_MAJOR |= 0x01;
#else
	SW_VER_MAJOR |= 0x00;			//预充系统
#endif

	// 校准工装
#if defined(CABTOOL_64CH)
	SW_VER_MINOR |= 0x1000;
#endif
	// 上位机协议
#if defined(ZHONGYUAN_BTS)
	SW_VER_MINOR |= 0x0100;
	// 中原项目数据库
#elif defined(ZHONGYUAN_DB)
	SW_VER_MINOR |= 0x0200;
#endif
	// 下位机协议
#if defined(PROTOCOL_5V160A)
	SW_VER_MINOR |= 0x0030;
#elif defined(PROTOCOL_5V80A)
	SW_VER_MINOR |= 0x0020;
#else
	SW_VER_MINOR |= 0x0010;
#endif
	// PLC针床类别
#if defined(PLC_JS_VF)
	SW_VER_MINOR |= 0x0005;
#elif defined(PLC_JS_CT)
	SW_VER_MINOR |= 0x0004;
#elif defined(PLC_JYT)
	SW_VER_MINOR |= 0x0003;
#elif defined(SIEMENS_PLC)
	SW_VER_MINOR |= 0x0002;
#else
	SW_VER_MINOR |= 0x0000;
#endif

	// 附加功能
#if defined(CHANNEL_START_DELAY)
	SW_VER_NUMBER |= 0x1000;
#endif
#if defined(AUX_VOLTAGE_ACQ)
	SW_VER_NUMBER |= 0x0100;
#endif
#if defined(AUX_TEMPERATURE_ACQ)
	SW_VER_NUMBER |= 0x0010;
#endif
#if defined(ZHONGHANG_SYSTEM_PROTECTION)
	SW_VER_NUMBER |= 0x0001;
#endif

	// 将版本信息写入指定文件，用于升级小工具读取此信息。
	char base_path[128] ={'\0'};
	char fl_path[128] ={'\0'};
	char cmd[128];

	if (!(argc > 1))  return 0;

	sprintf(fl_path, "%s", args[1]);
	strncpy(base_path,fl_path,strlen(fl_path)-strlen("/conf"));
	
	sprintf(cmd, "echo %02X.%04X.%04X-%04d > %s/bin/version", SW_VER_MAJOR, SW_VER_MINOR, SW_VER_NUMBER, BUILD_NOMBER,base_path);
	system(cmd);

	sprintf(cmd, "%02X.%04X.%04X-%04d",SW_VER_MAJOR, SW_VER_MINOR, SW_VER_NUMBER, BUILD_NOMBER);
	set_mc_version(cmd);
	
	int ret = access(WORK_STEP_XML_PATH, 0);
	if (ret)
	{
		mkdir(WORK_STEP_XML_PATH, 0777);
	}
#if defined(ENABLE_RUNSTATUS_SAVE)
	ret = access(RUN_STATUS_XML_PATH, 0);
	if (ret)
	{
		mkdir(RUN_STATUS_XML_PATH, 0777);
	}
#endif
	ret = access(GLOBAL_PRO_XML_PATH, 0);
	if (ret)
	{
		mkdir(GLOBAL_PRO_XML_PATH, 0777);
	}
	sleep(1);

	Configuration::instance()->conf_dir_set(args[1]);
	
	try
	{
	    //日志
	    sprintf(fl_path, "%s/%s", args[1], LOG_FILE_NAME);
        ILog4zManager::getRef().config(fl_path);
        data_id_process = ILog4zManager::getRef().findLogger("ProcessData");
        data_id_stop = ILog4zManager::getRef().findLogger("StopData");
        cell_id_temp = ILog4zManager::getRef().findLogger("TempData");

        ILog4zManager::getRef().start();

		LOGFMTI("=============== BFG software version:%02X.%04X.%04X-%04d ===============", SW_VER_MAJOR, SW_VER_MINOR, SW_VER_NUMBER, BUILD_NOMBER);

        //读取配置文件
        sprintf(fl_path, "%s/%s", args[1], CONFIG_FILE_NAME);
        Configuration::instance()->read_config_file(fl_path);

        // 获取电源柜名
    	FormatConvert::instance().set_line_no(Configuration::instance()->line_no());
    	FormatConvert::instance().set_cabinet_no(Configuration::instance()->cabinet_no());

		set_app_start_time();
		Configuration::instance()->plc_enable_config();
		Configuration::instance()->vacuum_enable_config();
		Configuration::instance()->tempAcq_enable_config();
		Configuration::instance()->cellTemp_contrast_config();
		Configuration::instance()->cellTemp_range_config();
		
		g_TcpServer.init();
		thread cli_monitor_t = g_TcpServer.run();

		// 辅助温度采集盒
#ifdef AUX_TEMPERATURE_ACQ
		g_AuxTempAcq.init();
		thread auxTempAcq_t = g_AuxTempAcq.run();
#endif

#if defined(AWH_FORMATION_SYS)
		for(int i = 0; i < CAN_PORT_NUM; i++)
		{
			int bitrate = 500000;
			char dev_name[64];

			if(Configuration::instance()->can_bitrate_config(i+1,bitrate))
			{
				sprintf(dev_name, "can%d", i);
				g_can_dev[i].can_init(dev_name,bitrate);
			}
		}

		thread can_recv_t = CANRecvTask().run();
#endif
    	// 库位线程
    	Cell* cells[MAX_CELLS_NBR];
    	thread cells_t[MAX_CELLS_NBR];
    	for (int i=0; i<MAX_CELLS_NBR; i++)
    	{
#if defined(AWH_FORMATION_SYS)
    		int can_port = 0;
			
    		Configuration::instance()->can_cell_config(i+1,can_port);
    		cells[i] = new Cell(i+1,can_port);
#else
			cells[i] = new Cell(i+1);
#endif
            cells_t[i] = cells[i]->run();
    	}

#if defined(AWH_FORMATION_SYS)
		CANSendTask can0_SendTask(0);
		CANSendTask can1_SendTask(1);
		thread can0_send_t = can0_SendTask.run();
		thread can1_send_t = can1_SendTask.run();
#endif		
		// 针床PLC
#if defined(ENABLE_PLC)
		thread pins_t = PinsWorker().run();
#endif

		// 辅助电压采集盒
#ifdef AUX_VOLTAGE_ACQ
		AuxVolAcq auxVolAcqs[MAX_CELLS_NBR];
		thread    auxVolAcq_t[MAX_CELLS_NBR];

		for (int i=0; i<MAX_CELLS_NBR; i++)
		{
    		auxVolAcqs[i]  = AuxVolAcq(i + 1);
			auxVolAcq_t[i] = auxVolAcqs[i].run();
		}
#endif

#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
		thread cabMeter_t = CabinetMeter().run();
#endif
		// 与上位机的心跳
#ifdef OFFLINE_PROTECTION
		thread cabHeartbeat_t = CabinetHeartbeat().run();
#endif
		// 数据库线程
#ifdef STOREDATA
		thread cabinetDB_t = g_CabDBServer.run();
#endif
		// 机柜线程(在主线程中运行任务循环)
        Cabinet cab;
        cab.run();

		// 线程阻塞，等待结束退出
#if defined(ENABLE_PLC)
		pins_t.join();
#endif

#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
		cabMeter_t.join();
#endif

#if defined(AWH_FORMATION_SYS)
		can_recv_t.join();
		can0_send_t.join();
		can1_send_t.join();
#endif

#ifdef AUX_VOLTAGE_ACQ
        for (auto& t: auxVolAcq_t)
        {
            t.join();
        }
#endif

#ifdef AUX_TEMPERATURE_ACQ
		auxTempAcq_t.join();
#endif

	cli_monitor_t.join();

    for (auto& t: cells_t)
    {
        t.join();
    }

   	for (int i=0; i<MAX_CELLS_NBR; i++)
	{
		if(cells[i])
		{
			delete cells[i];
		}	
	}
		
#if defined(ENABLE_RUNSTATUS_SAVE)
		sys_mutex_free();
#endif
		
#ifdef STOREDATA
		cabinetDB_t.join();
#endif
		
#ifdef OFFLINE_PROTECTION
		cabHeartbeat_t.join();
#endif		
	}
	catch(bad_alloc &exp)
	{
		LOGFMTE("new failed, description:%s", exp.what());
	}
	catch (int &e)
	{
		LOGFMTE("Exception Code: %d", e);
	}

    ILog4zManager::getRef().stop();

	return 0;
}


