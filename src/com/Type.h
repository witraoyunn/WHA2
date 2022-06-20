
#ifndef TYPE_H
#define TYPE_H

#include <string>
#include "utility.h"
#include "Configuration.h"

#include "ChannelProto_JN9504.h"


//命令及返回
typedef enum
{
#ifndef PROTOCOL_5V160A	
	/* ---------------------------充放电机下位机指令------------------------- */
    CMD_SET_PROTECTION = 0x01,    		// 保护设置功能
    ACK_SET_PROTECTION = 0x02,       	// 保护设置响应

    CMD_QUERY_PROTECTION = 0x03,    	// 查询保护设置
    ACK_QUERY_PROTECTION = 0x04,    	// 查询保护回复

    ACK_TRIG_PROTECTION = 0x05,    		// 保护触发

    CMD_SET_SAMPLE = 0x06,    			// 采样条件设置
    ACK_SET_SAMPLE = 0x07,    			// 采样设置响应

    CMD_QUERY_PROCESS_DATA = 0x08,  	// 记录数据查询
    ACK_QUERY_PROCESS_DATA = 0x09,  	// 记录数据查询回复

    CMD_SET_STEP = 0x0A,        		// 工步设置指令
    ACK_SET_STEP = 0x0B,        		// 工步设置响应

    CMD_RUN_CONTROL = 0x0C,        		// 运行状态设置
    ACK_RUN_CONTROL = 0x0D,        		// 运行设置回复

    CMD_QUERY_VERSION = 0x0E,        	// 软件版本查询
    ACK_QUERY_VERSION = 0x0F,        	// 软件版本回复

    CMD_QUERY_ADDRESS = 0x10,        	// 地址查询
    ACK_QUERY_ADDRESS = 0x11,        	// 地址查询回复

    CMD_SET_SYS_TIME = 0x12,        	// 系统时间设定
    ACK_SET_SYS_TIME = 0x13,        	// 系统时间设置回复

    CMD_CLEAR_WARNING = 0x14,        	// 清除警告命令
    ACK_CLEAR_WARNING = 0x15,        	// 清除警告命令回复

    CMD_SET_PARALLEL = 0x16,        	// 并机模式设定
    ACK_SET_PARALLEL = 0x17,        	// 并机模式设定回复

    CMD_QUERY_AUTO_RUN = 0x18,        	// 断电自运行查询
    ACK_QUERY_AUTO_RUN = 0x19,        	// 断线自运行查询回复

    CMD_SET_AUTO_RUN = 0x1A,        	// 断电自运行设定
    ACK_SET_AUTO_RUN = 0x1B,        	// 断电自运行设定回复

    CMD_SET_WAVE_SHARP= 0x1C,        	// 波动突变值/设定
    ACK_SET_WAVE_SHARP = 0x1D,        	// 波动值设定回复

    CMD_QUERY_WAVE_SHARP = 0x1E,        // 波动值查询
    ACK_QUERY_WAVE_SHARP = 0x1F,        // 波动值查询回复

	CMD_SET_CALIBRATION = 0x20,			// 老化校准设置
	ACK_SET_CALIBRATION = 0x21,     	// 老化校准设置回复

	CMD_QUERY_CALIBRATION = 0x22,   	// 老化校准设置查询
	ACK_QUERY_CALIBRATION = 0x23,    	// 老化校准查询回复

	CMD_CHECK_PROTECTION = 0x24,   		// 预充电检测保护
    ACK_CHECK_PROTECTION = 0x25,   		// 预充电检测返回

    CMD_BEAT_TRIGGER = 0x26,   			// 心跳检测
    ACK_BEAT_TRIGGER = 0x27,   			// 心跳检测返回

    ACK_STEP_TRIG_PROTECTION = 0x28,    // 单工步保护触发
    
    CMD_SET_PROTECTION_MERGE = 0x2A,   	// 全局保护设置合并指令
    ACK_SET_PROTECTION_MERGE = 0x2B,   	// 全局保护设置合并指令返回
#endif
	/* ---------------------------上位机非控制类指令 ----------------------- */
	CMD_SET_BATTERY_BARCODE		= 0x30,			// 设置电池条码
	ACK_SET_BATTERY_BARCODE		= 0x31,

	CMD_SET_AUX_PARAM			= 0x32,			// 设置辅助类参数
	ACK_SET_AUX_PARAM			= 0x33,

	CMD_POST_STATUS_INFO		= 0x34,			// 推送状态信息
	ACK_POST_STATUS_INFO		= 0x35,
	
	CMD_SET_TECH_INFO			= 0x36,			// 设置工艺信息
	ACK_SET_TECH_INFO			= 0x37,
	
	/* --------------------------- 上位机与中位机之间的指令 ---------------- */
    CMD_CONNECT     			= 0x50,         // 连接
    ACK_CONNECTED   			= 0x51,

    CMD_DIS_CONNECT   			= 0x52,       	// 断开连接
    ACK_DIS_CONNECTED 			= 0x53,

    ENABLE_RECORD 				= 0x54,       	// 开始上传记录数据
    DISABLE_RECORD 				= 0x55,      	// 停止上传记录数据

    CMD_AUTO_TECH_CTRL     		= 0x56,         // 自动化工艺控制
    ACK_AUTO_TECH_CTRL   		= 0x57,

	CMD_FIRE_PROC_SET     		= 0x58,         // 消防保护逻辑配置
    ACK_FIRE_PROC_SET   		= 0x59,

	CMD_QUERY_MC_VERSION		= 0x5A,			// 查询中下位机版本
	ACK_QUERY_MC_VERSION		= 0x5B,
	
	CMD_QUERY_ALARM_INFO		= 0x5C,			// 查询告警数据

	/* ---------------------------针床指令----------------------------- */
    CELL_PINS_STATUS          	= 0x60, 		// 针床状态信息
    CELL_PROBE_EXTEND          	= 0x61, 		// 探针压合
    CELL_PROBE_RETRACT         	= 0x62, 		// 探针脱开
    CELL_OPEN_HIGH_VACUUM      	= 0x63, 		// 打开高真空
    CELL_CLOSE_HIGHT_VACUUM    	= 0x64, 		// 关闭高真空
    CELL_OPEN_LOW_VACUUM       	= 0x65, 		// 打开低真空
    CELL_CLOSE_LOW_VACUUM      	= 0x66, 		// 关闭低真空
    CELL_OPEN_DRAIN_VACUUM     	= 0x67, 		// 打开泄真空
    CELL_CLOSE_DRAIN_VACUUM    	= 0x68, 		// 关闭泄真空
    CELL_SET_VACUUM_PARAM      	= 0x69, 		// 设置真空值
    CELL_START_VACUUM          	= 0x6A, 		// 启动抽真空
    CELL_STOP_VACUUM           	= 0x6B, 		// 停止抽真空
    CELL_CLEAR_WARNNING        	= 0x6C, 		// 清除报警
    CELL_SET_TEMP_THRESHOLD    	= 0x6D, 		// 设置阈值 (电池周边环境温度设定)
    CELL_POWER_ON              	= 0x6E, 		// 工装取电开
    CELL_POWER_OFF             	= 0x6F, 		// 工装取电关
    CELL_CONSTANT_TEMP_SET     	= 0x80, 		// 恒温设置
    CELL_DEBUG_START_VACUUM    	= 0x82, 		// 单点调试 启动抽真空
    CELL_DEBUG_READ_VACUUM     	= 0x83, 		// 单点调试 读取负压值
    CELL_SET_AUTO_MODE     	    = 0x85, 		// 自动模式设置
    CELL_SET_MANUAL_MODE     	= 0x86, 		// 手动模式设置
    CELL_OPEN_FIRE_FAN     	    = 0x87, 		// 打开消防风扇（A级消防）
    CELL_CLOSE_FIRE_FAN     	= 0x88, 		// 关闭消防风扇
    CELL_OPEN_FIRE_DOR     	    = 0x89, 		// 打开消防门
    CELL_CLOSE_FIRE_DOR     	= 0x8A, 		// 关闭消防门
    CELL_CLEAN_SMOKE_ALARM     	= 0x8B, 		// 消除烟雾告警
    CELL_OPEN_FIRE_SPRAY    	= 0x8C, 		// 打开消防喷淋（A级消防）
    CELL_CLOSE_FIRE_SPRAY     	= 0x8D, 		// 关闭消防喷淋

    DCR_ACK_SCAN_RESULT         = 0x70,
    DCR_CMD_PREPARE_INSPECT     = 0x71,
    DCR_ACK_PREPARE_FINISHED    = 0x72,
    DCR_ACK_REQUEST_EXIT        = 0x73,
    DCR_CMD_ALLOW_EXIT          = 0x74,
    DCR_CMD_RESET_STATE         = 0x75,

#if defined(AWH_PRECHARGE_SYS)
	CELL_COOL_FAN_ON            =  0x90,        //冷风扇开
	CELL_COOL_FAN_OFF           =  0x91,        //冷风扇关
	CELL_AIR_FAN_ON             =  0x92,        //抽风机开
	CELL_AIR_FAN_OFF            =  0x93,        //抽风机关
	CELL_LOAD                   =  0x94,        //初始化操作
	CELL_AUTO_START             =  0x95,        //自启动操作
	CELL_STOP                   =  0x96,        //停止操作
	CELL_TEMP_SET               =  0x97,        //设置库位温度告警
	CELL_SET_TARY_STATUS        =  0x98,        //反馈托盘信息
    CELL_TARY_OUT               =  0x99,        //托盘出库
    
	PINS_STATUS_QUERY			= 0xB0,			// 针床状态查询
	FIRE_EMERGENCY_BRAKE		= 0xB2,			// 消防紧急制动
	PINS_ALARM_QUERY			= 0xB4,			// 针床告警查询
#elif (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS)) 
	CELL_TRAY_UP				= 0x90,			//托盘上升
	CELL_TRAY_DOWN				= 0x91,			//托盘下降
	CELL_LEAK_RATE_TEST_MANUAL	= 0x92,			//漏率测试
	CELL_CALIBRATE_MODE			= 0x93,			//校准模式
	CELL_FIRE_DOOR_UP			= 0x94,			//消防门上升
	CELL_FIRE_DOOR_DOWN			= 0x95,			//消防门下降
	CELL_BLOCK_TEST_MANUAL		= 0x96,			//手动模式堵塞测试
	CELL_RED_LIGHT				= 0x97,			//系统红灯
	CELL_YELLOW_LIGHT			= 0x98,			//系统黄灯
	CELL_GREEN_LIGHT			= 0x99,			//系统绿灯
	CELL_BUZZER					= 0x9A,			//蜂鸣器
	CELL_OPT1 					= 0x9B,			//库位OPT1
	CELL_OPT2					= 0x9C,			//库位OPT2
	PLC_REG_READ				= 0x9D,			// PLC寄存器读
	PLC_REG_WRITE				= 0x9E,			// PLC寄存器写
	CELL_VAC_PARA_SET			= 0x9F,			//负压参数设置
	CELL_BLOCK_TEST_AUTO		= 0xA0,			//自动模式堵塞测试
	CELL_LEAK_RATE_TEST_AUTO	= 0xA1,			//自动模式漏率测试
	CELL_START_VACUUM_AUTO		= 0xA2,			// 抽真空
	CELL_DRAIN_VACUUM_AUTO		= 0xA3,			// 泄真空

	PINS_STATUS_QUERY			= 0xB0,			// 针床状态查询
	PLC_SW_VERSION				= 0xB1,			// PLC软件版本
	FIRE_EMERGENCY_BRAKE		= 0xB2,			// 消防紧急制动
	PINS_ALARM_SYNC				= 0xB3,			// 针床告警同步
	PINS_ALARM_QUERY			= 0xB4,			// 针床告警查询
	CELL_VACUUM_TEST_CLEAR		= 0xB5,			// 堵塞保压测试结果清除
#endif
	CMD_TRANSMIT_ENABLE			= 0xC0,			//使能透传
	CMD_TRANSMIT_DATA			= 0xC1,			//透传数据
	CMD_TRANSMIT_DISABLE		= 0xC2,			//禁用透传

	SYS_ALARM_CODE_REPORT		= 0xC3,			// 告警码上报
	/* ---------------------------------------------------------------- */	
	SYS_COMMON_CMD 				= 0xF0,			// 系统通用命令，仅与中位机有关
	SYS_DO_PROTECT				= 0xF1, 		// 系统保护，仅与中位机有关

 	BF_PLC_CMD   				= 0xD0,			// 化成PLC命令
 	BF_PLC_NOTIFY  				= 0xD1,			// 化成PLC通知 	
 	BG_PLC_CMD   				= 0xD2,			// 分容PLC命令
 	BG_PLC_NOTIFY  				= 0xD3,			// 分容PLC通知
} Function_Code_e;

typedef enum {
    SUB_CMD_SYSTEM_INFO         = 0x01,         // 系统信息
    SUB_CMD_SYSTEM_FAULT        = 0x02,         // 告警信息
} SystemGet_SubCode_Enum;


#define GLOBAL_PROTECTION_ITEM_TOTAL		0x18		// 全局保护子命令码总数
#define PREB_CALIBRATION_ITEM_TOTAL			0x12		// 老化校准子命令码总数

//全局保护类型
/*
typedef enum
{
    GLOBAL_OVP = 0x01,      	// 充电过压保护
    GLOBAL_OCP = 0x02,      	// 充电过流保护
    GLOBAL_UVP = 0x03,      	// 放电欠压保护
    GLOBAL_UCP = 0x04,      	// 放电过流保护
    GLOBAL_OCAP = 0x05,     	// 充电过容保护
    GLOBAL_DC_OL = 0x06,    	// 直流输出限流保护
    GLOBAL_OTP = 0x07,      	// 电池温度上限保护，变更为电池过温告警 (消防：电池超温)
    GLOBAL_REVERSE = 0x08,  	// 反接保护
    GLOBAL_ISLAND_ERR = 0x09,	// 孤岛效应保护故障
    GLOBAL_OUT_ERR = 0x0A,		// 输出端短路故障
    GLOBAL_SHORT_ERR = 0x0B, 	// 电池短路故障
    GLOBAL_FAN_ERR = 0x0C,   	// 风扇故障
    GLOBAL_BUS_ERR = 0x0D,  	// 输入BUS高故障
    GLOBAL_SVP = 0x0E,      	// 电压波动保护
    GLOBAL_SCP = 0x10,      	// 电流波动保护
    GLOBAL_CONNECTOR_ERR = 0x12,// 输出端子接插故障
    GLOBAL_UTP = 0x13,     		// 电池温度下限保护值
    GLOBAL_UCAP = 0x14,     	// 容量过放保护
    GLOBAL_CRP = 0x15,      	// 接触阻抗保护
    GLOBAL_OCDP = 0x16,     	// 电流超差保护
    GLOBAL_OVDP = 0x17,     	// 电压超差保护
    GLOBAL_LRP = 0x18,      	// 回路阻抗保护
    GLOBAL_POVP = 0x19,     	// 电源过压保护
    GLOBAL_POW_OTP = 0x20,   	// 电源模块过温保护
    GLOBAL_WD_RESET = 0x2A, 	// DSP看门狗复位告警
    GLOBAL_BAT_OTW = 0x2B, 		// 电池过温预警 (消防：电池过温)
    GLOBAL_BAT_VOTP = 0x2C, 	// 消防：电池超高温
} GLOBAL_PROTECTION_ENUM;
*/

//电源物理量
#ifndef PROTOCOL_5V160A	
typedef enum
{
    PARAM_T = 0x1,
    PARAM_V = 0x2,
    PARAM_I = 0x3,
    PARAM_R = 0x4,
    PARAM_P = 0x5,
    PARAM_C = 0x7,
    PARAM_VAC = 0x8,
    PARAM_INVALID = 0xFF
} PARAM_TYPE_ENUM;
#else
typedef enum
{
    PARAM_T = 0x1,
    PARAM_V = 0x2,
    PARAM_I = 0x3,
    PARAM_C = 0x4,
    PARAM_E = 0x5,

    PARAM_INVALID = 0xFF
} PARAM_TYPE_ENUM;
#endif

//工步类型
#ifndef PROTOCOL_5V160A	
typedef enum
{
    STEP_AGING           = 0x1,
    STEP_CHARGE_CC       = 0x2,
    STEP_CHARGE_CV       = 0x3,
    STEP_CHARGE_CC_CV    = 0x4,
    STEP_DISCHARGE_CC    = 0x5,
    STEP_DISCHARGE_CV    = 0x6,
    STEP_DISCHARGE_CW    = 0x8,
    STEP_LOOP_START      = 0x9,
    STEP_LOOP_STOP       = 0xA,
    STEP_DISCHARGE_CC_CV = 0xB,
    STEP_VAC             = 0xC,
    STEP_END             = 0XD,
    STEP_DISCHARGE_CP	 = 0xE,
	STEP_INVALID         = 0xFF
} Step_Type_e;
#endif

//工步状态
typedef enum
{
	STEP_STOP		= 0x00, 	// 停止指令，记录数据无此状态
	STEP_PAUSE		= 0x01, 	// 暂停指令/记录数据的运行暂停状态
	STEP_PROCESS	= 0x02, 	// 工步开始指令/记录数据的运行状态
//	STEP_FAULT		= 0x03, 	// 错误状态
	STEP_FINISH		= 0x04, 	// 工步完成 (配方完成时的最后一帧数据)
//	STEP_START		= 0x05,		// 单步开始指令
//	STEP_CHANGE		= 0x06,		// 工步变化
	STEP_TEST		= 0x07,		// 接触检测中，此时也在充放电
	STEP_STANDBY	= 0x08,		// 待机状态
	STEP_ALARM  	= 0x09,		// 告警状态，清除告警后变为待机状态
	STEP_BEGIN      = 0x0A,	    // 工艺流程开始 (配方启动时的第一帧数据)
	STEP_VACNEXT  	= 0x0B,		// 負壓工步結束運行下一工步
} Step_Status_e;

typedef enum
{
    PINS_VAC_TAP 				= 0x01,			// 高低真空阀门选择
    PINS_PUMP_VAC_TIME 			= 0x02,			// 抽真空总时间
    PINS_OPEN_VAC_TIME 			= 0x03,			// 抽真空开时间
    PINS_CLOSE_VAC_TIME 		= 0x04,			// 抽真空关时间
    PINS_VAC_VALUE 				= 0x05,			// 最后目标真空值
    PINS_KEEP_VAC_TIME 			= 0x06,			// 保压时间 / 漏率测试时间
    PINS_KEEP_VAC_VALUE 		= 0x07,			// 保压值 / 漏率负压值
    PINS_VAC_TOL 				= 0x08,			// 保压容错值 / 漏率负压容错值
    PINS_LEAK_OPEN_VAC_TIME  	= 0x09,			// 泄真空开时间
    PINS_LEAK_CLOSE_VAC_TIME	= 0x0A,			// 泄真空关时间
    PINS_LEAK_RATIO 			= 0x0B,			// 保压漏率   / 泄漏率
    PINS_TEMP_THRESH_HOLD 		= 0x0C,			// 热电偶设定值 (改为电池周边环境温度阀值)
    PINS_CONSTANT_TEMP 			= 0x0D,			// 恒温控制值
    PINS_CONSTANT_TEMP_DIV 		= 0x0E,			// 恒温偏差值
    PINS_NPRESSURE_TOL			= 0x0F,			// 负压容错值
    PINS_VAC_TYPE_SELECT 		= 0x10,			// 抽泄真空类型选择
    PINS_NPRESSURE_VALUE 		= 0x11,			// 负压读取值	 					
    PINS_POWER_TEMP_THRESH 		= 0x12,			// 电源柜环境温度值	 	
    PINS_HVAC_VALUE_MAX			= 0x13,			// 高真空范围最大值   
    PINS_HVAC_OPEN_TIME 		= 0x14,			// 高真空抽气时间
    PINS_HVAC_CLOSE_TIME 		= 0x15,			// 高真空停顿时间
    PINS_HVAC_MODE 				= 0x16,			// 高真空真空模式
    PINS_LVAC_VALUE_MAX			= 0x17,			// 低真空范围最大值     
    PINS_LVAC_OPEN_TIME 		= 0x18,			// 低真空抽气时间
    PINS_LVAC_CLOSE_TIME 		= 0x19,			// 低真空停顿时间
    PINS_LVAC_MODE 				= 0x1A,			// 低真空真空模式
	PINS_VAC_PARAM_TYPE 		= 0x1B,			// 参数类型：抽真空、泄真空、漏率
    PINS_VAC_WAVE_VALUE			= 0x1C,			// 负压波动保护值
    PINS_VAC_WAVE_TIME			= 0x1D,			// 负压波动保护时间（检测窗口）

	PINS_PARAM_MAX				= 0x20
} PINS_PARAM_TYPE_ENUM;

typedef enum
{
	PINS_TECH_CLEAN 		= 0,				// 清除
	PINS_TECH_BATTERY 		= 1,				// 电池
	PINS_TECH_VAC     		= 2,				// 负压工装
	PINS_TECH_PT1000  		= 5,				// PT1000工装
	PINS_TECH_CALIBRATION 	= 7,				// 校准工装
} PINS_TECH_TYPE_ENUM;

typedef enum
{
	SYS_COM_VER_GET				= 1,			// 获取中位机软件版本信息
	SYS_CELL_ALARM_HANDLE		= 2,			// 需要中位机处理的告警
	SYS_PINS_STATUS_SYNC		= 3,			// 针床状态同步
	SYS_PROBE_STATUS_SYNC		= 4,			// 针床压合状态同步
	SYS_AUX_BAT_TEMP_SYNC		= 5,			// 电池温度同步
	SYS_CELL_CH_EN_SYNC			= 6,			// 库位通道使能状态同步
	SYS_CELL_CHARGING_SYNC		= 7,			// 库位充放电状态同步
	SYS_CELL_VAC_NEXT           = 8,			// 多段负压工步指令
	SYS_PINS_ALARM_CLEAR        = 9,			// 针床告警清除指令

	SYS_AUTO_TECH_START         = 11,		    // 自动化工艺启动
	SYS_AUTO_TECH_ABORT         = 12,		    // 自动化工艺终止
	SYS_VAC_FINISH           	= 13,		    // 针床保压完成
	SYS_POW_STEP_FINISH       	= 14,		    // 电源工步完成
	SYS_PINS_TECH_FINISH       	= 15,		    // 针床自动化流程结束
	SYS_PINS_PROBE_RETRACT      = 16,		    // 弹开针床
	SYS_SET_VAC_LEAK_TEST    	= 17,		    // 设置泄漏率测试使能
	SYS_TECH_ACQ_FINISH    		= 18,		    // 工装采集完成
	SYS_TECH_TEST_FINISH    	= 19,		    // 工装测试完成
	SYS_AUTO_TECH_INIT          = 20,		    // 自动化工艺初始化
	SYS_AUTO_TECH_START_FAIL    = 21,		    // 自动化工艺初始化
	SYS_VAC_SET                 = 22,           // 多段負壓工藝負壓值

	SYS_BLOCK_TEST_ACK			= 23,			// 堵塞测试结果
	SYS_LEAK_TEST_ACK			= 24,			// 保压测试结果
	SYS_PUMP_VAC_ACK			= 25,			// 抽真空结果
	SYS_LEAK_VAC_ACK			= 26,			// 泄真空结果
	SYS_TRAY_STATE_SET			= 27,			// 托盘状态设置
} Sub_Func_Sys_Common_e;

typedef enum 
{
	BF_PLC_START_CMD		  = 1,		// 化成上料开始
	BF_SCAN_CODE_NOTIFY 	  = 2,		// 化成PLC扫码比对结果 通知>>MES
	BF_WMS_PUT_INTO_NOTIFY	  = 3,		// 物流系统入库 通知>>中位机
	BF_WMS_TAKE_OUT_PALLET	  = 4,		// 物流系统取走空托盘 通知>>WMS
	BF_COLD_COMP_STARTING	  = 5,		// 化成冷压启动 通知>>MES
	BF_COLD_COMP_CMPLETED	  = 6,		// 化成冷压结束 通知>>MES
	BF_WMS_TAKE_OUT_READY	  = 7,		// 化成下料准备就绪 通知>>WMS
	BF_WMS_PALLET_READY	      = 8,		// 物流系统空托盘到达 通知>>中位机
} BF_SUB_FUN_CODE_ENUM;

typedef enum 
{
	BG_PLC_START_CMD		  = 1,		// 分容开始
	BG_PLC_PUT_INTO_NOTIFY	  = 2,		// 分容PLC库位入料状态 通知>>WMS
	BG_WMS_PUT_INTO_NOTIFY	  = 3,		// 物流系统入库 通知>>中位机
	BG_PLC_CELL_PALLET_READY  = 4,		// 分容PLC托盘到位 通知>>WMS
	BG_PLC_PRESS_ETAB_CMD	  = 5,		// 分容PLC夹极耳
	BG_PLC_CELL_START_READY   = 6,		// 分容PLC库位启动准备OK 通知>>MES
	BG_PLC_RELEASE_ETAB_CMD   = 7,		// 分容PLC松开极耳
	BG_PLC_ETAB_STATUS_NOTIFY = 8,		// 分容PLC极耳松开 通知>>MES
	BG_WMS_TAKE_OUT_NOTIFY	  = 9,		// 物流系统出库 通知>>WMS
} BG_SUB_FUN_CODE_ENUM;

typedef struct _recvData_t
{
	unsigned short len;
	unsigned char data[512];	
}recvData_t;


#ifndef PROTOCOL_5V160A
	//#pragma pack(1)
	typedef struct
	{
		bool   is_stop;					// 截止数据标志
		int    cell_no;
		int    ch_no;
		
		char   bat_type[128];			// 电芯类型
		char   stepConf_no[128];		// 工步配方的编号
		char   pallet_barcode[128];		// 托盘码
		char   bat_barcode[128];		// 电芯码
	    uint8  run_state;
	    uint32 run_time;
	    float  voltage;			// 电池电压
	    float  current;
	    float  capacity;
	    float  temperature;
	    int    step_no;
	    uint8  step_type;
	    uint16 loop_no ;
	    long   timestamp;
	    uint16 sum_step;
		float  energy;
		float  ratio;           // 横流比
		float  pvol;            // 功率线电压
		float  t_sink;			// 辅助电压
		float  v_cotact;		// 探针电压
		float  r_cotact;		// 接触电阻
		float  r_output;		// 输出回路电阻
		uint16 loop_times;		// 当前循环次数
		int    error_code;		// 当前发生的故障码
		float  vac_val;			// 当前负压值
		float  cell_temper[6];  // 库位温度
	} Step_Process_Data_t;
#endif


#if defined(AWH_PRECHARGE_SYS)

typedef struct Fx_Error_Code
{
	bool PressureError;
	bool EStop;
	bool PLCHeartBeat;
	bool DoorOpen;
	bool MoveUpTimeout;
	bool MoveDownTimeout;
	bool TrayReverse;
	bool NoTray;
	bool TrayNotReady;
	bool FanOverload;
	bool RollerOverload;
	bool RollerRunTimeout;
	bool RollerReverseRunTimeout;
	bool LeftFanError;
	bool RightFanError;
	bool TopFanError;
	bool Smoke1Alarm;
	bool Smoke2Alarm;
	bool CabSmokeAlarm;
	bool PinSmokeAlarm;
	bool PinSmokeHeavyAlarm;
	bool HighTempAlarm;
} Fx_Error_Code_t;

#endif

#if 0
//#pragma pack(1)
typedef struct
{
    int      cabinet;
    int      cell;
#if defined(PLC_JS_VF)
	float    temp[2];
#elif defined(PLC_JYT)
	float    temp[8];
	float    c_temp;
	int      step_no;
#else
	float    temp[7];
#endif
	float    vacuum;
#ifdef PLC_JYT
	uint16_t err_code[5];
#else
    uint32_t err_code;					// 错误码
#endif
	bool     is_auto_mode;				// 自动模式
	bool     is_cell_empty;				// 托盘空
	bool     is_extend;					// 针床压合
	bool     is_retract;				// 针床脱开
	bool     is_smoke_alarm;			// 烟雾告警
#ifdef PLC_JS_VF
	bool     is_HVAC_open;				// 高真空阀开
	bool     is_LVAC_open;				// 低真空阀开
	bool     is_leak_VAC_open;			// 泄真空阀开
	bool     is_emergency_stop;			// 急停
	bool     is_palletizer_entry;		// 码垛机进入针床
	bool     is_temp_fan_open;			// 均温风机打开
	bool     is_smoke_fan_open;			// 排烟风机打开
	bool     is_fire_dor_open;			// 消防门打开
	bool     is_power_open;				// 工装取电打开
	bool     is_sys_vac_normal;		    // 系统气压正常
	bool     is_fire_fan_open;			// 消防风扇打开
	bool     is_leakage_ok;             // 漏率正常
	bool     is_plc_connect;            // plc在线 
#endif
#ifdef PLC_JS_CT
	int      smokeStatus;				// 烟感状态
#endif
} Pins_Status_Data_t;
#endif

#if defined(AWH_PRECHARGE_SYS)
typedef struct
{
    int      cell;
	Fx_Error_Code_t err_code;
	float temp[CELL_TEMP_ACQ_NUM];
	float temp_warning;
	bool  is_auto_mode;				    // 自动模式
	bool  is_extend;					// 针床压合
	bool  is_retract;				    // 针床脱开
	bool  is_tool_power;                //工装取电状态
	bool  is_cool_fan;                  //冷风扇状态
	bool  is_air_fan;                   //抽风机状态
	bool  is_loaded;                	//初始化完成
	bool  is_tray_ready;                //托盘在位状态
	bool  is_auto_start;                //自动启动状态
	bool  is_can_intray;                //允许入库
	bool  is_plc_connect;               // plc在线
	bool  is_tray_inready;              //入库到位通知 
	bool  is_tray_outready;             //托盘出库通知
} Pins_Status_Data_t;
#else
typedef struct
{
	float	temp[CELL_TEMP_ACQ_NUM];
	float	vacuum;

	int		warn_poweroff_delay;	// 告警断电延迟秒数
	int		emstop_poweroff_delay;	// 紧急停止断电延迟秒数
	int		blockTestResult;		// 堵塞测试
	float	keepVac_start;			// 开始保压压力值
	float	keepVac_end;			// 完成保压压力值
	float	leakRatioResult;		// 漏率结果
	int		keepVacTestResult;		// 保压测试
	bool	pins_extend;			// 针床压合到位
	bool	pins_unpress;			// 针床脱开到位
	bool    tray_in_place;			// 托盘在位
	bool	fire_door_open;			// 消防门是否开启
	bool    is_auto_mode;			// 是否自动模式
	bool    is_plc_connect;         // plc在线
	bool    is_cell_forbid;         // 库位禁用
	bool	alarm_ahead_smoke;		//库位前方烟雾告警
	bool	alarm_rear_smoke;		//库位后方烟雾告警
} Pins_Status_Data_t;
#endif

//command.
//#pragma pack(1)
#ifndef PROTOCOL_5V160A
typedef struct
{
    uint8 func_code;
    int value;
} Func_Data_t;

typedef struct
{
    Func_Data_t spec_data[3];
    Func_Data_t record_cond[3];

    uint8       stop1_type[3];
    Func_Data_t stop1_data[3];
	Func_Data_t stop1_result[3];
    uint8       stop1_level[3];

#ifdef ZHONGHANG_PRJ
	Func_Data_t protection[29];
#else
	Func_Data_t protection[27];
#endif
} Step_Param_t;

typedef struct
{
    Step_Param_t general_data;
	Func_Data_t  loop_data[5];
} Step_Data_t;

typedef struct
{
    uint16       step_nbr;
    Step_Type_e  step_type;
    Step_Data_t  step_data;
} Ch_Step_Data_t;

typedef struct
{
	Func_Data_t 	func_data;
    Func_Data_t 	subList[MAX_FUCN_LIST_SIZE];
	int      		subList_len;
    Ch_Step_Data_t	step_data;
} Channel_Data_t;
#endif


typedef struct
{
    uint8           sel_chans[MAX_CHANNELS_NBR];
    Channel_Data_t  data;
} Cmd_Pow_Data_t;

typedef struct
{
#ifdef PROTOCOL_5V160A
	uint8_t 		ch_no;
#endif
    uint8 			func_code;
    Channel_Data_t  data;
} Cmd_Channel_Msg_t;

#if defined(AWH_PRECHARGE_SYS)
typedef struct
{
    float vac_params[PINS_PARAM_MAX];

	float temp;
	int tray_info;
    //bool  chargeDischargeStatus;
	struct
	{
    	int code;
    	int value;
	} sub_func;
} Cmd_Pins_Data_t;
#elif (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
typedef struct
{
	Func_Data_t sub_params[PINS_PARAM_MAX];
} Cmd_Pins_Data_t;
#else
typedef struct
{
    float vac_params[PINS_PARAM_MAX];
	struct
	{
    	int code;
    	int value;
	} sub_func;
} Cmd_Pins_Data_t;
#endif


typedef struct
{
	int  sub_code;
	int  value;
	Pins_Status_Data_t  pins_status;
} System_Inside_Msg_t;

typedef struct
{
    uint8               func_code;
    int                 cell_no;
    Cmd_Pins_Data_t     pins_msg;
	System_Inside_Msg_t inside_msg;
} Cmd_Pins_Msg_t;

typedef struct
{
    int  code;
    int  value;
    int  step_no;
} Sub_Cmd_Msg_t;

typedef struct
{
    Cmd_Pow_Data_t   pow;
    Cmd_Pins_Data_t  pin;
	Sub_Cmd_Msg_t    sub_msg;
} Cmd_Cell_Data_t;

typedef struct
{
	char str[128];
} Battery_Barcode_t;

typedef struct
{
    uint8               func_code;
    Cmd_Cell_Data_t     cell_msg;
	Battery_Barcode_t   tech_info[4];
	Battery_Barcode_t   barcode[MAX_CHANNELS_NBR];
	System_Inside_Msg_t inside_msg;
} Cmd_Cell_Msg_t;

typedef struct
{
    int cell_no;	
	int fuc_code;
	struct
	{
    	int   sub_code;
    	float value;
	} data[4];
	int size;

	struct
	{
    	int     sub_code;
    	int64_t value;
	} status;
} Cmd_Aux_Param_t;

typedef struct
{	
    int        cell;
	struct tm  dt;
	float      vacs[MAX_CHANNELS_NBR];	
} Pins_Vac_ReportForms_t;

//typedef struct
//{
//	char station[16];					// 站点名称
//	char traySn[128];					// 托盘码
//	char cabinetNo[16];					// 柜号
//	char stockSn[128];					// 库位码
//	char stockNo[16];					// 库号
//	char operate[8];					// 入库/出库
//	char cells[MAX_CHANNELS_NBR][128];	// 电芯码
//	char date[32];						// 日期时间
//} BTS_Info_t;							// 四方通讯协议

//返回数据类型
typedef enum
{
	RESPONSE_CELLDATA  = 0x00,
	RESPONSE_RECORD    = 0x01,
	RESPONSE_EXCEPTION = 0x02,
	RESPONSE_REPLY     = 0x03,
	RESPONSE_TRANSMIT_REPLY = 0x04,
	RESPONSE_CELLREPLY = 0x05,
	RESPONSE_CELLALARM = 0x06,
	RESPONSE_CAB_METER = 0x07,		//20220210新增  电源柜电表数据上报
	RESPONSE_VERSION   = 0x08,		//20220422新增 版本查询应答
	RESPONSE_NOTIFY    = 0x10,
	RESPONSE_CHAN_LINK = 0x11,
} RESPONSE_TYPE_ENUM;

// 分容PLC
typedef struct
{
    float vac_params[PINS_PARAM_MAX];
    float temp_threshold;
} CMD_BFG_PLC_DATA_UNION;

typedef struct
{
    uint8                  func_code;
    int                    cell_no;
    CMD_BFG_PLC_DATA_UNION data;
} CMD_BFG_PLC_DATA_STRUCT;

// 物流系统WMS
typedef struct
{
    uint8                 func_code;
    int                   cell_no;
    bool                  status;
} CMD_WMS_DATA_STRUCT;

#endif //TYPE_H


