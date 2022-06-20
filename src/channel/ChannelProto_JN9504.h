#ifndef _CHANNELPROTO_JN9504_H_
#define _CHANNELPROTO_JN9504_H_
#include <stdint.h>
#include <string>
#include <vector>
#include "RingQueue.h"
#include "ChannelState.h"

#if (defined(AWH_FORMATION_SYS) || defined(AWH_PRECHARGE_SYS))
#define MODULE_CH_NUM				8		// 中航武汉 化成预充 一台下位机控8个通道
#else
#define MODULE_CH_NUM				4		// 中航武汉 分容 	一台下位机控4个通道
#endif

#define MAX_FUCN_LIST_SIZE 		200		//48

#ifdef PROTOCOL_5V160A
typedef enum
{
/* ---------------------------JN9504通信指令------------------------- */
//1全局保护
    CMD_SET_PROTECTION 			= 0x01,     // 保护设置功能
    ACK_SET_PROTECTION 			= 0x81,     // 保护设置响应

    CMD_QUERY_PROTECTION 		= 0x02,    	// 查询保护设置
    ACK_QUERY_PROTECTION 		= 0x82,    	// 查询保护回复

    ACK_TRIG_PROTECTION			= 0x83,    	// 保护触发
//2记录类
    CMD_QUERY_PROCESS_DATA		= 0x04,  	// 记录数据查询
    ACK_QUERY_PROCESS_DATA		= 0x84,  	// 记录数据查询回复
//3工步类
    CMD_SET_STEP 				= 0x05,     // 工步设置指令
    ACK_SET_STEP 				= 0x85,     // 工步设置响应

    CMD_RUN_CONTROL 			= 0x06,     // 工步运行状态设置
    ACK_RUN_CONTROL 			= 0x86,     // 工步运行状态设置回复

	CMD_STEP_INQ				= 0x07,		// 工步查询命令 
	ACK_STEP_INQ				= 0x87,		// 工步查询回复 

    ACK_STEP_TRIG_PROTECTION 	= 0x88,    	// 工步保护触发
//4校准类
	CMD_SET_CALIBRATION 		= 0x09,		// 老化校准设置
	ACK_SET_CALIBRATION 		= 0x89,     // 老化校准设置回复

	CMD_QUERY_CALIBRATION 		= 0x0A,   	// 老化校准设置查询
	ACK_QUERY_CALIBRATION 		= 0x8A,    	// 老化校准查询回复

	CMD_AD_DATA_INQ				= 0x0B,		// AD数据读取命令 
	CMD_AD_DATA_INQ_REPLY		= 0x8B,		// AD数据读取回复 
//5系统类
	CMD_SYSTEM_GET				= 0x0C, 	// 系统查询
	ACK_SYSTEM_GET				= 0x8C, 	// 系统查询回复

	CMD_SYSTEM_SET				= 0x0D, 	// 系统设置
	ACK_SYSTEM_SET				= 0x8D, 	// 系统设置回复

	CMD_SYSTEM_UPDATE			= 0x0E,		// 系统升级命令/回复
//6升级类
	CMD_ACDC_UPDATE 			= 0x60,		// AC/DC升级命令 
	CMD_ACDC_UPDATE_REPLY		= 0xE0,		// AC/DC升级回复 
//7内部保护
	CMD_SET_INTERNAL_PROTECTION	= 0x0F, 	// 内部保护设置
	ACK_SET_INTERNAL_PROTECTION	= 0x8F, 	// 内部保护响应

	CMD_QUERY_INTERNAL_PROTECTION = 0x10,	//内部保护查询
	ACK_QUERY_INTERNAL_PROTECTION = 0x90,	//内部保护查询护回复
	
	CMD_INTERNAL_TRIG_PROTECT 	= 0xA0,		// 内部保护触发
	CMD_ACDC_TRIG_PROTECT 		= 0xA1,		// ACDC保护触发
	
	//CMD_INTERNAL_PROTECT_INQ	= 0x10,		// 内部保护设置查询命令 ???
	//CMD_INTERNAL_PROTECT_TRIG	= 0xA0,		// 内部保护触发命令 
	//CMD_ACDC_PROTECT_TRIG		= 0xA1,		// AC/DC保护触发命令

	CMD_QUERY_TEMPERATURE		= 0x75,		//查询温度
	ACK_QUERY_TEMPERATURE		= 0xF5,		//温度查询回复
	
	CMD_INVALID					= 0xFF,		//无效命令
}JN9504_Function_Code_e;


//全局保护类型
typedef enum
{
    GLOBAL_OverVolPro 	= 0x01,     // 电池电压上限
    GLOBAL_UnderVolPro 	= 0x02,     // 电池电压下限
    //GLOBAL_LoopVolDiff = 0x03,     // 回路压差
    GLOBAL_C_OverCurPro = 0x03,     // 充电过流
    GLOBAL_D_OverCurPro = 0x04,     // 放电过流
    GLOBAL_OverCapPro	= 0x05,    	// 过容保护
    
    //GLOBAL_OverTotalCap = 0x07,     // 总容量过容
	GLOBAL_InitVolUpperPro = 0x06,  // 初始电压上限
    GLOBAL_InitVolLowerPro = 0x07,	// 初始电压下限
    GLOBAL_ContactTimePro  = 0x08,	// 接触检测时间
    GLOBAL_ContactCurPro = 0x09, 	// 接触检测电流
    GLOBAL_ContactVolPro = 0x0A,   	// 接触检测电压

	GLOBAL_OTHRE_ERR = 0xFE,	// 其他故障
    GLOBAL_INVALID 	 = 0xFF,  	// 无效
} JN9504_Global_Protection_e;

typedef enum
{    
    //STEP_DISCHARGE_CW    = 0x8,
    STEP_LOOP_START      = 0x9,
    STEP_LOOP_STOP       = 0xA,
    //STEP_DISCHARGE_CC_CV = 0xB,
    //STEP_VAC             = 0xC,
    //STEP_END             = 0XD,
    //STEP_DISCHARGE_CP	 = 0xE,
} Old_Step_Type_e;

//工步类型
typedef enum
{
    STEP_STANDING        = 0x1,		//静置
    STEP_CHARGE_CC       = 0x2,		//恒流充电
    STEP_CHARGE_CV       = 0x3,		//恒压充电
    STEP_CHARGE_CC_CV    = 0x4,		//恒流恒压充电
    
    STEP_DISCHARGE_CC    = 0x5,		//恒流放电
    STEP_DISCHARGE_CV    = 0x6,		//恒压放电
    STEP_DISCHARGE_CC_CV = 0x7,		//恒流恒压放电
    
    STEP_CHARGE_CP    	 = 0x8,		//恒功率充电
    STEP_DISCHARGE_CP    = 0x9,		//恒功率放电
    
    STEP_END       		 = 0xA,		//结束

	STEP_LOOP			 = 0x10,	//循环
	STEP_NEG_PRESS		 = 0x11,	//负压
	STEP_INVALID		 = 0xFF,	//无效
} JN9504_Step_Type_e;


//工步参数类型
typedef enum
{
	PARAM_TYPE_TIME	 		= 0x01,		//时间
	PARAM_TYPE_VOLTAGE  	= 0x02,		//电压
	PARAM_TYPE_CURRENT  	= 0x03,		//电流
	PARAM_TYPE_CAPACITY 	= 0x04,		//电容
	PARAM_TYPE_ELECTRICITY 	= 0x05,		//电量
	PARAM_TYPE_INVALID 		= 0xFF		//无效
}JN9504_Param_Type_e;

//工步状态
typedef enum
{
	EXEC_STOP		= 0x00, 	// 停止
	EXEC_PAUSE		= 0x01, 	// 暂停
	EXEC_PROCESS	= 0x02, 	// 启动
	EXEC_RESUME		= 0x03, 	// 继续
	EXEC_JUMP		= 0x04,		// 跳转
	EXEC_INVALID	= 0xFF, 	// 无效
} JN9504_Step_EXEC_e;



typedef enum
{
	CONSTANT_CURRENT_FLG = 0,	//恒流标志
	CONSTANT_VOLTAGE_FLG = 1,	//恒压标志
	CONSTANT_CURRENT_END = 2,	//恒流结束暂态
	CONSTANT_VOLTAGE_BEGIN = 3,	//恒压初始暂态
}JN9504_CC_CV_Flag_e;


//校准参数类型
typedef enum
{
    CAL_VsenseSetK_C = 0x01,	//1. 充电-电池电压基准设定参数K
    CAL_VsenseSetB_C = 0x02,    //2. 充电-电池电压基准设定参数B
    CAL_VmodSetK_C 	= 0x03,		//3. 充电-模块端口电压基准设定参数K
    CAL_VmodSetB_C 	= 0x04,		//4. 充电-模块端口电压基准设定参数B
    CAL_IhSetK_C 	= 0x05,    	//5. 充电-高档位电流基准设定参数K
    CAL_IhSetB_C 	= 0x06,    	//6. 充电-高档位电流基准设定参数B
    CAL_IlSetK_C 	= 0x07,    	//7. 充电-低档位电流基准设定参数K
    CAL_IlSetB_C 	= 0x08,    	//8. 充电-低档位电流基准设定参数B
    CAL_VsenseK_C 	= 0x09,    	//9. 充电-电池电压采样参数K
    CAL_VsenseB_C	= 0x0A,    	//10. 充电-电池电压采样参数B
    CAL_VmodK_C  	= 0x0B,    	//11. 充电-模块端口电压采样参数K
	CAL_VmodB_C 	= 0x0C,		//12. 充电-模块端口电压采样参数B
	CAL_IhK_C 		= 0x0D,		//13. 充电-输出电流高档位采样参数K
    CAL_IhB_C 		= 0x0E,    	//14. 充电-输出电流高档位采样参数B
    CAL_IlK_C 		= 0x0F,    	//15. 充电-输出电流低档位采样参数K
    CAL_IlB_C 		= 0x10,    	//16. 充电-输出电流低档位采样参数B
    CAL_VlugK_C 	= 0x11,   	//17. 充电-极耳电压采样参数K
    CAL_VlugB_C 	= 0x12,    	//18. 充电-极耳电压采样参数B
    CAL_VsenseSetK_D = 0x13,    //19. 放电-电池电压基准设定参数K
    CAL_VsenseSetB_D = 0x14,    //20. 放电-电池电压基准设定参数B
    CAL_VmodSetK_D 	= 0x15,    	//21. 放电-模块端口电压基准设定参数K
    CAL_VmodSetB_D 	= 0x16,    	//22. 放电-模块端口电压基准设定参数B
    CAL_IhSetK_D  	= 0x17,     //23. 放电-高档位电流基准设定参数K
    CAL_IhSetB_D   	= 0x18,    	//24. 放电-高档位电流基准设定参数B
    CAL_IlSetK_D	= 0x19,   	//25. 放电-低档位电流基准设定参数K
    CAL_IlSetB_D 	= 0x1A,   	//26. 放电-低档位电流基准设定参数B
	CAL_VsenseK_D 	=	0x1B,	//27. 放电-电池电压采样参数K
	CAL_VsenseB_D	= 	0x1C,	//28. 放电-电池电压采样参数B
	CAL_VmodK_D		=	0x1D,	//29. 放电-模块端口电压采样参数K
	CAL_VmodB_D		= 	0x1E,	//30. 放电-模块端口电压采样参数B
	CAL_IhK_D		=	0x1F,	//31. 放电-输出电流高档位采样参数K
	CAL_IhB_D		= 	0x20,	//32. 放电-输出电流高档位采样参数B
    CAL_IlK_D     	= 0x21, 	//33. 放电-输出电流低档位采样参数K
    CAL_IlB_D		= 0x22, 	//34. 放电-输出电流低档位采样参数B
    CAL_PARAM_INVALID = 0xFF,
} CAL_PARAM_ENUM;

//AD数据读取类型
typedef enum
{
	CELL_VOLTAGE_SAMPLE = 0x01,
	PORT_VOLTAGE_SAMPLE = 0x02,
	OUT_CURRENT_SAMPLE	= 0x03,
	PROBE_VOLTAGE_SAMPLE= 0x04,
}JN9504_AD_Date_Type_e;

//系统信息查询
typedef enum
{
	SOFTWARE_VERSION 	= 0x01,
	WARNING_INFO_QUERY 	= 0x02,
}JN9504_SystemInquiry_e;

//系统指令设置
typedef enum
{
    SUB_CMD_CLEAR_WARNING 		= 0x01,   	// 清除警告命令
    SUB_CMD_SET_SYS_TIME 		= 0x02, 	// 系统时间设定
    SUB_CMD_BEAT_TRIGGER 		= 0x03, 	// 心跳检测
    SUB_CMD_WORK_MODE 			= 0x04,  	// 系统模式设置：正常模式/校准模式
    SUB_CMD_UPLOAD_ENABLE 		= 0x05,  	// 使能数据上抛
	SUB_CMD_UPLOAD_DISABLE		= 0x06,		// 禁止数据上抛
	SUB_CMD_UPLOAD_CYCLE  		= 0x07,		// 数据上抛周期
	SUB_CMD_SYSTEM_INIT			= 0x08,		// 系统初始化
	SUB_CMD_SYSTEM_HALT			= 0x09,		// 系统停止
	SUB_CMD_HEARTBEAT_ENABLE	= 0x0A,		// 使能心跳包
	SUB_CMD_HEARTBEAT_DISABLE	= 0x0B,		// 禁止心跳包
	SUB_CMD_PROBE_NO_PRESS		= 0x0C,		// 探针未压合
	SUB_CMD_PROBE_PRESSING		= 0x0D,		// 探针压合中
	SUB_CMD_PROBE_PRESSED		= 0x0E,		// 探针压合
} JN9504_SystemSet_e;

//内部保护设置
typedef enum
{
	INTER_OutCurLimit	= 0x01,
	INTER_BusVolUpper	= 0x02,
	INTER_BusVolLower	= 0x03,
	INTER_ConstCur_Threshold	= 0x04,
	INTER_ConstVol_Threshold	= 0x05,
	INTER_ShortCirc_VolDropAmp	= 0x06,
	INTER_ShortCirc_VolDropCnt	= 0x07,
	INTER_SenseOverVolPro		= 0x08,
	INTER_ModuleOverTempPro		= 0x09,
	INTER_EnvirOverTempPro		= 0x0A	
}JN9504_Inter_Protection_e;

//内部保护触发

//ACDC模块保护触发


typedef struct
{
	int cell_no;
	int ch_no;
	int func_code;
	int sub_code;
	uint8_t ch_mask;
}Response_Data_t;

//#pragma pack(1)
typedef struct
{
    uint8 func_code;
    float value;
} Func_Data_t;

typedef struct
{
    Func_Data_t spec_data[3];

    Func_Data_t abort_data[4];
    uint8       abort_level[4];

	Func_Data_t protection[6];
} Step_Param_t;

typedef struct
{
    Step_Param_t general_data;
	Func_Data_t  loop_data[5];
} Step_Data_t;

typedef struct
{
    uint16       step_nbr;
	JN9504_Step_Type_e step_type;
    Step_Data_t  step_data;
} Ch_Step_Data_t;

typedef struct
{
	Func_Data_t 	func_data;
    Func_Data_t 	subList[MAX_FUCN_LIST_SIZE];
	int      		subList_len;
    Ch_Step_Data_t	step_data;
} Channel_Data_t;


typedef struct
{
	uint16 start_idx;
	uint16 frame_len;
	uint8 ch_mask;
}frame_info_t;

typedef struct
{
	int device_no;				//下位机设备号
	char device_ver[64];		//下位机版本号
}lower_mc_version_t;

class ChannelProto_JN9504
{
public:
	ChannelProto_JN9504();
	ChannelProto_JN9504(uint8_t lastFeature);
	
	uint8_t sendCmdPack(uint8_t *cmd_buffer,int ch_no, int func_code, Channel_Data_t &data);
	int recvAckUnPack(RingQueue<uint8> &m_buffer,int m_cell_no,int m_ch_no,int *ack_size,ChannelState &m_state);

	static bool splicingPacket_resolve(uint8 *data,int dataLen,std::vector<frame_info_t> &n_frame);
	static std::string get_stepname_string(uint8_t stepType);
	
private:	
	void globalProtect_set(Channel_Data_t &data,uint8_t *buffer,uint8_t &size);
	void globalProtect_query(uint8_t *buffer,uint8_t &size);
	//void processData_query(Channel_Data_t &data,uint8_t *buffer,uint8_t &size);
	void workstep_set(Channel_Data_t &data,uint8_t *buffer,uint8_t &size);
	void workstep_exec_set(Channel_Data_t &data,uint8_t *buffer,uint8_t &size);
	void workstep_query(Channel_Data_t &data,uint8_t *buffer,uint8_t &size);
	void calibrate_param_set(Channel_Data_t &data,uint8_t *buffer,uint8_t &size);
	void calibrate_param_query(uint8_t *buffer,uint8_t &size);
	void AD_sample_query(Channel_Data_t &data,uint8_t *buffer,uint8_t &size);
	void system_info_query(Channel_Data_t &data,uint8_t *buffer,uint8_t &size);
	void system_subcmd_set(Channel_Data_t &data,uint8_t *buffer,uint8_t &size);
	void system_update_set(Channel_Data_t &data,uint8_t *buffer,uint8_t &size);
	void internalProtect_set(Channel_Data_t &data,uint8_t *buffer,uint8_t &size);
	void internalProtect_query(uint8_t *buffer,uint8_t &size);
	void temperature_query(Channel_Data_t &data,uint8_t *buffer,uint8_t &size);
	
	void response_reply_send(int value,Response_Data_t &m_data);
	void response_reply_send(std::vector<float> &vals,Response_Data_t &m_data);
	void response_exception_send(int error_code,Response_Data_t &m_data);

	void ack_set_reply(uint8_t *sub_buffer,int &index,int sub_len,Response_Data_t &m_data);
	void ack_query_reply(uint8_t *sub_buffer,int &index,int sub_len,Response_Data_t &m_data);
	void ack_process_data_reply(uint8_t *sub_buffer,int &index,int sub_len,Response_Data_t &m_data,ChannelState &m_state);
	void ack_step_set_reply(uint8_t *sub_buffer,int &index,Response_Data_t &m_data);
	void ack_trig_protection_reply(uint8_t *sub_buffer,int &index,int sub_len,Response_Data_t &m_data,ChannelState &m_state);
	void ack_system_get_reply(uint8_t *sub_buffer,int &index,int sub_len,Response_Data_t &m_data,ChannelState &m_state);
	void ack_system_set_reply(uint8_t *sub_buffer,int &index,int sub_len,Response_Data_t &m_data,ChannelState &m_state);
	void ack_temperature_reply(uint8_t *sub_buffer,int &index,int sub_len,Response_Data_t &m_data,ChannelState &m_state);

private:
#if defined(AWH_FORMATION_SYS)
	uint8_t m_last_run_state;		//上一运行状态
#else
	uint16_t m_last_step_no;		//上一工步号
#endif
	uint8_t feat; 
};

#endif
#endif



