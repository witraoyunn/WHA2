#ifndef _CHANNELPROTO_JN9504_H_
#define _CHANNELPROTO_JN9504_H_
#include <stdint.h>
#include <string>
#include <vector>
#include "RingQueue.h"
#include "ChannelState.h"

#if (defined(AWH_FORMATION_SYS) || defined(AWH_PRECHARGE_SYS))
#define MODULE_CH_NUM				8		// �к��人 ����Ԥ�� һ̨��λ����8��ͨ��
#else
#define MODULE_CH_NUM				4		// �к��人 ���� 	һ̨��λ����4��ͨ��
#endif

#define MAX_FUCN_LIST_SIZE 		200		//48

#ifdef PROTOCOL_5V160A
typedef enum
{
/* ---------------------------JN9504ͨ��ָ��------------------------- */
//1ȫ�ֱ���
    CMD_SET_PROTECTION 			= 0x01,     // �������ù���
    ACK_SET_PROTECTION 			= 0x81,     // ����������Ӧ

    CMD_QUERY_PROTECTION 		= 0x02,    	// ��ѯ��������
    ACK_QUERY_PROTECTION 		= 0x82,    	// ��ѯ�����ظ�

    ACK_TRIG_PROTECTION			= 0x83,    	// ��������
//2��¼��
    CMD_QUERY_PROCESS_DATA		= 0x04,  	// ��¼���ݲ�ѯ
    ACK_QUERY_PROCESS_DATA		= 0x84,  	// ��¼���ݲ�ѯ�ظ�
//3������
    CMD_SET_STEP 				= 0x05,     // ��������ָ��
    ACK_SET_STEP 				= 0x85,     // ����������Ӧ

    CMD_RUN_CONTROL 			= 0x06,     // ��������״̬����
    ACK_RUN_CONTROL 			= 0x86,     // ��������״̬���ûظ�

	CMD_STEP_INQ				= 0x07,		// ������ѯ���� 
	ACK_STEP_INQ				= 0x87,		// ������ѯ�ظ� 

    ACK_STEP_TRIG_PROTECTION 	= 0x88,    	// ������������
//4У׼��
	CMD_SET_CALIBRATION 		= 0x09,		// �ϻ�У׼����
	ACK_SET_CALIBRATION 		= 0x89,     // �ϻ�У׼���ûظ�

	CMD_QUERY_CALIBRATION 		= 0x0A,   	// �ϻ�У׼���ò�ѯ
	ACK_QUERY_CALIBRATION 		= 0x8A,    	// �ϻ�У׼��ѯ�ظ�

	CMD_AD_DATA_INQ				= 0x0B,		// AD���ݶ�ȡ���� 
	CMD_AD_DATA_INQ_REPLY		= 0x8B,		// AD���ݶ�ȡ�ظ� 
//5ϵͳ��
	CMD_SYSTEM_GET				= 0x0C, 	// ϵͳ��ѯ
	ACK_SYSTEM_GET				= 0x8C, 	// ϵͳ��ѯ�ظ�

	CMD_SYSTEM_SET				= 0x0D, 	// ϵͳ����
	ACK_SYSTEM_SET				= 0x8D, 	// ϵͳ���ûظ�

	CMD_SYSTEM_UPDATE			= 0x0E,		// ϵͳ��������/�ظ�
//6������
	CMD_ACDC_UPDATE 			= 0x60,		// AC/DC�������� 
	CMD_ACDC_UPDATE_REPLY		= 0xE0,		// AC/DC�����ظ� 
//7�ڲ�����
	CMD_SET_INTERNAL_PROTECTION	= 0x0F, 	// �ڲ���������
	ACK_SET_INTERNAL_PROTECTION	= 0x8F, 	// �ڲ�������Ӧ

	CMD_QUERY_INTERNAL_PROTECTION = 0x10,	//�ڲ�������ѯ
	ACK_QUERY_INTERNAL_PROTECTION = 0x90,	//�ڲ�������ѯ���ظ�
	
	CMD_INTERNAL_TRIG_PROTECT 	= 0xA0,		// �ڲ���������
	CMD_ACDC_TRIG_PROTECT 		= 0xA1,		// ACDC��������
	
	//CMD_INTERNAL_PROTECT_INQ	= 0x10,		// �ڲ��������ò�ѯ���� ???
	//CMD_INTERNAL_PROTECT_TRIG	= 0xA0,		// �ڲ������������� 
	//CMD_ACDC_PROTECT_TRIG		= 0xA1,		// AC/DC������������

	CMD_QUERY_TEMPERATURE		= 0x75,		//��ѯ�¶�
	ACK_QUERY_TEMPERATURE		= 0xF5,		//�¶Ȳ�ѯ�ظ�
	
	CMD_INVALID					= 0xFF,		//��Ч����
}JN9504_Function_Code_e;


//ȫ�ֱ�������
typedef enum
{
    GLOBAL_OverVolPro 	= 0x01,     // ��ص�ѹ����
    GLOBAL_UnderVolPro 	= 0x02,     // ��ص�ѹ����
    //GLOBAL_LoopVolDiff = 0x03,     // ��·ѹ��
    GLOBAL_C_OverCurPro = 0x03,     // ������
    GLOBAL_D_OverCurPro = 0x04,     // �ŵ����
    GLOBAL_OverCapPro	= 0x05,    	// ���ݱ���
    
    //GLOBAL_OverTotalCap = 0x07,     // ����������
	GLOBAL_InitVolUpperPro = 0x06,  // ��ʼ��ѹ����
    GLOBAL_InitVolLowerPro = 0x07,	// ��ʼ��ѹ����
    GLOBAL_ContactTimePro  = 0x08,	// �Ӵ����ʱ��
    GLOBAL_ContactCurPro = 0x09, 	// �Ӵ�������
    GLOBAL_ContactVolPro = 0x0A,   	// �Ӵ�����ѹ

	GLOBAL_OTHRE_ERR = 0xFE,	// ��������
    GLOBAL_INVALID 	 = 0xFF,  	// ��Ч
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

//��������
typedef enum
{
    STEP_STANDING        = 0x1,		//����
    STEP_CHARGE_CC       = 0x2,		//�������
    STEP_CHARGE_CV       = 0x3,		//��ѹ���
    STEP_CHARGE_CC_CV    = 0x4,		//������ѹ���
    
    STEP_DISCHARGE_CC    = 0x5,		//�����ŵ�
    STEP_DISCHARGE_CV    = 0x6,		//��ѹ�ŵ�
    STEP_DISCHARGE_CC_CV = 0x7,		//������ѹ�ŵ�
    
    STEP_CHARGE_CP    	 = 0x8,		//�㹦�ʳ��
    STEP_DISCHARGE_CP    = 0x9,		//�㹦�ʷŵ�
    
    STEP_END       		 = 0xA,		//����

	STEP_LOOP			 = 0x10,	//ѭ��
	STEP_NEG_PRESS		 = 0x11,	//��ѹ
	STEP_INVALID		 = 0xFF,	//��Ч
} JN9504_Step_Type_e;


//������������
typedef enum
{
	PARAM_TYPE_TIME	 		= 0x01,		//ʱ��
	PARAM_TYPE_VOLTAGE  	= 0x02,		//��ѹ
	PARAM_TYPE_CURRENT  	= 0x03,		//����
	PARAM_TYPE_CAPACITY 	= 0x04,		//����
	PARAM_TYPE_ELECTRICITY 	= 0x05,		//����
	PARAM_TYPE_INVALID 		= 0xFF		//��Ч
}JN9504_Param_Type_e;

//����״̬
typedef enum
{
	EXEC_STOP		= 0x00, 	// ֹͣ
	EXEC_PAUSE		= 0x01, 	// ��ͣ
	EXEC_PROCESS	= 0x02, 	// ����
	EXEC_RESUME		= 0x03, 	// ����
	EXEC_JUMP		= 0x04,		// ��ת
	EXEC_INVALID	= 0xFF, 	// ��Ч
} JN9504_Step_EXEC_e;



typedef enum
{
	CONSTANT_CURRENT_FLG = 0,	//������־
	CONSTANT_VOLTAGE_FLG = 1,	//��ѹ��־
	CONSTANT_CURRENT_END = 2,	//����������̬
	CONSTANT_VOLTAGE_BEGIN = 3,	//��ѹ��ʼ��̬
}JN9504_CC_CV_Flag_e;


//У׼��������
typedef enum
{
    CAL_VsenseSetK_C = 0x01,	//1. ���-��ص�ѹ��׼�趨����K
    CAL_VsenseSetB_C = 0x02,    //2. ���-��ص�ѹ��׼�趨����B
    CAL_VmodSetK_C 	= 0x03,		//3. ���-ģ��˿ڵ�ѹ��׼�趨����K
    CAL_VmodSetB_C 	= 0x04,		//4. ���-ģ��˿ڵ�ѹ��׼�趨����B
    CAL_IhSetK_C 	= 0x05,    	//5. ���-�ߵ�λ������׼�趨����K
    CAL_IhSetB_C 	= 0x06,    	//6. ���-�ߵ�λ������׼�趨����B
    CAL_IlSetK_C 	= 0x07,    	//7. ���-�͵�λ������׼�趨����K
    CAL_IlSetB_C 	= 0x08,    	//8. ���-�͵�λ������׼�趨����B
    CAL_VsenseK_C 	= 0x09,    	//9. ���-��ص�ѹ��������K
    CAL_VsenseB_C	= 0x0A,    	//10. ���-��ص�ѹ��������B
    CAL_VmodK_C  	= 0x0B,    	//11. ���-ģ��˿ڵ�ѹ��������K
	CAL_VmodB_C 	= 0x0C,		//12. ���-ģ��˿ڵ�ѹ��������B
	CAL_IhK_C 		= 0x0D,		//13. ���-��������ߵ�λ��������K
    CAL_IhB_C 		= 0x0E,    	//14. ���-��������ߵ�λ��������B
    CAL_IlK_C 		= 0x0F,    	//15. ���-��������͵�λ��������K
    CAL_IlB_C 		= 0x10,    	//16. ���-��������͵�λ��������B
    CAL_VlugK_C 	= 0x11,   	//17. ���-������ѹ��������K
    CAL_VlugB_C 	= 0x12,    	//18. ���-������ѹ��������B
    CAL_VsenseSetK_D = 0x13,    //19. �ŵ�-��ص�ѹ��׼�趨����K
    CAL_VsenseSetB_D = 0x14,    //20. �ŵ�-��ص�ѹ��׼�趨����B
    CAL_VmodSetK_D 	= 0x15,    	//21. �ŵ�-ģ��˿ڵ�ѹ��׼�趨����K
    CAL_VmodSetB_D 	= 0x16,    	//22. �ŵ�-ģ��˿ڵ�ѹ��׼�趨����B
    CAL_IhSetK_D  	= 0x17,     //23. �ŵ�-�ߵ�λ������׼�趨����K
    CAL_IhSetB_D   	= 0x18,    	//24. �ŵ�-�ߵ�λ������׼�趨����B
    CAL_IlSetK_D	= 0x19,   	//25. �ŵ�-�͵�λ������׼�趨����K
    CAL_IlSetB_D 	= 0x1A,   	//26. �ŵ�-�͵�λ������׼�趨����B
	CAL_VsenseK_D 	=	0x1B,	//27. �ŵ�-��ص�ѹ��������K
	CAL_VsenseB_D	= 	0x1C,	//28. �ŵ�-��ص�ѹ��������B
	CAL_VmodK_D		=	0x1D,	//29. �ŵ�-ģ��˿ڵ�ѹ��������K
	CAL_VmodB_D		= 	0x1E,	//30. �ŵ�-ģ��˿ڵ�ѹ��������B
	CAL_IhK_D		=	0x1F,	//31. �ŵ�-��������ߵ�λ��������K
	CAL_IhB_D		= 	0x20,	//32. �ŵ�-��������ߵ�λ��������B
    CAL_IlK_D     	= 0x21, 	//33. �ŵ�-��������͵�λ��������K
    CAL_IlB_D		= 0x22, 	//34. �ŵ�-��������͵�λ��������B
    CAL_PARAM_INVALID = 0xFF,
} CAL_PARAM_ENUM;

//AD���ݶ�ȡ����
typedef enum
{
	CELL_VOLTAGE_SAMPLE = 0x01,
	PORT_VOLTAGE_SAMPLE = 0x02,
	OUT_CURRENT_SAMPLE	= 0x03,
	PROBE_VOLTAGE_SAMPLE= 0x04,
}JN9504_AD_Date_Type_e;

//ϵͳ��Ϣ��ѯ
typedef enum
{
	SOFTWARE_VERSION 	= 0x01,
	WARNING_INFO_QUERY 	= 0x02,
}JN9504_SystemInquiry_e;

//ϵͳָ������
typedef enum
{
    SUB_CMD_CLEAR_WARNING 		= 0x01,   	// �����������
    SUB_CMD_SET_SYS_TIME 		= 0x02, 	// ϵͳʱ���趨
    SUB_CMD_BEAT_TRIGGER 		= 0x03, 	// �������
    SUB_CMD_WORK_MODE 			= 0x04,  	// ϵͳģʽ���ã�����ģʽ/У׼ģʽ
    SUB_CMD_UPLOAD_ENABLE 		= 0x05,  	// ʹ����������
	SUB_CMD_UPLOAD_DISABLE		= 0x06,		// ��ֹ��������
	SUB_CMD_UPLOAD_CYCLE  		= 0x07,		// ������������
	SUB_CMD_SYSTEM_INIT			= 0x08,		// ϵͳ��ʼ��
	SUB_CMD_SYSTEM_HALT			= 0x09,		// ϵͳֹͣ
	SUB_CMD_HEARTBEAT_ENABLE	= 0x0A,		// ʹ��������
	SUB_CMD_HEARTBEAT_DISABLE	= 0x0B,		// ��ֹ������
	SUB_CMD_PROBE_NO_PRESS		= 0x0C,		// ̽��δѹ��
	SUB_CMD_PROBE_PRESSING		= 0x0D,		// ̽��ѹ����
	SUB_CMD_PROBE_PRESSED		= 0x0E,		// ̽��ѹ��
} JN9504_SystemSet_e;

//�ڲ���������
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

//�ڲ���������

//ACDCģ�鱣������


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
	int device_no;				//��λ���豸��
	char device_ver[64];		//��λ���汾��
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
	uint8_t m_last_run_state;		//��һ����״̬
#else
	uint16_t m_last_step_no;		//��һ������
#endif
	uint8_t feat; 
};

#endif
#endif



