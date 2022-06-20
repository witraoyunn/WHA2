#include "log4z.h"
#include "PinsGroup_AXMPreCharge.h"

using namespace std;

#if defined(AWH_PRECHARGE_SYS)
/****************************************************************/
//PLC�澯D500��ʼ
#define  SYS_WARING_BASE_ADDR                     500
#define  SYS_WARING_PRESSUR_ERROR_OFFSET          0         //��ѹ�쳣
#define  SYS_WARING_STOP_OFFSET                   1         //ϵͳ����ͣ�{
#define  SYS_WARING_HEART_OFFSET                  2         //PLC����
#define  SYS_WARING_DOR_OFFSET                    3         //ά���]

#define  CELL_WARING_ADDR1                        501
#define  CELL_WARING_MOVE_UPTIMEOUT_OFFSET        0         //�����������г�ʱ
#define  CELL_WARING_MOVE_DOWNTIMEOUT_OFFSET      1         //�����������г�ʱ
#define  CELL_WARING_TRAY_REVERSE_OFFSET          2         //���̷���
#define  CELL_WARING_NO_TRAY_OFFSET               3         //���бa
#define  CELL_WARING_TRAY_NOTREADY_OFFSET         4         //����δ��ʹ
#define  CELL_WARING_FAN_OVERLOAD_OFFSET          5         //�ŷ������
#define  CELL_WARING_ROLLER_OVERLOAD_OFFSET       6         //��Ͳ����
#define  CELL_WARING_ROLLER_RUN_TIMEOUT_OFFSET    7         //��Ͳ��ⳬʱ

#define  CELL_WARING_ROLLER_REVERSE_RUN_TIMEOUT_OFFSET    8         //��Ͳ���ⳬʱ
#define  CELL_WARING_LEFT_FAN_ERROR_OFFSET        9         //����ŷ����쎫
#define  CELL_WARING_RIGHT_FAN_ERROR_OFFSET       10         //�Ҳ��ŷ����쎫
#define  CELL_WARING_TOP_FAN_ERROR_OFFSET         11        //�����ŷ����쎫

#define  CELL_WARING_ADDR2                        502
#define  CELL_WARING_SMOKE_ALARM1_OFFSET       	  0         //�̸и澯1	
#define  CELL_WARING_SMOKE_ALARM2_OFFSET       	  1         //�̸и澯2	
#define  CELL_WARING_CAB_SMOKE_ALARM_OFFSET       2         //����̸и澯
#define  CELL_WARING_PIN_SMOKE_ALARM_OFFSET       3         //�봲��и澯
#define  CELL_WARING_PIN_SMOKE_HEAVY_ALARM_OFFSET   4         //�봲Ũ�и澯
#define  CELL_WARING_HIGH_TEMP_ALARM_OFFSET       5         //���¸澯

#define  HEART_ADDR                               503

#define  CELL_STATUS_ADDR                         504
#define  CELL_STATUS_TRAY_ISEXTEND_OFFSET         0          //����ѹ�ϵ�λ״��
#define  CELL_STATUS_TRAY_ISRETRACT_OFFSET        1          //�����ѿ���λ״��
#define  CELL_STATUS_TOOL_POWER_ON_OFFSET         2          //��װȡ��״��
#define  CELL_STATUS_COOL_FAN_ON_OFFSET           3          //��ȴ����״��
#define  CELL_STATUS_AIR_FAN_ON_OFFSET            4          //����״��
#define  CELL_STATUS_AUTO_MODE_ON_OFFSET          5          //���Զ�ģ��
#define  CELL_STATUS_IS_LOADED_OFFSET             6          //��ʼ���귿
#define  CELL_STATUS_TRAY_READY_OFFSET            7          //������λ״��
#define  CELL_STATUS_IS_AUTO_START_OFFSET         8          //�Զ�����״��

#define  CELL_SET_ADDR                            505
#define  CELL_SET_TRAY_MOVEUP_OFFSET              0	         //��������
#define  CELL_SET_TRAY_MOVEDOWN_OFFSET            1	         //�����½�
#define  CELL_SET_TOOL_POWER_OFFSET               2	         //��װȡ��
#define  CELL_SET_COOL_FAN_OFFSET                 3	         //��ȴ����
#define  CELL_SET_AIR_FAN_OFFSET                  4	         //���c
#define  CELL_SET_CLEAR_ALARM_OFFSET              5	         //����澯
#define  CELL_SET_LOAD_OFFSET                     6	         //��ʼ����ʹ
#define  CELL_SET_AUTO_START_OFFSET               7	         //�Զ���������
#define  CELL_SET_STOP_OFFSET                     8	         //ֹͣ����

#define  CELL_TEM_ADDR                            510       //510 ~ 516 7����λ��
#define  CELL_TEM_SET_ADDR                        517		//��λ�澯�¶�

#define  INCELL_ONOFF_ADDR                        520  //�������
#define  INCELL_STATUS_ADDR                       530  //��⵽λ֪ͨ
#define  INCELL_TRAY_INFO_ADDR                    531  //����״̬����

#define  OUTCELL_ONOFF_ADDR                       532  //���̳���֪ͨ
#define  OUTCELL_STATUS_ADDR                      533  //���̳������


PinsGroup_AXMPreCharge::PinsGroup_AXMPreCharge()
{
	is_plc_connected = false;
	heart_timeout_cnt = 0;
}

PinsGroup_AXMPreCharge::~PinsGroup_AXMPreCharge()
{

}

bool PinsGroup_AXMPreCharge::do_plc_connect(const char *ip,int port)
{
	bool status = false;
	status = m_slmp_dev.SLMPconnect(ip,port);
	if(status)
	{
		is_plc_connected = true;
	}
	else
	{
		is_plc_connected = false;
	}

	return status;
}

bool PinsGroup_AXMPreCharge::is_connected()
{
	return is_plc_connected;
}

void PinsGroup_AXMPreCharge::close_connect()
{
	is_plc_connected = false;
	m_slmp_dev.SLMPclose();
}

//��ѹ�쳣״��
bool PinsGroup_AXMPreCharge::get_sys_pressur_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(SYS_WARING_BASE_ADDR,SYS_WARING_PRESSUR_ERROR_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_sys_pressur_alarm register addr:%d offset:%d failed !",SYS_WARING_BASE_ADDR,SYS_WARING_PRESSUR_ERROR_OFFSET);
		return false;
	}
	return status;
}

//ϵͳ�쳣״��
bool PinsGroup_AXMPreCharge::get_sys_stop_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(SYS_WARING_BASE_ADDR,SYS_WARING_STOP_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_sys_stop_alarm register addr:%d offset:%d failed !",SYS_WARING_BASE_ADDR,SYS_WARING_STOP_OFFSET);
		return false;
	}
	return status;
}

//PLC�����쳣״��
bool PinsGroup_AXMPreCharge::get_sys_heart_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(SYS_WARING_BASE_ADDR,SYS_WARING_HEART_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_sys_heart_alarm register addr:%d offset:%d failed !",SYS_WARING_BASE_ADDR,SYS_WARING_HEART_OFFSET);
		return false;
	}
	return status;
}

// �����쳣״��
bool PinsGroup_AXMPreCharge::get_sys_dor_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(SYS_WARING_BASE_ADDR,SYS_WARING_DOR_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_sys_dor_alarm register addr:%d offset:%d failed !",SYS_WARING_BASE_ADDR,SYS_WARING_DOR_OFFSET);
		return false;
	}
	return status;
}

// �����������г�ʱ�쳣
bool PinsGroup_AXMPreCharge::get_move_uptimeout_alarm()
{
	bool status = false;
	bool ret = false;
	ret= m_slmp_dev.read_bit(CELL_WARING_ADDR1,CELL_WARING_MOVE_UPTIMEOUT_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_move_uptimeout_alarm register addr:%d offset:%d failed !",CELL_WARING_ADDR1,CELL_WARING_MOVE_UPTIMEOUT_OFFSET);
		return false;
	}
	return status;
}

// �����������г�ʱ�쳣
bool PinsGroup_AXMPreCharge::get_move_downtimeout_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_WARING_ADDR1,CELL_WARING_MOVE_DOWNTIMEOUT_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_move_downtimeout_alarm register addr:%d offset:%d failed !",CELL_WARING_ADDR1,CELL_WARING_MOVE_DOWNTIMEOUT_OFFSET);
		return false;
	}
	return status;
}

// ���̷����쳣
bool PinsGroup_AXMPreCharge::get_tray_reverse_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_WARING_ADDR1,CELL_WARING_TRAY_REVERSE_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_tray_reverse_alarm register addr:%d offset:%d failed !",CELL_WARING_ADDR1,CELL_WARING_TRAY_REVERSE_OFFSET);
		return false;
	}
	return status;
}

// �������쎫
bool PinsGroup_AXMPreCharge::get_no_tray_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_WARING_ADDR1,CELL_WARING_NO_TRAY_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_no_tray_alarm register addr:%d offset:%d failed !",CELL_WARING_ADDR1,CELL_WARING_NO_TRAY_OFFSET);
		return false;
	}
	return status;
}

// ����δ��λ�쎫
bool PinsGroup_AXMPreCharge::get_tray_not_ready_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_WARING_ADDR1,CELL_WARING_TRAY_NOTREADY_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_tray_not_ready_alarm register addr:%d offset:%d failed !",CELL_WARING_ADDR1,CELL_WARING_TRAY_NOTREADY_OFFSET);
		return false;
	}
	return status;
}

// �ŷ�������쎫
bool PinsGroup_AXMPreCharge::get_fan_overload_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_WARING_ADDR1,CELL_WARING_FAN_OVERLOAD_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_fan_overload_alarm register addr:%d offset:%d failed !",CELL_WARING_ADDR1,CELL_WARING_FAN_OVERLOAD_OFFSET);
		return false;
	}
	return status;
}

// ��Ͳ�����쳣
bool PinsGroup_AXMPreCharge::get_roller_overload_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_WARING_ADDR1,CELL_WARING_ROLLER_OVERLOAD_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_roller_overload_alarm register addr:%d offset:%d failed !",CELL_WARING_ADDR1,CELL_WARING_ROLLER_OVERLOAD_OFFSET);
		return false;
	}
	return status;
}

// ��Ͳ��ⳬʱ�쳣
bool PinsGroup_AXMPreCharge::get_roller_run_timeout_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_WARING_ADDR1,CELL_WARING_ROLLER_RUN_TIMEOUT_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_roller_run_timeout_alarm register addr:%d offset:%d failed !",CELL_WARING_ADDR1,CELL_WARING_ROLLER_RUN_TIMEOUT_OFFSET);
		return false;
	}
	return status;
}

// ��Ͳ���ⳬʱ
bool PinsGroup_AXMPreCharge::get_roller_reverse_run_timeout_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_WARING_ADDR1,CELL_WARING_ROLLER_REVERSE_RUN_TIMEOUT_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_roller_reverse_run_timeout_alarm register addr:%d offset:%d failed !",CELL_WARING_ADDR1,CELL_WARING_ROLLER_REVERSE_RUN_TIMEOUT_OFFSET);
		return false;
	}
	return status;
}

// ����ŷ����쎫
bool PinsGroup_AXMPreCharge::get_left_fan_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_WARING_ADDR1,CELL_WARING_LEFT_FAN_ERROR_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_left_fan_alarm register addr:%d offset:%d failed !",CELL_WARING_ADDR1,CELL_WARING_LEFT_FAN_ERROR_OFFSET);
		return false;
	}
	return status;
}

// �Ҳ��ŷ����쎫
bool PinsGroup_AXMPreCharge::get_right_fan_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_WARING_ADDR1,CELL_WARING_RIGHT_FAN_ERROR_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_right_fan_alarm register addr:%d offset:%d failed !",CELL_WARING_ADDR1,CELL_WARING_RIGHT_FAN_ERROR_OFFSET);
		return false;
	}
	return status;
}

// �����ŷ����쎫
bool PinsGroup_AXMPreCharge::get_top_fan_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_WARING_ADDR1,CELL_WARING_TOP_FAN_ERROR_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_top_fan_alarm register addr:%d offset:%d failed !",CELL_WARING_ADDR1,CELL_WARING_TOP_FAN_ERROR_OFFSET);
		return false;
	}
	return status;
}

// �̸и澯1
bool PinsGroup_AXMPreCharge::get_smake1_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_WARING_ADDR2,CELL_WARING_SMOKE_ALARM1_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_smake1_alarm register addr:%d offset:%d failed !",CELL_WARING_ADDR2,CELL_WARING_SMOKE_ALARM1_OFFSET);
		return false;
	}
	return status;
}

// �̸и澯2
bool PinsGroup_AXMPreCharge::get_smake2_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_WARING_ADDR2,CELL_WARING_SMOKE_ALARM2_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_smake2_alarm register addr:%d offset:%d failed !",CELL_WARING_ADDR2,CELL_WARING_SMOKE_ALARM2_OFFSET);
		return false;
	}
	return status;
}

// ����̸и澯
bool PinsGroup_AXMPreCharge::get_cab_smake_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_WARING_ADDR2,CELL_WARING_CAB_SMOKE_ALARM_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_cab_smake_alarm register addr:%d offset:%d failed !",CELL_WARING_ADDR2,CELL_WARING_CAB_SMOKE_ALARM_OFFSET);
		return false;
	}
	return status;
}

// �봲��и澯
bool PinsGroup_AXMPreCharge::get_pin_smake_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_WARING_ADDR2,CELL_WARING_PIN_SMOKE_ALARM_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_pin_smake_alarm register addr:%d offset:%d failed !",CELL_WARING_ADDR2,CELL_WARING_PIN_SMOKE_ALARM_OFFSET);
		return false;
	}
	return status;
}

// �봲Ũ�и澯
bool PinsGroup_AXMPreCharge::get_pin_smake_heavy_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_WARING_ADDR2,CELL_WARING_PIN_SMOKE_HEAVY_ALARM_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_pin_smake_heavy_alarm register addr:%d offset:%d failed !",CELL_WARING_ADDR2,CELL_WARING_PIN_SMOKE_HEAVY_ALARM_OFFSET);
		return false;
	}
	return status;
}

// ���¸澯
bool PinsGroup_AXMPreCharge::get_high_temp_alarm()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_WARING_ADDR2,CELL_WARING_HIGH_TEMP_ALARM_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("get_high_temp_alarm register addr:%d offset:%d failed !",CELL_WARING_ADDR2,CELL_WARING_HIGH_TEMP_ALARM_OFFSET);
		return false;
	}
	return status;
}

// ��������
bool PinsGroup_AXMPreCharge::get_heartbeat()
{
	uint16_t currVal = 0;
	bool ret = false;
	
	ret = m_slmp_dev.read_register(HEART_ADDR,currVal);
	if(!ret)
	{
		LOGFMTE("get_heartbeat register addr:%d failed !",HEART_ADDR);
		return false;
	}

	if (currVal != 2)
	{
		if (++heart_timeout_cnt > 5)	
		{
			heart_timeout_cnt = 0;
			return false;
		}
	}
	else
	{
		ret = m_slmp_dev.write_register(HEART_ADDR,1);
		if(!ret)
		{
			LOGFMTE("set_heartbeat 1 register addr:%d failed !",HEART_ADDR);
			return false;
		}
		heart_timeout_cnt = 0;
	}

	return true;
}


// ����ѹ�ϵ�λ״��
bool PinsGroup_AXMPreCharge::tray_is_extend()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_STATUS_ADDR,CELL_STATUS_TRAY_ISEXTEND_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("tray_is_extend register addr:%d offset:%d failed !",CELL_STATUS_ADDR,CELL_STATUS_TRAY_ISEXTEND_OFFSET);
		return false;
	}
	return status;
}

// �����ѿ���λ״��
bool PinsGroup_AXMPreCharge::tray_is_retract()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_STATUS_ADDR,CELL_STATUS_TRAY_ISRETRACT_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("tray_is_retract register addr:%d offset:%d failed !",CELL_STATUS_ADDR,CELL_STATUS_TRAY_ISRETRACT_OFFSET);
		return false;
	}
	return status;
}

// ��װȡ��״��
bool PinsGroup_AXMPreCharge::tool_power_onoff()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_STATUS_ADDR,CELL_STATUS_TOOL_POWER_ON_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("tool_power_onoff register addr:%d offset:%d failed !",CELL_STATUS_ADDR,CELL_STATUS_TOOL_POWER_ON_OFFSET);
		return false;
	}
	return status;
}

// ��ȴ����״��
bool PinsGroup_AXMPreCharge::cool_fan_onoff()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_STATUS_ADDR,CELL_STATUS_COOL_FAN_ON_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("cool_fan_onoff register addr:%d offset:%d failed !",CELL_STATUS_ADDR,CELL_STATUS_COOL_FAN_ON_OFFSET);
		return false;
	}
	return status;
}

// ����״��
bool PinsGroup_AXMPreCharge::air_fan_onoff()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_STATUS_ADDR,CELL_STATUS_AIR_FAN_ON_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("air_fan_onoff register addr:%d offset:%d failed !",CELL_STATUS_ADDR,CELL_STATUS_AIR_FAN_ON_OFFSET);
		return false;
	}
	return status;
}

// ���Զ�ģ��
bool PinsGroup_AXMPreCharge::plc_mode()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_STATUS_ADDR,CELL_STATUS_AUTO_MODE_ON_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("plc_mode register addr:%d offset:%d failed !",CELL_STATUS_ADDR,CELL_STATUS_AUTO_MODE_ON_OFFSET);
		return false;
	}
	return status;
}

// ��ʼ���귿
bool PinsGroup_AXMPreCharge::plc_is_loaded()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_STATUS_ADDR,CELL_STATUS_IS_LOADED_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("plc_is_loaded register addr:%d offset:%d failed !",CELL_STATUS_ADDR,CELL_STATUS_IS_LOADED_OFFSET);
		return false;
	}
	return status;
}

// ������λ״��
bool PinsGroup_AXMPreCharge::tray_is_ready()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_STATUS_ADDR,CELL_STATUS_TRAY_READY_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("tray_is_ready register addr:%d offset:%d failed !",CELL_STATUS_ADDR,CELL_STATUS_TRAY_READY_OFFSET);
		return false;
	}
	return status;
}

// �Զ�����״��
bool PinsGroup_AXMPreCharge::is_auto_start()
{
	bool status = false;
	bool ret = false;
	ret = m_slmp_dev.read_bit(CELL_STATUS_ADDR,CELL_STATUS_IS_AUTO_START_OFFSET,status);
	if(!ret)
	{
		LOGFMTE("is_auto_start register addr:%d offset:%d failed !",CELL_STATUS_ADDR,CELL_STATUS_IS_AUTO_START_OFFSET);
		return false;
	}
	return status;
}

// ��������
bool PinsGroup_AXMPreCharge::set_tray_moveup()
{
	bool ret = false;
	ret = m_slmp_dev.write_bit(CELL_SET_ADDR,CELL_SET_TRAY_MOVEUP_OFFSET,1);
	if(!ret)
	{
		LOGFMTE("set_tray_moveup register addr:%d offset:%d failed !",CELL_SET_ADDR,CELL_SET_TRAY_MOVEUP_OFFSET);
		return false;
	}
	return ret;
}

// �����½�
bool PinsGroup_AXMPreCharge::set_tray_movedown()
{
	bool ret = false;
	ret = m_slmp_dev.write_bit(CELL_SET_ADDR,CELL_SET_TRAY_MOVEDOWN_OFFSET,1);
	if(!ret)
	{
		LOGFMTE("set_tray_movedown register addr:%d offset:%d failed !",CELL_SET_ADDR,CELL_SET_TRAY_MOVEUP_OFFSET);
		return false;
	}
	return ret;
}

// ��װȡ�翪
bool PinsGroup_AXMPreCharge::set_tool_power_on()
{
	bool ret = false;
	ret = m_slmp_dev.write_bit(CELL_SET_ADDR,CELL_SET_TOOL_POWER_OFFSET,1);
	if(!ret)
	{
		LOGFMTE("set_tool_power_on register addr:%d offset:%d failed !",CELL_SET_ADDR,CELL_SET_TOOL_POWER_OFFSET);
		return false;
	}
	return ret;
}

// ��װȡ���
bool PinsGroup_AXMPreCharge::set_tool_power_off()
{
	bool ret = false;
	ret = m_slmp_dev.write_bit(CELL_SET_ADDR,CELL_SET_TOOL_POWER_OFFSET,0);
	if(!ret)
	{
		LOGFMTE("set_tool_power_off register addr:%d offset:%d failed !",CELL_SET_ADDR,CELL_SET_TOOL_POWER_OFFSET);
		return false;
	}
	return ret;
}

// ��ȴ���ȿ�
bool PinsGroup_AXMPreCharge::set_cool_fan_on()
{
	bool ret = false;
	ret = m_slmp_dev.write_bit(CELL_SET_ADDR,CELL_SET_COOL_FAN_OFFSET,1);
	if(!ret)
	{
		LOGFMTE("set_cool_fan_on register addr:%d offset:%d failed !",CELL_SET_ADDR,CELL_SET_COOL_FAN_OFFSET);
		return false;
	}
	return ret;
}

// ��ȴ���ȃ�
bool PinsGroup_AXMPreCharge::set_cool_fan_off()
{
	bool ret = false;
	ret = m_slmp_dev.write_bit(CELL_SET_ADDR,CELL_SET_COOL_FAN_OFFSET,0);
	if(!ret)
	{
		LOGFMTE("set_cool_fan_off register addr:%d offset:%d failed !",CELL_SET_ADDR,CELL_SET_COOL_FAN_OFFSET);
		return false;
	}
	return ret;
}

// ������
bool PinsGroup_AXMPreCharge::set_air_fan_on()
{
	bool ret = false;
	ret = m_slmp_dev.write_bit(CELL_SET_ADDR,CELL_SET_AIR_FAN_OFFSET,1);
	if(!ret)
	{
		LOGFMTE("set_air_fan_on register addr:%d offset:%d failed !",CELL_SET_ADDR,CELL_SET_AIR_FAN_OFFSET);
		return false;
	}
	return ret;
}

// ������
bool PinsGroup_AXMPreCharge::set_air_fan_off()
{
	bool ret = false;
	ret = m_slmp_dev.write_bit(CELL_SET_ADDR,CELL_SET_AIR_FAN_OFFSET,0);
	if(!ret)
	{
		LOGFMTE("set_air_fan_off register addr:%d offset:%d failed !",CELL_SET_ADDR,CELL_SET_AIR_FAN_OFFSET);
		return false;
	}
	return ret;
}

// ����澯
bool PinsGroup_AXMPreCharge::set_clear_alarm()
{
	bool ret = false;
	ret = m_slmp_dev.write_bit(CELL_SET_ADDR,CELL_SET_CLEAR_ALARM_OFFSET,1);
	if(!ret)
	{
		LOGFMTE("set_clear_alarm register addr:%d offset:%d failed !",CELL_SET_ADDR,CELL_SET_CLEAR_ALARM_OFFSET);
		return false;
	}
	return ret;
}

// ��ʼ����ʹ
bool PinsGroup_AXMPreCharge::set_load()
{
	bool ret = false;
	ret = m_slmp_dev.write_bit(CELL_SET_ADDR,CELL_SET_LOAD_OFFSET,1);
	if(!ret)
	{
		LOGFMTE("set_load register addr:%d offset:%d failed !",CELL_SET_ADDR,CELL_SET_LOAD_OFFSET);
		return false;
	}
	return ret;
}

// �Զ���������
bool PinsGroup_AXMPreCharge::set_auto_start()
{
	bool ret = false;
	ret = m_slmp_dev.write_bit(CELL_SET_ADDR,CELL_SET_AUTO_START_OFFSET,1);
	if(!ret)
	{
		LOGFMTE("set_auto_startregister addr:%d offset:%d failed !",CELL_SET_ADDR,CELL_SET_AUTO_START_OFFSET);
		return false;
	}
	return ret;
}

// ֹͣ����
bool PinsGroup_AXMPreCharge::set_stop()
{
	bool ret = false;
	ret = m_slmp_dev.write_bit(CELL_SET_ADDR,CELL_SET_STOP_OFFSET,1);
	if(!ret)
	{
		LOGFMTE("set_stop addr:%d offset:%d failed !",CELL_SET_ADDR,CELL_SET_STOP_OFFSET);
		return false;
	}
	return ret;
}


// ��λ�¶�
bool PinsGroup_AXMPreCharge::set_cell_temp(float temp[CELL_TEMP_ACQ_NUM])
{
	bool ret = false;
	for(int i =0 ; i < CELL_TEMP_ACQ_NUM ; i++)
	{
		ret = m_slmp_dev.write_register(CELL_TEM_ADDR+i,temp[i]*10);
		if(!ret)
		{
			LOGFMTE("set_cell_temp addr:%d failed !",CELL_TEM_ADDR+i);
			return false;
		}
	}
	return ret;
}

// ���ÿ�λ�¶ȸ澯�Ђ�
bool PinsGroup_AXMPreCharge::set_cell_temp_waring(float temp)
{
	bool ret = false;
	ret = m_slmp_dev.write_register(CELL_TEM_SET_ADDR,temp*10);
	if(!ret)
	{
		LOGFMTE("set_cell_temp_waring addr:%d failed !",CELL_TEM_SET_ADDR);
		return false;
	}
	return ret;
}

// ��ȡ��λ�¶ȸ澯�Ђ�
uint16_t PinsGroup_AXMPreCharge::get_cell_temp_waring()
{
	uint16_t val = 0 ;
	bool ret = false;
	ret = m_slmp_dev.read_register(CELL_TEM_SET_ADDR,val);
	if(!ret)
	{
		LOGFMTE("get_cell_temp_waring addr:%d failed !",CELL_TEM_SET_ADDR);
		return false;
	}
	return val;
}

//�������״��
uint16_t PinsGroup_AXMPreCharge::get_incell_onoff()
{
	uint16_t val = 0 ;
	bool ret = false;
	ret = m_slmp_dev.read_register(INCELL_ONOFF_ADDR,val);
	if(!ret)
	{
		LOGFMTE("get_incell_onoff addr:%d failed !",INCELL_ONOFF_ADDR);
		return false;
	}
	return val;
}

// ��⵽λ֪ͨ
uint16_t PinsGroup_AXMPreCharge::is_incell_status()
{
	uint16_t val =0 ;
	bool ret = false;
	ret = m_slmp_dev.read_register(INCELL_STATUS_ADDR,val);
	if(!ret)
	{
		LOGFMTE("is_incell_status addr:%d failed !",INCELL_STATUS_ADDR);
		return false;
	}
	return val;
}

// ����״̬����
bool PinsGroup_AXMPreCharge::incell_tray_info(uint16_t val)
{
	bool ret = false;
	ret = m_slmp_dev.write_register(INCELL_TRAY_INFO_ADDR,val);
	if(!ret)
	{
		LOGFMTE("incell_tray_info addr:%d failed !",INCELL_TRAY_INFO_ADDR);
		return false;
	}
	return ret;
}

// ֪ͨ���̳���
bool PinsGroup_AXMPreCharge::set_outcell_onoff()
{
	bool ret = false;
	ret = m_slmp_dev.write_register(OUTCELL_ONOFF_ADDR,1);
	if(!ret)
	{
		LOGFMTE("set_outcell_onoff addr:%d failed !",OUTCELL_ONOFF_ADDR);
		return false;
	}
	return ret;
}

// ���̳������
uint16_t PinsGroup_AXMPreCharge::is_outcell_status()
{
	bool ret = false;
	uint16_t val =0 ;
	ret = m_slmp_dev.read_register(OUTCELL_STATUS_ADDR,val);
	if(!ret)
	{
		LOGFMTE("is_outcell_status addr:%d failed !",OUTCELL_STATUS_ADDR);
		return false;
	}
	return val;
}
bool PinsGroup_AXMPreCharge::get_error_code(Fx_Error_Code_t &err_code)
{
	bool ret = false;
	uint16_t err[3];
	// ����
	ret = m_slmp_dev.read_register(SYS_WARING_BASE_ADDR,err[0]);
	if(!ret)
	{
		LOGFMTE("read warn info addr:%d failed !",SYS_WARING_BASE_ADDR);
		return false;
	}

	err_code.PressureError =  (err[0] >> SYS_WARING_PRESSUR_ERROR_OFFSET ) & 1;
	err_code.EStop =  (err[0] >> SYS_WARING_STOP_OFFSET ) & 1;
	err_code.PLCHeartBeat =(err[0] >> SYS_WARING_HEART_OFFSET ) & 1;
	err_code.DoorOpen = (err[0] >> SYS_WARING_DOR_OFFSET ) & 1;

	ret = m_slmp_dev.read_register(CELL_WARING_ADDR1,err[1]);
	if(!ret)
	{
		LOGFMTE("read warn info addr:%d failed !",CELL_WARING_ADDR1);
		return false;
	}
	err_code.MoveUpTimeout = (err[1] >> CELL_WARING_MOVE_UPTIMEOUT_OFFSET ) & 1;
	err_code.MoveDownTimeout = (err[1] >> CELL_WARING_MOVE_DOWNTIMEOUT_OFFSET ) & 1;
	err_code.TrayReverse = (err[1] >> CELL_WARING_TRAY_REVERSE_OFFSET ) & 1;
	err_code.NoTray = (err[1] >> CELL_WARING_NO_TRAY_OFFSET ) & 1;
	err_code.TrayNotReady = (err[1] >> CELL_WARING_TRAY_NOTREADY_OFFSET ) & 1;
	err_code.FanOverload = (err[1] >> CELL_WARING_FAN_OVERLOAD_OFFSET ) & 1;
	err_code.RollerOverload = (err[1] >> CELL_WARING_ROLLER_OVERLOAD_OFFSET ) & 1;
	err_code.RollerRunTimeout = (err[1] >> CELL_WARING_ROLLER_RUN_TIMEOUT_OFFSET ) & 1;
	err_code.RollerReverseRunTimeout = (err[1] >> CELL_WARING_ROLLER_REVERSE_RUN_TIMEOUT_OFFSET ) & 1;
	err_code.LeftFanError = (err[1] >> CELL_WARING_LEFT_FAN_ERROR_OFFSET ) & 1;
	err_code.RightFanError =(err[1] >> CELL_WARING_RIGHT_FAN_ERROR_OFFSET ) & 1;
	err_code.TopFanError = (err[1] >> CELL_WARING_TOP_FAN_ERROR_OFFSET ) & 1;

	ret = m_slmp_dev.read_register(CELL_WARING_ADDR2,err[2]);
	if(!ret)
	{
		LOGFMTE("read warn info addr:%d failed !",CELL_WARING_ADDR2);
		return false;
	}
	err_code.Smoke1Alarm =(err[2] >> CELL_WARING_SMOKE_ALARM1_OFFSET ) & 1;
	err_code.Smoke2Alarm = (err[2] >> CELL_WARING_SMOKE_ALARM2_OFFSET ) & 1;
	err_code.CabSmokeAlarm = (err[2] >> CELL_WARING_CAB_SMOKE_ALARM_OFFSET ) & 1;
	err_code.PinSmokeAlarm = (err[2] >> CELL_WARING_PIN_SMOKE_ALARM_OFFSET ) & 1;
	err_code.PinSmokeHeavyAlarm = (err[2] >> CELL_WARING_PIN_SMOKE_HEAVY_ALARM_OFFSET ) & 1;
	err_code.HighTempAlarm = (err[2] >> CELL_WARING_HIGH_TEMP_ALARM_OFFSET ) & 1;

	return ret;
}

bool PinsGroup_AXMPreCharge::get_status_data(Pins_Status_Data_t &status)
{
	bool ret = false;
	uint16_t val =0 ;
	ret = m_slmp_dev.read_register(CELL_STATUS_ADDR,val);
	if(!ret)
	{
		LOGFMTE("get_status_data addr:%d failed !",CELL_STATUS_ADDR);
		return false;
	}
	
	status.is_auto_mode   = (val >> CELL_STATUS_AUTO_MODE_ON_OFFSET ) & 1;
	status.is_extend      = (val >> CELL_STATUS_TRAY_ISEXTEND_OFFSET ) & 1;
	status.is_retract     = (val >> CELL_STATUS_TRAY_ISRETRACT_OFFSET ) & 1;
	status.is_tool_power  = (val >> CELL_STATUS_TOOL_POWER_ON_OFFSET ) & 1;
	status.is_cool_fan    = (val >> CELL_STATUS_COOL_FAN_ON_OFFSET ) & 1;
	status.is_air_fan     = (val >> CELL_STATUS_AIR_FAN_ON_OFFSET ) & 1;
	status.is_loaded      = (val >> CELL_STATUS_IS_LOADED_OFFSET ) & 1;
	status.is_tray_ready  = (val >> CELL_STATUS_TRAY_READY_OFFSET ) & 1;
	status.is_auto_start  = (val >> CELL_STATUS_IS_AUTO_START_OFFSET ) & 1;
	status.is_can_intray  = get_incell_onoff();
	status.is_tray_inready  = is_incell_status();
	status.is_tray_outready  = is_outcell_status();
	status.is_plc_connect = is_plc_connected;

	return ret;
}



#endif


