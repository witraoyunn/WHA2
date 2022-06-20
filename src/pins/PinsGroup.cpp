#include <sstream>
#include <cmath>
#include "utility.h"
#include "log4z.h"
#include "S7Client.h"
#include "PinsGroup.h"


#ifdef PLC_JS_CT

	#define SYSTEM_REG_ADDR(x)						(12000 + x)					// PLC系统寄存器
	#define CELL_REG_ADDR(a, b)						(12100 + a + ((b-1) * 250))	// 库位相关寄存器, a是偏移量，b是库号
	/* 系统寄存器 */
	#define BEAT_PLUSE_ADDR 						0		// PLC心跳
	#define CABINET_SIGNAL_ADDR 					1		// 电源柜急停
	#define CABINET_TEMP_ADDR 					    2		// 电源柜环境温度一、二
	#define CABINET_AC_STATUS_ADDR 					8		// AC侧报警
	/* 库位寄存器 */
	#define STATE_WORK_MODE_BASE_ADDR				0		// 手自动模式
	#define STATE_CELL_TRAY_BASE_ADDR				1		// 托盘状态
	#define STATE_PINS_BASE_ADDR					2		// 针床状态
	#define CTRL_PINS_BASE_ADDR 					20		// 针床控制
	#define CTRL_POWER_BASE_ADDR					21		// 工装取电控制
	#define SET_CHARGE_STATUS_BASE_ADDR 			22		// 充放电工作状态
	#define READ_TEMPE_BASE_ADDR					36		// 1个热电偶温度、6个库位温度
	#define READ_SMOKE_SENSOR_ADDR					50		// 电源柜、针床烟感告警状态
	#define FIRE_PRO_TEMP_BASE_ADDR 				62		// 热电偶温度阈值设置
	#define C_TEMP_BASE_ADDR						84		// 恒温温度值设置
	#define C_TEMP_DIV_BASE_ADDR					86		// 恒温偏差值设置
	#define ERROR_CODE_BASE_ADDR					94		// 故障状态
	#define CLEAR_WARN_BASE_ADDR					100		// 清除告警
	#define STATE_VAC_BASE_ADDR 					106		// 真空状态
	#define READ_VAC_VALUE_BASE_ADDR				108		// 真空值
	
	#define START_VAC_BASE_ADDR 					150		// 真空启动控制
	#define START_VAC_SEQ_BASE_ADDR 				151		// 真空流程启动控制
	#define SET_VAC_TYPE_BASE_ADDR 					152		// 保压真空阀选择
	#define SET_VAC_VALUE_BASE_ADDR 				154		// 最后真空值设置
	#define SET_KEEP_VAC_VALUE_BASE_ADDR			156		// 保压真空值设置
	#define SET_LEAK_VAC_TIME_BASE_ADDR 			166 	// 泄真空时间设置
	#define SET_VAC_TIME_BASE_ADDR					172		// 流程总时间设置
	#define SET_KEEP_VAC_TIME_BASE_ADDR 			176		// 保压时间设置
	#define SET_KEEP_VAC_AMP_BASE_ADDR				186		// 真空保压幅度值设置
	#define SET_OPEN_VAC_TIME_BASE_ADDR 			188		// 抽真空打开时间设置
	#define SET_CLOSE_VAC_TIME_BASE_ADDR			192		// 抽真空关闭时间设置
	#define SET_LEAK_OPEN_VAC_TIME_BASE_ADDR		196		// 泄真空打开时间设置
	#define SET_LEAK_CLOSE_VAC_TIME_BASE_ADDR		200		// 泄真空关闭时间设置
	#define SET_LEAK_RATIO_BASE_ADDR				204		// 抽真空漏率设置

#elif defined(PLC_JS_VF)

	#define CELL_GET_REG_ADDR(a, b) 				(10 + a + ((b-1) * 10))		// 库位状态寄存器, a是偏移量，b是库号
	#define CELL_SET_REG_ADDR(a, b) 				(150 + a + ((b-1) * 40)) 	// 库位动作寄存器, a是偏移量，b是库号
	/* 系统、其他寄存器 */
	#define GET_PLC_INFO_ADDR 						6		// 系统状态信息
	#define GET_VAC_VALUE_ADDR						100		// 100~106 第一~四层负压压力
	#define SET_EXHAUST_SMOKE_ADDR					136 	// 强制排烟
	#define SET_FIRE_FAN_TIMER_ADDR					138 	// 消防风扇电源关闭时间
	/* 库位状态寄存器 */
	#define GET_ALARM_INFO_ADDR						4		// 告警信息
	#define GET_STATUS_INFO_ADDR					6		// 状态信息
	/* 库位控制寄存器 */
	#define SET_PLC_WORK_STATUS_ADDR				4		// PLC工作模式设置
	#define SET_TRAY_ADDR							6		// 托盘压合/脱开
	#define SET_HVAC_ONOFF_ADDR						8		// 高真空阀开/关
	#define SET_LVAC_ONOFF_ADDR						10		// 低真空阀开/关
	#define SET_LEAK_VAC_ONOFF_ADDR					12		// 破真空阀开/关
	#define SET_SMOKE_ALARM_ADDR					14		// 烟雾告警
	#define CLEAN_ALARM_ADDR						18		// 清除异常
	#define SET_POWER_ONOFF_ADDR					20		// 工装取电开/关
	#define SET_FIRE_FAN_ONOFF_ADDR					22		// 消防风扇开/关
	#define SET_FIRE_DOR_ONOFF_ADDR					24		// 消防门开/关
	#define SET_HVAC_MAX_VALUE_ADDR					26		// 高真空范围最大值
	#define SET_HVAC_ON_TIME_ADDR					28		// 高真空抽气时间
	#define SET_HVAC_OFF_TIME_ADDR					30		// 高真空停顿时间
	#define SET_HVAC_WORK_MODE_ADDR					32		// 高真空工作模式
	#define SET_LVAC_MAX_VALUE_ADDR 				26		// 低真空范围最大值
	#define SET_LVAC_ON_TIME_ADDR					36		// 低真空抽气时间
	#define SET_LVAC_OFF_TIME_ADDR					38		// 低真空停顿时间
	#define SET_LVAC_WORK_MODE_ADDR 				40		// 低真空工作模式
	
#elif defined(PLC_JYT)

	#define SYSTEM_REG_ADDR(x)						(0 + x) 			// PLC系统寄存器
	#define CELL_REG_ADDR(a, b) 					(a + (b-1) * 86) 	// 库位相关寄存器, a是偏移量，b是库号
	/* 系统寄存器 */
	#define BEAT_PLUSE_ADDR 						0		// bit 0: 心跳
	#define CABINET_SIGNAL_ADDR 					0		// bit 1: 急停
	#define GET_GLOBAL_ALARM_ADDR					8		// 全局告警点位
	/* 库位寄存器 */
	#define CTRL_PB_BASE_ADDR 						72		// 按钮（控制点位）
	#define FIRE_PRO_TEMP_BASE_ADDR 				74		// 火警温度阈值设置
	#define C_TEMP_BASE_ADDR 						76		// 环境温度阈值设置 (恒温)
	#define C_TEMP_DIV_BASE_ADDR					78		// 环境温度容差设置 (恒温)
	#define SET_KEEP_VAC_AMP_BASE_ADDR				80 		// 负压容差设置
	#define SET_LEAK_RATIO_BASE_ADDR				82 		// 漏率负压值设置
	#define SET_VAC_VALUE_BASE_ADDR 				84 		// 自动负压压力设置
	#define SET_LEAK_VAC_VALUE_BASE_ADDR 			86		// 漏率判定值设置
	#define SET_VAC_LKG_BOUNDARY_BASE_ADDR 			88		// 高低负压分界值设置
	#define SET_VAC_LKG_LT_BASE_ADDR				90		// 负压漏率测量时间设置
	#define SET_OPEN_HVAC_TIME_BASE_ADDR 			94 		// 高真空打开时间设置
	#define SET_CLOSE_HVAC_TIME_BASE_ADDR			98 		// 高真空关闭时间设置
	#define SET_OPEN_LVAC_TIME_BASE_ADDR			102		// 低真空打开时间设置
	#define SET_CLOSE_LVAC_TIME_BASE_ADDR			106		// 低真空关闭时间设置
	#define SET_LEAK_OPEN_VAC_TIME_BASE_ADDR		110		// 泄真空打开时间设置
	#define SET_LEAK_CLOSE_VAC_TIME_BASE_ADDR		114		// 泄真空关闭时间设置
	#define SET_GONGZHUANG_ID_BASE_ADDR				118		// 工装编号设置

	#define GET_ACT_INDICATE_BASE_ADDR				122		// 动作指示
	#define GET_STATUS_INDICATE_BASE_ADDR			124		// 状态指示
	#define GET_CURRUNT_STEP_NO_BASE_ADDR			128		// 当前步编号
	#define READ_VAC_VALUE_BASE_ADDR				130		// 当前负压值
	#define ERROR_CODE_BASE_ADDR					132		// A、B、C类故障，68~76
	#define READ_TEMPE_BASE_ADDR					142		// 7个库位温度 1个火警温度，78~92
	
#else

	#define BEAT_PLUSE_ADDR 						0
	#define GROUP_MEM_SIZE 							80
	#define CABINET_SIGNAL_ADDR 					10
	#define CABINET_TEMP1_ADDR 						14
	#define CABINET_TEMP2_ADDR 						18

//#define CTRL_MODE_BASE_ADDR         1
//#define ERROR_CODE_BASE_ADDR        2
	#define STATE_CELL_TRAY_BASE_ADDR 				3
	#define CTRL_PINS_BASE_ADDR 					4
	#define STATE_PINS_BASE_ADDR 					5
	#define START_VAC_BASE_ADDR 					6
	#define STATE_VAC_BASE_ADDR 					7
	#define CLEAR_WARN_BASE_ADDR 					8
	#define START_1230_BASE_ADDR 					9
	#define STATE_1230_BASE_ADDR 					10
//#define STATE_SMOKE_BASE_ADDR       11
	#define SET_VAC_VALUE_BASE_ADDR 				14
	#define SET_KEEP_VAC_VALUE_BASE_ADDR 			18
	#define READ_VAC_VALUE_BASE_ADDR 				22
	#define SET_VAC_TIME_BASE_ADDR 					26

	#define SET_KEEP_VAC_TIME_BASE_ADDR 			42
	#define READ_TEMPE_BASE_ADDR 					46

	#define START_VAC_SEQ_BASE_ADDR 				1508
	#define SET_KEEP_VAC_AMP_BASE_ADDR 				1510
	#define START_1230_FANS_BASE_ADDR 				1518
	#define ERROR_CODE_BASE_ADDR 					1520
	#define SET_OPEN_VAC_TIME_BASE_ADDR 			1528
	#define SET_CLOSE_VAC_TIME_BASE_ADDR 			1532

	#define SET_LEAK_OPEN_VAC_TIME_BASE_ADDR 		1536
	#define SET_LEAK_CLOSE_VAC_TIME_BASE_ADDR 		1540
	#define SET_LEAK_RATIO_BASE_ADDR 				1544
	#define SET_LEAK_VAC_TIME_BASE_ADDR 			1548

#endif

using namespace std;


PinsGroup::PinsGroup(int group_no)
    : m_group(group_no)
{
#ifdef PLC_JYT
	memset((void *)&m_read_reg, 0, sizeof(m_read_reg));
#endif
}

PinsGroup &PinsGroup::operator=(const PinsGroup &obj)
{
	if (this != &obj)
	{
		this->m_group    = obj.m_group;
#ifdef PLC_JYT
		this->m_read_reg = obj.m_read_reg;
#endif
	}

	return *this;
}

PinsGroup::~PinsGroup()
{
}

bool PinsGroup::query_heartbeat_signal()
{
	static int     cnt = 0;
	static uint8_t lastVal = 0;
	uint8_t        currVal = 0;
	
#ifdef SIEMENS_PLC
	currVal = S7Client::instance()->read_byte(BEAT_PLUSE_ADDR);
	return true;
#elif defined(PLC_JS_CT)
	currVal = S7Client::instance()->read_byte(SYSTEM_REG_ADDR(BEAT_PLUSE_ADDR));
#elif defined(PLC_JS_VF)
	currVal = S7Client::instance()->read_bit(GET_PLC_INFO_ADDR, 0);
#elif defined(PLC_JYT)
	currVal = S7Client::instance()->read_byte(SYSTEM_REG_ADDR(BEAT_PLUSE_ADDR)) & 0x01;
#endif

//	LOGFMTD("heartbeat: %02x.", currVal);
	if (currVal == lastVal)
	{
		if (++cnt > 10)			// 10S
		{
			cnt = 0;
			return false;
		}
	}
	else
	{
		cnt     = 0;
		lastVal = currVal;
	}

	return true;
}

bitset<8> PinsGroup::query_cabinet_signal()
{
	bitset<8> ret(0x00);
	
#ifdef SIEMENS_PLC
	uint8_t status = S7Client::instance()->read_byte(CABINET_SIGNAL_ADDR);
	bitset<2> bitvec(status);
	ret[0] = bitvec[0];
	ret[1] = bitvec[1];
#elif defined(PLC_JS_CT)
	// bit依次为：电源柜急停、电源柜断电、AC侧过流、AC侧过压、AC侧欠压
	uint8_t status_1 = S7Client::instance()->read_byte(SYSTEM_REG_ADDR(CABINET_SIGNAL_ADDR));
	bitset<1> bitvec_1(status_1);
	ret[0] = bitvec_1[0];

	uint8_t status_2 = S7Client::instance()->read_byte(SYSTEM_REG_ADDR(CABINET_AC_STATUS_ADDR));
	bitset<4> bitvec_2(status_2);
	ret[1] = bitvec_2[0];
	ret[2] = bitvec_2[1];
	ret[3] = bitvec_2[2];
	ret[4] = bitvec_2[3];
#elif defined(PLC_JS_VF)
	// bit5：电箱急停，bit6：面板急停
	uint8_t status = S7Client::instance()->read_byte(7);
	bitset<8> bitvec(status);
	ret[0] = bitvec[5];
	ret[1] = bitvec[6];
#endif

	return ret;
}

#if defined(PLC_JYT)
// 读取全局告警信息
void PinsGroup::query_global_alarm(uint16_t err[2])
{
	S7Client::instance()->read_byte(SYSTEM_REG_ADDR(GET_GLOBAL_ALARM_ADDR), 
		(uint8_t *)err, sizeof(err));
	for (int i=0; i<2; i++)
	{
		endian_swap((uint8_t *)&err[i], 0, sizeof(uint16_t));
	}
}
#endif

// 读取电源柜温度	
#if defined(PLC_JYT)

void PinsGroup::query_power_cabinet_temp(float temp[16])
{
	uint16_t buf[16];
	
	S7Client::instance()->read_byte(2, 0, (uint8_t *)&buf, sizeof(buf));
	for (int i=0; i<16; i++)
	{
		endian_swap((uint8_t *)&buf[i], 0, sizeof(uint16_t));
		temp[i] = (float)buf[i] / 10.0;
	}
}

#elif defined(PLC_JS_CT)

void PinsGroup::query_power_cabinet_temp(float temp[4])
{
	for (int i=0; i<2; ++i)
	{
		uint8_t val_0 = S7Client::instance()->read_byte(SYSTEM_REG_ADDR(CABINET_TEMP_ADDR + i*2));
		uint8_t val_1 = S7Client::instance()->read_byte(SYSTEM_REG_ADDR(CABINET_TEMP_ADDR + i*2 + 1));
		temp[i] = (float)((int)val_1 | (int)val_0 << 8) / 10.0;
	}

	for (int i=2; i<4; ++i)
	{
		uint8_t val_0 = S7Client::instance()->read_byte(SYSTEM_REG_ADDR(CABINET_TEMP_ADDR + 3 + i*2));
		uint8_t val_1 = S7Client::instance()->read_byte(SYSTEM_REG_ADDR(CABINET_TEMP_ADDR + 3 + i*2 + 1));
		temp[i] = (float)((int)val_1 | (int)val_0 << 8) / 10.0;
	}
}

#endif

#ifdef PLC_JYT

void PinsGroup::set_action_bits(int bit_no, bool val)
{
	uint16_t action = S7Client::instance()->read_word(CELL_REG_ADDR(CTRL_PB_BASE_ADDR, m_group));
	if (val)
	{
		action |= 1 << bit_no;
	}
	else
	{
		action &= ~(1 << bit_no);
	}
	S7Client::instance()->write(CELL_REG_ADDR(CTRL_PB_BASE_ADDR, m_group), action);
}

bool PinsGroup::get_status_bits(int bit_no)
{	
	return ((m_read_reg.lampWord >> bit_no) & 1) != 0;
}

// 初始化
void PinsGroup::set_plc_init(bool flag)
{
	uint16_t action = 0;
	if (flag)
	{
		action |= 1 << 9;
	}
	S7Client::instance()->write(CELL_REG_ADDR(CTRL_PB_BASE_ADDR, m_group), action);
}

// 设置工装编号
void PinsGroup::set_device_type(int type)
{
	uint16 val = type;
	S7Client::instance()->write(CELL_REG_ADDR(SET_GONGZHUANG_ID_BASE_ADDR, m_group), val);
}

// 获取当前工步号
int PinsGroup::get_step_no()
{
	return m_read_reg.step;
}

// 泄漏率测试Enable/Disable
void PinsGroup::enable_vac_leak(bool en)
{
	set_action_bits(10, en);
}

// A级消防控制
void PinsGroup::fire_protect_A(bool on)
{
	set_action_bits(6, on);
}

// B级消防控制
void PinsGroup::fire_protect_B(bool on)
{
	set_action_bits(7, on);
}

#endif

// 设置充放电状态
void PinsGroup::set_chargeDischarge_status(bool flag)
{
#if defined(PLC_JS_CT)
	S7Client::instance()->write_bit(CELL_REG_ADDR(SET_CHARGE_STATUS_BASE_ADDR, m_group), 0, flag);
#elif defined(PLC_JYT)
	set_action_bits(5, flag);
#endif
}

// 工装供电
void PinsGroup::power_ON()
{
#ifdef SIEMENS_PLC
	S7Client::instance()->write_bit((CTRL_PINS_BASE_ADDR + m_group * GROUP_MEM_SIZE), 2, true);
#elif defined(PLC_JS_CT)
	S7Client::instance()->write(CELL_REG_ADDR(CTRL_POWER_BASE_ADDR, m_group), (uint8_t)2);
#elif defined(PLC_JS_VF)
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_POWER_ONOFF_ADDR, m_group), (uint16_t)1);
//#elif defined(PLC_JYT)
//	set_action_bits(4, false);
//	set_action_bits(3, true);
#endif
}

// 工装断电
void PinsGroup::power_OFF()
{
#ifdef SIEMENS_PLC
	S7Client::instance()->write_bit((CTRL_PINS_BASE_ADDR + m_group * GROUP_MEM_SIZE), 2, false);
#elif defined(PLC_JS_CT)
	S7Client::instance()->write(CELL_REG_ADDR(CTRL_POWER_BASE_ADDR, m_group), (uint8_t)1);
#elif defined(PLC_JS_VF)
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_POWER_ONOFF_ADDR, m_group), (uint16_t)2);
//#elif defined(PLC_JYT)
//	set_action_bits(3, false);
//	set_action_bits(4, true);
#endif
}

// 托盘上升
void PinsGroup::pins_extend()
{
#ifdef SIEMENS_PLC
	S7Client::instance()->write_bit((CTRL_PINS_BASE_ADDR + m_group * GROUP_MEM_SIZE), 0, true);
#elif defined(PLC_JS_CT)
	S7Client::instance()->write(CELL_REG_ADDR(CTRL_PINS_BASE_ADDR, m_group), (uint8_t)2);
#elif defined(PLC_JS_VF)
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_TRAY_ADDR, m_group), (uint16_t)1);
#elif defined(PLC_JYT)
	set_action_bits(2, false);
	set_action_bits(1, true);
#endif
}

// 托盘下降
void PinsGroup::pins_retract()
{
#ifdef SIEMENS_PLC
	S7Client::instance()->write_bit(CTRL_PINS_BASE_ADDR + m_group * GROUP_MEM_SIZE, 0, false);
#elif defined(PLC_JS_CT)
	S7Client::instance()->write(CELL_REG_ADDR(CTRL_PINS_BASE_ADDR, m_group), (uint8_t)1);
#elif defined(PLC_JS_VF)
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_TRAY_ADDR, m_group), (uint16_t)2);
#elif defined(PLC_JYT)
	set_action_bits(1, false);
	set_action_bits(2, true);
#endif
}

// 开高真空
void PinsGroup::open_high_vac()
{
#ifdef SIEMENS_PLC
	uint8 value = S7Client::instance()->read_byte(START_VAC_BASE_ADDR + m_group * GROUP_MEM_SIZE);
	uint8 command_id = value | 0x01;
	S7Client::instance()->write(START_VAC_BASE_ADDR + m_group * GROUP_MEM_SIZE, command_id);
#elif defined(PLC_JS_CT)
	S7Client::instance()->write(CELL_REG_ADDR(START_VAC_BASE_ADDR, m_group), (uint8_t)1);
#elif defined(PLC_JS_VF)
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_HVAC_ONOFF_ADDR, m_group), (uint16_t)1);
#endif
}

// 关高真空
void PinsGroup::close_high_vac()
{
#ifdef SIEMENS_PLC
	uint8 value = S7Client::instance()->read_byte(START_VAC_BASE_ADDR + m_group * GROUP_MEM_SIZE);
	uint8 command_id = value & 0xFE;
	S7Client::instance()->write(START_VAC_BASE_ADDR + m_group * GROUP_MEM_SIZE, command_id);
#elif defined(PLC_JS_CT)
	S7Client::instance()->write(CELL_REG_ADDR(START_VAC_BASE_ADDR, m_group), (uint8_t)0);
#elif defined(PLC_JS_VF)
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_HVAC_ONOFF_ADDR, m_group), (uint16_t)2);
#endif
}

// 开低真空
void PinsGroup::open_low_vac()
{
#ifdef SIEMENS_PLC
	uint8 value = S7Client::instance()->read_byte(START_VAC_BASE_ADDR + m_group * GROUP_MEM_SIZE);
	uint8 command_id = value | 0x02;
	S7Client::instance()->write(START_VAC_BASE_ADDR + m_group * GROUP_MEM_SIZE, command_id);
#elif defined(PLC_JS_CT)
	S7Client::instance()->write(CELL_REG_ADDR(START_VAC_BASE_ADDR, m_group), (uint8_t)2);
#elif defined(PLC_JS_VF)
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_LVAC_ONOFF_ADDR, m_group), (uint16_t)1);
#endif
}

// 关低真空
void PinsGroup::close_low_vac()
{
#ifdef SIEMENS_PLC
	uint8 value = S7Client::instance()->read_byte(START_VAC_BASE_ADDR + m_group * GROUP_MEM_SIZE);
	uint8 command_id = value & 0xFD;
	S7Client::instance()->write(START_VAC_BASE_ADDR + m_group * GROUP_MEM_SIZE, command_id);
#elif defined(PLC_JS_CT)
	S7Client::instance()->write(CELL_REG_ADDR(START_VAC_BASE_ADDR, m_group), (uint8_t)0);
#elif defined(PLC_JS_VF)
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_LVAC_ONOFF_ADDR, m_group), (uint16_t)2);
#endif
}

// 开泄真空
void PinsGroup::open_minus_vac()
{
#ifdef SIEMENS_PLC
	uint8 value = S7Client::instance()->read_byte(START_VAC_BASE_ADDR + m_group * GROUP_MEM_SIZE);
	uint8 command_id = value | 0x08;
	S7Client::instance()->write(START_VAC_BASE_ADDR + m_group * GROUP_MEM_SIZE, command_id);
#elif defined(PLC_JS_CT)
	S7Client::instance()->write(CELL_REG_ADDR(START_VAC_BASE_ADDR, m_group), (uint8_t)4);
//#elif defined(PLC_JYT)
//	set_action_bits(14, true);
#endif
}

// 关泄真空
void PinsGroup::close_minus_vac()
{
#ifdef SIEMENS_PLC
	uint8 value = S7Client::instance()->read_byte(START_VAC_BASE_ADDR + m_group * GROUP_MEM_SIZE);
	uint8 command_id = value & 0xF7;
	S7Client::instance()->write(START_VAC_BASE_ADDR + m_group * GROUP_MEM_SIZE, command_id);
#elif defined(PLC_JS_CT)
	S7Client::instance()->write(CELL_REG_ADDR(START_VAC_BASE_ADDR, m_group), (uint8_t)0);
//#elif defined(PLC_JYT)
//	set_action_bits(14, false);
#endif
}

// 启动负压工艺
void PinsGroup::start_vac_seq()
{
#ifdef SIEMENS_PLC
	uint8 command_id = 10;
	S7Client::instance()->write(START_VAC_SEQ_BASE_ADDR + m_group * GROUP_MEM_SIZE, command_id);
#elif defined(PLC_JS_CT)
	S7Client::instance()->write(CELL_REG_ADDR(START_VAC_SEQ_BASE_ADDR, m_group), (uint8_t)1);
#elif defined(PLC_JS_VF)

#elif defined(PLC_JYT)
	set_action_bits(13, false);
	set_action_bits(11, true);
#endif
}

// 结束负压工艺
void PinsGroup::stop_vac_seq()
{
#ifdef SIEMENS_PLC
	uint8 command_id = 11;
	S7Client::instance()->write(START_VAC_SEQ_BASE_ADDR + m_group * GROUP_MEM_SIZE, command_id);
#elif defined PLC_JS_CT
	S7Client::instance()->write(CELL_REG_ADDR(START_VAC_SEQ_BASE_ADDR, m_group), (uint8_t)0);
#elif defined(PLC_JS_VF)
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_HVAC_ONOFF_ADDR, m_group), (uint8_t)2);
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_LVAC_ONOFF_ADDR, m_group), (uint8_t)2);
#elif defined(PLC_JYT)
	set_action_bits(11, false);
	set_action_bits(13, true);
#endif
}

// 設置負壓值
void PinsGroup::set_vac_value(float value)
{
#ifdef PLC_JYT
	int val;

	val = (int)value*10;
	S7Client::instance()->write(CELL_REG_ADDR(SET_VAC_VALUE_BASE_ADDR, m_group), (uint8_t)((val >> 8) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_VAC_VALUE_BASE_ADDR + 1, m_group), (uint8_t)(val & 0xFF));
#endif
}

// 设置负压参数
void PinsGroup::set_vac_params(float *vac_param)
{	
#ifdef SIEMENS_PLC

	S7Client::instance()->write_bit(START_VAC_BASE_ADDR + m_group * GROUP_MEM_SIZE, 5, vac_param[PINS_VAC_TAP]);
	S7Client::instance()->write(SET_VAC_TIME_BASE_ADDR + m_group * GROUP_MEM_SIZE, vac_param[PINS_PUMP_VAC_TIME]);
	S7Client::instance()->write(SET_OPEN_VAC_TIME_BASE_ADDR + m_group * GROUP_MEM_SIZE, vac_param[PINS_OPEN_VAC_TIME]);
	S7Client::instance()->write(SET_CLOSE_VAC_TIME_BASE_ADDR + m_group * GROUP_MEM_SIZE, vac_param[PINS_CLOSE_VAC_TIME]);
	S7Client::instance()->write(SET_VAC_VALUE_BASE_ADDR + m_group * GROUP_MEM_SIZE, vac_param[PINS_VAC_VALUE]);
	S7Client::instance()->write(SET_KEEP_VAC_TIME_BASE_ADDR + m_group * GROUP_MEM_SIZE, vac_param[PINS_KEEP_VAC_TIME]);
	S7Client::instance()->write(SET_KEEP_VAC_VALUE_BASE_ADDR + m_group * GROUP_MEM_SIZE, vac_param[PINS_KEEP_VAC_VALUE]);
	S7Client::instance()->write(SET_KEEP_VAC_AMP_BASE_ADDR + m_group * GROUP_MEM_SIZE, vac_param[PINS_VAC_TOL]);

	S7Client::instance()->write(SET_LEAK_OPEN_VAC_TIME_BASE_ADDR + m_group * GROUP_MEM_SIZE, vac_param[PINS_LEAK_OPEN_VAC_TIME]);
	S7Client::instance()->write(SET_LEAK_CLOSE_VAC_TIME_BASE_ADDR + m_group * GROUP_MEM_SIZE, vac_param[PINS_LEAK_CLOSE_VAC_TIME]);
	S7Client::instance()->write(SET_LEAK_RATIO_BASE_ADDR + m_group * GROUP_MEM_SIZE, vac_param[PINS_LEAK_RATIO]);

#elif defined(PLC_JYT)
	int   i   = 0;
	float d   = 0;
	int   val = 0;

	// 漏率负压值
	i	= (int) vac_param[PINS_KEEP_VAC_VALUE];
	d	= vac_param[PINS_KEEP_VAC_VALUE] - (float)i;
	val = (i * 10) + (int)(d * 10.0);
	S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_RATIO_BASE_ADDR, m_group), (uint8_t)((val >> 8) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_RATIO_BASE_ADDR + 1, m_group), (uint8_t)(val & 0xFF));
	// 漏率测试时间
	i	= (int) vac_param[PINS_KEEP_VAC_TIME];
	d	= vac_param[PINS_KEEP_VAC_TIME] - (float)i;
	val = (i * 1000) + (int)(d * 1000.0);
	val *= 60;	// 上位机下发的单位是分钟
	S7Client::instance()->write(CELL_REG_ADDR(SET_VAC_LKG_LT_BASE_ADDR, m_group), (uint8_t)((val >> 24) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_VAC_LKG_LT_BASE_ADDR + 1, m_group), (uint8_t)((val >> 16) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_VAC_LKG_LT_BASE_ADDR + 2, m_group), (uint8_t)((val >> 8) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_VAC_LKG_LT_BASE_ADDR + 3, m_group), (uint8_t)(val & 0xFF));
	// 漏率容错值
	i	= (int) vac_param[PINS_VAC_TOL];
	d	= vac_param[PINS_VAC_TOL] - (float)i;
	val = (i * 10) + (int)(d * 10.0);
	S7Client::instance()->write(CELL_REG_ADDR(SET_KEEP_VAC_AMP_BASE_ADDR, m_group), (uint8_t)((val >> 8) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_KEEP_VAC_AMP_BASE_ADDR + 1, m_group), (uint8_t)(val & 0xFF));
	// 漏率判定值
	i	= (int) vac_param[PINS_LEAK_RATIO];
	d	= vac_param[PINS_LEAK_RATIO] - (float)i;
	val = (i * 10) + (int)(d * 10.0);
	S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_VAC_VALUE_BASE_ADDR, m_group), (uint8_t)((val >> 8) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_VAC_VALUE_BASE_ADDR + 1, m_group), (uint8_t)(val & 0xFF));

	// 抽真空开时间
	i	= (int) vac_param[PINS_OPEN_VAC_TIME];
	d	= vac_param[PINS_OPEN_VAC_TIME] - (float)i;
	val = (i * 1000) + (int)(d * 1000.0);
	S7Client::instance()->write(CELL_REG_ADDR(SET_OPEN_HVAC_TIME_BASE_ADDR, m_group), (uint8_t)((val >> 24) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_OPEN_HVAC_TIME_BASE_ADDR + 1, m_group), (uint8_t)((val >> 16) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_OPEN_HVAC_TIME_BASE_ADDR + 2, m_group), (uint8_t)((val >> 8) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_OPEN_HVAC_TIME_BASE_ADDR + 3, m_group), (uint8_t)(val & 0xFF));
	// 抽真空关时间
	i	= (int) vac_param[PINS_CLOSE_VAC_TIME];
	d	= vac_param[PINS_CLOSE_VAC_TIME] - (float)i;
	val = (i * 1000) + (int)(d * 1000.0);
	S7Client::instance()->write(CELL_REG_ADDR(SET_CLOSE_HVAC_TIME_BASE_ADDR, m_group), (uint8_t)((val >> 24) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_CLOSE_HVAC_TIME_BASE_ADDR + 1, m_group), (uint8_t)((val >> 16) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_CLOSE_HVAC_TIME_BASE_ADDR + 2, m_group), (uint8_t)((val >> 8) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_CLOSE_HVAC_TIME_BASE_ADDR + 3, m_group), (uint8_t)(val & 0xFF));
//	// 抽真空时长
//	i	= (int) vac_param[PINS_PUMP_VAC_TIME];
//	d	= vac_param[PINS_PUMP_VAC_TIME] - (float)i;
//	val = (i * 1000) + (int)(d * 1000.0);
//	S7Client::instance()->write(CELL_REG_ADDR(SET_VAC_TIME_BASE_ADDR, m_group), (uint8_t)((val >> 24) & 0xFF));
//	S7Client::instance()->write(CELL_REG_ADDR(SET_VAC_TIME_BASE_ADDR + 1, m_group), (uint8_t)((val >> 16) & 0xFF));
//	S7Client::instance()->write(CELL_REG_ADDR(SET_VAC_TIME_BASE_ADDR + 2, m_group), (uint8_t)((val >> 8) & 0xFF));
//	S7Client::instance()->write(CELL_REG_ADDR(SET_VAC_TIME_BASE_ADDR + 3, m_group), (uint8_t)(val & 0xFF));
	// 最后真空值
	i	= (int) vac_param[PINS_VAC_VALUE];
	d	= vac_param[PINS_VAC_VALUE] - (float)i;
	val = (i * 10) + (int)(d * 10.0);
	S7Client::instance()->write(CELL_REG_ADDR(SET_VAC_VALUE_BASE_ADDR, m_group), (uint8_t)((val >> 8) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_VAC_VALUE_BASE_ADDR + 1, m_group), (uint8_t)(val & 0xFF));
	// 保压容错值
	i	= (int) vac_param[PINS_VAC_TOL];
	d	= vac_param[PINS_VAC_TOL] - (float)i;
	val = (i * 10) + (int)(d * 10.0);
	S7Client::instance()->write(CELL_REG_ADDR(SET_KEEP_VAC_AMP_BASE_ADDR, m_group), (uint8_t)((val >> 8) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_KEEP_VAC_AMP_BASE_ADDR + 1, m_group), (uint8_t)(val & 0xFF));

	// 泄真空开时间
	i	= (int) vac_param[PINS_LEAK_OPEN_VAC_TIME];
	d	= vac_param[PINS_LEAK_OPEN_VAC_TIME] - (float)i;
	val = (i * 1000) + (int)(d * 1000.0);
	S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_OPEN_VAC_TIME_BASE_ADDR, m_group), (uint8_t)((val >> 24) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_OPEN_VAC_TIME_BASE_ADDR + 1, m_group), (uint8_t)((val >> 16) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_OPEN_VAC_TIME_BASE_ADDR + 2, m_group), (uint8_t)((val >> 8) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_OPEN_VAC_TIME_BASE_ADDR + 3, m_group), (uint8_t)(val & 0xFF));

	// 泄真空关时间
	i	= (int) vac_param[PINS_LEAK_CLOSE_VAC_TIME];
	d	= vac_param[PINS_LEAK_CLOSE_VAC_TIME] - (float)i;
	val = (i * 1000) + (int)(d * 1000.0);
	S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_CLOSE_VAC_TIME_BASE_ADDR, m_group), (uint8_t)((val >> 24) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_CLOSE_VAC_TIME_BASE_ADDR + 1, m_group), (uint8_t)((val >> 16) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_CLOSE_VAC_TIME_BASE_ADDR + 2, m_group), (uint8_t)((val >> 8) & 0xFF));
	S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_CLOSE_VAC_TIME_BASE_ADDR + 3, m_group), (uint8_t)(val & 0xFF));

#elif defined(PLC_JS_CT)
	int   i   = 0;
	float d   = 0;
	int   val = 0;

	// 通过真空类型参数判断下发的是抽真空参数，还是泄真空参数
	if (vac_param[PINS_VAC_TYPE_SELECT] < 0.0)
	{
		// 泄真空开时间
		if (vac_param[PINS_VAC_TYPE_SELECT] == 0.0)						// 脉冲式
		{
			i	= (int) vac_param[PINS_LEAK_OPEN_VAC_TIME];
			d	= vac_param[PINS_LEAK_OPEN_VAC_TIME] - (float)i;
			val = (i * 1000) + (int)(d * 1000.0);
		}
		else if (vac_param[PINS_VAC_TYPE_SELECT] == 1.0)				// 直接式
		{
			val = 1 * 1000;
		}
		else
		{
			return;
		}
		S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_OPEN_VAC_TIME_BASE_ADDR, m_group), (uint8_t)((val >> 24) & 0xFF));
		S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_OPEN_VAC_TIME_BASE_ADDR + 1, m_group), (uint8_t)((val >> 16) & 0xFF));
		S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_OPEN_VAC_TIME_BASE_ADDR + 2, m_group), (uint8_t)((val >> 8) & 0xFF));
		S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_OPEN_VAC_TIME_BASE_ADDR + 3, m_group), (uint8_t)(val & 0xFF));

		// 泄真空关时间
		if (vac_param[PINS_VAC_TYPE_SELECT] == 0.0)						// 脉冲式
		{
			i	= (int) vac_param[PINS_LEAK_CLOSE_VAC_TIME];
			d	= vac_param[PINS_LEAK_CLOSE_VAC_TIME] - (float)i;
			val = (i * 1000) + (int)(d * 1000.0);
		}
		else if (vac_param[PINS_VAC_TYPE_SELECT] == 1.0)				// 直接式
		{
			val = 0 * 1000;
		}
		
		S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_CLOSE_VAC_TIME_BASE_ADDR, m_group), (uint8_t)((val >> 24) & 0xFF));
		S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_CLOSE_VAC_TIME_BASE_ADDR + 1, m_group), (uint8_t)((val >> 16) & 0xFF));
		S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_CLOSE_VAC_TIME_BASE_ADDR + 2, m_group), (uint8_t)((val >> 8) & 0xFF));
		S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_CLOSE_VAC_TIME_BASE_ADDR + 3, m_group), (uint8_t)(val & 0xFF));
	}
	else
	{
		// 抽真空开时间
		if (vac_param[PINS_VAC_TYPE_SELECT] == 0.0)						// 脉冲式
		{
			i	= (int) vac_param[PINS_OPEN_VAC_TIME];
			d	= vac_param[PINS_OPEN_VAC_TIME] - (float)i;
			val = (i * 1000) + (int)(d * 1000.0);
		}
		else if (vac_param[PINS_VAC_TYPE_SELECT] == 1.0)				// 两段式
		{
			val = 1 * 1000;
		}
		else
		{
			return;
		}
		S7Client::instance()->write(CELL_REG_ADDR(SET_OPEN_VAC_TIME_BASE_ADDR, m_group), (uint8_t)((val >> 24) & 0xFF));
		S7Client::instance()->write(CELL_REG_ADDR(SET_OPEN_VAC_TIME_BASE_ADDR + 1, m_group), (uint8_t)((val >> 16) & 0xFF));
		S7Client::instance()->write(CELL_REG_ADDR(SET_OPEN_VAC_TIME_BASE_ADDR + 2, m_group), (uint8_t)((val >> 8) & 0xFF));
		S7Client::instance()->write(CELL_REG_ADDR(SET_OPEN_VAC_TIME_BASE_ADDR + 3, m_group), (uint8_t)(val & 0xFF));
		// 抽真空关时间
		if (vac_param[PINS_VAC_TYPE_SELECT] == 0.0)						// 脉冲式
		{
			i	= (int) vac_param[PINS_CLOSE_VAC_TIME];
			d	= vac_param[PINS_CLOSE_VAC_TIME] - (float)i;
			val = (i * 1000) + (int)(d * 1000.0);
		}
		else if (vac_param[PINS_VAC_TYPE_SELECT] == 1.0)				// 两段式
		{
			val = 1 * 1000;
		}
		S7Client::instance()->write(CELL_REG_ADDR(SET_CLOSE_VAC_TIME_BASE_ADDR, m_group), (uint8_t)((val >> 24) & 0xFF));
		S7Client::instance()->write(CELL_REG_ADDR(SET_CLOSE_VAC_TIME_BASE_ADDR + 1, m_group), (uint8_t)((val >> 16) & 0xFF));
		S7Client::instance()->write(CELL_REG_ADDR(SET_CLOSE_VAC_TIME_BASE_ADDR + 2, m_group), (uint8_t)((val >> 8) & 0xFF));
		S7Client::instance()->write(CELL_REG_ADDR(SET_CLOSE_VAC_TIME_BASE_ADDR + 3, m_group), (uint8_t)(val & 0xFF));
		// 抽真空时长
		i   = (int) vac_param[PINS_PUMP_VAC_TIME];
		d   = vac_param[PINS_PUMP_VAC_TIME] - (float)i;
		val = (i * 1000) + (int)(d * 1000.0);
		S7Client::instance()->write(CELL_REG_ADDR(SET_VAC_TIME_BASE_ADDR, m_group), (uint8_t)((val >> 24) & 0xFF));
		S7Client::instance()->write(CELL_REG_ADDR(SET_VAC_TIME_BASE_ADDR + 1, m_group), (uint8_t)((val >> 16) & 0xFF));
		S7Client::instance()->write(CELL_REG_ADDR(SET_VAC_TIME_BASE_ADDR + 2, m_group), (uint8_t)((val >> 8) & 0xFF));
		S7Client::instance()->write(CELL_REG_ADDR(SET_VAC_TIME_BASE_ADDR + 3, m_group), (uint8_t)(val & 0xFF));
		// 最后真空值
		i   = (int) vac_param[PINS_VAC_VALUE];
		d   = vac_param[PINS_VAC_VALUE] - (float)i;
		val = (i * 10) + (int)(d * 10.0);
		S7Client::instance()->write(CELL_REG_ADDR(SET_VAC_VALUE_BASE_ADDR, m_group), (uint8_t)((val >> 8) & 0xFF));
		S7Client::instance()->write(CELL_REG_ADDR(SET_VAC_VALUE_BASE_ADDR + 1, m_group), (uint8_t)(val & 0xFF));
		// 真空保压幅度值
		i   = (int) vac_param[PINS_VAC_TOL];
		d   = vac_param[PINS_VAC_TOL] - (float)i;
		val = (i * 10) + (int)(d * 10.0);
		S7Client::instance()->write(CELL_REG_ADDR(SET_KEEP_VAC_AMP_BASE_ADDR, m_group), (uint8_t)((val >> 8) & 0xFF));
		S7Client::instance()->write(CELL_REG_ADDR(SET_KEEP_VAC_AMP_BASE_ADDR + 1, m_group), (uint8_t)(val & 0xFF));
		// 抽真空漏率
		i   = (int) vac_param[PINS_LEAK_RATIO];
		d   = vac_param[PINS_LEAK_RATIO] - (float)i;
		val = (i * 10) + (int)(d * 10.0);
		S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_RATIO_BASE_ADDR, m_group), (uint8_t)((val >> 8) & 0xFF));
		S7Client::instance()->write(CELL_REG_ADDR(SET_LEAK_RATIO_BASE_ADDR + 1, m_group), (uint8_t)(val & 0xFF));
	}
	
#elif defined(PLC_JS_VF)
	int   i   = 0;
	float d   = 0;
	int   val = 0;
	
	// 高真空真空最大值
	i	= (int) vac_param[PINS_HVAC_VALUE_MAX];
	d	= vac_param[PINS_HVAC_VALUE_MAX] - (float)i;
	val = (i * 10) + (int)(d * 10.0);
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_HVAC_MAX_VALUE_ADDR, m_group), (uint8_t)((val >> 8) & 0xFF));
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_HVAC_MAX_VALUE_ADDR + 1, m_group), (uint8_t)(val & 0xFF));
	// 高真空抽气时间
	i	= (int) vac_param[PINS_HVAC_OPEN_TIME];
	d	= vac_param[PINS_HVAC_OPEN_TIME] - (float)i;
	val = (i * 10) + (int)(d * 10.0);
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_HVAC_ON_TIME_ADDR, m_group), (uint8_t)((val >> 8) & 0xFF));
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_HVAC_ON_TIME_ADDR + 1, m_group), (uint8_t)(val & 0xFF));
	// 高真空停顿时间
	i	= (int) vac_param[PINS_HVAC_CLOSE_TIME];
	d	= vac_param[PINS_HVAC_CLOSE_TIME] - (float)i;
	val = (i * 10) + (int)(d * 10.0);
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_HVAC_OFF_TIME_ADDR, m_group), (uint8_t)((val >> 8) & 0xFF));
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_HVAC_OFF_TIME_ADDR + 1, m_group), (uint8_t)(val & 0xFF));

	// 低真空真空最大值
	i	= (int) vac_param[PINS_LVAC_VALUE_MAX];
	d	= vac_param[PINS_LVAC_VALUE_MAX] - (float)i;
	val = (i * 10) + (int)(d * 10.0);
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_LVAC_MAX_VALUE_ADDR, m_group), (uint8_t)((val >> 8) & 0xFF));
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_LVAC_MAX_VALUE_ADDR + 1, m_group), (uint8_t)(val & 0xFF));
	// 低真空抽气时间
	i	= (int) vac_param[PINS_LVAC_OPEN_TIME];
	d	= vac_param[PINS_LVAC_OPEN_TIME] - (float)i;
	val = (i * 10) + (int)(d * 10.0);
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_LVAC_ON_TIME_ADDR, m_group), (uint8_t)((val >> 8) & 0xFF));
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_LVAC_ON_TIME_ADDR + 1, m_group), (uint8_t)(val & 0xFF));
	// 低真空停顿时间
	i	= (int) vac_param[PINS_LVAC_CLOSE_TIME];
	d	= vac_param[PINS_LVAC_CLOSE_TIME] - (float)i;
	val = (i * 10) + (int)(d * 10.0);
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_LVAC_OFF_TIME_ADDR, m_group), (uint8_t)((val >> 8) & 0xFF));
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_LVAC_OFF_TIME_ADDR + 1, m_group), (uint8_t)(val & 0xFF));
#endif
}

void PinsGroup::set_constant_temp(float temp, float tempDiv)
{
#if defined(PLC_JS_CT) || defined(PLC_JYT)
	int   i = (int) temp;
	float d = temp - (float)i;
	int val = (i * 10) + (int)(d * 10.0);
	S7Client::instance()->write(CELL_REG_ADDR(C_TEMP_BASE_ADDR, m_group), (uint16_t)val);

	i   = (int) tempDiv;
	d   = tempDiv - (float)i;
	val = (i * 10) + (int)(d * 10.0);
	S7Client::instance()->write(CELL_REG_ADDR(C_TEMP_DIV_BASE_ADDR, m_group), (uint16_t)val);
#endif
}

void PinsGroup::set_fireProtection_temp(float temp)
{
#if defined(PLC_JS_CT) || defined(PLC_JYT)
	int   i = (int) temp;
	float d = temp - (float)i;
	int val = (i * 10) + (int)(d * 10.0);
	S7Client::instance()->write(CELL_REG_ADDR(FIRE_PRO_TEMP_BASE_ADDR, m_group), (uint16_t)val);
#endif
}

// 清除告警
void PinsGroup::clear_alarm()
{
#ifdef SIEMENS_PLC
	uint8 command_id = 1;
	S7Client::instance()->write(CLEAR_WARN_BASE_ADDR + m_group * GROUP_MEM_SIZE, command_id);
	msleep(1000);
	command_id = 0;
	S7Client::instance()->write(CLEAR_WARN_BASE_ADDR + m_group * GROUP_MEM_SIZE, command_id);
#elif defined(PLC_JS_CT)
	S7Client::instance()->write(CELL_REG_ADDR(CLEAR_WARN_BASE_ADDR, m_group), (uint8_t)1);
	msleep(1000);
	S7Client::instance()->write(CELL_REG_ADDR(CLEAR_WARN_BASE_ADDR, m_group), (uint8_t)0);
#elif defined(PLC_JS_VF)
	S7Client::instance()->write(CELL_SET_REG_ADDR(CLEAN_ALARM_ADDR, m_group), (uint16_t)1);
	msleep(1000);
	S7Client::instance()->write(CELL_SET_REG_ADDR(CLEAN_ALARM_ADDR, m_group), (uint16_t)0);
#elif defined(PLC_JYT)
	set_action_bits(8, true);
	set_action_bits(6, false);
	set_action_bits(7, false);

	msleep(1000);
	set_action_bits(8, false);
#endif
}

#ifdef PLC_JS_VF

void PinsGroup::set_work_mode(bool auto_mode)
{
	uint16_t value = auto_mode ? 1 : 2;
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_PLC_WORK_STATUS_ADDR, m_group), value);
//	LOGFMTD("VW%d=%d", CELL_SET_REG_ADDR(SET_PLC_WORK_STATUS_ADDR, m_group), value); 
}

void PinsGroup::fire_fan_on_off(bool on)
{
	uint16_t value = on ? 1 : 2;
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_FIRE_FAN_ONOFF_ADDR, m_group), value);
}

void PinsGroup::fire_dor_on_off(bool on)
{
	uint16_t value = on ? 1 : 2;
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_FIRE_DOR_ONOFF_ADDR, m_group), value);
}

void PinsGroup::set_lvac_work_mode(uint16_t mode)
{
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_LVAC_WORK_MODE_ADDR, m_group), mode);
}

void PinsGroup::set_hvac_work_mode(uint16_t mode)
{
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_HVAC_WORK_MODE_ADDR, m_group), mode);
}



void PinsGroup::clear_smoke_alarm()
{
	uint16_t value = 1;
	S7Client::instance()->write(CELL_SET_REG_ADDR(SET_SMOKE_ALARM_ADDR, m_group), value);
}

#endif


#if  defined(PLC_JYT)
Pins_Status_Data_t PinsGroup::get_env_datas()
{
	// 寄存器地址是连续的，一次性读取效率最高
	S7Client::instance()->read_byte(CELL_REG_ADDR(GET_ACT_INDICATE_BASE_ADDR, m_group), 
		(uint8_t *)&m_read_reg, sizeof(m_read_reg));
	endian_swap((uint8_t *)&m_read_reg.actWord, 0, sizeof(m_read_reg.actWord));
	endian_swap((uint8_t *)&m_read_reg.lampWord, 0, sizeof(m_read_reg.lampWord));
	endian_swap((uint8_t *)&m_read_reg.step, 0, sizeof(m_read_reg.step));
	endian_swap((uint8_t *)&m_read_reg.vacPV, 0, sizeof(m_read_reg.vacPV));
	endian_swap((uint8_t *)&m_read_reg.errA, 0, sizeof(m_read_reg.errA));
	endian_swap((uint8_t *)&m_read_reg.errB1, 0, sizeof(m_read_reg.errB1));
	endian_swap((uint8_t *)&m_read_reg.errB2, 0, sizeof(m_read_reg.errB2));
	endian_swap((uint8_t *)&m_read_reg.errC1, 0, sizeof(m_read_reg.errC1));
	endian_swap((uint8_t *)&m_read_reg.errC2, 0, sizeof(m_read_reg.errC2));
	for (int i=0; i<8; i++)
	{
		endian_swap((uint8_t *)&m_read_reg.pins_temp[i], 0, sizeof(m_read_reg.pins_temp[0]));
	}
	
	// 更新返回值
	Pins_Status_Data_t data;

	get_error_code(data.err_code);
	data.is_smoke_alarm = (data.err_code[0] >> 1) & 1;

	data.is_auto_mode   = is_auto_mode();
	data.is_cell_empty  = is_cell_empty();
	data.is_extend      = is_pins_extend();
	data.is_retract     = is_pins_retract();
	data.vacuum         = get_vac_value();
	//data.step_no        = get_step_no();
	
	get_temperature(data.temp);
	
	return data;
}

#elif (defined(PLC_JS_VF) || defined(PLC_JS_CT))

Pins_Status_Data_t PinsGroup::get_env_datas()
{
	Pins_Status_Data_t data;

#ifdef PLC_JS_VF
	// 控制状态
	uint16_t ctr_status = 0;
	ctr_status  = S7Client::instance()->read_byte(CELL_GET_REG_ADDR(GET_STATUS_INFO_ADDR , m_group));
	ctr_status <<= 8;
	ctr_status |= S7Client::instance()->read_byte(CELL_GET_REG_ADDR(GET_STATUS_INFO_ADDR + 1, m_group));
	uint16_t sys_status = 0;
	sys_status  = S7Client::instance()->read_byte(GET_PLC_INFO_ADDR);
	sys_status <<= 8;
	sys_status |= S7Client::instance()->read_byte(GET_PLC_INFO_ADDR + 1);
#endif

	// 故障码
	//data.err_code       = get_error_code();
	// 状态
	data.is_auto_mode   = is_auto_mode(); //中原0:维护模式。1正常模式，中航0：手动 1：自动
	data.is_cell_empty  = is_cell_empty();
	data.is_extend      = is_pins_extend();
	data.is_retract     = is_pins_retract();
	data.vacuum         = get_vac_value();
#ifdef PLC_JS_VF
	data.is_HVAC_open        = (ctr_status & (1 << 4)) != 0;
	data.is_LVAC_open        = (ctr_status & (1 << 5)) != 0;
	data.is_leak_VAC_open    = (ctr_status & (1 << 6)) != 0;
	data.is_emergency_stop   = (ctr_status & (1 << 7)) != 0;
	data.is_palletizer_entry = (ctr_status & (1 << 8)) != 0;
	data.is_temp_fan_open    = (ctr_status & (1 << 9)) != 0;
	data.is_smoke_fan_open   = (ctr_status & (1 << 10)) != 0;
	data.is_fire_dor_open    = (ctr_status & (1 << 11)) != 0;
	data.is_power_open       = (ctr_status & (1 << 12)) != 0;

	data.is_sys_vac_normal   = (sys_status & (1 << 0)) == 0;
	data.is_fire_fan_open    = (sys_status & (1 << 3)) != 0;
	data.is_plc_connect = S7Client::instance()->isconnect();
	//data.is_plc_maintmode = is_maint_mode();//1:维护模式。0正常模式
#endif
	get_temperature(data.temp);
	
#ifdef PLC_JS_CT
	data.is_smoke_alarm = (data.err_code & 0x80) != 0;
	data.smokeStatus = S7Client::instance()->read_byte(CELL_REG_ADDR(READ_SMOKE_SENSOR_ADDR, m_group));
#endif

	return data;
}
#endif

// 是否为自动模式
bool PinsGroup::is_auto_mode()
{
	bool status = true;
#ifdef PLC_JS_CT
	status = S7Client::instance()->read_bit(CELL_REG_ADDR(STATE_WORK_MODE_BASE_ADDR, m_group), 0);
#elif defined(PLC_JS_VF)
	status = S7Client::instance()->read_bit(CELL_GET_REG_ADDR(GET_STATUS_INFO_ADDR + 1, m_group), 0) == 0;
#elif defined(PLC_JYT)
	status = get_status_bits(3);
#endif

	return status;
}

// 是否为维护模式
bool PinsGroup::is_maint_mode()
{
	bool status = true;
#ifdef PLC_JS_CT
	status = S7Client::instance()->read_bit(CELL_REG_ADDR(STATE_WORK_MODE_BASE_ADDR, m_group), 0);
#elif defined(PLC_JS_VF)
	status = S7Client::instance()->read_bit(CELL_GET_REG_ADDR(GET_STATUS_INFO_ADDR + 1, m_group), 0) == 1;
#elif defined(PLC_JYT)
	status = get_status_bits(1);
#endif

	return status;
}


// 托盘是否为空
bool PinsGroup::is_cell_empty()
{
	bool empty = true;
	
#ifdef SIEMENS_PLC
	empty = !S7Client::instance()->read_bit(STATE_CELL_TRAY_BASE_ADDR + m_group * GROUP_MEM_SIZE, 0);
#elif defined(PLC_JS_CT)
	uint8_t placed = S7Client::instance()->read_byte(CELL_REG_ADDR(STATE_CELL_TRAY_BASE_ADDR, m_group));
	empty = ((placed & 0x03) != 0) ? false : true;
#elif defined(PLC_JS_VF)
	empty = S7Client::instance()->read_bit(CELL_GET_REG_ADDR(GET_STATUS_INFO_ADDR + 1, m_group), 1) != 1;
#elif defined(PLC_JYT)
	empty = !get_status_bits(17);
#endif

	return empty;
}

// 针床是否压合
bool PinsGroup::is_pins_extend()
{
	bool extend = false;
	
#ifdef SIEMENS_PLC
	extend = S7Client::instance()->read_bit(STATE_PINS_BASE_ADDR + m_group * GROUP_MEM_SIZE, 0);
#elif defined(PLC_JS_CT)
	extend = S7Client::instance()->read_bit(CELL_REG_ADDR(STATE_PINS_BASE_ADDR, m_group), 1);
#elif defined(PLC_JS_VF)
	extend = S7Client::instance()->read_bit(CELL_GET_REG_ADDR(GET_STATUS_INFO_ADDR + 1, m_group), 2) == 1;
#elif defined(PLC_JYT)
	extend = get_status_bits(26);
#endif

	return extend;
}

// 针床是否脱开
bool PinsGroup::is_pins_retract()
{
	bool retract = false;

#ifdef SIEMENS_PLC
	retract = S7Client::instance()->read_bit(STATE_PINS_BASE_ADDR + m_group * GROUP_MEM_SIZE, 1);
#elif defined(PLC_JS_CT)
	retract = S7Client::instance()->read_bit(CELL_REG_ADDR(STATE_PINS_BASE_ADDR, m_group), 0);
#elif defined(PLC_JS_VF)
	retract = S7Client::instance()->read_bit(CELL_GET_REG_ADDR(GET_STATUS_INFO_ADDR + 1, m_group), 3) == 1;
#elif defined(PLC_JYT)
	retract = get_status_bits(27);
#endif

	return retract;
}

// 库门是否打开
bool PinsGroup::is_dor_open()
{
#if defined(PLC_JS_VF)
	return S7Client::instance()->read_bit(CELL_GET_REG_ADDR(GET_ALARM_INFO_ADDR, m_group), 0) == 1;
#elif defined(PLC_JYT)
	return !get_status_bits(18);
#else
	return false;
#endif
}

// 是否保壓中
bool PinsGroup::is_keep_vac()
{
	bool status = false;
	
#if defined(PLC_JYT)
	status = get_status_bits(31);
#endif

	return status;
}

// 读取故障信息
#ifdef PLC_JYT

void PinsGroup::get_error_code(uint16_t err[5])
{
	err[0] = m_read_reg.errA;
	err[1] = m_read_reg.errB1;
	err[2] = m_read_reg.errB2;
	err[3] = m_read_reg.errC1;
	err[4] = m_read_reg.errC2;
}

#else

int PinsGroup::get_error_code()
{
	int val = 0;
	
#ifdef SIEMENS_PLC
	val = S7Client::instance()->read_int(ERROR_CODE_BASE_ADDR + m_group * GROUP_MEM_SIZE);
#elif defined PLC_JS_CT
	val = S7Client::instance()->read_int(CELL_REG_ADDR(ERROR_CODE_BASE_ADDR, m_group));
#elif defined(PLC_JS_VF)
	val  = S7Client::instance()->read_byte(CELL_GET_REG_ADDR(GET_ALARM_INFO_ADDR , m_group));
	val <<= 8;
	val |= S7Client::instance()->read_byte(CELL_GET_REG_ADDR(GET_ALARM_INFO_ADDR + 1, m_group));
#endif

	return val;
}

#endif

// 读取当前负压值
float PinsGroup::get_vac_value()
{
	float value = 0.0;
	
#ifdef SIEMENS_PLC
	value = S7Client::instance()->read_float(READ_VAC_VALUE_BASE_ADDR + m_group * GROUP_MEM_SIZE);
#elif defined(PLC_JYT)
	value = (float)m_read_reg.vacPV / 10.0;
#else
	uint8_t val_0 = 0;
	uint8_t val_1 = 0;

	#if defined(PLC_JS_CT)
		val_0 = S7Client::instance()->read_byte(CELL_REG_ADDR(READ_VAC_VALUE_BASE_ADDR, m_group));
		val_1 = S7Client::instance()->read_byte(CELL_REG_ADDR(READ_VAC_VALUE_BASE_ADDR + 1, m_group));
	#elif defined(PLC_JS_VF)
		val_0 = S7Client::instance()->read_byte(GET_VAC_VALUE_ADDR + (m_group-1)*2);
		val_1 = S7Client::instance()->read_byte(GET_VAC_VALUE_ADDR + (m_group-1)*2 + 1);
	#endif

	if ((val_0 & 0x80) != 0)
	{
		value = (float)((int)(0xFFFF0000 | (val_0 << 8) | val_1)) / 10.0;
	}
	else
	{
		value = (float)((int)val_1 | ((int)val_0 << 8)) / 10.0;
	}
#endif

	return value;
}

// 真空状态
bool PinsGroup::get_vac_status()
{
	uint8 value = 0;
	
#ifdef SIEMENS_PLC
	value = S7Client::instance()->read_byte(STATE_VAC_BASE_ADDR + m_group * GROUP_MEM_SIZE);
#elif defined(PLC_JS_CT)
	value = S7Client::instance()->read_byte(CELL_REG_ADDR(STATE_VAC_BASE_ADDR, m_group));
#endif

	return (value != 0) ? true : false;
}

#ifdef PLC_JS_VF
void PinsGroup::get_temperature(float temp[2])
{
	uint8_t val_0;
	uint8_t val_1;

	switch (m_group)
	{
		case 1:
			val_0   = S7Client::instance()->read_byte(120);
			val_1   = S7Client::instance()->read_byte(120 + 1);
			temp[0] = (float)((int)val_1 | (int)val_0 << 8) / 10.0;
			val_0   = S7Client::instance()->read_byte(122);
			val_1   = S7Client::instance()->read_byte(122 + 1);
			temp[1] = (float)((int)val_1 | (int)val_0 << 8) / 10.0;
			break;
			
		case 2:	
			val_0   = S7Client::instance()->read_byte(124);
			val_1   = S7Client::instance()->read_byte(124 + 1);
			temp[0] = (float)((int)val_1 | (int)val_0 << 8) / 10.0;
			val_0   = S7Client::instance()->read_byte(140);
			val_1   = S7Client::instance()->read_byte(140 + 1);
			temp[1] = (float)((int)val_1 | (int)val_0 << 8) / 10.0;		
			break;
			
		case 3:	
			val_0   = S7Client::instance()->read_byte(142);
			val_1   = S7Client::instance()->read_byte(142 + 1);
			temp[0] = (float)((int)val_1 | (int)val_0 << 8) / 10.0;
			val_0   = S7Client::instance()->read_byte(144);
			val_1   = S7Client::instance()->read_byte(144 + 1);
			temp[1] = (float)((int)val_1 | (int)val_0 << 8) / 10.0;		
			break;
			
		case 4:	
			val_0   = S7Client::instance()->read_byte(146);
			val_1   = S7Client::instance()->read_byte(146 + 1);
			temp[0] = (float)((int)val_1 | (int)val_0 << 8) / 10.0;
			val_0   = S7Client::instance()->read_byte(148);
			val_1   = S7Client::instance()->read_byte(148 + 1);
			temp[1] = (float)((int)val_1 | (int)val_0 << 8) / 10.0;		
			break;
	}
}
#elif defined(PLC_JYT)
void PinsGroup::get_temperature(float temp[8])
{
	for (int i = 0; i < 8; i++)
	{
		temp[i] = (float)m_read_reg.pins_temp[i] / 10.0;
	}
}

#else
void PinsGroup::get_temperature(float temp[7])
{
	for (int i = 0; i < 7; i++)
	{
	#ifdef SIEMENS_PLC
		temp[i] = S7Client::instance()->read_float(READ_TEMPE_BASE_ADDR + m_group * GROUP_MEM_SIZE + i * 4);
	#elif defined(PLC_JS_CT)
		uint8_t val_0 = S7Client::instance()->read_byte(CELL_REG_ADDR(READ_TEMPE_BASE_ADDR + i*2, m_group));
		uint8_t val_1 = S7Client::instance()->read_byte(CELL_REG_ADDR(READ_TEMPE_BASE_ADDR + i*2 + 1, m_group));
		temp[i] = (float)((int)val_1 | (int)val_0 << 8) / 10.0;
	#endif
	}
}
#endif

void PinsGroup::start_1230_sys()
{
}

bool PinsGroup::state_1230_sys()
{
	return false;
}

void PinsGroup::start_1230_fans()
{
}

