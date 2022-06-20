#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
#include <string>
#include <time.h>

#include "CabinetMeter.h"
#include "MsgFactory.h"
#include "Configuration.h"
#include "FormatConvert.h"
#include "log4z.h"

using namespace std;

CabinetMeter::CabinetMeter()
{
	itm_idx = 0;
	m_meterNo = METER_NO_1;
	m_port_No = 0;
	m_rtu_master = NULL;
	m_valid_flag = false;

	for(int i = 0; i < METER_CNT; i++)
	{
		memset((void*)&m_mtr_data[i],0,sizeof(meter_data_t));
		memset((void*)&u_i_ratio[i],0,sizeof(meter_UI_attr_t));
		itm_read_mask[i] = 0;
		m_finish_flag[i] = false;
	}
}

CabinetMeter::~CabinetMeter()
{
	if (m_rtu_master != NULL)
	{
		modbus_close(m_rtu_master);
		modbus_free(m_rtu_master);
	}
}

thread CabinetMeter::run()
{
	return thread(&CabinetMeter::work, this);
}

void CabinetMeter::work()
{
	time_t curr_tm = 0;
	time_t last_tm = 0;

	try
	{
		//上报上位机
		MsgFactory::instance()->create_cab_to_ext_reply_pusher(Configuration::instance()->ext_reply_socket());
	}
	catch (zmq::error_t &e)
	{
		LOGFMTE("Create ZMQ communication error. code: %d, description:%s", e.num(), e.what());
		exit(-209);
	}

	while (1)
    {
		msleep(400);
		
		curr_tm = time(NULL);			
		if (curr_tm - last_tm >= 4)				//抄表周期4s
		{
			last_tm = curr_tm;
		
			if(!m_valid_flag)
			{
				serial_port_detect();			//检测有效串口
				continue;
			}
			else							
			{
				vector<meter_data_t> data_vector;
				meter_data_t data;

				m_valid_flag = false;
				
				for(int i= 0; i < METER_CNT; i++)
				{
					convert_meter_data(i+1,data);
					data_vector.push_back(data);
					
					if(itm_read_mask[i] > 0)		//电表抄读正常 通信口有效
					{
						m_valid_flag = true;
						itm_read_mask[i] = 0;
					}
					m_finish_flag[i] = false;		//重置抄表完成标志
				}		

				string josn_str = FormatConvert::instance().cab_meter_to_string(data_vector);
				MsgFactory::instance()->cab_to_ext_reply_pusher().send(josn_str);
			}
		}	
		
		if(m_valid_flag) 					//开始抄表
		{
			read_meter_item();
		}
	}

	try
    {
		MsgFactory::instance()->destory_cab_to_ext_reply_pusher();
    }
    catch (zmq::error_t &e)
    {
		LOGFMTE("destory ZMQ communication error. code: %d, description:%s", e.num(), e.what());
    }
}


void CabinetMeter::serial_port_detect()
{
	//modbus_t *m_rtu_tmp;
	char port_path[64];
	//uint16_t dest[2];
	bool ret;

	if(m_port_No == 0)
	{
		sprintf(port_path,"%s","/dev/ttyS0");
	}
	else
	{
		sprintf(port_path,"%s","/dev/ttyS1");
	}

	m_rtu_master = modbus_new_rtu(port_path,9600,'N',8,1);	
	//设置从地址为1
	modbus_set_slave(m_rtu_master,METER_NO_1);
	//建立链接
    modbus_connect(m_rtu_master);
	//应答超时
	modbus_set_response_timeout(m_rtu_master,0,400000);

	ret = read_register(IrAt_ADDR,&u_i_ratio[0].IrAt,2);
	if (!ret)					//读取失败
	{
		modbus_close(m_rtu_master);
		modbus_free(m_rtu_master);

		m_port_No = !(m_port_No);
	}
	else							//读取成功 表明检测到有效通信口
	{
		msleep(150);
		//设置从地址为2
		modbus_set_slave(m_rtu_master,METER_NO_2);
		read_register(IrAt_ADDR,&u_i_ratio[1].IrAt,2);
		//LOGFMTD("Meter1 IrAt:%d UrAt:%d   Meter2 IrAt:%d UrAt:%d",u_i_ratio[0].IrAt,u_i_ratio[0].UrAt,u_i_ratio[1].IrAt,u_i_ratio[1].UrAt);
		
		m_valid_flag = true;
		set_meter_item(METER_NO_1);
	}
}


void CabinetMeter::add_item(int addr,int len,float *out)
{
	meter_item_t n_item;

	n_item.addr = addr;
	n_item.len = len;
	n_item.out = out;

	m_mtr_items.push_back(n_item);	
}

void CabinetMeter::set_meter_item(int meterNo)
{
	meter_data_t *p_mtr_data;

	m_mtr_items.clear();

	if(meterNo == METER_NO_1)
	{
		m_meterNo = METER_NO_1;
		p_mtr_data = (meter_data_t *)&m_mtr_data[0];
	}
	else
	{
		m_meterNo = METER_NO_2;
		p_mtr_data = (meter_data_t *)&m_mtr_data[1];
	}
	modbus_set_slave(m_rtu_master,m_meterNo);

	p_mtr_data->timestamp = millitimestamp();
	add_item(FORWARD_ENERGY_ADDR,1,&(p_mtr_data->forward_energy));
	add_item(REVERSE_ENERGY_ADDR,1,&(p_mtr_data->reverse_energy));
#if 0
	add_item(PHASE_UA_ADDR,3,p_mtr_data->wU);
	add_item(CURRENT_IA_ADDR,3,p_mtr_data->wI);
	add_item(POWER_A_ADDR,3,p_mtr_data->wP);
#endif
}


bool CabinetMeter::read_register(int addr, uint16_t *out, int len)
{
	uint16_t dest[len];
	int      nb = len;
	
	int ret = modbus_read_registers(m_rtu_master, addr, nb, dest);
	if (ret != nb)
	{
		//LOGFMTD("CabinetMeter::read err, %s.",modbus_strerror(ret));
		return false;
	}
	
	memcpy(out, dest, sizeof(dest));
	return true;
}

bool CabinetMeter::read_register(int addr, float *out, int len)
{
	uint16_t dest[2*len];
	int	     nb = 2*len;

	int ret = modbus_read_registers(m_rtu_master, addr, nb, dest);
	if (ret != nb)
	{
		LOGFMTD("CabinetMeter::read err, %s.",modbus_strerror(ret));
		return false;
	}

	for (int i=0; i<len; i++)
	{
	#if 1
		out[i] = modbus_get_float_dcba(dest + (i*2));
	#else
		out[i] = modbus_get_float_badc(dest + (i*2));		//for debug simulate
	#endif
	}

	return true;
}

void CabinetMeter::read_meter_item()
{
	if(!m_finish_flag[m_meterNo-1])
	{
		bool status = false;
		meter_item_t n_item = m_mtr_items[itm_idx];
		
		status = read_register(n_item.addr,n_item.out,n_item.len);
		if(status)
		{
			itm_read_mask[m_meterNo-1] |= (0x00000001 << itm_idx);
		}
		else
		{
			memset(n_item.out,0,n_item.len*sizeof(float));
			itm_read_mask[m_meterNo-1] &= (~(0x00000001 << itm_idx));
		}

		itm_idx++;
		if(itm_idx >= m_mtr_items.size())			//当前电表抄读完成
		{
			int the_other_meter = 0;

			m_finish_flag[m_meterNo-1] = true;
			if(m_meterNo == METER_NO_1)
			{
				the_other_meter = METER_NO_2;
			}
			else
			{
				the_other_meter = METER_NO_1;
			}
			set_meter_item(the_other_meter);
			itm_idx = 0;
		}
	}
}

void CabinetMeter::convert_meter_data(int meterNo,meter_data_t &val)
{
	if(meterNo > 0 && meterNo < 3)
	{
		float f_IrAt = (float)u_i_ratio[meterNo-1].IrAt;
		float f_UrAt = (float)u_i_ratio[meterNo-1].UrAt*0.1;
		
		val.forward_energy = m_mtr_data[meterNo-1].forward_energy*f_IrAt*f_UrAt;
		val.reverse_energy = m_mtr_data[meterNo-1].reverse_energy*f_IrAt*f_UrAt;
		val.timestamp = m_mtr_data[meterNo-1].timestamp;
#if 0
		for(int i = 0; i < 3; i++)
		{
			val.wU[i] = m_mtr_data[meterNo-1].wU[i]*f_UrAt*0.1;			//电压倍率0.1
			val.wI[i] = m_mtr_data[meterNo-1].wI[i]*f_IrAt*0.001;		//电流倍率0.001
			val.wP[i] = m_mtr_data[meterNo-1].wP[i]*f_IrAt*f_UrAt*0.1;	//功率倍率0.1
		}
#endif
	}
}
#endif

