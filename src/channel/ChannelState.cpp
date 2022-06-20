#include <string.h>
#include "log4z.h"
#include "ChannelState.h"
#include "ReplyHandler.h"
#include "FormatConvert.h"
#include "cabinet_db.h"

using namespace std;

ChannelState::ChannelState()
{
	reCounter = 5;
	recFreq = 5;
	totalRunTime = 0;
	m_heartbeat_cnt  = 0;
	re_suspend_cnt = 0;
#ifndef PROTOCOL_5V160A		
	m_last_hb_tm     = 0;
    m_is_running     = false;
#endif	
    m_is_step_finish = false;
	//m_is_step_vac = false;
#if (defined(AWH_GRADING_SYS) || defined(AWH_PRECHARGE_SYS))	
	m_is_stepcache_empty = false;
#endif
	m_rec_update_flag = false;
	m_err_code.clear();
	
	memset(m_battery_type, 0, sizeof(m_battery_type));
	memset(m_stepConf_number, 0, sizeof(m_stepConf_number));
	memset(m_pallet_barcode, 0, sizeof(m_pallet_barcode));
	memset(m_batch_number, 0, sizeof(m_batch_number));
	memset(m_battery_barcode, 0, sizeof(m_battery_barcode));
	memset(lower_version, 0, sizeof(lower_version));

	memset((void*)&m_capacity_rec,0,sizeof(Step_Capacity_Record_t));
	memset((void*)&addStatus,0,sizeof(Step_Process_Addition_Data_t));
	memset((void*)&m_record,0,sizeof(Step_Process_Record_Data_t));
	memset((void*)&m_processData,0,sizeof(Step_Process_Data_t));
	memset((void*)&m_last_processData,0,sizeof(Step_Process_Data_t));
}

ChannelState& ChannelState::operator=(const ChannelState& obj)
{
    if (this != &obj)
    {
        this->m_heartbeat_cnt = obj.m_heartbeat_cnt;
#ifndef PROTOCOL_5V160A			
        this->m_last_hb_tm = obj.m_last_hb_tm;
		this->m_is_running = obj.m_is_running;
#endif			
		this->reCounter = obj.reCounter;
		this->recFreq = obj.recFreq;
		this->re_suspend_cnt = obj.re_suspend_cnt;
		//this->m_is_step_vac = obj.m_is_step_vac;
		//this->m_vac_value = obj.m_vac_value;
		this->m_is_step_finish = obj.m_is_step_finish;
#if (defined(AWH_GRADING_SYS) || defined(AWH_PRECHARGE_SYS))		
		this->m_is_stepcache_empty = obj.m_is_stepcache_empty;
#endif
		this->m_rec_update_flag = obj.m_rec_update_flag;	

		this->addStatus = obj.addStatus;
		this->m_record = obj.m_record;	

		memset(m_battery_type, 0, sizeof(m_battery_type));
		memset(m_stepConf_number, 0, sizeof(m_stepConf_number));
		memset(m_pallet_barcode, 0, sizeof(m_pallet_barcode));
		memset(m_batch_number, 0, sizeof(m_batch_number));
		memset(m_battery_barcode, 0, sizeof(m_battery_barcode));
		memset(lower_version, 0, sizeof(lower_version));
		
		memset((void*)&m_processData,0,sizeof(Step_Process_Data_t));
		memset((void*)&m_last_processData,0,sizeof(Step_Process_Data_t));
		memset((void*)&m_capacity_rec,0,sizeof(Step_Capacity_Record_t));
    }

    return *this;
}

void ChannelState::set_heartbeat_tm()
{
	m_heartbeat_cnt++;
}

void ChannelState::reset_heartbeat_tm()
{
#ifndef PROTOCOL_5V160A
	m_last_hb_tm    = time(NULL);
#endif
	m_heartbeat_cnt = 0;
}

bool ChannelState::is_heartbeat_timeout()
{
	return m_heartbeat_cnt > HEARTBEAT_TIMEOUT_MAX;
}

#ifndef PROTOCOL_5V160A	
int ChannelState::get_heartbeat_lost_tm()
{
	time_t nowtm = time(NULL);
	return nowtm - m_last_hb_tm;
}

void ChannelState::set_running(bool flag)
{
    m_is_running = flag;
}

bool ChannelState::is_running()
{ 
	return m_is_running;
}
#endif

#if 0
void ChannelState::set_step_vac(bool flag)
{
	m_is_step_vac = flag;
}

bool ChannelState::is_step_vac()
{
	return m_is_step_vac;
}

void ChannelState::set_vac_value(int8_t value)
{
 	m_vac_value = value;
}

int8_t ChannelState::get_vac_value()
{
	return m_vac_value;
}
#endif

void ChannelState::set_step_finish(bool flag)
{
	m_is_step_finish = flag;
}

bool ChannelState::is_step_finish()
{
	return m_is_step_finish;
}

#if (defined(AWH_GRADING_SYS) || defined(AWH_PRECHARGE_SYS))
void ChannelState::set_stepcache_empty(bool flag)
{
	m_is_stepcache_empty = flag;
}

bool ChannelState::is_stepcache_empty()
{
	return m_is_stepcache_empty;
}
#endif

void ChannelState::push_error_code(int code)
{
	m_err_code.push_back(code);
}

void ChannelState::clear_error_code()
{
	m_err_code.clear();
	m_prev_err_code.clear();
}

int ChannelState::get_lastest_error_code()
{
	uint32 num = m_err_code.size();
	
	if(num > 0)
	{
		return m_err_code[num-1];
	}
	else
	{
		return 0;
	}
}

vector<int> ChannelState::get_error_code()
{
	for (uint32_t i=0; i<m_err_code.size(); ++i)
	{
		bool is_happend = false;
		
		for (uint32_t j=0; j<m_prev_err_code.size(); ++j)
		{
			if (m_err_code[i] == m_prev_err_code[j])
			{
				is_happend = true;
				break;
			}
		}

		if (!is_happend)
		{
			m_prev_err_code.push_back(m_err_code[i]);
		}
	}
	
	return m_prev_err_code;
}

void ChannelState::set_version_info(char *str)
{
	if(strlen(str) > 0)
	{
		strcpy(lower_version, str);
	}
	else
	{
		memset(lower_version,'\0',sizeof(lower_version));
	}
}

bool ChannelState::get_version_info(char *str)
{
	if(strlen(lower_version) > 0)
	{
		strcpy(str,lower_version);
		memset(lower_version,'\0',sizeof(lower_version));
		return true;
	}
	else
	{
		return false;
	}
}

void ChannelState::set_addition_status(Step_Process_Addition_Data_t &data)
{
	addStatus = data;
#if defined(AWH_FORMATION_SYS)		//化成系统抽完真空下发工步后可能正好收到一帧等待
	if(m_processData.run_state == STATE_WAITING)
	{
		m_processData.sum_step = m_last_processData.sum_step;
		m_processData.loop_no = m_last_processData.loop_no;
	}
#endif
}

Step_Process_Addition_Data_t ChannelState::get_addition_status()
{
	return addStatus;
}

bool ChannelState::is_processData_update()
{
	return m_rec_update_flag;
}

void ChannelState::update_step_processData(Step_Process_Record_Data_t &rec)
{
	m_record = rec;
	m_rec_update_flag = true;
}

void ChannelState::get_step_processData(Step_Process_Record_Data_t &rec)
{
	rec = m_record;
	m_rec_update_flag = false;
}

void ChannelState::set_processData_info_code()
{
	memcpy(m_processData.bat_type, m_battery_type,INFO_SIZE);
	memcpy(m_processData.stepConf_no, m_stepConf_number,INFO_SIZE);
	memcpy(m_processData.pallet_barcode, m_pallet_barcode,INFO_SIZE);
	memcpy(m_processData.batch_no, m_batch_number,INFO_SIZE);
	memcpy(m_processData.bat_barcode, m_battery_barcode,INFO_SIZE);
}

void ChannelState::reset_info_code()
{
	memset(m_battery_type, 0, INFO_SIZE);
	memset(m_stepConf_number, 0, INFO_SIZE);
	memset(m_pallet_barcode, 0, INFO_SIZE);
	memset(m_batch_number, 0, INFO_SIZE);
	memset(m_battery_barcode, 0, INFO_SIZE);
}

void ChannelState::cache_step_processRecord(Step_Process_Data_t &rec,bool has_record)
{
	if(recFreq > 1)
	{
		if(m_record_deque.size() >= 10)
		{
			m_record_deque.pop_front();  //只缓存10条记录数据
		}
		
		if(!has_record) 		//已存数据不缓存了
		{
			m_record_deque.push_back(rec);
		}
	}
}

void ChannelState::report_processRecord_cache(int cellNo)
{
	if(recFreq > 1)
	{
		for(auto element = m_record_deque.begin();element != m_record_deque.end();element++)
		{
			string store_josn_str;
			store_josn_str = FormatConvert::instance().process_data_to_store_str_new(*element);
			LOG_INFO(data_id_process, store_josn_str);
					
			g_CabDBServer.pushbackRecord(cellNo,*element);
		}
	}
}

void ChannelState::push_temperature_data(Acq_Temperature_Data_t data)
{
	m_temperature_data.push_back(data);
}

void ChannelState::clear_temperature_data()
{
	m_temperature_data.clear();
}

void ChannelState::get_temperature_data(vector<Acq_Temperature_Data_t> &tempData)
{
	for(uint32 i = 0; i < m_temperature_data.size(); i++)
	{
		tempData.push_back(m_temperature_data[i]);
	}
}

void ChannelState::set_record_frequency(int rect)
{
	if(rect < 0)
	{
		return;
	}

	recFreq = rect;
	reCounter = rect;
	if(rect > 600)	//最长10min
	{
		recFreq = 600;
	}
}

void ChannelState::set_total_runtime(int time)
{
	totalRunTime = time;
}

int ChannelState::get_total_runtime()
{
	return totalRunTime;
}

void ChannelState::do_runtime_counter()
{
	int tmpRunTime = 0;
	
	if((m_processData.run_state == STATE_RUNNING) || (m_processData.run_state == STATE_START)
		|| (m_processData.run_state == STATE_NO_RUN && m_last_processData.run_state == STATE_RUNNING))
	{
		if((m_processData.step_no != m_last_processData.step_no) && (m_last_processData.step_no != 0))
		{
			totalRunTime += (int)m_last_processData.run_time;
		}
		
		tmpRunTime = (int)m_processData.run_time;
		
		m_processData.totalTime = (totalRunTime + tmpRunTime);					
	}

	// 运行中
	if((m_processData.run_state >= STATE_RUNNING && m_processData.run_state <= STATE_START)
	|| (m_processData.run_state == STATE_NO_RUN && m_last_processData.run_state == STATE_RUNNING))
	{
		if((m_processData.step_no != m_last_processData.step_no) && (m_last_processData.step_no != 0))
		{
			m_last_processData.cc_capacity = m_capacity_rec.cc_capacity;
			m_last_processData.cc_second = m_capacity_rec.cc_time;
			m_last_processData.cv_capacity = m_last_processData.capacity - m_capacity_rec.cc_capacity;
			if((int)m_last_processData.run_time >= m_capacity_rec.cc_time)
			{
				m_last_processData.cv_second = (int)m_last_processData.run_time - m_capacity_rec.cc_time;
			}
			else
			{
				m_last_processData.cc_second = (int)m_last_processData.run_time;
				m_last_processData.cv_second = 0;
			}
			
			memset((void*)&m_capacity_rec,0,sizeof(Step_Capacity_Record_t));
			if(m_processData.step_type != STEP_STANDING)
			{
				if(m_processData.CC_CV_flag == 0)
				{
					m_capacity_rec.cc_time++;
				}
				else if(m_processData.CC_CV_flag == 1)
				{
					m_capacity_rec.cv_time++;
				}
			}
		}
		else	//工步未切换
		{
			if(m_processData.step_type != STEP_STANDING)
			{
				if(m_processData.CC_CV_flag == 0)	//恒流
				{
					m_capacity_rec.cc_capacity = m_processData.capacity;
					m_capacity_rec.cc_time++;
				}
				else if((m_processData.CC_CV_flag != 0) && (m_last_processData.CC_CV_flag == 0)) //恒流结束
				{
					m_capacity_rec.cc_capacity = m_processData.capacity;
					m_capacity_rec.cc_time = (int)m_processData.run_time;
				}
				else if(m_processData.CC_CV_flag == 1)	//恒压
				{
					m_capacity_rec.cv_capacity = m_processData.capacity - m_capacity_rec.cc_capacity;
					m_capacity_rec.cv_time++;
				}
			}
		}
	}
	
	if((m_last_processData.run_state == STATE_NO_RUN || m_last_processData.run_state == STATE_RUNNING) 
		&& (m_last_processData.step_no > 0))
	{
		if((m_processData.err_mask > 0) && (m_processData.step_no == 0))	//下位机告警
		{
			m_last_processData.cc_capacity = m_capacity_rec.cc_capacity;
			m_last_processData.cc_second = m_capacity_rec.cc_time;
			m_last_processData.cv_capacity = m_capacity_rec.cv_capacity;
			//m_last_processData.cv_second = m_capacity_rec.cv_time;
			if((int)m_last_processData.run_time >= m_capacity_rec.cc_time)
			{
				m_last_processData.cv_second = (int)m_last_processData.run_time - m_capacity_rec.cc_time;
			}
			else
			{
				m_last_processData.cc_second = (int)m_last_processData.run_time;
				m_last_processData.cv_second = 0;
			}
		}
	}

	//停止
	if(m_last_processData.run_state == STATE_RUNNING)
	{
		if((m_processData.err_mask > 0) && (m_processData.step_no > 0))		//中位机或下位机告警
		{
			m_processData.cc_capacity = m_capacity_rec.cc_capacity;
			m_processData.cc_second = m_capacity_rec.cc_time;
			m_processData.cv_capacity = m_capacity_rec.cv_capacity;
			m_processData.cv_second = m_capacity_rec.cv_time;
		}

		if((m_processData.run_state == STATE_NO_RUN) && (m_processData.step_no == 0)) //主动停止
		{
			m_last_processData.cc_capacity = m_capacity_rec.cc_capacity;
			m_last_processData.cc_second = m_capacity_rec.cc_time;
			m_last_processData.cv_capacity = m_capacity_rec.cv_capacity;
			m_last_processData.cv_second = m_capacity_rec.cv_time;
		}
	}
}

void ChannelState::response_record_hook(Step_Process_Data_t &processData)
{
	m_processData = processData;
}

void ChannelState::push_data_to_CabDBServer(int cellNo,Step_Process_Data_t &data)
{
#ifdef STOREDATA
	if(g_CabDBServer.get_data_deque_size(cellNo) < 32)
	{
		g_CabDBServer.pushbackRecord(cellNo,data);
	}
#endif
}

void ChannelState::saveCutOffData(int cellNo,Step_Process_Data_t &data)
{
	data.is_stop = true;
	string store_josn_str = FormatConvert::instance().process_data_to_store_str_new(data);
	LOG_INFO(data_id_process, store_josn_str);
	LOG_INFO(data_id_stop, store_josn_str);

	push_data_to_CabDBServer(cellNo,data);
}

void ChannelState::saveProcessData(int cellNo,Step_Process_Data_t &data)
{
	data.is_stop = false;
	string store_josn_str = FormatConvert::instance().process_data_to_store_str_new(data);
	LOG_INFO(data_id_process, store_josn_str);

	push_data_to_CabDBServer(cellNo,data);
}

int ChannelState::return_suspend_cnt()
{
	return re_suspend_cnt;
}

void ChannelState::response_record_send(int cell_no,uint8 errorType,int &errorCode,bool &err_flag)
{
	//电芯类型 配方名 托盘码
	set_processData_info_code();

	m_processData.run_result = 0;

	if(m_processData.run_state == STATE_START)
	{
		totalRunTime = 0;
		memset((void*)&m_capacity_rec,0,sizeof(Step_Capacity_Record_t));

		if(m_last_processData.run_state == STATE_FINISH)
		{
			m_last_processData.run_state = STATE_NO_RUN;
			m_last_processData.step_no = 0;
		}
	}

	if(errorType > 0)
	{
		m_processData.err_mask |= errorType;
	}

	if(m_processData.err_mask > 0)
	{
		err_flag = true;
	}

	//运行过程中模块暂停
	if(m_processData.run_state == STATE_PAUSE)
	{
		if(re_suspend_cnt > 10)
		{
			err_flag = true;			//连续3条记录数据为暂停状态判通道异常
		}
		else
		{
			re_suspend_cnt++;
		}
	}
	else
	{
		re_suspend_cnt = 0;
	}
	
	// 统计运行时间
	do_runtime_counter();
	
	// 记录数据，发送给上位机
	//if (Configuration::instance()->need_send_detail_data(m_data.cell_no))
	{
		string josn_str = FormatConvert::instance().record_to_string(m_processData);
		MsgFactory::instance()->cell_to_ext_reply_pusher(cell_no).send(josn_str);
	}

	//有效工步状态: 运行 等待 完成 或从运行变为未运行
	if((m_processData.run_state > STATE_PAUSE && m_processData.run_state < STATE_CONTACT)
		|| (m_processData.run_state == STATE_NO_RUN && m_last_processData.run_state == STATE_RUNNING))		
	{
		bool has_recorded = false;
		
		if(m_last_processData.run_state != STATE_FINISH)	//只记录一次完成状态
		{
			//截止数据
			if(m_processData.step_no != m_last_processData.step_no)
			{
				reCounter = 0;
				
				if(m_processData.run_state == STATE_FINISH)  		//完成工步
				{
					m_last_processData.run_state = STATE_FINISH;	//修改上一个运行工步状态为"完成"
					m_last_processData.run_result = 1;
					reset_info_code();								//工艺完成 清除工艺信息
				}
				
				if((m_last_processData.step_no != 0) && (m_last_processData.err_mask == 0))
				{
					m_last_processData.timestamp += 1;				//避免时间戳重复
					saveCutOffData(cell_no,m_last_processData);		//上一工步最后1条数据
					has_recorded = true;
				}

				if((m_processData.run_state != STATE_FINISH) && (m_processData.run_state != STATE_NO_RUN))
				{
					saveCutOffData(cell_no,m_processData);		//新工步第1条数据
					has_recorded = true;
				}
			}
			else
			{
				//过程数据
				reCounter++;
				if(reCounter >= recFreq)
				{
					reCounter = 0;
					if(m_processData.run_state == STATE_WAITING)
					{
						m_processData.totalTime = m_last_processData.totalTime;
					}

					if(errorType == 0)    //中位机无告警才记录过程数据
					{
						saveProcessData(cell_no,m_processData);
					}
					has_recorded = true;
				}
				
#if defined(AWH_FORMATION_SYS)
				if(m_processData.loop_no == m_last_processData.loop_no)
				{
					m_processData.sum_step = m_last_processData.sum_step;
				}
#endif				
			}
		}

		cache_step_processRecord(m_processData,has_recorded);		//缓存最近10条
	}

	if(errorType > 0)		//中位机告警项
	{
		if(m_last_processData.run_state == STATE_RUNNING)
		{
			if(m_processData.step_no > 0)
			{	
				m_processData.run_state = STATE_NO_RUN;
				m_processData.timestamp += 2;
				m_processData.run_result = 2;						// 异常结束 NG
				m_processData.ng_reason = errorCode;				// 告警码
				errorCode = 0;
				saveCutOffData(cell_no,m_processData);				// 告警时刻的数据作为截止数据
				reset_info_code();									//工艺完成 清除工艺信息
			}
		}
	}
	else					//下位机告警项
	{
		if(m_processData.err_mask > 0)
		{
			if((m_last_processData.step_no > 0) && (m_processData.step_no == 0)) 	//告警
			{	
				m_last_processData.run_state = STATE_NO_RUN;
				m_last_processData.timestamp += 2;
				m_last_processData.run_result = 2;						// 异常结束 NG
				m_last_processData.ng_reason = errorCode;				// 告警码
				errorCode = 0;
				saveCutOffData(cell_no,m_last_processData);				// 告警时刻的数据作为截止数据
				reset_info_code();										//工艺完成 清除工艺信息
			}
		}
	}

	if((m_last_processData.run_state == STATE_RUNNING) && (m_processData.err_mask == 0))
	{
		if((m_processData.run_state == STATE_NO_RUN) && (m_processData.step_no == 0)) //主动停止
		{
			m_last_processData.run_state = STATE_NO_RUN;
			m_last_processData.timestamp += 3;	
#if (defined(AWH_FORMATION_SYS) || defined(AWH_GRADING_SYS))
			if(errorCode != 0)
			{
				m_last_processData.run_result = 2;
				m_last_processData.ng_reason = errorCode;		// 告警码
				errorCode = 0;
			}
			else
#endif
			{
				m_last_processData.run_result = 1;				// 正常结束
			}
			saveCutOffData(cell_no,m_last_processData);		// 告警前1次数据作为截止数据
			reset_info_code();								//工艺完成 清除工艺信息
		}
	}
	
	//备份
	if(m_processData.run_state != STATE_WAITING)	//等待状态不计
	{
		m_last_processData = m_processData;
	}
}


