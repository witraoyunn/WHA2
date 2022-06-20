#include <string>
#include "FormatConvert.h"
#include "ChannelServer.h"
#include "PinsWorker.h"
#include "CANSocketStack.h"

using namespace std;

uint8 ChannelServer::cellfeat[MAX_CELLS_NBR] = {0};

#if defined(AWH_FORMATION_SYS)
ChannelServer::ChannelServer(int cell_no, int ch_no,int can_idx):Channel(cell_no,ch_no)
#else
ChannelServer::ChannelServer(int cell_no, int ch_no):Channel(cell_no,ch_no)
#endif
{
	//printf("ChannelServer constructor cell_no=%d ch_no=%d\n",cell_no,ch_no);
	ch_fd = -1;
	m_error = 0;
	m_last_error = 0;
#if defined(AWH_FORMATION_SYS)	
	m_can_idx = can_idx;
#endif
	m_temp_update_flg = false;
	m_pause_cmd_flg = false;
	
	global_ERC_happen = false;
	step_ERC_happen = false;
	internal_ERC_happen = false;
	ACDC_ERC_happen = false;
	
	global_alarm_mask = 0;
	step_alarm_mask = 0;

	mid_error_code.clear();
	
	for(int i=0;i<GLOBAL_EXC_NUM;i++)
	{
		p_global_ExcTask[i] = NULL;
	}

	for(int i=0;i<STEP_EXC_NUM;i++)
	{
		p_step_ExcTask[i] = NULL;
	}
}

ChannelServer::~ChannelServer()
{
	for(int i=0;i<GLOBAL_EXC_NUM;i++)
	{
		if(p_global_ExcTask[i])
		{
			delete p_global_ExcTask[i];
			p_global_ExcTask[i] = NULL;
		}
	}

	for(int i=0;i<STEP_EXC_NUM;i++)
	{
		if(p_step_ExcTask[i])
		{
			delete p_step_ExcTask[i];
			p_step_ExcTask[i] = NULL;
		}
	}
}

#ifndef AWH_FORMATION_SYS
void ChannelServer::set_fd(int fd)
{
	ch_fd = fd;			
	reset_heartbeat();	// 心跳重置
}

int ChannelServer::get_fd()
{
	return ch_fd;
}

void ChannelServer::send_data(uint8 *src, int size)
{
	send(ch_fd,src,size,0);
	if(src[7]!=0x0D || (src[8]!=0x03 && src[8]<0x0C))
	{
		LOGFMTD("send-->%d-%2d: %s", m_cell_no, m_ch_no, hexstring(src, size).c_str());
	}
}
#endif

void ChannelServer::recv_data(recvData_t &recv)
{
	if(recv.len > 0)
	{
		recv_queue.push_back(recv);
	}
}

bool ChannelServer::isConnected()
{
	return m_is_connected;
}

void ChannelServer::disconnect()
{
#ifndef AWH_FORMATION_SYS
	if(ch_fd > 0)
	{
		close(ch_fd);
	}
#endif	

	m_is_connected = false;
	
	// 应答
    //string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, m_ch_no, ACK_DIS_CONNECTED, -1, -1);
    //MsgFactory::instance()->cell_to_ext_reply_pusher(m_cell_no).send(josn_str);
}

void ChannelServer::add_addition_status(Step_Process_Addition_Data_t &data)
{
	m_replyer.m_state.set_addition_status(data);
}

int ChannelServer::handle_reply(bool &err_flag)
{	
	if(!m_is_connected)  
	{
		return -1;
	}

	recvData_t recv;
	while(recv_queue.size() > 0)
	{
		try
		{
			recv = recv_queue.front();
			recv_queue.pop_front();

			// 处理接收的数据
			if(recv.len > 0)
			{
				m_replyer.append((uint8 *)recv.data, recv.len);
			}

			if(m_replyer.m_state.is_processData_update())
			{
				//更新最近的两次记录数据
#if (defined(AWH_GRADING_SYS) || defined(AWH_PRECHARGE_SYS))				
				if(m_record.proceData.run_state != STATE_WAITING)	//预充 定容 不缓存等待状态下的通道数据
#endif
				{
					m_last_record.proceData = m_record.proceData;
				}
				m_replyer.m_state.get_step_processData(m_record.proceData);

#if defined(AWH_FORMATION_SYS)
				if(m_record.proceData.step_no > 0 && m_record.proceData.step_no <= stepArrary.size())
				{
					m_record.proceData.vac_set = stepArrary[m_record.proceData.step_no-1].VacValue;
				}
#endif				
				//查询下位机是否已产生告警
				check_lower_error_alarm();
				//全局普通保护任务
				do_global_exception_task(PROTECT_TYPE_GENERAL);
				//单工步保护任务
				do_step_exception_task();
				
				//记录数据上报及存储数据上抛
				do_record_send_task(err_flag);
				//保护触发额外任务
				do_exception_extra_task();
			}
		}
		catch (int &e)
		{
#ifndef AWH_FORMATION_SYS		
			if(ch_fd > 0)
			{
				close(ch_fd);
			}
#endif			
			m_is_connected = false;

			e = -603;
			string josn_str = FormatConvert::instance().alarm_to_string(e, m_cell_no, m_ch_no);
			MsgFactory::instance()->cell_alarm_pusher(m_cell_no).send(josn_str);
		}
	}
	
	if(m_temp_update_flg)
	{
		//全局温度保护任务
		if((time(NULL) - get_app_start_time()) > 60)		//上电60s后才检温度采集盒
		{
			do_global_exception_task(PROTECT_TYPE_TEMP);
		}
		
		m_temp_update_flg = false;
	}

	return 0;
}

#if defined(AUTO_CONNECT)
void ChannelServer::sync_system_time()
{
    if (m_is_connected)
    {
        Channel_Data_t data = Channel_Data_t();
		
		data.func_data.func_code = SUB_CMD_SET_SYS_TIME;
		data.func_data.value	 = 0;

		cellfeat[m_cell_no-1]++;
#if defined(AWH_FORMATION_SYS)
		if(cellfeat[m_cell_no-1] > 117)
		{
			cellfeat[m_cell_no-1] = 1;
		}
#endif
		Serialize data_stream(m_ch_no, CMD_SYSTEM_SET, data,cellfeat[m_cell_no-1]);
#if defined(AWH_FORMATION_SYS)
		g_can_dev[m_can_idx].CanWriteCache(m_cell_no, (m_ch_no-1)/MODULE_CH_NUM+1,cellfeat[m_cell_no-1],data_stream.bytes_buffer(),data_stream.bytes_size());
#else
		send_data(data_stream.bytes_buffer(), data_stream.bytes_size());
#endif
    }
}
#endif

void ChannelServer::reset_heartbeat()
{
	m_replyer.m_state.reset_heartbeat_tm();
}

bool ChannelServer::check_beating(bool en)
{
#ifndef AWH_FORMATION_SYS
    if (m_is_connected)
#endif
    {
        if (m_replyer.m_state.is_heartbeat_timeout())
        {
        	m_replyer.m_state.reset_heartbeat_tm();
        	if(en)
        	{	
				return false;
			}
			else
			{
				return true;
			}
        }
		// 下位机没收到心跳指令就会断开连接（对下位机而言，工步记录数据的上传，不能做为网络通信正常的依据）
		Channel_Data_t data = Channel_Data_t();
		
		data.func_data.func_code = SUB_CMD_BEAT_TRIGGER;
		data.func_data.value     = 0;

		cellfeat[m_cell_no-1]++;
		
#if defined(AWH_FORMATION_SYS)
		if(cellfeat[m_cell_no-1] > 117)
		{
			cellfeat[m_cell_no-1] = 1;
		}
#endif

		Serialize data_stream(m_ch_no, CMD_SYSTEM_SET, data,cellfeat[m_cell_no-1]);
#if defined(AWH_FORMATION_SYS)
		g_can_dev[m_can_idx].CanWriteCache(m_cell_no, (m_ch_no-1)/MODULE_CH_NUM+1,cellfeat[m_cell_no-1],data_stream.bytes_buffer(),data_stream.bytes_size());
#else
		send_data(data_stream.bytes_buffer(), data_stream.bytes_size());
#endif

		if(en)
		{
			m_replyer.m_state.set_heartbeat_tm();	
		}
    }

    return true;
}

int ChannelServer::execute()
{
    if (m_cmd_queue.empty())  return -1;

	// 发送命令给下位机
	if (!m_is_connected)  return -1;

	Cmd_Channel_Msg_t command = m_cmd_queue.front();
	m_cmd_queue.pop_front();				//发送成功  命令出队列

	cellfeat[m_cell_no-1]++;

#if defined(AWH_FORMATION_SYS)
	if(cellfeat[m_cell_no-1] > 117)
	{
		cellfeat[m_cell_no-1] = 1;
	}
#endif

	Serialize data_stream(command.ch_no, command.func_code, command.data,cellfeat[m_cell_no-1]);
#if defined(AWH_FORMATION_SYS)
	g_can_dev[m_can_idx].CanWriteCache(m_cell_no, (m_ch_no-1)/MODULE_CH_NUM+1,cellfeat[m_cell_no-1],data_stream.bytes_buffer(),data_stream.bytes_size());
#else
	send_data(data_stream.bytes_buffer(), data_stream.bytes_size());
#endif

	if(m_ch_no % MODULE_CH_NUM != 0)
	{
		msleep(10);
	}

	return 0;
}


void ChannelServer::transmit_data(uint8 *data,int len)
{
	// 发送命令给下位机
	if (!m_is_connected)  
	{
		return;
	}

	if(len > 6)
	{
#if defined(AWH_FORMATION_SYS)
		g_can_dev[m_can_idx].CanWriteCache(m_cell_no, (m_ch_no-1)/MODULE_CH_NUM+1,data[4],data,len);
#else
		send_data(data, len);
#endif
	}
}

#if defined(FORMATION_TECH_AUTO)
void ChannelServer::start_process(int step_no)
{
    if (!m_is_connected)  return; 
		
    Channel_Data_t data = Channel_Data_t();
    data.func_data.func_code = EXEC_PROCESS;
    data.func_data.value     = step_no;
	
	cellfeat[m_cell_no-1]++;
	Serialize data_stream(m_ch_no, CMD_RUN_CONTROL, data, cellfeat[m_cell_no-1]);

    send_data(data_stream.bytes_buffer(), data_stream.bytes_size());
}
#endif

void ChannelServer::stop_process()
{
    if (!m_is_connected)  return;
		
    Channel_Data_t data = Channel_Data_t();
    data.func_data.func_code = EXEC_STOP;
    data.func_data.value     = 1.0;
	
	cellfeat[m_cell_no-1]++;

#if defined(AWH_FORMATION_SYS)
	if(cellfeat[m_cell_no-1] > 117)
	{
		cellfeat[m_cell_no-1] = 1;
	}
#endif

	Serialize data_stream(m_ch_no, CMD_RUN_CONTROL, data, cellfeat[m_cell_no-1]);
#if defined(AWH_FORMATION_SYS)
	g_can_dev[m_can_idx].CanWriteCache(m_cell_no, (m_ch_no-1)/MODULE_CH_NUM+1,cellfeat[m_cell_no-1],data_stream.bytes_buffer(),data_stream.bytes_size());
#else
    send_data(data_stream.bytes_buffer(), data_stream.bytes_size());
#endif
	set_cmd_pause_flag(false);			//重置工步暂停标志
}

void ChannelServer::pause_process()
{
    if (!m_is_connected)  return;
		
    Channel_Data_t data = Channel_Data_t();
    data.func_data.func_code = EXEC_PAUSE;
    data.func_data.value     = 1.0;
	
	cellfeat[m_cell_no-1]++;
	
#if defined(AWH_FORMATION_SYS)
	if(cellfeat[m_cell_no-1] > 117)
	{
		cellfeat[m_cell_no-1] = 1;
	}
#endif

	Serialize data_stream(m_ch_no, CMD_RUN_CONTROL, data, cellfeat[m_cell_no-1]);
#if defined(AWH_FORMATION_SYS)
	g_can_dev[m_can_idx].CanWriteCache(m_cell_no, (m_ch_no-1)/MODULE_CH_NUM+1,cellfeat[m_cell_no-1],data_stream.bytes_buffer(),data_stream.bytes_size());
#else
    send_data(data_stream.bytes_buffer(), data_stream.bytes_size());
#endif
}

void ChannelServer::resume_process()
{
    if (!m_is_connected)  return;
		
    Channel_Data_t data = Channel_Data_t();
    data.func_data.func_code = EXEC_RESUME;
    data.func_data.value     = 1.0;
	
	cellfeat[m_cell_no-1]++;
#if defined(AWH_FORMATION_SYS)
	if(cellfeat[m_cell_no-1] > 117)
	{
		cellfeat[m_cell_no-1] = 1;
	}
#endif
	Serialize data_stream(m_ch_no, CMD_RUN_CONTROL, data, cellfeat[m_cell_no-1]);
#if defined(AWH_FORMATION_SYS)
	g_can_dev[m_can_idx].CanWriteCache(m_cell_no, (m_ch_no-1)/MODULE_CH_NUM+1,cellfeat[m_cell_no-1],data_stream.bytes_buffer(),data_stream.bytes_size());
#else
    send_data(data_stream.bytes_buffer(), data_stream.bytes_size());
#endif

}

#if defined(FORMATION_TECH_AUTO)
void ChannelServer::vac_step_process()
{
    if (!m_is_connected)  return;
		
    Channel_Data_t data = Channel_Data_t();
    data.func_data.func_code = STEP_VACNEXT;
    data.func_data.value     = 1.0;

	cellfeat[m_cell_no-1]++;
	Serialize data_stream(m_ch_no, CMD_RUN_CONTROL, data, cellfeat[m_cell_no-1]);

    send_data(data_stream.bytes_buffer(), data_stream.bytes_size());
}
#endif

void ChannelServer::tray_state_set(int state)
{
    if (!m_is_connected)  return;
		
    Channel_Data_t data = Channel_Data_t();
	uint8_t cmd = 0;
	
	if(state == TARY_UNPRESS_CMD)
	{
		cmd = SUB_CMD_PROBE_NO_PRESS;			//未压合
	}
	else if(state == TRAY_PROCESSING_CMD)
	{
		cmd = SUB_CMD_PROBE_PRESSING;			//压合中
	}
	else
	{
		cmd = SUB_CMD_PROBE_PRESSED;			//已压合
	}
    data.func_data.func_code = cmd;
    data.func_data.value     = 0;

	cellfeat[m_cell_no-1]++;

#if defined(AWH_FORMATION_SYS)
	if(cellfeat[m_cell_no-1] > 117)
	{
		cellfeat[m_cell_no-1] = 1;
	}
#endif

	Serialize data_stream(m_ch_no, CMD_SYSTEM_SET, data, cellfeat[m_cell_no-1]);
#if defined(AWH_FORMATION_SYS)
	g_can_dev[m_can_idx].CanWriteCache(m_cell_no, (m_ch_no-1)/MODULE_CH_NUM+1,cellfeat[m_cell_no-1],data_stream.bytes_buffer(),data_stream.bytes_size());
#else
    send_data(data_stream.bytes_buffer(), data_stream.bytes_size());
#endif
}

void ChannelServer::query_version()
{
    if (!m_is_connected)  return;

    Channel_Data_t data = Channel_Data_t();
    data.func_data.func_code = SOFTWARE_VERSION;
    data.func_data.value     = 0.0;
	
	cellfeat[m_cell_no-1]++;
#if defined(AWH_FORMATION_SYS)
	if(cellfeat[m_cell_no-1] > 117)
	{
		cellfeat[m_cell_no-1] = 1;
	}
#endif
	Serialize data_stream(m_ch_no, CMD_SYSTEM_GET, data, cellfeat[m_cell_no-1]);
#if defined(AWH_FORMATION_SYS)
	g_can_dev[m_can_idx].CanWriteCache(m_cell_no, (m_ch_no-1)/MODULE_CH_NUM+1,cellfeat[m_cell_no-1],data_stream.bytes_buffer(),data_stream.bytes_size());
#else
    send_data(data_stream.bytes_buffer(), data_stream.bytes_size());
#endif
}

bool ChannelServer::return_version_data(char *data)
{
	return m_replyer.m_state.get_version_info(data);
}

int ChannelServer::get_loop_vector_size()
{
	return multiLoopStep.size();
}

void ChannelServer::clear_loop_vector()
{
	multiLoopStep.clear();
}

void ChannelServer::push_loop_info(loopstep_info_t data)
{
	multiLoopStep.push_back(data);
}

int ChannelServer::get_loop_stepNo(int idx)
{
	return multiLoopStep[idx].StepNo;
}

void ChannelServer::set_loop_count(int idx,int count)
{
	multiLoopStep[idx].LoopCnt = count;
}

void ChannelServer::increase_loop_count(int idx)
{
	multiLoopStep[idx].LoopCnt += 1;
}

void ChannelServer::decrease_loop_count(int idx)
{
	if(multiLoopStep.size() > 0)
	{
		if(multiLoopStep[idx].LoopCnt > 0)
		{
			multiLoopStep[idx].LoopCnt -= 1;
		}
	}
}

int ChannelServer::get_loop_count(int idx)
{
	return multiLoopStep[idx].LoopCnt;
}

void ChannelServer::set_processData_RecTime(int recTime)
{
	m_replyer.m_state.set_record_frequency(recTime);
}

void ChannelServer::set_total_RunTime(int time)
{
	m_replyer.m_state.set_total_runtime(time);
}

int ChannelServer::get_total_RunTime()
{
	return m_replyer.m_state.get_total_runtime();
}

void ChannelServer::set_cmd_pause_flag(bool flag)
{
	m_pause_cmd_flg = flag;
}

bool ChannelServer::get_cmd_pause_flag()
{
	return m_pause_cmd_flg;
}

void ChannelServer::update_temperature_Data(AuxTempAcq_Record_Data_t &temp)
{
	//m_last_record.tempData = m_record.tempData;
	//m_record.tempData = temp;
	memcpy((void*)&m_last_record.tempData,(void*)&m_record.tempData,sizeof(AuxTempAcq_Record_Data_t));
	memcpy((void*)&m_record.tempData,(void*)&temp,sizeof(AuxTempAcq_Record_Data_t));
	m_temp_update_flg = true;
}

void ChannelServer::get_temperature_Data(AuxTempAcq_Record_Data_t &temp)
{
	//temp = m_record.tempData;
	memcpy((void*)&temp,(void*)&m_record.tempData,sizeof(AuxTempAcq_Record_Data_t));
}

int ChannelServer::get_channel_runstate()
{
	return m_record.proceData.run_state;
}

uint16_t ChannelServer::get_channel_step_no()
{
	return m_record.proceData.step_no;
}

int ChannelServer::get_channel_suspend_cnt()
{
	return m_replyer.m_state.return_suspend_cnt();
}

void ChannelServer::init_globalProtect_para()
{
	m_global_protect.VoltageFluctuateRange = -99.9;
	m_global_protect.AuxVoltage = -99.9;
	m_global_protect.CurrentFluctuateRange = -99.9;
	m_global_protect.TotalCapacityProtect = 0.0;
	m_global_protect.ContactResistance = -99.9;
	m_global_protect.LoopResistance = -99.9;
	m_global_protect.EndTotalCapacityLower = 0.0;
	m_global_protect.EndTotalCapacityUpper = 0.0;
	m_global_protect.VacuumFluctuate = -99.9;
	m_global_protect.BatteryLowerTemp = -99.9;
	m_global_protect.BatteryOverTemp = -99.9;
	m_global_protect.BatteryUltraTemp = -99.9;
	m_global_protect.BatteryUltraHighTemp = -99.9;
	m_global_protect.BatteryTempSmallFluctuateRange = -99.9;
	m_global_protect.BatteryTempLargeFluctuate = -99.9;
	m_global_protect.BatteryTempSuddenRise = -99.9;
	m_global_protect.CellLowerTemp = -99.9;
	m_global_protect.CellUltraTemp = -99.9;
	m_global_protect.CellUltraHighTemp = -99.9;

	m_global_protect.VoltageFluctuateCount = 0;
	m_global_protect.CurrentFluctuateCount = 0;
	m_global_protect.BatteryTempSmallFluctuateCount = 0;
}
	
void ChannelServer::get_globalProtect_info(global_protect_t &data)
{
	data = m_global_protect;
}

bool ChannelServer::get_globalProtect_para()
{
	XmlProcAPI globalProfile;

	char fl_path[128];
	char fl_buff[64];
	
	sprintf(fl_path,"%s",GLOBAL_PRO_XML_PATH);
	sprintf(fl_buff,"/GlobalProtect%d-%d.xml",m_cell_no,m_ch_no);
	strcat(fl_path,fl_buff);

	int ret = access(fl_path, 0);
	if (ret == 0)
	{		
		if(globalProfile.xmlopen(fl_path))
		{
			globalProfile.getGlobalProtect(m_global_protect);
			return true;
		}
		else
		{
			return false;
		}
	}

	return false;
}

void ChannelServer::get_workstep_recipe()
{
	XmlProcAPI recipefile; 
	char fl_path[128];
	char fl_buff[64];
	int TotalStep = 0;

	sprintf(fl_path,"%s",WORK_STEP_XML_PATH);
	sprintf(fl_buff,"/WorkstepRecipe%d-%d.xml",m_cell_no,m_ch_no);
	strcat(fl_path,fl_buff);

	int ret = access(fl_path, 0);
	if (ret == 0)
	{
		recipefile.xmlopen(fl_path);
		stepArrary.clear();
		
		TotalStep = recipefile.getTotalWorkstepCount();
	
		work_step_recipe_t stepInfo;
		for(int i = 1;i <= TotalStep; i++)
		{
			if(recipefile.getWorkstepPara(stepInfo,i))
			{
				stepArrary.push_back(stepInfo);
			}
		}
	}
}

void ChannelServer::get_workstep_info(uint32 index,work_step_recipe_t &step_info)
{
	if(stepArrary.size() > index)
	{
		step_info = stepArrary[index];
	}
}

uint32 ChannelServer::get_workstep_stepsize()
{
	return stepArrary.size();
}

void ChannelServer::set_stepProtect_Para(step_protect_t &para)
{
	m_step_protect = para;
#if 0
	printf("-----------------cell%d channel%d------------------\n",m_cell_no,m_ch_no);
	printf("UpperVoltag=%f\n",para.UpperVoltage);
	printf("LowerVoltage=%f\n",para.LowerVoltage);
	printf("VoltageOverDiff=%f\n",para.VoltageOverDiff);
	
	printf("PositiveDeltaVoltage=%f\n",para.PositiveDeltaVoltage);
	printf("PositiveDeltaVoltageCount=%d\n",para.PositiveDeltaVoltageCount);
	printf("NegativeDeltaVoltage=%f\n",para.NegativeDeltaVoltage);
	printf("NegativeDeltaVoltageCount=%d\n",para.NegativeDeltaVoltageCount);
	printf("CrossStepPositiveDeltaVoltage=%f\n",para.CrossStepPositiveDeltaVoltage);
	printf("CrossStepNegativeDeltaVoltage=%f\n",para.CrossStepNegativeDeltaVoltage);
	
	printf("UpperTimeVoltage=%f\n",para.UpperTimeVoltage);
	printf("LowerTimeVoltage=%f\n",para.LowerTimeVoltage);
	printf("TimeVoltageCheckTime=%d\n",para.TimeVoltageCheckTime);
	printf("RangeVoltageCheckTime1=%d\n",para.RangeVoltageCheckTime1);
	printf("RangeVoltageCheckTime2=%d\n",para.RangeVoltageCheckTime2);
	printf("DeltaTimeVoltage=%f\n",para.DeltaTimeVoltage);
	
	printf("OverCurrent=%f\n",para.OverCurrent);
	printf("CurrentOverDiff=%f\n",para.CurrentOverDiff);
	printf("PositiveDeltaCurrent=%f\n",para.PositiveDeltaCurrent);
	printf("PositiveDeltaCurrentCount=%d\n",para.PositiveDeltaCurrentCount);
	printf("NegativeDeltaCurrent=%f\n",para.NegativeDeltaCurrent);
	printf("NegativeDeltaCurrentCount=%d\n",para.NegativeDeltaCurrentCount);

	printf("OverCapacity=%f\n",para.OverCapacity);
	printf("BatteryUpperTemp=%f\n",para.BatteryUpperTemp);
	printf("BatteryLowerTemp=%f\n",para.BatteryLowerTemp);
#endif
}


void ChannelServer::push_error_code(int code)
{
	mid_error_code.push_back(code);
}

void ChannelServer::clear_error_code()
{
	mid_error_code.clear();
}

vector<int> ChannelServer::return_all_error_code()
{
	vector<int> err_code;
	
	err_code = m_replyer.m_state.get_error_code();									//下位机告警码
	if(mid_error_code.size() > 0)
	{
		err_code.insert(err_code.end(),mid_error_code.begin(),mid_error_code.end());	//在末尾插入中位机告警码
	}
	
	return err_code;
}

void ChannelServer::init_tempAcq_detect_exception()
{
	int mERC = 0;

	mERC = 72;
	if(p_global_ExcTask[CHAN_TEMPACQ_DETECT_INDEX] != NULL)
	{
		delete p_global_ExcTask[CHAN_TEMPACQ_DETECT_INDEX];
		//p_global_ExcTask[CHAN_TEMPACQ_DETECT_INDEX] = NULL;
	}
	p_global_ExcTask[CHAN_TEMPACQ_DETECT_INDEX] = new ChanTempAcqBoxDetect;
	p_global_ExcTask[CHAN_TEMPACQ_DETECT_INDEX]->init_task(mERC);
	
	mERC = 71;
	if(p_global_ExcTask[CELL_TEMPACQ_DETECT_INDEX] != NULL)
	{
		delete p_global_ExcTask[CELL_TEMPACQ_DETECT_INDEX];
		//p_global_ExcTask[CELL_TEMPACQ_DETECT_INDEX] = NULL;
	}
	p_global_ExcTask[CELL_TEMPACQ_DETECT_INDEX] = new CellTempAcqBoxDetect;
	p_global_ExcTask[CELL_TEMPACQ_DETECT_INDEX]->init_task(mERC);
}

void ChannelServer::init_global_exception()
{	
	int bERC[GLOBAL_EXC_NUM] = {0};
	
	for(int i=0;i<GLOBAL_EXC_NUM;i++)
	{
		if((i == CHAN_TEMPACQ_DETECT_INDEX) || (i == CELL_TEMPACQ_DETECT_INDEX))
		{
			continue;
		}
		
		if(p_global_ExcTask[i] != NULL)
		{
			delete p_global_ExcTask[i];
			p_global_ExcTask[i] = NULL;
		}
	}

	//电压波动
	if((m_global_protect.VoltageFluctuateRange > 0.0) && (m_global_protect.VoltageFluctuateCount> 0))
	{
		p_global_ExcTask[0] = new VoltageFluc;
		bERC[0] = 51;
	}

	//辅助电压异常
	if(m_global_protect.AuxVoltage > 0.0)
	{
		p_global_ExcTask[1] = new AuxtVoltageErr;
		bERC[1] = 52;
	}

	//电流波动
	if((m_global_protect.CurrentFluctuateRange > 0.0) && (m_global_protect.CurrentFluctuateCount> 0))
	{
		p_global_ExcTask[2] = new CurrntFluc;
		bERC[2] = 53;
	}

	//总电荷容量上限
	if(fabs(m_global_protect.TotalCapacityProtect) > 0.0001)	//总容量为负表示过放  为正表示过充
	{
		p_global_ExcTask[3] = new CapacityExceed;
		bERC[3] = 54;
	}

	//接触阻抗
	if(m_global_protect.ContactResistance > 0.0)
	{
		p_global_ExcTask[4] = new ContactResExceed;
		bERC[4] = 55;
	}

	//回路阻抗
	if(m_global_protect.LoopResistance > 0.0)
	{
		p_global_ExcTask[5] = new LoopResExceed;
		bERC[5] = 56;
	}

	//结束总电荷容量下限
	if(fabs(m_global_protect.EndTotalCapacityLower) > 0.0001)	//总容量为负表示过放  为正表示过充
	{
		p_global_ExcTask[6] = new EndCapacityLower;
		bERC[6] = 68;
	}

	//结束总电荷容量上限
	if(fabs(m_global_protect.EndTotalCapacityUpper) > 0.0001)	//总容量为负表示过放  为正表示过充
	{
		p_global_ExcTask[7] = new EndCapacityUpper;
		bERC[7] = 69;
	}

#if defined(AWH_FORMATION_SYS)	
	//负压波动保护
	if(m_global_protect.VacuumFluctuate > 0.0)			//负压波动保护
	{
		p_global_ExcTask[8] = new VacuumFluc;
		bERC[8] = 70;
	}
#endif

	//通道温度采集盒告警任务 占位
	//p_global_ExcTask[9]

	//电池温度下限
	if(m_global_protect.BatteryLowerTemp >= -50.0)
	{
		p_global_ExcTask[10] = new BatteryTempLowerLimit;
		bERC[10] = 57;
	}

	//电池温度过温
	if(m_global_protect.BatteryOverTemp >= -50.0)
	{
		p_global_ExcTask[11] = new BatteryTempOverLimit;
		bERC[11] = 58;
	}

	//电池温度超温
	if(m_global_protect.BatteryUltraTemp >= -50.0)
	{
		p_global_ExcTask[12] = new BatteryTempUltra;
		bERC[12] = 59;
	}

	//电池温度超高温
	if(m_global_protect.BatteryUltraHighTemp >= -50.0)
	{
		p_global_ExcTask[13] = new BatteryTempUltraOver;	
		bERC[13] = 60;
	}

	//温度小波动
	if((m_global_protect.BatteryTempSmallFluctuateRange > 0.0) && (m_global_protect.BatteryTempSmallFluctuateCount> 0))
	{
		p_global_ExcTask[14] = new BatteryTempSmallFluc;
		bERC[14] = 61;
	}

	//温度大波动
	if(m_global_protect.BatteryTempLargeFluctuate > 0.0)
	{
		p_global_ExcTask[15] = new BatteryTempLargeFluc;
		bERC[15] = 62;
	}
	
	//温度突升
	if(m_global_protect.BatteryTempSuddenRise > 0.0)
	{
		p_global_ExcTask[16] = new BatteryTempSuddenRise;
		bERC[16] = 63;
	}

	//库位温度采集盒告警任务 占位
	//p_global_ExcTask[17]
	
	//库位温度下限
	if(m_global_protect.CellLowerTemp >= -50.0)
	{
		p_global_ExcTask[18] = new CellTempLowerLimit;
		bERC[18] = 64;
	}

	//库位温度超温
	if(m_global_protect.CellUltraTemp >= -50.0)
	{
		p_global_ExcTask[19] = new CellTempOver;
		bERC[19] = 65;
	}

	//库位温度超高温
	if(m_global_protect.CellUltraHighTemp >= -50.0)
	{
		p_global_ExcTask[20] = new CellTempUltraOver;	
		bERC[20] = 66;
	}

	p_global_ExcTask[21] = new CellTempContrast;
	bERC[21] = 73;

	for(int i=0;i<GLOBAL_EXC_NUM;i++)
	{
		if((i == CHAN_TEMPACQ_DETECT_INDEX) || (i == CELL_TEMPACQ_DETECT_INDEX))
		{
			continue;
		}
		
		if(p_global_ExcTask[i] != NULL)
		{
			p_global_ExcTask[i]->init_task(bERC[i]);
		}
	}
}

void ChannelServer::init_step_exception()
{
	int bERC[STEP_EXC_NUM] = {0};

	for(int i=0;i<STEP_EXC_NUM;i++)
	{
		if(p_step_ExcTask[i] != NULL)
		{
			delete p_step_ExcTask[i];
			p_step_ExcTask[i] = NULL;
		}
	}
	
	//定时电压上限
	if((m_step_protect.UpperTimeVoltage > 0.0) && (m_step_protect.TimeVoltageCheckTime > 0))
	{
		p_step_ExcTask[0] = new TimingVoltageUpper;
		bERC[0] = 51;
	}

	//定时电压下限
	if((m_step_protect.LowerTimeVoltage > 0.0) && (m_step_protect.TimeVoltageCheckTime > 0))
	{
		p_step_ExcTask[1] = new TimingVoltageLower;
		bERC[1] = 52;
	}

	//电压正跳变
	if((m_step_protect.PositiveDeltaVoltage > 0.0) && (m_step_protect.PositiveDeltaVoltageCount > 0))
	{
		p_step_ExcTask[2] = new VoltagePosJump;
		bERC[2] = 54;
	}

	//电压负跳变
	if((m_step_protect.NegativeDeltaVoltage > 0.0) && (m_step_protect.NegativeDeltaVoltageCount > 0))
	{
		p_step_ExcTask[3] = new VoltageNegJump;
		bERC[3] = 55;
	}

	//跨工步电压变化上限
	if(m_step_protect.CrossStepPositiveDeltaVoltage > 0.0001)
	{
		p_step_ExcTask[4] = new CrossStepVoltageUpper;
		bERC[4] = 56;
	}

	//跨工步电压变化下限
	if(m_step_protect.CrossStepNegativeDeltaVoltage > 0.0001)
	{
		p_step_ExcTask[5] = new CrossStepVoltageLower;
		bERC[5] = 57;
	}

	//电流正跳变
	if((m_step_protect.PositiveDeltaCurrent > 0.0) && (m_step_protect.PositiveDeltaCurrentCount > 0))
	{
		p_step_ExcTask[6] = new CurrentPosJump;
		bERC[6] = 58;
	}

	//电流负跳变
	if((m_step_protect.NegativeDeltaCurrent > 0.0) && (m_step_protect.NegativeDeltaCurrentCount > 0))
	{
		p_step_ExcTask[7] = new CurrentNegJump;
		bERC[7] = 59;
	}

	//电池温度上限
	if(m_step_protect.BatteryUpperTemp >= -50.0)
	{
		p_step_ExcTask[8] = new StepCellTempUpper;
		bERC[8] = 60;
	}

	//电池温度下限
	if(m_step_protect.BatteryLowerTemp >= -50.0)
	{
		p_step_ExcTask[9] = new StepCellTempLower;
		bERC[9] = 61;
	}

	//定时电压变化上限保护
	if((m_step_protect.UpperDeltaRangeVoltage > 0.0) && (m_step_protect.RangeVoltageCheckTime1 > 0) && (m_step_protect.RangeVoltageCheckTime2 > 0))
	{
		p_step_ExcTask[10] = new TimingVoltageUpperDetect;
		bERC[10] = 62;
	}

	//定时电压变化下限保护
	if((m_step_protect.LowerDeltaRangeVoltage > 0.0) && (m_step_protect.RangeVoltageCheckTime1 > 0) && (m_step_protect.RangeVoltageCheckTime2 > 0))
	{
		p_step_ExcTask[11] = new TimingVoltageLowerDetect;
		bERC[11] = 63;
	}

	//工步结束电压下限
	if(m_step_protect.LowerStepStopVoltage > 0.0001)
	{
		p_step_ExcTask[12] = new StepEndVoltageLower;
		bERC[12] = 64;
	}

	//工步结束容量下限
	if(m_step_protect.LowerStepStopCapacity > 0.0001)
	{
		p_step_ExcTask[13] = new StepEndCapacityLower;
		bERC[13] = 65;
	}

	//工步结束时间下限
	if(m_step_protect.LowerStepStopTime > 0)
	{
		p_step_ExcTask[14] = new StepEndTimeLower;
		bERC[14] = 66;
	}

	//周期电压变化上限
	if((m_step_protect.UpperDeltaCycleVoltage > 0.0) && (m_step_protect.VoltageCheckCycleTime > 0))
	{
		p_step_ExcTask[15] = new CycleVoltageUpperDetect;
		bERC[15] = 67;
	}

	//周期电压变化下限
	if((m_step_protect.LowerDeltaCycleVoltage > 0.0) && (m_step_protect.VoltageCheckCycleTime > 0))
	{
		p_step_ExcTask[16] = new CycleVoltageLowerDetect;
		bERC[16] = 68;
	}
	
	for(int i=0;i<STEP_EXC_NUM;i++)
	{
		if(p_step_ExcTask[i] != NULL)
		{
			p_step_ExcTask[i]->init_task(bERC[i]);
		}
	}
}
	
void ChannelServer::do_global_exception_task(uint8 proType)
{
	int startItem;
	int endItem;
	
	if(proType == PROTECT_TYPE_GENERAL)
	{
		startItem = 0;
		endItem = GLOBAL_EXC_GENERAL_NUM;
	}
	else
	{
		startItem = GLOBAL_EXC_GENERAL_NUM;
		endItem = GLOBAL_EXC_NUM;
	}

	for(int i = startItem; i < endItem; i++)
	{
		if(p_global_ExcTask[i] != NULL)
		{
			if(p_global_ExcTask[i]->run_task(m_global_protect,m_record,m_last_record))
			{
				if((global_alarm_mask & (1 << i)) == 0) //相同告警只上报1次
				{
					p_global_ExcTask[i]->exception_alarm_send(m_cell_no,m_ch_no);
					m_error = (-2000 - p_global_ExcTask[i]->return_erc_code());
					push_error_code(m_error);			 //缓存中位机告警码
					global_ERC_happen = true;
					global_alarm_mask |= (1 << i);
				}

				//温度采集盒异常其他温度保护也不用判断了
				//if((i == CHAN_TEMPACQ_DETECT_INDEX) || (i == CELL_TEMPACQ_DETECT_INDEX))
				if(i == CHAN_TEMPACQ_DETECT_INDEX)
				{
					#if 0
					//工步运行中的温度采集盒异常通道暂停
					if((m_record.proceData.run_state == STATE_RUNNING) || (m_record.proceData.run_state == STATE_WAITING))
					{
						pause_process();
					}
					#endif
					
					break;
				}
			}
			else
			{
				if((i == CHAN_ULTRA_TEMP_INDEX) || (i == CHAN_ULTRAHIGH_TEMP_INDEX))
				{
					global_alarm_mask &= (~(0x00000001 << i));
				}
			}
		}
	}
}

uint8 ChannelServer::cell_ultra_temp_counter()
{
	uint8 cnt = 0;

	for(int i = 0; i < CELL_TEMP_ACQ_NUM; i++)
	{
		if((m_global_protect.CellUltraTemp > 0.0) && (m_record.tempData.cellTemp[i] > m_global_protect.CellUltraTemp))
		{
			cnt++;
		}
	}

	return cnt;
}

uint8 ChannelServer::cell_ultrahigh_temp_counter()
{
	uint8 cnt = 0;

	for(int i = 0; i < CELL_TEMP_ACQ_NUM; i++)
	{
		if((m_global_protect.CellUltraHighTemp > 0.0) && (m_record.tempData.cellTemp[i] > m_global_protect.CellUltraHighTemp))
		{
			cnt++;
		}
	}

	return cnt;
}

void ChannelServer::do_step_exception_task()
{
	for(int i=0;i<STEP_EXC_NUM;i++)
	{
		if(p_step_ExcTask[i] != NULL)
		{
			if(p_step_ExcTask[i]->run_task(m_step_protect,m_record,m_last_record))
			{
				if((step_alarm_mask & (1 << i)) == 0) //相同告警只上报1次
				{
					p_step_ExcTask[i]->exception_alarm_send(m_cell_no,m_ch_no);
					m_error = (-2100 - p_step_ExcTask[i]->return_erc_code());
					push_error_code(m_error);			//缓存中位机告警码
					step_ERC_happen = true;
					step_alarm_mask |= (1 << i);
				}
			}
		}
	}
}

void ChannelServer::set_error_code(int error)
{
	m_error = error;
}

void ChannelServer::check_lower_error_alarm()
{
	int error = m_replyer.m_state.get_lastest_error_code();

	if(error != 0)
	{
		m_error = error;
	}

	if((error != 0) && (m_last_error != error))
	{
		m_last_error = error;
		//LOGFMTD("%d-%2d m_error:%d", m_cell_no, m_ch_no, m_error);
		
		if(error < -2300)		//ACDC保护告警
		{
			ACDC_ERC_happen = true;
		}
		else if(error < -2200)	//内部保护告警
		{
			internal_ERC_happen = true;
		}
		else if(error < -2100)	//单工步告警
		{
			step_ERC_happen = true;
		}
		else if(error < -2000)	//全局告警
		{
			global_ERC_happen = true;
		}
	}
}

bool ChannelServer::is_protect_alarm_clear()
{
	if((global_alarm_mask != 0) || (step_alarm_mask != 0))
	{
		return false;
	}

	return true;
}

void ChannelServer::clear_protect_alarm_mask()
{
	global_alarm_mask = 0;
	step_alarm_mask = 0;
	
	global_ERC_happen = false;
	step_ERC_happen = false;
	internal_ERC_happen = false;
	ACDC_ERC_happen = false;
	
	m_replyer.m_state.clear_error_code();
	clear_error_code();
}


bool ChannelServer::is_error_in_global_alarm(uint8 errIdx)
{
	if((global_alarm_mask & (0x00000001 << errIdx)) != 0)
	{
		return true;
	}
	return false;
}


void ChannelServer::do_record_send_task(bool &err_flag)
{	
	uint8 err_type = 0;
	
	if(global_alarm_mask > 0)
	{
		err_type |= 0x01;
	}

	if(step_alarm_mask > 0)
	{
		err_type |= 0x02;
	}
	
	m_replyer.m_state.response_record_send(m_cell_no,err_type,m_error,err_flag);
}

void ChannelServer::do_exception_extra_task()
{
	if(global_ERC_happen || step_ERC_happen || internal_ERC_happen || ACDC_ERC_happen)
	{		
		global_ERC_happen = false;
		step_ERC_happen = false;
		internal_ERC_happen = false;
		ACDC_ERC_happen = false;
		
		m_replyer.m_state.report_processRecord_cache(m_cell_no);
#if 0
		if(!step_ERC_happen && !internal_ERC_happen && !ACDC_ERC_happen)
		{
			//如果是温度采集盒告警
			if(((global_alarm_mask & (0x00000001 << CHAN_TEMPACQ_DETECT_INDEX)) != 0)
				|| ((global_alarm_mask & (0x00000001 << CELL_TEMPACQ_DETECT_INDEX)) != 0))
			{
				return;
			}
		}
#endif
		stop_process();
	}
}

