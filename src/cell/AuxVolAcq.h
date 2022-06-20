#ifndef AUXVOLACQ_H
#define AUXVOLACQ_H

#ifdef AUX_VOLTAGE_ACQ

#include <string>
#include <thread>
#include <time.h>

#include "Type.h"
#include "GNSocket.h"

#define AUX_VOL_ACQ_CH_NUM			16					// 单个辅助电压采集盒的通道数量
#define AUX_VOL_ACQ_CELL_NUM		3					// 单个库的辅助电压采集盒的数量
#define AUX_VOL_ACQ_NUM				(3 * MAX_CELLS_NBR)	// 辅助电压采集盒的数量

#define AUX_VOL_ACQ_IP				"192.168.1.%d"
#define AUX_VOL_ACQ_PORT			5000

#define RECV_BUFFER_SIZE			512

class AuxVolAcq
{		
public:
    AuxVolAcq() {}
    AuxVolAcq(int cell_no);
    AuxVolAcq& operator= (const AuxVolAcq& obj);
	~AuxVolAcq();
	
	std::thread run();

private:
    void work();
	void workLogic();
	void connect();
	void connect(int module_no);
	void disconnect();
	void disconnect(int module_no);
	void request_msg_handle();
	void response_msg_handle();
	void package(int module_no, uint8_t funcode, uint8_t subcode, const uint8_t *data=NULL, int len=0);
	int  unpackage(int module_no, const uint8_t *data, int len);
	void alarmHandle(int module_no, uint16_t devStatus);

private:
	int       m_cell_no;								// 库号
	bool      m_moduleEn_flag[AUX_VOL_ACQ_CELL_NUM];	// 采集盒使能标志
    GNSocket  m_socket[AUX_VOL_ACQ_CELL_NUM];			// socket
	bool      m_isConnected[AUX_VOL_ACQ_CELL_NUM];		// 连接状态
	
    time_t    m_reconnect_tm = 0;						// 记录网络重连时间
	char      m_recv_buff[AUX_VOL_ACQ_CELL_NUM][RECV_BUFFER_SIZE];
	int       m_recv_len[AUX_VOL_ACQ_CELL_NUM];
    time_t    m_last_query_tm = 0;						// 最后一次查询时间
  	int		  m_query_lost_tm[AUX_VOL_ACQ_CELL_NUM]; 	// 查询超时次数

	bool      m_isProbeExtend = false;					// 针床压合状态
};

#endif
#endif

