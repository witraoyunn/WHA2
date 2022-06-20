#ifndef AUXTEMPACQ_H
#define AUXTEMPACQ_H

#include <string>
#include <thread>
#include <arpa/inet.h>

#include "Type.h"
#include "EpollMan.h"

#define AUX_TEMP_ACQ_CH_NUM		16					// 单个辅助温度采集盒的通道数量
#define CHAN_TEMP_ACQ_CHANNELS	12
#define CELL_TEMP_ACQ_CHANNELS	3

//#define AUX_TEMP_CHANNEL_NUM	16					// 用于通道温度采集的采集盒数量
//#define AUX_TEMP_CELL_NUM		8					// 用于库位温度采集的采集盒数量
#define AUX_TEMP_ACQ_NUM		24					// 辅助温度采集盒的总数量

typedef enum
{
	SUB_CMD_TEMP_UP_CYCLE	= 0x01,			// 电池温度上抛周期
    SUB_CMD_BAT_TEMP_QUERY 	= 0x02,   		// 电池温度查询
} TempAcq_SubCode_Enum;

typedef struct
{
	float chan1;
	float chan2;
	float chan3;
	float chan4;
	float chan5;
	float chan6;
	float chan7;
	float chan8;
	float chan9;
	float chan10;
	float chan11;
	float chan12;
	float chan13;
	float chan14;
	float chan15;
	float chan16;
}channel_temperature_t;


class TempAcqBox
{
public:
	TempAcqBox();
	~TempAcqBox();
	
	void set_fd(int fd);
	bool isConnected();
	void disconnect();
	void configuration(int deviceNo,uint8 cnt);
	void acquire_temperature_execute();
private:
	void send_data(uint8 *src, int size);

public:
	bool m_is_connected;
private:
	int dev_fd;
	int dev_no;
	uint8 feat;
	std::vector<uint8> m_channels;
};

class AuxTempAcq
{	
public:
	AuxTempAcq();
	~AuxTempAcq();

	void init();
	std::thread run();
	void get_channel_temperature(int cellNo,channel_temperature_t &front,channel_temperature_t &mid,channel_temperature_t &back);
	void get_cell_temperature(int cellNo,channel_temperature_t &data);

private:
    void work();
	void tempAcqBox_state_update();
	int handle_a_frame(int *ack_size);
	void recv_handle(uint8 *p_buf, int size);
	void work_logic_handle();
	void readLock(pthread_rwlock_t *rwlock);
	void writeLock(pthread_rwlock_t *rwlock);
	void rwUnLock(pthread_rwlock_t *rwlock);

private:
#if 1
	pthread_rwlock_t entire_rwlock;
#else
	pthread_rwlock_t ch_rwlock;
	pthread_rwlock_t cell_rwlock;
#endif
	long m_dwLastTick;

	EpollMan m_epoll_tempAcq;
	ChannelProto_JN9504 recvReply;
	RingQueue<uint8> m_buffer;
	ChannelState m_state;
	
	TempAcqBox m_tempAcq_box[AUX_TEMP_ACQ_NUM];
	uint8 m_failed_count[AUX_TEMP_ACQ_NUM];
	
	float m_channel_temp[MAX_CELLS_NBR][MAX_CHANNELS_NBR];	//各通道电池温度
	float m_cell_temp[MAX_CELLS_NBR][AUX_TEMP_ACQ_CH_NUM];	//库位温度
	uint8 ch_acq_err_cnt[MAX_CELLS_NBR][MAX_CHANNELS_NBR];
	uint8 cell_acq_err_cnt[MAX_CELLS_NBR][AUX_TEMP_ACQ_CH_NUM];
};

extern AuxTempAcq g_AuxTempAcq;

#endif

