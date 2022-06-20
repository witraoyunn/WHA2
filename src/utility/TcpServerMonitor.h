#ifndef _TCP_SERVER_MONITOR_H_
#define _TCP_SERVER_MONITOR_H_

#include <string>
#include <thread>
#include <map>
#include "ChannelProto_JN9504.h"
#include "AuxTempAcq.h"

#define SERV_PORT 5000

enum conn_state_e
{
	NoConnect = 0,
	NewConnect = 1,
	Connected = 2,
	DisConnect =3,
};

enum temperature_type_e
{
	CHANNEL_ACQ_T = 0,
	CELL_ACQ_T = 1,
};

typedef struct
{
	int state;
	int ch_fd;
	std::string ip;
}client_state_info_t;

typedef struct
{
	int device;
	int cellNo;
	int chanNo[MODULE_CH_NUM];
}cell_channel_No_t;

typedef struct
{
	int device;
	int cellNo;
	int channelTempNo[AUX_TEMP_ACQ_CH_NUM];
	int cellTempNo[AUX_TEMP_ACQ_CH_NUM];
}tempAcq_cell_channel_No_t;

class TcpServerMonitor
{
public:
	TcpServerMonitor();
	~TcpServerMonitor();

	int init();
	std::thread run();
	void readLock(int cell_no);
	void writeLock(int cell_no);
	void rwUnLock(int cell_no);	
	void OSSemPend(pthread_mutex_t *mutex);
	void OSSemPost(pthread_mutex_t *mutex);

	int get_client_state(int cell_no, int device_no);
	void set_client_state(int cell_no, int device_no, int state);
#ifndef AWH_FORMATION_SYS
	int get_client_connfd(int cell_no, int device_no);
	std::string get_client_ip(int cell_no, int device_no);
	bool get_client_channels_by_ip(std::string ip, std::vector<int> &channels);
	bool get_client_channels_by_fd(int fd, int &device, std::vector<int> &channels);
	void delete_fd_from_map(int fd);
#else
	bool get_client_channels(int cell_no,int device_no, std::vector<int> &channels);
#endif

	int get_tempAcqBox_connfd(int device_no);
	int get_tempAcqBox_state(int device_no);
	void set_tempAcqBox_state(int device_no, int state);
	bool get_tempAcqNo_by_fd(int fd, tempAcq_cell_channel_No_t &tempAcqNo);
	void delete_fd_from_tempAcqmap(int fd);
private:
	void accept_monitor();

private:
	int sock_fd;
	pthread_mutex_t tempAcqMutex;
	pthread_mutex_t channelMutex;
	pthread_rwlock_t rwlock[MAX_CELLS_NBR];
	std::map<std::string,tempAcq_cell_channel_No_t> tempAcq_map_ip_chanNo;
	std::map<int,tempAcq_cell_channel_No_t> tempAcq_map_fd_chanNo;
#ifndef AWH_FORMATION_SYS	
	std::map<std::string,cell_channel_No_t> map_ip_chanNo;
	std::map<int,cell_channel_No_t> map_fd_chanNo;
#else
	std::map<int,cell_channel_No_t> cell_device_chanNo;
#endif	
	client_state_info_t tempAcq_info_table[AUX_TEMP_ACQ_NUM];
	client_state_info_t client_info_table[MAX_CELLS_NBR][MAX_CHANNELS_NBR];
};

extern TcpServerMonitor g_TcpServer;

#endif