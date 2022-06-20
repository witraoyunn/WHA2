#ifndef CHANNEL_H_
#define CHANNEL_H_

#include <thread>
#include <string>
#include <deque>
#include <queue>
#include <list>
#include <time.h>
#include "GNSocket.h"
#include "Serialize.h"
#include "ReplyHandler.h"
#include "ChannelState.h"
#include "Type.h"

#define HEARTBEAT_LOST_ABORT	32		// 心跳丢失时间
#define MAX_READ_BYTE_NBR		512
#define MAX_CMD_TIMEOUT     	5   	//unit:second.


class Channel
{
public:
    Channel() {}
    Channel(int cell_no, int ch_no);
	virtual ~Channel(){}

	Channel& operator= (const Channel& obj);
    void push_command(Cmd_Channel_Msg_t ch_cmd);

	bool is_process_end();						// 工步运行完成或异常结束
	void reset_step_finish_status();		// 重置工步开始标志
#if (defined(AWH_GRADING_SYS) || defined(AWH_PRECHARGE_SYS))
	bool is_stepcache_null();					// 缓存工步为空
	void reset_stepcache_status();			// 重置工步缓存标志
#endif	
	
#if 0
	void set_vac_val(float val);
	void set_celltemp_value(float *temper);
#endif
#ifndef PROTOCOL_5V160A	
	bool is_running();
#endif
#if 0	
	bool is_vac_step();
	int8_t vac_Param();
#endif	

	void set_tech_info(int type, const char *src);
	void set_barcode(const char *src);
	void clear_channel_info();

public:
	bool						  m_is_connected;

//private:
protected:
    int  m_cell_no;
    int  m_ch_no;

    std::deque<Cmd_Channel_Msg_t> m_cmd_queue;
    ReplyHandler                  m_replyer;
};


#ifndef PROTOCOL_5V160A	
class ChannelClient:public Channel
{
public:
	ChannelClient(int cell_no, int ch_no):Channel(cell_no,ch_no){}
	~ChannelClient(){}
	
    void connect();
    void disconnect();
	bool isConnected();
	//bool is_channel_exist() { return m_is_en; }
	int  execute();
	void sync_system_time();
	bool check_beating();
    void start_process(int step_no);	// 开始运行工步
    void stop_process();				// 停止运行工步
	void pause_process();				// 暂停运行工步
	void vac_step_process();			// 多段负压工步

	void send_data(uint8 *src, int size);
	std::vector<int> handle_reply();

public:
	GNSocket m_device_socket;

};
#endif

#endif //CHANNEL_H_

