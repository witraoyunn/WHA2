#ifndef CABINET_DB_H_
#define CABINET_DB_H_

#include <thread>
#include <list>
#include "sqlite3.h"
#include <deque>
#include "Type.h"
#include "ChannelProto_JN9504.h"

class CabinetDateBase
{
public:
    CabinetDateBase();
    ~CabinetDateBase();
		
    std::thread run();
	
	void pushbackRecord(int cellNo,Step_Process_Data_t &data);
	bool popfrontRecord(int cellNo,Step_Process_Data_t &data);
	int get_data_deque_size(int cellNo);
	
private:
	void work();

#ifdef ZHONGYUAN_DB
	void offline_data_handle();
#endif

private:
#ifdef ZHONGYUAN_DB
	sqlite3 *m_db;
	std::list<Step_Process_Data_t> m_data_list;
#else
	pthread_mutex_t mutex[MAX_CELLS_NBR];
	std::deque<Step_Process_Data_t> m_data_deque[MAX_CELLS_NBR];
#endif

    int m_line_no;
    int m_cabinet_no;
};

extern CabinetDateBase g_CabDBServer;
#endif

