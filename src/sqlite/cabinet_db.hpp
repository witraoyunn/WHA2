#ifndef CABINET_DB_H_
#define CABINET_DB_H_

#include <thread>
#include <list>
#include "sqlite3.h"

class CabinetDateBase
{
public:
    CabinetDateBase();
    ~CabinetDateBase();
	
    std::thread run();

private:
	void work();

private:
	sqlite3 *m_db;
	std::list<std::string>  m_data_list;
};

#endif

