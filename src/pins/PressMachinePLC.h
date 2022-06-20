#ifndef PRESSMACHINEPLC_H
#define PRESSMACHINEPLC_H

#include <thread>
#include <string>
#include <stdint.h>

#include "ModbusTcpClient.h"

class PressMachinePLC {

public:
	PressMachinePLC() {}
    PressMachinePLC(int first_cell_no, int second_cell_no);
    PressMachinePLC& operator= (const PressMachinePLC& obj);

    std::thread run();

private:
	ModbusTcpClient *m_modbus_client;

	int  m_1th_cell_no;
	int  m_2th_cell_no;

private:
	void work();
//	int  statusUpdate();
//	void messageHandle();
//	void alarmDetectHandle();
};

#endif

