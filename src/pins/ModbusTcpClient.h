#ifndef MODBUSTCPCLIENT_H_
#define MODBUSTCPCLIENT_H_

#include "modbus.h"

#include <stdint.h>
#include <vector>

class ModbusTcpClient
{
public:
	ModbusTcpClient(const char *ip, int port);
	~ModbusTcpClient();

	void connect();

	int read(int addr, bool *out, int len=1);
	int read(int addr, uint8_t *out, int len=1);
	int	read(int addr, uint16_t *out, int len=1);
	int read(int addr, int *out, int len=1);
	int read(int addr, float *out, int size=1);

	void write(int addr, bool val);
	void write(int addr, uint8_t val);
	void write(int addr, uint16_t val);
	void write(int addr, const uint16_t *val, int size);
	void write(int addr, int val);
	void write(int addr, const int *val, int size);
	void write(int addr, float val);
	void write(int addr, const float *val, int size);

private:
	modbus_t *m_client;
};

#endif // MODBUSTCPCLIENT_H_

