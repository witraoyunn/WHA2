#include "ModbusTcpClient.h"
#include "log4z.h"
#include "modbus-tcp.h"

#include <string.h>

using namespace std;


ModbusTcpClient::ModbusTcpClient(const char *ip, int port)
{
	m_client = modbus_new_tcp(ip, port);

	if (m_client == NULL)
	{
		LOGFMTT("ModbusTcpClient::m_client is null,Unable to allocate libmodbus context---<<<" );
		throw -200;
	}
}

ModbusTcpClient::~ModbusTcpClient()
{
	if (m_client != NULL)
	{
		modbus_close(m_client);
		modbus_free(m_client);
	}
}

void ModbusTcpClient::connect()
{
	if (modbus_connect(m_client) < 0)
	{
		LOGFMTT("ModbusTcpClient::%s failed!", __FUNCTION__);
		throw - 201;
	}
}

int ModbusTcpClient::read(int addr, bool *out, int len)
{
	uint8_t dest[len];
	int     nb = len;

	int ret = modbus_read_bits(m_client, addr, nb, dest);
	if (ret != nb)
	{
		LOGFMTT("ModbusTcpClient::%s, %s.", __FUNCTION__, modbus_strerror(ret));
		throw - 205;
	}

	memcpy(out, dest, sizeof(dest));
	return len;
}

int ModbusTcpClient::read(int addr, uint8_t *out, int len)
{
	uint8_t dest[len];
	int     nb = len * 8;
	
	int ret = modbus_read_bits(m_client, addr, nb, dest);
	if (ret != nb)
	{
		LOGFMTT("ModbusTcpClient::%s, %s.", __FUNCTION__, modbus_strerror(ret));
		throw - 205;
	}
	
	for (int i=0; i<len; ++i)
	{
		*(out + i) = 0;
		
		for (int j=0; j<8; ++j)
		{
			*(out + i) |= dest[i*8 + j] << j;
		}
	}
	return len;
}

int ModbusTcpClient::read(int addr, uint16_t *out, int len)
{
	uint16_t dest[len];
	int      nb = len;
	
	int ret = modbus_read_registers(m_client, addr, nb, dest);
	if (ret != nb)
	{
		LOGFMTT("ModbusTcpClient::%s, %s.", __FUNCTION__, modbus_strerror(ret));
		throw - 205;
	}
	
	memcpy(out, dest, sizeof(dest));
	return len;
}

int ModbusTcpClient::read(int addr, int *out, int len)
{
	uint16_t dest[2*len];
	int	     nb = 2*len;

	int ret = modbus_read_registers(m_client, addr, nb, dest);
	if (ret != nb)
	{
		LOGFMTT("ModbusTcpClient::%s, %s.", __FUNCTION__, modbus_strerror(ret));
		throw - 205;
	}

	for (int i=0; i<len; i++)
	{
		out[i] = MODBUS_GET_INT32_FROM_INT16(dest, i*2);
	}
	
	return len;
}

int ModbusTcpClient::read(int addr, float *out, int len)
{
	uint16_t dest[2*len];
	int	     nb = 2*len;

	int ret = modbus_read_registers(m_client, addr, nb, dest);
	if (ret != nb)
	{
		LOGFMTT("ModbusTcpClient::%s, %s.", __FUNCTION__, modbus_strerror(ret));
		throw - 205;
	}

	// 三菱FX5U PLC
	for (int i=0; i<len; i++)
	{
		out[i] = modbus_get_float_badc(dest + (i*2));
	}

	return len;
}

void ModbusTcpClient::write(int addr, uint8_t val)
{
	uint8_t src[8];

	for (int i=0; i<8; ++i)
	{
		src[i] = (val >> i) & 0x01;
	}
	
	int ret = modbus_write_bits(m_client, addr, sizeof(src), src);
	if (ret != 1)
	{
		LOGFMTT("ModbusTcpClient::%s, %s.", __FUNCTION__, modbus_strerror(ret));
		throw - 204;
	}
	LOGFMTD("Modbus write: addr=%d, value=%d", addr, val);
}

void ModbusTcpClient::write(int addr, bool val)
{
	int ret = modbus_write_bit(m_client, addr, val);
	if (ret != 1)
	{
		LOGFMTT("ModbusTcpClient::%s, %s.", __FUNCTION__, modbus_strerror(ret));
		throw - 204;
	}
	LOGFMTD("Modbus write: addr=%d, value=%d", addr, val);
}

void ModbusTcpClient::write(int addr, uint16_t val)
{
	int ret = modbus_write_register(m_client, addr, val);
	if (ret != 1)
	{
		LOGFMTT("ModbusTcpClient::%s, %s.", __FUNCTION__, modbus_strerror(ret));
		throw - 204;
	}
	LOGFMTD("Modbus write: addr=%d, value=%d", addr, val);
}

void ModbusTcpClient::write(int addr, const uint16_t *val, int size)
{
	int	nb = size;
	
	int ret = modbus_write_registers(m_client, addr, nb, val);
	if (ret != nb)
	{
		LOGFMTT("ModbusTcpClient::%s, %s.", __FUNCTION__, modbus_strerror(ret));
		throw - 204;
	}
}

void ModbusTcpClient::write(int addr, int val)
{
	uint16_t src[2];
	int	     nb = 2;

	MODBUS_SET_INT32_TO_INT16(src, 0, val);

	int ret = modbus_write_registers(m_client, addr, nb, src);
	if (ret != nb)
	{
		LOGFMTT("ModbusTcpClient::%s, %s.", __FUNCTION__, modbus_strerror(ret));
		throw - 204;
	}
	LOGFMTD("Modbus write: addr=%d, value=%d", addr, val);
}

void ModbusTcpClient::write(int addr, const int *val, int size)
{
	uint16_t src[2*size];
	int	     nb = 2*size;

	for (int i=0; i<size; i++)
	{
		MODBUS_SET_INT32_TO_INT16(src, i*2, val[i]);
	}
	
	int ret = modbus_write_registers(m_client, addr, nb, src);
	if (ret != nb)
	{
		LOGFMTT("ModbusTcpClient::%s, %s.", __FUNCTION__, modbus_strerror(ret));
		throw - 204;
	}
}

void ModbusTcpClient::write(int addr, float val)
{
	uint16_t src[2];
	int	   nb = 2;

	// 三菱FX5U PLC
	modbus_set_float_badc(val, src);

	int ret = modbus_write_registers(m_client, addr, nb, src);
	if (ret != nb)
	{
		LOGFMTT("ModbusTcpClient::%s, %s.", __FUNCTION__, modbus_strerror(ret));
		throw - 204;
	}
	LOGFMTD("Modbus write: addr=%d, value=%f", addr, val);
}

void ModbusTcpClient::write(int addr, const float *val, int size)
{
	uint16_t src[2*size];
	int	     nb = 2*size;

	for (int i=0; i<size; i++)
	{
		// 三菱FX5U PLC
		modbus_set_float_badc(val[i], &src[i*2]);
	}
	
	int ret = modbus_write_registers(m_client, addr, nb, src);
	if (ret != nb)
	{
		LOGFMTT("ModbusTcpClient::%s, %s.", __FUNCTION__, modbus_strerror(ret));
		throw - 204;
	}
}

