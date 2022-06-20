#include <bitset>
#include <stdio.h>
#include <stdlib.h>
#include "utility.h"
#include "snap7.h"
#include "utility.h"
#include "S7Client.h"
#include "log4z.h"

#define DB_NO 1

//将X的第Y位置1
#define SETBIT(x,y) 		(x |= (1<<y))
//将X的第Y位清0
#define CLRBIT(x,y) 		(x &= ~(1<<y))

using namespace std;

S7Client* S7Client::instance()
{
    static S7Client obj;
    return &obj;
}

S7Client::~S7Client()
{
    m_client->Disconnect();
    delete m_client;
}

bool S7Client::isconnect()
{
	return m_is_connected;
}

void S7Client::connect(const char *ip)
{
	m_is_connected = false;
	
    m_client = new TS7Client();

    short LocalTSAP = 0x0200;
    short RemoteTSAP = 0x0200;

    int error_code = m_client->SetConnectionParams(ip, LocalTSAP, RemoteTSAP);
    if (error_code != 0)
    {
        delete m_client;
        LOGFMTE("S7Client::%s SetConnectionParams to %s (LocalTSAP:%d, RemoteTSAP:%d) failed!", __FUNCTION__, ip, LocalTSAP, RemoteTSAP);
        throw -1000;
    }

    error_code = m_client->Connect();

    if (error_code != 0)
    {
        delete m_client;
        LOGFMTE("S7Client::%s Connect to %s (LocalTSAP:%d, RemoteTSAP:%d) failed!", __FUNCTION__, ip, LocalTSAP, RemoteTSAP);
        throw -1000;
    }
	
	m_is_connected = true;
}

void S7Client::disconnect()
{ 
//	m_client->Disconnect();
	delete m_client;
}

bool S7Client::read_bit(uint32 start_address, int bit_offset)
{
	if (!m_is_connected)  return false;
	
    uint8 value = 0;
    if (m_client->DBRead(DB_NO, start_address, 1, (void *)&value) != 0)
    {
		m_is_connected = false;
		return false;
    }
    bitset<8> bitvec(value);
    return bitvec[bit_offset];
}

uint8 S7Client::read_byte(uint32 start_address)
{
	if (!m_is_connected)  return 0;
	
    uint8 value = 0;
    if (m_client->DBRead(DB_NO, start_address, 1, (void *)&value) != 0)
    {
		m_is_connected = false;
		return 0;
    }
    return value;
}

void S7Client::read_byte(uint32 start_address, uint8 *buf, int size)
{
	if (!m_is_connected)
	{
		memset(buf, 0, size);
		return;
	}
	
    if (m_client->DBRead(DB_NO, start_address, size, buf) != 0)
    {
		m_is_connected = false;
		memset(buf, 0, size);
		return;
    }
}

void S7Client::read_byte(uint8 DBNumber, uint32 start_address, uint8 *buf, int size)
{
	if (!m_is_connected)
	{
		memset(buf, 0, size);
		return;
	}
	
    if (m_client->DBRead(DBNumber, start_address, size, buf) != 0)
    {
		m_is_connected = false;
		memset(buf, 0, size);
		return;
    }
}

uint16 S7Client::read_word(uint32 start_address)
{
	if (!m_is_connected)  return 0;
	
    uint32 byte_size = 2;
    uint16 value = 0;
    if (m_client->DBRead(DB_NO, start_address, byte_size, (void *)&value) != 0)
    {
		m_is_connected = false;
		return 0;
    }
    endian_swap((uint8 *)&value, 0, byte_size);
    return value;
}

int S7Client::read_int(uint32 start_address)
{
	if (!m_is_connected)  return 0;
	
    uint32 byte_size = 4;
    int    value = 0;
    if (m_client->DBRead(DB_NO, start_address, byte_size, (void *)&value) != 0)
    {
		m_is_connected = false;
		return 0;
    }
    endian_swap((uint8 *)&value, 0, byte_size);
    return value;
}

float S7Client::read_float(uint32 start_address)
{
	if (!m_is_connected)  return 0.0;
	
    uint32 byte_size = 4;
    float  value = 0;
    if (m_client->DBRead(DB_NO, start_address, byte_size, (void *)&value) != 0)
    {
		m_is_connected = false;
		return 0.0;
    }
    endian_swap((uint8 *)&value, 0, byte_size);
    return value;
}

void S7Client::read_float(uint32 start_address, int size, float *array)
{
	if (!m_is_connected)
	{
		memset((void *)array, 0, size);
		return;
	}
	
    uint32 byte_size = 4;
    if (m_client->DBRead(DB_NO, start_address, size, (void *)array) != 0)
    {
		m_is_connected = false;
		memset((void *)array, 0, size);
		return;
    }
    for(int i = 0; i < size; i++)
    {
        endian_swap((uint8 *)array, byte_size*i, byte_size);
    }
}

void S7Client::write_bit(uint32 start_address, int bit_offset, bool value)
{
	if (!m_is_connected)  return;
	
    uint32 byte_size = 1;
    uint8  byte_value = 0;
    if (m_client->DBRead(DB_NO, start_address, byte_size, (void *)&byte_value) != 0)
    {
		m_is_connected = false;
		return;
    }
    if (value)
    {
        SETBIT(byte_value, bit_offset);
    }
    else
    {
        CLRBIT(byte_value, bit_offset);
    }
    if (m_client->DBWrite(DB_NO, start_address, byte_size, (void *)&byte_value) != 0)
    {
		m_is_connected = false;
    }
}

void S7Client::write(uint32 start_address, uint8 value)
{
	if (!m_is_connected)  return;
	
    uint32 byte_size = 1;
    if (m_client->DBWrite(DB_NO, start_address, byte_size, (void *)&value) != 0)
    {
		m_is_connected = false;
		return;
    }
}

void S7Client::write(uint32 start_address, uint16 value)
{
	if (!m_is_connected)  return;
	
    uint32 byte_size = 2;
    endian_swap((uint8 *)&value, 0, byte_size);
    if (m_client->DBWrite(DB_NO, start_address, byte_size, (void *)&value) != 0)
    {
		m_is_connected = false;
		return;
    }
}

void S7Client::write(uint32 start_address, float value)
{
	if (!m_is_connected)  return;
	
    uint32 byte_size = 4;
    endian_swap((uint8 *)&value, 0, byte_size);
    if (m_client->DBWrite(DB_NO, start_address, byte_size, (void *)&value) != 0)
    {
		m_is_connected = false;
		return;
    }
}

void S7Client::write(uint32 start_address, int value)
{
	if (!m_is_connected)  return;
	
    uint32 byte_size = 4;
    endian_swap((uint8 *)&value, 0, byte_size);
    if (m_client->DBWrite(DB_NO, start_address, byte_size, (void *)&value) != 0)
    {
		m_is_connected = false;
		return;
    }
}

