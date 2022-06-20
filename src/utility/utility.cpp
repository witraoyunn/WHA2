#include <sstream>
#include <iomanip>
#include <stdio.h>
#include <ctype.h>
#include "utility.h"
#include "log4z.h"
#include <time.h>
#include <sys/time.h>
#include <string>
#include <thread>

static time_t startstamp = 0;
#if defined(ENABLE_RUNSTATUS_SAVE)
pthread_mutex_t sys_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif
char mc_version[64] = {0};

long millitimestamp()
{
    struct timeval tv;
 
    gettimeofday(&tv, NULL);
    return (tv.tv_sec*1000 + tv.tv_usec/1000);
}

void set_mc_version(char *ver)
{
	if(strlen(ver) > 0)
	{
		strcpy(mc_version, ver);
	}
}

bool get_mc_version(char *ver)
{
	if(strlen(mc_version) > 0)
	{
		strcpy(ver, mc_version);
		return true;
	}
	else
	{
		return false;
	}
}

void set_app_start_time()
{
	startstamp = time(NULL);
}

time_t get_app_start_time()
{
	return startstamp;
}

#if defined(ENABLE_RUNSTATUS_SAVE)
void sys_mutex_pend()
{
	pthread_mutex_lock(&sys_mutex);
}

void sys_mutex_post()
{
	pthread_mutex_unlock(&sys_mutex);
}

void sys_mutex_free()
{
	pthread_mutex_destroy(&sys_mutex);
}
#endif

uint16 crc16(uint8 *data, int len)
{
    uint16 crc_code = 0xFFFF;
    if (len > 0)
    {
        for (int i = 0; i < len; i++)
        {
            crc_code = (uint16)(crc_code ^ (data[i]));
            for (int j = 0; j < 8; j++)
            {
                crc_code = (crc_code & 1) != 0 ? (uint16)((crc_code >> 1) ^ 0xA001) : (uint16)(crc_code >> 1);
            }
        }

        union
        {
            uint16 code;
            uint8 bytes[2];
        } uint16_union;

        uint16_union.code = 0;
        uint16_union.bytes[1] = (uint8)((crc_code & 0xFF00) >> 8); //高位置
        uint16_union.bytes[0] = (uint8)(crc_code & 0x00FF);      //低位置
        crc_code = uint16_union.code;
    }

    return crc_code;
}

void endian_swap(uint8 *p_data, int start_idx, int length)
{
    int cnt = length / 2;
    int start = start_idx;
    int end = start_idx + length - 1;
    for (int i = 0; i < cnt; i++)
    {
        uint8 tmp = p_data[start+i];
        p_data[start+i] = p_data[end-i];
        p_data[end-i] = tmp;
    }
}

std::string hexstring(uint8 *data, int len)
{
    std::ostringstream os;
	static const char alpha[] = "0123456789ABCDEF";

    for(int i = 0; i < len; i++)
    {
		os << alpha[data[i] >> 4] << alpha[data[i] & 0x0F] << " ";
    }

    return os.str();
}

#define HEXDUMP_COLS 16
void hexdump(void *mem, int len)
{
    int i, j;
    for(i = 0; i < len + ((len % HEXDUMP_COLS) ? (HEXDUMP_COLS - len % HEXDUMP_COLS) : 0); i++)
    {
        if(i % HEXDUMP_COLS == 0)
        {
            printf("0x%04X: ", i);
        }

        if(i < len)
        {
            printf("%02X ", 0xFF & ((char*)mem)[i]);
        }
        else
        {
            printf("   ");
        }

        if(i % HEXDUMP_COLS == (HEXDUMP_COLS - 1))
        {
            for(j = i - (HEXDUMP_COLS - 1); j <= i; j++)
            {
                if(j >= len)
                {
                    putchar(' ');
                }
                else if(isprint((((char*)mem)[j] & 0x7F)))
                {
                    putchar(0xFF & ((char*)mem)[j]);
                }
                else
                {
                    putchar('.');
                }
            }
            putchar('\n');
        }
    }
}

uint16 bytes_to_uint16(uint8 *p_data, uint8 size)
{
//    LOGFMTT("utility::%s data:%s size:%d --->>>", __FUNCTION__, hexstring(p_data, size).c_str(), size);

    union
    {
        uint16 code;
        uint8 bytes[2];
    } uint16_union;

    uint16_union.code = 0;
    for(uint8 i = 0; i < size; i++)
    {
        uint16_union.bytes[i] = p_data[size - 1 -i];
    }

//    LOGFMTT("utility::%s data:0x%x ---<<<", __FUNCTION__, uint16_union.code);

    return uint16_union.code;
}

void uint16_to_bytes(uint16 data, uint8 *p_data)
{
//    LOGFMTT("utility::%s data:0x%x --->>>", __FUNCTION__, data);
    union
    {
        uint32 code;
        uint8 bytes[2];
    } uint32_union;

    uint32_union.code = data;

    std::ostringstream os;
    os << std::showbase << std::hex;
    for(uint8 i = 0; i < 2; i++)
    {
        p_data[i] = uint32_union.bytes[2 - 1 -i];
        os << (int)p_data[i] << ", ";
    }

//    LOGFMTT("utility::%s data:%s ---<<<", __FUNCTION__, os.str().c_str());
}

uint32 bytes_to_uint32(uint8 *p_data, uint8 size)
{
//    LOGFMTT("utility::%s data:%s size:%d --->>>", __FUNCTION__, hexstring(p_data, size).c_str(), size);

    union
    {
        uint32 code;
        uint8 bytes[4];
    } uint32_union;

    uint32_union.code = 0;

    for(uint8 i = 0; i < size; i++)
    {
        uint32_union.bytes[i] = p_data[size - 1 -i];
    }

//    LOGFMTT("utility::%s data:%d ---<<<", __FUNCTION__, uint32_union.code);

    return uint32_union.code;
}

void uint32_to_bytes(uint32 data, uint8 *p_data)
{
//    LOGFMTT("utility::%s data:0x%x --->>>", __FUNCTION__, data);
    union
    {
        uint32 code;
        uint8 bytes[4];
    } uint32_union;

    uint32_union.code = data;

    std::ostringstream os;
    os << std::showbase << std::hex;
    for(uint8 i = 0; i < 4; i++)
    {
        p_data[i] = uint32_union.bytes[4 - 1 -i];
        os << (int)p_data[i] << ", ";
    }

//    LOGFMTT("utility::%s data:%s ---<<<", __FUNCTION__, os.str().c_str());
}

int bytes2_to_int16(uint8 *p_data)
{
//    LOGFMTT("utility::%s data:%s--->>>", __FUNCTION__, hexstring(p_data, 2).c_str());

    union
    {
        short code;
        uint8 bytes[2];
    } int16_union;

    int16_union.code = 0;

    int16_union.bytes[0] = p_data[1];
    int16_union.bytes[1] = p_data[0];

//    LOGFMTT("utility::%s short:%d data:0x%x dec:%d---<<<", __FUNCTION__, sizeof(short), int16_union.code, int16_union.code);

    return int16_union.code;
}

int bytes4_to_int(uint8 *p_data)
{
//    LOGFMTT("utility::%s data:%s --->>>", __FUNCTION__, hexstring(p_data, 4).c_str());

    union
    {
        int code;
        uint8 bytes[4];
    } int_union;

    int_union.code = 0;

    int_union.bytes[0] = p_data[3];
    int_union.bytes[1] = p_data[2];
    int_union.bytes[2] = p_data[1];
    int_union.bytes[3] = p_data[0];

//    LOGFMTT("utility::%s data:0x%x ---<<<", __FUNCTION__, int_union.code);

    return int_union.code;
}

int bytes_to_int(uint8 *p_data, uint8 size)
{
	if (size == 2)
	{
		if ((*p_data & 0x80) != 0)
		{
			return (int) (0xFFFF0000 | (*p_data << 8) | *(p_data+1));
		}
		else
		{
			return (int) ((*p_data << 8) | *(p_data+1));
		}
	}
	if (size == 3)
	{
		if ((*p_data & 0x80) != 0)
		{
			return (int) (0xFF000000 | (*p_data << 16) | (*(p_data+1) << 8) | *(p_data+2));
		}
		else
		{
			return (int) ((*p_data << 16) | (*(p_data+1) << 8) | *(p_data+2));
		}
	}
	else if (size == 4)
	{	
		return (int) ((*p_data << 24) | (*(p_data+1) << 16) | (*(p_data+2) << 8) | *(p_data+3));
	}
	else
	{
		return 0;
	}
}

void int_to_bytes(int data, uint8 *p_data)
{
//    LOGFMTT("utility::%s data:%d --->>>", __FUNCTION__, data);

    union
    {
        int code;
        uint8 bytes[4];
    } int_union;

    int_union.code = data;

//    LOGFMTW("utility::%s int_union.bytes:%s ---<<<", __FUNCTION__, hexstring(int_union.bytes, 4).c_str());

    p_data[0] = int_union.bytes[3];
    p_data[1] = int_union.bytes[2];
    p_data[2] = int_union.bytes[1];
    p_data[3] = int_union.bytes[0];

//    LOGFMTT("utility::%s p_data:%s ---<<<", __FUNCTION__, hexstring(p_data, 4).c_str());
}


long bytes_to_long(uint8 *p_data)
{
//    LOGFMTT("utility::%s data:%s --->>>", __FUNCTION__, hexstring(p_data, 8).c_str());

    union
    {
        long code;
        uint8 bytes[8];
    } int_union;

    int_union.code = 0;

    int_union.bytes[0] = p_data[7];
    int_union.bytes[1] = p_data[6];
    int_union.bytes[2] = p_data[5];
    int_union.bytes[3] = p_data[4];
    int_union.bytes[4] = p_data[3];
    int_union.bytes[5] = p_data[2];
    int_union.bytes[6] = p_data[1];
    int_union.bytes[7] = p_data[0];

//    LOGFMTT("utility::%s data:%ld ---<<<", __FUNCTION__, int_union.code);

    return int_union.code;
}


void long_to_bytes(long data, uint8 *p_data)
{
//    LOGFMTT("utility::%s data:%ld --->>>", __FUNCTION__, data);

    union
    {
        long code;
        uint8 bytes[8];
    } int_union;

    int_union.code = data;

//    LOGFMTW("utility::%s int_union.bytes:%s ---<<<", __FUNCTION__, hexstring(int_union.bytes, 8).c_str());

    p_data[0] = int_union.bytes[7];
    p_data[1] = int_union.bytes[6];
    p_data[2] = int_union.bytes[5];
    p_data[3] = int_union.bytes[4];
    p_data[4] = int_union.bytes[3];
    p_data[5] = int_union.bytes[2];
    p_data[6] = int_union.bytes[1];
    p_data[7] = int_union.bytes[0];

//    LOGFMTT("utility::%s p_data:%s ---<<<", __FUNCTION__, hexstring(p_data, 8).c_str());
}

