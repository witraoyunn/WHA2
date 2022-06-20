#ifndef UTILITY_H_
#define UTILITY_H_
#include <string>
#include <unistd.h>
#include "Configuration.h"

#define msleep(x) usleep((x)*1000)

typedef unsigned char   uint8;
typedef unsigned short  uint16;
typedef unsigned int    uint32;

long millitimestamp();
void set_mc_version(char *ver);
bool get_mc_version(char *ver);
void set_app_start_time();
time_t get_app_start_time();

#if defined(ENABLE_RUNSTATUS_SAVE)
void sys_mutex_pend();
void sys_mutex_post();
void sys_mutex_free();
#endif

uint16 crc16(uint8 *data, int len);
void endian_swap(uint8 *p_data, int start_idx, int length);

std::string hexstring(uint8 *data, int len);
void hexdump(void *mem, int len);

uint16 bytes_to_uint16(uint8 *p_data, uint8 size = 2);
void uint16_to_bytes(uint16 data, uint8 *p_data);

uint32 bytes_to_uint32(uint8 *p_data, uint8 size = 4);
void uint32_to_bytes(uint32 data, uint8 *p_data);

int bytes2_to_int16(uint8 *p_data);
int bytes4_to_int(uint8 *p_data);
int bytes_to_int(uint8 *p_data, uint8 size = 4);
void int_to_bytes(int data, uint8 *p_data);

long bytes_to_long(uint8 *p_data);
void long_to_bytes(long data, uint8 *p_data);


#endif //UTILITY_H_

