#ifndef S7CLIENT_H_
#define S7CLIENT_H_

#include "utility.h"
#include "snap7.h"

class TS7Client;

class S7Client
{
public:
    static S7Client* instance();

    void connect(const char *ip);
    void disconnect();

    bool   read_bit(uint32 start_address, int bit_offset);
    uint8  read_byte(uint32 start_address);
	void   read_byte(uint32 start_address, uint8 *buf, int size);
    void   read_byte(uint8 DBNumber, uint32 start_address, uint8 *buf, int size);
    uint16 read_word(uint32 start_address);
    int    read_int(uint32 start_address);
    float  read_float(uint32 start_address);
    void   read_float(uint32 start_address, int size, float *array);

    void   write_bit(uint32 start_address, int bit_offset, bool value);
    void   write(uint32 start_address, uint8 value);
    void   write(uint32 start_address, uint16 value);
    void   write(uint32 start_address, int value);
    void   write(uint32 start_address, float value);
	bool   isconnect();

private:
    S7Client(){}
    S7Client(const S7Client&){}
    S7Client& operator=(const S7Client&);
    ~S7Client();

    TS7Client *m_client;
	bool       m_is_connected = false;
};

#endif //S7CLIENT_H_

