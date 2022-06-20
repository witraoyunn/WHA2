#ifndef SERIALIZE_H_
#define SERIALIZE_H_

#include "utility.h"
#include "Type.h"

#ifdef PROTOCOL_5V160A
	#define MAX_CMD_SIZE 512
#else
	#define MAX_CMD_SIZE 256
#endif

class Serialize
{
public:

#ifdef PROTOCOL_5V160A
	Serialize(int ch_no, int func_code, Channel_Data_t &data,uint8 feat);
#else
    Serialize(int func_code, Channel_Data_t &data);
#endif

    uint8 * bytes_buffer(){ return m_cmd_buffer; }
    int bytes_size(){ return m_size; }
	
private:	
    uint8 m_cmd_buffer[MAX_CMD_SIZE];
	ChannelProto_JN9504 sendSerialize;
    int m_size;
};

#endif // SERIALIZE_H_

