#ifndef OMRON_FIN_NET_H
#define OMRON_FIN_NET_H
#include <string>
#include "GNSocket.h"
class  OmronFinsNet
{
public:
	OmronFinsNet();
	~OmronFinsNet();
	void connect_server(std::string ip, int port);
	void initialize();
	void read_from_server(int length, char* data);
	void read_header_from_server(char* data);
	void reader_command_content_from_server(int length, char* data);

	void read_data(char *pack, int pack_len, char* data, int* data_len);
	bool read_bool(std::string addr);
	int read_int(std::string addr);
	void write_bool(std::string addr, bool value);

	void build_read_command(std::string addr, int length, char* command, int* command_len, bool isBit);
	void build_write_command(std::string addr, char* value, int value_len, char* command, int* command_len, bool isBit);
	void pack_command(char* cmd, int cmd_len, char* data_pack, int* data_pack_len);
private:
	char ICF = 0x80;
	char RSV = 0x00;
	char GCT = 0x02;
	char DNA = 0x00;
	char DA1 = 0x09;
	char DA2 = 0x00;
	char SNA = 0x00;
	char SA1 = 0x10;
	char SA2 = 0x00;
	char SID = 0x00;
	char handSingle[20] =
	{
		0x46, 0x49, 0x4E, 0x53, // FINS
		0x00, 0x00, 0x00, 0x0C, // 后面的命令长度
		0x00, 0x00, 0x00, 0x00, // 命令码
		0x00, 0x00, 0x00, 0x00, // 错误码
		0x00, 0x00, 0x00, 0x10  // 节点号
	};
	GNSocket m_socket;


	int get_command_length(char * header);
	int get_error_code(char* response_content);
	void get_data(char* response_content, char* data, int* data_len);
};
#endif // !OMRON_FIN_NET_H