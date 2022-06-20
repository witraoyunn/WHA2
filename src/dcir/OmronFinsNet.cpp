#include "OmronFinsNet.h"
#include "GNString.h"
#include <vector>
#include <sstream>
#include <string.h>

using namespace std;
#define W_BIT_COMMAND  {0x31, 0xB1}     // W区  位操作符
#define W_WORD_COMMAND 0xB1     // W区  字操作符
#define COMMAND_LENGTH 0x1A   // 指令长度
#define HEADER_LENGTH  8      // 指令头长度


union int_byte_union
{
	int len;
	char bytes[4];
};

OmronFinsNet::OmronFinsNet()
{
	m_socket = GNSocket();
}

OmronFinsNet::~OmronFinsNet()
{
}

void OmronFinsNet::connect_server(std::string ip, int port)
{
	if (&m_socket != NULL)
	{
		m_socket.connect_server(ip.c_str(), port);
		//m_socket.set_recv_timeout(5000);
	}
}


void OmronFinsNet::initialize()
{
	char response[50] = { 0 };
	int response_len;
	try
	{
		read_data(handSingle, 20, response, &response_len);
		DA1 = *(response + 0xF);
		handSingle[19] = DA1;
	}
	catch (const std::exception&)
	{

	}
}


void OmronFinsNet::build_read_command(std::string addr, int length, char* command, int* command_len, bool isBit)
{
	char cmd[8] = { 0 };
	cmd[0] = 0x01;
	cmd[1] = 0x01;

	char singn[2] = { 0 };
	if (addr[0] == 'D' || addr[0] == 'd')
	{
		singn[0] = 0x02;
		singn[1] = 0x82;
	}
	else if (addr[0] == 'W' || addr[0] == 'w')
	{
		singn[0] = 0x31;
		singn[1] = 0xB1;
	}

	if (isBit)
	{
		cmd[2] = singn[0];
		vector<string> tagNames = split(addr.substr(1), "."); // 去掉W 标识符
		int content = parse_int(tagNames[0]);
		cmd[3] = (content & 0xFF00) >> 8;
		cmd[4] = (content & 0x0FF);
		if (tagNames.size() > 1)
		{
			cmd[5] = parse_char(tagNames[1]);
		}
	}
	else
	{
		cmd[2] = singn[1];
		int content = parse_int(addr.substr(1));
		cmd[3] = (content & 0xFF00) >> 8;
		cmd[4] = (content & 0x0FF);
	}
	cmd[6] = (length / 256);                       // 长度
	cmd[7] = (length % 256);
	memcpy(command, cmd, 8);
	*command_len = 8;
}

void OmronFinsNet::pack_command(char* cmd, int cmd_len, char* data_pack, int* data_pack_len)
{

	char buffer[50] = { 0 };
	memcpy(buffer, handSingle, 4);
	*data_pack_len = COMMAND_LENGTH + cmd_len;
	int tem = *data_pack_len - 8;
	buffer[4] = (tem & 0xFF00000) >> 24;
	buffer[5] = (tem & 0x00FF0000) >> 16;
	buffer[6] = (tem & 0x0000FF00) >> 8;
	buffer[7] = (tem & 0x000000FF);

	buffer[11] = 0x02;

	buffer[16] = ICF;
	buffer[17] = RSV;
	buffer[18] = GCT;
	buffer[19] = DNA;
	buffer[20] = DA1;
	buffer[21] = DA2;
	buffer[22] = SNA;
	buffer[23] = SA1;
	buffer[24] = SA2;
	buffer[25] = SID;

	memcpy(data_pack, buffer, 26);
	memcpy(data_pack + 26, cmd, cmd_len);
}
void OmronFinsNet::build_write_command(std::string addr, char* value, int value_len, char* command, int* command_len, bool isBit)
{
	char cmd[30] = { 0 };
	cmd[0] = 0x01;
	cmd[1] = 0x02;

	char singn[2] = { 0 };
	if (addr[0] == 'D' || addr[0] == 'd')
	{
		singn[0] = 0x02;
		singn[1] = 0x82;
	}
	else if (addr[0] == 'W' || addr[0] == 'w')
	{
		singn[0] = 0x31;
		singn[1] = 0xB1;
	}

	if (isBit)
	{
		cmd[2] = singn[0];
		vector<string> tagNames = split(addr.substr(1), "."); // 去掉W 标识符
		int content = parse_int(tagNames[0]);
		cmd[3] = (content & 0xFF00) >> 8;
		cmd[4] = (content & 0x0FF);
		if (tagNames.size() > 1)
		{
			cmd[5] = parse_char(tagNames[1]);
		}
		cmd[6] = value_len / 256;
		cmd[7] = value_len % 256;
	}
	else
	{
		cmd[2] = singn[1];
		int content = parse_int(addr.substr(1));
		cmd[3] = (content & 0xFF00) >> 8;
		cmd[4] = (content & 0x0FF);
		cmd[6] = value_len / 2 / 256;
		cmd[7] = value_len / 2 % 256;
	}

	memcpy(command, cmd, 8);
	memcpy(command + 8, value, value_len);
	*command_len = 8 + value_len;
}


void OmronFinsNet::read_from_server(int length, char* data)
{
	m_socket.recv_msg(data, length);
}

void OmronFinsNet::read_header_from_server(char* data)
{
	char v_data[8] = { 0 };
	read_from_server(HEADER_LENGTH, v_data);
	memcpy(data, v_data, 8);
}

void OmronFinsNet::reader_command_content_from_server(int length, char* data)
{
	m_socket.recv_msg(data, length);
}


bool OmronFinsNet::read_bool(std::string addr)
{
	char cmd[100] = { 0 };
	int cmd_len;
	build_read_command(addr, 1, cmd, &cmd_len, true);

	char pack[100] = { '\0' };

	int pack_len;
	pack_command(cmd, cmd_len, pack, &pack_len);

	char data[30] = { 0 };
	int data_len = 0;
	read_data(pack, pack_len, data, &data_len);

	// 22 位位数据位
	return 1 == data[22];
}

void OmronFinsNet::read_data(char* pack, int pack_len, char* data, int* data_len)
{
	try
	{
		m_socket.send_msg(pack, pack_len); // 发请求

		char header[8] = { 0 };
		read_header_from_server(header); // 读响应头

		int cmd_len = get_command_length(header);
		char content_tem[30] = { 0 };
		reader_command_content_from_server(cmd_len, content_tem); // 读响应内容

		int err_code = get_error_code(content_tem); // 解析错误码
		if (err_code > 0)
		{
			throw err_code;
		}

		// 解析数据块
		memcpy(data, content_tem, cmd_len);
		*data_len = cmd_len;
	}
	catch (int ec)
	{
		throw ec;
	}
}


void OmronFinsNet::write_bool(std::string addr, bool value)
{
	char cmd[100] = { 0 };
	int cmd_len;

	char v_val = (char)(value ? 1 : 0);
	build_write_command(addr, &v_val, 1, cmd, &cmd_len, true);

	char pack[100] = { '\0' };

	int pack_len;
	pack_command(cmd, cmd_len, pack, &pack_len);

	char data[30] = { 0 };
	int data_len = 0;
	read_data(pack, pack_len, data, &data_len);
}


// 4-7位是命令长度
int OmronFinsNet::get_command_length(char* header)
{
	int_byte_union len;
	len.bytes[3] = *(header + 4);
	len.bytes[2] = *(header + 5);
	len.bytes[1] = *(header + 6);
	len.bytes[0] = *(header + 7);

	int len_tem = len.len;
	return len_tem;
}

// 4-7位是错误码
int OmronFinsNet::get_error_code(char* response_content)
{
	int_byte_union len;
	len.bytes[3] = *(response_content + 4);
	len.bytes[2] = *(response_content + 5);
	len.bytes[1] = *(response_content + 6);
	len.bytes[0] = *(response_content + 7);

	int len_tem = len.len;
	return len_tem;
}

void  OmronFinsNet::get_data(char* response_content, char* data, int* data_len)
{
	int v_data_len = strlen(response_content) - 14;
	*data_len = v_data_len;

	memcpy(data, response_content, v_data_len);
}

int OmronFinsNet::read_int(std::string addr)
{
	char cmd[100] = { 0 };
	int cmd_len;
	build_read_command(addr, 2, cmd, &cmd_len, false);

	char pack[100] = { '\0' };

	int pack_len;
	pack_command(cmd, cmd_len, pack, &pack_len);

	char data[30] = { 0 };
	int data_len = 0;
	read_data(pack, pack_len, data, &data_len);

	// 22-25 位位数据位

	int_byte_union tem;
	tem.bytes[0] = data[23];
	tem.bytes[1] = data[24];
	tem.bytes[2] = data[25];
	tem.bytes[3] = data[26];

	return tem.len;
}
