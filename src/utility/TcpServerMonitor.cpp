#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include "log4z.h"
#include "Type.h"
#include "TcpServerMonitor.h"
#include "Configuration.h"

using namespace std;

TcpServerMonitor g_TcpServer;

TcpServerMonitor::TcpServerMonitor()
{
	sock_fd = -1;
	for(int i = 0; i < MAX_CELLS_NBR; i++)
	{
		for(int j = 0; j < MAX_CHANNELS_NBR; j++)
		{
			client_info_table[i][j].state = 0;
			client_info_table[i][j].ch_fd = 0;		
		}
	}

	for(int i = 0; i<AUX_TEMP_ACQ_NUM; i++)
	{
		tempAcq_info_table[i].state = 0;
		tempAcq_info_table[i].ch_fd = 0;
	}
}

TcpServerMonitor::~TcpServerMonitor()
{
#ifndef AWH_FORMATION_SYS
	for(int i = 0; i < MAX_CELLS_NBR; i++)
	{
		for(int j = 0; j < MAX_CHANNELS_NBR; j++)
		{
			if(client_info_table[i][j].ch_fd > 0)
			{
				close(client_info_table[i][j].ch_fd);
			}
		}
	}
#endif	

	for(int i = 0; i<AUX_TEMP_ACQ_NUM; i++)
	{
		if(tempAcq_info_table[i].ch_fd > 0)
		{
			close(tempAcq_info_table[i].ch_fd);
		}		
	}
		
	if(sock_fd > 0)
	{
		close(sock_fd);
	}
	pthread_mutex_destroy(&channelMutex);
	pthread_mutex_destroy(&tempAcqMutex);
	for(int i=0;i<MAX_CELLS_NBR;i++)
	{
		pthread_rwlock_destroy(&rwlock[i]);
	}
}


int TcpServerMonitor::init()
{
	int optval = 1;
	int	ret = -1;

	string ip_addr="";
	string server_ip="";
	unsigned short port;
	struct sockaddr_in serv_addr;
	cell_channel_No_t cellchanNo;
	tempAcq_cell_channel_No_t tempAcqNo;

	pthread_mutex_init(&channelMutex, NULL);
	pthread_mutex_init(&tempAcqMutex, NULL);
	for(int i=0;i<MAX_CELLS_NBR;i++)
	{
		pthread_rwlock_init(&rwlock[i],NULL);
	}

	port = Configuration::instance()->channel_server_socket(server_ip);
	printf("server ip:%s:%d \n",server_ip.c_str(),port);

	for (int i = 1; i <= MAX_CELLS_NBR; i++)
	{
		for (int j = 1; j <= (MAX_CHANNELS_NBR+MODULE_CH_NUM-1)/MODULE_CH_NUM; j++)
		{
			vector<int> channels;

			memset((void*)&cellchanNo,0,sizeof(cell_channel_No_t));
#if defined(AWH_FORMATION_SYS)											//武汉化成使用CAN通信	
			Configuration::instance()->device_socket(i,j,channels);
#else
			Configuration::instance()->device_socket(i,j,ip_addr,channels);
			if(ip_addr != "")
#endif
			{
				cellchanNo.cellNo = i;
				cellchanNo.device = j;
				for(uint32 k = 0; k < channels.size(); k++)
				{
					cellchanNo.chanNo[k] = channels[k];
				}
#if defined(AWH_FORMATION_SYS)
				cell_device_chanNo.insert(std::make_pair((i<<16)+j,cellchanNo));
#else
				map_ip_chanNo.insert(std::make_pair(ip_addr,cellchanNo));
#endif
			}
		}
	}

	for (int i = 1; i <= MAX_CELLS_NBR*3; i++)
	{
		int cellNo = 0;
		vector<int> cellTempNo;
		vector<int> channelTempNo;
		
		Configuration::instance()->temperature_acqBox_socket(i,ip_addr,cellNo,channelTempNo,cellTempNo);
		if(ip_addr != "")
		{
			for(uint32 j = 0; j < channelTempNo.size(); j++)
			{
				tempAcqNo.channelTempNo[j] = channelTempNo[j];
			}
			for(uint32 j = 0; j < cellTempNo.size(); j++)
			{
				tempAcqNo.cellTempNo[j] = cellTempNo[j];
			}
			
			tempAcqNo.device = i;
			tempAcqNo.cellNo = cellNo;
			tempAcq_map_ip_chanNo.insert(std::make_pair(ip_addr,tempAcqNo));
		}
	}

/*
	for(auto element = map_ip_chanNo.cbegin();element != map_ip_chanNo.cend();++element)
	{
		printf("%s -> cell%d channel%d\n", element->first.c_str(), element->second.cellNo, element->second.chanNo); 
	}
*/
	
	// 创建一个TCP套接字
	sock_fd = socket(AF_INET, SOCK_STREAM,0);
	if (sock_fd < 0) 
	{
		return sock_fd;
	}

	int flag = fcntl(sock_fd, F_GETFL, 0);
	fcntl(sock_fd, F_SETFL, flag | O_NONBLOCK);
	
	// 设置该套接字使之可以重新绑定端口
	ret = setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR,(void *)&optval, sizeof(optval));
	if (ret < 0) 
	{
		return ret;
	}
	
	// 初始化服务器端地址结构
	memset(&serv_addr, 0, sizeof (struct sockaddr_in));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(port);
	//serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	ret = inet_aton(server_ip.c_str(),&serv_addr.sin_addr);
	if(ret == 0)
	{
		return -1;
	}
	
	// 将套接字绑定到本地端口
	ret = bind(sock_fd, (struct sockaddr *)&serv_addr,sizeof (struct sockaddr_in));
	if (ret < 0) 
	{
		return ret;
	}

	return 0;
}

void TcpServerMonitor::readLock(int cell_no)
{
	pthread_rwlock_rdlock(&rwlock[cell_no-1]);
}

void TcpServerMonitor::writeLock(int cell_no)
{
	pthread_rwlock_wrlock(&rwlock[cell_no-1]);
}

void TcpServerMonitor::rwUnLock(int cell_no)
{
	pthread_rwlock_unlock(&rwlock[cell_no-1]);
}

void TcpServerMonitor::OSSemPend(pthread_mutex_t *mutex)
{
	pthread_mutex_lock(mutex);
}

void TcpServerMonitor::OSSemPost(pthread_mutex_t *mutex)
{
	pthread_mutex_unlock(mutex);
}

thread TcpServerMonitor::run()
{
    return thread(&TcpServerMonitor::accept_monitor, this);
}

void TcpServerMonitor::accept_monitor()
{
	int conn_fd;
	struct sockaddr_in cli_addr;
	socklen_t cli_len;

	// 将套接字转化为监听套接字
	if (listen(sock_fd, MAX_CELLS_NBR*MAX_CHANNELS_NBR) < 0) 
	{
		LOGFMTE("start listen failed!");
		return;
	}
	
	cli_len = sizeof (struct sockaddr_in);
	while(1)
	{
		msleep(15);
		// 通过accept接受客户端的连接请求，并返回连接套接字用于收发数据
		conn_fd = accept(sock_fd, (struct sockaddr *)&cli_addr, &cli_len);
		if (conn_fd < 0) 
		{
			continue;
		}
		int flag = fcntl(conn_fd, F_GETFL, 0);
		fcntl(conn_fd, F_SETFL, flag | O_NONBLOCK);

		string cli_ip = inet_ntoa(cli_addr.sin_addr);
		uint16_t cli_port = ntohs(cli_addr.sin_port);
		LOGFMTD("accept a new client, ip:%s:%d", cli_ip.c_str(),cli_port);

#ifndef AWH_FORMATION_SYS							//武汉化成使用CAN通信
		auto found = map_ip_chanNo.find(cli_ip);
		if(found != map_ip_chanNo.end())
		{
			writeLock(found->second.cellNo);
			client_info_table[found->second.cellNo-1][found->second.device-1].ch_fd = conn_fd;
			client_info_table[found->second.cellNo-1][found->second.device-1].ip = cli_ip;
			client_info_table[found->second.cellNo-1][found->second.device-1].state = NewConnect;
			rwUnLock(found->second.cellNo);

			cell_channel_No_t cellchanNo;
			memcpy(&cellchanNo,&(found->second),sizeof(cell_channel_No_t));
			OSSemPend(&channelMutex);
			map_fd_chanNo.insert(std::make_pair(conn_fd,cellchanNo));
			OSSemPost(&channelMutex);					
		}
		else
#endif			
		{
			auto acqfound = tempAcq_map_ip_chanNo.find(cli_ip);
			if(acqfound != tempAcq_map_ip_chanNo.end())
			{
				OSSemPend(&tempAcqMutex);
				tempAcq_info_table[acqfound->second.device-1].ch_fd = conn_fd;
				tempAcq_info_table[acqfound->second.device-1].ip = cli_ip;
				tempAcq_info_table[acqfound->second.device-1].state = NewConnect;
					
				tempAcq_cell_channel_No_t tempAcqNo;

				memcpy(&tempAcqNo,&(acqfound->second),sizeof(tempAcq_cell_channel_No_t));
				tempAcq_map_fd_chanNo.insert(std::make_pair(conn_fd,tempAcqNo));
				OSSemPost(&tempAcqMutex);
			}
			else
			{
				close(conn_fd);
			}
		}
	}
}

int TcpServerMonitor::get_client_state(int cell_no, int device_no)
{
	readLock(cell_no);
	int state = client_info_table[cell_no-1][device_no-1].state;
	rwUnLock(cell_no);

	return state;	
}

void TcpServerMonitor::set_client_state(int cell_no, int device_no, int state)
{
	writeLock(cell_no);
	client_info_table[cell_no-1][device_no-1].state = state;
	rwUnLock(cell_no);
}

#ifndef AWH_FORMATION_SYS
int TcpServerMonitor::get_client_connfd(int cell_no, int device_no)
{
	readLock(cell_no);
	int conn_fd = client_info_table[cell_no-1][device_no-1].ch_fd;
	rwUnLock(cell_no);
	
	return conn_fd;
}

string TcpServerMonitor::get_client_ip(int cell_no, int device_no)
{
	readLock(cell_no);
	string ip = client_info_table[cell_no-1][device_no-1].ip;
	rwUnLock(cell_no);

	return ip;	
}

bool TcpServerMonitor::get_client_channels_by_ip(string ip, vector<int> &channels)
{
	auto found = map_ip_chanNo.find(ip);
	if(found != map_ip_chanNo.end())
	{
		for(int i = 0; i < MODULE_CH_NUM; i++)
		{
			if(found->second.chanNo[i])
			{
				channels.push_back(found->second.chanNo[i]);
			}
		}

		return true;
	}

	return false;
}

bool TcpServerMonitor::get_client_channels_by_fd(int fd, int &device, vector<int> &channels)
{
	auto found = map_fd_chanNo.find(fd);
	if(found != map_fd_chanNo.end())
	{
		device = found->second.device;
		for(int i = 0; i < MODULE_CH_NUM; i++)
		{
			if(found->second.chanNo[i])
			{
				channels.push_back(found->second.chanNo[i]);
			}
		}

		return true;
	}

	return false;				
}

void TcpServerMonitor::delete_fd_from_map(int fd)
{
	OSSemPend(&channelMutex);
	map_fd_chanNo.erase(fd);
	OSSemPost(&channelMutex);
}
#else
bool TcpServerMonitor::get_client_channels(int cell_no,int device_no, vector<int> &channels)
{
	auto found = cell_device_chanNo.find((cell_no<<16)+device_no);
	if(found != cell_device_chanNo.end())
	{
		for(int i = 0; i < MODULE_CH_NUM; i++)
		{
			if(found->second.chanNo[i])
			{
				channels.push_back(found->second.chanNo[i]);
			}
		}

		return true;
	}

	return false;	
}
#endif

int TcpServerMonitor::get_tempAcqBox_connfd(int device_no)
{
	OSSemPend(&tempAcqMutex);
	int conn_fd = tempAcq_info_table[device_no-1].ch_fd;
	OSSemPost(&tempAcqMutex);
	
	return conn_fd;
}

int TcpServerMonitor::get_tempAcqBox_state(int device_no)
{
	OSSemPend(&tempAcqMutex);
	int state = tempAcq_info_table[device_no-1].state;
	OSSemPost(&tempAcqMutex);

	return state;	
}

void TcpServerMonitor::set_tempAcqBox_state(int device_no, int state)
{
	OSSemPend(&tempAcqMutex);
	tempAcq_info_table[device_no-1].state = state;
	OSSemPost(&tempAcqMutex);
}

bool TcpServerMonitor::get_tempAcqNo_by_fd(int fd, tempAcq_cell_channel_No_t &tempAcqNo)
{
	auto found = tempAcq_map_fd_chanNo.find(fd);
	if(found != tempAcq_map_fd_chanNo.end())
	{
		memcpy(&tempAcqNo,&(found->second),sizeof(tempAcq_cell_channel_No_t));

		return true;
	}

	return false;
}

void TcpServerMonitor::delete_fd_from_tempAcqmap(int fd)
{
	OSSemPend(&tempAcqMutex);
	tempAcq_map_fd_chanNo.erase(fd);
	OSSemPost(&tempAcqMutex);
}



