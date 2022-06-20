#include "AuxTempAcq.h"
#include "utility.h"
#include "log4z.h"
#include "FormatConvert.h"
#include "MsgFactory.h"

#include <string.h>
#include <sys/types.h>  
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <errno.h>
#include <time.h>
#include "ChannelProto_JN9504.h"
#include "TcpServerMonitor.h"
#include "Serialize.h"


using namespace std;

AuxTempAcq g_AuxTempAcq;

TempAcqBox::TempAcqBox()
{
	feat = 0;
	dev_fd = 0;
	dev_no = 0;
	m_is_connected = false;
}

TempAcqBox::~TempAcqBox()
{

}

void TempAcqBox::set_fd(int fd)
{
	dev_fd = fd;
}

bool TempAcqBox::isConnected()
{
	return m_is_connected;
}

void TempAcqBox::disconnect()
{
	if(m_is_connected)
	{
		if(dev_fd > 0)
		{
			close(dev_fd);
		}

		m_is_connected = false;
	}
}

void TempAcqBox::configuration(int deviceNo,uint8 cnt)
{
	m_channels.clear();
	for(int i = 1; i <= cnt; i++)
	{
		m_channels.push_back(i);
	}
	dev_no = deviceNo;
}

void TempAcqBox::send_data(uint8 *src, int size)
{
	send(dev_fd,src,size,0);
}

void TempAcqBox::acquire_temperature_execute()
{
    if (!m_is_connected)  return;
		
    Channel_Data_t data = Channel_Data_t();
	data.subList_len = m_channels.size();
	for(int i = 0;i < data.subList_len; i++)
	{
		data.subList[i].func_code = m_channels[i];
	}
	
	feat++;
	Serialize data_stream(0, CMD_QUERY_TEMPERATURE, data, feat);

	send_data(data_stream.bytes_buffer(), data_stream.bytes_size());
	//LOGFMTD("TempBox%d send-->: %s",dev_no,hexstring(data_stream.bytes_buffer(), data_stream.bytes_size()).c_str());
}


AuxTempAcq::AuxTempAcq()
{
	m_dwLastTick = 0;

	for(int i = 0;i < MAX_CELLS_NBR;i++)
	{
		for(int j = 0;j < MAX_CHANNELS_NBR;j++)
		{
			m_channel_temp[i][j] = -99.8;
		}

		for(int j = 0;j < AUX_TEMP_ACQ_CH_NUM;j++)
		{
			m_cell_temp[i][j] = -99.8;
		}
	}

	memset(m_failed_count,0,sizeof(m_failed_count));
	memset(ch_acq_err_cnt,0,sizeof(ch_acq_err_cnt));
	memset(cell_acq_err_cnt,0,sizeof(cell_acq_err_cnt));
}

AuxTempAcq::~AuxTempAcq()
{
#if 1
	pthread_rwlock_destroy(&entire_rwlock);
#else
	pthread_rwlock_destroy(&ch_rwlock);
	pthread_rwlock_destroy(&cell_rwlock);
#endif
}

void AuxTempAcq::init()
{
	m_epoll_tempAcq.create(AUX_TEMP_ACQ_NUM);

#if 1
	pthread_rwlock_init(&entire_rwlock,NULL);
#else	
	pthread_rwlock_init(&ch_rwlock,NULL);
	pthread_rwlock_init(&cell_rwlock,NULL);
#endif

	for(int i = 0;i < AUX_TEMP_ACQ_NUM;i++)
	{
		m_tempAcq_box[i].configuration(i+1,AUX_TEMP_ACQ_CH_NUM);
	}
}

void AuxTempAcq::tempAcqBox_state_update()
{
	int ch_fd = -1;
	int state;
	
	for(int i = 1; i <= AUX_TEMP_ACQ_NUM; i++)
	{
		ch_fd = g_TcpServer.get_tempAcqBox_connfd(i);
		if(ch_fd <= 0)
		{
			continue;
		}
		
		state = g_TcpServer.get_tempAcqBox_state(i);

		switch(state)
		{
			case NewConnect:
				m_epoll_tempAcq.addfd(ch_fd,EPOLLIN);
				g_TcpServer.set_tempAcqBox_state(i,Connected);

				m_tempAcq_box[i-1].m_is_connected = true;
				m_tempAcq_box[i-1].set_fd(ch_fd);
				
				break;

			case DisConnect:
				m_epoll_tempAcq.deletefd(ch_fd);
				g_TcpServer.set_tempAcqBox_state(i,NoConnect);

				m_tempAcq_box[i-1].m_is_connected = false;
				m_tempAcq_box[i-1].set_fd(-1);	
				
				break;

			default:
				break;
		}		
	}
}

thread AuxTempAcq::run()
{
	return thread(&AuxTempAcq::work, this);
}

void AuxTempAcq::work()
{	
	m_dwLastTick = millitimestamp();
	
	while (1)
    {
		msleep(45);
		tempAcqBox_state_update();

		if((millitimestamp() - m_dwLastTick) > 450)
		{
			for(int i = 0;i < AUX_TEMP_ACQ_NUM;i++)
			{
				m_tempAcq_box[i].acquire_temperature_execute();
				work_logic_handle();

				if(m_failed_count[i]++ > 4)
				{
					m_failed_count[i] = 0;

				#if 1
					writeLock(&entire_rwlock);
					for(int j = 0;j < CHAN_TEMP_ACQ_CHANNELS;j++)
					{
						m_channel_temp[i/3][(i%3)*CHAN_TEMP_ACQ_CHANNELS + j] = -99.8;
					}

					for(int j = 0;j < CELL_TEMP_ACQ_CHANNELS ;j++)
					{
						m_cell_temp[i/3][(i%3)*CELL_TEMP_ACQ_CHANNELS + j] = -99.8;
					}
					rwUnLock(&entire_rwlock);

				#else
					if(i<AUX_TEMP_CHANNEL_NUM)
					{
						writeLock(&ch_rwlock);
						for(int j = 0;j < AUX_TEMP_ACQ_CH_NUM;j++)
						{
							m_channel_temp[i/2][(i%2)*16 + j] = -99.8;
						}
						rwUnLock(&ch_rwlock);
					}
					else
					{
						writeLock(&cell_rwlock);
						for(int j = 0;j < AUX_TEMP_ACQ_CH_NUM;j++)
						{
							m_cell_temp[i%8][j] = -99.8;
						}
						rwUnLock(&cell_rwlock);
					}
				#endif
				}
			}

			m_dwLastTick = millitimestamp();
		}
	}
}

int AuxTempAcq::handle_a_frame(int *ack_size)
{
	return recvReply.recvAckUnPack(m_buffer,0,0,ack_size,m_state);
}

void AuxTempAcq::recv_handle(uint8 *p_buf, int size)
{
    m_buffer.push(p_buf, size);

	int handle_size = 0;
	
    while (m_buffer.valid_size() > 9)	
	{
		if(handle_a_frame(&handle_size) > 0)
		{
			m_buffer.remove(handle_size);
		}
		else
		{
			m_buffer.remove(size);
		}
    }
}

void AuxTempAcq::work_logic_handle()
{
   	int fd = 0;
	unsigned int ch_Event = 0;
	int nfds = 0;

	nfds = m_epoll_tempAcq.waitEvents(25);
	
	for (int i = 0; i < nfds; i++)
	{
		m_epoll_tempAcq.getEvents(fd,ch_Event);
		if (ch_Event & EPOLLIN)
		{	
			tempAcq_cell_channel_No_t mTempAcqNo;
			bool found = g_TcpServer.get_tempAcqNo_by_fd(fd,mTempAcqNo);

			uint8 tmp[1024];
			int tmpLen;
			tmpLen = recv(fd,tmp,1024,0);
			//printf("tmpLen=%d ch_Event=%x\n",tmpLen,ch_Event);

			if(tmpLen>0 && found)
			{	
				m_state.clear_temperature_data();
				recv_handle(tmp,tmpLen);
				
				vector<Acq_Temperature_Data_t> newData;
				m_state.get_temperature_data(newData);

				m_failed_count[mTempAcqNo.device-1] = 0;

#if 1
				writeLock(&entire_rwlock);
				for(uint32 i = 0;i < newData.size(); i++)
				{
					uint8 chan = 0;
					uint8 idx = newData[i].chan;

					if(idx <= CHAN_TEMP_ACQ_CHANNELS)
					{
						chan = mTempAcqNo.channelTempNo[idx-1];
						if(chan != 0)
						{
							if(newData[i].temp > -50.0 || ch_acq_err_cnt[mTempAcqNo.cellNo-1][chan-1]++ > 3)
							{
								m_channel_temp[mTempAcqNo.cellNo-1][chan-1] = newData[i].temp;
								ch_acq_err_cnt[mTempAcqNo.cellNo-1][chan-1] = 0;
							}
						}
					}
					else
					{
						chan = mTempAcqNo.cellTempNo[idx-CHAN_TEMP_ACQ_CHANNELS-1];
						if(chan != 0)
						{
							if(newData[i].temp > -50.0 || cell_acq_err_cnt[mTempAcqNo.cellNo-1][chan-1]++ > 3)
							{
								m_cell_temp[mTempAcqNo.cellNo-1][chan-1] = newData[i].temp;
								cell_acq_err_cnt[mTempAcqNo.cellNo-1][chan-1] = 0;
							}
						}
					}
				}
				rwUnLock(&entire_rwlock);
#else
					
				if(mTempAcqNo.type == CHANNEL_ACQ_T)	//通道
				{
					writeLock(&ch_rwlock);
					for(uint32 i = 0;i < newData.size(); i++)
					{
						uint8 idx = newData[i].chan;
						uint8 chan = mTempAcqNo.chanNo[idx-1];
						m_channel_temp[mTempAcqNo.cellNo-1][chan-1] = newData[i].temp;
					}
					rwUnLock(&ch_rwlock);
				}
				else	//库位
				{
					writeLock(&cell_rwlock);
					for(uint32 i = 0;i < newData.size(); i++)
					{
						uint8 idx = newData[i].chan;
						uint8 chan = mTempAcqNo.chanNo[idx-1];
						m_cell_temp[mTempAcqNo.cellNo-1][chan-1] = newData[i].temp;
					}
					rwUnLock(&cell_rwlock);
				}
#endif
				//LOGFMTD("TempBox%d recv<--: %s", mTempAcqNo.device, hexstring(tmp, tmpLen).c_str());								
			}

			//0:客户端主动断开  //-1: 异常断开 (重插网线、下位机重启)
			if((tmpLen == 0) || ((tmpLen < 0) && (errno != EINTR) && (errno != EWOULDBLOCK) && (errno != EAGAIN) && (errno != 104)))
			{
				m_epoll_tempAcq.deletefd(fd);
				g_TcpServer.delete_fd_from_tempAcqmap(fd);
				close(fd);
				
				if(found)
				{
					g_TcpServer.set_tempAcqBox_state(mTempAcqNo.device,DisConnect);
				}	
			}			
		}
		else if((ch_Event & EPOLLHUP) || (ch_Event & EPOLLERR))  
		{
			m_epoll_tempAcq.deletefd(fd);
			g_TcpServer.delete_fd_from_tempAcqmap(fd);
			close(fd);
		}
	}
}


void AuxTempAcq::readLock(pthread_rwlock_t *rwlock)
{
	pthread_rwlock_rdlock(rwlock);
}

void AuxTempAcq::writeLock(pthread_rwlock_t *rwlock)
{
	pthread_rwlock_wrlock(rwlock);
}

void AuxTempAcq::rwUnLock(pthread_rwlock_t *rwlock)
{
	pthread_rwlock_unlock(rwlock);
}

void AuxTempAcq::get_channel_temperature(int cellNo,channel_temperature_t &front,channel_temperature_t &mid,channel_temperature_t &back)
{
	readLock(&entire_rwlock);
	memcpy((void*)&front,(void*)&m_channel_temp[cellNo-1][0],sizeof(float)*CHAN_TEMP_ACQ_CHANNELS);
	memcpy((void*)&mid,(void*)&m_channel_temp[cellNo-1][12],sizeof(float)*CHAN_TEMP_ACQ_CHANNELS);
	memcpy((void*)&back,(void*)&m_channel_temp[cellNo-1][24],sizeof(float)*CHAN_TEMP_ACQ_CHANNELS);
	rwUnLock(&entire_rwlock);
}

void AuxTempAcq::get_cell_temperature(int cellNo,channel_temperature_t &data)
{
	readLock(&entire_rwlock);
	memcpy((void*)&data,(void*)&m_cell_temp[cellNo-1][0],sizeof(float)*AUX_TEMP_ACQ_CH_NUM);
	rwUnLock(&entire_rwlock);
}


