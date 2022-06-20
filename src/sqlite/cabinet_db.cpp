#include "cabinet_db.h"
#include "sqlite3.h"
#include "log4z.h"
#include "Configuration.h"
#include "FormatConvert.h"
#include "MsgFactory.h"

#include <iostream>
#include <sstream>

using namespace std;


#ifdef ZHONGYUAN_DB

static int    g_selected_num = 0;
static string g_selected_str;

/*
  第一个参数通过sqlite3_exec的第第四个参数传入的
  第二个参数是结果行的列数
  第三个参数是行中列数据的指针
  第四个参数是行中列名称的指针
 */
static int callback(void *NotUsed, int argc, char **argv, char **azColName)
{
	if (argc > 0)
	{
		ostringstream os;
		os << "{";
		for (int i=0; i<argc; i++)
		{
			if ((strcmp(azColName[i], "id") == 0) ||
				(strcmp(azColName[i], "BatteryType") == 0) ||
				(strcmp(azColName[i], "StepConfNo") == 0) ||
				(strcmp(azColName[i], "PalletCode") == 0) ||
				(strcmp(azColName[i], "BATTERY_NUMBER") == 0))
			{
				os << "\"" << string(azColName[i]) << "\":\"" << string(argv[i]) << "\",";
			}
			else
			{
				os << "\"" << string(azColName[i]) << "\":" << string(argv[i]) << ",";
			}
		}
		os << "}";
		
		g_selected_str = os.str();
	}
	else
	{
		g_selected_num = 0;
	}
	
	return 0;
}

CabinetDateBase::CabinetDateBase()
{
	m_db = NULL;
	m_line_no = Configuration::instance()->line_no();
	m_cabinet_no = Configuration::instance()->cabinet_no();
}

CabinetDateBase::~CabinetDateBase()
{
	if (m_db != NULL)
	{
		sqlite3_close(m_db);
	}
}

thread CabinetDateBase::run()
{
    return thread(&CabinetDateBase::work, this);
}

void CabinetDateBase::work()
{
	char *zErrMsg = 0;
	int  rc;
	
	// 建立线程间通讯
	for (int cell=0; cell<MAX_CELLS_NBR; cell++)
	{
		MsgFactory::instance()->create_data_to_int_store_puller(cell+1, Configuration::instance()->int_data_store_socket(cell+1));
	}

	// 打开本地数据库
	rc = sqlite3_open("lbfs.db", &m_db);
	if (rc)
	{
		LOGFMTE("Can't open database: %s", sqlite3_errmsg(m_db));
		exit(0);
	}
	else
	{
		LOGFMTD("Opened database successfully");

		// 建表
		const char *table_1 = "CREATE TABLE IF NOT EXISTS process(" \
			"id CHAR(128) PRIMARY KEY NOT NULL," \
			"BatteryType CHAR(128)," \
			"StepConfNo CHAR(128)," \
			"PalletCode CHAR(128)," \
           	"batteryCode CHAR(128)," \
           	"LineNo INT," \
           	"CabNo INT," \
           	"CellNo INT," \
           	"ChannelNo INT," \
           	"RunState INT," \
           	"RunTime BIGINT," \
           	"Voltage FLOAT," \
           	"Current FLOAT," \
           	"Capacity FLOAT," \
           	"BatteryTemperature FLOAT," \
           	"StepNo INT," \
           	"StepType INT," \
           	"LoopNo INT," \
           	"IsStop BOOLEAN" \
           	"CurrentTime CHAR(64)," \
           	"SumStep INT," \
           	"Ratio FLOAT," \
           	"Energy FLOAT," \
           	"Povl FLOAT," \
           	"CotactRes FLOAT," \
           	"LoopTimes INT," \
           	"ErrorCode INT," \
           	"Vaccum FLOAT" \
			")";
		const char *table_2 = "CREATE TABLE IF NOT EXISTS stop(" \
			"id CHAR(128) PRIMARY KEY NOT NULL," \
			"BatteryType CHAR(128)," \
			"StepConfNo CHAR(128)," \
			"PalletCode CHAR(128)," \
           	"batteryCode CHAR(128)," \
           	"LineNo INT," \
           	"CabNo INT," \
           	"CellNo INT," \
           	"ChannelNo INT," \
           	"RunState INT," \
           	"RunTime BIGINT," \
           	"Voltage FLOAT," \
           	"Current FLOAT," \
           	"Capacity FLOAT," \
           	"BatteryTemperature FLOAT," \
           	"StepNo INT," \
           	"StepType INT," \
           	"LoopNo INT," \
           	"IsStop BOOLEAN" \
           	"CurrentTime CHAR(64)," \
           	"SumStep INT," \
           	"Ratio FLOAT," \
           	"Energy FLOAT," \
           	"Povl FLOAT," \
           	"CotactRes FLOAT," \
           	"LoopTimes INT," \
           	"ErrorCode INT," \
           	"Vaccum FLOAT" \
			")";
		rc = sqlite3_exec(m_db, table_1, NULL, NULL, &zErrMsg);
		if (rc != SQLITE_OK)
		{
			LOGFMTE("SQL error: %s", zErrMsg);
			sqlite3_free(zErrMsg);
		}
		rc = sqlite3_exec(m_db, table_2, NULL, NULL, &zErrMsg);
		if (rc != SQLITE_OK)
		{
			LOGFMTE("SQL error: %s", zErrMsg);
			sqlite3_free(zErrMsg);
		}
	}

	// 业务循环
	while (1)
	{
		msleep(10);
		
		try
		{
			// 接收库位消息
			for (int cell=0; cell<MAX_CELLS_NBR; cell++)
			{
				Step_Process_Data_t data;
				while (MsgFactory::instance()->data_to_int_store_puller(cell+1).recevie(data, ZMQ_DONTWAIT))
				{
					// 列表缓存
					m_data_list.push_back(data);
					// 发送给服务器
					string store_josn_str = FormatConvert::instance().process_data_to_store_str(data);
				}
			}

			// 接收服务器返回结果
			string ret_id;
			bool   is_stop;
			// 遍历数据列表
			for (std::list<Step_Process_Data_t>::iterator it = m_data_list.begin(); it != m_data_list.end(); it++)
			{
				// 计算id
				ostringstream os;
				os << m_line_no << "-" << m_cabinet_no << "-" << it->cell_no << "-" << it->ch_no << "-"
				   << string(it->bat_barcode) << "-" << it->timestamp;
				string id = os.str();

				// id相同，则从列表中删除
				if ((id == ret_id) && (is_stop == it->is_stop))
				{
					m_data_list.erase(it);
					break;
				}
			}
			
			// 脱机数据入本地数据库（数据记录时间超时>5秒）	
			for (std::list<Step_Process_Data_t>::iterator it = m_data_list.begin(); it != m_data_list.end(); it++)
			{
				time_t nt = time(NULL);
				if ((nt - it->timestamp) > 5)
				{
					string sql = FormatConvert::instance().process_data_to_sql_str(*it, it->is_stop ? "stop" : "process");
					rc = sqlite3_exec(m_db, sql.c_str(), NULL, NULL, &zErrMsg);
					if (rc != SQLITE_OK)
					{
						LOGFMTE("SQL error: %s", zErrMsg);
						sqlite3_free(zErrMsg);
					}
					m_data_list.erase(it);
				}
			}
			
			// 离线数据入库（网络恢复后执行）
			offline_data_handle();
		}
		catch (int &e)
		{
		}	
	}

	for (int cell=0; cell<MAX_CELLS_NBR; cell++)
	{
		MsgFactory::instance()->destory_data_to_int_store_puller(cell+1);
	}
}

void CabinetDateBase::offline_data_handle()
{
	char *zErrMsg = 0;
	int  rc;

	// 查询过程数据表
	while (1)
	{
		const char *sql = "SELECT * FROM process LIMIT 1";
		rc = sqlite3_exec(m_db, sql, &callback, NULL, &zErrMsg);
		if (rc != SQLITE_OK)
		{
			LOGFMTE("SQL error: %s", zErrMsg);
			sqlite3_free(zErrMsg);
			break;
		}
		else
		{
			if (g_selected_num > 0)
			{
				// 发送给服务器（用另一个网络端口，与正常数据入库分开）
				// g_selected_str
				// 等待应答
				string ret;
				string del = "DELETE FROM process WHERE ID=" + ret;
				rc = sqlite3_exec(m_db, del.c_str(), NULL, NULL, &zErrMsg);
				if (rc != SQLITE_OK)
				{
					LOGFMTE("SQL error: %s", zErrMsg);
					sqlite3_free(zErrMsg);
					break;
				}
			}
			else
			{
				break;
			}
		}
	}	
		
	// 截止数据表
	while (1)
	{
		const char *sql = "SELECT * FROM stop LIMIT 1";
		rc = sqlite3_exec(m_db, sql, &callback, NULL, &zErrMsg);
		if (rc != SQLITE_OK)
		{
			LOGFMTE("SQL error: %s", zErrMsg);
			sqlite3_free(zErrMsg);
			break;
		}
		else
		{
			if (g_selected_num > 0)
			{
				// 发送给服务器（用另一个网络端口，与正常数据入库分开）
				// g_selected_str
				// 等待应答
				string ret;
				string del = "DELETE FROM stop WHERE ID=" + ret;
				rc = sqlite3_exec(m_db, del.c_str(), NULL, NULL, &zErrMsg);
				if (rc != SQLITE_OK)
				{
					LOGFMTE("SQL error: %s", zErrMsg);
					sqlite3_free(zErrMsg);
					break;
				}
			}
			else
			{
				break;
			}
		}
	}	
}
#else

CabinetDateBase g_CabDBServer;

CabinetDateBase::CabinetDateBase()
{
	m_line_no = Configuration::instance()->line_no();
	m_cabinet_no = Configuration::instance()->cabinet_no();
	
	for(int i=0;i<MAX_CELLS_NBR;i++)
	{
		pthread_mutex_init(&mutex[i], NULL);
	}
}

CabinetDateBase::~CabinetDateBase()
{
	for(int i=0;i<MAX_CELLS_NBR;i++)
	{
		pthread_mutex_destroy(&mutex[i]);
	}
}

void CabinetDateBase::pushbackRecord(int cellNo,Step_Process_Data_t &data)
{
	pthread_mutex_lock(&mutex[cellNo-1]);
	m_data_deque[cellNo-1].push_back(data);
	pthread_mutex_unlock(&mutex[cellNo-1]);
}

bool CabinetDateBase::popfrontRecord(int cellNo,Step_Process_Data_t &data)
{
	if(m_data_deque[cellNo-1].size() > 0)
	{
		pthread_mutex_lock(&mutex[cellNo-1]);
		data = m_data_deque[cellNo-1].front();	
		m_data_deque[cellNo-1].pop_front();
		pthread_mutex_unlock(&mutex[cellNo-1]);
		return true;
	}
	else
	{
		return false;
	}
}

int CabinetDateBase::get_data_deque_size(int cellNo)
{
	return m_data_deque[cellNo-1].size();
}

thread CabinetDateBase::run()
{
    return thread(&CabinetDateBase::work, this);
}

void CabinetDateBase::work()
{	
	// 建立d2k通讯
	MsgFactory::instance()->create_data_to_ext_store_pusher(Configuration::instance()->ext_data_store_socket());

	// 业务循环
	while (1)
	{
		msleep(10);
		
		try
		{
			// 接收库位消息
			for (int cell=0; cell<MAX_CELLS_NBR; cell++)
			{
				Step_Process_Data_t data;
				
				while(get_data_deque_size(cell+1)>0)
				{
					if(popfrontRecord(cell+1,data))
					{
						// 发送给服务器
						string store_josn_str = FormatConvert::instance().process_data_to_store_str_new(data);
						//LOGFMTD("d2k: %s", store_josn_str.c_str());
						MsgFactory::instance()->data_to_ext_store_pusher().send(store_josn_str);
					}				
				}
			}

			// 脱机数据本地保存
			// TODO
		}
		catch (int &e)
		{
			LOGFMTE("data ext store failed! %d", e);
		}	
	}

	MsgFactory::instance()->destory_data_to_ext_store_pusher();
}

#endif

