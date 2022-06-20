#include "log4z.h"
#include "Scanner.h"
#include "GNSocket.h"

using namespace std;

void Scanner::connect(const char *ip, unsigned port)
{
    LOGFMTT("Task Scanner::%s --->>>",__FUNCTION__);

    m_socket.connect_server(ip, port);
    //m_socket.set_recv_timeout(120);

    LOGFMTT("Task Scanner::%s ---<<<",__FUNCTION__);
}

bool Scanner::barcode(std::string &code)
{
    LOGFMTT("Task Scanner::%s --->>>",__FUNCTION__);

    bool result = true;
    string start_cmd = "LON\r";
    string stop_cmd = "LOFF\r";

    try
    {
        m_socket.send_recv(start_cmd, code);
    }
    catch(int &e)
    {
        result = false;
    }

    LOGFMTT("Task Scanner::%s success:%d barcode:%s---<<<",__FUNCTION__, result, code.c_str());

    return result;
}


