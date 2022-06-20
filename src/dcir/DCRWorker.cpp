#include <thread>
#include <sstream>
#include <string>
#include "log4z.h"
#include "Type.h"
#include "MsgFactory.h"
#include "DCRWorker.h"

#include "Configuration.h"
#include "FormatConvert.h"

using namespace std;

DCRWorker::DCRWorker(int cell_no)
        : m_cell_no(cell_no)
        , m_state(ST_IDLE)
{}

thread DCRWorker::run()
{
    return thread(&DCRWorker::work, this);
}

void DCRWorker::work()
{
    LOGFMTT("new Task DCRWorker::%s --->>>",__FUNCTION__);

    CipClient::instance()->initialize(Configuration::instance()->plc_ip_address().c_str(), 9600);
    m_scaner.connect(Configuration::instance()->scanner_ip_address().c_str(), 9004);

    MsgFactory::instance()->create_pins_alarm_pusher(Configuration::instance()->ext_alarm_socket());
    try
    {
        MsgFactory::instance()->create_pins_puller(Configuration::instance()->int_pins_puller_socket());
        MsgFactory::instance()->create_pins_to_ext_reply_pusher(Configuration::instance()->ext_reply_socket());
    }
    catch (zmq::error_t &e)
    {
        LOGFMTE("Create ZMQ communication error. code: %d, description:%s", e.num(), e.what());
        string josn_str = FormatConvert::instance().alarm_to_string(-209);
        MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
    }

    while(1)
    {
        msleep(500);
        try
        {
            if (m_state == ST_IDLE && CipClient::instance()->check_battery_enter())
            {
                string barcode = "0123456789";
//                m_scaner.barcode(barcode);
                string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, -1, DCR_ACK_SCAN_RESULT, -1, barcode);
                MsgFactory::instance()->pins_to_ext_reply_pusher().send(josn_str);
                m_state = ST_SCANED;
            }

            int station_id = 0;
            if (m_state == ST_MOVING && CipClient::instance()->check_move_battery_finish(&station_id))
            {
                string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, -1, DCR_ACK_PREPARE_FINISHED, -1, station_id);
                MsgFactory::instance()->pins_to_ext_reply_pusher().send(josn_str);
                m_state = ST_INSPECTTING;
            }

            if (m_state == ST_MOVING && CipClient::instance()->check_request_exit())
            {
                string josn_str = FormatConvert::instance().reply_to_string(m_cell_no, -1, DCR_ACK_REQUEST_EXIT, -1, -1);
                MsgFactory::instance()->pins_to_ext_reply_pusher().send(josn_str);
				m_state = ST_EXITING;
            }

            Cmd_Pins_Msg_t receive_data = Cmd_Pins_Msg_t();
            if (MsgFactory::instance()->pins_puller().recevie(receive_data, ZMQ_DONTWAIT))
            {
                LOGFMTD("DCRWorker Receive Msg, PinsGroup NO:%d, Func_code:0x%x", receive_data.cell_no, receive_data.func_code);

                switch (receive_data.func_code)
                {
                    case DCR_CMD_PREPARE_INSPECT:
                    {
                        CipClient::instance()->request_move_battery(m_state == ST_SCANED);
                        m_state = ST_MOVING;
                        break;
                    }
                    case DCR_CMD_ALLOW_EXIT:
                    {
                        CipClient::instance()->permit_exit();
                        m_state = ST_IDLE;
                        break;
                    }
                    case DCR_CMD_RESET_STATE:
                    {
                        m_state = ST_IDLE;
                        break;
                    }
                    default:
                        break;
                }

                string josn_str = FormatConvert::instance().reply_to_string(receive_data.cell_no, -1, receive_data.func_code, -1, true);
                MsgFactory::instance()->pins_to_ext_reply_pusher().send(josn_str);
            }

        }
        catch (int &e)
        {
            string josn_str = FormatConvert::instance().alarm_to_string(e);
            MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
        }
    }

    try
    {
        MsgFactory::instance()->destory_pins_puller();
        MsgFactory::instance()->destory_pins_to_ext_reply_pusher();
    }
    catch (zmq::error_t &e)
    {
        LOGFMTE("Create ZMQ communication error. code: %d, description:%s", e.num(), e.what());
        string josn_str = FormatConvert::instance().alarm_to_string(-209);
        MsgFactory::instance()->pins_alarm_pusher().send(josn_str);
    }

    MsgFactory::instance()->destory_pins_alarm_pusher();

	LOGFMTT("Task DCRWorker::%s ---<<<", __FUNCTION__);
}


