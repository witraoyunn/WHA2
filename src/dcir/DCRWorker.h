/**************************************************************
* Copyright (C) 2019, 上海鼎格信息科技有限公司
* All rights reserved.
* 产品号   :
* 所属组件 :
* 模块名称 :
* 文件名称 : DCRWorker.h
* 概要描述 :
* 历史记录 :
* 版本      日期          作者           内容
* 1.0       20190416     zhangw
***************************************************************/
#ifndef DCRWORKER_H_
#define DCRWORKER_H_

#include <string.h>
#include "zmq.hpp"
#include "Scanner.h"
#include "CipClient.h"

class DCRWorker
{
public:
    DCRWorker(int cell_no);

    std::thread run();
    void work();

private:
    Scanner m_scaner;
    //CipClient m_plc  = *CipClient :: instance();

    int m_cell_no;

    enum state
    {
        ST_IDLE,
        ST_SCANED,
        ST_MOVING,
        ST_INSPECTTING,
		ST_EXITING,
    } m_state;
};

#endif //DCRWORKER_H_

