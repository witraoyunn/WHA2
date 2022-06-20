#ifndef _LIBXMLAPI_H_  
#define _LIBXMLAPI_H_  
  
#include <stdio.h>  
#include <stdlib.h>  
#include <stdint.h>
#include <libxml/parser.h>  
#include <libxml/tree.h>  
#include <libxml/xpath.h>  
#include <libxml/xpathInternals.h>  
#include <libxml/xmlmemory.h>  
#include <libxml/xpointer.h>  
#include <string>  
#include <iostream>  
#include "utility.h"
#include "ChannelState.h"

#define RUN_STATUS_XML_PATH 				"runStatusxml"
#define GLOBAL_PRO_XML_PATH 				"globalProxml"
#define WORK_STEP_XML_PATH 					"workstepxml"


typedef struct
{
	float OverVoltage;
	float UnderVoltage;
	float VoltageFluctuateRange;
	
	float DischargeOverCurrent;
	float ChargeOverCurrent;

	float CurrentFluctuateRange;
	
	float ContactResistance;
	float LoopResistance;

	float BatteryLowerTemp;
	float BatteryOverTemp;
	float BatteryUltraTemp;
	float BatteryUltraHighTemp;
	float BatteryTempSmallFluctuateRange;
	
	float BatteryTempLargeFluctuate;
	float BatteryTempSuddenRise;

	float CellLowerTemp;
	float CellUltraTemp;
	float CellUltraHighTemp;
	float OverCapacity;
	float TotalCapacityProtect;

	float InitUpperVoltage;
	float InitLowerVoltage;
	
	float ContactTestCurrent;
	float ContactTestDeltaVoltage;
	float AuxVoltage;

	float EndTotalCapacityUpper;
	float EndTotalCapacityLower;
	float VacuumFluctuate;

	int VoltageFluctuateCount;
	int CurrentFluctuateCount;
	int BatteryTempSmallFluctuateCount;
	int ContactTestTime;
}global_protect_t;

typedef struct
{
	float UpperVoltage;
	float LowerVoltage;
	float VoltageOverDiff;
	
	float PositiveDeltaVoltage;
	int PositiveDeltaVoltageCount;
	float NegativeDeltaVoltage;
	int NegativeDeltaVoltageCount;
	float CrossStepPositiveDeltaVoltage;
	float CrossStepNegativeDeltaVoltage;
	
	float UpperTimeVoltage;
	float LowerTimeVoltage;
	int TimeVoltageCheckTime;
	int RangeVoltageCheckTime1;
	int RangeVoltageCheckTime2;
	//float DeltaTimeVoltage;
	float UpperDeltaRangeVoltage;
	float LowerDeltaRangeVoltage;

	int VoltageCheckCycleTime;
	float UpperDeltaCycleVoltage;
	float LowerDeltaCycleVoltage;
	
	float OverCurrent;
	float CurrentOverDiff;
	float PositiveDeltaCurrent;
	int PositiveDeltaCurrentCount;
	float NegativeDeltaCurrent;
	int NegativeDeltaCurrentCount;
	
	float OverCapacity;
	float BatteryUpperTemp;
	float BatteryLowerTemp;

	float LowerStepStopVoltage;
	float LowerStepStopCapacity;
	int LowerStepStopTime;
}step_protect_t;

typedef struct
{
	int StepNo;
	char StepName[64];
	
	float RegularCurrent;
	float RegularVoltage;

	int RecordFrequency;
	
	float AbortCurrent;
	int CurrentAbortPrio;
	float AbortVoltage;
	int VoltageAbortPrio;
	float AbortCapacity;
	int CapacityAbortPrio;
	int AbortMinute;
	int AbortSecond;
	int TimeAbortPrio;
	
	int AbortAction;
	int JumpToStep;
	
	int CycleFrom;
	int CycleCount;
	float VacValue;
	int VacTime;
	float LeakValue;
	int LeakTime;
	step_protect_t stepProtect;
}work_step_recipe_t;

typedef struct
{
	int Active;
	int Finish;
	
	int StepNo;
	int JumpTo;
#if (defined(AWH_GRADING_SYS) || defined(AWH_PRECHARGE_SYS))	
	bool loop_flag;
	bool loop_finish;
	int8_t loop_clear_delay;
#endif
	int LoopNo;
	int StepSum;
	int TotalTime;

	char BatteryType[INFO_SIZE];	//电池类型
	char RecipeName[INFO_SIZE];		//配方名称
	char TrayNo[INFO_SIZE];			//托盘码
	char BatchNo[INFO_SIZE];		//批次号
	char BatteryNo[INFO_SIZE];		//电芯码
}channel_run_status_t;

typedef struct
{
	int StepNo;
	int LoopCnt;
}loopstep_info_t;

/*
typedef struct   
{  
    uint8_t steptype; 
	format_data_t rated[3];
	format_data_t endvalue[4];
	format_data_t stepprotect[6];
}work_step_info_t;  
*/

class XmlProcAPI  
{  
public:  
	XmlProcAPI();  
	XmlProcAPI(const char *xml_file_path);  
	~XmlProcAPI();  

    bool xmlopen(const char *xml_file_path);
	
	void update_node_value(const char *key, const char *value);
	void update_node_attr(const char *key, const char *attr_name, const char *value);
	void endUpdate();
	bool getRunStatus(channel_run_status_t &runStatus,int chanNo);
	bool saveRunStatus(const char* m_save_path,channel_run_status_t* chanRun,int chanCnt,int cellNo);

	void getGlobalProtect(global_protect_t &global_v);
	bool saveGlobalProtect(const char* m_save_path,global_protect_t &global_v,int chanNo,int cellNo);
	
	int getTotalWorkstepCount();
	bool getWorkstepPara(work_step_recipe_t &stepInfo,int stepNO);
	bool saveStepRecipeData(const char *m_save_path,work_step_recipe_t* stepInfo,int stepCount);  

private:  	
    xmlNodePtr search_node_ptr(const char *sz_expr);  
	xmlXPathObjectPtr get_XPath_object(const char* key);
  
private:  
    char m_sz_path[512];  
    xmlDocPtr m_pdoc_read;  
    xmlNodePtr m_proot;  
	//int totalSteps;
};  
  
#endif // 
