#include <string.h>  
#include "Configuration.h"
#include "XmlProcAPI.h"

using namespace std;

XmlProcAPI::XmlProcAPI()  
{  
	m_pdoc_read = NULL;  
    m_proot = NULL;  
	memset(m_sz_path, 0,sizeof(m_sz_path));  
}  
	  
XmlProcAPI::~XmlProcAPI()  
{  
	if (m_pdoc_read)  
	{  
		xmlFreeDoc(m_pdoc_read);  
		m_pdoc_read = NULL;  
	}  
}  

XmlProcAPI::XmlProcAPI(const char *xml_file_path)  
{  
	m_pdoc_read = NULL;  
    m_proot = NULL;  
	memset(m_sz_path, 0,sizeof(m_sz_path));  

	if (xml_file_path)  
	{  
		xmlopen(xml_file_path);  
	}  
}  

bool XmlProcAPI::xmlopen(const char *xml_file_path)  
{  
	bool bret = false;  
     
	//m_pdoc_read = NULL;  
	//m_proot = NULL;  
	//bzero(m_sz_path, sizeof(m_sz_path));  
	
	if (xml_file_path)  
	{  
		strcpy(m_sz_path, xml_file_path);  

		xmlKeepBlanksDefault(0);  
		m_pdoc_read = xmlReadFile(xml_file_path, "UTF-8", XML_PARSE_RECOVER);  
		m_proot = xmlDocGetRootElement(m_pdoc_read);  
 
		if (m_proot)  
		{  
			bret = true;  
		}  
	}  

	return bret;  
}  

xmlXPathObjectPtr XmlProcAPI::get_XPath_object(const char* key)
{
	if (key == NULL)  
	{  
		return NULL;  
	}  
	
    //获取xpath上下文
    xmlXPathContextPtr context = xmlXPathNewContext(m_pdoc_read);
    if (!context)
    {
        return NULL;
    }
    
    //获取xpath对象指针
    xmlChar *xml_key = BAD_CAST(key);
    xmlXPathObjectPtr result = xmlXPathEvalExpression(xml_key, context);

    //释放xpath上下文
    xmlXPathFreeContext(context);

    if (!result)
    {
        return NULL;
    }
    //结果为空
    if (xmlXPathNodeSetIsEmpty(result->nodesetval))
    {
    	xmlXPathFreeObject(result);
        return NULL;
    }

    return result;
}
	  
xmlNodePtr XmlProcAPI::search_node_ptr(const char *sz_expr)  
{  
	xmlNodePtr node_ret = NULL;  
  
	xmlXPathObjectPtr result = get_XPath_object(sz_expr);
	
	if(result)
	{
		node_ret = xmlXPtrBuildNodeList(result); 
		xmlXPathFreeObject(result);
	}
	
	return node_ret;  
}  


//更新结点内容
void XmlProcAPI::update_node_value(const char *key, const char *value)
{	
	xmlXPathObjectPtr result = get_XPath_object(key);
    if (!result)
    {
        return;
    }
	
	xmlChar *xml_value = NULL;
    xmlNodeSetPtr nodeset = result->nodesetval;
    for (int i = 0; i < nodeset->nodeNr; i++)
    {
        //更新结点值
		xml_value = BAD_CAST(value);
		xmlNodeSetContent(nodeset->nodeTab[i], xml_value);
    }
    xmlXPathFreeObject(result);
}


//更新结点属性值
void XmlProcAPI::update_node_attr(const char *key, const char *attr_name, const char *value)
{
	xmlXPathObjectPtr result = get_XPath_object(key);
    if (!result)
    {
        return;
    }

	xmlChar *xml_prop_name = NULL;
	xmlChar *xml_prop_value = NULL;
	xmlNodeSetPtr nodeset = result->nodesetval;
	for (int i = 0; i < nodeset->nodeNr; i++)
    {
        //更新结点值
		xml_prop_name = BAD_CAST(attr_name);
		xml_prop_value = BAD_CAST(value);
		xmlSetProp(nodeset->nodeTab[i], xml_prop_name, xml_prop_value);
    }
    xmlXPathFreeObject(result);
}

void XmlProcAPI::endUpdate()
{
 	xmlSaveFormatFileEnc(m_sz_path, m_pdoc_read, "UTF-8", 1);	
}

bool XmlProcAPI::getRunStatus(channel_run_status_t &runStatus,int chanNo)  
{
	if((chanNo > MAX_CHANNELS_NBR) || (chanNo < 1))
	{
		return false;
	}
	
	if (m_proot)  
	{  
		char xpathRep[128];  
		xmlChar *str = NULL;
		
		sprintf(xpathRep, "//runStatus/channel[@No='%d']/Active", chanNo);
		xmlNodePtr node = search_node_ptr(xpathRep);  
		if(node)
		{
			str = xmlNodeGetContent(node);  
			runStatus.Active = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNode(node);
		}
		
		sprintf(xpathRep, "//runStatus/channel[@No='%d']/StepNo", chanNo);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			runStatus.StepNo = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNode(node);
		}

		sprintf(xpathRep, "//runStatus/channel[@No='%d']/JumpTo", chanNo);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			runStatus.JumpTo = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNode(node);
		}

		sprintf(xpathRep, "//runStatus/channel[@No='%d']/LoopNo", chanNo);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			runStatus.LoopNo = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNode(node);
		}

		sprintf(xpathRep, "//runStatus/channel[@No='%d']/StepSum", chanNo);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			runStatus.StepSum = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNode(node);
		}

		sprintf(xpathRep, "//runStatus/channel[@No='%d']/TotalTime", chanNo);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			runStatus.TotalTime = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNode(node);
		}	

		sprintf(xpathRep, "//runStatus/channel[@No='%d']/BatteryType", chanNo);
		node = search_node_ptr(xpathRep);  
		if(node)
		{
			str = xmlNodeGetContent(node);
			snprintf(runStatus.BatteryType,128,"%s",(const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNode(node);
		}

		sprintf(xpathRep, "//runStatus/channel[@No='%d']/RecipeName", chanNo);
		node = search_node_ptr(xpathRep);  
		if(node)
		{
			str = xmlNodeGetContent(node);
			snprintf(runStatus.RecipeName,128,"%s",(const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNode(node);
		}

		sprintf(xpathRep, "//runStatus/channel[@No='%d']/TrayNo", chanNo);
		node = search_node_ptr(xpathRep);  
		if(node)
		{
			str = xmlNodeGetContent(node);
			snprintf(runStatus.TrayNo,128,"%s",(const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNode(node);
		}

		sprintf(xpathRep, "//runStatus/channel[@No='%d']/BatchNo", chanNo);
		node = search_node_ptr(xpathRep);  
		if(node)
		{
			str = xmlNodeGetContent(node);
			snprintf(runStatus.BatchNo,128,"%s",(const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNode(node);
		}

		sprintf(xpathRep, "//runStatus/channel[@No='%d']/BatteryNo", chanNo);
		node = search_node_ptr(xpathRep);  
		if(node)
		{
			str = xmlNodeGetContent(node);	
			snprintf(runStatus.BatteryNo,128,"%s",(const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNode(node);
		}
	}

	return true;
}

bool XmlProcAPI::saveRunStatus(const char* m_save_path,channel_run_status_t* chanRun,int chanCnt,int cellNo)  
{
	if (NULL == m_save_path)  
	{  
		return false;  
	}  

    char sz_temp[128];
	
	xmlDocPtr pdoc = xmlNewDoc(BAD_CAST("1.0"));  
	xmlNodePtr rootNode = xmlNewNode(NULL, BAD_CAST"runStatus");
	xmlDocSetRootElement(pdoc, rootNode);  
	sprintf(sz_temp, "%d", cellNo); 
    xmlNewTextChild(rootNode, NULL, BAD_CAST"cellNo", BAD_CAST(sz_temp)); 
	sprintf(sz_temp, "%d", chanCnt); 
    xmlNewTextChild(rootNode, NULL, BAD_CAST"channelCnt", BAD_CAST(sz_temp));  

	for(int i = 0; i < chanCnt; i++)
	{
		xmlNodePtr chanNode = xmlNewNode(NULL, BAD_CAST"channel");  
		sprintf(sz_temp, "%d", i+1);
		xmlNewProp(chanNode, BAD_CAST"No", BAD_CAST(sz_temp));	
		xmlAddChild(rootNode, chanNode);  

		sprintf(sz_temp, "%d", chanRun[i].Active);
		xmlNewTextChild(chanNode, NULL, BAD_CAST"Active", BAD_CAST(sz_temp));
		//sprintf(sz_temp, "%d", chanRun[i].Finish);
		//xmlNewTextChild(chanNode, NULL, BAD_CAST"Finish", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", chanRun[i].StepNo);
		xmlNewTextChild(chanNode, NULL, BAD_CAST"StepNo", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", chanRun[i].JumpTo);
		xmlNewTextChild(chanNode, NULL, BAD_CAST"JumpTo", BAD_CAST(sz_temp));

		sprintf(sz_temp, "%d", chanRun[i].LoopNo);
		xmlNewTextChild(chanNode, NULL, BAD_CAST"LoopNo", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", chanRun[i].StepSum);
		xmlNewTextChild(chanNode, NULL, BAD_CAST"StepSum", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", chanRun[i].TotalTime);
		xmlNewTextChild(chanNode, NULL, BAD_CAST"TotalTime", BAD_CAST(sz_temp));

		if(string(chanRun[i].BatteryType) != "")
		{
			sprintf(sz_temp, "%s", chanRun[i].BatteryType);
		}
		else
		{
			sprintf(sz_temp, "%d", 0);
		}
		xmlNewTextChild(chanNode, NULL, BAD_CAST"BatteryType", BAD_CAST(sz_temp));

		if(string(chanRun[i].RecipeName) != "")
		{
			sprintf(sz_temp, "%s", chanRun[i].RecipeName);
		}
		else
		{
			sprintf(sz_temp, "%d", 0);
		}		
		xmlNewTextChild(chanNode, NULL, BAD_CAST"RecipeName", BAD_CAST(sz_temp));

		if(string(chanRun[i].TrayNo) != "")
		{
			sprintf(sz_temp, "%s", chanRun[i].TrayNo);
		}
		else
		{
			sprintf(sz_temp, "%d", 0);
		}
		xmlNewTextChild(chanNode, NULL, BAD_CAST"TrayNo", BAD_CAST(sz_temp));

		if(string(chanRun[i].BatchNo) != "")
		{
			sprintf(sz_temp, "%s", chanRun[i].BatchNo);
		}
		else
		{
			sprintf(sz_temp, "%d", 0);
		}
		xmlNewTextChild(chanNode, NULL, BAD_CAST"BatchNo", BAD_CAST(sz_temp));

		if(string(chanRun[i].BatteryNo) != "")
		{
			sprintf(sz_temp, "%s", chanRun[i].BatteryNo);
		}
		else
		{
			sprintf(sz_temp, "%d", 0);
		}
		xmlNewTextChild(chanNode, NULL, BAD_CAST"BatteryNo", BAD_CAST(sz_temp));
	}

	xmlSaveFormatFileEnc(m_save_path, pdoc, "UTF-8", 1);
	xmlFreeDoc(pdoc);
	return true;
}

void XmlProcAPI::getGlobalProtect(global_protect_t &global_v)
{	
	if (m_proot)
	{  
		char xpathRep[128];  
		char item_str[64];
		xmlChar *str = NULL;
		xmlNodePtr node = NULL;

		//要与global_protect_t成员顺序一致
		string float_items[] = {"OverVoltage", "UnderVoltage","VoltageFluctuateRange","DischargeOverCurrent","ChargeOverCurrent",
		"CurrentFluctuateRange","ContactResistance","LoopResistance","BatteryLowerTemp","BatteryOverTemp",
		"BatteryUltraTemp","BatteryUltraHighTemp","BatteryTempSmallFluctuateRange","BatteryTempLargeFluctuate","BatteryTempSuddenRise",
		"CellLowerTemp","CellUltraTemp","CellUltraHighTemp","OverCapacity", "TotalCapacityProtect",
		"InitUpperVoltage","InitLowerVoltage","ContactTestCurrent","ContactTestDeltaVoltage","AuxVoltage",
		"EndTotalCapacityUpper","EndTotalCapacityLower","VacuumFluctuate"};
		
		string int_items[] = {"VoltageFluctuateCount","CurrentFluctuateCount","BatteryTempSmallFluctuateCount","ContactTestTime"};

		float *p_flt_global = (float*)&global_v.OverVoltage;
		for(int i = 0; i < 28; i++)
		{
			sprintf(xpathRep,"//GlobalProtect/channel/");
			sprintf(item_str,"%s",float_items[i].c_str());
			strcat(xpathRep,item_str);

			node = search_node_ptr(xpathRep);
			if(node)
			{
				str = xmlNodeGetContent(node);  
				*(p_flt_global+i) = atof((const char*)BAD_CAST(str));
				xmlFree(str);
				xmlFreeNode(node);
			}
		}

		int *p_int_global = (int*)&global_v.VoltageFluctuateCount;
		for(int i=0; i < 4; i++)
		{
			sprintf(xpathRep,"//GlobalProtect/channel/");
			sprintf(item_str,"%s",int_items[i].c_str());
			strcat(xpathRep,item_str);

			node = search_node_ptr(xpathRep); 
			if(node)
			{
				str = xmlNodeGetContent(node);  
				*(p_int_global+i) = atoi((const char*)BAD_CAST(str));
				xmlFree(str);
				xmlFreeNode(node);
			}
		}
	}
}

bool XmlProcAPI::saveGlobalProtect(const char* m_save_path,global_protect_t &global_v,int chanNo,int cellNo)  
{
	if (NULL == m_save_path)  
	{  
		return false;  
	}  

    char sz_temp[64];  
	
	xmlDocPtr pdoc = xmlNewDoc(BAD_CAST("1.0"));  
	xmlNodePtr rootNode = xmlNewNode(NULL, BAD_CAST"GlobalProtect");
	xmlDocSetRootElement(pdoc, rootNode);  
	sprintf(sz_temp, "%d", cellNo); 
    xmlNewTextChild(rootNode, NULL, BAD_CAST"cellNo", BAD_CAST(sz_temp)); 
	sprintf(sz_temp, "%d", chanNo);
	xmlNewTextChild(rootNode, NULL, BAD_CAST"channelNo", BAD_CAST(sz_temp)); 

	xmlNodePtr chanNode = xmlNewNode(NULL, BAD_CAST"channel");  
	xmlAddChild(rootNode, chanNode);  

	sprintf(sz_temp, "%f", global_v.OverVoltage); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"OverVoltage", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%f", global_v.UnderVoltage); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"UnderVoltage", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%f", global_v.VoltageFluctuateRange); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"VoltageFluctuateRange", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%d", global_v.VoltageFluctuateCount); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"VoltageFluctuateCount", BAD_CAST(sz_temp));

	sprintf(sz_temp, "%f", global_v.DischargeOverCurrent); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"DischargeOverCurrent", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%f", global_v.ChargeOverCurrent); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"ChargeOverCurrent", BAD_CAST(sz_temp));

	sprintf(sz_temp, "%f", global_v.CurrentFluctuateRange); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"CurrentFluctuateRange", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%d", global_v.CurrentFluctuateCount); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"CurrentFluctuateCount", BAD_CAST(sz_temp));

	sprintf(sz_temp, "%f", global_v.ContactResistance); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"ContactResistance", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%f", global_v.LoopResistance); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"LoopResistance", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%f", global_v.BatteryLowerTemp); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"BatteryLowerTemp", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%f", global_v.BatteryOverTemp); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"BatteryOverTemp", BAD_CAST(sz_temp));

	sprintf(sz_temp, "%f", global_v.BatteryUltraTemp); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"BatteryUltraTemp", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%f", global_v.BatteryUltraHighTemp); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"BatteryUltraHighTemp", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%f", global_v.BatteryTempSmallFluctuateRange); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"BatteryTempSmallFluctuateRange", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%d", global_v.BatteryTempSmallFluctuateCount); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"BatteryTempSmallFluctuateCount", BAD_CAST(sz_temp));

	sprintf(sz_temp, "%f", global_v.BatteryTempLargeFluctuate); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"BatteryTempLargeFluctuate", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%f", global_v.BatteryTempSuddenRise); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"BatteryTempSuddenRise", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%f", global_v.CellLowerTemp); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"CellLowerTemp", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%f", global_v.CellUltraTemp); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"CellUltraTemp", BAD_CAST(sz_temp));

	sprintf(sz_temp, "%f", global_v.CellUltraHighTemp); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"CellUltraHighTemp", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%f", global_v.OverCapacity); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"OverCapacity", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%f", global_v.TotalCapacityProtect); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"TotalCapacityProtect", BAD_CAST(sz_temp));

	sprintf(sz_temp, "%f", global_v.InitUpperVoltage); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"InitUpperVoltage", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%f", global_v.InitLowerVoltage); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"InitLowerVoltage", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%d", global_v.ContactTestTime); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"ContactTestTime", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%f", global_v.ContactTestCurrent); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"ContactTestCurrent", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%f", global_v.ContactTestDeltaVoltage); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"ContactTestDeltaVoltage", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%f", global_v.AuxVoltage); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"AuxVoltage", BAD_CAST(sz_temp));

	sprintf(sz_temp, "%f", global_v.EndTotalCapacityUpper); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"EndTotalCapacityUpper", BAD_CAST(sz_temp));
	sprintf(sz_temp, "%f", global_v.EndTotalCapacityLower); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"EndTotalCapacityLower", BAD_CAST(sz_temp));

	sprintf(sz_temp, "%f", global_v.VacuumFluctuate); 
	xmlNewTextChild(chanNode, NULL, BAD_CAST"VacuumFluctuate", BAD_CAST(sz_temp));

	xmlSaveFormatFileEnc(m_save_path, pdoc, "UTF-8", 1);  
	xmlFreeDoc(pdoc);  
	return true;  
}


int XmlProcAPI::getTotalWorkstepCount()  
{
	int count = -1;  
	if (m_proot)  
	{
		xmlNodePtr node = search_node_ptr("//TotalStep");  
		
		xmlChar *str = xmlNodeGetContent(node);  
		count = atoi((const char*)BAD_CAST(str));  
		xmlFree(str);
		xmlFreeNodeList(node);
	}

	return count;  
}

bool XmlProcAPI::getWorkstepPara(work_step_recipe_t &stepInfo,int stepNO)  
{  
	bool bret = false; 
	
	if((stepNO > 255) || (stepNO < 1))
	{
		return bret;
	}
	
	if (m_proot)  
	{  
		char xpathRep[128];  
		xmlChar *str = NULL;

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepNo", stepNO);
		xmlNodePtr node = search_node_ptr(xpathRep);  
		if(node)
		{
			str = xmlNodeGetContent(node);  
			stepInfo.StepNo = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}
		
		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepName", stepNO);
		node = search_node_ptr(xpathRep);  
		if(node)
		{
			str = xmlNodeGetContent(node);  
			sprintf(stepInfo.StepName,"%s",(const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}
		
		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/RegularCurrent", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.RegularCurrent = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/RegularVoltage", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.RegularVoltage = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/RecordFrequency", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.RecordFrequency = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/AbortCurrent", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.AbortCurrent = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/CurrentAbortPrio", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.CurrentAbortPrio = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/AbortVoltage", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.AbortVoltage = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/VoltageAbortPrio", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.VoltageAbortPrio = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/AbortCapacity", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.AbortCapacity = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/CapacityAbortPrio", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.CapacityAbortPrio = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/AbortMinute", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.AbortMinute = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/AbortSecond", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.AbortSecond = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/TimeAbortPrio", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.TimeAbortPrio = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/AbortAction", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.AbortAction = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/JumpToStep", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.JumpToStep = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/CycleFrom", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.CycleFrom = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/CycleCount", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.CycleCount = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/VacuumValue", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.VacValue = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/VacuumTime", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.VacTime = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/ReleaseValue", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.LeakValue = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/ReleaseTime", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.LeakTime = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/UpperVoltage", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.UpperVoltage = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/LowerVoltage", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.LowerVoltage = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/VoltageOverDiff", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.VoltageOverDiff = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}
		
		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/PositiveDeltaVoltage", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.PositiveDeltaVoltage = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}
		
		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/PositiveDeltaVoltageCount", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.PositiveDeltaVoltageCount = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/NegativeDeltaVoltage", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.NegativeDeltaVoltage = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}
	
		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/NegativeDeltaVoltageCount", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.NegativeDeltaVoltageCount = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/CrossStepPositiveDeltaVoltage", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.CrossStepPositiveDeltaVoltage = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}
		
		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/CrossStepNegativeDeltaVoltage", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.CrossStepNegativeDeltaVoltage = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}
		
		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/UpperTimeVoltage", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.UpperTimeVoltage = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/LowerTimeVoltage", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.LowerTimeVoltage = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/TimeVoltageCheckTime", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.TimeVoltageCheckTime = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}
	
		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/RangeVoltageCheckTime1", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.RangeVoltageCheckTime1 = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/RangeVoltageCheckTime2", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.RangeVoltageCheckTime2 = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/UpperDeltaRangeVoltage", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.UpperDeltaRangeVoltage = atof((const char*)BAD_CAST(str));	
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/LowerDeltaRangeVoltage", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.LowerDeltaRangeVoltage = atof((const char*)BAD_CAST(str));	
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/VoltageCheckCycleTime", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.VoltageCheckCycleTime = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/UpperDeltaCycleVoltage", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.UpperDeltaCycleVoltage = atof((const char*)BAD_CAST(str));	
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/LowerDeltaCycleVoltage", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.LowerDeltaCycleVoltage = atof((const char*)BAD_CAST(str));	
			xmlFree(str);
			xmlFreeNodeList(node);
		}		
		
		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/OverCurrent", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.OverCurrent = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/CurrentOverDiff", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.CurrentOverDiff = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/PositiveDeltaCurrent", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.PositiveDeltaCurrent = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}
	
		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/PositiveDeltaCurrentCount", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.PositiveDeltaCurrentCount = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/NegativeDeltaCurrent", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.NegativeDeltaCurrent = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/NegativeDeltaCurrentCount", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.NegativeDeltaCurrentCount = atoi((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/OverCapacity", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.OverCapacity = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/BatteryLowerTemp", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.BatteryLowerTemp = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/BatteryUpperTemp", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.BatteryUpperTemp = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/LowerStepStopVoltage", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.LowerStepStopVoltage = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/LowerStepStopCapacity", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.LowerStepStopCapacity = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}

		sprintf(xpathRep, "//StepData/StepItems[@No='%d']/StepProtect/LowerStepStopTime", stepNO);
		node = search_node_ptr(xpathRep);
		if(node)
		{
			str = xmlNodeGetContent(node);
			stepInfo.stepProtect.LowerStepStopTime = atof((const char*)BAD_CAST(str));
			xmlFree(str);
			xmlFreeNodeList(node);
		}
	
		bret = true;  
	}  
  
	return bret;  
}  


bool XmlProcAPI::saveStepRecipeData(const char* m_save_path,work_step_recipe_t* stepInfo,int stepCount)  
{  
	if (NULL == m_save_path)  
	{
		return false;  
	}

    char sz_temp[64];  
	//char xpathRep[64];  
	
	xmlDocPtr pdoc = xmlNewDoc(BAD_CAST("1.0"));  
	xmlNodePtr rootNode = xmlNewNode(NULL, BAD_CAST"StepData");
	//sprintf(sz_temp, "%d", active); 		
	//xmlNewProp(rootNode, BAD_CAST"active", BAD_CAST(sz_temp));  
	xmlDocSetRootElement(pdoc, rootNode);  
	sprintf(sz_temp, "%d", stepCount); 
    xmlNewTextChild(rootNode, NULL, BAD_CAST"TotalStep", BAD_CAST(sz_temp));  
	
	for(int i=0;i<stepCount;i++)
	{
		xmlNodePtr workstepNode = xmlNewNode(NULL, BAD_CAST"StepItems");  
		//bzero(sz_temp, sizeof(sz_temp));  
		sprintf(sz_temp, "%d", i+1);
		xmlNewProp(workstepNode, BAD_CAST"No", BAD_CAST(sz_temp));  
		xmlAddChild(rootNode, workstepNode);  
		
		sprintf(sz_temp, "%d", stepInfo[i].StepNo); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"StepNo", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%s", stepInfo[i].StepName); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"StepName", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].RegularCurrent); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"RegularCurrent", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].RegularVoltage); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"RegularVoltage", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", stepInfo[i].RecordFrequency); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"RecordFrequency", BAD_CAST(sz_temp));
		
		sprintf(sz_temp, "%f", stepInfo[i].AbortCurrent); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"AbortCurrent", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", stepInfo[i].CurrentAbortPrio); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"CurrentAbortPrio", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].AbortVoltage); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"AbortVoltage", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", stepInfo[i].VoltageAbortPrio); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"VoltageAbortPrio", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].AbortCapacity); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"AbortCapacity", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", stepInfo[i].CapacityAbortPrio); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"CapacityAbortPrio", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", stepInfo[i].AbortMinute); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"AbortMinute", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", stepInfo[i].AbortSecond); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"AbortSecond", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", stepInfo[i].TimeAbortPrio); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"TimeAbortPrio", BAD_CAST(sz_temp));
		
		sprintf(sz_temp, "%d", stepInfo[i].AbortAction); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"AbortAction", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", stepInfo[i].JumpToStep); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"JumpToStep", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", stepInfo[i].CycleFrom); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"CycleFrom", BAD_CAST(sz_temp));
		//sprintf(sz_temp, "%d", stepInfo[i].CycleTo); 
		//xmlNewTextChild(workstepNode, NULL, BAD_CAST"CycleTo", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", stepInfo[i].CycleCount); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"CycleCount", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].VacValue); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"VacuumValue", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", stepInfo[i].VacTime); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"VacuumTime", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].LeakValue); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"ReleaseValue", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", stepInfo[i].LeakTime); 
		xmlNewTextChild(workstepNode, NULL, BAD_CAST"ReleaseTime", BAD_CAST(sz_temp));

		xmlNodePtr stepProtectNode = xmlNewNode(NULL, BAD_CAST"StepProtect");  
		xmlAddChild(workstepNode, stepProtectNode);  

		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.UpperVoltage); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"UpperVoltage", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.LowerVoltage); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"LowerVoltage", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.VoltageOverDiff); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"VoltageOverDiff", BAD_CAST(sz_temp));

		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.PositiveDeltaVoltage); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"PositiveDeltaVoltage", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", stepInfo[i].stepProtect.PositiveDeltaVoltageCount); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"PositiveDeltaVoltageCount", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.NegativeDeltaVoltage); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"NegativeDeltaVoltage", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", stepInfo[i].stepProtect.NegativeDeltaVoltageCount); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"NegativeDeltaVoltageCount", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.CrossStepPositiveDeltaVoltage); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"CrossStepPositiveDeltaVoltage", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.CrossStepNegativeDeltaVoltage); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"CrossStepNegativeDeltaVoltage", BAD_CAST(sz_temp));

		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.UpperTimeVoltage);
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"UpperTimeVoltage", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.LowerTimeVoltage);
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"LowerTimeVoltage", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", stepInfo[i].stepProtect.TimeVoltageCheckTime);
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"TimeVoltageCheckTime", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", stepInfo[i].stepProtect.RangeVoltageCheckTime1); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"RangeVoltageCheckTime1", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", stepInfo[i].stepProtect.RangeVoltageCheckTime2); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"RangeVoltageCheckTime2", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.UpperDeltaRangeVoltage); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"UpperDeltaRangeVoltage", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.LowerDeltaRangeVoltage); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"LowerDeltaRangeVoltage", BAD_CAST(sz_temp));

		sprintf(sz_temp, "%d", stepInfo[i].stepProtect.VoltageCheckCycleTime);
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"VoltageCheckCycleTime", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.UpperDeltaCycleVoltage);
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"UpperDeltaCycleVoltage", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.LowerDeltaCycleVoltage);
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"LowerDeltaCycleVoltage", BAD_CAST(sz_temp));

		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.OverCurrent); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"OverCurrent", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.CurrentOverDiff); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"CurrentOverDiff", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.PositiveDeltaCurrent); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"PositiveDeltaCurrent", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", stepInfo[i].stepProtect.PositiveDeltaCurrentCount); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"PositiveDeltaCurrentCount", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.NegativeDeltaCurrent); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"NegativeDeltaCurrent", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", stepInfo[i].stepProtect.NegativeDeltaCurrentCount); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"NegativeDeltaCurrentCount", BAD_CAST(sz_temp));

		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.OverCapacity); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"OverCapacity", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.BatteryLowerTemp); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"BatteryLowerTemp", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.BatteryUpperTemp); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"BatteryUpperTemp", BAD_CAST(sz_temp));

		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.LowerStepStopVoltage); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"LowerStepStopVoltage", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%f", stepInfo[i].stepProtect.LowerStepStopCapacity); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"LowerStepStopCapacity", BAD_CAST(sz_temp));
		sprintf(sz_temp, "%d", stepInfo[i].stepProtect.LowerStepStopTime); 
		xmlNewTextChild(stepProtectNode, NULL, BAD_CAST"LowerStepStopTime", BAD_CAST(sz_temp));
	}

	xmlSaveFormatFileEnc(m_save_path, pdoc, "UTF-8", 1);  
	xmlFreeDoc(pdoc);  
	return true;  
}  


