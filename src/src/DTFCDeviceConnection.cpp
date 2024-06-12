/**********************************************************************************
File name:	  DTFCDeviceConnection.cpp
Author:       Shizhe
Version:      V1.6.1
Date:	 	  2016-3-2
Description:  A base class for device connect
Others:       None

History:
	1. Date: 2015-09-15
	Author: Kimbo
	Modification: Refactor this class
***********************************************************************************/

/********************************** File includes *********************************/
#include <DTFCDeviceConnection.h>

/********************************** Current libs includes *************************/

/*********************************** Name space ***********************************/
using namespace std;
using namespace dtfeverest;
using namespace dtfeverest::dtfhwdrivers;

/********************************** Static varible init ***************************/
bool DTFCDeviceConnection::m_str_map_inited = false;
CStrMap DTFCDeviceConnection::m_str_map;

/***********************************************************************************
Function:     DTFCDeviceConnection
Description:  The constructor of DTFCDeviceConnection
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
DTFCDeviceConnection::DTFCDeviceConnection()
{
	if (!m_str_map_inited)
	{
		m_str_map_inited = true;
		buildStrMap();
	}

	m_dc_port_name = "Unknown port name";
	m_dc_port_type = "Unknown port type";
	m_dc_device_name = "Unknown device type";
}

/***********************************************************************************
Function:     ~DTFCDeviceConnection
Description:  The destructor of DTFCDeviceConnection
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
DTFCDeviceConnection::~DTFCDeviceConnection()
{
	close();
}

/***********************************************************************************
Function:     buildStrMap
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void DTFCDeviceConnection::buildStrMap(void)
{
	m_str_map[STATUS_NEVER_OPENED] = "never opened";
	m_str_map[STATUS_OPEN] = "open";
	m_str_map[STATUS_OPEN_FAILED] = "open failed";
	m_str_map[STATUS_CLOSED_NORMALLY] = "closed";
	m_str_map[STATUS_CLOSED_ERROR] = "closed on error";
}

/***********************************************************************************
Function:     buildStrMap
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
const char * DTFCDeviceConnection::getStatusMessage(int messageNumber) const
{
	CStrMap::const_iterator it;
	if ((it = m_str_map.find(messageNumber)) != m_str_map.end())
	{
		return (*it).second.c_str();
	}
	else
	{
		return NULL;
	}
}

/***********************************************************************************
Function:     buildStrMap
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void DTFCDeviceConnection::setPortName(const char *portName)
{
	if (portName != NULL)
	{
		m_dc_port_name = portName;
	}
	else
	{
		m_dc_port_name = "Unknown port name";
	}
}

/***********************************************************************************
Function:     buildStrMap
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
const char *DTFCDeviceConnection::getPortName(void) const
{
	return m_dc_port_name.c_str();
}

/***********************************************************************************
Function:     buildStrMap
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void DTFCDeviceConnection::setPortType(const char *portType)
{
	if (portType != NULL)
	{
		m_dc_port_type = portType;
	}
	else
	{
		m_dc_port_type = "Unknown port type";
	}
}



/***********************************************************************************
Function:     getPortType
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
const char *DTFCDeviceConnection::getPortType(void) const
{
	return m_dc_port_type.c_str();
}

/***********************************************************************************
Function:     setDeviceName
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void DTFCDeviceConnection::setDeviceName(const char *device_name)
{
	if (device_name != NULL)
	{
		m_dc_device_name = device_name;
	}
	else
	{
		m_dc_device_name = "Unknown device name";
	}
}

/***********************************************************************************
Function:     getDeviceName
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
const char *DTFCDeviceConnection::getDeviceName(void) const
{
	return m_dc_device_name.c_str();
}
