/**********************************************************************************
File name:	  DTFCCountDown.cpp
Author:       Kimbo
Version:      V1.5.0
Date:	 	  2016-4-25
Description:  Time class
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/

/********************************** File includes *********************************/
#include "DTFCCountDown.h"

/******************************* Current libs includes ****************************/
#include "DTFCTime.h"

/******************************* System libs includes *****************************/
#include <iostream>
#include <stdio.h>

/*********************************** Name space ***********************************/
using namespace std;
using namespace dtfeverest;
using namespace dtfeverest::dtfhwdrivers;

/***********************************************************************************
Function:     DTFCCountDown
Description:  The constructor of Count down count
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
DTFCCountDown::DTFCCountDown()
{
    m_end_time = INVALID_TIMESTAMP;
    m_end_flag = false;
    m_time_ms = 0.0;
}

/***********************************************************************************
Function:     DTFCCountDown
Description:  The constructor of Count down count
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
DTFCCountDown::DTFCCountDown(double time_ms)
{
    m_end_time = INVALID_TIMESTAMP;
    m_end_flag = false;
    setTime(time_ms);
}

/***********************************************************************************
Function:     ~DTFCCountDown
Description:  The destructor of file path class
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
DTFCCountDown::~DTFCCountDown()
{

}

/***********************************************************************************
Function:     setTime
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void DTFCCountDown::setTime(double time_ms)
{
    m_end_flag = false;
    m_time_ms = time_ms;
    m_end_time = DTFCTime::addTime(DTFCTime::getCpuTime(), time_ms);
}

/***********************************************************************************
Function:     isEnd
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool DTFCCountDown::isEnd() const
{
    return DTFCTime::getCpuTime() > m_end_time? true: false;
}

/***********************************************************************************
Function:     getLeftTime
Description:  Get left time, unit is ms
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
double DTFCCountDown::getLeftTime() const
{
    if(m_end_time != INVALID_TIMESTAMP)
    {
        return isEnd()? 0 : DTFCTime::timeDifference(DTFCTime::getCpuTime(), m_end_time);
    }
    else
    {
        printf("[DTFCCountDown] end time is INVALID_TIMESTAMP!\n");
        return -1.0;
    }
}


