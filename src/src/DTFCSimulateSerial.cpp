/**********************************************************************************
File name:	  DTFCSimulateSerial.cpp
Author:       Shizhe
Version:      V1.6.1
Date:	 	  2016-3-2
Description:  The class is used to receive robot packet
Others:       None

History:
	1. Date: 2015-09-21
	Author: Kimbo
	Modification: Refactor this class
***********************************************************************************/

/********************************** File includes *********************************/
#include <DTFCSimulateSerial.h>

/********************************** Current libs includes *************************/
#include <DTFCArcTime.h>
#include <DTFCCountDown.h>

/********************************** System libs includes **************************/
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <iostream>

/********************************** Name space ************************************/
using namespace std;
using namespace dtfeverest;
using namespace dtfeverest::dtfhwdrivers;


/***********************************************************************************
Function:     DTFCSimulateSerial
Description:  The constructor of DTFCSimulateSerial
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
DTFCSimulateSerial::DTFCSimulateSerial()
{
    m_port_name = "none";
	m_baud_rate = 0;
	m_status = STATUS_NEVER_OPENED;
}

/***********************************************************************************
Function:     DTFCSimulateSerial
Description:  The Destructor of DTFCSimulateSerial
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
DTFCSimulateSerial::~DTFCSimulateSerial()
{
	if (m_fp)
	{
		m_fp.close();
	}
}

/***********************************************************************************
Function:     internalOpen
Description:  Open the serial port with the internal serial port
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
int DTFCSimulateSerial::internalOpen(void)
{
	if (m_status == STATUS_OPEN)
	{
		printf("[DTFCSimulateSerial]Open: Serial port already open!\n");
		return OPEN_ALREADY_OPEN;
	}

    m_fp.open(m_port_name.c_str(), std::ifstream::in);
    if(!m_fp)
    {
        printf("[DTFCSimulateSerial] Cound not open file %s!\n", m_port_name.c_str());
        m_status = STATUS_OPEN_FAILED;
        return STATUS_OPEN_FAILED;
    }
    else
    {
        m_status = STATUS_OPEN;
    }

	printf("[DTFCSimulateSerial]open: Successfully opened and configured simulate serial port!\n");
	return 0;
}

/***********************************************************************************
Function:     openSimple
Description:  Open the serial port with the internal serial port
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool DTFCSimulateSerial::openSimple(void)
{
	if (internalOpen() == 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/***********************************************************************************
Function:     setPort
Description:  Open the serial port with the internal serial port
Input:        port: The serial port to connect to, or NULL which defaults to
			        COM1 for windows and /dev/ttyS0 for linux
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void DTFCSimulateSerial::setPort(const char *port)
{
	if (port == NULL)
	{
		m_port_name = "none";
	}
	else
	{
		m_port_name = port;
	}
}

/***********************************************************************************
Function:     getPort
Description:  Open the serial port with the internal serial port
Input:        None
Output:       None
Return:       The seiral port to connect to
Others:       None
***********************************************************************************/
const char * DTFCSimulateSerial::getPort(void)
{
  return m_port_name.c_str();
}

/***********************************************************************************
Function:     open
Description:  Open the serial port with the internal serial port
Input:        port: The serial port to connect to, or NULL which defaults to
			        COM1 for windows and /dev/ttyS0 for linux
Output:       None
Return:       None
Others:       None
***********************************************************************************/
int DTFCSimulateSerial::open(const char *port)
{
	setPort(port);
	return internalOpen();
}

/***********************************************************************************
Function:     close
Description:  Close the serial port
Input:        None
Output:       None
Return:       0 for success, otherwise one of the close enums
Others:       None
***********************************************************************************/
bool DTFCSimulateSerial::close(void)
{
	if(m_fp)
	{
	    m_fp.close();
	}
	return true;
}

/***********************************************************************************
Function:     setBaud
Description:  Set serial baud
Input:        rate: the baud rate to set the connection to
Output:       None
Return:       whether the set succeeded
Others:       None
***********************************************************************************/
bool DTFCSimulateSerial::setBaud(int rate)
{
	return true;
}

/***********************************************************************************
Function:     getBaud
Description:  Start serial baud
Input:        None
Output:       None
Return:       the current baud rate of the connection
Others:       None
***********************************************************************************/
int DTFCSimulateSerial::getBaud(void)
{
    return m_baud_rate;
}

/***********************************************************************************
Function:     write
Description:  Send data
Input:        data: the data to send
			  size: the size of the data to send
Output:       None
Return:       true if hardware write data successfully, false otherwise
Others:       None
***********************************************************************************/
int DTFCSimulateSerial::write(const char *data, unsigned int size)
{
	printf("[DTFCSimulateSerial]::write: Connection invalid.!\n");
	return -1;
}

/***********************************************************************************
Function:     write
Description:  Read data
Input:        data: data buffer for read
			  size: the size of the data to read
			  msWait: overlay time
Output:       None
Return:       true if hardware read data successfully, false otherwise
Others:       None
***********************************************************************************/
int DTFCSimulateSerial::read(const char *data, unsigned int size, unsigned int msWait)
{
	unsigned int bytesRead = 0;

    if(!m_fp)
    {
        printf("[DTFCSimulateSerial] Finish read data!\n");
        return -1;
    }

    char *ptr = const_cast<char *>(data);

    while(!m_fp.eof())
    {
        char ch = 0;
        int read_num = 0;
        m_fp >> read_num;
        ch = (char)read_num;
        ptr[bytesRead] = ch;
        bytesRead++;
        if(bytesRead == size)
        {
            break;
        }
        printf("[DTFCSimulateSerial] read ch %d!\n", int(ch));
    }
    return bytesRead;
}

/***********************************************************************************
Function:     getStatus
Description:  Get the serial port status
Input:        None
Output:       None
Return:       return the status the port
Others:       None
***********************************************************************************/
int DTFCSimulateSerial::getStatus(void)
{
	return m_status;
}
