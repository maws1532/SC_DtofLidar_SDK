/*********************************************************************************
File name:	  DTFCLidarPacketReceiver.h
Author:       Kimbo
Version:      V1.7.1
Date:	 	  2017-02-03
Description:  lidar packet receiver
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/

/********************************* File includes **********************************/
#include "DTFCLidarPacketReceiver.h"

/******************************* Current libs includes ****************************/
#include "DTFCDeviceConnection.h"
#include "DTFCCountDown.h"

/********************************** Name space ************************************/
using namespace dtfeverest;
using namespace dtfeverest::dtfhwdrivers;

#define LIDAR_PACEKT_HEADER1 0xAA
#define LIDAR_PACEKT_HEADER2 0x00

/***********************************************************************************
Function:     DTFCLidarPacketReceiver
Description:  The constructor of DTFCLidarPacketReceiver
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
DTFCLidarPacketReceiver::DTFCLidarPacketReceiver()
{
    m_device_conn = NULL;
    m_log_when_receive_time_over = false;
    reset();
    m_counter = 0;
    m_SNState = GetSN_ING;
}

/***********************************************************************************
Function:     DTFCLidarPacketReceiver
Description:  The destructor of DTFCLidarPacketReceiver
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
DTFCLidarPacketReceiver::~DTFCLidarPacketReceiver()
{
    if(m_save_fp)
    {
        m_save_fp.close();
    }
}

/***********************************************************************************
Function:     receivePacket
Description:  Receive lidar packet, if return true, it means receive a valid packet
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool DTFCLidarPacketReceiver::receivePacket(DTFCLidarPacket *packet)
{
	/* Judge whether serial is connecting */
	if (packet == NULL || m_device_conn == NULL || m_device_conn->getStatus() != DTFCDeviceConnection::STATUS_OPEN)
	{
		printf("[DTFCLidarPacketReceiver] receivePacket: connection not open!\n");
		return false;
	}
    /* Read packet */
	m_count_down.setTime((double)m_params.packet_max_time_ms);
	char ch;
	while(1)
	{
		if(m_count_down.isEnd())
		{
			//printf("[DTFCLidarPacketReceiver] Receive packet time %5.2f ms is over!\n", m_count_down.getInputTime());
            printf("(LINE:%d)-<FUNCTION:%s> revice data over\n",__LINE__,__FUNCTION__);
            packet->m_lidar_erro = LIDAR_ERROR_TIME_OVER;
			if(m_log_when_receive_time_over)
            {
                printf("[DTFCLidarPacketReceiver] Receive packet time is over!\n");
            }
            TPacketResult packet_result = readPacket(packet, ch);
			return false;
		}
        if(LIDAR_ERROR_TIME_OVER == packet->m_lidar_erro)
        {
            packet->m_lidar_erro = LIDAR_ERROR_NONE;
        }
        
        int read_bytes = m_device_conn->read((char *)&ch, 1, 1);
		if(read_bytes == 0)
		{
			
            if(GetSN_ING == GetSNFlag())
                return false;
            else
                continue;
		}
		else if(read_bytes < 0)
		{
		    printf("[DTFCLidarPacketReceiver] finish read data read bytes is %d!\n", read_bytes);
		    return false;
		}
		else
		{
		    TPacketResult packet_result = readPacket(packet, ch);
		    switch(packet_result)
		    {
                case PACKET_ING: break;
		        case PACKET_SUCCESS:
		        {
                    reset();
		            return true;
		        }
		        case PACKET_FAILED:
		        {
                    reset();
		            return false;
		        }
		    }
		}
	}

	printf("[DTFCLidarPacketReceiver] It should not come to here!\n");
	return false;
}

/***********************************************************************************
Function:     readPacket
Description:  Read packet, if it return ture, it means read complete packet or enter
              erro state
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
DTFCLidarPacketReceiver::TPacketResult DTFCLidarPacketReceiver::readPacket(DTFCLidarPacket *packet, u8 ch)
{
    TPacketResult packet_result = PACKET_ING;
    switch(m_state)
    {
        case STATE_HEADER1: packet_result = processStateHeader1(packet, ch); break;
        case STATE_HEADER2: packet_result = processStateHeader2(packet, ch); break;
        case STATE_LENGHT: packet_result =  processStateLength(packet, ch); break;
        case STATE_ACQUIRE_DATA: packet_result = processStateAcquireData(packet, ch); break;
        default:
            printf("[DTFCLidarPacketReceiver] Enter erro state %d!\n", m_state);
        break;
    }
    return packet_result;
}

/***********************************************************************************
Function:     processStateHeader1
Description:  Proces state header1
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
DTFCLidarPacketReceiver::TPacketResult DTFCLidarPacketReceiver::processStateHeader1(DTFCLidarPacket *packet, u8 ch)
{
    if(ch == LIDAR_PACEKT_HEADER1)
    {
        packet->reset();
        packet->pushBack(ch);
        m_state = STATE_HEADER2;
        m_count_down.setTime(m_params.packet_wait_time_ms);

    }
    else
    {

        if(GetSN_ING == GetSNFlag())
        {
            return PACKET_FAILED;
        }
    }
    
    return PACKET_ING;
}

/***********************************************************************************
Function:     processStateHeader2
Description:  Proces state header2
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
DTFCLidarPacketReceiver::TPacketResult DTFCLidarPacketReceiver::processStateHeader2(DTFCLidarPacket *packet, u8 ch)
{
    if(ch == LIDAR_PACEKT_HEADER2)
    {
        packet->pushBack(ch);
        m_state = STATE_LENGHT;
        if(packet->m_error_Data_Wrong)
            packet->m_error_Data_Wrong = false;
    }
    else
    {
        if(!packet->m_error_Data_Wrong)
        {
            packet->m_error_Data_Wrong = true;
            packet->Timeout_Data_Wrong.setTime((double)m_params.Data_Wrong_time_out);
            return PACKET_FAILED;
        }
        reset();
        if(GetSN_ING == GetSNFlag())
            return PACKET_FAILED;
        
        printf("[DTFCLidarPacketReceiver] Find erro header2 0x%x!\n", ch);
    }
    return PACKET_ING;
}

/***********************************************************************************
Function:     processStateLength
Description:  Process state length
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
DTFCLidarPacketReceiver::TPacketResult DTFCLidarPacketReceiver::processStateLength(DTFCLidarPacket *packet, u8 ch)
{
#if 1
    /* Limit packet length */
    if(ch < 6 || ch > 250)
    {
        reset();
        if(GetSN_ING == GetSNFlag())
            return PACKET_FAILED;
        printf("[DTFCLidarPacketReceiver] Find erro length is 0x%x!\n", (ch));
        return PACKET_ING;
    }
#endif
    packet->pushBack(ch);

    // Add 2bytes for receive CRC16, sub 3 bytes for header(1bytes) and length(2bytes)
    m_packet_length = (int)ch + 2 - 3;

    m_state = STATE_ACQUIRE_DATA;

    return PACKET_ING;
}

/***********************************************************************************
Function:     processStateAcquireData
Description:  Process state acquire data
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
DTFCLidarPacketReceiver::TPacketResult DTFCLidarPacketReceiver::processStateAcquireData(DTFCLidarPacket *packet, u8 ch)
{
    m_actual_count++;
    packet->pushBack(ch);
    if(m_actual_count == m_packet_length)
    {
        reset();
		
        if(packet->verifyCheckSum(packet->getPrototypeCode()))
        {
            if(packet->m_error_crc)
            {
                packet->m_error_crc = false;
            }
            return PACKET_SUCCESS;
        }
        else
        {
            if(!packet->m_error_crc)
            {
                packet->m_error_crc = true;
                packet->Timeout_CRC.setTime((double)m_params.CRC_max_time_ms);
            }
            return PACKET_FAILED;
        }
    }
    return PACKET_ING;
}

/***********************************************************************************
Function:     reset
Description:  Reset
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void DTFCLidarPacketReceiver::reset()
{
    m_state = STATE_HEADER1;
    m_actual_count = 0;
    m_packet_length = 0;
}
/***********************************************************************************
Function:     SetSNFlag
Description:  Set SN Flag
Input:        SNState
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void DTFCLidarPacketReceiver::SetSNFlag(SNState state)
{
        m_SNState = state;

}
/***********************************************************************************
Function:     GetSNFlag
Description:  Get SN Flag
Input:        None
Output:       SNState
Return:       None
Others:       None
***********************************************************************************/
DTFCLidarPacketReceiver::SNState DTFCLidarPacketReceiver::GetSNFlag()
{
        return m_SNState;

}