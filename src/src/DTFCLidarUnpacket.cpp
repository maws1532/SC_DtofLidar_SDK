/*********************************************************************************
File name:	  DTFCLidarUnpacket.h
Author:       Kimbo
Version:      V1.7.1
Date:	 	  2017-02-06
Description:  3irobotics Lidar unpacket
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/

/********************************* File includes **********************************/
#include "DTFCLidarUnpacket.h"

/******************************* System libs includes *****************************/
#include <string.h>

/********************************** Name space ************************************/
using namespace dtfeverest;
using namespace dtfeverest::dtfhwdrivers;
/***********************************************************************************
Function:     DTFCLidarUnpacket
Description:  The constructor of DTFCLidarUnpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
DTFCLidarUnpacket::DTFCLidarUnpacket()
{
    
}

/***********************************************************************************
Function:     DTFCLidarUnpacket
Description:  The destructor of DTFCLidarUnpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
DTFCLidarUnpacket::~DTFCLidarUnpacket()
{

}
/***********************************************************************************
Function:     unpacketLidarScan
Description:  Lidar unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TToothScan DTFCLidarUnpacket::unpacketLidarScan(DTFCLidarPacket &packet)
{
    TToothScan tooth_scan;
    u16 length = packet.getParamLength();
    u8 *buffer = packet.getParamPtr();
    u16 tooth_angle = 0;
    u16 distance_number = 0;
    u16 distance = 0;

    // Get tooth angle, unit is 0.01 degree
    tooth_angle = DTFCLidarPacket::bufToUByte2(buffer);
    tooth_scan.angle = float(tooth_angle) / 100.0;

    // Get distance number
    // tooth_angle + distance1 + distance2 + ...
    //    2bytes       2bytes      2bytes  + ...
    //                      length - 2(tooth_angle_bytes)
    //  distance_number =   -----------------------------
    //                            2(distance_bytes)
    distance_number = (length - 2) / 2;
    tooth_scan.distance.resize(distance_number);
//    printf("[DTFCLidarUnpacket] angle %5.2f, distance_number %d !\n",tooth_scan.angle, distance_number);

    // Get distance, unit is 0.25mm
    for(size_t i = 0 ; i < distance_number; i++)
    {
        distance = DTFCLidarPacket::bufToUByte2(buffer + 2 * i + 2);

        tooth_scan.distance[i] = float(distance) / 4.0f / 1000.0f;
    }

    return tooth_scan;
}

/***********************************************************************************
Function:     unpacketLidarScan2
Description:  Lidar unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TToothScan DTFCLidarUnpacket::unpacketLidarScan2(DTFCLidarPacket &packet)
{
    TToothScan tooth_scan;
    u16 length = packet.getParamLength();
    u8 *buffer = packet.getParamPtr();
    u16 tooth_angle = 0;
    u16 distance_number = 0;
    u16 distance = 0;
    u8 signal = 0;

    // Get tooth angle, unit is 0.01 degree
    tooth_angle = DTFCLidarPacket::bufToUByte2(buffer);
    tooth_scan.angle = float(tooth_angle) / 100.0;

    // Get distance number
    // tooth_angle + distance1 + distance2 + ...
    //    2bytes       2bytes      2bytes  + ...
    //                      length - 2(tooth_angle_bytes)
    //  distance_number =   -----------------------------
    //                            2(distance_bytes)
    distance_number = (length - 2) / 3;
    tooth_scan.distance.resize(distance_number);
    tooth_scan.signal.resize(distance_number);
//    printf("[DTFCLidarUnpacket] distance_number %d! packe length %d\n", distance_number, length);

    // Get distance, unit is 0.25mm
//    packet.printHex();
    for(size_t i = 0 ; i < distance_number; i++)
    {
        signal = DTFCLidarPacket::bufToUByte((buffer+2) + 3*i);
        distance = DTFCLidarPacket::bufToUByte2((buffer+2) + 3*i + 1);
        tooth_scan.distance[i] = float(distance) / 4.0f / 1000.0f;
        tooth_scan.signal[i] = int(signal);
//        printf("[DTFCLidarUnpacket]sigal %d! distance %5.2f\n", tooth_scan.signal[i], tooth_scan.distance[i]);
    }
    return tooth_scan;
}

/***********************************************************************************
Function:     unpacketNewLidarScanNoSingal
Description:  Lidar unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TToothScan DTFCLidarUnpacket::unpacketNewLidarScanNoSingal(DTFCLidarPacket &packet)
{
    TToothScan tooth_scan;
    int head_ptr_offset = 0;
    u16 length = packet.getParamLength();
    u8 *buffer = packet.getParamPtr();
    u16 tooth_angle = 0;
    u8  lidar_speed = 0;
    s16 lidar_offset_angle = 0;
    u16 distance_number = 0;
    u16 distance = 0;

    // Get tooth angle, unit is 0.01 degree
    lidar_speed = DTFCLidarPacket::bufToUByte(buffer + head_ptr_offset);                   head_ptr_offset += 1;
    lidar_offset_angle = DTFCLidarPacket::bufToByte2(buffer + head_ptr_offset);            head_ptr_offset += 2;
    tooth_angle = DTFCLidarPacket::bufToUByte2(buffer + head_ptr_offset);                  head_ptr_offset += 2;

    tooth_scan.angle = float(tooth_angle) * 0.01f;
    tooth_scan.offset_valid = false;
    tooth_scan.offset_angle = lidar_offset_angle * 0.01f;
    tooth_scan.lidar_speed = lidar_speed * 0.05f;

    // Get distance number
    // tooth_angle + distance1 + distance2 + ...
    //    2bytes       2bytes      2bytes  + ...
    //                      length - head_data_bytes(5)
    //  distance_number =   -----------------------------
    //                            2(distance_bytes)
    distance_number = (length - head_ptr_offset) / 2;
    tooth_scan.distance.resize(distance_number);
//    printf("[DTFCLidarUnpacket] angle %5.2f, distance_number %d !\n",tooth_scan.angle, distance_number);

    // Get distance, unit is 0.25mm
    for(size_t i = 0 ; i < distance_number; i++)
    {
        distance = DTFCLidarPacket::bufToUByte2(buffer + 2 * i + head_ptr_offset);

        tooth_scan.distance[i] = float(distance) / 4.0f / 1000.0f;
    }

    return tooth_scan;
}

/***********************************************************************************
Function:     unpacketLidarScan2
Description:  Lidar unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TToothScan DTFCLidarUnpacket::unpacketNewLidarScanHasSingal(DTFCLidarPacket &packet)
{
    TToothScan tooth_scan;
    int head_ptr_offset = 0;
    u16 length = packet.getParamLength();
    u8 *buffer = packet.getParamPtr();
    u16 tooth_angle = 0;
    u8  lidar_speed = 0;
    s16 lidar_offset_angle = 0;
    u16 distance_number = 0;
    u16 distance = 0;
    u8 signal = 0;
    u16 tooth_angle_start = 0, tooth_angle_end = 0;
    int tmpnum = 0;
    // Get tooth angle, unit is 0.01 degree
    lidar_speed = DTFCLidarPacket::bufToUByte(buffer + head_ptr_offset);                   head_ptr_offset += 1;
    lidar_offset_angle = DTFCLidarPacket::bufToByte2(buffer + head_ptr_offset);            head_ptr_offset += 2;
    tooth_angle_start = DTFCLidarPacket::bufToUByte2(buffer + head_ptr_offset);            head_ptr_offset += 2;
    tooth_angle_end = DTFCLidarPacket::bufToUByte2(buffer + head_ptr_offset);            	head_ptr_offset += 2;

    tooth_scan.angle = float(tooth_angle_start) * 0.01f;
    tooth_scan.angleEnd = float(tooth_angle_end) * 0.01f;
    tooth_scan.offset_valid = false;
    tooth_scan.offset_angle = lidar_offset_angle * 0.01f;
    tooth_scan.lidar_speed = lidar_speed * 0.05f;


    // Get distance number
    // tooth_angle + distance1 + distance2 + ...
    //    2bytes       2bytes      2bytes  + ...
    //                      length - head_data_bytes(5)
    //  distance_number =   -----------------------------
    //                            2(distance_bytes)
    distance_number = (length - head_ptr_offset) / 3;
    tooth_scan.distance.resize(distance_number);
    tooth_scan.signal.resize(distance_number);
//    printf("[DTFCLidarUnpacket] tooth_scan.angle %5.2f tooth_scan.offset_angle %5.2f, tooth_scan.lidar_speed %5.2f, distance_number %d! packe length %d\n",
//              tooth_scan.angle, tooth_scan.offset_angle, tooth_scan.lidar_speed, distance_number, length);

    // Get distance, unit is 0.25mm
//    packet.printHex();
    for(size_t i = 0 ; i < distance_number; i++)
    {
        signal = DTFCLidarPacket::bufToUByte((buffer+ head_ptr_offset) + 3*i);
        distance = DTFCLidarPacket::bufToUByte2((buffer+ head_ptr_offset) + 3*i + 1);
        // if((distance < 6000)&&(distance > 0))
        // {
        //     tmpnum = (distance / 400) * signal * signal;
        //     signal = ((tmpnum / 100) > 255)?255:(tmpnum / 100);
        // }
        // else
        // {
        //     signal = 1;
        // }
        
        tooth_scan.distance[i] = float(distance) / 4.0f / 1000.0f;
        tooth_scan.signal[i] = (((signal/16)*(distance/100)+10)*5>255)?255:u8(((signal/16)*(distance/100)+10)*5);

        if(tooth_scan.distance[i] > 0.15)//15cm
        {
            tooth_scan.Shield_count++;
            //printf("Dis:%6.2f\n\r", tooth_scan.distance[i]);
        }
        //printf("[DTFCLidarUnpacket]sigal %d! distance %5.2f\n", tooth_scan.signal[i], tooth_scan.distance[i]);
    }
    return tooth_scan;
}

/***********************************************************************************
Function:     unpacketHealthInfo
Description:  Health info unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TLidarError DTFCLidarUnpacket::unpacketHealthInfo(DTFCLidarPacket &packet)
{
    u8 health_info = *(packet.getParamPtr());
    return TLidarError(health_info);
}

/***********************************************************************************
Function:     unpacketLidarSpeed
Description:  Health info unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
int DTFCLidarUnpacket::unpacketLidarSpeed(DTFCLidarPacket &packet)
{
    int lidar_speed = *(packet.getParamPtr());
    return lidar_speed;
}
/***********************************************************************************
Function:     unpacketLidarInformation
Description:  Health info unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
u8 *DTFCLidarUnpacket::unpacketLidarInformation(DTFCLidarPacket &packet)
{
    u8 *pTemp = packet.getParamPtr();

    return pTemp;
}
/***********************************************************************************
Function:     unpacketLidarversion
Description:  Health info unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
u8 DTFCLidarUnpacket::UnpackerLidarVersion(DTFCLidarPacket &packet)
{
    u8 ver = packet.getPrototypeCode();
    return ver;
}