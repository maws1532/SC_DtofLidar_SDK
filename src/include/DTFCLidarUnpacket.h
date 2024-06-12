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

#ifndef EVEREST_LIDAR_DTFCLIDARUNPACKET_H
#define EVEREST_LIDAR_DTFCLIDARUNPACKET_H

/******************************* Current libs includes ****************************/
#include "DTFCLidarPacket.h"

/******************************* System libs includes *****************************/
#include <vector>

/******************************* Other libs includes ******************************/

namespace dtfeverest
{
	namespace dtfhwdrivers
	{
	    struct TToothScan
	    {
            TToothScan() : offset_valid(false), Shield_count(0), offset_angle(0.0), lidar_speed(-1.0), angle(0.0), angleEnd(0.0), distance(), signal(){ }

            float getAngle() 	{ return angle; }
	        float getAngleEnd() { return angleEnd; }
            size_t getSize() 	{ return distance.size(); }

            bool                  offset_valid;
            int                  Shield_count;
            float                 offset_angle;     // unit: degree
            float                 lidar_speed;      // unit: lidar speed
            float                 angle;            // unit: degree start
	        float                 angleEnd;         // unit: degree end
            std::vector<float>    distance;         // unit: meter
            std::vector<int>      signal;           // range: 0 - 255
	    };

        

		class DTFCLidarUnpacket
		{
            public:

                /* Constructor */
                DTFCLidarUnpacket();

                /* Destructor */
                ~DTFCLidarUnpacket();

                /* Lidar unpacket */
                static TToothScan unpacketLidarScan(DTFCLidarPacket &packet);

                /* Lidar unpacket */
                static TToothScan unpacketLidarScan2(DTFCLidarPacket &packet);

                /* Lidar unpacket */
                static TToothScan unpacketNewLidarScanHasSingal(DTFCLidarPacket &packet);

                /* Lidar unpacket */
                static TToothScan unpacketNewLidarScanNoSingal(DTFCLidarPacket &packet);

                /* Health info unpacket */
                static TLidarError unpacketHealthInfo(DTFCLidarPacket &packet);

                /* Lidar speed */
                static int unpacketLidarSpeed(DTFCLidarPacket &packet);
                /* Lidar Information */
                static u8 *unpacketLidarInformation(DTFCLidarPacket &packet);

                /*get lidar version*/                
                static u8 UnpackerLidarVersion(DTFCLidarPacket &packet); 



		};
	}
}

#endif


