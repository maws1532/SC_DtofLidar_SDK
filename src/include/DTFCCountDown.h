/**********************************************************************************
File name:	  DTFCCountDown.h
Author:       Kimbo
Version:      V1.6.0
Date:	 	  2016-7-12
Description:  Count down class
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/

#ifndef EVEREST_LIDAR_DTFCCOUNTDOWN_H_
#define EVEREST_LIDAR_DTFCCOUNTDOWN_H_

/******************************* Current libs includes ****************************/
#include "DTFCTime.h"

/********************************** System includes *******************************/
#include <string>


namespace dtfeverest
{
    namespace dtfhwdrivers
    {
        class DTFCCountDown
        {
            public:
                /* DTFCCountDown */
                DTFCCountDown();

                /* DTFCCountDown */
                DTFCCountDown(double time_ms);

                /* Destructor */
                ~DTFCCountDown();

                /* Set time */
                void setTime(double time_ms);

                /* Is end */
                bool isEnd() const;

                /* Get left time, unit is ms */
                double getLeftTime() const;

                /* Get left time, unit is ms */
                double getLeftMsTime() const { return getLeftTime() * 1000.0;}

                /* Get Input time */
                double getInputTime() const { return m_time_ms;}

            private:
                TTimeStamp                  m_end_time;
                bool                        m_end_flag;
                double                      m_time_ms;
        };
    }
}

#endif


