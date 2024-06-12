/*********************************************************************************
File name:	  DTFC3iroboticsLidar.h
Author:       Kimbo
Version:      V1.7.1
Date:	 	  2017-02-03
Description:  3irobotics lidar sdk
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/

#ifndef EVEREST_LIDAR_DTFC3IROBOTICSLIDAR_H
#define EVEREST_LIDAR_DTFC3IROBOTICSLIDAR_H

/******************************* Current libs includes ****************************/
#include "DTFCLidarPacket.h"
#include "DTFCLidarUnpacket.h"
#include "DTFCLidarPacketReceiver.h"
#include "DTFCSerialConnection.h"
#include "DTFCSimulateSerial.h"

/******************************* System libs includes *****************************/
#include <vector>


/******************************* Other libs includes ******************************/

namespace dtfeverest
{
	namespace dtfhwdrivers
	{
	    struct TLidarScan
	    {
	        TLidarScan() : angle(), distance(), signal(){ }

	        void insert(TLidarScan &scan)
	        {
                this->angle.insert(this->angle.end(), scan.angle.begin(), scan.angle.end());
                this->distance.insert(this->distance.end(), scan.distance.begin(), scan.distance.end());
                this->signal.insert(this->signal.end(), scan.signal.begin(), scan.signal.end());
	        }

	        void clear()
	        {
	            angle.clear();
	            distance.clear();
	            signal.clear();
	        }

            size_t getSize() {return angle.size();}

	        std::vector<float> angle;
	        std::vector<float> distance;
	        std::vector<int>   signal;

	    };
        struct ErrorTimeOut
        {
            bool speedflag;
            bool Shieldflag;
        };

        enum TLidarGrabResult
        {
            LIDAR_GRAB_ING = 0,
            LIDAR_GRAB_SUCESS,
            LIDAR_GRAB_ERRO,
            LIDAR_GRAB_ELSE
        };

        enum TLidarCommandID
        {
            I3LIDAR_DISTANCE = 0xA9,
            I3LIDAR_HEALTH   = 0xAB,
            I3LIDAR_LIDAR_SPEED  = 0xAE,
            I3LIDAR_NEW_DISTANCE = 0xAD
        };
        enum TLidarVersion
        {
            LIDAR_NONE = 0,
            LIDAR_2_6_K, 
            LIDAR_2_1_K,
            TS12D_LIDAR_3_9_K,
        };

		class DTFC3iroboticsLidar
		{
            public:
                enum TGrabScanState
                {
                    GRAB_SCAN_FIRST = 0,
                    GRAB_SCAN_ELSE_DATA
                };
                enum PWMPolarityState
                {
                    INVERSED = 0,
                    NORMAL
                };

                /* Constructor */
                DTFC3iroboticsLidar();

                /* Destructor */
                ~DTFC3iroboticsLidar();

                /* Set device connect */
                bool initilize(DTFCDeviceConnection *device_connect);

                /* Get scan data */
                TLidarGrabResult getScanData();

                /* Get Lidar Error */
                TLidarError getLidarError() { return m_packet.m_lidar_erro; }

                /* Get lidar scan */
                TLidarScan& getLidarScan() { return m_lidar_scan; }

                /* Set data with signal*/
                void setDataWithSignal(bool data_with_signal) {m_data_with_signal = data_with_signal;}

                /* Return true if receive lidar speed */
                bool isReceiveLidarSpeed() {bool flag = m_receive_lidar_speed; m_receive_lidar_speed = false; return flag;}

                /* Get Lidar current speed */
                double getLidarCurrentSpeed() {return m_current_lidar_speed;}

                /*get Lidar expect speed*/
                double GetLidarExpectspeed() {return Expectspeed;}

                /* Enable log when receive timer overs */
                void enableLogWhenReceiveTimeOvers(bool state) {m_receiver.enableLogWhenReceiveTimeOvers(state);}

                /* pwn Write */
                void PwmWriteData(const char *file_name, int64_t data);

                /* pwd Write */
                void PwmWriteData(const char *file_name, const char *data);

                /* pwm Init */
                void PwmInit(PWMPolarityState state);

                /* Control Lidar Speed */
                void controlLidarSpeed();

                /*get pwm*/
                int32_t GetPwm();

                /*get max pwm limit*/
                u8 GetPWMMaxLimit();

                /*set max pwm limit*/
                int SetPWMMaxLimit(u8 limit);
                
                /*Set Lidar Expect speed*/
                int SetLidarExpectSpeed(double speed);

                /*set scan error time*/
                int ScanErrTimeOut(DTFCLidarPacket *packet);

                /*control Lidar Pause*/
                void ControlLidarPause();

                /*connect Liar Start*/
                void ConnectLidarStart();

                /*Set Device Node number*/
                void SetDeviceNodeID(u8 num);

                /*Get Device Node number*/
                u8 GetDeviceNodeID();

                /*Get Lidar SN code*/
                std::string GetLidarSNCode();

                /*get lidar Firm version*/
                std::string GetLidarFirmwareVersion();

                /*Get Lidar Hardware Version*/
                std::string GetLidarHardwareVersion();

                /*Get Lidar Type*/
                std::string GetLidarType();

                /*get lidar information*/
                bool GetDeviceInfo();

                /*set pwm polarity*/
                bool SetPwmpolarity(PWMPolarityState state);

                /*get lidar vesion*/
                TLidarVersion GetLidarversion();

                /*Setlidar versio*/
                void SetLidarversion(TLidarVersion ver);

                /*Get SDK version*/
                std::string  GetSDKVersion(); 

            private:
                /* Analysis packet */
                TLidarGrabResult analysisPacket(DTFCLidarPacket &lidar_packet);

                /* Analysis tooth scan */
                TLidarGrabResult analysisToothScan(DTFCLidarPacket &lidar_packet);

                /* Analysis new tooth scan */
                TLidarGrabResult analysisNewToothScan(DTFCLidarPacket &lidar_packet);

                /* Analysis health info */
                TLidarGrabResult analysisHealthInfo(DTFCLidarPacket &lidar_packet);

                /* Analysis lidar speed */
                TLidarGrabResult analysisLidarSpeed(DTFCLidarPacket &lidar_packet);

                /* Reset scan grab */
                void resetScanGrab();

                /* Combine scan */
                TLidarGrabResult combineScan(TToothScan &tooth_scan);

                /* Return true if tooth scan is first */
                bool isFirstScan(TToothScan &tooth_scan);

                /* Grab first scan */
                TLidarGrabResult grabFirstScan(TToothScan &tooth_scan);

                /* Change tooth scan to lidar scan */
                void toothScan2LidarScan(TToothScan &tooth_scan, TLidarScan &lidar_scan);

                /* Control Lidar PWM */
                void controlLidarPWM(int32_t percent);//yuy

                /**/
                void LidarTranform(float &Angle, float &Dis);



                /* Params */
                struct TParams
                {
                    /* Constructor */
                    TParams()
                    {
                        packet_wait_time_ms = 100;
                        tooth_number = 16;
                        scan_time_out_ms = 150;
                        speed_time_out = 3500;
                        Shield_time_out = 3500;
                        GetSN_time_out = 5000;
                    }

                    size_t packet_wait_time_ms;
                    size_t scan_time_out_ms;
                    size_t speed_time_out;
                    size_t Shield_time_out;
                    size_t GetSN_time_out;
                    int tooth_number;
                };
            public:
                TLidarScan              m_lidar_scan;
                ErrorTimeOut            Error_timeout;
                char pProInfopBuf[128];
                char SNCode[64];
                char SoftwareV[16];
                char HardwareV[16];
                char Lidartype[8];
                std::string Lds_str;
                TLidarVersion LidarV;
            private:
                DTFCDeviceConnection       *m_device_connect;
                DTFCLidarPacketReceiver    m_receiver;
                TParams                 m_params;
                //TLidarScan              m_lidar_scan;
                //TLidarError             m_lidar_erro;
                TGrabScanState          m_grab_scan_state;
                PWMPolarityState        m_pwm_polarity_state;
                int                     m_grab_scan_count;
                int                     m_Shield_count;
                float                   m_last_scan_angle;
                DTFCLidarPacket            m_packet;
                TToothScan              m_remainder_tooth_scan;
                bool                    m_remainder_flag;
                bool                    m_data_with_signal;

                bool                    m_receive_lidar_speed;
                double                  m_current_lidar_speed;
                double                  Expectspeed;

                DTFCCountDown              m_data_count_down;
                DTFCCountDown              m_speed_count_down;
                DTFCCountDown              m_stop_count_down;
                DTFCCountDown              m_Shield_count_down;
                DTFCCountDown              m_GetSNcount_down;
                //调整速度相关变量 
                double  error;
                double  last_error;
                double  error_sum;
                int32_t percent;
                bool    speedStableFlag;
                uint8_t countSpeed;
                u8 Node_num;
                u8 MaxPwm;
                std::string SDKVersion;

		};
	}
}

#endif


