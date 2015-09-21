#ifndef _SHIPHON_IO__H_
#define _SHIPHON_IO__H_

#include <iostream>
#include <std_msgs/String.h>
#include <stdio.h>
#include <pthread.h>
#include <string>
#include <cstring>
#include <unistd.h>
#include <time.h>
#include<sys/socket.h>
#include <netinet/in.h>
#include<arpa/inet.h>
#include <ros/ros.h>

#include "../Shiphon_IO/ShareUtil.h"

//const unsigned short   	MsgBufMax = 256;
#define SHIPHONE_BUFF_LEN 1000
#define MAX_SHIPHONE_OFF_COUNT 20

enum E_SENSOR_STATUS {E_SENSOR_STATUS_OFF, E_SENSOR_STATUS_INIT, E_SENSOR_STATUS_ON } ;

const double   			_epsilon_E9 = 1E9;

const double   			_epsilon_E6 = 1E6;
const double   			_epsilon_E3 = 1E3;


#pragma pack (1)

using namespace std;


/* VALIDITY_WORD_EGI_STRUCT */
typedef struct {
	unsigned char                                   GPS_Data_Not_Valid_Egi; //
	unsigned char                                   Temperature_Fail_Egi; //
	unsigned char                                   NA_Egi; //
	unsigned char                                   NA1_Egi; //
	unsigned char                                   NA2_Egi; //
	unsigned char                                   NA3_Egi; //
	unsigned char                                   NA4_Egi; //
	unsigned char                                   NA5_Egi; //
	unsigned char                                   Severe_INS_Fail_Egi; // when set navigation data is not valid
	unsigned char                                   External_Voltage_Fail_Egi; //
	unsigned char                                   Antenna_Fail_Egi; //
	unsigned char                                   ONE_PPS_Fail_Egi; //
	unsigned char                                   IMU_FAIL_Egi; // when set, navigation data is not valid
	unsigned char                                   Gps_Fail_Egi; // includes:1 PPS fail,antenna fail,GPS_Data not Valid
	unsigned char                                   ODO_fail_Egi; //
	unsigned char                                   INS_fail_Egi; // INS fails includes:NAV or SI failure, communication failure
} VALIDITY_WORD_EGI_STRUCT;

/* EIGHT_SPARE_BYTE_ARRAY */
typedef char EIGHT_SPARE_BYTE_ARRAY[8];

/* S2_STATUS_STRUCT */
typedef struct {
	unsigned char                                   Zero_Velocity_update_in_progress; //
	unsigned char                                   NA_Egi; //
	unsigned char                                   INS_or_ODO_or_GPS_malfunction; //
	unsigned char                                   Travel_Lock_status; //
	unsigned char                                   NA1_Egi; //
	unsigned char                                   INS_in_SH_align_mode; //
	unsigned char                                   INS_alert; //
	unsigned char                                   INS_in_align_on_the_move; //
} S2_STATUS_STRUCT;

/* S1_STATUS_STRUCT */
typedef struct {
	unsigned char                                   INS_in_startup_mode; //
	unsigned char                                   Startup_Complete; //
	unsigned char                                   INS_normal_or_SH_or_move_align; //
	unsigned char                                   INS_in_survey_mode; // set when INS enters survey mode
	unsigned char                                   INS_in_Exclusive_ZUPT_mode; //
	unsigned char                                   Zero_velocity_stop_request; // when set ,INS determines it needs a zero velocity stop
	unsigned char                                   Position_update_request; // when set, INS determines it needs a position update
	unsigned char                                   INS_ready_for_Align_on_the_move; //
} S1_STATUS_STRUCT;

/* S4_STATUS_STRUCT */
typedef struct {
	unsigned char                                   SH_Shutdown_test_completed; //
	unsigned char                                   Integrated_mode_of_operation; //
	unsigned char                                   INS_SH_Shutdown_failed; //
	unsigned char                                   INS_Shutdown_complete_successful; //
	unsigned char                                   NA_Egi; //
	unsigned char                                   INS_in_standby_mode; //
	unsigned char                                   INS_wait_for_gps; //
	unsigned char                                   NA1_Egi; //
} S4_STATUS_STRUCT;

/* S3_STATUS_STRUCT */
typedef struct {
	unsigned char                                   Odo_calibration_in_process; //
	unsigned char                                   Odo_damping_in_process; //
	unsigned char                                   Odo_calibration_completed; //
	unsigned char                                   NA_Egi; //
	unsigned char                                   NA1_Egi; //
	unsigned char                                   INS_in_motion; //
	unsigned char                                   Orientation_attitude_data_valid; //
	unsigned char                                   Degraded_Survey; //
} S3_STATUS_STRUCT;

/* STATUS_WORD_STRUCT */
typedef struct {
	S2_STATUS_STRUCT                                s2_status; //
	S1_STATUS_STRUCT                                s1_status; //
	S4_STATUS_STRUCT                                s4_status; //
	S3_STATUS_STRUCT                                s3_status; //
} STATUS_WORD_STRUCT;

/* ALERT_DATA_D2_STRUCT */
typedef struct {
	unsigned char                                   Bit_Reserved; //
	unsigned char                                   Bit_Reserved1; //
	unsigned char                                   SH_Attitude_not_good; //
	unsigned char                                   Unable_to_complete_align; //
	unsigned char                                   Align_interrupt; //
	unsigned char                                   Bit_Reserved2; //
	unsigned char                                   Zero_Velocity_update_interrupt; //
	unsigned char                                   Bit_Reserved3; //
} ALERT_DATA_D2_STRUCT;

/* ALERT_WORD_1_STRUCT */
typedef struct {
	ALERT_DATA_D2_STRUCT                            Alert_D2; //
} ALERT_WORD_1_STRUCT;

/* ALERT_DATA_D3_STRUCT */
typedef struct {
	unsigned char                                   Gyro_Y_Anode_Two_Temperature; // Gyro Y anode two temperature is out of range
	unsigned char                                   Gyro_Z_Anode_One_Temperature; // Gyro Z anode one temperature is out of range
	unsigned char                                   Gyro_Z_Anode_Two_Temperature; // Gyro Z anode two temperature is out of range
	unsigned char                                   Gyro_X_Dither_Temperature; // Gyro X dither temperature is out of range
	unsigned char                                   Gyro_X_Cathode_Temperature; // Gyro X cathode temperature is out of range
	unsigned char                                   Gyro_Y_Dither_Temperature; // Gyro Y dither temperature is out of range
	unsigned char                                   Gyro_Y_Cathode_Temperature; // Gyro Y cathode temperature is out of range
	unsigned char                                   Gyro_Z_Dither_Temperature; // Gyro Z dither temperature is out of range
} ALERT_DATA_D3_STRUCT;

/* ALERT_DATA_D4_STRUCT */
typedef struct {
	unsigned char                                   Bit_Reserved; //
	unsigned char                                   Bit_Reserved1; //
	unsigned char                                   GPS_position_conflict; //
	unsigned char                                   Altitude_update_rejected; //
	unsigned char                                   Horizontal_PositionUpdate_reject; //
	unsigned char                                   BITS_3_Reserved; //
} ALERT_DATA_D4_STRUCT;

/* ALERT_WORD_2_STRUCT */
typedef struct {
	ALERT_DATA_D3_STRUCT                            Alert_D3; //
	ALERT_DATA_D4_STRUCT                            Alert_D4; //
} ALERT_WORD_2_STRUCT;

/* ALERT_DATA_D5_STRUCT */
typedef struct {
	unsigned char                                   Invalid_update_or_data_request; //
	unsigned char                                   Invalid_mode_request; //
	unsigned char                                   BITS_3_Reserved; //
	unsigned char                                   Invalid_data_received; //
	unsigned char                                   Bit_Reserved; //
	unsigned char                                   Bit_Reserved1; //
} ALERT_DATA_D5_STRUCT;

/* ALERT_DATA_D6_STRUCT */
typedef struct {
	unsigned char                                   BITS_3_Reserved; //
	unsigned char                                   GPS_data_unusable; //
	unsigned char                                   Bit_Reserved; //
	unsigned char                                   Bit_Reserved1; //
	unsigned char                                   Bit_Reserved2; //
	unsigned char                                   ODO_data_unusable; //
} ALERT_DATA_D6_STRUCT;

/* ALERT_WORD_3_STRUCT */
typedef struct {
	ALERT_DATA_D5_STRUCT                            Alert_D5; //
	ALERT_DATA_D6_STRUCT                            Alert_D6; //
} ALERT_WORD_3_STRUCT;


/* Periodic_100Hz_Message */
/*  */
typedef struct {
	unsigned short                                  SOM; // syncronize
	unsigned char                					Message_ID_Accepted_From_EGI; //
	unsigned short                                  Block_length; //
	double                                          INS_time_of_Nav_data; //
	double                                          LAT_Egi; //
	double                                          LONG_Egi; //
	double                                          Altitude_MSL_EGI; //
	double                                          Pitch_PD_Egi; //
	double                                          Roll_PD_Egi; //
	double                                          Azimuth_PD_geographic; //
	double                                          Roll_rate_X_PD_Egi; //
	double                                          Pitch_rate_Y_PD_Egi; //
	double                                          Azimuth_rate_Z_PD_Egi; //
	double                                          Acc_X_Egi; //
	double                                          Acc_Y_Egi; //
	double                                          Acc_Z_Egi; //
	double                                          Velocity_north_Egi; //
	double                                          Velocity_East_Egi; //
	double                                          Velocity_down_Egi; //
	short                                           Alt_correction_Egi; //
	VALIDITY_WORD_EGI_STRUCT                        Validity_Word; // when a bit is set, it indicates that the function is valid
	unsigned short                                  Input_Message_Number_Echo_Egi; //
	EIGHT_SPARE_BYTE_ARRAY                          Eight_byte_Spare; //
	unsigned short                                  Checksum_Egi; //
} PHS_PERIODIC100HZMESSAGE;

/* Periodic_1Hz_Message */
/*  */
typedef struct {
	unsigned short                                  SOM; // syncronize
	unsigned char					                Message_ID_Accepted_From_EGI; //
	unsigned short                                  Block_length; //
	double                                          INS_time_of_Nav_data; //
	double                                          GPS_Time_Egi; //
	unsigned long                                   INS_Distance_Traveled; //
	double                                          GPS_Lat_Egi; //
	double                                          GPS_Long_Egi; //
	double                                          GPS_Altitude_Egi; //
	unsigned short                                  Alignment_Countdown; //
	STATUS_WORD_STRUCT                              Status_word; //
	ALERT_WORD_1_STRUCT                             Alert_word_1; //
	ALERT_WORD_2_STRUCT                             Alert_word_2; //
	ALERT_WORD_3_STRUCT                             Alert_word_3; //
	float                                           Azimuth_Error_RMS; //
	float                                           Velocity_error_RMS; //
	unsigned short                                  INS_Horizontal_Position_Error; //
	unsigned short                                  INS_Altitude_Error; //
	float                                           Roll_Error_RMS; //
	float                                           Pitch_Error_RMS; //
	unsigned char                                   Number_of_Satellites; //
	unsigned char 									Figure_of_Merit;
	// E_FOM                                           Figure_of_Merit; //
	unsigned short                                  Week; //
	unsigned short                                  GPS_UTC_offset; //
	unsigned short                                  Bytes_2_Reserved; //
	unsigned short                                  Checksum_Egi; //
} PHS_PERIODIC1HZMESSAGE;

class Shiphon_Ctrl {

public:
	Shiphon_Ctrl ();
	~ Shiphon_Ctrl ();

private:
       // Flag for the Receive Thread
	   unsigned char	m_IsTerminateThread;

	   // UDP IP, local port number and remote port number
	   char 				udpIP[16];
	   int  				udpLP, udpRP;

	   int 			        socketFd;
	   struct sockaddr_in   si_Remote;
	   struct sockaddr_in   si_Local;


	   // Control counters
	   unsigned long		rxCount;
	   unsigned short		rxSeqNumber;
	   unsigned int			m_iOffcount;


	   // Receive control
	   unsigned char		rxBuf[SHIPHONE_BUFF_LEN];
	   unsigned char		rxBufShift;
	   unsigned short		nRcvDataInBuff;

	   //TimeTags for synchronization control (TBD)
	   double       		m_rxTT;
	   double       		m_currTT;

	   bool					m_ShiphonConnectionActive;
	   E_SENSOR_STATUS      m_eShiphone100Status;

	   unsigned int 		no_connection_counter;

	   PHS_PERIODIC100HZMESSAGE Phs_periodic100hzmessage;
	   PHS_PERIODIC1HZMESSAGE   Phs_periodic1hzmessage;


public:
	   bool					Init (char *addr, unsigned int lPortID, unsigned int rPortID);
	   bool					Reset ();
	   int		    		ReceiveData ();

	   bool					PeriodicActivity ();
	   bool					CommConnect ();
	   bool        			CommThreadCreate ();
	   bool        			TimerCreate ();

	   void        			SetCurrentTimeTag ();
	   double      			GetCurrentTimeTag () { return m_currTT; }
	   double      			GetLastRxTimeTag () { return m_rxTT; }
	   void        			ResetLocalTimeTag (double &l_tt);

	   bool					GetShiphonActived ();

	   unsigned short		ParseData ();
	   bool					MsgVerification (unsigned short msgId);
	   void					CheckConnectionActive (bool valid, unsigned short msgCount);

private:
	  void					RxBufferInit ();
      static void *			Comm4RcvThread (void *pThis);
      void	    			ThreadFunc ();

      void        			SetReceiveTimeTag ();

      int 					Periodic_100Hz_Message_CONVERT_TO_PH(PHS_PERIODIC100HZMESSAGE* phsPtr, unsigned char* intfPtr);
      int 					Periodic_1Hz_Message_CONVERT_TO_PH(PHS_PERIODIC1HZMESSAGE* phsPtr, unsigned char* intfPtr);

};

#endif
