//============================================================================
// Name        : Shiphon_IO.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#include "../Shiphon_IO/Shiphon_IO.h"

#define BUFLEN	256

const double MILS_2_DEG = 0.05625; // (360/6400)

const double	ROS_CONTROL_REFRESH_TIME_MAX =	10.;
//const double	ROS_CONTROL_REFRESH_TIME_MAX =	0.3;


char kbKey = '\0';
char dbgKey = '\0';

struct timespec myTc;

Shiphon_Ctrl::Shiphon_Ctrl()
{
	m_ShiphonConnectionActive = false;
	rxSeqNumber = 0;
	socketFd = 0;
	m_iOffcount = 0;
	no_connection_counter = 0;

	clock_gettime(CLOCK_REALTIME, &myTc);
}


Shiphon_Ctrl::~Shiphon_Ctrl()
{
	sleep(1);
}

bool Shiphon_Ctrl::Init(char *addr, unsigned int lPortID, unsigned int rPortID)
{
	bool resVal = false;

	rxCount = 0;
	memset(rxBuf, 0, SHIPHONE_BUFF_LEN);
	rxBufShift = 0;
	nRcvDataInBuff = 0;
	m_rxTT = 0.;
	m_currTT = 0.;

	//bool IsOk = false;

	memcpy(udpIP, addr, strlen(addr));  // UDP IP "132.4.6.60"
	udpLP = lPortID; // 2010;
	udpRP = rPortID; // 4997;
	//char ch;

	memset (&Phs_periodic100hzmessage, 0, sizeof (PHS_PERIODIC100HZMESSAGE));
	memset (&Phs_periodic1hzmessage, 0, sizeof (PHS_PERIODIC1HZMESSAGE));


	printf("INIT --- IP = %s\n", udpIP);

	resVal = CommConnect();
	if (resVal)
		printf("Connection is OK\n");
	else
		printf("Connection was failed");
	sleep(0.1);

	if (resVal) {
		printf("before CommThreadCreate\n");
	    sleep(0.1);
	    CommThreadCreate();
	    printf("after CommThreadCreate\n");
	}

	return resVal;

}

bool Shiphon_Ctrl::Reset()
{
	// TBD
	return true;
}

void Shiphon_Ctrl::SetCurrentTimeTag ()
{
	clock_gettime(CLOCK_REALTIME, &myTc);
	m_currTT = myTc.tv_sec + myTc.tv_nsec / 1E9;

}

void Shiphon_Ctrl::ResetLocalTimeTag (double &l_tt)
{
	clock_gettime(CLOCK_REALTIME, &myTc);
	l_tt = myTc.tv_sec + myTc.tv_nsec / 1E9;

}


void Shiphon_Ctrl::SetReceiveTimeTag ()
{
	clock_gettime(CLOCK_REALTIME, &myTc);
	m_rxTT = myTc.tv_sec + myTc.tv_nsec / 1E9;

}

bool Shiphon_Ctrl::CommConnect() {

	int slen = sizeof(si_Remote);
	timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 1600;
	int retV;

	if ((socketFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		printf("Socket create error\n");
		// std::cout << "Socket create error";
		return false;
	}

	memset((char *) &si_Remote, 0, sizeof(si_Remote));
	si_Remote.sin_family = AF_INET;
	si_Remote.sin_port = htons(udpRP);
	//si_Remote.sin_addr.s_addr = inet_addr (udpIP);

	if (inet_aton(udpIP, &si_Remote.sin_addr) == 0) {
		//cout << "inet_aton() failed\n";
		printf("inet_aton() failed\n");
	}

	memset((char *) &si_Local, 0, sizeof(si_Local));
	si_Local.sin_family = AF_INET;
	si_Local.sin_port = htons(udpLP);
	si_Local.sin_addr.s_addr = htonl(INADDR_ANY);

	if (setsockopt(socketFd, SOL_SOCKET, SO_RCVTIMEO,
			reinterpret_cast<char*>(&tv), sizeof(timeval))) {
		printf("setsockopt() failed\n");
	}

	retV = bind(socketFd, (struct sockaddr *) &si_Local, sizeof(si_Local));
	if (retV == -1) {
		printf("bind error\n");
	}
	sleep(0.5);
//    printf ("socketFd = %d\n", socketFd);

	return true;
}

bool  Shiphon_Ctrl::GetShiphonActived ()
{
	const float dtTolerance = 10.;
	return (fabs(m_currTT - m_rxTT) < dtTolerance);
}



bool Shiphon_Ctrl::CommThreadCreate() {
	pthread_t t; // (&Comm4RcvThread);

	pthread_create(&t, NULL, &Comm4RcvThread, this);


	//t.join ();
	return true;
}

void * Shiphon_Ctrl::Comm4RcvThread(void * pParam) {

	Shiphon_Ctrl *myHandle = (Shiphon_Ctrl *) (pParam);

	printf("before ThreadFunc\n");
	myHandle->ThreadFunc();
	printf("after ThreadFunc\n");

}

void Shiphon_Ctrl::ThreadFunc() {
	unsigned char bufTmp[256] = { 0 };
	socklen_t slen = 0;
	//int slen = 0;
	int retVal = 0;
	//int err = 0;
	int errNum = 0;

	double currentTime;
	static double lastTime = 0.;
	double cutrentDT;

	static unsigned long loopCount = 0;

	unsigned char MsgID;

	while (!m_IsTerminateThread) {

		try {
			retVal = recvfrom(socketFd, bufTmp, SHIPHONE_BUFF_LEN, 0,
					(struct sockaddr *) &si_Remote, &slen);

		}
		catch (exception& err) {
//			printf("receive error #%d\n", (int)(err.what()));
			printf("receive error\n");
		}

 		 if (retVal > 0) {
			 clock_gettime(CLOCK_REALTIME, &myTc);
			 currentTime = myTc.tv_sec + myTc.tv_nsec / 1E9;
			 if (lastTime > 0.)
				  cutrentDT = currentTime - lastTime;

			 m_iOffcount = 0;
			 m_eShiphone100Status = E_SENSOR_STATUS_ON;
			 nRcvDataInBuff = 0; // TBD
			 memcpy(&(rxBuf[nRcvDataInBuff]), bufTmp, retVal);
			 nRcvDataInBuff += retVal;
			 MsgID = rxBuf[2];


			// if (dbgKey == 'r' || dbgKey == 'f') {
				//	lastTime = currentTime;
					//printf("[%.6f]Read ID: %d    Size: %d", currentTime, MsgID, nRcvDataInBuff);
/*					printf("[%.6f]Read: ", currentTime);
					for (int i = 0; i < retVal; i++) {
						printf("%02X   ", rxBuf[i]);
					}*/
				//	printf("\n");
			// }



			if (MsgID == 0x01) { // 100Hz, Message ID 01
				errNum = Periodic_100Hz_Message_CONVERT_TO_PH(&Phs_periodic100hzmessage, rxBuf);
				m_ShiphonConnectionActive = true;
				no_connection_counter = 0;
				loopCount++;
		    }
		//	else if (MsgID == 0x02) { // 1Hz,  Message ID 02

			//	errNum = Periodic_1Hz_Message_CONVERT_TO_PH(&Phs_periodic1hzmessage,  rxBuf);
					//setKeepAlive();
		//	}

			 slen = 0;
			 retVal = 0;
		 }
		 else {
			 if(no_connection_counter < 500) { // Reset data after EGI disconnected
			 	no_connection_counter++;
			 }
			 else  { // Reset data after EGI disconnected for several loops
			    //errNum = Periodic_1Hz_Message_CONVERT_TO_PH(&Phs_periodic1hzmessage,  m_sBuffSH2);
			 	m_eShiphone100Status = E_SENSOR_STATUS_OFF;
			 	m_ShiphonConnectionActive = false;
			 }
		 }

		 if (m_ShiphonConnectionActive && loopCount % 100 == 1) {
			 printf ("Lat: %.6f   Lon: %.6f   Alt: %.2f\n",
					 Phs_periodic100hzmessage.LAT_Egi * 180.,
					 Phs_periodic100hzmessage.LONG_Egi * 180.,
					 Phs_periodic100hzmessage.Altitude_MSL_EGI);

			 printf ("Pitch: %.2f   Roll: %.2f   Azim: %.2f\n",
					 Phs_periodic100hzmessage.Pitch_PD_Egi * MILS_2_DEG,
					 Phs_periodic100hzmessage.Roll_PD_Egi * MILS_2_DEG,
					 Phs_periodic100hzmessage.Azimuth_PD_geographic * MILS_2_DEG);

			 printf ("PitchRate: %.2f   RollRate: %.2f   AzimRate: %.2f\n",
					 Phs_periodic100hzmessage.Pitch_rate_Y_PD_Egi * MILS_2_DEG,
					 Phs_periodic100hzmessage.Roll_rate_X_PD_Egi * MILS_2_DEG,
					 Phs_periodic100hzmessage.Azimuth_rate_Z_PD_Egi * MILS_2_DEG);

			 printf ("VelEast: %.2f   VelNorth: %.2f   VelDown: %.2f\n",
					 Phs_periodic100hzmessage.Velocity_East_Egi,
					 Phs_periodic100hzmessage.Velocity_north_Egi,
					 Phs_periodic100hzmessage.Velocity_down_Egi);

			 printf ("Acc_X_Egi: %.2f   Acc_Y_Egi: %.2f   Acc_Z_Egi: %.2f\n\n",
					 Phs_periodic100hzmessage.Acc_X_Egi,
					 Phs_periodic100hzmessage.Acc_Y_Egi,
					 Phs_periodic100hzmessage.Acc_Z_Egi);
		 }
		 sleep (0.001);

	} // while
	printf("Comm4RcvThread was therminated\n");

}


// Periodic_1Hz_Message
int Shiphon_Ctrl::Periodic_1Hz_Message_CONVERT_TO_PH(PHS_PERIODIC1HZMESSAGE* phsPtr, unsigned char* dataPtr)
{
	// init interface error flag
	//Periodic_1Hz_Message_ERR_FLG = 0;
	int 				errFlg = 0;
	unsigned char		ucTmp;
	unsigned short 		usTmp;
	unsigned int 		uiTmp;
	unsigned long 		ulTmp;
	short				sTmp;
	int					iTmp;
	long				lTmp;
	float				fTmp;
	double				dTmp;

	unsigned char* intfPtr = NULL;

	intfPtr = dataPtr;


	// SOM
	usTmp = (unsigned short)(GetShort(intfPtr, true));
	if (usTmp > 42405)
		errFlg += 1;
	else
		phsPtr->SOM = usTmp;

	*intfPtr += 2;

	// Message_ID_Accepted_From_EGI
	ucTmp = *intfPtr;
	if (ucTmp > 14)
		errFlg += 1;
	else
		phsPtr->Message_ID_Accepted_From_EGI = ucTmp;
	*intfPtr++;


	// Block_length
	ucTmp = *intfPtr;
	if (ucTmp > 60)
		errFlg += 1;
	else
		phsPtr->Block_length = ucTmp;
	*intfPtr++;


	// INS_time_of_Nav_data
	ulTmp = (unsigned long)(GetLong(intfPtr, true));
	dTmp =  ulTmp * ((double)((((1048576.0) - (0.0)) / ((4294967295.0) - (0.0)))));
	phsPtr->INS_time_of_Nav_data = (double)(dTmp);
	*intfPtr+=4;

	// GPS_Time_Egi
	ulTmp =  (unsigned long)(GetLong(intfPtr, true));
	phsPtr->GPS_Time_Egi = (double)ulTmp;
	*intfPtr+=4;

	// INS_Distance_Traveled
	ulTmp =  (unsigned long)(GetLong(intfPtr, true));
	phsPtr->INS_Distance_Traveled = ulTmp;
	*intfPtr+=4;

	// GPS_Lat_Egi
	ulTmp =  (unsigned long)(GetLong(intfPtr, true));
	dTmp = ulTmp * ((double)((((1.0) - (-1.0)) / ((2147483647.0) - (-2147483648.0))))) + ((double)((-1.0 - (-2147483648.0 * (((1.0) - (-1.0)) / ((2147483647.0) - (-2147483648.0)))))));
	phsPtr->GPS_Lat_Egi = dTmp;
	*intfPtr+=4;

	// GPS_Long_Egi
	ulTmp =  (unsigned long)(GetLong(intfPtr, true));
	dTmp = ulTmp * ((double)((((1.0) - (-1.0)) / ((2147483647.0) - (-2147483648.0))))) + ((double)((-1.0 - (-2147483648.0 * (((1.0) - (-1.0)) / ((2147483647.0) - (-2147483648.0)))))));
	phsPtr->GPS_Long_Egi = dTmp;
	*intfPtr+=4;

	// GPS_Altitude_Egi
	sTmp = (short)(GetShort(intfPtr, true));
	if ((sTmp < -16384) || (sTmp > 16383))
		errFlg += 1;
	else
		phsPtr->GPS_Altitude_Egi = (double)(sTmp);
	*intfPtr+=2;

	// Alignment_Countdown
	usTmp = (unsigned short)(GetShort(intfPtr, true));
	if (usTmp > 900)
		errFlg += 1;
	else
		phsPtr->Alignment_Countdown = usTmp;
	*intfPtr+=2;

	// Structure - STATUS_WORD_STRUCT
	// Bits Structure - S2_STATUS_STRUCT
	ucTmp = *intfPtr;
	// Status_word.s2_status.Zero_Velocity_update_in_progress
	phsPtr->Status_word.s2_status.Zero_Velocity_update_in_progress = ((ucTmp & 0x80) >> 7);

	// Status_word.s2_status.NA_Egi
	phsPtr->Status_word.s2_status.NA_Egi = ((ucTmp & 0x40) >> 6);

	// Status_word.s2_status.INS_or_ODO_or_GPS_malfunction
	phsPtr->Status_word.s2_status.INS_or_ODO_or_GPS_malfunction = ((ucTmp & 0x20) >> 5);

	// Status_word.s2_status.Travel_Lock_status
	phsPtr->Status_word.s2_status.Travel_Lock_status = ((ucTmp & 0x10) >> 4);

	// Status_word.s2_status.NA1_Egi
	phsPtr->Status_word.s2_status.NA1_Egi = ((ucTmp & 0x08) >> 3);

	// Status_word.s2_status.INS_in_SH_align_mode
	phsPtr->Status_word.s2_status.INS_in_SH_align_mode = ((ucTmp & 0x04) >> 2);

	// Status_word.s2_status.INS_alert
	phsPtr->Status_word.s2_status.INS_alert = ((ucTmp & 0x02) >> 1);

	// Status_word.s2_status.INS_in_align_on_the_move
	phsPtr->Status_word.s2_status.INS_in_align_on_the_move = (ucTmp & 0x01);
	*intfPtr++;

	// Bits Structure - S1_STATUS_STRUCT
	ucTmp = *intfPtr;
	// Status_word.s1_status.INS_in_startup_mode
	phsPtr->Status_word.s1_status.INS_in_startup_mode = ((ucTmp & 0x80) >> 7);

	// Status_word.s1_status.Startup_Complete
	phsPtr->Status_word.s1_status.Startup_Complete = ((ucTmp & 0x40) >> 6);

	// Status_word.s1_status.INS_normal_or_SH_or_move_align
	phsPtr->Status_word.s1_status.INS_normal_or_SH_or_move_align = ((ucTmp & 0x20) >> 5);

	// Status_word.s1_status.INS_in_survey_mode
	phsPtr->Status_word.s1_status.INS_in_survey_mode = ((ucTmp & 0x10) >> 4);

	// Status_word.s1_status.INS_in_Exclusive_ZUPT_mode
	phsPtr->Status_word.s1_status.INS_in_Exclusive_ZUPT_mode = ((ucTmp & 0x08) >> 3);

	// Status_word.s1_status.Zero_velocity_stop_request
	phsPtr->Status_word.s1_status.Zero_velocity_stop_request = ((ucTmp & 0x04) >> 2);

	// Status_word.s1_status.Position_update_request
	phsPtr->Status_word.s1_status.Position_update_request = ((ucTmp & 0x02) >> 1);

	// Status_word.s1_status.INS_ready_for_Align_on_the_move
	phsPtr->Status_word.s1_status.INS_ready_for_Align_on_the_move = (ucTmp & 0x01);
	*intfPtr++;


	// Bits Structure - S4_STATUS_STRUCT
	ucTmp = *intfPtr;
	// Status_word.s4_status.SH_Shutdown_test_completed
	phsPtr->Status_word.s4_status.SH_Shutdown_test_completed = ((ucTmp & 0x80) >> 7);

	// Status_word.s4_status.Integrated_mode_of_operation
	phsPtr->Status_word.s4_status.Integrated_mode_of_operation = ((ucTmp & 0x40) >> 6);

	// Status_word.s4_status.INS_SH_Shutdown_failed
	phsPtr->Status_word.s4_status.INS_SH_Shutdown_failed = ((ucTmp & 0x20) >> 5);

	// Status_word.s4_status.INS_Shutdown_complete_successful
	phsPtr->Status_word.s4_status.INS_Shutdown_complete_successful = ((ucTmp & 0x10) >> 4);

	// Status_word.s4_status.NA_Egi
	phsPtr->Status_word.s4_status.NA_Egi = ((ucTmp & 0x08) >> 3);

	// Status_word.s4_status.INS_in_standby_mode
	phsPtr->Status_word.s4_status.INS_in_standby_mode = ((ucTmp & 0x04) >> 2);

	// Status_word.s4_status.INS_wait_for_gps
	phsPtr->Status_word.s4_status.INS_wait_for_gps = ((ucTmp & 0x02) >> 1);

	// Status_word.s4_status.NA1_Egi
	phsPtr->Status_word.s4_status.NA1_Egi = (ucTmp & 0x01);
	*intfPtr++;

	// Bits Structure - S3_STATUS_STRUCT
	ucTmp = *intfPtr;
	// Status_word.s3_status.Odo_calibration_in_process
	phsPtr->Status_word.s3_status.Odo_calibration_in_process = ((ucTmp & 0x80) >> 7);

	// Status_word.s3_status.Odo_damping_in_process
	phsPtr->Status_word.s3_status.Odo_damping_in_process = ((ucTmp & 0x40) >> 6);

	// Status_word.s3_status.Odo_calibration_completed
	phsPtr->Status_word.s3_status.Odo_calibration_completed = ((ucTmp & 0x20) >> 5);

	// Status_word.s3_status.NA_Egi
	phsPtr->Status_word.s3_status.NA_Egi = ((ucTmp & 0x10) >> 4);

	// Status_word.s3_status.NA1_Egi
	phsPtr->Status_word.s3_status.NA1_Egi = ((ucTmp & 0x08) >> 3);

	// Status_word.s3_status.INS_in_motion
	phsPtr->Status_word.s3_status.INS_in_motion = ((ucTmp & 0x04) >> 2);

	// Status_word.s3_status.Orientation_attitude_data_valid
	phsPtr->Status_word.s3_status.Orientation_attitude_data_valid = ((ucTmp & 0x02) >> 1);

	// Status_word.s3_status.Degraded_Survey
	phsPtr->Status_word.s3_status.Degraded_Survey = (ucTmp & 0x01);
	*intfPtr++;

	// Structure - ALERT_WORD_1_STRUCT
	// Bits Structure - ALERT_DATA_D2_STRUCT
	ucTmp = *intfPtr;
	// Alert_word_1.Alert_D2.Bit_Reserved
	phsPtr->Alert_word_1.Alert_D2.Bit_Reserved = ((ucTmp & 0x80) >> 7);

	// Alert_word_1.Alert_D2.Bit_Reserved1
	phsPtr->Alert_word_1.Alert_D2.Bit_Reserved1 = ((ucTmp & 0x40) >> 6);

	// Alert_word_1.Alert_D2.SH_Attitude_not_good
	phsPtr->Alert_word_1.Alert_D2.SH_Attitude_not_good = ((ucTmp & 0x20) >> 5);

	// Alert_word_1.Alert_D2.Unable_to_complete_align
	phsPtr->Alert_word_1.Alert_D2.Unable_to_complete_align = ((ucTmp & 0x10) >> 4);

	// Alert_word_1.Alert_D2.Align_interrupt
	phsPtr->Alert_word_1.Alert_D2.Align_interrupt = ((ucTmp & 0x08) >> 3);

	// Alert_word_1.Alert_D2.Bit_Reserved2
	phsPtr->Alert_word_1.Alert_D2.Bit_Reserved2 = ((ucTmp & 0x04) >> 2);

	// Alert_word_1.Alert_D2.Zero_Velocity_update_interrupt
	phsPtr->Alert_word_1.Alert_D2.Zero_Velocity_update_interrupt = ((ucTmp & 0x02) >> 1);

	// Alert_word_1.Alert_D2.Bit_Reserved3
	phsPtr->Alert_word_1.Alert_D2.Bit_Reserved3 = (ucTmp & 0x01);
	*intfPtr++;

	// Structure - ALERT_WORD_2_STRUCT
	// Bits Structure - ALERT_DATA_D3_STRUCT
	ucTmp = *intfPtr;
	// Alert_word_2.Alert_D3.Gyro_Y_Anode_Two_Temperature
	phsPtr->Alert_word_2.Alert_D3.Gyro_Y_Anode_Two_Temperature = ((ucTmp & 0x80) >> 7);

	// Alert_word_2.Alert_D3.Gyro_Z_Anode_One_Temperature
	phsPtr->Alert_word_2.Alert_D3.Gyro_Z_Anode_One_Temperature = ((ucTmp & 0x40) >> 6);

	// Alert_word_2.Alert_D3.Gyro_Z_Anode_Two_Temperature
	phsPtr->Alert_word_2.Alert_D3.Gyro_Z_Anode_Two_Temperature = ((ucTmp & 0x20) >> 5);

	// Alert_word_2.Alert_D3.Gyro_X_Dither_Temperature
	phsPtr->Alert_word_2.Alert_D3.Gyro_X_Dither_Temperature = ((ucTmp & 0x10) >> 4);

	// Alert_word_2.Alert_D3.Gyro_X_Cathode_Temperature
	phsPtr->Alert_word_2.Alert_D3.Gyro_X_Cathode_Temperature = ((ucTmp & 0x08) >> 3);

	// Alert_word_2.Alert_D3.Gyro_Y_Dither_Temperature
	phsPtr->Alert_word_2.Alert_D3.Gyro_Y_Dither_Temperature = ((ucTmp & 0x40) >> 2);

	// Alert_word_2.Alert_D3.Gyro_Y_Cathode_Temperature
	phsPtr->Alert_word_2.Alert_D3.Gyro_Y_Cathode_Temperature = ((ucTmp & 0x20) >> 1);

	// Alert_word_2.Alert_D3.Gyro_Z_Dither_Temperature
	phsPtr->Alert_word_2.Alert_D3.Gyro_Z_Dither_Temperature = (ucTmp & 0x01);
	*intfPtr++;

	// Bits Structure - ALERT_DATA_D4_STRUCT
	ucTmp = *intfPtr;
	// Alert_word_2.Alert_D4.Bit_Reserved
	phsPtr->Alert_word_2.Alert_D4.Bit_Reserved = ((ucTmp & 0x80) >> 7);

	// Alert_word_2.Alert_D4.Bit_Reserved1
	phsPtr->Alert_word_2.Alert_D4.Bit_Reserved1 = ((ucTmp & 0x4) >> 6);

	// Alert_word_2.Alert_D4.GPS_position_conflict
	phsPtr->Alert_word_2.Alert_D4.GPS_position_conflict = ((ucTmp & 0x20) >> 5);

	// Alert_word_2.Alert_D4.Altitude_update_rejected
	phsPtr->Alert_word_2.Alert_D4.Altitude_update_rejected = ((ucTmp & 0x10) >> 4);

	// Alert_word_2.Alert_D4.Horizontal_PositionUpdate_reject
	phsPtr->Alert_word_2.Alert_D4.Horizontal_PositionUpdate_reject = ((ucTmp & 0x08) >> 3);

	// Alert_word_2.Alert_D4.BITS_3_Reserved
	phsPtr->Alert_word_2.Alert_D4.BITS_3_Reserved = ((ucTmp & 0x04) >> 2);
	*intfPtr++;

	// Structure - ALERT_WORD_3_STRUCT
	// Bits Structure - ALERT_DATA_D5_STRUCT
	ucTmp = *intfPtr;
	// Alert_word_3.Alert_D5.Invalid_update_or_data_request
	phsPtr->Alert_word_3.Alert_D5.Invalid_update_or_data_request = ((ucTmp & 0x80) >> 7);

	// Alert_word_3.Alert_D5.Invalid_mode_request
	phsPtr->Alert_word_3.Alert_D5.Invalid_mode_request = ((ucTmp & 0x40) >> 6);

	// Alert_word_3.Alert_D5.BITS_3_Reserved
	phsPtr->Alert_word_3.Alert_D5.BITS_3_Reserved = ((ucTmp & 0x38) >> 3);

	// Alert_word_3.Alert_D5.Invalid_data_received
	phsPtr->Alert_word_3.Alert_D5.Invalid_data_received = ((ucTmp & 0x04) >> 2);

	// Alert_word_3.Alert_D5.Bit_Reserved
	phsPtr->Alert_word_3.Alert_D5.Bit_Reserved = ((ucTmp & 0x02) >> 1);

	// Alert_word_3.Alert_D5.Bit_Reserved1
	phsPtr->Alert_word_3.Alert_D5.Bit_Reserved1 = (ucTmp & 0x01);
	*intfPtr++;

	// Bits Structure - ALERT_DATA_D6_STRUCT
	ucTmp = *intfPtr;
	// Alert_word_3.Alert_D6.BITS_3_Reserved
	phsPtr->Alert_word_3.Alert_D6.BITS_3_Reserved = ((ucTmp & 0xE0) >> 5);

	// Alert_word_3.Alert_D6.GPS_data_unusable
	phsPtr->Alert_word_3.Alert_D6.GPS_data_unusable = ((ucTmp & 0x10) >> 4);

	// Alert_word_3.Alert_D6.Bit_Reserved
	phsPtr->Alert_word_3.Alert_D6.Bit_Reserved = ((ucTmp & 0x08) >> 3);

	// Alert_word_3.Alert_D6.Bit_Reserved1
	phsPtr->Alert_word_3.Alert_D6.Bit_Reserved1 = ((ucTmp & 0x04) >> 2);

	// Alert_word_3.Alert_D6.Bit_Reserved2
	phsPtr->Alert_word_3.Alert_D6.Bit_Reserved2 = ((ucTmp & 0x02) >> 1);

	// Alert_word_3.Alert_D6.ODO_data_unusable
	phsPtr->Alert_word_3.Alert_D6.ODO_data_unusable = (ucTmp & 0x01);
	*intfPtr++;

	// Azimuth_Error_RMS
	ucTmp = *intfPtr;
	if (ucTmp > 20470)
		errFlg++;
	else
		phsPtr->Azimuth_Error_RMS = (float)(ucTmp);
	*intfPtr++;


	// Velocity_error_RMS
	ucTmp = *intfPtr;
	if (ucTmp > 20000)
		errFlg++;
	else {
		dTmp = ucTmp * ((double)((((200.0) - (0.0)) / ((20000.0) - (0.0)))));
		phsPtr->Velocity_error_RMS = (float)(dTmp);
	}
	*intfPtr++;

	// INS_Horizontal_Position_Error
	usTmp = (unsigned short)(GetShort (intfPtr, true));
	if (usTmp > 2047)
		errFlg++;
	else
		phsPtr->INS_Horizontal_Position_Error = usTmp;
	*intfPtr+=2;



	// INS_Altitude_Error
	usTmp = (unsigned short)(GetShort (intfPtr, true));
	if (usTmp > 2047)
		errFlg++;
	else
		phsPtr->INS_Altitude_Error = usTmp;
	*intfPtr+=2;


	// Roll_Error_RMS
	usTmp = (unsigned short)(GetShort (intfPtr, true));
	if (usTmp > 20470)
		errFlg++;
	else
		dTmp = usTmp * ((double)((((2047.0) - (0.0)) / ((20470.0) - (0.0)))));
		phsPtr->Roll_Error_RMS = (float)(dTmp);
		*intfPtr+=2;


	// Pitch_Error_RMS
		usTmp = (unsigned short)(GetShort (intfPtr, true));
	if (usTmp > 20470)
		errFlg++;
	else
		dTmp = usTmp * ((double)((((2047.0) - (0.0)) / ((20470.0) - (0.0)))));
		phsPtr->Pitch_Error_RMS = (float)(dTmp);
	*intfPtr+=2;

	// Number_of_Satellites
	ucTmp = *intfPtr;
	if (ucTmp > 6)
		errFlg++;
	else
		phsPtr->Number_of_Satellites = ucTmp;
	*intfPtr++;


	// Figure_of_Merit
	ucTmp = *intfPtr;
	if (ucTmp > 10)
		errFlg++;
	else
		phsPtr->Figure_of_Merit = ucTmp;
	*intfPtr++;

	// Week
	phsPtr->Week = *intfPtr;
	*intfPtr++;


	// GPS_UTC_offset
	phsPtr->GPS_UTC_offset = *intfPtr;
	*intfPtr++;

	// Bytes_2_Reserved
	usTmp = (unsigned short)(GetShort (intfPtr, true));
	phsPtr->Bytes_2_Reserved = usTmp;
	*intfPtr+=2;

	// Checksum_Egi
	usTmp = (unsigned short)(GetShort (intfPtr, true));
	phsPtr->Checksum_Egi = usTmp;

	return errFlg;
}


// Periodic_100Hz_Message
int Shiphon_Ctrl::Periodic_100Hz_Message_CONVERT_TO_PH (PHS_PERIODIC100HZMESSAGE* phsPtr, unsigned char* dataPtr)
{

	// init interface error flag
	//Periodic_1Hz_Message_ERR_FLG = 0;
 	int 				errFlg = 0;
	unsigned char		ucTmp;
	unsigned short 		usTmp;
	unsigned int 		uiTmp;
	unsigned long 		ulTmp;
	short				sTmp;
	int					iTmp;
	long				lTmp;
	float				fTmp;
	double				dTmp;

	//unsigned char* intfPtr = NULL;

	unsigned char rxBufTmp[SHIPHONE_BUFF_LEN] = {0};
	memcpy (rxBufTmp, dataPtr, SHIPHONE_BUFF_LEN);



	//intfPtr = dataPtr;

	int currId = 0;

	// SOM
	//usTmp = (unsigned short)(GetShort(intfPtr, true));
	memcpy (&usTmp, &(rxBufTmp[currId]), 2);
	if (usTmp > 42405)
		errFlg += 1;
	else
		phsPtr->SOM = usTmp;

	//*intfPtr += 2;
	currId += 2;


	// Message_ID_Accepted_From_EGI
//	ucTmp = *intfPtr;
	ucTmp = rxBufTmp[currId];

	if (ucTmp > 14)
		errFlg += 1;
	else
		phsPtr->Message_ID_Accepted_From_EGI = ucTmp;
	//*intfPtr++;
	currId++;

	// Block_length
	//memcpy (&usTmp, &(rxBuf[currId]), 2);
	ucTmp =  rxBufTmp[currId];
//	usTmp = (unsigned short)(GetShort(intfPtr, true));
	//memcpy (&usTmp, intfPtr, sizeof(unsigned short));
	if (ucTmp > 60)
		errFlg += 1;
	else
//		phsPtr->Block_length = usTmp;
		phsPtr->Block_length = ucTmp;
	currId++;
//	currId += 2;
//	*intfPtr++;

	// INS_time_of_Nav_data
	//memcpy (&ulTmp, &(rxBufTmp[currId]), 4);
	ulTmp = (unsigned long)(GetLong (&(rxBufTmp[currId]), false));
	dTmp = ulTmp * ((double)((((1048576.0) - (0.0)) / ((4294967295.0) - (0.0)))));
	phsPtr->INS_time_of_Nav_data = dTmp;
	//*intfPtr+=4;
	currId+=4;

	// LAT_Egi
	//memcpy (&lTmp, &(rxBufTmp[currId]), 4);
	lTmp = (GetLong (&(rxBufTmp[currId]), false));
	dTmp = lTmp * ((double)((((1.0) - (-1.0)) / ((2147483647.0) - (-2147483648.0))))) + ((double)((-1.0 - (-2147483648.0 * (((1.0) - (-1.0)) / ((2147483647.0) - (-2147483648.0)))))));
	phsPtr->LAT_Egi = dTmp;

	currId+=4;
//	*intfPtr+=4;

	// LONG_Egi
	//memcpy (&lTmp, &(rxBufTmp[currId]), 4);
	lTmp = (GetLong (&(rxBufTmp[currId]), false));
	dTmp = lTmp * ((double)((((1.0) - (-1.0)) / ((2147483647.0) - (-2147483648.0))))) + ((double)((-1.0 - (-2147483648.0 * (((1.0) - (-1.0)) / ((2147483647.0) - (-2147483648.0)))))));
	phsPtr->LONG_Egi = dTmp;
	currId+=4;
	//	*intfPtr+=4;

	// Altitude_MSL_EGI
	//memcpy (&lTmp, &(rxBufTmp[currId]), 4);
	lTmp = GetLong (&(rxBufTmp[currId]), false);
	if ((lTmp < -163840) || (lTmp > 163830))
		errFlg += 1;
	else {
		dTmp = lTmp * ((double)((((16383.0) - (-16384.0)) / ((163830.0) - (-163840.0)))));
		phsPtr->Altitude_MSL_EGI = dTmp;
	}
	currId+=4;
	//	*intfPtr+=4;

	// Pitch_PD_Egi
	//memcpy (&sTmp, &(rxBufTmp[currId]), 2);
	sTmp = GetShort(&(rxBufTmp[currId]), false);
	if ((sTmp < -16000) || (sTmp > 16000))
		errFlg += 1;
	else {
		dTmp = sTmp * ((double)((((1600.0) - (-1600.0)) / ((16000.0) - (-16000.0)))));
		phsPtr->Pitch_PD_Egi = dTmp;
	}
	currId+=2;
	//*intfPtr+=2;


	// Roll_PD_Egi
	sTmp = GetShort(&(rxBufTmp[currId]), false);
	//memcpy (&sTmp, &(rxBufTmp[currId]), 2);
	//sTmp = GetShort(intfPtr, true);
	if ((sTmp < -16000) || (sTmp > 16000))
		errFlg += 1;
	else {
		dTmp = sTmp * ((double)((((1600.0) - (-1600.0)) / ((16000.0) - (-16000.0)))));
		phsPtr->Roll_PD_Egi = dTmp;
	}
	currId+=2;
	//	*intfPtr+=2;


	// Azimuth_PD_geographic
	usTmp = (unsigned short)(GetShort(&(rxBufTmp[currId]), false));
	//memcpy (&usTmp, &(rxBufTmp[currId]), 2);
	//usTmp = (unsigned short)(GetShort(intfPtr, true));
	if (usTmp > 64000)
		errFlg += 1;
	else
	{
		dTmp = usTmp * ((double)((((6400.0) - (0.0)) / ((64000.0) - (0.0)))));
		phsPtr->Azimuth_PD_geographic = dTmp;
	}
	currId+=2;
	// *intfPtr+=2;

	// Roll_rate_X_PD_Egi
	sTmp = GetShort(&(rxBufTmp[currId]), false);
	//memcpy (&sTmp, &(rxBufTmp[currId]), 2);
	//sTmp = GetShort(intfPtr, true);
	if ((sTmp < -32000) || (sTmp > 32000))
		errFlg += 1;
	else {
		dTmp = sTmp * ((double)((((1600.0) - (-1600.0)) / ((32000.0) - (-32000.0)))));
		phsPtr->Roll_rate_X_PD_Egi = dTmp;
	}
	currId+=2;
		// *intfPtr+=2;


	// Pitch_rate_Y_PD_Egi
	sTmp = GetShort(&(rxBufTmp[currId]), false);
	//memcpy (&sTmp, &(rxBufTmp[currId]), 2);
	//sTmp = GetShort(intfPtr, true);
	if ((sTmp < -32000) || (sTmp > 32000))
		errFlg += 1;
	else
	{
		dTmp = sTmp * ((double)((((1600.0) - (-1600.0)) / ((32000.0) - (-32000.0)))));
		phsPtr->Pitch_rate_Y_PD_Egi = dTmp;
	}
	currId+=2;
	//*intfPtr+=2;

	 //Azimuth_rate_Z_PD_Egi
	sTmp = GetShort(&(rxBufTmp[currId]), false);
	//memcpy (&sTmp, &(rxBufTmp[currId]), 2);
	//sTmp = GetShort(intfPtr, true);
	if ((sTmp < -32000) || (sTmp > 32000))
		errFlg += 1;
	else
	{
		dTmp = sTmp * ((double)((((1600.0) - (-1600.0)) / ((32000.0) - (-32000.0)))));
		phsPtr->Azimuth_rate_Z_PD_Egi = dTmp;
	}
	currId+=2;
		//**intfPtr+=2;

	// Acc_X_Egi
	sTmp = GetShort(&(rxBufTmp[currId]), false);
	//memcpy (&sTmp, &(rxBufTmp[currId]), 2);
	//sTmp = GetShort(intfPtr, true);
	if ((sTmp < -5000) || (sTmp > 5000))
		errFlg += 1;
	else
	{
		dTmp = sTmp * ((double)((((50.0) - (-50.0)) / ((5000.0) - (-5000.0)))));
		phsPtr->Acc_X_Egi = dTmp;
	}
	currId+=2;
	// *intfPtr+=2;

	 //Acc_Y_Egi
	sTmp = GetShort(&(rxBufTmp[currId]), false);
	//memcpy (&sTmp, &(rxBufTmp[currId]), 2);
	//sTmp = GetShort(intfPtr, true);
	if ((sTmp < -5000) || (sTmp > 5000))
		errFlg += 1;
	else
	{
		dTmp = sTmp * ((double)((((50.0) - (-50.0)) / ((5000.0) - (-5000.0)))));
		phsPtr->Acc_Y_Egi = dTmp;
	}
	currId+=2;
	//*intfPtr+=2;

	 //Acc_Z_Egi
	sTmp = GetShort(&(rxBufTmp[currId]), false);
	//memcpy (&sTmp, &(rxBufTmp[currId]), 2);
	//sTmp = GetShort(intfPtr, true);
	if ((sTmp < -5000) || (sTmp > 5000))
		errFlg += 1;
	else
	{
		dTmp = sTmp * ((double)((((50.0) - (-50.0)) / ((5000.0) - (-5000.0)))));
		phsPtr->Acc_Z_Egi = dTmp;
	}
	currId+=2;
	// *intfPtr+=2;

	 //Velocity_north_Egi
	sTmp = GetShort(&(rxBufTmp[currId]), false);
	//memcpy (&sTmp, &(rxBufTmp[currId]), 2);
	//sTmp = GetShort(intfPtr, true);
	if (sTmp > 32512)
		errFlg += 1;
	else
	{
		dTmp = sTmp * ((double)((((127.0) - (-128.0)) / ((32512.0) - (-32768.0)))));
		phsPtr->Velocity_north_Egi = dTmp;
	}
	currId+=2;
	//*intfPtr+=2;

	// Velocity_East_Egi
	sTmp = GetShort(&(rxBufTmp[currId]), false);
	//memcpy (&sTmp, &(rxBufTmp[currId]), 2);
	//sTmp = GetShort(intfPtr, true);
	if (sTmp > 32512)
		errFlg += 1;
	else
	{
		dTmp = sTmp * ((double)((((127.0) - (-128.0)) / ((32512.0) - (-32768.0)))));
		phsPtr->Velocity_East_Egi = dTmp;
	}
	currId+=2;
	//*intfPtr+=2;

	// Velocity_down_Egi
	sTmp = GetShort(&(rxBufTmp[currId]), false);
	//memcpy (&sTmp, &(rxBufTmp[currId]), 2);
	//sTmp = GetShort(intfPtr, true);
	if (sTmp > 32512)
		errFlg += 1;
	else
	{
		dTmp = sTmp * ((double)((((127.0) - (-128.0)) / ((32512.0) - (-32768.0)))));
		phsPtr->Velocity_down_Egi = dTmp;
	}
	currId+=2;
	// *intfPtr+=2;

	// Alt_correction_Egi
	sTmp = GetShort(&(rxBufTmp[currId]), false);
	//memcpy (&sTmp, &(rxBufTmp[currId]), 2);
	//phsPtr->Alt_correction_Egi = GetShort(intfPtr, true);
	currId+=2;
	//*intfPtr+=2;

	// Bits Structure - VALIDITY_WORD_EGI_STRUCT
	sTmp = GetShort(&(rxBufTmp[currId]), false);
	//memcpy (&sTmp, &(rxBufTmp[currId]), 2);
	//usTmp = (unsigned short)(GetShort (intfPtr, true));
	// Validity_Word.GPS_Data_Not_Valid_Egi
	phsPtr->Validity_Word.GPS_Data_Not_Valid_Egi = (unsigned char)((((usTmp & 0x8000) >> 15)));

	// Validity_Word.Temperature_Fail_Egi
	phsPtr->Validity_Word.Temperature_Fail_Egi = (unsigned char)((((usTmp & 0x4000) >> 14)));

	// Validity_Word.NA_Egi
	phsPtr->Validity_Word.NA_Egi = (unsigned char)((((usTmp & 0x2000) >> 13)));

	// Validity_Word.NA1_Egi
	phsPtr->Validity_Word.NA1_Egi = (unsigned char)((((usTmp & 0x1000) >> 12)));

	// Validity_Word.NA2_Egi
	phsPtr->Validity_Word.NA2_Egi = (unsigned char)((((usTmp & 0x0800) >> 11)));

	// Validity_Word.NA3_Egi
	phsPtr->Validity_Word.NA3_Egi = (unsigned char)((((usTmp & 0x0400) >> 10)));

	// Validity_Word.NA4_EgicurrId+=2;
	phsPtr->Validity_Word.NA4_Egi = (unsigned char)((((usTmp & 0x0200) >> 9)));

	// Validity_Word.NA5_Egi
	phsPtr->Validity_Word.NA5_Egi = (unsigned char)((((usTmp & 0x0100) >> 8)));

	// Validity_Word.Severe_INS_Fail_Egi
	phsPtr->Validity_Word.Severe_INS_Fail_Egi = (unsigned char)((((usTmp & 0x0080) >> 7)));

	// Validity_Word.External_Voltage_Fail_Egi
	phsPtr->Validity_Word.External_Voltage_Fail_Egi = (unsigned char)((((usTmp & 0x0040) >> 6)));

	// Validity_Word.Antenna_Fail_Egi
	phsPtr->Validity_Word.Antenna_Fail_Egi = (unsigned char)((((usTmp & 0x0020) >> 5)));

	// Validity_Word.ONE_PPS_Fail_Egi
	phsPtr->Validity_Word.ONE_PPS_Fail_Egi = (unsigned char)((((usTmp & 0x0010) >> 4)));

	// Validity_Word.IMU_FAIL_Egi
	phsPtr->Validity_Word.IMU_FAIL_Egi = (unsigned char)((((usTmp & 0x0008) >> 3)));

	// Validity_Word.Gps_Fail_Egi
	phsPtr->Validity_Word.Gps_Fail_Egi = (unsigned char)((((usTmp & 0x0004) >> 2)));

	// Validity_Word.ODO_fail_Egi
	phsPtr->Validity_Word.ODO_fail_Egi = (unsigned char)((((usTmp & 0x0002) >> 1)));

	// Validity_Word.INS_fail_Egi
	phsPtr->Validity_Word.INS_fail_Egi = (unsigned char)(((usTmp & 0x0001)));
	currId+=2;
	//*intfPtr+=2;

	// Input_Message_Number_Echo_Egi
	memcpy (&(phsPtr->Input_Message_Number_Echo_Egi), &(rxBufTmp[currId]), 2);
	//phsPtr->Input_Message_Number_Echo_Egi = (unsigned short)((GetShort(intfPtr, true)));
	currId+=2;
	// *intfPtr+=2;

	// Array - EIGHT_SPARE_BYTE_ARRAY

	//{
	//	int i1;
	//	int locationOffset1 = 0;
	//	for (i1 = 0; i1 < 8; i1++, locationOffset1+= 1 )
	//	{
			// Eight_byte_Spare[i1]
	//		phsPtr->Eight_byte_Spare[i1] = (char)((*(char *) (intfPtr + 50 + locationOffset1)));

	//	}
	//}

	currId+=8;
	//*intfPtr+=8;

	// Checksum_Egi
	memcpy (&(phsPtr->Checksum_Egi), &(rxBufTmp[currId]), 2);
//	phsPtr->Checksum_Egi = (unsigned short)(GetShort (intfPtr, true));

	return errFlg;
}


