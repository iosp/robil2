#include "LLICtrl.h"

#define BUFLEN	256

//#define _ARIK_TEST_

const int qin_headerSize = 16;

const BYTE qin_lli_MsgProp[] = { 0x06, 0x02 };

#ifdef JAUS_BIG_ENDIAN
const unsigned short qin_HeartBeatID = 0x0222;
const unsigned short qin_CtrlConfirmId = 0x0F00;
const unsigned short qin_CtrlRejectId = 0x1000;

const unsigned short qin_DriveCtrlCode = 0x0166;
const unsigned short qin_ManipulatorCtrlCode = 0x0169;

const unsigned short lli_HeartBeatConfirmId = 0x0242;
const unsigned short lli_CtrlCmdId = 0x0D00;
const unsigned short lli_CtrlReleaseId = 0x0E00;
const unsigned short lli_SetWrenchEffortId = 0x0504;
const unsigned short lli_SetJointEffortId = 0x0106;

#elif JAUS_LITLE_ENDIAN
const unsigned short qin_HeartBeatID = 0x2202;
const unsigned short qin_CtrlConfirmId = 0x000F;
const unsigned short qin_CtrlReject = 0x0010;

const unsigned short qin_DriveCtrlCode = 0x6601;
const unsigned short qin_ManipulatorCtrlCode = 0x6901;

const unsigned short lli_HeartBeatConfirmId = 0x4202;
const unsigned short lli_CtrlCmdId = 0x000D;
const unsigned short lli_CtrlReleaseId = 0x000E;
const unsigned short lli_SetWrenchEffortId = 0x0405;
const unsigned short lli_SetJointEffortId = 0x0601;
#endif


const double	ROS_CONTROL_REFRESH_TIME_MAX =	10.;
//const double	ROS_CONTROL_REFRESH_TIME_MAX =	0.3;


char kbKey = '\0';
char dbgKey = '\0';

struct timespec myTc;

// Bytes 2-3 (zero based) - message ID
// Bytes 14-15 - the short message counter (sequence mesage)

#ifdef JAUS_BIG_ENDIAN
// Query Heartbeat Pulse - 20 Hz
BYTE QIN_QueryHeartbeatMsg[] = { 0x06, 0x02, 0x02, 0x22, 0x01, 0xAE, 0x01, 0x05,
		0x01, 0x51, 0x01, 0xC8, 0x00, 0x00, 0xAA, 0xAA };

// Confirm Component Control:
//
// Drive Control Responce
BYTE QIN_DriveControlConfirmMsg[] = { 0x06, 0x02, 0x0F, 0x00, 0x01, 0xAE, 0x01,
		0x05, 0x01, 0x66, 0x01, 0xC8, 0x01, 0x00, 0xAA, 0xAA, 0x00 };
BYTE QIN_ManipulatorControlConfirmMsg[] = { 0x06, 0x02, 0x0F, 0x00, 0x01, 0xAE,
		0x01, 0x05, 0x01, 0x69, 0x01, 0xC8, 0x01, 0x00, 0xAA, 0xAA, 0x00 };

// Reject Control Responce



BYTE QIN_DriveControlRejectMsg[] = { 0x06, 0x02, 0x10, 0x00, 0x01, 0xAE, 0x01,
		0x05, 0x01, 0x66, 0x01, 0xC8, 0x00, 0x00, 0xAA, 0xAA };
BYTE QIN_ManipulatorControlRejectMsg[] = { 0x06, 0x02, 0x10, 0x00, 0x01, 0xAE,
		0x01, 0x05, 0x01, 0x69, 0x01, 0xC8, 0x00, 0x00, 0xAA, 0xAA };

//
// Incoming Messages (from LLI component)
//

// Report Heartbeat Pulse - reply for the Heartbeat Pulse Message (all states)
BYTE LLI_ReportQueryHeartbeatMsg[] = { 0x06, 0x02, 0x02, 0x42, 0x01, 0x51, 0x01,
		0xC8, 0x01, 0xAE, 0x01, 0x05, 0x00, 0x00, 0xAA, 0xAA };

// Request Component Control
//
// To Drive Control (the last Byte - LLI Authority Code)
BYTE LLI_DriveControlMsg[] = { 0x06, 0x02, 0x0D, 0x00, 0x01, 0x66, 0x01, 0xC8,
		0x01, 0xAE, 0x01, 0x05, 0x01, 0x00, 0xAA, 0xAA, 0x00 };
// To Manipulator Control (the last Byte - LLI Authority Code)
BYTE LLI_ManipulatorControlMsg[] = { 0x06, 0x02, 0x0D, 0x00, 0x01, 0x69, 0x01,
		0xC8, 0x01, 0xAE, 0x01, 0x05, 0x01, 0x00, 0xAA, 0xAA, 0x00 };

// Release Component Control
// To Drive Contorl
BYTE LLI_DriveControlReleaseMsg[] = { 0x06, 0x02, 0x0E, 0x00, 0x01, 0x66, 0x01,
		0xC8, 0x01, 0xAE, 0x01, 0x05, 0x00, 0x00, 0xAA, 0xAA };
// To Manipulator Contorl
BYTE LLI_ManipulatorControlReleaseMsg[] = { 0x06, 0x02, 0x0E, 0x00, 0x01, 0x69,
		0x01, 0xC8, 0x01, 0xAE, 0x01, 0x05, 0x00, 0x00, 0xAA, 0xAA };

// Set Wrench Effort
// Bytes 16, 17 - Presence Vector = 0x0021
// Bytes 18-19 - Percentage of Drive Effort (Throttel) by joystick position (X), short integer by range [-100...100]
// Bytes 20-27 - Reserved
// Bytes 28-29 - Percentage of Steering Effort by joystick position, short integer by range [-100...100]

BYTE LLI_SetWrenchEffortMsg[] = { 0x06, 0x02, 0x05, 0x04, 0x01, 0x66, 0x01,
		0xC8, 0x01, 0xAE, 0x01, 0x05, 0x06, 0x00, 0xAA, 0xAA, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00 };
        //0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


// Set Joint Effort (last two bytes - LLS/EFFORT/Throttel and LLS/EFFORT/Steering accordingly)
// Byte 16 - Number of jojnts = 0x02
// Bytes 17-18 - Percentage of Lift Effort by joystick position (X), short integer by range [-100...100]
// Bytes 19-20 - Percentage of Tilt Effort by joystick position (X), short integer by range [-100...100]

BYTE LLI_SetJointEffortMsg[] = { 0x06, 0x02, 0x01, 0x06, 0x01, 0x69, 0x01, 0xC8,
		0x01, 0xAE, 0x01, 0x05, 0x05, 0x00, 0xAA, 0xAA, 0x02, 0x00, 0x00, 0x00, 0x00 };
//#elif
// TBD
#endif


//extern CSharedTimerMeas *		m_glbTimeMeas;
CLLI_Ctrl::CLLI_Ctrl() {

	SetCurrentTimeTag ();

	m_DriveCurrentState.ctrlDevId = lli_Ctrl_Drive;
	m_DriveCurrentState.currState = lli_State_Off;
//	m_DriveCurrentState.respRequest = lli_RR__None;
	m_DriveCurrentState.respRequest = lli_RR__None;
	m_DriveCurrentState.devResponce = lli_RR__None;
	m_DriveCurrentState.timeTag = 0.;
	m_DriveCurrentState.effortTT = 0;
	m_DriveCurrentState.lastCmdTT = 0;
	m_ManipulatorCurrentState.ctrlDevId = lli_Ctrl_Manip;
	m_ManipulatorCurrentState.currState = lli_State_Off;
//	m_ManipulatorCurrentState.respRequest = lli_RR__None;
	m_ManipulatorCurrentState.respRequest = lli_RR__None;
	m_ManipulatorCurrentState.devResponce = lli_RR__None;
	m_ManipulatorCurrentState.timeTag = 0.; //m_DriveCurrentState.timeTag;
	m_ManipulatorCurrentState.effortTT = 0;
	m_ManipulatorCurrentState.lastCmdTT = 0;

}

CLLI_Ctrl::~CLLI_Ctrl() {

	if (!m_IsTerminateThread)
	   m_IsTerminateThread = true;

	//std::this_thread::sleep_for(10s);
	sleep(1);
	//if (m_hThread)
	//TerminateThread (m_hThread, 1);
}

bool CLLI_Ctrl::Init(char *addr, unsigned int lPortID, unsigned int rPortID)
{
	bool resVal = false;
	rxCount = 0;
	txCount = 0;
	memset(rxBuf, 0, MsgBufMax);
	memset(txBuf, 0, MsgBufMax);
	rxBufShift = 0;
	nRcvDataInBuff = 0;
	m_rxTT = 0.;
	m_currTT = 0.;
	m_txTT = 0.;

	m_HeartBeatResponseReq = false;
	m_qineticConnectionActive = false;
	m_txReady = false;

	memset (&m_TxSpooler, 0, TX_SPOOLER_SIZE * sizeof (unsigned short));

	ctrCodeWait2Responce[lli_Ctrl_Drive] = 0x0000;
	ctrCodeWait2Responce[lli_Ctrl_Manip] = 0x0000;

	memset (&(m_TxSpooler.msgIdBuffer), 0, TX_SPOOLER_SIZE * sizeof (unsigned short));
	m_TxSpooler.getId = 0;
	m_TxSpooler.setId = 0;
	m_TxSpooler.num = 0;

	reqThrottel_Val = 0;
	reqSteering_Val = 0;
	memset (&reqDevCtrlRelease, 0, 2 * sizeof (unsigned short));

	int place;
	bool IsOk = false;

	memcpy(udpIP, addr, strlen(addr));  // UDP IP "132.4.6.60"
	udpLP = lPortID; // 2010;
	udpRP = rPortID; // 4997;
	place = 0;
	char ch;

	printf("INIT --- IP = %s\n", addr);

	//m_currState == lli_State_Init;
	resVal = CommConnect();
	if (resVal)
		printf("Connection is OK\n");
	else
		printf("Connection was failed");
	sleep(0.1);

	printf("before CommThreadCreate\n");

	m_DriveCurrentState.reqState = lli_State_Init;
	m_DriveCurrentState.currState = lli_State_Init;
	m_DriveCurrentState.respRequest = lli_RR__None;
	m_DriveCurrentState.devResponce = lli_RR__None;
	m_DriveCurrentState.timeTag = 0.;
	m_DriveCurrentState.effortTT = 0.;
	m_DriveCurrentState.lastCmdTT = 0.;
	m_ManipulatorCurrentState.reqState = lli_State_Init;
	m_ManipulatorCurrentState.currState = lli_State_Init;
	m_ManipulatorCurrentState.respRequest = lli_RR__None;
	m_ManipulatorCurrentState.devResponce = lli_RR__None;
	m_ManipulatorCurrentState.timeTag = 0;
	m_ManipulatorCurrentState.effortTT = 0;
	m_ManipulatorCurrentState.lastCmdTT = 0;

	sleep(0.1);

	CommThreadCreate();
	printf("after CommThreadCreate\n");

	//TimerCreate ();
	return true;
}

bool CLLI_Ctrl::Reset() {
	// TBD
	return true;
}

void CLLI_Ctrl::SetCurrentTimeTag ()
{
	mutex_time.lock();
	clock_gettime(CLOCK_REALTIME, &myTc);
	mutex_time.unlock();
	m_currTT = myTc.tv_sec + myTc.tv_nsec / 1E9;
}

void CLLI_Ctrl::ResetLocalTimeTag (double &l_tt)
{
	SetCurrentTimeTag ();
	l_tt = myTc.tv_sec + myTc.tv_nsec / 1E9;

}


void CLLI_Ctrl::SetReceiveTimeTag ()
{
	SetCurrentTimeTag ();
	m_rxTT = myTc.tv_sec + myTc.tv_nsec / 1E9;

}


bool CLLI_Ctrl::CommConnect() {

	int i, slen = sizeof(si_Remote);
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
		return false;
	}



	memset((char *) &si_Local, 0, sizeof(si_Local));
	si_Local.sin_family = AF_INET;
	si_Local.sin_port = htons(udpLP);
	//if (inet_aton("192.168.101.101", &si_Local.sin_addr) == 0) {
	//			//cout << "inet_aton() failed\n";
	//			printf("inet_aton() failed\n");
	//			return false;
	//		}

	si_Local.sin_addr.s_addr = htonl(INADDR_ANY);

	if (setsockopt(socketFd, SOL_SOCKET, SO_RCVTIMEO,
			reinterpret_cast<char*>(&tv), sizeof(timeval))) {
		printf("setsockopt() failed\n");
		return false;
	}

	retV = bind(socketFd, (struct sockaddr *) &si_Local, sizeof(si_Local));
	if (retV == -1) {
		perror("bind error\n");
		return false;
	}
	sleep(0.5);
//    printf ("socketFd = %d\n", socketFd);

	return true;
}

int CLLI_Ctrl::ReceiveData() {
	// TBD
	return 0;
}


bool CLLI_Ctrl::TransmitData(short bufSize) {

	double currTime;

	SetCurrentTimeTag ();
	mutex_time.lock();
	currTime = myTc.tv_sec + myTc.tv_nsec / 1E9;
	mutex_time.unlock();

    sendto(socketFd, txBuf, bufSize, 0,
			(struct sockaddr *) &si_Remote, sizeof(si_Remote));

	if (dbgKey == 't' || dbgKey == 'f') {

		printf("[%.6f]Trx:", currTime);
		for (int i = 0; i < bufSize; i++) {
			printf("%02X  ", txBuf[i]);
		}
		printf("\n");
	}
	return true;
}

/*
bool CLLI_Ctrl::ResponcePreparation() {
	// for test only
	static long tmpCount = 0;

	if (m_qineticConnectionActive && m_HeartBeatResponseReq) {
		m_HeartBeatResponseReq = false;
		//memcpy (txBuf, LLI_ReportQueryHeartbeatMsg, sizeof (LLI_ReportQueryHeartbeatMsg));
		m_txReady = true;
	}

	return m_txReady;
}
*/

bool CLLI_Ctrl::SetDeviceRequest (LLI_CONTROL_DEVICE devCtrl, LLI_STATE stateReq)
{
	bool bVal = true;

	LLI_SM * devSM = (devCtrl == lli_Ctrl_Drive) ? &m_DriveCurrentState : &m_ManipulatorCurrentState;

	if (stateReq != devSM->reqState && stateReq != devSM->currState) {
		devSM->reqState = stateReq;
	}
	else
		bVal = false;

    return bVal;
}


bool CLLI_Ctrl::SetMsgId_To_TxSpooler (unsigned short msgId)
{
	bool testVal = true;
	int i, k;

	if (msgId < TX_MSGID_MAX && m_TxSpooler.num < TX_SPOOLER_SIZE) {

	    i = m_TxSpooler.getId;
	    for (k = 0; k < m_TxSpooler.num; k++) {
	    	if (m_TxSpooler.msgIdBuffer[i] == msgId) {
	    		testVal = false;
	    		break;
	    	}
	    	i = (i + 1) % TX_SPOOLER_SIZE;
	    }

	    if (testVal) {
	    	m_TxSpooler.msgIdBuffer[m_TxSpooler.setId] = msgId;
	    	m_TxSpooler.num++;
	    	if (m_TxSpooler.num < TX_SPOOLER_SIZE)
	    	   m_TxSpooler.setId = (m_TxSpooler.setId + 1) % TX_SPOOLER_SIZE;
	    	else
	    		m_TxSpooler.setId = 0xFFFF;
	    }
	}
	else
		testVal = false;

    return testVal;

}

/*
	 unsigned short CLLI_Ctrl::GetMsgId_From_TxSpooler ()
{
	// tbd
   return 0;
}
*/

void CLLI_Ctrl::ResetThrottelRequest ()
{
//#ifdef _ARIK_TEST_
	printf ("ResetThrottelRequest\n");
//#endif
	reqThrottel_Val = 0;
}

void CLLI_Ctrl::ResetSteeringRequest ()
{
//#ifdef _ARIK_TEST_
	printf ("ResetSteeringRequest\n");
//#endif
	reqSteering_Val = 0;
}

void CLLI_Ctrl::ResetJointRequest ()
{
//#ifdef _ARIK_TEST_
	printf ("ResetJointRequest\n");
//#endif
	reqJoints_Val[0] = 0;
	reqJoints_Val[1] = 0;

}

void CLLI_Ctrl::SetThrottelRequest (short reqVal)
{
	short valScaledTmp;

	if (reqVal >= 0)
		valScaledTmp = reqVal <= 100 ? reqVal : 100;
	else
		valScaledTmp = reqVal >= -100 ? reqVal : -100;

	reqThrottel_Val = JausRealToShort (valScaledTmp, -100, 100);

	//printf ("SetThrotelRequest: %d -> %d -> %d\n", reqVal, valScaledTmp, reqThrottel_Val);
	ResetLocalTimeTag (m_DriveCurrentState.effortTT);
	mutex_time.lock();
	m_DriveCurrentState.lastCmdTT = m_DriveCurrentState.effortTT;
	mutex_time.unlock();
}

void CLLI_Ctrl::SetSteeringRequest (short reqVal)
{
	short valScaledTmp;

	if (reqVal >= 0)
		valScaledTmp = reqVal <= 100 ? reqVal : 100;
	else
		valScaledTmp = reqVal >= -100 ? reqVal : -100;

	reqSteering_Val = JausRealToShort (valScaledTmp, -100, 100);

	//printf ("SetSteeringRequest: %d -> %d -> %d\n", reqVal, valScaledTmp, reqSteering_Val);

	ResetLocalTimeTag (m_DriveCurrentState.effortTT);
	mutex_time.lock();
	m_DriveCurrentState.lastCmdTT = m_DriveCurrentState.effortTT;
	mutex_time.unlock();
}


void CLLI_Ctrl::SetJointRequest (short reqVal1, short reqVal2)
{
	short valScaledTmp;

	if (reqVal1 >= 0)
	   valScaledTmp = reqVal1 <= 100 ? reqVal1 : 100;
	else
		valScaledTmp = reqVal1 >= -100 ? reqVal1 : -100;

	reqJoints_Val[0] = JausRealToShort (valScaledTmp, -100, 100);

	if (reqVal2 >= 0)
		valScaledTmp = reqVal2 <= 100 ? reqVal2 : 100;
	else
		valScaledTmp = reqVal2 >= -100 ? reqVal2 : -100;

	reqJoints_Val[1] = JausRealToShort (valScaledTmp, -100, 100);

	printf ("Before SetJointRequest: Scaling: %d --> %d       %d --> %d\n", reqVal1, reqJoints_Val[0], reqVal2, reqJoints_Val[1]);

	ResetLocalTimeTag (m_ManipulatorCurrentState.effortTT);
	mutex_time.lock();
	m_ManipulatorCurrentState.lastCmdTT = m_ManipulatorCurrentState.effortTT;
	mutex_time.unlock();

	printf ("After SetJointRequest: Scaling: %d --> %d       %d --> %d\n", reqVal1, reqJoints_Val[0], reqVal2, reqJoints_Val[1]);
}


void CLLI_Ctrl::PritnOfStatePassed (LLI_SM * devSM)
{
	static LLI_STATE prevState[2] = { lli_State_Off, lli_State_Off} ;

    if (prevState[devSM->ctrlDevId] != devSM->currState) {
    	printf ("[%.3f]Device #%d:  %d ---> %d state\n", m_currTT, devSM->ctrlDevId, prevState[devSM->ctrlDevId], devSM->currState);
    	prevState[devSM->ctrlDevId] = devSM->currState;
    }
}


bool CLLI_Ctrl::StateMachineSwitch(LLI_SM * devSM)
{
	PritnOfStatePassed (devSM);

	switch (devSM->currState) {
	   case lli_State_Init:

		   if (m_qineticConnectionActive)
		   			devSM->currState = lli_State_Standby;
		   break;

	   case lli_State_Standby:

		   if (devSM->reqState != devSM->currState) {
			   if (devSM->reqState == lli_State_Ready) {
				   if (!m_txDone) {
				      RequestComponentControl (devSM->ctrlDevId);
	    	          m_txDone = true;
				   /*
				   if (devSM->ctrlDevId == lli_Ctrl_Drive)
				      SetMsgId_To_TxSpooler (lli_TransmitDriveCtrl_Msg);
				   else
				      SetMsgId_To_TxSpooler (lli_TransmitManipulatorCtrl_Msg);
				      */
  				      devSM->currState = lli_State_Wait_Responce;
				      devSM->respRequest = lli_RR_Confirm;
				      devSM->timeTag = m_currTT;
				   }
			   }
			   else {
				   devSM->currState = devSM->reqState;
				   devSM->respRequest = lli_RR__None;
				   devSM->devResponce = lli_RR__None;
				   devSM->timeTag = 0.;
			       devSM->effortTT = 0.;
			   }
		   }
		   break;

	   case lli_State_Wait_Responce:

		   if (devSM->respRequest == devSM->devResponce) {
			   devSM->respRequest = lli_RR__None;
			   devSM->devResponce = lli_RR__None;
			   devSM->currState = lli_State_Ready;
			   devSM->timeTag = m_currTT;
		   }
		   else if (m_currTT - devSM->timeTag > ROS_CONTROL_REFRESH_TIME_MAX) {
               if (dbgKey == 'f')
			      printf ("[%.3f]lli_State_Wait_Responce: %.3f\n", m_currTT, m_currTT - devSM->timeTag);
			   devSM->reqState = lli_State_Standby;
			   devSM->currState = lli_State_Standby;
		       devSM->respRequest = lli_RR__None;
		       devSM->devResponce = lli_RR__None;
		       devSM->timeTag = m_currTT;
		   }
	       break;

	   case lli_State_Ready:


		   // Check of Release command from ROS
		   if (reqDevCtrlRelease[devSM->ctrlDevId]) {
			   if (devSM->respRequest != lli_RR_Reject) {
				   if (m_txDone)
					   break;
				   RequestComponentRelease (devSM->ctrlDevId);
		           m_txDone = true;
				  // devSM->respRequest = qin_CtrlRejectId;
				   devSM->respRequest = lli_RR_Reject;
				   //reqDevCtrlRelease = 0;
				   devSM->timeTag = m_currTT;
			   }
			   else if (devSM->respRequest == devSM->devResponce ||
				        m_currTT - devSM->timeTag > 0.3) {

	              if (dbgKey == 'f' && m_currTT - devSM->timeTag > 0.3) {
            		  printf ("[%.3f]No Reject Response: %.3f\n", m_currTT, m_currTT - devSM->timeTag);
	              }

			      devSM->reqState = lli_State_Standby;
			      devSM->currState = lli_State_Standby;
		          devSM->respRequest = lli_RR__None;
		          devSM->devResponce = lli_RR__None;
		          devSM->timeTag = 0.;
		          devSM->effortTT = 0.;
		          reqDevCtrlRelease[devSM->ctrlDevId] = false;
		      }
	          break;
		   }


		   if ((devSM->effortTT > 0.1) && (m_currTT - devSM->effortTT > ROS_CONTROL_REFRESH_TIME_MAX)) {
			   ResetEffortRequest (devSM->ctrlDevId);
			   devSM->effortTT = 0.;
		   }

		   if (m_currTT - devSM->timeTag > 0.1 && !m_txDone) {
			   SetEffortControl (devSM->ctrlDevId);
			   devSM->timeTag = m_currTT;
		   }

		   break;

	   default:
		   break;

	} // switch

	return true;
}


unsigned short CLLI_Ctrl::RequestComponentControl (LLI_CONTROL_DEVICE dev2Ctrl)
{
	unsigned short retVal = 0;

	BYTE L_AutorityCode[2] = { 0x38, 0x38 };

	if (dev2Ctrl == lli_Ctrl_Drive) {
		TransmitDriveCtrlMsg (L_AutorityCode[lli_Ctrl_Drive]);
	}
	else if (dev2Ctrl == lli_Ctrl_Manip) {
		TransmitManipulatorCtrlMsg (L_AutorityCode[lli_Ctrl_Manip]);
	}
	else retVal = 0xFFFF;

	if (retVal != 0xFFFF) {
	   retVal = qin_CtrlConfirmId;
	   m_txDone = true;
	}

	return retVal;
}


unsigned short CLLI_Ctrl::RequestComponentRelease (LLI_CONTROL_DEVICE dev2Ctrl)
{
	unsigned short retVal = 0;

 	if (dev2Ctrl == lli_Ctrl_Drive) {
		TransmitReleaseDriveCtrlMsg ();
 	}
	else if (dev2Ctrl == lli_Ctrl_Manip) {
		TransmitReleaseManipulatorCtrlMsg ();
	}

	else retVal = 0xFFFF;

	if (retVal != 0xFFFF) {
	   retVal = qin_CtrlConfirmId;
	   m_txDone = true;
	}
	else retVal = -1;

	if (retVal != -1) {
	   retVal = qin_CtrlConfirmId;
	   m_txDone = true;
	}
	return retVal;
}



bool CLLI_Ctrl::SetEffortControl (LLI_CONTROL_DEVICE dev2Ctrl)
{
	bool retVal = 0;

	if (dev2Ctrl == lli_Ctrl_Drive)
		TransmitWrenchEffortMsg ();
	else if (dev2Ctrl == lli_Ctrl_Manip) {
		TransmitJoinEffortMsg ();
	}
	else retVal = 0xFFFF;

	if (retVal != 0xFFFF) {
	   retVal = qin_CtrlConfirmId;
	   m_txDone = true;
	}

	return retVal;
}

void CLLI_Ctrl::ResetEffortRequest (LLI_CONTROL_DEVICE dev2Ctrl)
{
	if (dev2Ctrl == lli_Ctrl_Drive) {
		ResetThrottelRequest ();
		ResetSteeringRequest ();
	}
	else if (dev2Ctrl == lli_Ctrl_Manip)
		ResetJointRequest ();
}

bool CLLI_Ctrl::TimerCreate() {
	// T B D
	return false;
}


bool  CLLI_Ctrl::GetQinetiqActived ()
{
	const float dtTolerance = 10.;

	return (fabs(m_currTT - m_rxTT) < dtTolerance);
}


bool CLLI_Ctrl::PeriodicActivity() {
	static long periodicCount = 0;
	char ch;

	static short jointsValTest[2] = {0};
	static short jointId = 0;
	static short steeringValTest = 0;
	static short throttleValTest = 0;

	m_txDone = false;
	SetCurrentTimeTag ();

	m_qineticConnectionActive = GetQinetiqActived ();

	// Keyboard command simulation
	if (Kb_hit() != 0) {
		ch = getchar();
		kbKey = ch;


		switch (kbKey) {

		   case 27:					// Terminate
			   m_IsTerminateThread = true;

			   return false;

		   case 'r':  // received messages print
		   case 't':  // transmited messages print
		   case 'f':  // full network traffic print
			   dbgKey = (dbgKey) ? '\0' : ch;
			   break;


		   case '2':				// Standby State Request
			   m_DriveCurrentState.reqState = lli_State_Standby;
			   m_DriveCurrentState.currState = lli_State_Standby;
			   m_ManipulatorCurrentState.reqState = lli_State_Standby;
			   m_ManipulatorCurrentState.currState = lli_State_Standby;

			   break;

		   case '3':				// Ready State Request
		       m_DriveCurrentState.reqState = lli_State_Ready;

			   break;

		   case '4':				// Ready State Request
		       m_ManipulatorCurrentState.reqState = lli_State_Ready;

			   break;

           // Throttle control
		   case 'a':
			   throttleValTest  -= 10;
			   printf ("Throttle preparation: %d\n", throttleValTest);
		       break;

		   case 's':
			   throttleValTest  = 0;
			   SetThrottelRequest (throttleValTest);
			   printf ("Throttle preparation: %d\n", throttleValTest);
		       break;

		   case 'd':
			   throttleValTest  += 10;
			   printf ("Throttle preparation: %d\n", throttleValTest);
         		       break;

		   case 'w':
			   SetThrottelRequest (throttleValTest);
    		           break;


  
           // Steering control
		   case 'j':
			   steeringValTest  -= 10;
			   printf ("Steering preparation: %d\n", steeringValTest);
		       break;

		   case 'k':
			   steeringValTest  = 0;
			   SetSteeringRequest (steeringValTest);
			   printf ("Steering preparation: %d\n", steeringValTest);
		       break;

		   case 'l':
			   steeringValTest  += 10;
			   printf ("Steering preparation: %d\n", steeringValTest);
		       break;

		   case 'i':
		       SetSteeringRequest (steeringValTest);
		       break;

			// joint control
		   case 'z':
			   jointsValTest[jointId] += 10;
			   printf ("Joint #%d preparation: %d\n", jointId, jointsValTest[jointId]);
			   break;

		   case 'x':
			   jointsValTest[jointId] = 0;
			   SetJointRequest (jointsValTest[0], jointsValTest[1]);
			   printf ("Joint #%d preparation: %d\n", jointId, jointsValTest[jointId]);
			   break;

		   case 'c':
			   jointsValTest[jointId] -= 10;
			   printf ("Joint #%d preparation: %d\n", jointId, jointsValTest[jointId]);
			   break;

		   case 'v':
			   SetJointRequest (jointsValTest[0], jointsValTest[1]);
			   break;

		   case 'b':
			   jointsValTest[jointId] = 0;
			   SetJointRequest (jointsValTest[0], jointsValTest[1]);
			   jointId =  1 - jointId;
			   printf ("Joint ID switch to %d: %d\n", jointId, steeringValTest);
			   break;

		   case '8':				// Drive Control Release Request
			   reqDevCtrlRelease[lli_Ctrl_Drive] = true;

			   break;

		   case '9':				// Manipulator Control Release Request
			   reqDevCtrlRelease[lli_Ctrl_Manip] = true;

			   break;

		   default:
			   break;

		} // switch (...

		if (kbKey != '\0') {
			//printf ("kbKey switched to %c\n", kbKey);
			kbKey = '\0';
		}

	} // if (Kb_hit...


	// Get of ROS Command
	// T B D


	if (m_currTT - m_DriveCurrentState.lastCmdTT > 0.3) {
		if (reqThrottel_Val != 0)
		   ResetThrottelRequest ();
		if (reqSteering_Val != 0)
		   ResetSteeringRequest ();
	}

	if (m_currTT - m_ManipulatorCurrentState.lastCmdTT > 0.3) {

		if (reqJoints_Val[0] * reqJoints_Val[1] != 0)
		   ResetJointRequest ();
	}

	if (m_qineticConnectionActive && m_HeartBeatResponseReq ) {

		TransmitResponceHeartBit();
   		m_HeartBeatResponseReq = false;
   		m_txDone = true;
     }


    if (m_txDone)
		return true;

	StateMachineSwitch (&m_DriveCurrentState);
	StateMachineSwitch (&m_ManipulatorCurrentState);


	/*  if (m_txReady)
	 TransmitData ();
	 //else if (ResponcePreparation ())
	 else if (m_HeartBeatResponseReq) {
		TransmitResponceHeartBit();
		m_HeartBeatResponseReq = false;
	}
	*/

	return true;

}

void CLLI_Ctrl::TransmitResponceHeartBit() {
	//memcpy (txBuf, LLI_ReportQueryHeartbeatMsg, sizeof (LLI_ReportQueryHeartbeatMsg));
	static short msgCounter = 0;
	double currTime;
	static double prevTime = 0;

	SetCurrentTimeTag ();
	mutex_time.lock();
	currTime = myTc.tv_sec + myTc.tv_nsec / 1E9;
	mutex_time.unlock();

	txCount++;
	txSeqNumber++;
	memcpy(&LLI_ReportQueryHeartbeatMsg[14], &txSeqNumber, sizeof(short));

	memcpy (txBuf, LLI_ReportQueryHeartbeatMsg, sizeof(LLI_ReportQueryHeartbeatMsg));
	TransmitData (sizeof(LLI_ReportQueryHeartbeatMsg));

}

void CLLI_Ctrl::TransmitWrenchEffortMsg()
{
	const unsigned short presenceVector = 0x0021;
	memcpy(&(LLI_SetWrenchEffortMsg[16]), &presenceVector, sizeof(short));
	memcpy(&(LLI_SetWrenchEffortMsg[18]), &reqThrottel_Val, sizeof(unsigned short));
	memcpy(&(LLI_SetWrenchEffortMsg[20]), &reqSteering_Val, sizeof(unsigned short));

	txCount++;
	txSeqNumber++;
	memcpy(&LLI_SetWrenchEffortMsg[14], &txSeqNumber, sizeof(short));
	memcpy (txBuf, LLI_SetWrenchEffortMsg, sizeof(LLI_SetWrenchEffortMsg));
	mutex_time.lock();
	m_DriveCurrentState.effortTT = m_currTT;
	mutex_time.unlock();
	TransmitData (sizeof(LLI_SetWrenchEffortMsg));

}

void CLLI_Ctrl::TransmitJoinEffortMsg()
{
	memcpy(&(LLI_SetJointEffortMsg[17]), &(reqJoints_Val[0]), sizeof(short));
	memcpy(&(LLI_SetJointEffortMsg[19]), &(reqJoints_Val[1]), sizeof(short));
	txCount++;
	txSeqNumber++;
	memcpy(&LLI_SetJointEffortMsg[14], &txSeqNumber, sizeof(short));

	memcpy (txBuf, LLI_SetJointEffortMsg, sizeof(LLI_SetJointEffortMsg));

/*
	printf ("[%.3f -- %d]  ", m_currTT, (int)tmpTxFlag);
	for (int i = 0; i < sizeof(LLI_SetJointEffortMsg); i++) {
		printf ("%02X ", txBuf[i]);
	}
	printf ("\n");
*/
	m_ManipulatorCurrentState.effortTT = m_currTT;
	TransmitData (sizeof(LLI_SetJointEffortMsg));

}

void CLLI_Ctrl::TransmitDriveCtrlMsg(BYTE val) {

	memcpy(&(LLI_DriveControlMsg[16]), &val, 1);
//	memcpy(&(LLI_DriveControlMsg[16]), &val, sizeof(unsigned short));

	txCount++;
	txSeqNumber++;
	memcpy(&LLI_DriveControlMsg[14], &txSeqNumber, sizeof(short));

	memcpy (txBuf, LLI_DriveControlMsg, sizeof(LLI_DriveControlMsg));
	ctrCodeWait2Responce[lli_Ctrl_Drive] = qin_DriveCtrlCode;
	TransmitData (sizeof(LLI_DriveControlMsg));


}


void CLLI_Ctrl::TransmitManipulatorCtrlMsg(BYTE val) {

	memcpy(&(LLI_ManipulatorControlMsg[16]), &val, 1);
//	memcpy(&(LLI_ManipulatorControlMsg[16]), &val, sizeof(unsigned short));

	txCount++;
	txSeqNumber++;
	memcpy(&LLI_ManipulatorControlMsg[14], &txSeqNumber, sizeof(short));

	memcpy (txBuf, LLI_ManipulatorControlMsg, sizeof(LLI_ManipulatorControlMsg));
	ctrCodeWait2Responce[lli_Ctrl_Manip] = qin_ManipulatorCtrlCode;

	TransmitData (sizeof(LLI_ManipulatorControlMsg));


}


void CLLI_Ctrl::TransmitReleaseDriveCtrlMsg() {

	txCount++;
	txSeqNumber++;
	memcpy(&LLI_DriveControlReleaseMsg[14], &txSeqNumber, sizeof(short));

	memcpy (txBuf, LLI_DriveControlReleaseMsg, sizeof(LLI_DriveControlReleaseMsg));
	TransmitData (sizeof(LLI_DriveControlReleaseMsg));

	ctrCodeWait2Responce[lli_Ctrl_Drive] = qin_DriveCtrlCode;
}

void CLLI_Ctrl::TransmitReleaseManipulatorCtrlMsg() {

	txCount++;
	txSeqNumber++;
	memcpy(&LLI_ManipulatorControlReleaseMsg[14], &txSeqNumber, sizeof(short));

	memcpy (txBuf, LLI_ManipulatorControlReleaseMsg, sizeof(LLI_ManipulatorControlReleaseMsg));
	TransmitData (sizeof(LLI_ManipulatorControlReleaseMsg));

	ctrCodeWait2Responce[lli_Ctrl_Manip] = qin_ManipulatorCtrlCode;
}

unsigned short CLLI_Ctrl::ParseData() {

	unsigned short retVal = 0;
	BYTE msgHeader[qin_headerSize];
	unsigned short testValTmp;
	unsigned short codeTmp;
	unsigned short *wTemp;
	unsigned short seqMsgCount = 0;
	bool msgValid = false;
	short currId = 2;

	//if (nRcvDataInBuff >= qin_headerSize) {
	memcpy(&msgHeader, &(rxBuf[0]), sizeof(qin_headerSize));
	if (memcmp(&qin_lli_MsgProp, msgHeader, 2) == 0) {
		wTemp = (unsigned short *) &(msgHeader[2]);

		codeTmp = GetShort(&msgHeader[2], false);
		//if (MsgVerification (*wTemp)) {
/*
#ifdef _ARIK_TEST_
		if (dbgKey == 'a')
		   printf ("ParseData: %04X --> %04X\n", *wTemp, codeTmp);
#endif
*/
		switch (codeTmp) {
		case qin_HeartBeatID:
			if (nRcvDataInBuff == sizeof(QIN_QueryHeartbeatMsg)) {
				if (memcmp(rxBuf, QIN_QueryHeartbeatMsg, 14) == 0) {
					memcpy(&seqMsgCount, &(rxBuf[14]), 2);
					msgValid = true;
					m_HeartBeatResponseReq = true;
					SetReceiveTimeTag ();
					if (m_DriveCurrentState.currState == lli_State_Init)
					   m_DriveCurrentState.reqState = lli_State_Standby;
					if (m_ManipulatorCurrentState.currState == lli_State_Init)
						m_ManipulatorCurrentState.reqState = lli_State_Standby;
				}
			}

			break;

		case qin_CtrlConfirmId:

		   if (nRcvDataInBuff == sizeof(QIN_DriveControlConfirmMsg)) {
//#ifdef _ARIK_TEST_
		     //if (dbgKey == 'a')
		       // printf ("Case qin_CtrlConfirmId: Code: %04x = %04x\n", ctrCodeWait2Responce[lli_Ctrl_Manip], qin_DriveCtrlCode);

		//for (int i = 0; i )
//#endif
				if (memcmp(rxBuf, QIN_DriveControlConfirmMsg, 14) == 0) {
					memcpy(&testValTmp, &(QIN_DriveControlConfirmMsg[8]), 2);

					if (ctrCodeWait2Responce[lli_Ctrl_Drive] == qin_DriveCtrlCode)	{
						ctrCodeWait2Responce[lli_Ctrl_Drive] = 0;
						m_DriveCurrentState.devResponce = lli_RR_Confirm;
					}
				}
				else if (memcmp(rxBuf, QIN_ManipulatorControlConfirmMsg, 14) == 0) {
					memcpy(&testValTmp, &(QIN_ManipulatorControlConfirmMsg[8]), 2);

					if (ctrCodeWait2Responce[lli_Ctrl_Manip] == qin_ManipulatorCtrlCode) {
						ctrCodeWait2Responce[lli_Ctrl_Manip] = 0;
						m_ManipulatorCurrentState.devResponce = lli_RR_Confirm;
					}
				}
				memcpy(&seqMsgCount, &(rxBuf[14]), 2);
				msgValid = true;
				SetReceiveTimeTag ();
		   }

					/*
					if (ctrCodeWait2Responce[lli_Ctrl_Drive] == qin_DriveCtrlCode)	{ // &&
						//ctrCodeWait2Responce[lli_Ctrl_Drive] == testValTmp) {
							ctrCodeWait2Responce[lli_Ctrl_Drive] = 0;
//							m_DriveCurrentState.devResponce = qin_CtrlConfirmId;
							m_DriveCurrentState.devResponce = lli_RR_Confirm;
					}
					else if (ctrCodeWait2Responce[lli_Ctrl_Manip] == qin_ManipulatorCtrlCode {  // &&
							   ctrCodeWait2Responce[lli_Ctrl_Manip] == testValTmp) {
						ctrCodeWait2Responce[lli_Ctrl_Manip] = 0;
						//m_ManipulatorCurrentState.devResponce = qin_CtrlConfirmId;
						m_ManipulatorCurrentState.devResponce = lli_RR_Confirm;
					}

				*/

			break;

		case qin_CtrlRejectId:
//#ifdef _ARIK_TEST_
	//	if (dbgKey == 'a')
		//   printf ("Case qin_CtrlRejectId: %04X\n", codeTmp);
//#endif
			if (nRcvDataInBuff == sizeof(QIN_DriveControlRejectMsg)) {
				if (memcmp(&(rxBuf[nRcvDataInBuff]), QIN_DriveControlRejectMsg, 14) == 0) {
					memcpy(&testValTmp, &(QIN_DriveControlRejectMsg[8]), 2);

						if (ctrCodeWait2Responce[lli_Ctrl_Drive] == qin_DriveCtrlCode &&
							ctrCodeWait2Responce[lli_Ctrl_Drive] == testValTmp) {
								m_DriveCurrentState.reqState = lli_State_Standby;
								ctrCodeWait2Responce[lli_Ctrl_Drive] = 0;
// 								m_DriveCurrentState.devResponce = qin_CtrlRejectId;
								m_DriveCurrentState.devResponce = lli_RR_Reject;
						}
						else if (ctrCodeWait2Responce[lli_Ctrl_Manip] == qin_ManipulatorCtrlCode &&
								   ctrCodeWait2Responce[lli_Ctrl_Manip] == testValTmp) {
							m_ManipulatorCurrentState.reqState = lli_State_Standby;
							ctrCodeWait2Responce[lli_Ctrl_Manip] = 0;
//							m_ManipulatorCurrentState.devResponce = qin_CtrlRejectId;
							m_ManipulatorCurrentState.devResponce = lli_RR_Reject;
						}
						memcpy(&seqMsgCount, &(rxBuf[14]), 2);
						msgValid = true;
						SetReceiveTimeTag ();
				}
			}
			break;

		default:
			// Unknown message
			cerr << "Unknown package code = " << codeTmp << endl;
			break;
		} // switch


		// Check Connection Active by rxCount , T B D
		//CheckConnectionActive(msgValid, seqMsgCount);

		//if (m_qineticConnectionActive)
		//	m_currTT = m_rxTT;
		//} // if (MsgVerification
	} // if (memcmp
	nRcvDataInBuff = 0;

//	} // if (nRcvDataInBuff..

	return retVal;
}

void CLLI_Ctrl::CheckConnectionActive(bool valid, unsigned short msgCount) {
	const unsigned short counterTolerance = 5;
	static unsigned short notValidCounter = 0;

	if (valid) {
		notValidCounter = 0;
		if (rxCount == 0xffff)
			rxCount = 0;
		m_qineticConnectionActive = (rxCount < msgCount
				&& msgCount - rxCount < counterTolerance);
		rxCount = msgCount;
	} else {
		notValidCounter++;
		if (m_qineticConnectionActive)
			m_qineticConnectionActive = (notValidCounter < counterTolerance);
	}


}

void CLLI_Ctrl::RxBufferInit() {
	// TBD
}

void CLLI_Ctrl::TxBufferInit() {
	// TB

}

bool CLLI_Ctrl::CommThreadCreate() {
	pthread_t t; // (&Comm4RcvThread);

	pthread_create(&t, NULL, &Comm4RcvThread, this);

	/*
	 void* result;
	 printf ("before pthread_join\n");
	 pthread_join(t, &result);
	 printf ("after pthread_join\n");
	 */
	//t.join ();
	return true;
}

void * CLLI_Ctrl::Comm4RcvThread(void * pParam) {

	CLLI_Ctrl *myHandle = (CLLI_Ctrl *) (pParam);

	printf("before ThreadFunc\n");
	myHandle->ThreadFunc();
	printf("after ThreadFunc\n");

}

void CLLI_Ctrl::ThreadFunc() {
	unsigned char bufTmp[256] = { 0 };
	socklen_t slen = 0;
	//int slen = 0;
	int retVal = 0;

	double currentTime;
	static double lastTime = 0.;
	double cutrentDT;

	static long loopCount = 0;

	while (!m_IsTerminateThread) {
/*
		if (m_DriveCurrentState.currState == lli_State_Off ||
			m_DriveCurrentState.currState == lli_State_Init ||
			m_ManipulatorCurrentState.currState == lli_State_Off ||
			m_ManipulatorCurrentState.currState == lli_State_Init) {

			sleep(0.01);
			continue;
		}
*/
		try {
			retVal = recvfrom(socketFd, bufTmp, BUFLEN, 0,
					(struct sockaddr *) &si_Remote, &slen);

		} catch (exception& err) {
			printf("receive error #%s\n", err.what());
		}


		SetCurrentTimeTag ();
		mutex_time.lock();
		currentTime = myTc.tv_sec + myTc.tv_nsec / 1E9;
		mutex_time.unlock();
		if (lastTime > 0.)
			cutrentDT = currentTime - lastTime;

//		if (kbKey == 'd') {
		//	  printf ("R[%.6f]   ", currentTime);
		//  lastTime = currentTime;
		//	}


    if (retVal == -1) {
			// T B D
			//m_qineticConnectionActive = false;
		} else {



			if (retVal >= qin_headerSize) {


				nRcvDataInBuff = 0; // TBD
				memcpy(&(rxBuf[nRcvDataInBuff]), bufTmp, retVal);
				nRcvDataInBuff += retVal;

				if (dbgKey == 'r' || dbgKey == 'f') {
					lastTime = currentTime;
					printf("[%.6f]Read: ", currentTime);
					for (int i = 0; i < retVal; i++) {
						printf("%02X   ", rxBuf[i]);
					}
					printf("\n");
				}

				slen = 0;
				retVal = 0;

				ParseData();
			} // if (slen
		} // if

		sleep (0.001);

	} // while
	printf("Comm4RcvThread was therminated\n");

}

unsigned short CLLI_Ctrl::JausRealToUShort (short realVal, short lowerLimit, short upperLimit)
{
   unsigned long usVal = 0;

   usVal = (unsigned short)((realVal - lowerLimit) * ((double)(TWO_EXP_16 - 1) / (upperLimit - lowerLimit)));
//   usVal = (realVal - lowerLimit) * ((2E16 - 1) / (upperLimit - lowerLimit));

   printf ("JausRealToUShort: %d --> %ld\n", realVal, usVal);

   return (unsigned short) usVal;

}


short CLLI_Ctrl::JausRealToShort (short realVal, short lowerLimit, short upperLimit)
{
   short usVal = 0;

   usVal = (short)((realVal - (upperLimit + lowerLimit) /2.) *
		   2. * ( ((double)(TWO_EXP_16/2 - 1)) / (upperLimit - lowerLimit) ));

   return usVal;

}

void CLLI_Ctrl::DriveControlRequest ()
{
    m_DriveCurrentState.reqState = lli_State_Ready;

}


void CLLI_Ctrl::ManipulatorControlRequest ()
{
    m_ManipulatorCurrentState.reqState = lli_State_Ready;
}


void CLLI_Ctrl::DriveControlRelease ()
{
   reqDevCtrlRelease[lli_Ctrl_Drive] = true;
}


void CLLI_Ctrl::ManipulatorControlRelease ()
{
   reqDevCtrlRelease[lli_Ctrl_Manip] = true;

}

LLI_STATE CLLI_Ctrl::GetDriveCurrentState ()
{
	return m_DriveCurrentState.currState;

}

LLI_STATE CLLI_Ctrl::GetManipulatorCurrentState ()
{
	return m_ManipulatorCurrentState.currState;

}
