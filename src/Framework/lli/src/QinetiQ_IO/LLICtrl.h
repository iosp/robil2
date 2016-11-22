//#pragma once

#ifndef _LLICTRL_H_
#define _LLICTRL_H_


// #include <thread>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <stdio.h>
//#include <caca_conio.h>
#include <pthread.h>
#include <string>
#include <cstring>
#include <unistd.h>
#include <time.h>
#include<sys/socket.h>
#include <netinet/in.h>
#include<arpa/inet.h>

#include "QinetiQ_ICD.h"
#include "ShareUtil.h"
// #include "Share_QueryPerformanceTime.h"

#pragma pack (push, 1)

using namespace std;

//const WORD   rxBufLen = 256;
#define   TWO_EXP_16 65536

const unsigned short   	MsgBufMax = 128;
const double   			_epsilon_E9 = 1E9;
const double   			_epsilon_E6 = 1E6;
const double   			_epsilon_E3 = 1E3;

const short			TX_SPOOLER_SIZE = 5;
const short			TX_MSGID_MAX = 7;

typedef enum { lli_State_Off = 0, lli_State_Init, lli_State_Standby, lli_State_Wait_Responce, lli_State_Ready } LLI_STATE;
typedef enum { lli_RR__None = 0, lli_RR_Confirm, lli_RR_Reject } LLI_COTROL_SWITCH;
typedef enum { lli_Ctrl_Drive = 0, lli_Ctrl_Manip } LLI_CONTROL_DEVICE;

enum { lli_TransmitResponceHeartBit_Msg, lli_TransmitWrenchEffort_Msg, lli_TransmitJoinEffort_Msg, lli_TransmitDriveCtrl_Msg,
	lli_TransmitManipulatorCtrl_Msg, lli_TransmitReleaseDriveCtrl_Msg, lli_TransmitReleaseManipulatorCtrl_Msg
};


struct LLI_SM {
	LLI_CONTROL_DEVICE		ctrlDevId;
	LLI_STATE 				currState;
	LLI_STATE 				reqState;
	//unsigned short      	respRequest;	// responce request
//	unsigned short      	devResponce;	// responce request
	LLI_COTROL_SWITCH    	respRequest;
	LLI_COTROL_SWITCH       devResponce;
	double					timeTag;
	double					effortTT;
	double					lastCmdTT;
};

struct LLI_TX_SPOOLER {
	unsigned short msgIdBuffer[TX_SPOOLER_SIZE];
	unsigned short setId;
	unsigned short getId;
	unsigned short num;
};

class CLLI_Ctrl {
public:
	CLLI_Ctrl ();
	~ CLLI_Ctrl ();

private:

   unsigned char	m_IsTerminateThread;

   // UDP IP, local port number and remote port number
   char udpIP[16]; 
   int  udpLP, udpRP;

  // LLI_STATE		m_currState;
   unsigned long	rxCount;
   unsigned long	txCount;
   unsigned short	rxSeqNumber;
   unsigned short	txSeqNumber;
   unsigned char	rxBuf[MsgBufMax];
   unsigned char	txBuf[MsgBufMax];
   unsigned char	rxBufShift;

   unsigned short	nRcvDataInBuff;

   //TimeTags for synchronization control (TBD)
   double       	m_rxTT;
   double       	m_currTT;
   double			m_txTT;

   double			m_TrottelReqTT;
   double			m_SteeringReqTT;
   double			m_JointReqTT;


   bool			m_qineticConnectionActive;
   bool			m_HeartBeatResponseReq;
   bool			m_stateReq;
   bool			m_txReady;
   bool			m_txDone;

   LLI_SM		m_DriveCurrentState;
   LLI_SM		m_ManipulatorCurrentState;

   short reqThrottel_Val;
   short reqSteering_Val;
   short reqJoints_Val[2];

   bool  reqDevCtrlRelease[2];

   int 			        socketFd;
   struct sockaddr_in   si_Remote;
   struct sockaddr_in   si_Local;

   unsigned short	ctrCodeWait2Responce[2];

   LLI_TX_SPOOLER   m_TxSpooler;

public:

    bool		Init (char *addr, unsigned int lPortID, unsigned int rPortID);

	bool		Reset ();
	int		    ReceiveData ();
	bool		TransmitData (void *buf, short bufSize);
	bool		TransmitData (short bufSize);
	bool		CommandPreparation (unsigned long cnt);
	bool		ResponcePreparation ();
	bool		StateMachineSwitch ();
	bool        StateMachineSwitch(LLI_CONTROL_DEVICE devCtrl);
	bool        StateMachineSwitch(LLI_SM * devCtrlSM);

	bool		PeriodicActivity ();

	bool		CommConnect ();
	bool        CommThreadCreate ();
	bool        TimerCreate ();

    void        SetCurrentTimeTag ();
    double      GetCurrentTimeTag () { return m_currTT; }
    double      GetLastRxTimeTag () { return m_rxTT; }
    void        ResetLocalTimeTag (double &l_tt);

	unsigned short	ParseData ();
	bool		MsgVerification (unsigned short msgId);
	void		CheckConnectionActive (bool valid, unsigned short msgCount);

	bool        GetQinetiqActived ();

	// Set Control/Release Devices by ROS messages
	LLI_STATE   GetDriveCurrentState ();
	LLI_STATE   GetManipulatorCurrentState ();
	void        DriveControlRequest ();
	void        ManipulatorControlRequest ();
	void        DriveControlRelease ();
	void        ManipulatorControlRelease ();

	void        SetThrottelRequest (short reqVal);
	void        SetSteeringRequest (short reqVal);
	void        SetJointRequest (short reqVal1, short reqVal2);
	void        ResetEffortRequest (LLI_CONTROL_DEVICE devCtrl);
	void        ResetThrottelRequest ();
	void        ResetSteeringRequest ();
	void        ResetJointRequest ();

	//void        SetDrvCtrlReleaseRequest () { reqDevCtrlRelease[lli_Ctrl_Drive] = true; }
	//void        SetManipCtrlReleaseRequest () { reqDevCtrlRelease[lli_Ctrl_Manip] = true; }



private:
	void		RxBufferInit ();
	void		TxBufferInit ();
    static void *	Comm4RcvThread (void *pThis);
    void	    ThreadFunc ();

    void        TransmitResponceHeartBit ();
    void        TransmitWrenchEffortMsg ();
    void        TransmitJoinEffortMsg ();
    void        TransmitDriveCtrlMsg (unsigned short val);
    void        TransmitManipulatorCtrlMsg (unsigned short val);
    void        TransmitReleaseDriveCtrlMsg ();
    void        TransmitReleaseManipulatorCtrlMsg ();

    void        SetReceiveTimeTag ();

    bool        SetDeviceRequest (LLI_CONTROL_DEVICE devCtrl, LLI_STATE stateReq);

    unsigned short  RequestComponentControl (LLI_CONTROL_DEVICE dev2Ctrl);
    unsigned short  RequestComponentRelease (LLI_CONTROL_DEVICE dev2Ctrl);

    bool            SetEffortControl (LLI_CONTROL_DEVICE dev2Ctrl);

    bool        SetMsgId_To_TxSpooler (unsigned short msgId);
    unsigned short  GetMsgId_From_TxSpooler ();
    unsigned short  GetMsgsNum_From_TxSpooler ();

	void            PritnOfStatePassed (LLI_SM * devSM);

	short			JausRealToShort (short realVal, short lowerLimit, short upperLimit);
	unsigned short	JausRealToUShort (short realVal, short lowerLimit, short upperLimit);

};

#pragma pack (pop)


#endif
