#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>
#include <cstring>
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <boost/thread/mutex.hpp>

using namespace std;

#define BUFLEN 256  //Max length of buffer
#define BYTE unsigned char

const int qin_headerSize = 16;

const BYTE qin_lli_MsgProp[] = { 0x06, 0x02 };
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

const unsigned short lli_ManipulatorCtrlCode = 1284;


// Bytes 2-3 (zero based) - message ID
// Bytes 14-15 - the short message counter (sequence mesage)
#ifndef JAUS_BIG_ENDIAN
#define JAUS_BIG_ENDIAN
#endif
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

enum connectionState{init, waitToResponse, responsed};

class simQinetiqClient{

  float Throttel;
  float Steering;

public:

  simQinetiqClient(){}
  simQinetiqClient(simQinetiqClient const &) = delete;
  void operator=(simQinetiqClient const&) = delete;

  float getThrottel(){
    float temp;
    mutex_Throttel.lock();
    temp = Throttel;
    mutex_Throttel.unlock();
    return temp;
  }
  float getSteering(){
    float temp;
    mutex_Steering.lock();
    temp = Steering;
    mutex_Steering.unlock();
    return temp;
  }

  bool commConnect(string IP, int udpLP, int udpRP)
  {
    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 1600;

    if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
            printf("Socket create error\n");
            return false;
    }

    memset((char *) &si_Local, 0, sizeof(si_Local));
    si_Local.sin_family = AF_INET;
    si_Local.sin_port = htons(udpLP);
    si_Local.sin_addr.s_addr = htonl(INADDR_ANY);

    if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO,
                    reinterpret_cast<char*>(&tv), sizeof(timeval))) {
            printf("setsockopt() failed\n");
            return false;
    }
    int retV = bind(s, (struct sockaddr *) &si_Local, sizeof(si_Local));
    if (retV == -1) {
            perror("bind error\n");
            return false;
    }

    memset((char *) &si_Remote, 0, sizeof(si_Remote));
    si_Remote.sin_family = AF_INET;
    si_Remote.sin_port = htons(udpRP);

    if (inet_aton(IP.c_str(), &si_Remote.sin_addr) == 0) {
            printf("simQinetiqClient: inet_aton() failed\n");
            return false;
    }
    return true;
  }

  float reverseShortJausToReal(float realVal, float lowerLimit, float upperLimit)
  {
    float usVal = 0;
    usVal = (realVal * (upperLimit-lowerLimit) /((double)(65536./2. - 1) *2.)) + ((upperLimit+lowerLimit)/2.);
    return usVal;
  }

  short GetShort (unsigned char * a_pcBuff, bool LitleEndian)
  {
     short l_Short = 0x0000;

     if (!LitleEndian) {
             l_Short += *a_pcBuff;
             //*a_pcBuff++;
             l_Short <<= 8;
             l_Short += *(a_pcBuff+1);
     }
     else {
             l_Short += *(a_pcBuff+1);
             l_Short <<= 8;
             l_Short += *a_pcBuff;
    }
  }

  void HeartBeatFunc()
  {
    while(true)
    {
        sendto(s, QIN_QueryHeartbeatMsg, sizeof(QIN_QueryHeartbeatMsg), 0, (struct sockaddr *) &si_Remote, sizeof(si_Remote));
        switch (HBRespons) {
          case waitToResponse:
            printf("simQinetiqClient: no HeartBeat\n");

            //remark this line if you want one time message
            //HBRespons = init;
            break;
          case responsed:
            HBRespons = waitToResponse;
          default:
            break;
          }
        sleep(5);
    }
  }

  void safeThreadFunc()
  {
    struct timespec currentTime;
    double lastUpdateTime;
    double currTime;

    while(true)
    {
      clock_gettime(CLOCK_REALTIME, &currentTime);
      currTime = currentTime.tv_sec + currentTime.tv_nsec / 1E9;
      lastUpdateTime = lastUpdateOfCommands.tv_sec + lastUpdateOfCommands.tv_nsec / 1E9;
      if(currTime > lastUpdateTime + 0.5)
      {
        Throttel = 0;
        Steering = 0;
      }
      sleep(0.1);
    }
  }

  static void* HeratBeatThread(void* p)
  {
    simQinetiqClient *myHandle = (simQinetiqClient*) (p);

    myHandle->HeartBeatFunc();
  }

static void* safeThread(void* p)
{
  simQinetiqClient *myHandle = (simQinetiqClient*) (p);

  //myHandle->safeThreadFunc();
}

//limit temp to +-1
void limitToAbsOne(float& temp)
{
  if(temp > 1)       { temp =  1;          }
  else if(temp < -1) { temp = -1;          }
}
  void mainThreadFunc()
  {
    BYTE msgHeader[qin_headerSize];
    short shTemp = 0;
    float fTemp = 0;
    while (true)
    {
        memset(buf,'\0', BUFLEN);
        if (recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_Local, &slen) == -1)
        {
           //printf("simQinetiqClient: simQinetiqClient: ERROR: recvFunc\n");
        }
        else //data recieved
        {
            memcpy(&msgHeader, &(buf[0]), sizeof(qin_headerSize));
            if (memcmp(&qin_lli_MsgProp, msgHeader, 2) == 0)
            {
              unsigned short codeTmp = GetShort(&msgHeader[2], false);
              switch(codeTmp)
              {
                case qin_DriveCtrlCode:
                  //send response msg
                  sendto(s, QIN_DriveControlConfirmMsg, sizeof(QIN_DriveControlConfirmMsg), 0, (struct sockaddr *) &si_Remote, sizeof(si_Remote));
                  printf("QIN_DriveControlConfirmMsg\n");
                  break;
                case qin_ManipulatorCtrlCode:
                  sendto(s, QIN_ManipulatorControlConfirmMsg, sizeof(QIN_ManipulatorControlConfirmMsg), 0, (struct sockaddr *) &si_Remote, sizeof(si_Remote));
                  printf("qin_ManipulatorCtrlCode\n");
                  break;
                case lli_CtrlCmdId:
                  //send controlConfirmMsg for the driving and manipulation, otherwise the LLI will not move to ready state.
                  sendto(s, QIN_DriveControlConfirmMsg, sizeof(QIN_DriveControlConfirmMsg), 0, (struct sockaddr *) &si_Remote, sizeof(si_Remote));
                  sendto(s, QIN_ManipulatorControlConfirmMsg, sizeof(QIN_ManipulatorControlConfirmMsg), 0, (struct sockaddr *) &si_Remote, sizeof(si_Remote));
                  break;
                case lli_SetWrenchEffortId:
                  //recieve throttel and steering commands
                  shTemp = 0;
                  fTemp = 0;
                  memcpy(&shTemp, &(buf[18]), sizeof(short));
                  fTemp = reverseShortJausToReal(shTemp, -100, 100)/100.0;
                  limitToAbsOne(fTemp);
                  mutex_Throttel.lock();
                  Throttel = fTemp;
                  mutex_Throttel.unlock();
                  shTemp = 0;
                  fTemp = 0;
                  memcpy(&shTemp, &buf[20], sizeof(short));
                  fTemp  = reverseShortJausToReal(shTemp, -100, 100)/100.0;
                  limitToAbsOne(fTemp);
                  mutex_Steering.lock();
                  Steering = fTemp;
                  mutex_Steering.unlock();
                  //update last time update for saftyThread
                  clock_gettime(CLOCK_REALTIME, &lastUpdateOfCommands);

                  break;
                case lli_HeartBeatConfirmId:
                  //response for HB - do nothing
                  HBRespons = responsed;
                  //printf("lli_HeartBeatConfirmId\n");
                  break;
                case qin_CtrlRejectId:
                  printf("qin_CtrlRejectId\n");
                  break;
                case lli_CtrlReleaseId:
                  printf("lli_CtrlReleaseId\n");
                  break;
                case lli_SetJointEffortId:
                  //manipulation commands
//                  temp = 0;
//                  memcpy(&temp, &(buf[17]), sizeof(unsigned short));
//                  if(temp != 0)
//                  printf("(reqJoints_Val[0] = %u, ", temp);
//                  temp = 0;
//                  if(temp != 0)
//                  memcpy(&temp, &buf[19], sizeof(short));
//                  printf("(reqJoints_Val[1] = %u\n", temp);
                  break;
                default:
                  printf("default :-(\n");
                  break;
              }
            }
            else
              {
                printf("simQinetiqClient: ERROR in the header\n");
              }
        }
    }
  }

  static void* mainThread(void * p)
  {
    simQinetiqClient *myHandle = (simQinetiqClient*) (p);

    pthread_t HBThread;
    pthread_create(&HBThread, NULL, &HeratBeatThread, myHandle);

    pthread_t SaftyThread;
    pthread_create(&SaftyThread, NULL, &safeThread, myHandle);

    myHandle->mainThreadFunc();
  }

  void Init(string IP, int udpLP, int udpRP)
  {
      Throttel=0;
      Steering=0;
      commConnect(IP, udpLP, udpRP);
      slen=sizeof(si_Local);
      HBRespons = init;

      pthread_t recvFunc;
      pthread_create(&recvFunc, NULL, &mainThread, this);
  }

  struct sockaddr_in si_Local;
  struct sockaddr_in si_Remote;
  int s, i;
  socklen_t slen;
  unsigned char buf[BUFLEN];
  connectionState HBRespons;
  struct timespec lastUpdateOfCommands;

  private: boost::mutex mutex_Throttel;
  private: boost::mutex mutex_Steering;


};
