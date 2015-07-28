/*
 * ShareUtil.cpp
 *
 *  Created on: Jul 14, 2015
 *      Author: robil
 */


#include "ShareUtil.h"



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


short GetShort (unsigned char * a_pcBuff, short * a_pOffset, bool LitleEndian)
{
   short l_Short = 0x0000;

   if (!LitleEndian) {
	   l_Short += a_pcBuff[*a_pOffset];
	   *a_pOffset++;
	   l_Short <<= 8;
	   l_Short += a_pcBuff[*a_pOffset];
	   a_pOffset++;
   }
   else {
	   l_Short += a_pcBuff[*a_pOffset];
	   *a_pOffset++;
	   l_Short += a_pcBuff[*a_pOffset] <<= 8;
	   a_pOffset++;
   }
}

// TBD
short GetLong (unsigned char * a_pcBuff, short * a_pOffset, bool LitleEndian)
{
	// TBD
	return 0;
}

void  PutShort (unsigned char * a_pcBuff, short * a_pOffset, short a_nValue, bool LitleEndian)
{
	short valTmp = 0x0000;

	 if (!LitleEndian) {
		 valTmp = a_nValue & 0xFF00;
		 valTmp >>= 8;
		 a_pcBuff[*a_pOffset] = (unsigned char)valTmp;
		 *a_pOffset++;
		 valTmp = a_nValue & 0x00FF;
		 a_pcBuff[*a_pOffset] = (unsigned char)valTmp;
		 *a_pOffset++;
	 }
	 else {
		 valTmp = a_nValue & 0x00FF;
		 a_pcBuff[*a_pOffset] = (unsigned char)valTmp;
		 *a_pOffset++;
		 valTmp = a_nValue & 0xFF00;
		 valTmp >>= 8;
		 a_pcBuff[*a_pOffset] = (unsigned char)valTmp;
		 *a_pOffset++;
	 }

}

void  PutLong (unsigned char * a_pcBuff, short * a_pOffset, short a_nValue, bool LitleEndian)
{

}



int Kb_hit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}


