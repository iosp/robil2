/*
 * ShareUtil.h
 *
 *  Created on: Jul 14, 2015
 *      Author: robil
 */

#ifndef SHAREUTIL_H_
#define SHAREUTIL_H_

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>


short GetShort (unsigned char * a_pcBuff, short * a_pOffset, bool LitleEndian);
short GetShort (unsigned char * a_pcBuff, bool LitleEndian);
long  GetLong (unsigned char * a_pcBuff, bool LitleEndian);
long  GetLong (unsigned char * a_pcBuff, short * a_pOffset, bool LitleEndian);
void  PutShort (unsigned char * a_pcBuff, short * a_pOffset, short a_nValue, bool LitleEndian);
void  PutLong (unsigned char * a_pcBuff, short * a_pOffset, short a_nValue, bool LitleEndian);

int   Kb_hit(void);


#endif /* SHAREUTIL_H_ */
