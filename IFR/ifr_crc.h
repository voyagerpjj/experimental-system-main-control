/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025,  China,  IFR Laboratory, DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_usart.h
  * Version		: v3.0
  * Author		: LiuHao Lijiawei Albert panjiajun
  * Date			: 2025-11-10
  * Description	:
  *********************************************************************
  */

#ifndef __IFR_CRC_H_
#define __IFR_CRC_H_
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif
#include "main.h"

uint16_t CalcCRC_Modbus(uint8_t MsgToCRC[], int MsgLenToCRC);

#endif
