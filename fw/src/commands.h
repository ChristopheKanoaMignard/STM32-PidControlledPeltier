/**
  ******************************************************************************
  * @file           : commands.h
  * @brief          : Code to handle interactions with the host computer
  ******************************************************************************
  */
#ifndef __COMMANDS_H__
#define __COMMANDS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

extern uint8_t txBuf[];
extern uint8_t rxBuf[];
extern uint32_t rxLen;

void Parse();

#ifdef __cplusplus
}
#endif

#endif // __COMMANDS_H__