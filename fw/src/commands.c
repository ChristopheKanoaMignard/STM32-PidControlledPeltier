/**
  ******************************************************************************
  * @file           : commands.c
  * @brief          : Code to handle interactions with the host computer
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

extern uint8_t txBuf[];
extern uint8_t rxBuf[];
extern uint16_t adcBuf[];
extern uint32_t rxLen;
extern int16_t rawTiltFull[], smoothTiltFull[];
extern uint16_t fullIndex;

void Parse()
{
	int num, reg, val = 0;
	uint16_t txLen = 0;
	uint16_t numsPerBuf = 8;

	rxBuf[rxLen] = 0; //make sure the string is terminated with '\0'
	switch (rxBuf[0]) {
	case 'w': //write a register
		sscanf((char *)&rxBuf[1], "%x=%x\n", &reg, &val);
    	if (reg >= RegLast)
    	{
        	txLen = sprintf((char *)txBuf, "w%x=??, invalid reg #\n", reg);
    	}
    	else
    	{
        	SetReg(reg, val);
        	txLen = sprintf((char *)txBuf, "w%x=%x\n", reg, Regs.u16[reg]);
    	}
    	CDC_Transmit_HS(txBuf, txLen);
    	break;
	case 'r': //read a register
		sscanf((char *)&rxBuf[1], "%x\n", &reg);
    	if (reg >= RegLast)
    	{
        	txLen = sprintf((char *)txBuf, "r%x=??, invalid reg #\n", reg);
    	}
    	else
    	{
        	ReadReg(reg);
        	txLen = sprintf((char *)txBuf, "r%x=%x\n", reg, Regs.u16[reg]);
    	}
    	CDC_Transmit_HS(txBuf, txLen);
    	break;
	case 's': //write a nonvolatile parameter
		sscanf((char *)&rxBuf[1], "%x=%x\n", &reg, &val);
    	if (reg >= NvLast)
    	{
        	txLen = sprintf((char *)txBuf, "s%x=??, invalid reg #\n", reg);
    	}
    	else
    	{
        	Set_Parameter(reg, val);
        	txLen = sprintf((char *)txBuf, "s%x=%x\n", reg, SysParams.u16[reg]);
    	}
    	CDC_Transmit_HS(txBuf, txLen);
    	break;
	case 'g': //read a nonvolatile parameter
		sscanf((char *)&rxBuf[1], "%x\n", &reg);
    	if (reg >= NvLast)
    	{
        	txLen = sprintf((char *)txBuf, "g%x=??, invalid reg #\n", reg);
    	}
    	else
    	{
        	txLen = sprintf((char *)txBuf, "g%x=%x\n", reg, SysParams.u16[reg]);
    	}
    	CDC_Transmit_HS(txBuf, txLen);
    	break;
	case 'f': //flash all the current nonvolatile registers
		txLen = sprintf((char *)txBuf, "f\n");
    	CDC_Transmit_HS(txBuf, txLen);
    	break;
/*		
    case 'y': //send next portion of full tilt data buffer. Buffer is 4096 int16_t words, but txBuf is only 2048 unit8_t long, so need to call this 4 times
		//Had issues with sending binary words (kept getting read error), so switched to decimal. Even then, if numsPerBuf is > 8, get read timeout.
		txLen = 0;
    	for (int i = 0; i < numsPerBuf; ++i)
    	{
        	if (i == (numsPerBuf - 1))
            	txLen += sprintf((char *)&txBuf[txLen], "%d\n", smoothTiltFull[fullIndex++]);
        	else
            	txLen += sprintf((char *)&txBuf[txLen], "%d,", smoothTiltFull[fullIndex++]);
    	}
    	CDC_Transmit_HS(txBuf, txLen);
    	break;
	case 'z': //send next portion of full tilt data buffer. Buffer is 4096 int16_t words, but txBuf is only 2048 unit8_t long, so need to call this 4 times
		//Had issues with sending binary words (kept getting read error), so switched to decimal. Even then, if numsPerBuf is > 8, get read timeout.
		txLen = 0;
    	for (int i = 0; i < numsPerBuf; ++i)
    	{
        	if (i == (numsPerBuf - 1))
            	txLen += sprintf((char *)&txBuf[txLen], "%d\n", rawTiltFull[fullIndex++]);
        	else
            	txLen += sprintf((char *)&txBuf[txLen], "%d,", rawTiltFull[fullIndex++]);
    	}
    	CDC_Transmit_HS(txBuf, txLen);
		break;
*/
	default:
		txLen += sprintf((char *)&txBuf[txLen], "not a valid input\n");
		CDC_Transmit_HS(txBuf, txLen);
		break;
		
	}
	rxLen = 0;
}

