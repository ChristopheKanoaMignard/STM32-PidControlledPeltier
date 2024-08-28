#include "main.h"

#define PARAM_VERSION 1
#define START_WORD 0xA9

//The flash sector for params is Sector 9 (128kbytes), which is the last sector for STM32F411xE
//TODO: Probably the linker should be told that this not a valid sector for program code so an
//error will be generated if the linker tries to use it. Since this is a large sector, should probably
//use Sector 0 instead, and tell linker to put code starting at Sector 1.
#define PARAM_START 0x080c0000
#define PARAM_END 0x080cffff
#define PARAM_PAGE_SIZE 512

REG_BLOCK SysParams;
REG_BLOCK DefaultParams;

void Set_Defaults() {
    // Defaults 
	DefaultParams.u16[NvStart] = 0x55a0;   //just a dummy parameter for now (this code doesn't need nonvolatile parameters)
}

int Verify_Params(REG_BLOCK *pBlock, uint8_t ver) {

    if (pBlock->start!=START_WORD) return 0;
    if (pBlock->ver!=ver && ver!=255) return 0;

    uint8_t chksum = 0;
    for (int i=0; i<REG_SIZE8; i++) {
        chksum+=pBlock->u8[i];
    }
    if (chksum!=pBlock->chksum) return 0;

    return 1;
}

int Not_Erased(uint32_t addr) {

    for (int i=0; i<REG_SIZE; i+=4) {
        if (*(volatile uint32_t *)(addr+i) != 0xFFFFFFFF) return 1;
    }

    return 0;
}

REG_BLOCK *Get_Latest(uint8_t ver) {
    REG_BLOCK *pLatestBlock=NULL;
    REG_BLOCK *pSmallLatestBlock=NULL;
    uint8_t maxSeq = 0;
    uint8_t smallMaxSeq = 0;

    for(int i=PARAM_START; i<PARAM_END; i+=REG_SIZE) {
        REG_BLOCK *pBlock = (REG_BLOCK *)i;
        if (Verify_Params(pBlock,ver)) {
            if (pBlock->seq>maxSeq) {
                maxSeq = pBlock->seq;
                pLatestBlock = pBlock;
            }
            if (pBlock->seq>smallMaxSeq && pBlock->seq<128) {
                smallMaxSeq = pBlock->seq;
                pSmallLatestBlock = pBlock;
            }
        }
    }

    if (maxSeq>224 && smallMaxSeq!=0) { // We have wrapped around in seq
        return pSmallLatestBlock;
    } else {
        return pLatestBlock;
    }
}

HAL_StatusTypeDef FLASH_ErasePage(uint32_t writeAddr) {
	FLASH_EraseInitTypeDef EraseInit;
	EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS; 
	EraseInit.Sector = FLASH_BANK_1;
	EraseInit.Sector = FLASH_SECTOR_7; 
	//EraseInit.Sector = 1; 
	EraseInit.Sector = FLASH_VOLTAGE_RANGE_3;
	
    // Routine for the STM32F405
    // We're going to use the otherwise unused FLASH_ERROR_PGA to indicate a bad address
    if (writeAddr<0x8000000) {
        return FLASH_ERROR_PGA;
    } else if (writeAddr>=0x8000000 && writeAddr<0x8003FFF) {
	    EraseInit.Sector = FLASH_SECTOR_0;
    } else if (writeAddr>=0x8004000 && writeAddr<0x8007FFF) {
	    EraseInit.Sector = FLASH_SECTOR_1;
    } else if (writeAddr>=0x8008000 && writeAddr<0x800BFFF) {
	    EraseInit.Sector = FLASH_SECTOR_2;
    } else if (writeAddr>=0x800C000 && writeAddr<0x800FFFF) {
	    EraseInit.Sector = FLASH_SECTOR_3;
    } else if (writeAddr>=0x8010000 && writeAddr<0x801FFFF) {
	    EraseInit.Sector = FLASH_SECTOR_4;
    } else if (writeAddr>=0x8020000 && writeAddr<0x803FFFF) {
	    EraseInit.Sector = FLASH_SECTOR_5;
    } else if (writeAddr>=0x8040000 && writeAddr<0x805FFFF) {
	    EraseInit.Sector = FLASH_SECTOR_6;
    } else if (writeAddr>=0x8060000 && writeAddr<0x807FFFF) {
	    EraseInit.Sector = FLASH_SECTOR_7;
//    } else if (writeAddr>=0x8080000 && writeAddr<0x809FFFF) {
//	    EraseInit.Sector = FLASH_SECTOR_8;
//    } else if (writeAddr>=0x80A0000 && writeAddr<0x80BFFFF) {
//	    EraseInit.Sector = FLASH_SECTOR_9;
//    } else if (writeAddr>=0x80C0000 && writeAddr<0x80DFFFF) {
//	    EraseInit.Sector = FLASH_SECTOR_10;
//    } else if (writeAddr>=0x80E0000 && writeAddr<0x80FFFFF) {
//	    EraseInit.Sector = FLASH_SECTOR_11;
    } else {
        return FLASH_ERROR_PGA;
    }
	uint32_t SectorError;  //this could potentially be used for error diagnosis
	return (HAL_FLASHEx_Erase(&EraseInit, &SectorError));
}

int Write_Block(REG_BLOCK *pBlock) {

    pBlock->start = START_WORD;
    pBlock->ver = PARAM_VERSION;
    pBlock->chksum = 0;
    for (int i=0; i<REG_SIZE8; i++) {
        pBlock->chksum+=pBlock->u8[i];
    }

    uint32_t writeAddr = 0L;
    REG_BLOCK *pLatestBlock = Get_Latest(255); // get latest block of any version

    if (pLatestBlock!=NULL) {
        writeAddr = ((uint32_t)pLatestBlock) + REG_SIZE;
        if (writeAddr >= PARAM_END) {
            writeAddr = PARAM_START;
        }
        pBlock->seq = pLatestBlock->seq==255 ? 1: pLatestBlock->seq+1; // Wrap to 1 rather than 0
    } else {
        writeAddr = PARAM_START;
        pBlock->seq = 1;
    }

    // Verify that we have a clean block to write or we are on a page boundary
    while (Not_Erased(writeAddr) && writeAddr%PARAM_PAGE_SIZE) {
        writeAddr+=REG_SIZE;
        if (writeAddr >= PARAM_END) {
            writeAddr = PARAM_START;
        }
    }

	HAL_StatusTypeDef FLASHStatus;
    uint32_t *wrtPtr = (uint32_t *)pBlock;

	HAL_FLASH_Unlock();
    if (writeAddr%PARAM_PAGE_SIZE==0) { // we are on a new page, erase it
       FLASHStatus = FLASH_ErasePage(writeAddr);
	    if (FLASHStatus != HAL_OK) {
	       HAL_FLASH_Lock();
            return 1;
        }
    }

    for (int i=0; i<(REG_SIZE/4); i++) {
	    FLASHStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, writeAddr, wrtPtr[i]);

        if (*(volatile uint32_t *)writeAddr != wrtPtr[i]) {
	        HAL_FLASH_Lock();
            return 1;
        }

	    if (FLASHStatus != HAL_OK) {
	        HAL_FLASH_Lock();
            return 1;
        }

        writeAddr+=4;
    }

	HAL_FLASH_Lock();
    return 0;
}

int Flash_Params() {
    int status = Write_Block((REG_BLOCK *)&SysParams);
    return (status);
}

void Load_Params(uint8_t isForceDefault) {

    Set_Defaults();

    REG_BLOCK *pBlock = Get_Latest(255);
    if (pBlock != NULL  && (isForceDefault == 0)) {
        for (int i=0; i<REG_SIZE32; i++) {
            SysParams.u32[i] = pBlock->u32[i];
        }

    } else {
        for (int i=0; i<REG_SIZE32; i++) {
            SysParams.u32[i] = DefaultParams.u32[i];
        }
    }
}

void Set_Parameter(uint16_t nParam, uint16_t value) {

    if (nParam>=REG_SIZE16) return;

    switch (nParam) {
    case NvStart:
        SysParams.u16[nParam] = value;
        break;
    default:
        SysParams.u16[nParam] = value;
        break;
    }
}


