#ifndef STM32F407XX_FLASH_DRIVER_H
#define STM32F407XX_FLASH_DRIVER_H

#include "stm32f407xx.h"

// Flash interface registers bits
#define FLASH_ACR_LATENCY 	0
#define FLASH_ACR_PRFTEN		8
#define FLASH_ACR_ICEN			9
#define FLASH_ACR_DCEN			10
#define FLASH_ACR_ICRST			11
#define FLASH_ACR_DCRST			12

#define FLASH_SR_EOP				0
#define FLASH_SR_OPERR			1
#define FLASH_SR_WRPERR			4
#define FLASH_SR_PGAERR			5
#define FLASH_SR_PGPERR			6
#define FLASH_SR_PGSERR			7
#define FLASH_SR_RDERR			8

#define FLASH_CR_PG					0
#define FLASH_CR_SER				1
#define FLASH_CR_MER				2
#define FLASH_CR_SNB				3
#define FLASH_CR_PSIZE			4
#define FLASH_CR_STRT				5
#define FLASH_CR_EOPIE			6
#define FLASH_CR_ERRIE			7
#define FLASH_CR_LOCK				8

#define FLASH_OPTCR_OPTLOCK			0
#define FLASH_OPTCR_OPTSTRT			1
#define FLASH_OPTCR_BOR_LEV			2
#define FLASH_OPTCR_WDG_SW			5
#define FLASH_OPTCR_nRST_STOP		6
#define FLASH_OPTCR_nRST_STDBY	7
#define FLASH_OPTCR_RDP					8
#define FLASH_OPTCR_nWRP				16

#endif