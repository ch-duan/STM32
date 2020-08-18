/*
 * iap.h
 *
 *  Created on: Jul 28, 2020
 *      Author: ch
 */

#ifndef INC_IAP_H_
#define INC_IAP_H_
#include "main.h"
//((uint32_t)0x08000000)-((uint32_t)0x08010000)为bootLoader空间
#define APP_Flash_ADDR ((uint32_t)0x8008000)
#define APP_Flashh_END_ADDR ((uint32_t)0x8008800)
#define APP1STARTADDR ((uint32_t)0x08010000)
#define APP1ENDADDR ((uint32_t)0x0803FFFF)
#define APP2STARTADDR ((uint32_t)0x08010000)
#define APP2ENDADDR ((uint32_t)0x0803FFFF)
#define FLASH_PAGE_NBPERBANK  256

enum {
	FLASHIF_PROTECTION_NONE = 0,
	FLASHIF_PROTECTION_PCROPENABLED = 0x1,
	FLASHIF_PROTECTION_WRPENABLED = 0x2,
	FLASHIF_PROTECTION_RDPENABLED = 0x4,
};
enum {
	FLASHIF_OK = 0,
	FLASHIF_ERASEKO,
	FLASHIF_WRITINGCTRL_ERROR,
	FLASHIF_WRITING_ERROR,
	FLASHIF_PROTECTION_ERRROR
};
enum {
	FLASHIF_WRP_ENABLE, FLASHIF_WRP_DISABLE
};

typedef void (*pFunction)(void);
void jump2APP(uint32_t addr);
void FLASH_If_Init(void);
uint32_t FLASH_If_Erase(uint32_t StartAddr, uint32_t EndAddr);
uint32_t FLASH_If_GetWriteProtectionStatus(uint32_t addr);
uint32_t FLASH_If_Write_fast(uint32_t StartAddr, uint32_t *p_source,
		uint32_t length);
uint32_t FLASH_If_Write(uint32_t destination, uint32_t endAddr,
		uint64_t *p_source, uint32_t length);
uint32_t FLASH_If_WriteProtectionConfig(uint32_t protectionstate);

#endif /* INC_IAP_H_ */
