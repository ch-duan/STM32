/*
 * iap.c
 *
 *  Created on: Jul 28, 2020
 *      Author: ch
 */
#include "iap.h"
#include "main.h"
#include "stdio.h"

void jump2APP(uint32_t addr) {
	uint32_t JumpAddress;
	pFunction Jump_To_Application;

	//Check
	if (((*(__IO uint32_t*) addr) & 0x2FFE0000) == 0x20000000) {
		printf("APP Start...\n");
		// Jump to user application //
		HAL_SuspendTick();                                    //SysTick shutdown
		HAL_DeInit();                                         //Periphery DeInit
		HAL_RCC_DeInit();
		JumpAddress = *(__IO uint32_t*) (addr + 4);
		Jump_To_Application = (pFunction) JumpAddress;
		// Initialize user application's Stack Pointer //
		//__set_MSP(*(__IO uint32_t*) addr);
		//__DSB();
		__set_PRIMASK(1);
		Jump_To_Application();
	} else {
		printf("No APP found!!!\n");
	}
}

void FLASH_If_Init(void) {
	/* Unlock the Program memory */
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_FLASH_CLK_ENABLE();
	HAL_FLASH_Unlock();
	/* Clear all FLASH flags */
	__HAL_FLASH_CLEAR_FLAG(
			FLASH_FLAG_EOP | FLASH_FLAG_PGSERR | FLASH_FLAG_WRPERR | FLASH_FLAG_OPTVERR);
	/* Unlock the Program memory */
	HAL_FLASH_Lock();
}
uint32_t FLASH_If_Erase(uint32_t StartAddr, uint32_t EndAddr) {
	FLASH_EraseInitTypeDef pEraseInit;
	HAL_StatusTypeDef status = HAL_OK;
	uint32_t PageError = 0;
	HAL_FLASH_Unlock();
	if (StartAddr >= FLASH_BASE && EndAddr <= FLASH_BANK1_END) {
		pEraseInit.Banks = FLASH_BANK_1;
	} else if (StartAddr > FLASH_BANK1_END && EndAddr < FLASH_BANK2_END) {
		pEraseInit.Banks = FLASH_BANK_2;
	}
	pEraseInit.Page = (StartAddr - FLASH_BASE) / FLASH_PAGE_SIZE;
	pEraseInit.NbPages = (EndAddr - FLASH_BASE) / FLASH_PAGE_SIZE
			- pEraseInit.Page;
	pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	status = HAL_FLASHEx_Erase(&pEraseInit, &PageError);
	HAL_FLASH_Lock();
	if (status != HAL_OK) {
		return HAL_ERROR;
	}
	return HAL_OK;
}

uint32_t FLASH_If_GetWriteProtectionStatus(uint32_t addr) {
	uint32_t ProtectedPAGE = FLASHIF_PROTECTION_NONE;
	FLASH_OBProgramInitTypeDef OptionsBytesStruct1, OptionsBytesStruct2,
			OptionsBytesStruct3, OptionsBytesStruct4;

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	OptionsBytesStruct1.WRPArea = OB_WRPAREA_BANK1_AREAA;
	OptionsBytesStruct1.PCROPConfig = FLASH_BANK_1;
	OptionsBytesStruct2.WRPArea = OB_WRPAREA_BANK1_AREAB;
	OptionsBytesStruct2.PCROPConfig = FLASH_BANK_1;
	OptionsBytesStruct3.WRPArea = OB_WRPAREA_BANK2_AREAA;
	OptionsBytesStruct3.PCROPConfig = FLASH_BANK_2;
	OptionsBytesStruct4.WRPArea = OB_WRPAREA_BANK2_AREAB;
	OptionsBytesStruct4.PCROPConfig = FLASH_BANK_2;

	/* Check if there are write protected sectors inside the user flash area ***/
	HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct1);
	HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct2);
	HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct3);
	HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct4);

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();

	/* Check PCROP areas */
	if (OptionsBytesStruct1.PCROPEndAddr > OptionsBytesStruct1.PCROPStartAddr) {
		/* check if user area are included inside this range */
		if (OptionsBytesStruct1.PCROPStartAddr > addr) {
			ProtectedPAGE |= FLASHIF_PROTECTION_PCROPENABLED;
		}
	}

	if (OptionsBytesStruct2.PCROPEndAddr > OptionsBytesStruct2.PCROPStartAddr) {
		/* check if user area are included inside this range */
		if (OptionsBytesStruct1.PCROPStartAddr > addr) {
			ProtectedPAGE |= FLASHIF_PROTECTION_PCROPENABLED;
		}
	}

	/* check WRP */
	if (OptionsBytesStruct1.WRPEndOffset > OptionsBytesStruct1.WRPStartOffset) {
		/* check if area is inside the WRP Range */
		if ((OptionsBytesStruct1.WRPStartOffset * FLASH_PAGE_SIZE + FLASH_BASE)
				>= addr) {
			ProtectedPAGE |= FLASHIF_PROTECTION_WRPENABLED;
		}
	}

	if (OptionsBytesStruct2.WRPEndOffset > OptionsBytesStruct2.WRPStartOffset) {
		/* check if area is inside the WRP Range */
		if ((OptionsBytesStruct2.WRPStartOffset * FLASH_PAGE_SIZE + FLASH_BASE)
				>= addr) {
			ProtectedPAGE |= FLASHIF_PROTECTION_WRPENABLED;
		}
	}

	if (OptionsBytesStruct3.WRPEndOffset > OptionsBytesStruct3.WRPStartOffset) {
		/* check if area is inside the WRP Range */
		if ((OptionsBytesStruct3.WRPStartOffset * FLASH_PAGE_SIZE + FLASH_BASE
				+ FLASH_PAGE_SIZE * FLASH_PAGE_NBPERBANK) >= addr) {
			ProtectedPAGE |= FLASHIF_PROTECTION_WRPENABLED;
		}
	}

	if (OptionsBytesStruct4.WRPEndOffset > OptionsBytesStruct4.WRPStartOffset) {
		/* check if area is inside the WRP Range */
		if ((OptionsBytesStruct4.WRPStartOffset * FLASH_PAGE_SIZE + FLASH_BASE
				+ FLASH_PAGE_SIZE * FLASH_PAGE_NBPERBANK) >= addr) {
			ProtectedPAGE |= FLASHIF_PROTECTION_WRPENABLED;
		}
	}

	if (OptionsBytesStruct4.RDPLevel != OB_RDP_LEVEL_0) {
		ProtectedPAGE |= FLASHIF_PROTECTION_RDPENABLED;
	}

	return ProtectedPAGE;
}

uint32_t FLASH_If_Write_fast(uint32_t StartAddr, uint32_t *p_source,
		uint32_t length) {
	HAL_StatusTypeDef status = HAL_OK;
	uint32_t i = 0;
	HAL_FLASH_Unlock();
	for (i = 0; i < length / 256; i++) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FAST,
				StartAddr + i * 32 * sizeof(uint64_t), *(p_source + 32 * i))
				!= HAL_OK) {
			status = HAL_ERROR;
			break;
		}
	}
	HAL_FLASH_Lock();
	return status;
}

uint32_t FLASH_If_Write(uint32_t destination, uint32_t endAddr,
		uint64_t *p_source, uint32_t length) {
	uint32_t status = HAL_OK;
	uint32_t i = 0;

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* DataLength must be a multiple of 64 bit */
	for (i = 0; (i < length / 8) && (destination <= (endAddr - 8)); i++) {
		/* Device voltage range supposed to be [2.7V to 3.6V], the operation will
		 be done by word */
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, destination,
				p_source[i]) == HAL_OK) {
			/* Check the written value */
			if (*(uint64_t*) destination != *(p_source + i)) {
				/* Flash content doesn't match SRAM content */
				status = HAL_ERROR;
				break;
			}
			/* Increment FLASH destination address */
			destination += 8;
		} else {
			/* Error occurred while writing data in Flash memory */
			status = HAL_ERROR;
			break;
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();

	return status;
}

uint32_t FLASH_If_WriteProtectionConfig(uint32_t protectionstate) {
	FLASH_OBProgramInitTypeDef OptionsBytesStruct1;
	HAL_StatusTypeDef retr;

	/* Unlock the Flash to enable the flash control register access *************/
	retr = HAL_FLASH_Unlock();

	/* Unlock the Options Bytes *************************************************/
	retr |= HAL_FLASH_OB_Unlock();

	OptionsBytesStruct1.OptionType = OPTIONBYTE_WRP;
	OptionsBytesStruct1.WRPArea = OB_WRPAREA_BANK1_AREAA;
	if (protectionstate == FLASHIF_WRP_ENABLE) {
		/* Enable the WRP protection for all flash BANK1 */
		OptionsBytesStruct1.WRPEndOffset = FLASH_PAGE_NBPERBANK - 1;
		OptionsBytesStruct1.WRPStartOffset = 0x00;
	} else {
		/* Remove all WRP protection */
		OptionsBytesStruct1.WRPEndOffset = 0x00;
		OptionsBytesStruct1.WRPStartOffset = 0xFF;
	}
	retr |= HAL_FLASHEx_OBProgram(&OptionsBytesStruct1);

	OptionsBytesStruct1.OptionType = OPTIONBYTE_WRP;
	OptionsBytesStruct1.WRPArea = OB_WRPAREA_BANK1_AREAB;
	OptionsBytesStruct1.WRPEndOffset = 0x00;
	OptionsBytesStruct1.WRPStartOffset = 0xFF;
	retr |= HAL_FLASHEx_OBProgram(&OptionsBytesStruct1);

	OptionsBytesStruct1.OptionType = OPTIONBYTE_WRP;
	OptionsBytesStruct1.WRPArea = OB_WRPAREA_BANK2_AREAA;
	if (protectionstate == FLASHIF_WRP_ENABLE) {
		/* Enable the WRP protection for all flash BANK1 */
		OptionsBytesStruct1.WRPEndOffset = FLASH_PAGE_NBPERBANK - 1;
		OptionsBytesStruct1.WRPStartOffset = 0x00;
	} else {
		/* Remove all WRP protection */
		OptionsBytesStruct1.WRPEndOffset = 0x00;
		OptionsBytesStruct1.WRPStartOffset = 0xFF;
	}
	retr |= HAL_FLASHEx_OBProgram(&OptionsBytesStruct1);

	OptionsBytesStruct1.RDPLevel = OB_RDP_LEVEL_0;
	OptionsBytesStruct1.OptionType = OPTIONBYTE_WRP;
	OptionsBytesStruct1.WRPArea = OB_WRPAREA_BANK2_AREAB;
	OptionsBytesStruct1.WRPEndOffset = 0x00;
	OptionsBytesStruct1.WRPStartOffset = 0xFF;
	retr |= HAL_FLASHEx_OBProgram(&OptionsBytesStruct1);

	return (retr == HAL_OK ? FLASHIF_OK : FLASHIF_PROTECTION_ERRROR);
}

