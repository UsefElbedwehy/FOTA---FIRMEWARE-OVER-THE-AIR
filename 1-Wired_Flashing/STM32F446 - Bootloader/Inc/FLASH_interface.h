#ifndef FLASH_INTERFACE_H_
#define FLASH_INTERFACE_H_

#include "ErrType.h"

#define GET_MASK    0x01

#define VOLATAGE_RANGE_MASK 3U

/*
 * @FLASH_ACR_t
 *  
*/
typedef enum{
	
	ACR_LATENCY,
	ACR_PRFTEN=8,
	ACR_ICEN,
	ACR_DCEN,
	ACR_ICRST,
	ACR_DCRST
	
}FLASH_ACR_t;

/*
 * @FLASH_KEYR_t
 *  Flash key register
 *  The Flash key register is used to allow access to the Flash control register 
	and so, to allow program and erase operations.
*/
typedef enum{
	
	FKEYR=0
	
}FLASH_KEYR_t;

/*
 * @FLASH_OPTKEYR_t
 *  Flash option key register
 *  The Flash option key register is used to allow program and erase operations in the user 
	configuration sector.
*/
typedef enum{
	
	OPTFKEYR=0
	
}FLASH_OPTKEYR_t;

/*
 * @FLASH_SR_t
 *  Flash status register
*/
typedef enum{
	
	SR_EOP,
	SR_OPERR,
	SR_WRPERR=4,
	SR_PGAERR,
	SR_PGPERR,
	SR_PGSERR,
	SR_RDERR,
	SR_BSY=16
	
}FLASH_SR_t;

/*
 * @FLASH_CR_t
 *  Flash control register
*/
typedef enum{
	
	CR_PG,
	CR_SER,
	CR_MER,
	CR_SNB,
	CR_PSIZE=8,
	CR_STRT=16,
	CR_EOPIE=24,
	CR_ERRIE=25,
	CR_LOCK=31
	
}FLASH_CR_t;

/*
 * @FLASH_OPTCR_t
 *  Flash option control register
*/
typedef enum{
	
	OPTCR_OPTLOCK,
	OPTCR_OPTSTRT,
	OPTCR_BOR_LEV,
	OPTCR_WDG_SW=5,
	OPTCR_nRST_STOP=6,
	OPTCR_nRST_STDBY=7,
	OPTCR_RDP=8,
	OPTCR_nWRP=16,
	OPTCR_SPRMOD=31
	
}FLASH_OPTCR_t;

/** @defgroup FLASHEx_Type_Erase FLASH Type Erase
  * @{
  */ 
#define _FLASH_TYPEERASE_SECTORS         0x00000000U  /*!< Sectors erase only          */
#define _FLASH_TYPEERASE_MASSERASE       0x00000001U  /*!< Flash Mass erase activation */
/**
  * @}
  */
  
 /** @defgroup FLASHEx_Voltage_Range FLASH Voltage Range
  * @{
  */ 
#define _FLASH_VOLTAGE_RANGE_1        0x00000000U  /*!< Device operating range: 1.8V to 2.1V                */
#define _FLASH_VOLTAGE_RANGE_2        0x00000001U  /*!< Device operating range: 2.1V to 2.7V                */
#define _FLASH_VOLTAGE_RANGE_3        0x00000002U  /*!< Device operating range: 2.7V to 3.6V                */
#define _FLASH_VOLTAGE_RANGE_4        0x00000003U  /*!< Device operating range: 2.7V to 3.6V + External Vpp */
/**
  * @}
  */

#define _FLASH_BANK_1     1U /*!< Bank 1   */

typedef struct{
	 
	uint32_t TypeErase;   /*!< Mass erase or sector Erase.
                             This parameter can be a value of @ref FLASHEx_Type_Erase */

    uint32_t Banks;       /*!< Select banks to erase when Mass erase is enabled.
                             This parameter must be a value of @ref FLASHEx_Banks */

    uint32_t Sector;      /*!< Initial FLASH sector to erase when Mass erase is disabled
                             This parameter must be a value of @ref FLASHEx_Sectors */

    uint32_t NbSectors;   /*!< Number of sectors to be erased.
                             This parameter must be a value between 1 and (max number of sectors - value of Initial sector)*/

    uint32_t VoltageRange;/*!< The device voltage range which defines the erase parallelism
                             This parameter must be a value of @ref FLASHEx_Voltage_Range */
	
}FLASH_Config_t;



void Flash_VoidLock(void);
void FLASH_VoidUnLock(void);
FLASH_ErrorState_t FLASH_VoidErase(FLASH_Config_t* FLASH_Init );
FLASH_ErrorState_t FLASH_Wait(uint32_t Copy_u32TimeOut);
FLASH_ErrorState_t FLASH_VoidProgram(uint8_t copy_u8ProgType,uint32_t copy_u32Address, uint64_t copy_u64Data);


void FLASH_OPTKey_Lock(void);
void FLASH_OPTKey_UnLock(void);
FLASH_ErrorState_t FLASH_EnRWProtect(uint8_t copy_u8SectorDetails);
FLASH_ErrorState_t FLASH_EnWProtect(uint8_t copy_u8SectorDetails);
FLASH_ErrorState_t FLASH_DisRWProtect(void);
FLASH_ErrorState_t FLASH_DisWProtect(void);


#endif
