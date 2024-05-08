#include <stdint.h>
#include "Stm32F446xx.h"
#include "ErrType.h"
#include "FLASH_prv.h"
#include "FLASH_interface.h"


void Flash_VoidLock(void)
{
	_FLASH->FLASH_CR |= ( 1 << CR_LOCK); 
}

void FLASH_VoidUnLock(void)
{
	_FLASH->FLASH_KEYR = FLASH_KEY1_;
	_FLASH->FLASH_KEYR = FLASH_KEY2_;
}

FLASH_ErrorState_t FLASH_VoidErase(FLASH_Config_t* FLASH_Init)
{
	FLASH_ErrorState_t Local_u8Status = FLASH_OK;
	uint8_t Local_u8Iterator=0;
	/*Check that no Flash memory operation is ongoing by checking the BSY bit*/
	Local_u8Status = FLASH_Wait((uint32_t)50000UL);

	if(Local_u8Status == FLASH_OK)
	{


		if(FLASH_Init->TypeErase == _FLASH_TYPEERASE_MASSERASE)
		{
			FLASH_eraseMass((uint8_t) FLASH_Init->VoltageRange, FLASH_Init->Banks);
			/*Wait for the BSY bit to be cleared*/
			Local_u8Status = FLASH_Wait((uint32_t)50000UL);
			/*Mass Erase deactivated.*/
			_FLASH->FLASH_CR &=~ ( 1 << CR_MER);
		}
		else if(FLASH_Init->TypeErase == _FLASH_TYPEERASE_SECTORS)
		{


			for(Local_u8Iterator = FLASH_Init->Sector ;Local_u8Iterator < (FLASH_Init->NbSectors + FLASH_Init->Sector); Local_u8Iterator++)
			{
				Local_u8Status = FLASH_Wait((uint32_t)50000UL);
				Flash_eraseSector(Local_u8Iterator, (uint8_t) FLASH_Init->VoltageRange);
				/*Wait for the BSY bit to be cleared*/
				Local_u8Status = FLASH_Wait((uint32_t)50000UL);
				/*Sector Erase deactivated.*/
				_FLASH->FLASH_CR &=~ ( 1 << CR_SER) | ( 7 << CR_SNB);

			}
		}
	}
	return Local_u8Status;
}

static void Flash_eraseSector(uint8_t index ,uint8_t V_Range )
{
	/*Clear psize bits*/
	_FLASH->FLASH_CR &=~ (VOLATAGE_RANGE_MASK << CR_PSIZE);
	_FLASH->FLASH_CR |=  (V_Range << CR_PSIZE);
	/*Clear SNB and set the number of sector*/
	_FLASH->FLASH_CR &=~ ( 7 << CR_SNB);
	/*Activate sector erase*/
	_FLASH->FLASH_CR |=  ( 1 << CR_SER) | ( index << CR_SNB);
	/*Set the STRT bit in the FLASH_CR register*/
	_FLASH->FLASH_CR |= ( 1 << CR_STRT);
}
static void FLASH_eraseMass(uint8_t V_Range ,uint32_t Banks)
{
	/*Clear psize bits*/
	_FLASH->FLASH_CR &=~ ((VOLATAGE_RANGE_MASK) << CR_PSIZE);
	/*Mass Erase activated.*/
	_FLASH->FLASH_CR |= ( 1 << CR_MER);
	/*Set the STRT bit in the FLASH_CR register*/
	_FLASH->FLASH_CR |= ( 1 << CR_STRT) | ((uint32_t)V_Range << CR_PSIZE);
}
FLASH_ErrorState_t FLASH_Wait(uint32_t Copy_u32TimeOut)
{
	uint32_t Local_u32TickStart = 0U;
	while( ( (_FLASH->FLASH_SR >> SR_BSY) & GET_MASK) != 0)
	{
		Local_u32TickStart++;
		if(Copy_u32TimeOut != 0xFFFFFFFFU)
		{
			if((Local_u32TickStart > Copy_u32TimeOut) || (Local_u32TickStart == 0))
			{
				return FLASH_TIMEOUT;
			}
		}
	}
	return FLASH_OK;
}

FLASH_ErrorState_t FLASH_VoidProgram(uint8_t copy_u8ProgType,uint32_t copy_u32Address, uint64_t copy_u64Data)
{
	FLASH_ErrorState_t Local_u8Status = FLASH_OK;

	/*Check that no main Flash memory operation is ongoing by checking the BSY bit*/
	Local_u8Status = FLASH_Wait((uint32_t)50000UL);
	if(Local_u8Status == FLASH_OK)
	{

		if(copy_u8ProgType == 0)
		{
			voidProgramByte(copy_u32Address,(uint8_t)copy_u64Data);
		}
		else if(copy_u8ProgType == 1)
		{
			voidProgramHalfWord(copy_u32Address,(uint16_t)copy_u64Data);
		}
		else if(copy_u8ProgType == 2)
		{
			voidProgramWord(copy_u32Address,(uint32_t)copy_u64Data);
		}
		else
		{
			voidProgramDoubleWord(copy_u32Address,copy_u64Data);
		}

		/*Wait for the BSY bit to be cleared.*/
		Local_u8Status = FLASH_Wait((uint32_t)50000UL);

		/*Reset the PG bit in the FLASH_CR register*/
		_FLASH->FLASH_CR &=~ ( 1 << CR_PG);
	}


	return Local_u8Status;
}

static void voidProgramByte(uint32_t u32Address, uint8_t u8Data)
{
	/*Clear psize bits*/
	_FLASH->FLASH_CR &=~ ((VOLATAGE_RANGE_MASK) << CR_PSIZE);
	/*Set the PG bit in the FLASH_CR register*/
	_FLASH->FLASH_CR |= ( 1 << CR_PG);
	*((volatile uint8_t*)u32Address) = u8Data;
}
static void voidProgramHalfWord(uint32_t u32Address, uint16_t u16Data)
{
	/*Clear psize bits*/
	_FLASH->FLASH_CR &=~ ((VOLATAGE_RANGE_MASK) << CR_PSIZE);
	/*Set the STRT bit in the FLASH_CR register*/
	_FLASH->FLASH_CR |= ( 1 << CR_PSIZE);
	/*Set the PG bit in the FLASH_CR register*/
	_FLASH->FLASH_CR |= ( 1 << CR_PG);
	*((volatile uint16_t*)u32Address) = u16Data;
}

static void voidProgramWord(uint32_t u32Address, uint32_t u32Data)
{
	/*Clear psize bits*/
	_FLASH->FLASH_CR &=~ ((VOLATAGE_RANGE_MASK) << CR_PSIZE);
	/*Set the STRT bit in the FLASH_CR register*/
	_FLASH->FLASH_CR |= ( 2 << CR_PSIZE);
	/*Set the PG bit in the FLASH_CR register*/
	_FLASH->FLASH_CR |= ( 1 << CR_PG);
	*((volatile uint32_t*)u32Address) = u32Data;
}

static void voidProgramDoubleWord(uint32_t u32Address, uint64_t u64Data)
{
	/*Clear psize bits*/
	_FLASH->FLASH_CR &=~ ((VOLATAGE_RANGE_MASK) << CR_PSIZE);
	/*Set the STRT bit in the FLASH_CR register*/
	_FLASH->FLASH_CR |= ( 3 << CR_PSIZE);
	/*Set the PG bit in the FLASH_CR register*/
	_FLASH->FLASH_CR |= ( 1 << CR_PG);
	/*1st word*/
	*((volatile uint32_t*)u32Address) = (uint32_t)u64Data;
	/*2nd word*/
	*((volatile uint32_t*)u32Address+4) = (uint32_t)(u64Data >> 32);
}

void FLASH_OPTKey_UnLock(void)
{
	_FLASH->FLASH_OPTKEYR = FLASH_OPT_KEY1_;
	_FLASH->FLASH_OPTKEYR = FLASH_OPT_KEY2_;
}

void FLASH_OPTKey_Lock(void)
{
	_FLASH->FLASH_OPTCR |= ( 1 << OPTCR_OPTLOCK);
}

FLASH_ErrorState_t FLASH_EnWProtect(uint8_t copy_u8SectorDetails)
{
	FLASH_ErrorState_t Local_u8Status = FLASH_OK;
	/*Check that no Flash memory operation is ongoing by checking the BSY bit*/
	Local_u8Status = FLASH_Wait((uint32_t)50000UL);
	if(Local_u8Status == FLASH_OK)
	{
		/*RESET SPRMOD: WRITE MODE*/
		_FLASH->FLASH_OPTCR &=~ (1 << OPTCR_SPRMOD);
		/*Set sectors*/
		//Local_u8Complement = (~copy_u8SectorDetails);
		_FLASH->FLASH_OPTCR &=~ (copy_u8SectorDetails << OPTCR_nWRP);
		/*Set the option start bit (OPTSTRT) in the FLASH_OPTCR register*/
		_FLASH->FLASH_OPTCR |= (1 << OPTCR_OPTSTRT);
		/*Wait for the BSY bit to be cleared*/
		Local_u8Status = FLASH_Wait((uint32_t)50000UL);
	}
	return Local_u8Status;
}

FLASH_ErrorState_t FLASH_EnRWProtect(uint8_t copy_u8SectorDetails)
{
	FLASH_ErrorState_t Local_u8Status = FLASH_OK;
	/*Check that no Flash memory operation is ongoing by checking the BSY bit*/
	Local_u8Status = FLASH_Wait((uint32_t)50000UL);

	if(Local_u8Status == FLASH_OK)
	{
		/*SET SPRMOD */
		_FLASH->FLASH_OPTCR |= (1 << OPTCR_SPRMOD);
		/**Set sectors**/
		_FLASH->FLASH_OPTCR |= ( copy_u8SectorDetails << OPTCR_nWRP);

		/*Set the option start bit (OPTSTRT) in the FLASH_OPTCR register*/
		_FLASH->FLASH_OPTCR |= (1 << OPTCR_OPTSTRT);
		/*Wait for the BSY bit to be cleared*/
		Local_u8Status = FLASH_Wait((uint32_t)50000UL);
	}

	return Local_u8Status;
}

FLASH_ErrorState_t FLASH_DisWProtect(void)
{
	FLASH_ErrorState_t Local_u8Status = FLASH_OK;
	/*Check that no Flash memory operation is ongoing by checking the BSY bit*/
	Local_u8Status = FLASH_Wait((uint32_t)50000UL);
	if(Local_u8Status == FLASH_OK)
	{
		/*RESET SPRMOD: WRITE MODE*/
		_FLASH->FLASH_OPTCR &=~ (1 << OPTCR_SPRMOD);
		/*Set sectors*/
		_FLASH->FLASH_OPTCR |= (0b11111111 << OPTCR_nWRP);
		/*Set the option start bit (OPTSTRT) in the FLASH_OPTCR register*/
		_FLASH->FLASH_OPTCR |= (1 << OPTCR_OPTSTRT);
		/*Wait for the BSY bit to be cleared*/
		Local_u8Status = FLASH_Wait((uint32_t)50000UL);
	}
	return Local_u8Status;
}

FLASH_ErrorState_t FLASH_DisRWProtect(void)
{
	FLASH_ErrorState_t Local_u8Status = FLASH_OK;
	/*Check that no Flash memory operation is ongoing by checking the BSY bit*/
	Local_u8Status = FLASH_Wait((uint32_t)50000UL);

	if(Local_u8Status == FLASH_OK)
	{
		/*SET SPRMOD */
		_FLASH->FLASH_OPTCR |= (1 << OPTCR_SPRMOD);
		/**Set sectors**/
		_FLASH->FLASH_OPTCR &=~ ( 0b11111111U << OPTCR_nWRP);

		/*Set the option start bit (OPTSTRT) in the FLASH_OPTCR register*/
		_FLASH->FLASH_OPTCR |= (1 << OPTCR_OPTSTRT);
		/*Wait for the BSY bit to be cleared*/
		Local_u8Status = FLASH_Wait((uint32_t)50000UL);
	}

	return Local_u8Status;
}





















