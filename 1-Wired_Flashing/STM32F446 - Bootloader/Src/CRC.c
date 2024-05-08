#include <stdint.h>
#include "Stm32F446xx.h"
#include "CRC.h"

/*
 *@fn	  : CRC_u32Accumulate
 *@brief  : calculate the CRC value with accumulation
 *@param  : Copy_pu32Buffer
 *@param  : Copy_u32Length
 *@retval : Local_u32Accumulate
 *
*/
uint32_t CRC_u32Accumulate(uint32_t* Copy_pu32Buffer, uint32_t Copy_u32Length)
{
	uint32_t Local_u32Iterator=0;
	uint32_t Local_u32Accumulate=0;
	
	/*Pass data to the CRC calculator*/
	for( Local_u32Iterator = 0 ; Local_u32Iterator < Copy_u32Length ; Local_u32Iterator++ )
	{
		CRC->CRC_DR = Copy_pu32Buffer[Local_u32Iterator];
	}
	/*Read the previous CRC calculation result*/
	Local_u32Accumulate = CRC->CRC_DR;
	
	return Local_u32Accumulate;
}

/*
 *@fn	  : CRC_u32Calculate
 *@brief  : Calculate the CRC value without accumulation
 *@param  : Copy_pu32Buffer
 *@param  : Copy_u32Length
 *@retval : Local_u32CRCVal
 *
*/
uint32_t CRC_u32Calculate(uint32_t* Copy_pu32Buffer, uint32_t Copy_u32Length)
{
	uint32_t Local_u32Iterator=0;
	uint32_t Local_u32CRCVal=0;
	
	/*Reset the CRC data register*/
	CRC_voidResetCRC();
	
	/*Pass data to the CRC calculator*/
	for( Local_u32Iterator = 0 ; Local_u32Iterator < Copy_u32Length ; Local_u32Iterator++ )
	{
		CRC->CRC_DR = Copy_pu32Buffer[Local_u32Iterator];
	}
	/*Read the previous CRC calculation result*/
	Local_u32CRCVal = CRC->CRC_DR;
	
	return Local_u32CRCVal;
}

/*
 *@fn	  : CRC_voidResetCRC
 *@brief  : Resets the CRC calculation unit and sets the data register to 0xFFFFFFFF
 *@param  : void
 *@retval : void
 *
*/
void CRC_voidResetCRC(void)
{
	/*Resets the CRC calculation unit and sets the data register to 0xFFFFFFFF*/
	CRC->CRC_CR |= (1 << CR_RESET);
}
