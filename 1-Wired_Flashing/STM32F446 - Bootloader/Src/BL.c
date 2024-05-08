/*
 * BL.c
 *
 *  Created on: Apr 6, 2024
 *      Author: Usef
 */

#include <stdint.h>
#include "BL.h"
#include "BL_prv.h"
#include "CRC.h"
#include "USART_prv.h"
#include "USART_interface.h"
#include "FLASH_prv.h"
#include "FLASH_interface.h"
#include "main.h"
extern UART_HandleTypeDef huart1;;
extern UART_HandleTypeDef huart2;




static uint8_t u8VerifyCRC(uint8_t* copy_pu8DataArray , uint8_t Copy_u8Length ,uint32_t Copy_u32HostCRC)
{
	uint8_t Local_u8Iterator , Local_u8CRCStatus;
	uint32_t Local_u32AccCRC , Local_u32Temp;
	for(Local_u8Iterator=0 ;Local_u8Iterator < Copy_u8Length;Local_u8Iterator++)
	{
		Local_u32Temp = copy_pu8DataArray[Local_u8Iterator];
		Local_u32AccCRC = CRC_u32Accumulate(&Local_u32Temp, 1);
	}
	/*reset CRC calculation unit*/
	CRC_voidResetCRC();

	if(Local_u32AccCRC == Copy_u32HostCRC)
	{
		Local_u8CRCStatus = CRC_SUCCESS;
	}
	else
	{
		Local_u8CRCStatus = CRC_FAIL;
	}

	return Local_u8CRCStatus;
}

static void voidSendAck(uint8_t Copy_u8ReplyLength)
{
	uint8_t Local_u8AckPuffer[2] = {BL_ACK ,Copy_u8ReplyLength};

	HAL_UART_Transmit(&huart2, Local_u8AckPuffer, 2, HAL_MAX_DELAY);
}

static void voidSendNAck(void)
{
	uint8_t Local_u8NAck = BL_NACK;

	HAL_UART_Transmit(&huart2, &Local_u8NAck, 1, HAL_MAX_DELAY);
}

/*
 * @fn		: BL_VoidHandleGetVerCmd
 * @brief   : Get bootloader version command
 * @param	: copy_pu8CmdPacket
 * @retval	: void
 *
 * */
void BL_VoidHandleGetVerCmd(uint8_t* copy_pu8CmdPacket)
{
	uint8_t Local_u8BLVersion , Local_u8CRCStatus , Local_u8CmdLen;
	uint32_t Local_u32HostCRC;

	/**/

	Local_u8CmdLen = copy_pu8CmdPacket[0] + 1; /*the 1st byte already includes the length to follow*/

	Local_u32HostCRC = *((uint32_t*)(copy_pu8CmdPacket + Local_u8CmdLen - 4));

	Local_u8CRCStatus = u8VerifyCRC(copy_pu8CmdPacket, (Local_u8CmdLen - 4) , Local_u32HostCRC);

	if(Local_u8CRCStatus == CRC_SUCCESS)
	{
		voidSendAck(1u); /*bootloader version has size of 1 byte*/

		Local_u8BLVersion = BL_VERSION;

		HAL_UART_Transmit(&huart2, &Local_u8BLVersion, 1, HAL_MAX_DELAY);

	}
	else
	{
		voidSendNAck();
	}

}
/*
 * @fn		: BL_VoidHandleGetHelpCmd
 * @brief   : Get help command
 * @param	: copy_pu8CmdPacket
 * @retval	: void
 *
 * */
void BL_VoidHandleGetHelpCmd(uint8_t* copy_pu8CmdPacket)
{

	uint8_t  Local_u8CRCStatus , Local_u8CmdLen;
	uint32_t Local_u32HostCRC;

	/**/

	Local_u8CmdLen = copy_pu8CmdPacket[0] + 1; /*the 1st byte already includes the length to follow*/

	Local_u32HostCRC = *((uint32_t*)(copy_pu8CmdPacket + Local_u8CmdLen - 4));

	Local_u8CRCStatus = u8VerifyCRC(copy_pu8CmdPacket, (Local_u8CmdLen - 4) , Local_u32HostCRC);

	if(Local_u8CRCStatus == CRC_SUCCESS)
	{
		uint8_t Local_pu8BLCommands[]={

				BL_GET_VER       		  ,
				BL_GET_HELP      		  ,
				BL_GET_CID      	 	  ,
				BL_GET_RDP_STATUS         ,
				BL_GO_TO_ADDR       	  ,
				BL_FLASH_ERASE       	  ,
				BL_MEM_WRITE       		  ,
				BL_EN_RW_PROTECT          ,
				BL_MEM_READ       		  ,
				BL_READ_SECTOR_STATUS     ,
				BL_OTP_READ       		  ,
				BL_DIS_WR_PROTECT

		};

		voidSendAck(sizeof(Local_pu8BLCommands));

		HAL_UART_Transmit(&huart2, Local_pu8BLCommands, sizeof(Local_pu8BLCommands), HAL_MAX_DELAY);

	}
	else
	{
		voidSendNAck();
	}



}
/*
 * @fn		: BL_VoidHandleGetCIDCmd
 * @brief   : Get chip ID command
 * @param	: copy_pu8CmdPacket
 * @retval	: void
 *
 * */
void BL_VoidHandleGetCIDCmd(uint8_t* copy_pu8CmdPacket)
{
	uint8_t  Local_u8CRCStatus , Local_u8CmdLen;
	uint32_t Local_u32HostCRC;

	/**/

	Local_u8CmdLen = copy_pu8CmdPacket[0] + 1; /*the 1st byte already includes the length to follow*/

	Local_u32HostCRC = *((uint32_t*)(copy_pu8CmdPacket + Local_u8CmdLen - 4));

	Local_u8CRCStatus = u8VerifyCRC(copy_pu8CmdPacket, (Local_u8CmdLen - 4) , Local_u32HostCRC);

	if(Local_u8CRCStatus == CRC_SUCCESS)
	{
		uint16_t Local_u16DeviceID = (DBGMCU_ICCODE_REG & 0x0fff) ; // DBGMCU->IDCODE

		voidSendAck(2u);

		HAL_UART_Transmit(&huart2, (uint8_t*)&Local_u16DeviceID, 2 , HAL_MAX_DELAY);

	}
	else
	{
		voidSendNAck();
	}
}

/*
 * @fn		: BL_VoidHandleGetRDPStatusCmd
 * @brief   : Get RDP status command
 * @param	: copy_pu8CmdPacket
 * @retval	: void
 *
 * */
void BL_VoidHandleGetRDPStatusCmd(uint8_t* copy_pu8CmdPacket)
{
	uint8_t  Local_u8CRCStatus , Local_u8CmdLen;
	uint32_t Local_u32HostCRC;

	/**/

	Local_u8CmdLen = copy_pu8CmdPacket[0] + 1; /*the 1st byte already includes the length to follow*/

	Local_u32HostCRC = *((uint32_t*)(copy_pu8CmdPacket + Local_u8CmdLen - 4));

	Local_u8CRCStatus = u8VerifyCRC(copy_pu8CmdPacket, (Local_u8CmdLen - 4) , Local_u32HostCRC);

	if(Local_u8CRCStatus == CRC_SUCCESS)
	{
		uint8_t Local_u8RDPStatus = (uint8_t)((RDP_USER_OPTION_WORD >> 8) & 0xff);

		voidSendAck(1u);

		HAL_UART_Transmit(&huart2, &Local_u8RDPStatus, 1 , HAL_MAX_DELAY);

	}
	else
	{
		voidSendNAck();
	}
}

/*
 * @fn		: BL_VoidHandleGoToAddressCmd
 * @brief   : go to address command
 * @param	: copy_pu8CmdPacket
 * @retval	: void
 *
 * */
void BL_VoidHandleGoToAddressCmd(uint8_t* copy_pu8CmdPacket)
{
	uint8_t  Local_u8CRCStatus , Local_u8CmdLen;
	uint32_t Local_u32HostCRC;

	/**/

	Local_u8CmdLen = copy_pu8CmdPacket[0] + 1; /*the 1st byte already includes the length to follow*/

	Local_u32HostCRC = *((uint32_t*)(copy_pu8CmdPacket + Local_u8CmdLen - 4));

	Local_u8CRCStatus = u8VerifyCRC(copy_pu8CmdPacket, (Local_u8CmdLen - 4) , Local_u32HostCRC);

	if(Local_u8CRCStatus == CRC_SUCCESS)
	{
		uint32_t Local_u32Address;
		uint8_t Local_u8AddressValidateStatus;


		voidSendAck(1u);

		Local_u32Address = *((uint32_t*)&copy_pu8CmdPacket[2]);

		Local_u8AddressValidateStatus = u8ValidateAddress(Local_u32Address);

		HAL_UART_Transmit(&huart2, &Local_u8AddressValidateStatus, 1, HAL_MAX_DELAY);

		if(Local_u8AddressValidateStatus == VALID_ADDRESS)
		{

			/*Define a pointer to function*/
			void (*Local_pvFuncPtr)(void) = NULL;

			/*Increment address by 1 to make t-bit = 1 (thumb instruction)*/
			Local_u32Address |= 0x1;

			Local_pvFuncPtr = (void*)Local_u32Address;

			Local_pvFuncPtr();

		}



	}
	else
	{
		voidSendNAck();
	}
}

static uint8_t u8ValidateAddress(uint32_t Copy_u32Address)
{
	/*Address is valid if it is within : SRAM or FLASH*/
	uint8_t Local_u8AddressStatus;
	if((Copy_u32Address >= FLASH_BASE) && (Copy_u32Address <= FLASH_END ))
	{
		Local_u8AddressStatus = VALID_ADDRESS;
	}
	else if((Copy_u32Address >= SRAM1_BASE) && (Copy_u32Address <= (SRAM1_BASE + (128*1024)) ))
	{
		Local_u8AddressStatus = VALID_ADDRESS;
	}
	else
	{
		Local_u8AddressStatus = INVALID_ADDRESS;
	}

	return Local_u8AddressStatus;
}

/*
 * @fn		: BL_VoidHandleFlashEraseCmd
 * @brief   : Flash erase command
 * @param	: copy_pu8CmdPacket
 * @retval	: void
 *
 * */
void BL_VoidHandleFlashEraseCmd(uint8_t* copy_pu8CmdPacket)
{
	uint8_t  Local_u8CRCStatus , Local_u8CmdLen;
	uint32_t Local_u32HostCRC;

	/**/

	Local_u8CmdLen = copy_pu8CmdPacket[0] + 1; /*the 1st byte already includes the length to follow*/

	Local_u32HostCRC = *((uint32_t*)(copy_pu8CmdPacket + Local_u8CmdLen - 4));

	Local_u8CRCStatus = u8VerifyCRC(copy_pu8CmdPacket, (Local_u8CmdLen - 4) , Local_u32HostCRC);

	if(Local_u8CRCStatus == CRC_SUCCESS)
	{
		uint8_t Local_u8EraseStatus;

		voidSendAck(1u);

		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

		Local_u8EraseStatus = u8ExecuteFlashErase(copy_pu8CmdPacket[2], copy_pu8CmdPacket[3]);

		HAL_UART_Transmit(&huart2, &Local_u8EraseStatus, 1 , HAL_MAX_DELAY);

		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	}
	else
	{
		voidSendNAck();
	}
}
/*
 * @fn     : u8ExecuteFlashErase
 * @brief  : Execute flash erase
 * @param  : Copy_u8SectorNumber
 * @param  : Copy_u8NumberOfSectors
 * @retval : Local_u8ErrorStatus
 * */
static uint8_t u8ExecuteFlashErase(uint8_t Copy_u8SectorNumber ,uint8_t Copy_u8NumberOfSectors)
{
	uint8_t Local_ErrorStatus = 0;

	if((Copy_u8NumberOfSectors > 8) && (Copy_u8NumberOfSectors != 0xFF))
	{
		Local_ErrorStatus = 1;
	}
	else if((Copy_u8SectorNumber > 7) && (Copy_u8SectorNumber != 0xFF))
	{
		Local_ErrorStatus = 1;
	}
	else
	{
		FLASH_Config_t Local_MyErase;
		//FLASH_EraseInitTypeDef Local_MyErase;

		if(Copy_u8SectorNumber == 0xff )
		{
			/*mass erase option is required*/
			Local_MyErase.TypeErase = _FLASH_TYPEERASE_MASSERASE;
		}
		else
		{
			/*sector erase is required*/
			uint8_t Local_u8RemainingSectors = 8 - Copy_u8SectorNumber;
			if(Copy_u8NumberOfSectors > Local_u8RemainingSectors)
			{
				/*if number of sector is bigger than max, make it equal to the max*/
				Copy_u8NumberOfSectors = Local_u8RemainingSectors;
			}
			else
			{
				/*Do nothing*/
			}
			Local_MyErase.TypeErase = _FLASH_TYPEERASE_SECTORS;

			Local_MyErase.Sector = Copy_u8SectorNumber;

			Local_MyErase.NbSectors = Copy_u8NumberOfSectors;

		}
		Local_MyErase.VoltageRange = _FLASH_VOLTAGE_RANGE_3;

		Local_MyErase.Banks = _FLASH_BANK_1;

		/*unlock the flash before erasing*/
		FLASH_VoidUnLock();

		Local_ErrorStatus = FLASH_VoidErase(&Local_MyErase);

		/*lock the flash again*/
		Flash_VoidLock();

	}

	return Local_ErrorStatus;
}
/*
 * @fn     : BL_VoidHandleMemWriteCmd
 * @brief  : handle memory write command
 * @param  : copy_pu8CmdPacket
 * @retval : void
 * */
uint32_t i=0;
void BL_VoidHandleMemWriteCmd(uint8_t* copy_pu8CmdPacket)
{
	uint8_t  Local_u8CRCStatus , Local_u8CmdLen;
	uint32_t Local_u32HostCRC;

	/**/

	Local_u8CmdLen = copy_pu8CmdPacket[0] + 1; /*the 1st byte already includes the length to follow*/

	//Local_u32HostCRC = *((uint32_t*)(copy_pu8CmdPacket + Local_u8CmdLen - 4));

	//Local_u8CRCStatus = u8VerifyCRC(copy_pu8CmdPacket, (Local_u8CmdLen - 4) , Local_u32HostCRC);

	//if(Local_u8CRCStatus == CRC_SUCCESS)
	//{
		/*Extract base memory address*/
		uint32_t Local_u32Address = *(( uint32_t*)&copy_pu8CmdPacket[2]) + (64*i);

		uint8_t Local_u8WritingStatus;
		uint8_t Local_u8AddressStatus;
		i++;
		Local_u8AddressStatus = u8ValidateAddress(Local_u32Address);

		/*send the acknowledge*/
		//voidSendAck(1u);

		if(Local_u8AddressStatus == VALID_ADDRESS)
		{
			uint8_t Local_u8PayloadLength = copy_pu8CmdPacket[6];

			Local_u8WritingStatus = u8ExecuteMemWrite(&copy_pu8CmdPacket[7],Local_u32Address,Local_u8PayloadLength);
		}
		else
		{
			Local_u8WritingStatus = WRITING_ERROR;
		}

		HAL_UART_Transmit(&huart1, &Local_u8WritingStatus, 1 , HAL_MAX_DELAY);

	//}
	//else
	//{
	//	voidSendNAck();
	//}
}
/*
 * @fn     : u8ExecuteMemWrite
 * @brief  : Execute memory write
 * @param  : Copy_pu8Buffer
 * @param  : Copy_u32Address
 * @param  : Copy_u8Length
 * @retval : Local_u8ErrorStatus
 * */
static uint8_t u8ExecuteMemWrite(uint8_t* Copy_pu8Buffer ,uint32_t Copy_u32Address , uint8_t Copy_u8Length)
{
	uint8_t Local_u8ErrorStatus;
	if((Copy_u32Address >= FLASH_BASE) && (Copy_u32Address <= FLASH_END))
	{
		uint8_t Local_u8Iterator;
		/*unlock flash before writing*/
		FLASH_VoidUnLock();
		for(Local_u8Iterator=0;Local_u8Iterator < Copy_u8Length;Local_u8Iterator++)
		{
			Local_u8ErrorStatus = FLASH_VoidProgram(0, Copy_u32Address+Local_u8Iterator , (uint64_t)Copy_pu8Buffer[Local_u8Iterator]);
		}
		Flash_VoidLock();
	}
	else
	{
		/*writing in SRAM case*/
		uint8_t Local_u8Iterator;
		uint8_t* Local_pu8Dest = (uint8_t*)Copy_u32Address;
		for(Local_u8Iterator=0;Local_u8Iterator < Copy_u8Length;Local_u8Iterator++)
		{
			Local_pu8Dest[Local_u8Iterator] = Copy_pu8Buffer[Local_u8Iterator];
		}
	}
	return Local_u8ErrorStatus;
}

/*
 * @fn     : BL_VoidHandleEnRWProtectCmd
 * @brief  : enable Write protection or Read and write protection
 * @param  : copy_pu8CmdPacket
 * @retval : void
 * */
void BL_VoidHandleEnRWProtectCmd(uint8_t* copy_pu8CmdPacket)
{
	uint8_t  Local_u8CRCStatus , Local_u8CmdLen;
	uint32_t Local_u32HostCRC;

	/**/

	Local_u8CmdLen = copy_pu8CmdPacket[0] + 1; /*the 1st byte already includes the length to follow*/

	Local_u32HostCRC = *((uint32_t*)(copy_pu8CmdPacket + Local_u8CmdLen - 4));

	Local_u8CRCStatus = u8VerifyCRC(copy_pu8CmdPacket, (Local_u8CmdLen - 4) , Local_u32HostCRC);

	if(Local_u8CRCStatus == CRC_SUCCESS)
	{
		uint8_t Local_u8SectorDetails;
		uint8_t Local_u8ProtectionMode;
		uint8_t Local_u8ProtectingStatus;
		/*Extract Sector details*/
		Local_u8SectorDetails = copy_pu8CmdPacket[2];
		/*Extract protection mode*/
		Local_u8ProtectionMode = copy_pu8CmdPacket[3];

		voidSendAck(1u); /*bootloader version has size of 1 byte*/

		FLASH_OPTKey_UnLock();

		if(Local_u8ProtectionMode == W_PROTECTING)
		{
			/*Write Protection*/
			Local_u8ProtectingStatus = FLASH_EnWProtect(Local_u8SectorDetails);
		}
		else if(Local_u8ProtectionMode == RW_PROTECTING)
		{
			/*read and write protection*/
			Local_u8ProtectingStatus = FLASH_EnRWProtect(Local_u8SectorDetails);
		}
		else
		{
			Local_u8ProtectingStatus = PROTECTING_ERROR;
		}

		FLASH_OPTKey_Lock();

		HAL_UART_Transmit(&huart2, &Local_u8ProtectingStatus, 1 , HAL_MAX_DELAY);

	}
	else
	{
		voidSendNAck();
	}
}

void BL_VoidHandleMemReadCmd(uint8_t* copy_pu8CmdPacket)
{
	uint8_t  Local_u8CRCStatus , Local_u8CmdLen;
	uint32_t Local_u32HostCRC;

	/**/

	Local_u8CmdLen = copy_pu8CmdPacket[0] + 1; /*the 1st byte already includes the length to follow*/

	Local_u32HostCRC = *((uint32_t*)(copy_pu8CmdPacket + Local_u8CmdLen - 4));

	Local_u8CRCStatus = u8VerifyCRC(copy_pu8CmdPacket, (Local_u8CmdLen - 4) , Local_u32HostCRC);

	if(Local_u8CRCStatus == CRC_SUCCESS)
	{

		voidSendAck(1u);


	}
	else
	{
		voidSendNAck();
	}
}

void BL_VoidHandleReadSectorStatusCmd(uint8_t* copy_pu8CmdPacket)
{
	uint8_t  Local_u8CRCStatus , Local_u8CmdLen;
	uint32_t Local_u32HostCRC;

	/**/

	Local_u8CmdLen = copy_pu8CmdPacket[0] + 1; /*the 1st byte already includes the length to follow*/

	Local_u32HostCRC = *((uint32_t*)(copy_pu8CmdPacket + Local_u8CmdLen - 4));

	Local_u8CRCStatus = u8VerifyCRC(copy_pu8CmdPacket, (Local_u8CmdLen - 4) , Local_u32HostCRC);

	if(Local_u8CRCStatus == CRC_SUCCESS)
	{
		voidSendAck(1u); /*bootloader version has size of 1 byte*/

		//HAL_UART_Transmit(&huart2, &, , HAL_MAX_DELAY);

	}
	else
	{
		voidSendNAck();
	}
}

void BL_VoidHandleOTPReadCmd(uint8_t* copy_pu8CmdPacket)
{
	uint8_t  Local_u8CRCStatus , Local_u8CmdLen;
	uint32_t Local_u32HostCRC;

	/**/

	Local_u8CmdLen = copy_pu8CmdPacket[0] + 1; /*the 1st byte already includes the length to follow*/

	Local_u32HostCRC = *((uint32_t*)(copy_pu8CmdPacket + Local_u8CmdLen - 4));

	Local_u8CRCStatus = u8VerifyCRC(copy_pu8CmdPacket, (Local_u8CmdLen - 4) , Local_u32HostCRC);

	if(Local_u8CRCStatus == CRC_SUCCESS)
	{
		voidSendAck(1u); /*bootloader version has size of 1 byte*/

		//HAL_UART_Transmit(&huart2, &, , HAL_MAX_DELAY);

	}
	else
	{
		voidSendNAck();
	}
}

/*
 * @fn     : BL_VoidHandleEnRWProtectCmd
 * @brief  : Disable Write protection or Read and write protection
 * @param  : copy_pu8CmdPacket
 * @retval : void
 * */
void BL_VoidHandleDisWRProtectCmd(uint8_t* copy_pu8CmdPacket)
{
	uint8_t  Local_u8CRCStatus , Local_u8CmdLen;
	uint32_t Local_u32HostCRC;

	/**/

	Local_u8CmdLen = copy_pu8CmdPacket[0] + 1; /*the 1st byte already includes the length to follow*/

	Local_u32HostCRC = *((uint32_t*)(copy_pu8CmdPacket + Local_u8CmdLen - 4));

	Local_u8CRCStatus = u8VerifyCRC(copy_pu8CmdPacket, (Local_u8CmdLen - 4) , Local_u32HostCRC);

	if(Local_u8CRCStatus == CRC_SUCCESS)
	{
		uint8_t Local_u8ProtectingStatus;


		voidSendAck(1u); /*bootloader version has size of 1 byte*/

		FLASH_OPTKey_UnLock();

		/*Write Protection*/
		Local_u8ProtectingStatus = FLASH_DisWProtect();

		FLASH_OPTKey_Lock();

		HAL_UART_Transmit(&huart2, &Local_u8ProtectingStatus, 1 , HAL_MAX_DELAY);

	}
	else
	{
		voidSendNAck();
	}
}
