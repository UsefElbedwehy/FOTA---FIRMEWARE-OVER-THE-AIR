/*
 * BL_prv.h
 *
 *  Created on: Apr 6, 2024
 *      Author: Usef
 */

#ifndef INC_BL_PRV_H_
#define INC_BL_PRV_H_

#define CRC_SUCCESS				0u
#define CRC_FAIL				1u

#define BL_VERSION				1u

#define VALID_ADDRESS			0u
#define INVALID_ADDRESS			1u

#define WRITING_SUCCESS			0u
#define WRITING_ERROR			1u

#define PROTECTING_SUCCESS		0u
#define PROTECTING_ERROR		1u

#define RW_PROTECTING			0u
#define W_PROTECTING			1u



#define DBGMCU_ICCODE_REG 		*((volatile uint32_t*) 0xE0042000)
#define RDP_USER_OPTION_WORD    *((volatile uint32_t*) 0x1FFFC000)



static uint8_t u8VerifyCRC(uint8_t* copy_pu8DataArray , uint8_t Copy_u8Length ,uint32_t Copy_u32HostCRC);
static void voidSendAck(uint8_t Copy_u8ReplyLength);
static void voidSendNAck(void);
static uint8_t u8ValidateAddress(uint32_t Copy_u32Address);
static uint8_t u8ExecuteFlashErase(uint8_t Copy_u8SectorNumber ,uint8_t Copy_u8NumberOfSectors);
static uint8_t u8ExecuteMemWrite(uint8_t* Copy_pu8Buffer ,uint32_t Copy_u32Address , uint8_t Copy_u8Length);

#endif /* INC_BL_PRV_H_ */
