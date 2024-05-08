/*
 * BL.h
 *
 *  Created on: Apr 6, 2024
 *      Author: Usef
 */

#ifndef INC_BL_H_
#define INC_BL_H_

#define BL_ACK					0xA5
#define BL_NACK					0x7F

#define BL_GET_VER       		0x51
#define BL_GET_HELP      		0x52
#define BL_GET_CID      	 	0x53
#define BL_GET_RDP_STATUS       0x54
#define BL_GO_TO_ADDR       	0x55
#define BL_FLASH_ERASE       	0x56
#define BL_MEM_WRITE       		0x57
#define BL_EN_RW_PROTECT        0x58
#define BL_MEM_READ       		0x59
#define BL_READ_SECTOR_STATUS   0x5A
#define BL_OTP_READ       		0x5B
#define BL_DIS_WR_PROTECT       0x5C

void BL_VoidHandleGetVerCmd(uint8_t* copy_pu8CmdPacket);
void BL_VoidHandleGetHelpCmd(uint8_t* copy_pu8CmdPacket);
void BL_VoidHandleGetCIDCmd(uint8_t* copy_pu8CmdPacket);
void BL_VoidHandleGetRDPStatusCmd(uint8_t* copy_pu8CmdPacket);
void BL_VoidHandleGoToAddressCmd(uint8_t* copy_pu8CmdPacket);
void BL_VoidHandleFlashEraseCmd(uint8_t* copy_pu8CmdPacket);
void BL_VoidHandleMemWriteCmd(uint8_t* copy_pu8CmdPacket);
void BL_VoidHandleEnRWProtectCmd(uint8_t* copy_pu8CmdPacket);
void BL_VoidHandleMemReadCmd(uint8_t* copy_pu8CmdPacket);
void BL_VoidHandleReadSectorStatusCmd(uint8_t* copy_pu8CmdPacket);
void BL_VoidHandleOTPReadCmd(uint8_t* copy_pu8CmdPacket);
void BL_VoidHandleDisWRProtectCmd(uint8_t* copy_pu8CmdPacket);

#endif /* INC_BL_H_ */
