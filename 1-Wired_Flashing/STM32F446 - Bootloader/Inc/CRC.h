#ifndef CRC_H_
#define CRC_H_

/* @CRC_CR
 * @{
	*/
#define CR_RESET 0
/*
 * @}
    */

uint32_t CRC_u32Accumulate(uint32_t* Copy_pu32Buffer, uint32_t Copy_u32Length);

uint32_t CRC_u32Calculate(uint32_t* Copy_pu32Buffer, uint32_t Copy_u32Length);

void CRC_voidResetCRC(void);

#endif
