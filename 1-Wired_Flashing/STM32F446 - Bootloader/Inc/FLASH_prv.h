#ifndef FLASH_PRV_H_
#define FLASH_PRV_H_

/*
 *@FLASH_KEYS
 *The following values must be programmed consecutively to unlock the FLASH_CR register 
	and allow programming/erasing it
 *@{
*/
#define FLASH_KEY1_		0x45670123
#define FLASH_KEY2_		0xCDEF89AB
/*
 *@}
*/

/*
 *@FLASH_OPT_KEYS
 *The following values must be programmed consecutively to unlock the FLASH_OPTCR 
	register and allow programming it
 *@{
*/
#define FLASH_OPT_KEY1_		0x08192A3B
#define FLASH_OPT_KEY2_		0x4C5D6E7F
/*
 *@}
*/


static void Flash_eraseSector(uint8_t index ,uint8_t V_Range );
static void FLASH_eraseMass(uint8_t V_Range ,uint32_t Banks);
static void voidProgramByte(uint32_t u32Address, uint8_t u8Data);
static void voidProgramHalfWord(uint32_t u32Address, uint16_t u16Data);
static void voidProgramWord(uint32_t u32Address, uint32_t u32Data);
static void voidProgramDoubleWord(uint32_t u32Address, uint64_t u64Data);

#endif
