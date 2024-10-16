#ifndef _FLASH_H_
#define _FLASH_H_

#include "board.h"

/**************************************************************************************/
/* G431оƬ��128KFLASH������ҳ��ַ�ֲ����£�����1��BANK��64ҳ��ÿһҳ2kb��С */
#define STM32_FLASH_BASE        0x08000000      /* STM32 FLASH ��ʼ��ַ */
#define STM32_FLASH_SIZE        0x20000         /* STM32 FLASH �ܴ�С*/
#define STM32_FLASH_PAGE_SIZE   0x800           /* STM32 FLASH ҳ��С*/


u8 LL_flash_erase_page(u16 start, u8 len, u8 bank);     //ɾ��FLASH����
u8 LL_flash_read(u32 addr, u64* pdata64, u32 len_64);   //��Ƭ��FLASH
u8 LL_flash_write(u32 addr, u64* pdata64, u32 len_64);  //��Ƭ��FLASHд������

#endif
