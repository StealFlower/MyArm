#ifndef _FLASH_H_
#define _FLASH_H_

#include "board.h"

/**************************************************************************************/
/* G431芯片的128KFLASH容量的页地址分布如下，共有1个BANK，64页，每一页2kb大小 */
#define STM32_FLASH_BASE        0x08000000      /* STM32 FLASH 起始地址 */
#define STM32_FLASH_SIZE        0x20000         /* STM32 FLASH 总大小*/
#define STM32_FLASH_PAGE_SIZE   0x800           /* STM32 FLASH 页大小*/


u8 LL_flash_erase_page(u16 start, u8 len, u8 bank);     //删除FLASH扇区
u8 LL_flash_read(u32 addr, u64* pdata64, u32 len_64);   //读片上FLASH
u8 LL_flash_write(u32 addr, u64* pdata64, u32 len_64);  //向片上FLASH写入数据

#endif
