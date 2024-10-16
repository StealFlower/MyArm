#include "flash.h"
#include "stm32g4xx_hal_flash.h"

//按页删除片上FLASH数据
//start: 起始页号
//len:  待删除的页的数量
//bank: bank编号
//返回值：错误类型，0表示无错误
u8 LL_flash_erase_page(u16 start, u8 len, u8 bank)
{
  u32 err;
  u8 ret;
  FLASH_EraseInitTypeDef EraseInitStruct;
  u8 flash_count=0;
  
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;    //删除方式
  EraseInitStruct.Page        = start;                    //超始页号
  EraseInitStruct.NbPages     = len;                      //页的数量
  EraseInitStruct.Banks       = bank;                     //bank号
  
  HAL_FLASH_Unlock();         //解锁，以准备进行FLASH操作
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
  
  for(u8 i=0; i<10; i++)      //最多重复10次，如果仍然失败，则返回
  {
    ret = HAL_FLASHEx_Erase(&EraseInitStruct, &err);      //擦除
    if (ret == HAL_OK)
      break;
  }
  
  HAL_FLASH_Lock();           //上锁，以结束FLASH操作
  return ret;
}


//读片上FLASH
//addr: 32位的地址值,该值应当是8的整数倍，因为是按照64位的方式读取数据的
//pdata: 返回的数据首地址
//len: 待读取数据的长度， 长度是按u64的数量
//返回值: 错误类型，0表示无错误
u8 LL_flash_read(u32 addr, u64* pdata64, u32 len_64)
{
  u64 data;
  
  for(u32 i=0; i<len_64; i++)
  {
    data = *(__IO uint64_t *)(addr+i*8);
    pdata64[i] = data;
  }
  return 0;
}


//向片上FLASH写入数据，每次写入8字节，不够8字节的，以0xFF补足
//addr: 32位的地址值,该值应当是8的整数倍，因为是按照64位的方式读取数据的
//pdata: 待写入的数据首地址
//len: 待写入数据的长度，长度是按u64的数量
//返回值: 错误类型，0表示无错误
u8 LL_flash_write(u32 addr, u64* pdata64, u32 len_64)
{
  u8 ret;
  u64 read;
  
  HAL_FLASH_Unlock();           //上锁，以结束FLASH操作
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

  for(u16 j=0; j<len_64; j++) //按u64计算的
  {
    for(u8 i=0; i<10; i++)    //不超过10次的重复操作，以保证写入成功
    {
      ret = HAL_FLASH_Program(0, addr+j*8, *(pdata64+j));
      if(ret == HAL_OK)
      {
        read = *(__IO uint64_t *)(addr+j*8);  //在MCU认为写入正确以后，再次读取一下数据，并进行比对，如果比对不成功，也说明写入出错
        if(read != *(pdata64+j))
          ret = 255;
        return ret;
      }
    }
  }
  
  
  HAL_FLASH_Lock();           //上锁，以结束FLASH操作

  return ret;
}

