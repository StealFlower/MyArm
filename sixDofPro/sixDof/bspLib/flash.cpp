#include "flash.h"
#include "stm32g4xx_hal_flash.h"

//��ҳɾ��Ƭ��FLASH����
//start: ��ʼҳ��
//len:  ��ɾ����ҳ������
//bank: bank���
//����ֵ���������ͣ�0��ʾ�޴���
u8 LL_flash_erase_page(u16 start, u8 len, u8 bank)
{
  u32 err;
  u8 ret;
  FLASH_EraseInitTypeDef EraseInitStruct;
  u8 flash_count=0;
  
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;    //ɾ����ʽ
  EraseInitStruct.Page        = start;                    //��ʼҳ��
  EraseInitStruct.NbPages     = len;                      //ҳ������
  EraseInitStruct.Banks       = bank;                     //bank��
  
  HAL_FLASH_Unlock();         //��������׼������FLASH����
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
  
  for(u8 i=0; i<10; i++)      //����ظ�10�Σ������Ȼʧ�ܣ��򷵻�
  {
    ret = HAL_FLASHEx_Erase(&EraseInitStruct, &err);      //����
    if (ret == HAL_OK)
      break;
  }
  
  HAL_FLASH_Lock();           //�������Խ���FLASH����
  return ret;
}


//��Ƭ��FLASH
//addr: 32λ�ĵ�ֵַ,��ֵӦ����8������������Ϊ�ǰ���64λ�ķ�ʽ��ȡ���ݵ�
//pdata: ���ص������׵�ַ
//len: ����ȡ���ݵĳ��ȣ� �����ǰ�u64������
//����ֵ: �������ͣ�0��ʾ�޴���
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


//��Ƭ��FLASHд�����ݣ�ÿ��д��8�ֽڣ�����8�ֽڵģ���0xFF����
//addr: 32λ�ĵ�ֵַ,��ֵӦ����8������������Ϊ�ǰ���64λ�ķ�ʽ��ȡ���ݵ�
//pdata: ��д��������׵�ַ
//len: ��д�����ݵĳ��ȣ������ǰ�u64������
//����ֵ: �������ͣ�0��ʾ�޴���
u8 LL_flash_write(u32 addr, u64* pdata64, u32 len_64)
{
  u8 ret;
  u64 read;
  
  HAL_FLASH_Unlock();           //�������Խ���FLASH����
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

  for(u16 j=0; j<len_64; j++) //��u64�����
  {
    for(u8 i=0; i<10; i++)    //������10�ε��ظ��������Ա�֤д��ɹ�
    {
      ret = HAL_FLASH_Program(0, addr+j*8, *(pdata64+j));
      if(ret == HAL_OK)
      {
        read = *(__IO uint64_t *)(addr+j*8);  //��MCU��Ϊд����ȷ�Ժ��ٴζ�ȡһ�����ݣ������бȶԣ�����ȶԲ��ɹ���Ҳ˵��д�����
        if(read != *(pdata64+j))
          ret = 255;
        return ret;
      }
    }
  }
  
  
  HAL_FLASH_Lock();           //�������Խ���FLASH����

  return ret;
}

