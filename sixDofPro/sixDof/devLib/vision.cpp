#include "vision.h"
#include "crc.h"
#include "schedule.h"

Vision vision;

void Vision::init()
{
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_DMA_InitTypeDef LL_DMA_Struct = {0};

    /* Peripheral clock enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    /**USART1 GPIO Configuration
    PA9   ------> USART1_TX
    PA10   ------> USART1_RX
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */

    /* USART1_RX Init */
    LL_DMA_Struct.PeriphOrM2MSrcAddress = (uint32_t)&USART1->RDR;
    LL_DMA_Struct.MemoryOrM2MDstAddress = (uint32_t)tmpRecvBuff;
    LL_DMA_Struct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    LL_DMA_Struct.Mode = LL_DMA_MODE_NORMAL;
    LL_DMA_Struct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    LL_DMA_Struct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    LL_DMA_Struct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    LL_DMA_Struct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    LL_DMA_Struct.NbData = sizeof(tmpRecvBuff);
    LL_DMA_Struct.PeriphRequest = LL_DMAMUX_REQ_USART1_RX;
    LL_DMA_Struct.Priority = LL_DMA_PRIORITY_HIGH;
    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_1, &LL_DMA_Struct);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

    vision_Tx_DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)&USART1->TDR;
    vision_Tx_DMA_InitStruct.MemoryOrM2MDstAddress = (uint32_t)&vision_send_struct;
    vision_Tx_DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    vision_Tx_DMA_InitStruct.Mode = LL_DMA_MODE_NORMAL;
    vision_Tx_DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    vision_Tx_DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    vision_Tx_DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    vision_Tx_DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    vision_Tx_DMA_InitStruct.NbData = sizeof(vision_send_struct);
    vision_Tx_DMA_InitStruct.PeriphRequest = LL_DMAMUX_REQ_USART1_TX;
    vision_Tx_DMA_InitStruct.Priority = LL_DMA_PRIORITY_HIGH;
    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_2, &vision_Tx_DMA_InitStruct);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

    /* USART1 interrupt Init */
    NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
    NVIC_EnableIRQ(USART1_IRQn);

    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART1, &USART_InitStruct);
    LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_4);
    LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_4);
    LL_USART_DisableFIFO(USART1);
    LL_USART_ConfigAsyncMode(USART1);

    LL_USART_EnableDMAReq_RX(USART1);
    LL_USART_EnableDMAReq_TX(USART1);
    LL_USART_Enable(USART1);

    LL_USART_EnableIT_IDLE(USART1);
}

void Vision::sendData(void)
{
    /*****SetDefaultValue*****/
    // 设置传输数据长度
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
    this->vision_send_struct.header = 0xA5; // 帧头
    this->vision_send_struct.time_stamp = sysTickUptime*0.0000001;
    this->vision_send_struct.type = 5;
    this->vision_send_struct.joint_position[0] = Yaw1.getPosition()*0.017452007f;
    this->vision_send_struct.joint_position[1] = Pitch1.getPosition()*0.017452007f;
    this->vision_send_struct.joint_position[2] = Pitch2.getPosition()*0.017452007f;
    this->vision_send_struct.joint_position[3] = Roll1.getPosition()*0.017452007f;
    this->vision_send_struct.joint_position[4] = wrist.getPitchAngle()*0.017452007f;
    this->vision_send_struct.joint_position[5] = wrist.getRollAngle()*0.017452007f;

       
    Append_CRC16_Check_Sum((u8 *)&vision_send_struct, sizeof(vision_send_struct));
    // 打开DMA,开始发送
    LL_DMA_DeInit(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_2, &vision_Tx_DMA_InitStruct);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, sizeof(vision_send_struct));
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

}
// 串口中断
void USART1_IRQHandler(void)
{
    if (LL_USART_IsActiveFlag_IDLE(USART1))
    {
        // 清除空闲中断标志
        LL_USART_ClearFlag_IDLE(USART1);
        LL_USART_ReceiveData8(USART1);

        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);

        if(vision.tmpRecvBuff[0] == 0xA5 && vision.tmpRecvBuff[1] == 0x03){
            if(Verify_CRC16_Check_Sum((u8 *)&vision.tmpRecvBuff, sizeof(vision.vision_recv_struct))){
                memcpy((u8 *)(&vision.vision_recv_struct), vision.tmpRecvBuff, sizeof(vision.vision_recv_struct));
            }
            vision.online.update();
            }

        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, sizeof(vision.tmpRecvBuff));
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    }
}
