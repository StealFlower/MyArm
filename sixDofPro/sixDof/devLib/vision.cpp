#include "vision.h"
#include "crc.h"
#include "schedule.h"
Vision vision;


#if VISION_DEBUG
uint8_t DEFORCE_Flag[7];
#endif
Vision::Vision(){
    tmpArmBuff = new vision_Recv_BigArm_Struct[51];
}
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

    /* USART1_TX Init */
    //大臂数据包DMA初始化
//    vision_BigArm_Tx_DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)&USART1->TDR;
//    vision_BigArm_Tx_DMA_InitStruct.MemoryOrM2MDstAddress = (uint32_t)&BigArmSendStruct;
//    vision_BigArm_Tx_DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
//    vision_BigArm_Tx_DMA_InitStruct.Mode = LL_DMA_MODE_NORMAL;
//    vision_BigArm_Tx_DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
//    vision_BigArm_Tx_DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
//    vision_BigArm_Tx_DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
//    vision_BigArm_Tx_DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
//    vision_BigArm_Tx_DMA_InitStruct.NbData = sizeof(BigArmSendStruct);
//    vision_BigArm_Tx_DMA_InitStruct.PeriphRequest = LL_DMAMUX_REQ_USART1_TX;
//    vision_BigArm_Tx_DMA_InitStruct.Priority = LL_DMA_PRIORITY_HIGH;
//    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_2, &vision_BigArm_Tx_DMA_InitStruct);
//    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
    //算法通信数据包初始化
//    vision_Tx_DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)&USART1->TDR;
//    vision_Tx_DMA_InitStruct.MemoryOrM2MDstAddress = (uint32_t)&visionStruct;
//    vision_Tx_DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
//    vision_Tx_DMA_InitStruct.Mode = LL_DMA_MODE_NORMAL;
//    vision_Tx_DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
//    vision_Tx_DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
//    vision_Tx_DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
//    vision_Tx_DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
//    vision_Tx_DMA_InitStruct.NbData = sizeof(visionStruct);
//    vision_Tx_DMA_InitStruct.PeriphRequest = LL_DMAMUX_REQ_USART1_TX;
//    vision_Tx_DMA_InitStruct.Priority = LL_DMA_PRIORITY_HIGH;
//    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_2, &vision_Tx_DMA_InitStruct);
//    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
    //请求识别数据包初始化
    vision_AquireRecognize_Tx_DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)&USART1->TDR;
    vision_AquireRecognize_Tx_DMA_InitStruct.MemoryOrM2MDstAddress = (uint32_t)&AcquireInfo;
    vision_AquireRecognize_Tx_DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    vision_AquireRecognize_Tx_DMA_InitStruct.Mode = LL_DMA_MODE_NORMAL;
    vision_AquireRecognize_Tx_DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    vision_AquireRecognize_Tx_DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    vision_AquireRecognize_Tx_DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    vision_AquireRecognize_Tx_DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    vision_AquireRecognize_Tx_DMA_InitStruct.NbData = sizeof(AcquireInfo);
    vision_AquireRecognize_Tx_DMA_InitStruct.PeriphRequest = LL_DMAMUX_REQ_USART1_TX;
    vision_AquireRecognize_Tx_DMA_InitStruct.Priority = LL_DMA_PRIORITY_HIGH;
    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_2, &vision_AquireRecognize_Tx_DMA_InitStruct);
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
    LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_DisableFIFO(USART1);
    LL_USART_ConfigAsyncMode(USART1);

    LL_USART_EnableDMAReq_RX(USART1);
    LL_USART_EnableDMAReq_TX(USART1);
    LL_USART_Enable(USART1);

    LL_USART_EnableIT_IDLE(USART1);
}

void Vision::sendData(uint8_t type)
{
    switch(type)
    {
        case BigArmData:
            /*****SetDefaultValue*****/
            // 设置传输数据长度
//            LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
//            this->BigArmSendStruct.header = 0xA5; // 帧头
//            this->BigArmSendStruct.type = 5;	//数据
//            this->BigArmSendStruct.time_stamp = sysTickUptime;

//            Append_CRC16_Check_Sum((u8 *)&BigArmSendStruct, sizeof(BigArmSendStruct));
//            // 打开DMA,开始发送
//            LL_DMA_DeInit(DMA1, LL_DMA_CHANNEL_2);
//            LL_DMA_Init(DMA1, LL_DMA_CHANNEL_2, &vision_BigArm_Tx_DMA_InitStruct);
//            LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, sizeof(BigArmSendStruct));
//            LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
        break;
        case visionData:
            /*****SetDefaultValue*****/
            // 设置传输数据长度
//            LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
//            this->visionStruct.header = 0xA5; // 帧头
//            this->visionStruct.time_stamp = sysTickUptime;

//            Append_CRC16_Check_Sum((u8 *)&visionStruct, sizeof(visionStruct));
//            // 打开DMA,开始发送
//            LL_DMA_DeInit(DMA1, LL_DMA_CHANNEL_2);
//            LL_DMA_Init(DMA1, LL_DMA_CHANNEL_2, &vision_Tx_DMA_InitStruct);
//            LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, sizeof(visionStruct));
//            LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
        break;
        case AcquireRecognize:
            /*****SetDefaultValue*****/
            // 设置传输数据长度
            LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
            this->AcquireInfo.header = 0xA5; // 帧头
            this->AcquireInfo.time_stamp = sysTickUptime;
            this->AcquireInfo.detect_mode = 1;

            Append_CRC16_Check_Sum((u8 *)&AcquireInfo, sizeof(AcquireInfo));
            // 打开DMA,开始发送
            LL_DMA_DeInit(DMA1, LL_DMA_CHANNEL_2);
            LL_DMA_Init(DMA1, LL_DMA_CHANNEL_2, &vision_AquireRecognize_Tx_DMA_InitStruct);
            LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, sizeof(AcquireInfo));
            LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
        break;
    }
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

        if(vision.tmpRecvBuff[0] == 0xA5){
            switch(vision.tmpRecvBuff[1]){
                case navigation://第一种接收数据结构体
                    // if(Verify_CRC16_Check_Sum((u8 *)&vision.tmpRecvBuff, sizeof(vision.navigationData))){
                    //     memcpy((u8 *)(&vision.navigationData), vision.tmpRecvBuff, sizeof(vision.navigationData));
                    //     vision.online.update(); // 在线更新
                    // }
                    // else
                    //     memset(&vision.recvStruct1, 0, sizeof(vision.recvStruct1));
                break;
                case visionRecv2://第二种接收数据结构体
//                    if(Verify_CRC16_Check_Sum((u8 *)&vision.tmpRecvBuff, sizeof(vision.recvStruct2))){
//                        memcpy((u8 *)(&vision.recvStruct2), vision.tmpRecvBuff, sizeof(vision.recvStruct2));
//                        vision.online.update(); // 在线更新
//                    }
//                    else
//                        memset(&vision.recvStruct2, 0, sizeof(vision.recvStruct2));
                break;
                case visionRecvBigArm://大臂数据包
                    if(Verify_CRC16_Check_Sum((u8 *)&vision.tmpRecvBuff, sizeof(vision.BigArmRecvStruct))){
//                        if(vision.armDataCnt == 51 && !vision.ArmBuffFullFlag){
//                            vision.ArmBuffFullFlag = 1;
//                            vision.ArmReceiveOkFlag = 1;
//                            vision.armDataLength = 50;
//                            vision.armDataCnt = 0;
//                            break;
//                        }
//                        else if(!vision.ArmReceiveOkFlag){
                            memcpy((u8 *)(&vision.BigArmRecvStruct), vision.tmpRecvBuff, sizeof(vision.BigArmRecvStruct));
                            //memcpy((u8 *)(&vision.tmpArmBuff[vision.armDataCnt++]), vision.tmpRecvBuff, sizeof(vision.BigArmRecvStruct));
//                            if(vision.BigArmRecvStruct.TransmitOkFlag){
//                                vision.ArmReceiveOkFlag = 1;
//                                vision.armDataLength = vision.armDataCnt-1;
//                                vision.armDataCnt = 0;
//                            }
//                        }

                        vision.online.update(); // 在线更新
                    }
                    else
                        memset(&vision.BigArmRecvStruct, 0, sizeof(vision.BigArmRecvStruct));
                break;
                case visionRecvStation:
                    if(Verify_CRC16_Check_Sum((u8 *)&vision.tmpRecvBuff, sizeof(vision.recvRecognize_RecvStruct))){
                        memcpy((u8 *)(&vision.recvRecognize_RecvStruct), vision.tmpRecvBuff, sizeof(vision.recvRecognize_RecvStruct));
                    }
                    vision.online.update();
                break;

                default:
                    vision.online.update();
                break;
            }
        }

        // if (vision.tmpRecvBuff[0] == 0xA5 && Verify_CRC16_Check_Sum((u8 *)&vision.tmpRecvBuff, sizeof(vision.recvStruct))) // 帧头检测 和 CRC校验
        // {
        //     memcpy((u8 *)(&vision.recvStruct), vision.tmpRecvBuff, sizeof(vision.recvStruct));
        //     vision.online.update(); // 在线更新
        // }
        // else
        //     memset(&vision.recvStruct, 0, sizeof(vision.recvStruct));

        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, sizeof(vision.tmpRecvBuff));
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    }
}
