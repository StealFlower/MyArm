#include "virtualTask.h"
#include "dbus.h"

/*
DR16接收机输出的信号为标准的DBUS协议数据,当遥控器与接收机建立连接
后，接收机每隔14ms通过DBUS发送一帧18字节数据.
*/

/**
 * @ingroup TDT_DEVICE
 * @defgroup DBUS 遥控器解算
 * @brief 该类提供了遥控器的数据解算，以及常用的重要标志位
 */
RemoteCtrl rcCtrl;
using namespace RCS;

uint8_t sbusRxBuffer[RC_FRAME_LENGTH]; // DMA传输完的数据
uint8_t sbusRxBuf[SBUS_RX_BUF_NUM];    // DMA缓存区

void RemoteCtrl::init()
{
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_DMA_InitTypeDef LL_DMA_Struct = {0};

    /* 初始化时钟资源*/
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

    /* GPIO引脚初始化*/
    GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
    NVIC_EnableIRQ(USART2_IRQn);

    /* 串口初始化*/
    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = 100000;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_9B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_EVEN;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART2, &USART_InitStruct);
    LL_USART_SetTXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_DisableFIFO(USART2);
    LL_USART_ConfigAsyncMode(USART2);

    LL_USART_EnableDMAReq_RX(USART2);
    LL_USART_Enable(USART2);

    /*配置 DMA*/
    LL_DMA_Struct.PeriphOrM2MSrcAddress = (uint32_t)&USART2->RDR;
    LL_DMA_Struct.MemoryOrM2MDstAddress = (uint32_t)sbusRxBuf;
    LL_DMA_Struct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    LL_DMA_Struct.Mode = LL_DMA_MODE_CIRCULAR;
    LL_DMA_Struct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    LL_DMA_Struct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    LL_DMA_Struct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    LL_DMA_Struct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    LL_DMA_Struct.NbData = (uint32_t)SBUS_RX_BUF_NUM;
    LL_DMA_Struct.PeriphRequest = LL_DMAMUX_REQ_USART2_RX;
    LL_DMA_Struct.Priority = LL_DMA_PRIORITY_HIGH;
    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_3, &LL_DMA_Struct);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);

    LL_USART_EnableIT_IDLE(USART2);
}

// 数据处理
void RemoteCtrl::handleData(volatile const uint8_t *sbusBuf)
{
    // 右摇杆横向  范围+-660    [0]8bits+[1]3bits
    rc.ch[0] = ((sbusBuf[0] | (sbusBuf[1] << 8)) & 0x07ff) - 1024; // Channel 0
    // 右摇杆纵向   范围+-660   [1]5bits+[2]6bits
    rc.ch[1] = (((sbusBuf[1] >> 3) | (sbusBuf[2] << 5)) & 0x07ff) - 1024; // Channel 1
    // 左摇杆横向   范围+-660   [2]2bits+[3]8bits+[4]1bit
    rc.ch[2] = (((sbusBuf[2] >> 6) | (sbusBuf[3] << 2) | (sbusBuf[4] << 10)) & 0x07ff) - 1024; // Channel 2
    // 左摇杆纵向   范围+-660   [4]7bits+[5]4bits
    rc.ch[3] = (((sbusBuf[4] >> 1) | (sbusBuf[5] << 7)) & 0x07ff) - 1024; // Channel 3
    // 左上角拨轮 范围+-660 [16]8bis+[17]3bits
    rc.ch[4] = -(((sbusBuf[16] | (sbusBuf[17] << 8)) & 0x07ff) - 1024);
    if (rc.ch[4] > -20 && rc.ch[4] < 20)
        rc.ch[4] = 0;
    
    // 左边开关  132 上中下     [5]2bits
    rc.sw1 = (SWPos)(((sbusBuf[5] >> 4) & 0x000C) >> 2);// Switch left
    // 右边开关  132 上中下     [5]2bits
    rc.sw2 = (SWPos)(((sbusBuf[5] >> 4) & 0x0003)); // Switch right

    
    // 鼠标X值  范围-32768~32768  [6]8bits+[7]8bits
    mouse.vx = ((sbusBuf[6]) | (sbusBuf[7] << 8));
    // 鼠标Y值  范围-32768~32768  [8]8bits+[9]8bits
    mouse.vy = -((sbusBuf[8]) | (sbusBuf[9] << 8));
    // 鼠标z值  范围-32768~32768  [10]8bits+[11]8bits
    mouse.vz = ((sbusBuf[10]) | (sbusBuf[11] << 8));

    // 鼠标左键  01  [12]8bits
    mouse.left = sbusBuf[12];
    // 鼠标右键  01  [13]8bits
    mouse.right = sbusBuf[13];

    
    // 键盘值   01  [14]8bits+[15]8bits
    key.keyValue = sbusBuf[14] | (sbusBuf[15] << 8);

}


void RemoteCtrl::motionDetect()
{
    /*▲ 跳变检测*/
    if (online.isOnline())
    {
        rc.sw1Tick = (SWTick)(rc.sw1 - rc.lastSw1);
        rc.sw2Tick = (SWTick)(rc.sw2 - rc.lastSw2);
    }
    
    // 按键跳变检测-异或操作，相同为零，不同为一，那么跳变的位就为1，再赋值给单独的变量
    mouse.leftTick = mouse.left ^ mouse.lastLeft;
    mouse.rightTick = mouse.right ^ mouse.lastRight;
    keyTick.keyValue = key.keyValue ^ lastKey.keyValue;
    
    // 按下值检测-跳变值并上上一次值的取反-那么跳变且上一次为0的键为1
    mouse.leftPress = mouse.leftTick & (~mouse.lastLeft);
    mouse.rightPress = mouse.rightTick & (~mouse.lastRight);
    keyPress.keyValue = keyTick.keyValue & (~lastKey.keyValue);
    
    // 松开值检测
    mouse.leftRelease = mouse.leftTick & (mouse.lastLeft);
    mouse.rightRelease = mouse.rightTick & (mouse.lastRight);
    keyRelease.keyValue = keyTick.keyValue & (lastKey.keyValue);
    
    // ▲ 记录当前值
    rc.lastSw1 = rc.sw1;
    rc.lastSw2 = rc.sw2;
    
    mouse.lastLeft = mouse.left;
    mouse.lastRight = mouse.right;
    
    lastKey = key;
}


extern uint8_t RobotEnable;
/**
 * @brief 任务调度
 * @note 通过遥控器指令执行相关任务的挂起和恢复
 * @warning 脱力控制也在这边，注意一下
 */
void RemoteCtrl::taskSchedule()
{
    // 离线
    if (online.isOffLine())
		RobotEnable = DISABLE;
    
    // ▲ 脱力键
    if (rc.sw2 == RCS::Down)
    {
        // 如果前一状态为使能,调用失能时回调函数
        if (RobotEnable == ENABLE)
        {
            // 调用所有任务RC失能回调函数
            for (int i = 0; i < VirtualTask::taskNum; i++)
                VirtualTask::taskList[i]->disableCallBack();
        }
        RobotEnable = DISABLE;
    }
    #if AUTO_FORCE
    else
    #else
	//脱力状态解除，恢复挂起的任务，执行一些初始化
    if(rc.sw2Tick == RCS::Down_Mid)//上拨上力
    #endif
    {
        // 如果前一状态为失能,调用使能时回调函数
        if (RobotEnable == DISABLE)
        {
            // 调用所有任务RC使能回调函数
            for (int i = 0; i < VirtualTask::taskNum; i++)
                VirtualTask::taskList[i]->enableCallBack();
        }
        RobotEnable = ENABLE;
    }

    // 调用所有任务RC更新回调函数
    for (uint8_t i = 0; i < VirtualTask::taskNum; i++)
        VirtualTask::taskList[i]->rcUpdateCallBack();
}

uint8_t len;
/**
 * @brief 遥控器串口接收中断
 */
void USART2_IRQHandler(void)
{
    if (LL_USART_IsActiveFlag_IDLE(USART2))
    {
        // 清除空闲中断标志
        LL_USART_ClearFlag_IDLE(USART2);

        static uint16_t this_time_rx_len = 0;
        LL_USART_ReceiveData8(USART2);

        len = LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3);
        // 计算接收数据长度
        this_time_rx_len = SBUS_RX_BUF_NUM - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3);
        // 重新设置DMA
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);

        // 数据处理
        if (this_time_rx_len == RC_FRAME_LENGTH)
        {
            memcpy(sbusRxBuffer, sbusRxBuf, RC_FRAME_LENGTH);
            rcCtrl.handleData(sbusRxBuffer); // 遥控器接收数据处理
            rcCtrl.online.update(); // 在线检测更新
        }

        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, SBUS_RX_BUF_NUM);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
    }
}

