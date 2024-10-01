#include "fdcan.h"
#include "devList.h"

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
void MX_FDCAN1_Init(void)
{
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = ENABLE; // 使能自动重传输
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = DISABLE;
    //正常的位的时间 = (1+NominalTimeSeg1+NominalTimeSeg2)*tq
    //tq = 1/(Hpclk/NominalPrescaler)
    //波特率 = 1/正常的位的时间 = 1/[(1+5+4)*(1/(170M/17))] = 1M
    hfdcan1.Init.NominalPrescaler = 17;
    hfdcan1.Init.NominalSyncJumpWidth = 1;
    hfdcan1.Init.NominalTimeSeg1 = 5;
    hfdcan1.Init.NominalTimeSeg2 = 4;
    hfdcan1.Init.StdFiltersNbr = 1;
    hfdcan1.Init.ExtFiltersNbr = 1;//启用扩展帧过滤器
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    HAL_FDCAN_Init(&hfdcan1);

    FDCAN_FilterTypeDef sFilterConfig;
    FDCAN_FilterTypeDef sFilterConfig2;
    /* Configure Rx filter */
   sFilterConfig.IdType = FDCAN_STANDARD_ID;
   sFilterConfig.FilterIndex = 0;
   sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
   sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
   sFilterConfig.FilterID1 = 0x000;
   sFilterConfig.FilterID2 = 0x7ff;
   HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
	// 配置过滤器2,用于接收扩展帧
    sFilterConfig2.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig2.FilterIndex = 0;
    sFilterConfig2.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig2.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig2.FilterID1 = 0x00000000;   // 扩展帧的ID                00000000 00000000 00000000 00000000
    sFilterConfig2.FilterID2 = 0x00000000;   // 掩码，匹配所有扩展帧      00011111 11111111 11111111 11111101
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig2);

    /* Configure global filter:
       Filter all remote frames with STD and EXT ID
       Reject non matching frames with STD ID and EXT ID */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    /*Configure the Rx FIFO operation mode：
      Rx FIFO overwrite mode*/
    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);

    HAL_FDCAN_Start(&hfdcan1);

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 使能接收中断
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_BUS_OFF, 0); // 使能错误中断
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2); //使能发送完成中断
}

void MX_FDCAN2_Init(void)
{
    hfdcan2.Instance = FDCAN2;
    hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan2.Init.AutoRetransmission = ENABLE;
    hfdcan2.Init.TransmitPause = DISABLE;
    hfdcan2.Init.ProtocolException = DISABLE;
    hfdcan2.Init.NominalPrescaler = 17;
    hfdcan2.Init.NominalSyncJumpWidth = 1;
    hfdcan2.Init.NominalTimeSeg1 = 5;
    hfdcan2.Init.NominalTimeSeg2 = 4;
    hfdcan2.Init.StdFiltersNbr = 1;
    hfdcan2.Init.ExtFiltersNbr = 1;//启用扩展帧过滤器
    hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    HAL_FDCAN_Init(&hfdcan2);

    FDCAN_FilterTypeDef sFilterConfig;
    FDCAN_FilterTypeDef sFilterConfig2;
    /* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x7ff;
    HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);
    // 配置过滤器2,用于接收扩展帧
    sFilterConfig2.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig2.FilterIndex = 0;
    sFilterConfig2.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig2.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig2.FilterID1 = 0x00000000;   // 扩展帧的ID                00000000 00000000 00000000 00000000
    sFilterConfig2.FilterID2 = 0x00000000;   // 掩码，匹配所有扩展帧      00011111 11111111 11111111 11111101
    HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig2);
    /* Configure global filter:
       Filter all remote frames with STD and EXT ID
       Reject non matching frames with STD ID and EXT ID */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    /*Configure the Rx FIFO operation mode：
      Rx FIFO overwrite mode*/
    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan2, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);

    HAL_FDCAN_Start(&hfdcan2);

    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_BUS_OFF, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);
}
/* FDCAN3 init function */
void MX_FDCAN3_Init(void)
{
    hfdcan3.Instance = FDCAN3;
    hfdcan3.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan3.Init.AutoRetransmission = ENABLE;
    hfdcan3.Init.TransmitPause = DISABLE;
    hfdcan3.Init.ProtocolException = DISABLE;
    hfdcan3.Init.NominalPrescaler = 17;
    hfdcan3.Init.NominalSyncJumpWidth = 1;
    hfdcan3.Init.NominalTimeSeg1 = 5;
    hfdcan3.Init.NominalTimeSeg2 = 4;
    hfdcan3.Init.StdFiltersNbr = 1;
    hfdcan3.Init.ExtFiltersNbr = 0;
    hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    HAL_FDCAN_Init(&hfdcan3);

    FDCAN_FilterTypeDef sFilterConfig;

    /* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x7ff;
    HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig);

    /* Configure global filter:
       Filter all remote frames with STD and EXT ID
       Reject non matching frames with STD ID and EXT ID */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    /*Configure the Rx FIFO operation mode：
      Rx FIFO overwrite mode*/
    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan3, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);

    HAL_FDCAN_Start(&hfdcan3);

    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_BUS_OFF, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);
}

static uint32_t HAL_RCC_FDCAN_CLK_ENABLED = 0;

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (fdcanHandle->Instance == FDCAN1)
    {
        /* FDCAN1 clock enable */
        HAL_RCC_FDCAN_CLK_ENABLED++;
        if (HAL_RCC_FDCAN_CLK_ENABLED == 1)
        {
            __HAL_RCC_FDCAN_CLK_ENABLE();
        }

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**FDCAN1 GPIO Configuration
        PB8-BOOT0     ------> FDCAN1_RX
        PB9     ------> FDCAN1_TX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* FDCAN1 interrupt Init */
        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    }
    else if (fdcanHandle->Instance == FDCAN2)
    {
        /* FDCAN2 clock enable */
        HAL_RCC_FDCAN_CLK_ENABLED++;
        if (HAL_RCC_FDCAN_CLK_ENABLED == 1)
        {
            __HAL_RCC_FDCAN_CLK_ENABLE();
        }

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**FDCAN2 GPIO Configuration
        PB5     ------> FDCAN2_RX
        PB6     ------> FDCAN2_TX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* FDCAN2 interrupt Init */
        HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    }
    else if (fdcanHandle->Instance == FDCAN3)
    {
        /* FDCAN3 clock enable */
        HAL_RCC_FDCAN_CLK_ENABLED++;
        if (HAL_RCC_FDCAN_CLK_ENABLED == 1)
        {
            __HAL_RCC_FDCAN_CLK_ENABLE();
        }

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**FDCAN3 GPIO Configuration
        PB3     ------> FDCAN3_RX
        PB4     ------> FDCAN3_TX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF11_FDCAN3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* FDCAN3 interrupt Init */
        HAL_NVIC_SetPriority(FDCAN3_IT0_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
    }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *fdcanHandle)
{

    if (fdcanHandle->Instance == FDCAN1)
    {
        /* Peripheral clock disable */
        HAL_RCC_FDCAN_CLK_ENABLED--;
        if (HAL_RCC_FDCAN_CLK_ENABLED == 0)
        {
            __HAL_RCC_FDCAN_CLK_DISABLE();
        }

        /**FDCAN1 GPIO Configuration
        PB8-BOOT0     ------> FDCAN1_RX
        PB9     ------> FDCAN1_TX
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);

        /* FDCAN1 interrupt Deinit */
        HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
        /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

        /* USER CODE END FDCAN1_MspDeInit 1 */
    }
    else if (fdcanHandle->Instance == FDCAN2)
    {
        /* Peripheral clock disable */
        HAL_RCC_FDCAN_CLK_ENABLED--;
        if (HAL_RCC_FDCAN_CLK_ENABLED == 0)
        {
            __HAL_RCC_FDCAN_CLK_DISABLE();
        }

        /**FDCAN2 GPIO Configuration
        PB5     ------> FDCAN2_RX
        PB6     ------> FDCAN2_TX
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_6);

        /* FDCAN2 interrupt Deinit */
        HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
    }
    else if (fdcanHandle->Instance == FDCAN3)
    {
        /* Peripheral clock disable */
        HAL_RCC_FDCAN_CLK_ENABLED--;
        if (HAL_RCC_FDCAN_CLK_ENABLED == 0)
        {
            __HAL_RCC_FDCAN_CLK_DISABLE();
        }

        /**FDCAN3 GPIO Configuration
        PB3     ------> FDCAN3_RX
        PB4     ------> FDCAN3_TX
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3 | GPIO_PIN_4);

        /* FDCAN3 interrupt Deinit */
        HAL_NVIC_DisableIRQ(FDCAN3_IT0_IRQn);
    }
}

void FDCAN1_IT0_IRQHandler(void)
{

    HAL_FDCAN_IRQHandler(&hfdcan1);
}

void FDCAN2_IT0_IRQHandler(void)
{

    HAL_FDCAN_IRQHandler(&hfdcan2);
}

void FDCAN3_IT0_IRQHandler(void)
{

    HAL_FDCAN_IRQHandler(&hfdcan3);
}

uint8_t canSendMsgNum;
CanSendMsg **canSendMsgList;
CanSendMsg *canSendMsgView[16];

CanSendMsg::CanSendMsg(FDCAN_HandleTypeDef *hfdcan, bool enableFlag, uint32_t IdType) : enableFlag(enableFlag) , IdType(IdType)
{
    this->hfdcan = hfdcan;
    enableFlag = true;
    canSendMsgNum++;

    if (canSendMsgList == 0)
        canSendMsgList = (CanSendMsg **)malloc(sizeof(CanSendMsg *));
    else
        canSendMsgList = (CanSendMsg **)realloc(canSendMsgList, sizeof(CanSendMsg *) * canSendMsgNum);

    canSendMsgList[canSendMsgNum - 1] = this;
    canSendMsgView[canSendMsgNum - 1] = this;

    txHeader.IdType = IdType;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;
}

struct SelfCanFIFO
{
    SelfCanFIFO(FDCAN_HandleTypeDef *_hfdcan) : hfdcan(_hfdcan){}; // 构造器
    uint8_t push(MyCanTxStruct _canTxMsg); // 存入队列
    void trySend(void);                    // 尝试发送
    inline u8 getFIFOCount() { return (rear - front + FIFOLenth) % FIFOLenth; }; // 获取队列个数
    inline u8 getFIFOLenth() { return FIFOLenth; }                               // 获取队列容量
    inline FDCAN_HandleTypeDef *getCanX() { return hfdcan; };                    // 获取can
private:
    FDCAN_HandleTypeDef *hfdcan;
    MyCanTxStruct canTxMsgList[14] = {0};
    const u8 FIFOLenth = sizeof(canTxMsgList) / sizeof(MyCanTxStruct);

    // |消息|队列头|消息|消息|消息|队列尾|消息|消息|
    u8 front = 0; // 队列头索引,范围:0~FIFOLenth-1
    u8 rear = 0;  // 队列尾索引,范围:0~FIFOLenth-1
    u8 FIFO_fullFlag = 0;

    uint8_t pop(void);                     // 弹出队列
};

/// @brief 将消息存入队列并尝试发送
/// @param _canTxMsg 新消息
/// @return
uint8_t SelfCanFIFO::push(MyCanTxStruct _canTxMsg)
{
    // 添加消息 队列尾前进
    uint8_t nextRear = (rear + 1) % FIFOLenth;

    // 队列尾赶上队列头时，队列溢出
    if (nextRear == front)
    {
        FIFO_fullFlag = 1; // 邮箱满
        trySend();         // 尝试发送
        return 0;          // 溢出时退出 舍弃最新消息
    }

    // 队列未溢出 将消息添加至队列
    rear = nextRear;
    canTxMsgList[rear] = _canTxMsg;
    trySend(); // 尝试发送
    return 1;
}

uint8_t SelfCanFIFO::pop(void)
{
    // 队列头与队列尾一致 队列为空
    if (front == rear)
        return 0;
    // 队列不为空 弹出队列头
    front = (front + 1) % FIFOLenth;
    return 1;
}

void SelfCanFIFO::trySend()
{
    /* 禁止全局中断*/
    __disable_irq();
    while (1)
    {
        if (getFIFOCount() == 0) // 头和尾相同
            break;
        if (hfdcan->Instance->TXFQS & FDCAN_TXFQS_TFQF) // 邮箱满
            break;
        // 邮箱未满 添加消息到邮箱
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &canTxMsgList[front].txHeader, canTxMsgList[front].txData);
        pop(); // 弹栈
    }
    /*  使能全局中断 */
    __enable_irq();
}

SelfCanFIFO selfCanFIFO[3]{SelfCanFIFO(&hfdcan1), SelfCanFIFO(&hfdcan2), SelfCanFIFO(&hfdcan3)};

uint8_t self_CAN_Transmit(FDCAN_HandleTypeDef *hfdcan, MyCanTxStruct *TxMessage)
{
    if (hfdcan == &hfdcan1)
        return selfCanFIFO[0].push(*TxMessage);
    else if (hfdcan == &hfdcan2)
        return selfCanFIFO[1].push(*TxMessage);
    else
        return selfCanFIFO[2].push(*TxMessage);
}

Can can;
void Can::init(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1)
    {
        MX_FDCAN1_Init();
    }
    else if (hfdcan == &hfdcan2)
    {
        MX_FDCAN2_Init();
    }
    else if (hfdcan == &hfdcan3)
    {
        MX_FDCAN3_Init();
    }
}


/// @brief 将定义的所有使能的CAN消息添加到FIFO中并尝试发送
void Can::sendMsg()
{
    for (uint8_t i = 0; i < canSendMsgNum; i++)
    {
        if ((canSendMsgList[i])->enableFlag && (canSendMsgList[i])->IdType == FDCAN_STANDARD_ID)
        {
            MyCanTxStruct txMessage;
            txMessage.txHeader=canSendMsgList[i]->txHeader;
            memcpy(txMessage.txData,canSendMsgList[i]->txData,8);
            
            self_CAN_Transmit(canSendMsgList[i]->hfdcan,&txMessage);
        }
    }
}

void Can::sendExtMsg(){
    for(uint8_t i = 0;i < canSendMsgNum; i++){
        if((canSendMsgList[i])->enableFlag && (canSendMsgList[i])->IdType == FDCAN_EXTENDED_ID){
            MyCanTxStruct txMessage;
            txMessage.txHeader=canSendMsgList[i]->txHeader;
            memcpy(txMessage.txData,canSendMsgList[i]->txData,8);
            self_CAN_Transmit(canSendMsgList[i]->hfdcan,&txMessage);
        }
    }
}

/// @brief 脱力时CAN发送报文处理
void Can::disforceHandle()
{
    // 大疆电机脱力处理
    for(uint8_t i=0;i< Motor::objectNum;i++)
    {
        Motor::objectPtrList[i]->motorPowerOut(0);
    }

    // 达妙电机脱力处理
    for(uint8_t i=0;i< DmMotor::objectNum;i++)
    {
        DmMotor::objectPtrList[i]->mitCtrl(0, 0, 0, 0, 0);
    }

    // 气泵板脱力处理
    air.Stop();
	//宇树电机脱力处理
//	for(uint8_t i=0;i< utMotor::objectNum;i++)
//	{
//		utMotor::objectPtrList[i]->motorDisable();
//		utMotor::objectPtrList[i]->onforceFlag = 0;
//	}
    //...

}

u8 Pitch2CheckFlag = 1;
/// @brief 接收中断回调函数
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        /* Retrieve Rx messages from RX FIFO0 */
        if (hfdcan == &hfdcan1)
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                   &can.can1RxMsg.rxHeader, can.can1RxMsg.rxData);
            can.can1RxMsg.StdId = can.can1RxMsg.rxHeader.Identifier;

            // 大疆电调/银河电调数据接收处理
            if (can.can1RxMsg.StdId > 0x200 && can.can1RxMsg.StdId < 0x20D && can.can1RxMsg.rxHeader.IdType == FDCAN_STANDARD_ID)
            {
                if (motorList[0][can.can1RxMsg.StdId - 0x201].motorPoint != 0)
                    motorList[0][can.can1RxMsg.StdId - 0x201].motorPoint->canHandle(can.can1RxMsg.rxData);
            }

            //小米电机数据处理
//			if(can.can1RxMsg.rxHeader.IdType == FDCAN_EXTENDED_ID){
//				for(uint8_t i=0;i< CyberMotor::objectNum;i++)
//                {
//                    CyberMotor::objectPtrList[i]->canHandle(&can.can1RxMsg.rxHeader.Identifier , can.can1RxMsg.rxData);
//                }
//			}
            //...
        }
        else if (hfdcan == &hfdcan2)
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                   &can.can2RxMsg.rxHeader, can.can2RxMsg.rxData);
            can.can2RxMsg.StdId = can.can2RxMsg.rxHeader.Identifier;

            // 大疆电调/银河电调数据接收处理
            if (can.can2RxMsg.StdId > 0x200 && can.can2RxMsg.StdId < 0x20D && can.can2RxMsg.rxHeader.IdType == FDCAN_STANDARD_ID)
            {
                if (motorList[1][can.can2RxMsg.StdId - 0x201].motorPoint != 0)
                    motorList[1][can.can2RxMsg.StdId - 0x201].motorPoint->canHandle(can.can2RxMsg.rxData);
            }
         
            //小米电机接收数据处理
//            if(can.can2RxMsg.rxHeader.IdType == FDCAN_EXTENDED_ID){
//				for(uint8_t i=0;i< CyberMotor::objectNum;i++)
//                {
//                    CyberMotor::objectPtrList[i]->canHandle(&can.can2RxMsg.rxHeader.Identifier , can.can2RxMsg.rxData);
//                }
//			}
            //宇树电机副控回传参数处理
//            if((can.can2RxMsg.rxHeader.Identifier < 0x108) && (can.can2RxMsg.rxHeader.Identifier > 0x80)){
//				for(uint8_t i=0;i< utMotor::objectNum;i++){
//					utMotor::objectPtrList[i]->canHandle(&can.can2RxMsg.rxHeader.Identifier,can.can2RxMsg.rxData);
//				}
//            }
            //编码器数据接收
            // if(can.can2RxMsg.rxHeader.Identifier == 0x0A){
            //     memcpy(&BigPitch.caledPos,can.can2RxMsg.rxData,4);
            //     if(BigPitch.caledPos>300){
            //         BigPitch.caledPos -= 360.f;
            //     }
            //     BigPitch.posErr = BigPitch.setPosition - BigPitch.caledPos;
            // }
            // if(can.can2RxMsg.rxHeader.Identifier == 0x0B){
            //     memcpy(&Pitch2.Encoder_Angle,can.can2RxMsg.rxData,4);
            //     if(Pitch2.Encoder_Angle>300){
            //         Pitch2.Encoder_Angle -= 360.f;
            //     }
            //     Pitch2.posErr = Pitch2.setPosition - Pitch2.caledPos;
            // }
            // 达妙电机数据接收处理

            
            //...
        }
        else if (hfdcan == &hfdcan3)
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                   &can.can3RxMsg.rxHeader, can.can3RxMsg.rxData);
            can.can3RxMsg.StdId = can.can3RxMsg.rxHeader.Identifier;

            // 大疆电调/银河电调数据接收处理
            if (can.can3RxMsg.StdId > 0x200 && can.can3RxMsg.StdId < 0x20D && can.can3RxMsg.rxHeader.IdType == FDCAN_STANDARD_ID)
            {
                if (motorList[2][can.can3RxMsg.StdId - 0x201].motorPoint != 0)
                    motorList[2][can.can3RxMsg.StdId - 0x201].motorPoint->canHandle(can.can3RxMsg.rxData);
            }
           if (can.can3RxMsg.StdId == Yaw1.motorInfo.masterId){
                Yaw1.canHandle(can.can3RxMsg.rxData);
            }            
            if (can.can3RxMsg.StdId == Pitch1.motorInfo.masterId){
                Pitch1.canHandle(can.can3RxMsg.rxData);
            }
            if (can.can3RxMsg.StdId == Pitch2.motorInfo.masterId){
                Pitch2.canHandle(can.can3RxMsg.rxData);
            }  

            // 图传链路数据接收处理
            if(can.can3RxMsg.rxHeader.Identifier>0x400 && can.can3RxMsg.rxHeader.Identifier<0x405   //0x401~0x404
            || can.can3RxMsg.rxHeader.Identifier>0x500 && can.can3RxMsg.rxHeader.Identifier<0x502)  //0x501~0x502
            {
                imageTran.canHandle(&can.can3RxMsg);
            }
            //...
        }
    }
}

uint32_t errState;
/// @brief 错误中断回调函数
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    errState = HAL_FDCAN_GetError(&hfdcan1);
    CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
}


/// @brief 发送完成回调函数 发送完成继续发送MyFIFO中的消息
void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
    if (hfdcan == &hfdcan1)
        return selfCanFIFO[0].trySend();
    else if (hfdcan == &hfdcan2)
        return selfCanFIFO[1].trySend();
    else
        return selfCanFIFO[2].trySend();
}