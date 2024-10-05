#include "schedule.h"
#include "virtualtask.h"
#include "iwdg.h"
#include "dbus.h"
#include "fdcan.h"
#include "motor.h"
#include "devList.h"
#include "vision.h"
#include "SMUniversal.h"
#include "oreMotionStateMachine.h"
#include "kinematics.h"
/*初始化完成标志*/
uint8_t Init_OK;
/*机器人使能标志*/
uint8_t RobotEnable;
uint8_t MotorOffLineFlag;
uint8_t warningFlag;
uint8_t MotorOffLineNum;

uint8_t OverHeatFlag;
uint8_t OverHeatWarning;
void boardALLInit(void)
{
    delayMs(500);
    air.gugu.init();
	// CAN初始化
	can.init(&hfdcan1);
	can.init(&hfdcan2);
	can.init(&hfdcan3);
    Motor::canMsgInit();

    loadParam();
	// 遥控器初始化
	rcCtrl.init();
    // 视觉初始化
    vision.init();
	// 任务初始化
	for (uint8_t i = 0; i < VirtualTask::taskNum; i++)
		VirtualTask::taskList[i]->init();
	// 看门狗初始化
	//  iwdg.init();
	/*初始化完成*/

	Init_OK = 1;
}

void TDT_Loop_1000Hz(void) // 1ms执行一次
{
}
void TDT_Loop_500Hz(void) // 2ms执行一次
{
	// 遥控器跳变按下松开动作检测
	rcCtrl.motionDetect();
    // 遥控器任务调度
    rcCtrl.taskSchedule();

	// 任务调度
	for (uint8_t i = 0; i < VirtualTask::taskNum; i++)
	{
		// 调用运行态任务的主函数
		if ((rcCtrl.online.isOnline() && RobotEnable == ENABLE && VirtualTask::taskList[i]->status == taskStateRun))
			VirtualTask::taskList[i]->run();
		else if (VirtualTask::taskList[i]->alwaysRunFlag == true)
			VirtualTask::taskList[i]->run();
	}        
    DEBUG();
    
//	// CAN通讯处理
	if (RobotEnable == DISABLE)
		can.disforceHandle();//脱力时CAN发送消息处理
	can.sendMsg();

    // 视觉通信
	// AllOnlineObject离线检查
	for (uint8_t i = 0; i < onlineNum; i++)
		onlinePtrList[i]->check();
    // 遥控器掉线检查
    if(rcCtrl.online.isOnline() == false)
        RobotEnable = DISABLE;

    //复位
    if(rcCtrl.rc.sw2 == RCS::Down && rcCtrl.rc.sw1Tick == RCS::Mid_Down)
        softwareReset();
}

void TDT_Loop_200Hz(void) // 5ms执行一次
{
    vision.sendData();

}

float current;
void TDT_Loop_100Hz(void) // 10ms执行一次
{
    //防2006尖叫程序
    //2006堵转后，脱力3.6s后恢复上力，上力3s后电流不变，开始bb响并且不受控制，第5s再次脱力
    //2006在3s大电流后会触发堵转保护，2s内不受控制，然后开始脱力3.6s，
    //也就是2006从听到声音后5.6s后才重新使能
    //必须在3s内限制住电流
    //方差小于100 0000并且电流大于4000超过2.5s甚至是2s

//    SmallLift.curVar = SmallLift.currentVarCale.caleVar(SmallLift.canInfo.trueCurrent);
//    switch(SmallLift.lockState)
//    {
//        case 0:
//            if(ABS(SmallLift.curVar)<1000000 && ABS(SmallLift.canInfo.trueCurrent)>4000)
//            {
//                //开始计时
//                SmallLift.lockCnt++;
//                if(SmallLift.lockCnt==250)//2.5s
//                {
//                    SmallLift.lockCnt=0;
//                    SmallLift.lockState++;
//                }
//            }
//            else
//            {
//                //脱离检测范围
//                SmallLift.lockCnt=0;
//            }
//        break;
//        case 1:
//            //设置输出限幅
//            SmallLift.setOutLimit(4000);

//            SmallLift.lockCnt++;
//            if(SmallLift.lockCnt==10)//100ms
//            {
//                SmallLift.lockCnt=0;

//                //重新加载电机参数
//                SmallLiftLoadParam();
//                SmallLift.lockState=0;
//            }
//        break;
//    }

//    SmallClimb.curVar = SmallClimb.currentVarCale.caleVar(SmallClimb.canInfo.trueCurrent);
//    switch(SmallClimb.lockState)
//    {
//        case 0:
//            if(ABS(SmallClimb.curVar)<1000000 && ABS(SmallClimb.canInfo.trueCurrent)>4000)
//            {
//                //开始计时
//                SmallClimb.lockCnt++;
//                if(SmallClimb.lockCnt==250)//2.5s
//                {
//                    SmallClimb.lockCnt=0;
//                    SmallClimb.lockState++;
//                }
//            }
//            else
//            {
//                //脱离检测范围
//                SmallClimb.lockCnt=0;
//            }
//        break;
//        case 1:
//            //设置输出限幅
//            SmallClimb.setOutLimit(2000);

//            SmallClimb.lockCnt++;
//            if(SmallClimb.lockCnt==10)//100ms
//            {
//                SmallClimb.lockCnt=0;

//                //重新加载电机参数
//                SmallClimbLoadParam();
//                SmallClimb.lockState=0;
//            }
//        break;
//    }

//    PitchClimb.curVar = PitchClimb.currentVarCale.caleVar(PitchClimb.canInfo.trueCurrent);
//    switch(PitchClimb.lockState)
//    {
//        case 0:
//            if(ABS(PitchClimb.curVar)<1000000 && ABS(PitchClimb.canInfo.trueCurrent)>4000)
//            {
//                //开始计时
//                PitchClimb.lockCnt++;
//                if(PitchClimb.lockCnt==100)//1s
//                {
//                    PitchClimb.lockCnt=0;
//                    PitchClimb.lockState++;
//                }
//            }
//            else
//            {
//                //脱离检测范围
//                PitchClimb.lockCnt=0;
//            }
//        break;
//        case 1:
//            //设置输出限幅
//            PitchClimb.setOutLimit(2000);

//            PitchClimb.lockCnt++;
//            if(PitchClimb.lockCnt==10)//100ms
//            {
//                PitchClimb.lockCnt=0;

//                //重新加载电机参数
//                PitchClimbLoadParam();
//                PitchClimb.lockState=0;
//            }
//        break;
//    }
//    //清除达妙电机错误标志,但是当mos过温后不清除,实际场上会出现mos过温情况，所以又加上了，但需要注意这样做会损坏FOC
//        if(rcCtrl.key.SHIFT && rcCtrl.key.CTRL && rcCtrl.key.F || (rcCtrl.rc.ch[0] == -660 && rcCtrl.rc.ch[1] == 660 && rcCtrl.rc.ch[2] == 660  && rcCtrl.rc.ch[3] == 660 && rcCtrl.rc.sw1 == RCS::Up)){
//            for(uint8_t i = 0;i < DmMotor::objectNum;i++){
//                if(DmMotor::objectPtrList[i]->canInfo.errCode == DmMotor::OverCur || DmMotor::objectPtrList[i]->canInfo.errCode == DmMotor::OverLoad ||
//                    DmMotor::objectPtrList[i]->canInfo.errCode == DmMotor::CoiOverTemp || DmMotor::objectPtrList[i]->canInfo.errCode == DmMotor::UnderVol || DmMotor::objectPtrList[i]->canInfo.errCode == DmMotor::OverVol){
//                    DmMotor::objectPtrList[i]->clearErr();
//                    DmMotor::objectPtrList[i]->motorEnable();
//                }
//            }
//        }
//    if(!imageTran.customCtrlData.deltaInfo.onlineFlag){//控制器离线，发送软件复位帧
//        MyCanTxStruct txMessage;
//        txMessage.txHeader.Identifier = 0x030;
//        self_CAN_Transmit(&hfdcan3,&txMessage);
//    }
}

void TDT_Loop_50Hz(void) // 20ms执行一次
{
    ///TODO: 机械臂力矩限幅处理
    //电机离线报警标志位检查
    for(u8 i = 0;i<Motor::objectNum;i++){
        if(Motor::objectPtrList[i]->online.isOffLine()){
            MotorOffLineFlag = 1;
            MotorOffLineNum++;
        }
    }
    for(u8 i = 0;i<DmMotor::objectNum;i++){
        if(DmMotor::objectPtrList[i]->online.isOffLine()){
            MotorOffLineFlag = 1;
            MotorOffLineNum++;
        }
    }
    if(MotorOffLineFlag){
        warningFlag = 1;
        MotorOffLineFlag = 0;
    }
    else{
        warningFlag = 0;
        MotorOffLineFlag = 0;
        MotorOffLineNum = 0;
    }
    //电机过温检查
    for(u8 i = 0;i<Motor::objectNum;i++){
        if(Motor::objectPtrList[i]->canInfo.temperature>80 && Motor::objectPtrList[i]->motorInfo.type != M2006
           && Motor::objectPtrList[i]->motorInfo.type != RMDX4 && Motor::objectPtrList[i]->motorInfo.type != GM3510){
            OverHeatFlag = 1;
        }
    }
    for(u8 i = 0;i<DmMotor::objectNum;i++){
        if(DmMotor::objectPtrList[i]->err == DmMotor::CoiOverTemp || DmMotor::objectPtrList[i]->canInfo.errCode == DmMotor::OverLoad){
            OverHeatFlag = 1;
        }
    }
}

void TDT_Loop_20Hz(void) // 50ms执行一次
{
}

void TDT_Loop_10Hz(void) // 100ms执行一次
{
}

void TDT_Loop_2Hz(void) // 500ms执行一次
{
}

void TDT_Loop_1Hz(void) // 1000ms执行一次
{
}

/**
 * @ingroup TDT_Frame
 * @defgroup TDT_SCHEDULE_API schedule相关接口
 * @brief 该模块展示了schedule的相关接口
 */

void TDT_Loop(struct _Schedule *robotSchedule)
{
	uint64_t startTimeStamp = getSysTimeUs();
	TDT_Loop_1000Hz();
	robotSchedule->runTime_1ms = getSysTimeUs() - startTimeStamp;

	if (robotSchedule->cnt_2ms >= 2)
	{
		robotSchedule->cnt_2ms = 0;
		uint64_t startTimeStamp = getSysTimeUs();
		TDT_Loop_500Hz();
		robotSchedule->runTime_2ms = getSysTimeUs() - startTimeStamp;
	}
	if (robotSchedule->cnt_5ms >= 5)
	{
		robotSchedule->cnt_5ms = 0;
		uint64_t startTimeStamp = getSysTimeUs();
		TDT_Loop_200Hz();
		robotSchedule->runTime_5ms = getSysTimeUs() - startTimeStamp;
	}
	if (robotSchedule->cnt_10ms >= 10)
	{
		robotSchedule->cnt_10ms = 0;
		uint64_t startTimeStamp = getSysTimeUs();
		TDT_Loop_100Hz();
		robotSchedule->runTime_10ms = getSysTimeUs() - startTimeStamp;
	}
	if (robotSchedule->cnt_20ms >= 20)
	{
		robotSchedule->cnt_20ms = 0;
		uint64_t startTimeStamp = getSysTimeUs();
		TDT_Loop_50Hz();
		robotSchedule->runTime_20ms = getSysTimeUs() - startTimeStamp;
	}
	if (robotSchedule->cnt_50ms >= 50)
	{
		robotSchedule->cnt_50ms = 0;
		uint64_t startTimeStamp = getSysTimeUs();
		TDT_Loop_20Hz();
		robotSchedule->runTime_50ms = getSysTimeUs() - startTimeStamp;
	}
	if (robotSchedule->cnt_100ms >= 100)
	{
		robotSchedule->cnt_100ms = 0;
		uint64_t startTimeStamp = getSysTimeUs();
		TDT_Loop_10Hz();
		robotSchedule->runTime_100ms = getSysTimeUs() - startTimeStamp;
	}
	if (robotSchedule->cnt_500ms >= 500)
	{
		robotSchedule->cnt_500ms = 0;
		uint64_t startTimeStamp = getSysTimeUs();
		TDT_Loop_2Hz();
		robotSchedule->runTime_500ms = getSysTimeUs() - startTimeStamp;
	}
	if (robotSchedule->cnt_1000ms >= 1000)
	{
		robotSchedule->cnt_1000ms = 0;
		uint64_t startTimeStamp = getSysTimeUs();
		TDT_Loop_1Hz();
		robotSchedule->runTime_1000ms = getSysTimeUs() - startTimeStamp;
	}
	robotSchedule->CPU_usage = (robotSchedule->runTime_1ms * 1000 + robotSchedule->runTime_2ms * 500 + robotSchedule->runTime_5ms * 200 + robotSchedule->runTime_10ms * 100 + robotSchedule->runTime_20ms * 50 + robotSchedule->runTime_50ms * 20 + robotSchedule->runTime_100ms * 10 + robotSchedule->runTime_500ms * 2 + robotSchedule->runTime_1000ms) / 1e6f;
}
