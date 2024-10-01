#include "chassis.h"
#include "dbus.h"
#include "devList.h"
#include "oreMotionStateMachine.h"
#include "vision.h"
#define MAX_DPS (9158*6) //车轮速度软件限幅,不能大于真实值,否则会导致方向合成存在偏差

uint8_t NAVIGATION_ENABLE_FLAG;
enum SpeedCoefficient {ctrl=0,none=1,shift=2,};
enum {fb=0,lr=1,ro=2,};


//fb为麦轮前进方向
//lr为麦轮左右方向。与云台方向无关

//键盘按键速度系数，依据操作手手感、机器人性能调整
float speedCoefficient[5][5][3] = {
//  {//前后 左右 旋转
//    {0.20,0.20,1.00}, //按下ctrl时的前后、左右、自旋的速度系数
//    {0.60,0.60,0.75}, //不按下ctrl也不按下shift时的前后、左右、自旋的速度系数
//    {0.85,0.75,0.75}, //按下shift时的前后、左右、自旋的速度系数
//    {2.00,3.00,1.00}, //前后、左右、自旋(暂时没有用到)的加速加速度，越大起步越快。起步越快机器人越容易打滑
//    {2.00,2.00,1.00}, //前后、左右、自旋(暂时没有用到)的减速加速度，越大刹车越缓。刹车越急机器人越容易翻车
//  },

    //平跑 Lowest
    {//前后 左右 旋转
        {0.20,0.20,1.00},/*ctrl*/
        {0.50,0.50,0.75},/*none*/
        {0.85,0.75,0.75},/*shift*/
        {1.60,3.00,1.00},//加速加速度
        {2.00,2.00,1.00},//减速加速度
    },

    //取矿
    {//前后 左右 旋转
        {0.08,0.08,0.08},/*ctrl*/
        {0.15,0.15,0.15},/*none*/
        {0.25,0.25,0.25},/*shift*/
        {2.00,3.00,1.00},//加速加速度
        {2.00,2.00,1.00},//减速加速度
    },

    //兑换
    {//前后 左右 旋转
        {0.08,0.08,0.08},/*ctrl*/
        {0.10,0.10,0.10},/*none*/
        {0.16,0.16,0.24},/*shift*/
        {2.00,4.00,2.00},//加速加速度
        {2.00,4.00,2.00},//减速加速度
    },

    //三矿限制
    {//前后 左右 旋转
        {0.08,0.08,0.20},/*ctrl*/
        {0.30,0.30,0.30},/*none*/
        {0.60,0.40,0.40},/*shift*/
        {2.00,4.00,2.00},//加速加速度
        {1.00,1.00,0.50},//减速加速度
    },

    //平跑 Normal
    {//前后 左右 旋转
        {0.20,0.20,1.00},/*ctrl*/
        {0.50,0.50,0.75},/*none*/
        {0.65,0.65,0.75},/*shift*/
        {2.00,3.00,1.00},//加速加速度
        {2.00,2.00,1.00},//减速加速度
    },
};


Chassis::Chassis()
{
    //电机参数加载
    loadParam();
}


/**
  * @brief 电机PID参数加载
  */
void Chassis::loadParam()
{
    for(u8 i=0; i<4; i++)
    {
        //方案0
		motor[i].setPlan(0, spd, &spdParam[0]);
		motor[i].setParam(0,spd,2.4,0.2,0,400,16384);//kp高于2.5会抖
        //方案1
		motor[i].setPlan(1, spd, &spdParam[1]);
		motor[i].setParam(1,spd,2.5,0,0,0,16384);
    }
}


/**
  * @brief  获取当前底盘系数
  * @param  fbOrLr：0前后，1左右
  * @retval 返回当前底盘系数
  */
float Chassis::getNowIndex(u8 fbOrLr)
{
    //麦轮速度分解
    float dps = 0;
    if (fbOrLr == 0) //fb
    {
        dps += motor[0].canInfo.dps;//左前
        dps -= motor[1].canInfo.dps;//右前
        dps -= motor[2].canInfo.dps;//右后
        dps += motor[3].canInfo.dps;//左后
    }
    else//lr
    {
        dps += motor[0].canInfo.dps;
        dps += motor[1].canInfo.dps;
        dps -= motor[2].canInfo.dps;
        dps -= motor[3].canInfo.dps;
    }
    //返回对应系数
    return dps / (MAX_DPS * 4);
}


/**
  * @brief 遥控器反馈源更新
  */
void Chassis::remoteControlUpdate(Speed* structPointer)
{
    static float rcIndex;//遥控器摇杆灵敏度

    if(exchangeState.count>0 || exitPerfectState.count>0)
        rcIndex=0.05;//限制兑换时遥控器控制底盘的移动灵敏度
    else
        rcIndex=0.35;//考虑到机器人重心较高使用遥控器控制时限制底盘速度
    
    structPointer->fb = rcCtrl.rc.ch[3]*MAX_DPS/660.0f;
    structPointer->lr = rcCtrl.rc.ch[2]*MAX_DPS/660.0f;
    
    //按照遥控器的摇杆量 根据 云台方向 解算出底盘移动的方向
    //实现无论云台什么方向，云台的方向即为底盘的前进的正方向
    Speed tempSpeed={0};
    float yawPos_rad=0.0174532925*(gimbal.yaw.setPosition);
    tempSpeed.fb = structPointer->fb*cos(yawPos_rad)-structPointer->lr*sin(yawPos_rad);
    tempSpeed.lr = structPointer->lr*cos(yawPos_rad)+structPointer->fb*sin(yawPos_rad);
    
    //解算后的速度作为麦轮的设定速度
    structPointer->fb = tempSpeed.fb*rcIndex;
    structPointer->lr = tempSpeed.lr*rcIndex*1.2;
    structPointer->ro = rcCtrl.rc.ch[0]*MAX_DPS*rcIndex/660.0f;
}


/**
  * @brief 键盘反馈源更新
  */
void Chassis::keyboardUpdate(Speed* structPointer)
{
    //键盘按键系数模式选择
    if(takeGoldState.count>0 || takeSilveryState.count>0 || 
       takeFloorState.count>0 || exitPerfectState.count>0)
    {
        speedMode=1;//取矿状态下低速对位
    }
    else if(exchangeState.count>0)
    {
        speedMode=2;//兑换状态下低速平移
    }
    else if(ore[inThree].OreNum || ore[inTak].OreNum || ore[inArm].OreNum){
        speedMode = 3;
    }
    else
    {
        speedMode=0;//抬升机构降到最低时重心较低 速度可略大
    }
    
    
    //依据当前模式speedMode以及键盘是否按下ctrl/shift/none来选取相应的按键系数
    if (!rcCtrl.key.SHIFT && rcCtrl.key.CTRL)
        keyIndex = speedCoefficient[speedMode][ctrl];
    else if (!rcCtrl.key.SHIFT && !rcCtrl.key.CTRL)
        keyIndex = speedCoefficient[speedMode][none];
    else if (rcCtrl.key.SHIFT && !rcCtrl.key.CTRL)
        keyIndex = speedCoefficient[speedMode][shift];
    
    tarIndex.fb = (rcCtrl.key.W - rcCtrl.key.S); //键盘前后油门
    tarIndex.lr = (rcCtrl.key.D - rcCtrl.key.A); //键盘左右油门

//    Speed tempTarIndex={0};
//    float yawPos_rad=0.0174532925*(gimbal.yaw.setPosition);//转化为弧度制
//    tempTarIndex.fb = tarIndex.fb*cos(yawPos_rad)-tarIndex.lr*sin(yawPos_rad);
//    tempTarIndex.lr = tarIndex.lr*cos(yawPos_rad)+tarIndex.fb*sin(yawPos_rad);;
    
    //依据云台方向决定底盘前后左右的方向，
    //不使用上面的解算方式是因为浮点数与浮点数在判断相等时存在精度问题导致驱动防滑异常重置，可通过优化浮点数比较方法优化
    //TODO：(tarIndex.fb != lastIndex.fb) => abs( tarIndex.fb - lastIndex.fb) < 0.000001
    Speed tempTarIndex={0};
    if(gimbal.yaw.setPosition==gimYawForward)
    {
        tempTarIndex.fb = tarIndex.fb;
        tempTarIndex.lr = tarIndex.lr;
    }
    else if(gimbal.yaw.setPosition==gimYawBack || gimbal.yaw.setPosition == gimYawCheck)
    {
        tempTarIndex.fb = -tarIndex.fb;
        tempTarIndex.lr = -tarIndex.lr;
    }
    else if(gimbal.yaw.setPosition == gimYawRight){
        tempTarIndex.fb = -tarIndex.lr;
        tempTarIndex.lr = tarIndex.fb;
    }
    
    tarIndex.fb = tempTarIndex.fb;
    tarIndex.lr = tempTarIndex.lr;
    
    tarIndex.fb *= keyIndex[fb];
    tarIndex.lr *= keyIndex[lr];
    
    //平移油门突变时驱动防滑系统重制
    if (tarIndex.fb != lastIndex.fb)
    {
        nowIndex.fb = getNowIndex(fb);
        if(nowIndex.fb*tarIndex.fb>=0) //同号 加减速
            acc = ABS(tarIndex.fb)>=ABS(nowIndex.fb)?speedCoefficient[speedMode][3][fb]:speedCoefficient[speedMode][4][fb];
        else //异号 换向
            acc = speedCoefficient[speedMode][4][fb];
         asr[fb].setTar(nowIndex.fb, tarIndex.fb, 1.5*ABS(tarIndex.fb - nowIndex.fb)/ABS(acc));
    }
    if (tarIndex.lr != lastIndex.lr)
    {
        nowIndex.lr = getNowIndex(lr);
        if(nowIndex.lr*tarIndex.lr>=0) //同号 加减速
            acc = ABS(tarIndex.lr)>=ABS(nowIndex.lr)?speedCoefficient[speedMode][3][lr]:speedCoefficient[speedMode][4][lr];
        else //异号 换向
            acc = speedCoefficient[speedMode][4][lr];
        asr[lr].setTar(nowIndex.lr, tarIndex.lr, 1.5*ABS(tarIndex.lr - nowIndex.lr)/ABS(acc));
    }
    lastIndex=tarIndex;
    
    
    //速度设定系数更新
    setIndex.fb = asr[0].update();
    setIndex.fb = LIMIT(setIndex.fb,-1,1);
    setIndex.lr = asr[1].update();
    setIndex.lr = LIMIT(setIndex.lr,-1,1);
    //使用卡尔曼滤波对鼠标x轴原始值进行滤波，鼠标x轴原始反馈值间断掉帧
    setIndex.ro = yawKf.KalmanFilter(0.008f * rcCtrl.mouse.vx, 0.5f, 20.0f, 0) * keyIndex[ro];
    setIndex.ro = LIMIT(setIndex.ro,-1,1);
    
    
    //速度设定值更新
    structPointer->fb = setIndex.fb * MAX_DPS;
    structPointer->lr = setIndex.lr * MAX_DPS;
    structPointer->ro = setIndex.ro * MAX_DPS;
}


/**
  * @brief 取矿时底盘低速往前怼保证机器人能抵住资源岛实现定位
  */
void Chassis::takeOreUpdate(Speed* structPointer)
{
    structPointer->lr = 0;
    // if(catchState.count > 0)
    // {
    //     structPointer->fb = 150*6;
    //     structPointer->ro = 0;
    // }
    if(takeGoldState.count > 0 )
    {
        structPointer->lr = 150 * 6;
        structPointer->ro = 0;
    }
    else if(takeSilveryState.count > 0){
        structPointer->lr = -150 * 6;       
    }
    else
    {
        structPointer->fb = 0;
        structPointer->ro = 0;
    }
}
/** @brief 麦轮底盘解算实际速度，待检验
  * @param structPointer 传入目标速度，单位为m/s
  */
uint16_t wheel2WorldIndex;
void Chassis::calcRealWheelSpeed(Speed* structPointer){
    ///TODO:加入单位转换(电机速度环与现实单位进行连接,然后修改参数850)
    wheelSetSpeed[0] = (structPointer->fb - structPointer->lr - (CHASSIS_L + CHASSIS_W)*structPointer->ro)/CHASSIS_R * 850; //左前
    wheelSetSpeed[1] = -(structPointer->fb + structPointer->lr + (CHASSIS_L + CHASSIS_W)*structPointer->ro)/CHASSIS_R * 850; //右前
    wheelSetSpeed[2] = -(structPointer->fb - structPointer->lr + (CHASSIS_L + CHASSIS_W)*structPointer->ro)/CHASSIS_R * 850; //右后
    wheelSetSpeed[3] = (structPointer->fb + structPointer->lr - (CHASSIS_L + CHASSIS_W)*structPointer->ro)/CHASSIS_R * 850; //左后
}

void Chassis::visionSpeedUpdate(Speed* structPointer){

//    static double time_stamp_tmp;
//    if(vision.navigationData.time_stamp){//&& time_stamp_tmp!=vision.navigationData.time_stamp
//        structPointer->fb = (float)vision.navigationData.target_y_speed;
//        structPointer->lr = -(float)vision.navigationData.target_x_speed;
//        time_stamp_tmp = vision.navigationData.time_stamp;
//    }
//    else if(vision.yawData.time_stamp && time_stamp_tmp!=vision.yawData.time_stamp){
//        structPointer->ro += (float)vision.yawData.yaw_change;
//        time_stamp_tmp = vision.yawData.time_stamp;
//    }
}

/**
  * @brief 底盘更新：合成各来源的速度vx vy w，解算每个轮组的速度wheelSetSpeed[0]~[3]
  */
void Chassis::update()
{
    //遥控器更新
    remoteControlUpdate(&allFeedbackSourceSetSpeedList[remoteControlSetSpeed]);
    //键盘更新
    keyboardUpdate(&allFeedbackSourceSetSpeedList[keyBoardSetSpeed]);
    //取矿自动前进
    takeOreUpdate(&allFeedbackSourceSetSpeedList[autoSetSpeed]);
    //定位导航速度更新
    visionSpeedUpdate(&allFeedbackSourceSetSpeedList[visionSetSpeed]);
    // 可添加底盘速度来源，同时需要修改来源总数FEED_SOURCE_NUM以及添加SetSpeedList枚举索引
    // ...Update(&allFeedbackSourceSetSpeedList[...]);

    //各反馈源速度合成
    Speed totalSetSpeed={0};
    for(u8 i=0;i<FEED_SOURCE_NUM;i++)
    {
        totalSetSpeed.fb += allFeedbackSourceSetSpeedList[i].fb;
        totalSetSpeed.lr += allFeedbackSourceSetSpeedList[i].lr;
        totalSetSpeed.ro += allFeedbackSourceSetSpeedList[i].ro;
    }
    //双边拨杆置中底盘才可动
    if((rcCtrl.rc.sw1 != RCS::Mid) || (rcCtrl.rc.sw2 != RCS::Mid))
        memset(&totalSetSpeed, 0, sizeof(totalSetSpeed));

    //各轮速度计算
    if(vision.online.isOnline() && NAVIGATION_ENABLE_FLAG){
        calcRealWheelSpeed(&totalSetSpeed);
    }
    else{
        wheelSetSpeed[0] = (+totalSetSpeed.fb + totalSetSpeed.lr + totalSetSpeed.ro);
        wheelSetSpeed[1] = (-totalSetSpeed.fb + totalSetSpeed.lr + totalSetSpeed.ro);
        wheelSetSpeed[2] = (-totalSetSpeed.fb - totalSetSpeed.lr + totalSetSpeed.ro);
        wheelSetSpeed[3] = (+totalSetSpeed.fb - totalSetSpeed.lr + totalSetSpeed.ro);
    }

    //速度限幅
    /*方向合成*/
    float setMax = ABS(wheelSetSpeed[0]);//最大速度值
	//寻找最大速度
    for (u8 i = 0; i < 4; i++)
    {
        if (ABS(wheelSetSpeed[i]) > setMax)
            setMax = ABS(wheelSetSpeed[i]);
    }
	if (setMax > MAX_DPS)
	{
		for (u8 i = 0; i < 4; i++)
			wheelSetSpeed[i] = wheelSetSpeed[i] * MAX_DPS / setMax;
	}

    //输出
    for(u8 i=0; i<4; i++)
    {
        motor[i].planIndex = this->planIndex;
        motor[i].ctrlSpeed(wheelSetSpeed[i]);
    }

}
