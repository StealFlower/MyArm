#ifndef __JUDGEMENT_H
#define __JUDGEMENT_H
#include "board.h"
#include "online.h"


//机器人ID
enum RobotIdDef
{
	ID_HERO = 1,          //英雄
	ID_ENGINEER = 2,      //工程
	ID_INFANTRY1 = 3,     //步兵
	ID_INFANTRY2 = 4,     //步兵
	ID_INFANTRY3 = 5,     //步兵
	ID_AIR = 6,           //空中
	ID_GUARD = 7          //哨兵
};

#define ID1_17mm_SHOOT_NUM 0
#define ID2_17mm_SHOOT_NUM 1
#define ID1_42mm_SHOOT_NUM 2

enum JudgementStep
{
	STEP_HEADER = 0,		//帧头数据获取
	STEP_HEADER_CRC8 = 1,	//CRC8校验
	STEP_CMDID_GET = 2,		//获取命令码
	STEP_DATA_TRANSFER = 3, //数据转移（从JudgeDataBuffer转移至cmd_id对应的联合体中）
	STEP_DATA_CRC16 = 4,	//CRC16校验（暂未加校验）
};

#define JUDGE_BUFFER_LENGTH 255

#define PURE_DATA_LEN (113)			   //不算ID与帧头的长度，用于定义data[Pure_data_len]
#define DATA_LENGTH (6 + PURE_DATA_LEN) //不算帧头的长度，用于赋值frame_header.data_length
#define ALL_LEN (PURE_DATA_LEN + 15)   //总长，用于CRC16校验


#pragma pack(1)
//30字节，总共240Bit
struct CustomInfo
{
	int16_t leftX : 10;
	int16_t leftY : 10;
	int16_t leftZ : 10;
	int16_t rightX : 10;
	int16_t rightY : 10;
	int16_t rightZ : 10; // 60
	uint16_t leftKeyCnt : 10;//奇数按下偶数松开
	uint16_t rightKeyCnt : 10; // 80
    uint8_t ref[20];
};

#pragma pack(0)

#pragma pack(1)
struct CommentInfo
{

};
#pragma pack(0)


#pragma pack(1) //用于结构体的内存对齐， 方便使用联合体调用

class Judgement
{
	struct FrameHeader
	{
		unsigned char sof;			//帧头
		unsigned short dataLength; //数据长度（0x301包含内容ID、发送者ID以及接收者ID）
		unsigned char seq;			//包序号，可填0
		unsigned char crc8;			//CRC8-校验
		unsigned short cmdid;		//命令ID
	};
	
public:
    Judgement();
	/******************以下函数调用频率最高为10Hz***************/
	// 0x301联合体数据发送，需将dataCmdId、senderId、receiverId填写完，一般不调用此函数
	void customSend(u8 count);
	//填写与机器人交互的数据后调用此函数可直接发送
	void robotsCommunication(uint16_t dataCmdId, RobotIdDef robotIdDef, u8 dataLenth);
	//发送sendGraphics个图形
	void graphicDraw(u8 sendGraphics);
	//发送字符传
	void characterDraw();
    //评论区发送信息，支持中文
    void commentSend();
	//删除图层
	void graphicDel();
	/***********************************************************/
	//发送地图命令
	void mapCommandSend();
	//发送自定义控制器信息
	void customCtrlSend();
	//初始化
    void init();
    //队列解析
	void ringQueue();
    //在线检测
    Online online;

	uint16_t IdToMate(RobotIdDef robotIdDef);
	uint16_t IdToMate(uint16_t robotId);
	uint16_t ClientId();
	uint16_t IdToEnemy(RobotIdDef robotIdDef);
	uint16_t IdToEnemy(uint16_t robotId);

    //比赛状态数据 0x0001
    struct GameStatus
	{
		uint8_t gameType : 4;
		uint8_t gameProgress : 4;
		uint16_t stageRemainTime;
		uint64_t syncTimeStamp;
	} gameStatus;
	
	//比赛结果数据 
	struct GameResult
	{
		uint8_t winner;
	} gameResult;
	
    // 机器人血量数据 
    union GameRobotHP
	{
		u16 teamHp[2][8];
		u16 allHp[16];
		struct
		{
			uint16_t red_1RobotHp;
			uint16_t red_2RobotHp;
			uint16_t red_3RobotHp;
			uint16_t red_4RobotHp;
			uint16_t red_5RobotHp;
			uint16_t red_7RobotHp;
			uint16_t redOutpostHp;//红方前哨站
			uint16_t redBaseHp;//红方基地
			uint16_t blue_1RobotHp;
			uint16_t blue_2RobotHp;
			uint16_t blue_3RobotHp;
			uint16_t blue_4RobotHp;
			uint16_t blue_5RobotHp;
			uint16_t blue_7RobotHp;
			uint16_t blueOutpostHp;//蓝方前哨站
			uint16_t blueBaseHp;//蓝方基地
		} singleHp;
	} gameRobotHP;
	
	//飞镖 
	struct DartStatus
	{
		uint8_t dartBelong;
		uint16_t stageRemainingTime;
	} dartStatus;
	
	//人工智能挑战赛加成与惩罚区状态 
	struct ICRA_BuffDebuffZoneStatus_t
	{
		uint8_t f1ZoneStatus : 1;//红方回血区
		uint8_t f1ZoneBuffDebuffStatus : 3;
		uint8_t f2ZoneStatus : 1;//红方弹药补给区
		uint8_t f2ZoneBuffDebuffStatus : 3;
		uint8_t f3ZoneStatus : 1;//蓝方回血区
		uint8_t f3ZoneBuffDebuffStatus : 3;
		uint8_t f4ZoneStatus : 1;//蓝方弹药补给区
		uint8_t f4ZoneBuffDebuffStatus : 3;
		uint8_t f5ZoneStatus : 1;//禁止射击区
		uint8_t f5ZoneBuffDebuffStatus : 3;
		uint8_t f6ZoneStatus : 1;//禁止移动区
		uint8_t f6ZoneBuffDebuffStatus : 3;
		//剩余弹量
		uint16_t red1BulletLeft;
		uint16_t red2BulletLeft;
		uint16_t blue1BulletLeft;
		uint16_t blue2BulletLeft;
	} ICRA_BuffDebuffZoneStatus;
   
	//场地事件数据 0x0101
	struct EventData
	{
		uint32_t eventType;
	} eventData;
	
	//补给站动作标识 
	struct SupplyProjectileAction
	{
		uint8_t supplyProjectileId;
		uint8_t supplyRobotId;
		uint8_t supplyProjectileStep;
		uint8_t supplyProjectileNum;
	} supplyProjectileAction;
	
	//裁判警告信息
	struct RefereeWarning
	{
		uint8_t level;
		uint8_t foulRobotId;
	} refereeWarning;
	
	//飞镖
	struct DartRemainingTime
	{
		uint8_t time;
	} dartRemainingTime;
	
	//比赛机器人状态 0x0201
	struct GameRobotStatus
	{
		uint8_t robotId;
		uint8_t robotLevel;
		uint16_t remainHp;
		uint16_t maxHp;
		uint16_t shooterId1_17mmCoolingRate;//枪口每秒冷却值
		uint16_t shooterId1_17mmCoolingLimit;//枪口热量上限
		uint16_t shooterId1_17mmSpeedLimit;//枪口上限速度
		uint16_t shooterId2_17mmCoolingRate;
		uint16_t shooterId2_17mmCoolingLimit;
		uint16_t shooterId2_17mmSpeedLimit;
		uint16_t shooterId1_42mmCoolingRate;
		uint16_t shooterId1_42mmCoolingLimit;
		uint16_t shooterId1_42mmSpeedLimit;
		uint16_t chassisPowerLimit;
		uint8_t mainsPowerGimbalOutput : 1;
		uint8_t mainsPowerChassisOutput : 1;
		uint8_t mainsPowerShooterOutput : 1;
	} gameRobotStatus;
	
	//实时功率热量数据
	struct PowerHeatData
	{
		uint16_t chassisVolt;
		uint16_t chassisCurrent;
		float chassisPower;
		uint16_t chassisPowerBuffer;
		uint16_t shooterId1_17mmCoolingHeat;
		uint16_t shooterId2_17mmCoolingHeat;
		uint16_t shooterId1_42mmCoolingHeat;
	} powerHeatData;
	
	//机器人位置
	struct GameRobotPos
	{
		float x;
		float y;
		float z;
		float yaw;
	} gameRobotPos;
	
	//机器人增益
	struct Buff
	{
		uint8_t powerRuneBuff;
	} buff;
	
	//空中机器人能量状态
	struct AerialRobotEnergy
	{
		uint8_t attackTime;
	} aerialRobotEnergy;
	
	//伤害状态
	struct RobotHurt
	{
		uint8_t armorId : 4;
		uint8_t hurtType : 4;
	} robotHurt;
	
	//实时射击信息
	struct ShootData
	{
		uint8_t bulletType;
		uint8_t shooterId;
		uint8_t bulletFreq;
		float bulletSpeed;
	} shootData;
	
	//子弹剩余发射数
	struct BulletRemaining
	{
		uint16_t bulletRemainingNum_17mm;
		uint16_t bulletRemainingNum_42mm;
		uint16_t coinRemainingNum;
	} bulletRemaining;
	
	//机器人 RFID 状态
	struct RfidStatus
	{
		uint32_t rfidStatus;
	} rfidStatus;
	
	//飞镖
	struct DartClientCmd
	{
		uint8_t dartLaunchOpeningStatus;
		uint8_t dartAttackTarget;
		uint16_t targetChangeTime;
		uint8_t firstDartSpeed;
		uint8_t secondDartSpeed;
		uint8_t thirdDartSpeed;
		uint8_t fourthDartSpeed;
		uint16_t lastDartLaunchTime;
		uint16_t operateLaunchCmdTime;
	} dartClientCmd;
	
	
	
    
	//交互数据接收信息 0x0301
	struct StudentInteractiveData
	{
		uint16_t dataCmdId;
		uint16_t senderId;
		uint16_t receiverId;
		uint8_t data[112];
	} studentRecviveData;
	
	//客户端删除图形 机器人间通信
	struct ClientCustomGraphicDelete
	{
		uint8_t operateType;
		uint8_t layer;
	};
	
	//图形数据
	struct GraphicDataStruct
	{
		uint8_t graphicName[3];         //图形名 在删除,修改等操作中,作为客户端的索引。          
		uint32_t operateType : 3;       //图形操作 0空操作1增加2修改3删除
		uint32_t graphicType : 3;       //图形类型 0直线1矩形2整圆3椭圆4圆弧5浮点数6整型数7字符
		uint32_t layer : 4;             //图层数 0~9 
		uint32_t color : 4;             //颜色 0红蓝主色1黄色2绿色3橙色4紫红色5粉色6青色7黑色8白色
		
        uint32_t startAngle : 9;        //起始角度 [0,360]
		uint32_t endAngle : 9;          //终止角度 [0,360]
		uint32_t width : 10;            //线宽
		uint32_t startX : 11;           //起点x坐标
		uint32_t startY : 11;           //起点y坐标
		uint32_t radius : 10;           //字体大小或半径
		uint32_t endX : 11;             //终点x坐标 
		uint32_t endY : 11;             //终点y坐标    
	};
    
    //数字数据
	struct NumberDataStruct
	{
		uint8_t graphicName[3];         //图形名 在删除,修改等操作中,作为客户端的索引。          
		uint32_t operateType : 3;       //图形操作 0空操作1增加2修改3删除
		uint32_t graphicType : 3;       //图形类型 0直线1矩形2整圆3椭圆4圆弧5浮点数6整型数7字符
		uint32_t layer : 4;             //图层数 0~9 
		uint32_t color : 4;             //颜色 0红蓝主色1黄色2绿色3橙色4紫红色5粉色6青色7黑色8白色
		
        uint32_t startAngle : 9;        //起始角度 [0,360]
		uint32_t endAngle : 9;          //终止角度 [0,360]
		uint32_t width : 10;            //线宽
		uint32_t startX : 11;           //起点x坐标
		uint32_t startY : 11;           //起点y坐标
		int32_t num;    
	};
	
	//客户端绘制字符
	struct ClientCustomCharacter
	{
		GraphicDataStruct graphic_data_struct;
		uint8_t data[30];
	};
	
	//客户端下发信息
	struct MapCommand
	{
		float targetPositionX;
		float targetPositionY;
		float targetPositionZ;
		uint8_t cmdKeyboard;
		uint16_t targetRobotId;
	};
	//自定义控制器信息
	union CustomCtrl
	{
		CustomInfo info;
		uint8_t data[30];
	}customCtrl;
	struct CustomCtrData
	{
		struct FrameHeader frameHeader;
		CustomCtrl customCtrl;
		uint16_t CRC16;
	}customCtrlData;
	
	//交互数据信息
	struct SendUnionData
	{
		struct FrameHeader frameHeader;
		uint16_t dataCmdId;
		uint16_t senderId;
		uint16_t receiverId;

		union
		{
			uint8_t studentSendData[113];
			struct ClientCustomGraphicDelete clientCustomGraphicDelete;
			struct GraphicDataStruct graphicDataStruct[7];
			struct ClientCustomCharacter clientCustomCharacter;
		};
		uint16_t CRC16;
	} sendUnionData;
	
    //向评论区发送机器人信息
    union Comment
    {
        CommentInfo info;
        uint8_t data[30];
    }comment;
    struct CommentData
    {
        struct FrameHeader frameHeader;
        uint16_t senderId;
        uint16_t receiverId;
        Comment comment;
        uint16_t CRC16;
    }commentData;
	
	struct MapCommandData
	{
		struct FrameHeader frameHeader;
		struct MapCommand mapCommand;
		uint16_t CRC16;
	} mapCommandData;

	
	struct WrongStatusCnt
	{
		int CRC8_Wrong_cnt;	 //CRC8校验错误次数累计
		int CRC16_Wrong_cnt; //CRC16校验错误次数累计
		int CYCLE_Wrong_cnt; //包圈错误次数累计
		int SEQ_Wrong_cnt;	 //包序号错误次数累计
		/*******若以上错误同时出现，则需提高裁判系统解析频率与优先级（防止丢包）******/
	} wrongStatusCnt;
	
#pragma pack()
	
	uint16_t shootNum[3] = {0};
	u8 jgmtOffline = 0;		 //裁判系统离线
	int hurtCount = 0;		 //裁判系统受到伤害才更新装甲板,此次与上一次hurtCount不相同即为受攻击
	
	inline void addFullCount() { judgementFullCount++; }
	
private:
	enum JudgeDataId
	{
		STATUS_DATA = 0x0001, //比赛状态数据 
		RESULT_DATA = 0x0002, //比赛结果数据
		ROBOT_HP_DATA = 0x0003, //比赛结果数据
		DART_STATUS = 0x0004,
		ICRA_BUFF_DEBUFF_ZONE_STATUS = 0x0005, //人工智能挑战赛加成与惩罚状态

		EVENT_DATA = 0x0101,			   //场地事件数据
		SUPPLY_PROJECTILE_ACTION = 0x0102, //场地补给站动作标识数据
		ROBOT_WARNING_DATA = 0x104, //裁判警告数据
		DART_REMAINING_TIME = 0x105, //飞镖发射口倒计时

		GAME_ROBOT_STATUS = 0x0201,//机器人状态数据
		POWER_HEAT_DATA = 0x0202, //实时功率热量数据
		GAME_ROBOT_POS = 0x0203, //机器人位置数据
		BUFF = 0x0204, //机器人增益数据
		AERIAL_ROBOT_ENERGY = 0x0205,//空中机器人能量状态数据 
		ROBOT_HURT = 0x0206, //伤害状态数据
		SHOOT_DATA = 0x0207, //实时射击数据
		BULLET_REMAINING = 0x208, //子弹剩余发送数，空中机器人以及哨兵机器人发送 
		RFID_STATUS = 0x209, //机器人 RFID 状态
		DART_CLIENT_CMD = 0x20A, //飞镖机器人客户端指令书

		STUDENT_INTERACTIVE_HEADER_DATA = 0x301, //机器人间交互数据
		CUSTOMCTRLDATA = 0x302
	};

    JudgementStep judgementStep;

	uint8_t judgeDataBuffer[JUDGE_BUFFER_LENGTH];
	uint16_t judgementFullCount;
	
	uint8_t fullDataBuffer[128];
	void usart3Config(void);
	unsigned char getLength(FrameHeader *frameHeader);
	void usart3SendBytes(void *ptr, u8 len);
	void getJudgeData();

};
extern Judgement judgement;


extern "C" void DMA1_Channel4_IRQHandler(void);
//#endif
#endif