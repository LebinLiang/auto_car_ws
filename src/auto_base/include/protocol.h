#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#define HEADER_SOF 0xA5
#define END1_SOF 0x0D
#define END2_SOF 0x0A

#pragma pack(push, 1)
typedef enum
{
  GAME_STATUS_FDB_ID = 0x0001,
  CHASSIS_ODOM_FDB_ID = 0x0101,
  CHASSIS_CTRL_CMD_ID = 0x0102,
  //GAME_RESULT_MSG_ID = 0x0002,
  ROBOT_HEAL_MSG_ID = 0x0003,
  DART_LAUNCH_STA_ID = 0x0004,
  RMUA_MSG_ID = 0x0005,
  AREA_EVENT_MSG_ID = 0x0101,
  SUPPLY_ACTION_MSG_ID = 0x0102,
  VISION_CMD_ID = 0x0103,
  REFEREE_WARNING_MSG_ID = 0x0104,
  DART_TIME_MSG_ID = 0x0105,
  //ROBOT_STA_ID = 0x001,
  ROBOT_HOT_STA_ID = 0x0202,
  ROBOT_LOCATR_MSG_ID = 0x0203,
  ROBOT_BUFF_MSG_ID = 0x0204,
  INJURE_MSG_ID = 0x0206,
  SHOOT_MSG_ID = 0x0207,
  BULLET_REMAINING_ID =0x0208,
  RFID_STA_ID = 0x0209,
  DART_CMD_ID = 0x020A,
 ROBOT_COM_MSG_ID = 0x301,
  CUS_CTRLER_CMD_ID = 0x0302,
  CUS_MAP_REC_ID = 0x0303,
  
} referee_data_cmd_id_type;

typedef struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

// chassis control message
typedef struct
{
  float vx;
  float vy;
  float vw;
  uint8_t super_cup;
	
}  chassis_ctrl_info_t;

// chassis odom feedback message
typedef struct
{
  
  float x_fdb;
  float y_fdb;
  float vx_fdb;
  float vy_fdb;
  float vw_fdb;
  float gimbal_yaw_fdb;
  float gimbal_pitch_fdb;
  float gimbal_yaw_imu;
  float gimbal_yaw_rate_imu;
  float gimbal_pitch_imu;
  float gimbal_pitch_rate_imu;
	float super_cup_state;

  // Printf() {

  //   std::cout << std::endl
  //             << 
  // }
}  chassis_odom_info_t;

// vision control message
typedef struct
{
	float gimbal_yaw_cmd;
	float gimbal_pitch_cmd;
	float shoot_cmd;
	float friction_cmd;
  
  float aiming_flag;
  float pit;
  float yaw;
  float dis;
  float tof;
  float connect_status;        //连接状态：0（状态异常） 1（正常连接） 2（正常且执行指令中）
  float process_status;       //流程状态： 0（空闲）  1（进行中） 2（导航中） 3（完成）
  float cmd_mode;             //执行指令： 0（空闲）  1 （退回哨兵巡逻区）2（前往防守前哨站）3（前往我方占领区）4（前往敌方占领区）
                                 //5(进攻敌方前哨站)   6（进攻敌方基地） 
  float spinning_mode;        //小陀螺状态： 0（小陀螺关闭）  1（小陀螺关闭）                     
  float navigation_staus;     //导航状态：   0(空闲) 1（已到达）2（运行中） 3（规划失败）
  float location_staus;       //定位状态:    0(正常) 1（漂移） 2（重定位中）
  float defend_mode;          //防御模式：    0（无敌状态） 1（无敌状态解除）  //我方前哨站击毁前 自动哨兵处于无敌状态
  float  auto_aim;             //自动瞄准状态： 0（空闲） 1（索敌中）2（识别到敌方机器人）3（射击中）4（自瞄异常）
  float    robot_x;              //自动哨兵位置X
  float    robot_y;              //自动哨兵位置Y
  float    robot_vx;             //自动哨兵速度vx
  float    robot_vy;             //自动哨兵速度vy
  float    robot_vw;             //自动哨兵速度vw
	float    robot_gyro_yaw;       //哨兵云台角度yaw
  
}  vision_ctrl_info_t;

// game status feedback message
typedef struct
{
	uint8_t area;
	float shoot_number;
	float health_point;
	uint8_t game_state;
	uint8_t heat_rest;
	
}  game_status_info_t;

//��Ϣ����
typedef struct
{
	float x;
  float y;
  float yaw;
  float vx;
  float vy;
  float vw;
	float gyro_z;
	float gyro_yaw;
	
} message_info_t;
typedef struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;
//
typedef  struct
{
  float game_type;
  float game_progress;
  float shooter_cooling_rate;
  float shooter_speed_limit;
  float shooter_cooling_limit;
  float robot_id;
  float  remain_HP;
  float max_HP;
  float armor_id;
  float hurt_type;
  float bullet_freq;
  float bullet_speed;
  float bullet_remaining_num;
  float key_board;
  float other_robot_id;
  float  tgx;
  float  tgy;
  float  tgz;
  float  blue_x;
  float  blue_y;
  float  blue_confiden;
  float  red_x;
  float  red_y;
  float  red_confiden;
} ext_game_robot_status_t;
//
typedef struct
{
  uint8_t armor_id : 4;
  uint8_t hurt_type: 4;
}ext_robot_hurt_t;

typedef struct 
{
  uint8_t bullet_type;
  uint8_t shooter_id;
  uint8_t bullet_freq;
  float   bullet_speed;
}ext_shoot_data_t;

typedef  struct
{
  uint16_t bullet_remaining_num_17mm;
  uint16_t bullet_remaining_num_42mm;
  uint16_t coin_remaining_num;
}ext_bullet_remaining_t;

typedef  struct 
{
  float blue_x;
  float blue_y;
  float blue_confiden;
  float red_x;
  float red_y;
  float red_confiden;
}enemy_locate_t;

typedef  struct 
{
  uint16_t game_type;
  uint16_t game_progress;
  uint16_t shooter_cooling_rate;
  uint16_t shooter_speed_limit;
  uint16_t shooter_cooling_limit;
  uint16_t robot_id;
  uint16_t remain_HP;
  uint16_t max_HP;
  uint16_t armor_id;
  uint16_t hurt_type;
  uint16_t bullet_freq;
  uint16_t bullet_speed;
  uint16_t bullet_remaining_num;
  uint16_t key_board;
  uint16_t other_robot_id;
  uint16_t aaa;
  uint16_t bbb;
  uint16_t ccc;
  float  tgx;
  float  tgy;
  float  tgz;
  float  blue_x;
  float  blue_y;
  float  blue_confiden;
  float  red_x;
  float  red_y;
  float  red_confiden;

  
}user_data_t;

#pragma pack(pop)
#endif //ROBOMASTER_PROTOCOL_H
