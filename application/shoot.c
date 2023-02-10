/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "referee_usart_task.h"

#define shoot_fric1_on(pwm) fric1_on((pwm)) //摩擦轮1pwm宏定义
#define shoot_fric2_on(pwm) fric2_on((pwm)) //摩擦轮2pwm宏定义
#define shoot_fric_off()    fric_off()      //关闭两个摩擦轮

#define shoot_laser_on()    laser_on()      //激光开启宏定义
#define shoot_laser_off()   laser_off()     //激光关闭宏定义
//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)


extern miniPC_info_t miniPC_info;

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          堵转倒转处理
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back_17mm(void);

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control_17mm(void);



shoot_control_t shoot_control;          //射击数据


int16_t temp_rpm_left;
int16_t temp_rpm_right;

fp32 temp_speed_setALL = 11.5;//目前 ICRA Only

/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{

    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    //遥控器指针
    shoot_control.shoot_rc = get_remote_control_point();
    //电机指针
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
    //初始化PID
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    //更新数据
    shoot_feedback_update();
    ramp_init(&shoot_control.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    ramp_init(&shoot_control.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    shoot_control.fric_pwm1 = FRIC_OFF;
    shoot_control.fric_pwm2 = FRIC_OFF;
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
		
		/*12-28-2021 SZL add for 
		infantry pid shooter friction wheel LEFT and RIGHT
		Everything above keep the same as the old PWM shooter
		*/
		//初始化基本射击参数
		shoot_control.currentLeft_speed_set = 0;
		shoot_control.currentRight_speed_set = 0;
		shoot_control.currentLIM_shoot_speed_17mm = 0;
		
		
		//LEFT friction PID const init
		static const fp32 Left_friction_speed_pid[3] = {M3508_LEFT_FRICTION_PID_KP, M3508_LEFT_FRICTION_PID_KI, M3508_LEFT_FRICTION_PID_KD};
		//RIGHT friction PID const init
		static const fp32 Right_friction_speed_pid[3] = {M3508_RIGHT_FRICTION_PID_KP, M3508_RIGHT_FRICTION_PID_KI, M3508_RIGHT_FRICTION_PID_KD};

		//电机指针 M3508屁股 左右摩擦轮
		shoot_control.left_friction_motor_measure = get_left_friction_motor_measure_point();
		shoot_control.right_friction_motor_measure = get_right_friction_motor_measure_point();
		
		//初始化PID
		PID_init(&shoot_control.left_fric_motor_pid, PID_POSITION, Left_friction_speed_pid, M3508_LEFT_FRICTION_PID_MAX_OUT, M3508_LEFT_FRICTION_PID_MAX_IOUT);
		PID_init(&shoot_control.right_fric_motor_pid, PID_POSITION, Right_friction_speed_pid, M3508_RIGHT_FRICTION_PID_MAX_OUT, M3508_RIGHT_FRICTION_PID_MAX_IOUT);
	
}

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */

//===============================================
//uint8_t robot_Level = 0;

int16_t shoot_control_loop(void)
{

    shoot_set_mode();        //设置状态机
    shoot_feedback_update(); //更新数据
	
//		//开始判断速度上限
//	  robot_Level = get_robot_level();

//	 	 if(robot_Level == 0){
//	  shoot_control.fric1_ramp.max_value = FRIC_LV1;
//    shoot_control.fric2_ramp.max_value = FRIC_LV1;
//	 }else if(robot_Level == 1){
//	  shoot_control.fric1_ramp.max_value = FRIC_LV1;
//    shoot_control.fric2_ramp.max_value = FRIC_LV1;
//	 }else if(robot_Level == 2){
//	 	shoot_control.fric1_ramp.max_value = FRIC_LV2;
//    shoot_control.fric2_ramp.max_value = FRIC_LV2;
//	 }else if(robot_Level == 3){
//	 	shoot_control.fric1_ramp.max_value = FRIC_LV3;
//    shoot_control.fric2_ramp.max_value = FRIC_LV3;
//	 }else{
//	 	shoot_control.fric1_ramp.max_value = FRIC_LV1;
//    shoot_control.fric2_ramp.max_value = FRIC_LV1;
//	 }
//		//以上为老版本的------------------------------------	 

//------------------修改等级判断 Texas A&M 比赛使用
	 if(toe_is_error(REFEREE_TOE))
   {
      shoot_control.referee_current_shooter_17mm_speed_limit = INITIAL_PROJECTILE_SPEED_LIMIT_17mm; 
   }
	 else
	 {
			shoot_control.referee_current_shooter_17mm_speed_limit = get_shooter_id1_17mm_speed_limit();
	 }
	 
	 /*记得添加 数据超出最大合理数值时的操作*/
	 if(shoot_control.referee_current_shooter_17mm_speed_limit > 18)
	 {
		 shoot_control.referee_current_shooter_17mm_speed_limit = 18;
	 }
	 
	 //17mm 的两档
	 //shoot_control.referee_current_shooter_17mm_speed_limit = 18;//强制使其=18 用于调试-----------------------------------------------------------------------------------------------
	 if(shoot_control.referee_current_shooter_17mm_speed_limit == 15)
	 {
		 shoot_control.currentLIM_shoot_speed_17mm = (fp32)(15 - 3.0);//待定----------------------------
		 shoot_control.predict_shoot_speed = shoot_control.currentLIM_shoot_speed_17mm + 2;//待定
		 /*1) 发给ZYZ那 15.5 测出来14.5
		   2) 发给ZYZ那 14.0 测出来 14.0
		 */
	 }
	 else if(shoot_control.referee_current_shooter_17mm_speed_limit == 18)
	 {//6-15之前的自瞄一直是按这个测试的
		 // 18- 4.5 为 RMUL 实际 16.7-17.1 - .3 m/s 单速标定 SZL
		 shoot_control.currentLIM_shoot_speed_17mm = (fp32)(18 - 4.5);
		 shoot_control.predict_shoot_speed = shoot_control.currentLIM_shoot_speed_17mm + 3;
		 /*
		 1) 发给ZYZ那 16.5 测出来 16.5
		 */
	 }
	 else
	 {//默认射速15
		 shoot_control.currentLIM_shoot_speed_17mm = (fp32)(15 - 3.0);//待定-----------------------------
		 shoot_control.predict_shoot_speed = shoot_control.currentLIM_shoot_speed_17mm + 2;//待定
	 }
	 
	 
    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0;
    }
    else if(shoot_control.shoot_mode ==SHOOT_READY_BULLET)
    {
        shoot_control.trigger_speed_set = 0.0f;
        shoot_control.speed_set = 0.0f;
        //这个if 并不是 基本没啥用
        shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
				//shoot_control.trigger_speed_set = 0.0f;//------------
        //设置拨弹轮的速度
         shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_BULLET)
    {
        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;//-----------------------------------------
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control_17mm();
    }
    else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        //设置拨弹轮的拨动速度,并开启堵转反转处理
        shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED;
        trigger_motor_turn_back_17mm();
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE)
    {
        shoot_control.speed_set = 0.0f;
    }

    if(shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_laser_off();
        shoot_control.given_current = 0;
        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
//        ramp_calc(&shoot_control.fric1_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
//        ramp_calc(&shoot_control.fric2_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
			
				shoot_control.fric_pwm1 = FRIC_OFF;
				shoot_control.fric_pwm2 = FRIC_OFF;
				//关闭不需要斜坡关闭
			
			
			//SZL添加, 也可以使用斜波开启 低通滤波
			shoot_control.currentLeft_speed_set = M3508_FRIC_STOP;
			shoot_control.currentRight_speed_set = M3508_FRIC_STOP;
    }
    else
    {
        shoot_laser_on(); //激光开启
			
				
				//6-17未来可能增加串级PID----
				
        //计算拨弹轮电机PID
        PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        
#if TRIG_MOTOR_TURN
				shoot_control.given_current = -(int16_t)(shoot_control.trigger_motor_pid.out);
#else
				shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
#endif
        if(shoot_control.shoot_mode < SHOOT_READY_BULLET)
        {
            shoot_control.given_current = 0;
        }
        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
        ramp_calc(&shoot_control.fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        ramp_calc(&shoot_control.fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
				
				//SZL添加, 也可以使用斜波开启 低通滤波
				shoot_control.currentLeft_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
				shoot_control.currentRight_speed_set = shoot_control.currentLIM_shoot_speed_17mm;

    }

    shoot_control.fric_pwm1 = (uint16_t)(shoot_control.fric1_ramp.out);// + 19);
    shoot_control.fric_pwm2 = (uint16_t)(shoot_control.fric2_ramp.out);
		
		
		
    shoot_fric1_on(shoot_control.fric_pwm1);
    shoot_fric2_on(shoot_control.fric_pwm2);
		
		//vTaskDelay(5);
		
		//M3508_fric_wheel_spin_control(-tempLeft_speed_set, tempRight_speed_set);
		M3508_fric_wheel_spin_control(-shoot_control.currentLeft_speed_set, shoot_control.currentRight_speed_set);
		
    return shoot_control.given_current;
}




/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
/*
关于摩擦轮状态机的切换流程: 上拨一次 SHOOT_READY_FRIC; 再拨一次 SHOOT_STOP;
斜坡启动 换启动 之前几秒钟是缓启动 缓输出; 当斜坡到MAX时 即完成预热

*/
static void shoot_set_mode(void)
{
    static int8_t last_s = RC_SW_UP;

    //上拨判断， 一次开启，再次关闭
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;//上拨一次开启摩擦轮
			  shoot_control.user_fire_ctrl = user_SHOOT_SEMI;//开启摩擦轮 默认auto
			  shoot_control.key_Q_cnt = 2;
    }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;//上拨一次再关闭摩擦轮
			  shoot_control.key_Q_cnt = 0;
    }
				
    //处于中档， 可以使用键盘开启摩擦轮
    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC; 
				shoot_control.user_fire_ctrl = user_SHOOT_AUTO;//开启摩擦轮 默认auto
    }
    //处于中档， 可以使用键盘关闭摩擦轮
    else if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_STOP;
			  shoot_control.key_Q_cnt = 0;
    }

		//处于中档时的 按键Q 按下检测 即 用户火控状态 模式判断
		if(switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && (shoot_control.shoot_mode > SHOOT_STOP))
		{
				//shoot_control.key_Q_cnt++;
				if(shoot_control.last_key_Q_sts == 0)
				{
					shoot_control.key_Q_cnt++;
					//shoot_control.shoot_mode = SHOOT_READY;
					shoot_control.last_key_Q_sts = 1;
				}
				else
				{
					shoot_control.last_key_Q_sts = 1;
				}
		}
		else
		{
			 shoot_control.last_key_Q_sts = 0;
		}
		
		if(shoot_control.key_Q_cnt > 2)
		{
			shoot_control.key_Q_cnt = 1;//实现 周期性
		}
		
		if(shoot_control.key_Q_cnt == 1)
		{
			shoot_control.user_fire_ctrl = user_SHOOT_AUTO;
		}
		else if(shoot_control.key_Q_cnt == 2)
		{
			shoot_control.user_fire_ctrl = user_SHOOT_SEMI;
		}
		else if(shoot_control.key_Q_cnt == 0)
		{
			shoot_control.user_fire_ctrl = user_SHOOT_OFF;
		}
		//---------Q按键计数以及相关检测结束---------
		
    if(shoot_control.shoot_mode == SHOOT_READY_FRIC && shoot_control.fric1_ramp.out == shoot_control.fric1_ramp.max_value && shoot_control.fric2_ramp.out == shoot_control.fric2_ramp.max_value)
    {
        shoot_control.shoot_mode = SHOOT_READY_BULLET; //当摩擦轮完成预热 //A
    }
    else if(shoot_control.shoot_mode == SHOOT_READY_BULLET) //&& shoot_control.key == SWITCH_TRIGGER_ON)
    {
			shoot_control.shoot_mode = SHOOT_READY;  //shoot_control.key被默认初始化为0 所以:第一次会进入A 第二次会进入这儿 使得shoot_mode = SHOOT_READY
    }
    else if(0) //shoot_control.shoot_mode == SHOOT_READY && shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode = SHOOT_READY_BULLET;//从不会进入这个else if
    }
    else if(shoot_control.shoot_mode == SHOOT_READY)
    {
			if(shoot_control.trigger_motor_17mm_is_online)//发射机构断电时, shoot_mode状态机不会被置为发射相关状态
			{
        //下拨一次或者鼠标按下一次，进入射击状态
        if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || (shoot_control.press_l && shoot_control.last_press_l == 0))
        {
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
			}
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE)
    {
        shoot_control.key_time++;
				//微动开关 抖动时间到了之后, 再弄成SHOOT_READY_BULLET
				//现在是 缓冲时间
        if(shoot_control.key_time > SHOOT_DONE_KEY_OFF_TIME)
        {
            shoot_control.key_time = 0;
            shoot_control.shoot_mode = SHOOT_READY_BULLET;
        }
    }
/*
    if(shoot_control.shoot_mode > SHOOT_READY_FRIC){ //自动开火指令处理
		   if(shootCommand == 0xff){
			 shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
			 }else if(shootCommand == 0x00){
			 shoot_control.shoot_mode = SHOOT_READY_BULLET;
			 }
		}
	*/
		
		/*更改自瞄开启逻辑  X按键计数*/
		if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_X)
		{
			if(shoot_control.last_key_X_sts == 0)
			{
				shoot_control.key_X_cnt++;
				shoot_control.last_key_X_sts = 1;
			}
			else
			{
				shoot_control.last_key_X_sts = 1;
			}
		}
		else
		{
			shoot_control.last_key_X_sts = 0;
		}
		
		if(shoot_control.key_X_cnt > 2)
		{
			shoot_control.key_X_cnt = 1;//实现 周期性
		}
		//press X to turn on auto aim, 1=aid 2=lock 
		//或 即按键只能开启aim
		if(shoot_control.key_X_cnt == 0)
		{
			miniPC_info.autoAimFlag = 0;
		}
		else if(shoot_control.key_X_cnt == 1) 
		{
			miniPC_info.autoAimFlag = 1;
		}
		else if(shoot_control.key_X_cnt == 2)
		{
//			miniPC_info.autoAimFlag = 2;
			miniPC_info.autoAimFlag = 1;
		}
		
		if(shoot_control.press_r_time == PRESS_LONG_TIME_R || shoot_control.press_key_V_time == PRESS_LONG_TIME_V)
		{
			miniPC_info.autoAimFlag = 2;
			//shoot_control.key_X_cnt = 2;
		}
//		else
//		{
//			miniPC_info.autoAimFlag = 1;
//			shoot_control.key_X_cnt = 1;
//		}
		
		if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_C) // press C to turn off auto aim
		{
			miniPC_info.autoAimFlag = 0;
			shoot_control.key_X_cnt = 0;
		}
		//X按键计数以及相关检测结束
		
		//10-某一天-2022修改
		//连续发弹判断; 发射机构断电时, shoot_mode状态机不会被置为发射相关状态
    if(shoot_control.shoot_mode > SHOOT_READY_FRIC && shoot_control.trigger_motor_17mm_is_online)
    {
        //鼠标长按一直进入射击状态 保持连发
				//(shoot_control.user_fire_ctrl==user_SHOOT_AUTO && shoot_control.press_l)
			
				if(shoot_control.user_fire_ctrl==user_SHOOT_SEMI)
				{
					if (((miniPC_info.shootCommand == 0xff) && (miniPC_info.autoAimFlag > 0))|| (shoot_control.press_l_time == PRESS_LONG_TIME_L ) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
					{
							shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode =SHOOT_READY_BULLET;
					}
				}
				else if(shoot_control.user_fire_ctrl==user_SHOOT_AUTO)
				{
					if (((miniPC_info.shootCommand == 0xff) && (miniPC_info.autoAimFlag > 0)) || (shoot_control.press_l ))
					{
							shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode =SHOOT_READY_BULLET;
					}
				}
				else
				{
					if (((miniPC_info.shootCommand == 0xff) && (miniPC_info.autoAimFlag > 0)) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
					{
							shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode =SHOOT_READY_BULLET;
					}
				}
    }

    get_shooter_id1_17mm_heat_limit_and_heat(&shoot_control.heat_limit, &shoot_control.heat);
    if(!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
    {
        if(shoot_control.shoot_mode == SHOOT_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode =SHOOT_READY_BULLET;
        }
    }
		//调试: 难道referee uart掉线后 就没有热量保护了?
		
//    //如果云台状态是 无力状态，就关闭射击
//    if (gimbal_cmd_to_shoot_stop())
//    {
//        shoot_control.shoot_mode = SHOOT_STOP;
//    }

    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}
/**
  * @brief          射击数据更新
	shoot motor 是拨弹电机
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //二阶低通滤波
#if TRIG_MOTOR_TURN
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] - (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;
#else
		speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;
#endif
		
		//电机是否离线检测
		/*只扫描一次按键这个思路*/
		if(toe_is_error(TRIGGER_MOTOR_TOE))
		{
			shoot_control.trigger_motor_17mm_is_online = 0x00;
		}
		else
		{
			shoot_control.trigger_motor_17mm_is_online = 0x01;
		}

//    /*
//		别看这行注释哈, 它写错了: 电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
//		应该是:
//		这几句话的目的是判断电机方向的, 对码盘值进行积分时, 需要确定那一帧step的方向, 不能用rpm来, rpm是瞬时的
//		*/
//    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
//    {
//        shoot_control.ecd_count--;
//    }
//    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
//    {
//        shoot_control.ecd_count++;
//    }

////    if (shoot_control.ecd_count == FULL_COUNT)
//    {
//        shoot_control.ecd_count = -(FULL_COUNT - 1);//-(FULL_COUNT - 1);
//    }
//    else if (shoot_control.ecd_count == -FULL_COUNT)
//    {
//        shoot_control.ecd_count = FULL_COUNT - 1;
//    }
//    //计算输出轴角度 5-19之前
//		//ecd_count 编码器 计数 数的圈数 整数
//		//之前的转了几圈 + 当前的编码器值 将其转换为弧度制
//    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
		
		//添加了码盘值积分后 对拨弹盘angle的计算 SZL 5-19
		//之前的转了几圈 + 当前的编码器值 将其转换为弧度制 马盘值里程计
#if TRIG_MOTOR_TURN
		shoot_control.angle = -(shoot_control.shoot_motor_measure->total_ecd + shoot_control.shoot_motor_measure->delta_ecd) * MOTOR_ECD_TO_ANGLE;
#else
		shoot_control.angle = (shoot_control.shoot_motor_measure->total_ecd + shoot_control.shoot_motor_measure->delta_ecd) * MOTOR_ECD_TO_ANGLE;
		//shoot_control.angle = (shoot_control.shoot_motor_measure->total_ecd + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
#endif

		//其实可以把所有按键相关状态机放到这里 从set mode中移到这里面 虽然会有耦合
		
		//按键V记时, V只是记录了上一次状态, 但是没有计数
		if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_V)
		{
			if(shoot_control.press_key_V_time < PRESS_LONG_TIME_V)
			{
				shoot_control.press_key_V_time++;
			}
			shoot_control.last_key_V_sts = 1;
		}
		else
		{
			shoot_control.last_key_V_sts = 0;
			shoot_control.press_key_V_time = 0;
		}
    //鼠标按键
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    //长按计时
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME_L)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    if (shoot_control.press_r)
    {
        if (shoot_control.press_r_time < PRESS_LONG_TIME_R)
        {
            shoot_control.press_r_time++;
        }
    }
    else
    {
        shoot_control.press_r_time = 0;
    }

    //射击开关下档时间计时
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }
		//12-30-2021 SZL 添加 friction 电机 反馈 数据
		shoot_control.left_fricMotor.fricW_speed = M3508_FRIC_MOTOR_RPM_TO_LINEAR_VETOR_SEN * shoot_control.left_friction_motor_measure->speed_rpm;
		shoot_control.right_fricMotor.fricW_speed = M3508_FRIC_MOTOR_RPM_TO_LINEAR_VETOR_SEN * shoot_control.right_friction_motor_measure->speed_rpm;
		
		//Added for J-scope debug
		temp_rpm_right = shoot_control.right_friction_motor_measure->speed_rpm;
		temp_rpm_left = shoot_control.left_friction_motor_measure->speed_rpm;
		
}

static void trigger_motor_turn_back_17mm(void)
{
    if( shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    }
    else
    {
        shoot_control.speed_set = -shoot_control.trigger_speed_set;
    }

    if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
    }
}

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control_17mm(void)
{
    //每次拨动 1/4PI的角度
    if (shoot_control.move_flag == 0)
    {
        shoot_control.set_angle = (shoot_control.angle + PI_TEN);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format
        shoot_control.move_flag = 1;
    }
		
		/*这段代码的测试是在NewINF v6.4.1 中测试的, 也就是不会出现:(发射机构断电时, shoot_mode状态机不会被置为发射相关状态)
		整体的逻辑是: 如果发射机构断电, shoot_mode状态机不会被置为发射相关状态, 不会进入此函数; 这段代码只是在这里保险
	  电机掉线, 即发射机构断电特征出现时, 放弃当前发射请求*/
		if(shoot_control.trigger_motor_17mm_is_online == 0x00)
		{
				shoot_control.set_angle = shoot_control.angle;
				return;
		}
		
    if(0)//shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode = SHOOT_DONE;
    }
    //到达角度判断
    if ((shoot_control.set_angle - shoot_control.angle) > 0.05f)//(rad_format(shoot_control.set_angle - shoot_control.angle) > 0.0005f)//0.15f) //pr改动前为0.05f shooter_rad_format
    {
        //没到达一直设置旋转速度
        shoot_control.trigger_speed_set = TRIGGER_SPEED;
        trigger_motor_turn_back_17mm();
    }
    else
    {
        shoot_control.move_flag = 0;
			  shoot_control.shoot_mode = SHOOT_DONE; //pr test
    }
   
}

const shoot_control_t* get_robot_shoot_control()
{
	return &shoot_control;
}

