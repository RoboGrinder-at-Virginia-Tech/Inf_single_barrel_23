/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       servo_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-21-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "remote_control.h"
#include "shoot.h"
#include "client_ui_task.h"


extern ui_info_t ui_info;
extern shoot_control_t shoot_control;

/*
SZL MG-995���
��ʼֵ: min500 max2500
����:
��Сֵ: 500
�����ϰ�Min ���ó�500 ��ס�ݼ� ����Min (=500)״̬��Ϊ���չر� <=> ��Ϊ��100%ȷ���ر�

���ֵ 2000
�����ϰ�Max ����Ϊ 2000 ��ס�������� ���ӵ��ӽ�max (>1500)�ж�Ϊ���ո� ����

�궨:
Hero�ĵ��չر� PWM = 655 = Min; MAX2000

���� PWM = 1585 ��
		 PWM = 655 ��
*/

#define SERVO_MIN_PWM   655 //����
#define SERVO_MAX_PWM   1575//��

#define AMMO_BOX_COVER_CLOSE_STATE SERVO_MAX_PWM
#define AMMO_BOX_COVER_OPEN_STATE (SERVO_MIN_PWM)

#define PWM_DETAL_VALUE 10 //10

//#define SERVO1_ADD_PWM_KEY  KEY_PRESSED_OFFSET_Z
//#define SERVO2_ADD_PWM_KEY  KEY_PRESSED_OFFSET_X
//#define SERVO3_ADD_PWM_KEY  KEY_PRESSED_OFFSET_C
//#define SERVO4_ADD_PWM_KEY  KEY_PRESSED_OFFSET_V

//#define SERVO_MINUS_PWM_KEY KEY_PRESSED_OFFSET_SHIFT

#define INF_AMMO_BOX_ADD_PWM_KEY KEY_PRESSED_OFFSET_Z
#define SERVO_ADD_PWM_KEY KEY_PRESSED_OFFSET_CTRL

const RC_ctrl_t *servo_rc;
//const static uint16_t servo_key[4] = {SERVO1_ADD_PWM_KEY, SERVO2_ADD_PWM_KEY, SERVO3_ADD_PWM_KEY, SERVO4_ADD_PWM_KEY};
uint16_t servo_pwm[4] = {SERVO_MAX_PWM, SERVO_MAX_PWM, SERVO_MAX_PWM, SERVO_MAX_PWM};
/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          �������
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_task(void const * argument)
{
    servo_rc = get_remote_control_point();

    while(1)
    {
        for(uint8_t i = 0; i < 4; i++)
        {
//            if(servo_rc->key.v & KEY_PRESSED_OFFSET_Z)
//            {
//                servo_pwm[i] -= PWM_DETAL_VALUE;
//            }
//            else if( (servo_rc->key.v & SERVO_ADD_PWM_KEY) && (servo_rc->key.v & KEY_PRESSED_OFFSET_Z))
//            {
//                servo_pwm[i] += PWM_DETAL_VALUE;
//            }
						
						if((servo_rc->key.v & SERVO_ADD_PWM_KEY) && (servo_rc->key.v & KEY_PRESSED_OFFSET_Z))
            {
                servo_pwm[i] -= PWM_DETAL_VALUE;
            }
            else if(servo_rc->key.v & KEY_PRESSED_OFFSET_Z)
            {
                servo_pwm[i] += PWM_DETAL_VALUE;
            }

            //limit the pwm
           //����pwm
            if(servo_pwm[i] < SERVO_MIN_PWM)
            {
                servo_pwm[i] = SERVO_MIN_PWM;
            }
            else if(servo_pwm[i] > SERVO_MAX_PWM)
            {
                servo_pwm[i] = SERVO_MAX_PWM;
            }
						
						//�ж� Ammo Box Cover FSM
						if((servo_pwm[i] < AMMO_BOX_COVER_OPEN_STATE) || (servo_pwm[i] == AMMO_BOX_COVER_OPEN_STATE))
						{//���տ�
							//ui_info.ui_ammoBox_sts = ammoOPEN;
							shoot_control.ammoBox_sts = ammoOPEN;
						}
						else if((servo_pwm[i] > AMMO_BOX_COVER_CLOSE_STATE) || (servo_pwm[i] == AMMO_BOX_COVER_CLOSE_STATE))
						{//���չ�
							//ui_info.ui_ammoBox_sts = ammoOFF;
							shoot_control.ammoBox_sts = ammoOFF;
						}

            servo_pwm_set(servo_pwm[i], i);
        }
        osDelay(10);
    }
}


