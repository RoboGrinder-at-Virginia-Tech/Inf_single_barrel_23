/**
  ****************************(C) COPYRIGHT 2022 RoboGrinder at Virginia Tech****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.���̹��ʿ���
  * @note       Based on competition rules: chassis power control.
  *             Two ways to limit chassis power 
	*							1) The overall program structure is from DJI 2019 example code and file "chassis_power_control"
	*									based on remaining buffer # to regulate and to control the raw esc control current(message send to can bus)
	*             2) Speed adaptive chassis power control; the program will regulate and control the PID speed of each wheel first.
  *           
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
	*  V2.0.0     July-20-2022    Zelin Shen      2. re-write basic chassis power control function; support dynamic charging power
	*																								 add speed adaptive chassis power control;
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 RoboGrinder at Virginia Tech****************************
  */
#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H
#include "chassis_task.h"
#include "main.h"

#define USE_SpeedAdaptiveChassisPowerControl 0
//#define HERO_CHASSIS_POWER_CONTROL
#define INFANTRY_CHASSIS_POWER_CONTROL

//���̳�ʼ״̬ʱ �������� Ҳ������͹���
#ifdef HERO_CHASSIS_POWER_CONTROL
#define INITIAL_STATE_CHASSIS_POWER_LIM 50.0f
#else
#define INITIAL_STATE_CHASSIS_POWER_LIM 40.0f
#endif

#define MAX_REASONABLE_CHARGE_PWR 160

/*
ԭʼ����:
#define POWER_LIMIT         500.0f//500
#define WARNING_POWER       300.0f //40.0f   
#define WARNING_POWER_BUFF  50.0f   

WARNING_POWER��ҪС��POWER_LIMIT
5<chassis_power_buffer<(WARNING_POWER_BUFF=40)ʱ, ֻ��*debuf => ���������� <=> buffer_debuff_total_current_limit * power_scale
buffer_debuff_total_current_limit �������������������� ��� ����(����)

<5ʱ, ��֤�������û���������
7-10-2022 ������ֵ
#define POWER_LIMIT         500.0f//500
#define WARNING_POWER       300.0f//40.0f   
#define WARNING_POWER_BUFF  40.0f//50.0f   

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f
����ע��:
64000.0f = 16000 * 4, 16000ָ16A

�޳������� ֱ������
*/
#define POWER_LIMIT         500.0f//500
#define WARNING_POWER       400.0f//40.0f   
#define WARNING_POWER_BUFF  40.0f//50.0f   

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f   //����ϵͳ����ʱ�ܵ�������
#define BUFFER_TOTAL_CURRENT_LIMIT      40000.0f	//���� ����������С��40jʱ 5<chassis_power_buffer<(WARNING_POWER_BUFF=40)
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f  //�������� ʱ ����

#define POWER_TOTAL_CURRENT_LIMIT_WHEN_NO_BUFF_USED 20833.3f

//Speed-adaptive chassis power control; related parameter
/*������һ�β���: ����֮ǰ:
#define WARNING_ENERGY_BUFF  40.0f 
#define MINIMUM_ENERGY_BUFF 10.0f //�趨�������� Σ��ֵ
...
#define ENERGY_BUFF_OUTPUT_CUTOFF_POINT 3.0f
*/
#define WARNING_ENERGY_BUFF  30.0f//35.0f //40.0f 
#define MINIMUM_ENERGY_BUFF  10.0f //10.0f //�趨�������� Σ��ֵ
#define MAX_POWER_VALUE 400.0f //300.0f//220.0f

#define REFEREE_OFFLINE_POWER_LIM_VAL MAX_POWER_VALUE //100.0f

/*When ENERGY_BUFF_OUTPUT_CUTOFF_POINT is reached, output will be disabled; and the buff eng need to be recharged to MINIMUM_ENERGY_BUFF to re-enable the output*/
#define ENERGY_BUFF_OUTPUT_CUTOFF_POINT 5.0f //3.0f

//����ϵͳ���� ����������Ϣ ���; ʵʱ�����������ݣ�0x0202������Ƶ�ʣ�50Hz; ��0.02s
#define CHASSIS_REFEREE_COMM_TIME 0.02f;

/*
7-28-2022 ֮ǰ����:
#define superCap_WARNING_ENERGY_BUFF 1500.0f
#define superCap_MINIMUM_ENERGY_BUFF 700.0f
#define superCap_MAX_POWER_VALUE 300.0f

#define superCap_ENERGY_BUFF_OUTPUT_CUTOFF_POINT 650.0f

7-21-2022 �ó������ݵĹ��ʿ���; speed-adaptive chassis power control
*/
#define superCap_WARNING_ENERGY_BUFF 1200.0f //1500.0f
#define superCap_MINIMUM_ENERGY_BUFF 750.0f //700.0f
//7-28-2022��, �ɻ��������ĵ��ظ�Ϊ ���ڵ�ѹ�ĵ���, ��Ϊ����Ҫֱ��һ��, ���Һ� ֱ���ٶ�����Ӧ���̹��ʿ��� �㷨�� �������������
#define superCap_WARNING_VOL 20.0f //��ӦsuperCap_WARNING_ENERGY_BUFF
#define superCap_MINIMUM_VOL 15.81f //superCap_MINIMUM_ENERGY_BUFF
#define superCap_MAX_POWER_VALUE 400.0f//300.0f

#define superCap_REFEREE_OFFLINE_POWER_LIM_VAL superCap_MAX_POWER_VALUE

/*When superCap_ENERGY_BUFF_OUTPUT_CUTOFF_POINT is reached, output will be disabled; and the buff eng need to be recharged to MINIMUM_ENERGY_BUFF to re-enable the output*/
#define superCap_ENERGY_BUFF_OUTPUT_CUTOFF_POINT 650.0f//700.0f //650.0f
#define superCap_VOL_OUTPUT_CUTOFF_POINT 3.5f //13.50f //14.72f

//����ϵͳ���� ����������Ϣ ���; ʵʱ�����������ݣ�0x0202������Ƶ�ʣ�50Hz; ��0.02s
#define superCap_CHASSIS_SUPERCAP_COMM_TIME 0.02f; //����ʹ�ô���ֵ���㷨�����ر����, Ŀǰ��ʱ����0.02f

/*���²���δʹ��*/
#define superCap_POWER_LIMIT         500.0f//500
#define superCap_WARNING_POWER       300.0f//40.0f   
#define superCap_WARNING_POWER_BUFF  40.0f//50.0f   

#define superCap_NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4
#define superCap_BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define superCap_POWER_TOTAL_CURRENT_LIMIT       20000.0f
/*--------------*/

typedef enum
{
	below_ENERGY_CRITICAL_POINT,
	above_ENERGY_CRITICAL_POINT,
}energy_buff_output_cutoff_point_status_e;//this enum is for debug only

typedef enum
{
	adp_cpc_MAX_loop_cnt_reached,
	adp_cpc_NORMAL,
}speed_adaptive_chassis_power_control_result_status_e;//�������ṹ�� ��ͬ�ĳ������ݹ���

typedef struct
{
	 fp32 motor_final_current[4];
	 fp32 motor_final_total_current;
	 uint16_t chassis_power_limit;//��ǰ�����˳�繦������
	 uint16_t superCap_charge_pwr;//��ǰ �������� ��繦��
	
	 fp32 chassis_power;//����ϵͳ ���̹���, ��������˳������ݳ�繦��
   fp32 chassis_power_buffer;//����ϵͳ 60J��������
   fp32 superCap_e_buffer;
	 fp32 superCap_vol;
	
   fp32 total_current;
	
	 fp32 total_current_limit;
	
	 fp32 buffer_debuff_total_current_limit;
	
	 //...�����Ĳ��� ֻ��Ҫ�ٶ�����Ӧ��صı���
	 uint8_t robot_id;
	 
	 //speed adaptive related variable
	 speed_adaptive_chassis_power_control_result_status_e adp_pwr_ctrl_result_status;
	 fp32 total_current_unit_amp;
	
	 energy_buff_output_cutoff_point_status_e ene_cutoff_sts;
	 fp32 critical_val; //critical val to disable or enable the output; below this val=cutoff output; above this val=re-enable output
	
	 fp32 p_max;//��ǰ֡������� ���ù�������; ��λΪ ��
	 uint32_t max_speed_adp_loop_cnt; //max speed adaptive loop count when function runs once excluding max loop cnt; ����ʱ��ѭ���˼���
	 uint32_t num_of_normal_loop; //number of times that the speed adaptive mechanism worked normally
	 uint32_t num_loop_limit_reached; //number of times that the program reached the upper limit of the loop
	 uint32_t current_loop_cnt;
	
}chassis_energy_control_t;// ʹ�ó�������ʱ�Ĺ��ʱջ��ṹ��

//��ʹ�ó�������ʱ ֱ��ʱ �Ĺ��ʱջ��ṹ��
typedef struct
{
	 fp32 motor_final_current[4];
	 fp32 motor_final_total_current;
	 uint16_t chassis_power_limit;//��ǰ�����˳�繦������
	 fp32 chassis_power;
   fp32 chassis_power_buffer;
   
   fp32 total_current;
	
	 fp32 total_current_limit;//=���漸���е�����һ��
	
	 fp32 buffer_debuff_total_current_limit; //���� ����������С��40jʱ 5<chassis_power_buffer<(WARNING_POWER_BUFF=40) ���ڼ���״̬ʱ�� �����Ƶ���
	 fp32 buffer_minimum_total_current_limit; // <5 ʱ�ı�֤, ��������buffer������
	 fp32 power_total_current_limit; //���� ��ֵ���� ʱ ����
	
	 uint8_t robot_id;
	 
	 //speed adaptive related variable
	 speed_adaptive_chassis_power_control_result_status_e adp_pwr_ctrl_result_status;
	 fp32 total_current_unit_amp;
	
	 energy_buff_output_cutoff_point_status_e ene_cutoff_sts;
	 fp32 critical_power_buffer; //critical val to disable or enable the output; below this val=cutoff output; above this val=re-enable output
	
	 fp32 p_max;//��ǰ֡������� ���ù�������; ��λΪ ��
	 uint32_t max_speed_adp_loop_cnt; //max speed adaptive loop count when function runs once excluding max loop cnt; ����ʱ��ѭ���˼���
	 uint32_t num_of_normal_loop; //number of times that the speed adaptive mechanism worked normally
	 uint32_t num_loop_limit_reached; //number of times that the program reached the upper limit of the loop
	 uint32_t current_loop_cnt;
}chassis_energy_control_direct_connect_t;

/*
 fp32 chassis_power = 0.0f;
 fp32 chassis_power_buffer = 0.0f;
 fp32 total_current_limit = 0.0f;
 fp32 total_current = 0.0f;
 robot_id = get_robot_id();
*/

/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data 
  * @retval         none
  */
/**
  * @brief          ���ƹ��ʣ���Ҫ���Ƶ������
  * @param[in]      chassis_power_control: ��������
  * @retval         none
  */
extern void chassis_power_control(chassis_move_t *chassis_power_control);
extern void speed_adaptive_chassis_power_control(chassis_move_t *chassis_power_control);
extern void superCap_speed_adaptive_chassis_power_control(chassis_move_t *chassis_power_control);
#endif
