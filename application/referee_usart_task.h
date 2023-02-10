
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM����ϵͳ���ݴ���
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H
#include "main.h"
#include "struct_typedef.h"

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024
#define PC_FIFO_BUF_LENGTH 1024

/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          ����ϵͳ����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void referee_usart_task(void const * argument);

/*
@brief         ������ͻ���������    ��UART1)
                Rui Peng 2021/2/25
								���԰汾
*/
void sendPack(uint8_t cmd_ID,uint8_t level, uint8_t robot_ID);
	
/*
@brief  ����CV����EE/ͨѶ����
*/
/*
Autoaim delta ���귢��TypeC��
Auto fire LOCK �������귢��TypeC
*/

#define HEADER 0xaf //��ͷ
#define AUTOFIRE 0xff //�Զ�����
#define AUTOAIM 0x00 //�Զ���׼
#define MANUEL  0xfa //���ֶ�ģʽ
#define BIGBUFF 0xbb //����
#define SINBIGBUFF 0xaa //���ٴ��

#define LEVEL_I  0x01 //1��
#define LEVEL_II 0x02 //2��
#define LEVEL_III 0x03 //3��

#define ROBOTID_RED 0xff  //������������ɫ
#define ROBOTID_BLUE 0x00  //������������ɫ

/*
//miniPC ״̬��

//����Cap������
	uint8_t max_charge_pwr_command;
	uint8_t fail_safe_charge_pwr_command;
	
//���յ��� ����ת���������
float EBPct_fromCap; //����������0J 0%-100%
*/
//SZL 5-31-2022 EE->CV
struct typeC_msg_to_PC_t
{
		uint8_t header;
		uint8_t cmd_id;
		uint16_t shooter_speed;//PID estimate speed
		uint8_t robot_id;
		//uint16_t (*INS_pc_quat)[4];����ָ��, ָ�������ָ��
		uint16_t INS_pc_quat[4];
		uint8_t checksum;
};

/*
@brief         ����λ��ͨѶ  ����   �����ͣ� ��UART1)
                Rui Peng 2021/2/25
								���԰汾
*/
void sendData_Task_EE_To_PC(void);




/*
@brief         ��λ��ͨѶ ����  ���� ��ʼ����UART1)
                Rui Peng 2021/2/25
								���԰汾
*/
void pc_control_init(void);


/*
@brief         ��λ��ͨѶ ����  ��UART1)
                Rui Peng 2021/2/25
								���԰汾
*/


//֮ǰ��8 ������11���ֽ�
#define PACK_LENG 11
#define GIMBAL_MOVEMENT 1
//buffer�е�index
#define CMDID_INDEX 1
#define CV_status 1
#define YAW_INDEX_MSB 2
#define YAW_INDEX_LSB 3
#define PITCH_INDEX_MSB 4
#define PITCH_INDEX_LSB 5
#define DISTANCE_INDEX 6
#define SHOOT_INDEX 7

//uint8_t  autoAimFlag = 1; //�Զ���׼����״̬ 0�� 1�Զ���׼
//uint8_t shootCommand = 0x00;//�Զ�����ָ��  0x00 = ͣ��  0xff = ����
//uint8_t fricCommand = 0x01;// Ħ����ת��ָ��  0x01 =��ת  0x02 = ��ת
//fp32 yawMove = 0;  //��̨����
//fp32 pitchMove = 0;  //��̨����
//uint32_t timeFlag = 0; //
typedef enum
{
	miniPC_offline,
	miniPC_online
}miniPC_connection_status_e;//�������ṹ�� ��ͬ�ĳ������ݹ���
typedef struct
{
	//MiniPC �Ǳ߷���������Ϣ
	uint8_t cmd_id;
	uint8_t cv_status; //0����ر�, 1AID, 2LOCK
	int16_t yawCommand; //delta yaw
	int16_t pitchCommand; //delta pitch
	uint16_t dis_raw;
	uint8_t shootCommand;//�Զ�����ָ��  0x00 = ͣ��  0xff = ����
	
	
	//����FSM �Լ� ����
	fp32 dis; //��λת��֮��ľ���
	/*���������0 ��˵��û��ʶ��Ŀ��*/
	uint8_t autoAimFlag; //�Զ���׼����״̬ 0�� 1�Զ���׼AID 2�Զ���׼LOCK
	
	uint8_t enemy_detected;//0 δʶ��, 1ʶ����
	
	fp32 yawMove_aid;
	fp32 pitchMove_aid;
	
	fp32 yawMove_absolute;
	fp32 pitchMove_absolute;
	
	miniPC_connection_status_e miniPC_connection_status;
}miniPC_info_t;

//struct gimbal_cmd
//{
//	uint8_t head;
//	uint8_t id;
//	int16_t pitch;
//	int16_t yaw;
//};

#endif
