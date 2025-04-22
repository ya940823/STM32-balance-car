#ifndef __BALANCECAR_H
#define __BALANCECAR_H

#include "stm32f10x.h"

typedef struct
{
	uint8_t buff[100];
	uint8_t flag;		//���յ����ݵı�־λ
	uint8_t len;
}TypeUsart3;

extern TypeUsart3 MyUsart3;

/**********�Ƕȿ��ƺ궨��**********/									
#define    CAR_ZERO_ANGLE (0)		 

/******�ٶȿ�����غ궨��******/
#define CAR_POSITION_SET      0
#define CAR_SPEED_SET         0

#define CAR_POSITION_MAX	8000       
#define CAR_POSITION_MIN	(-8000)   
/******���������غ궨��******/
#define MOTOR_OUT_MAX           1000	   //ռ�ձ������ֵ
#define MOTOR_OUT_MIN         (-1000)    //ռ�ձȸ����ֵ

extern float CLB_fCarAngle;					
extern float CLB_fBluetoothSpeed;
extern float CLB_fBluetoothDirectionR;
extern float CLB_fBluetoothDirectionL;
extern uint8_t CLB_u8MainEventCount;
extern uint8_t CLB_u8SpeedControlCount;
extern float CLB_fSpeedControlOut,CLB_fCarAngle_P;
extern float CLB_fAngleControlOut;
extern float CLB_fSpeedControlOutNew;
extern uint8_t CLB_u8SpeedControlPeriod;
extern uint8_t CLB_u8DirectionControlPeriod;
extern uint8_t CLB_u8DirectionControlCount;
extern uint8_t CLB_u8trig;
extern uint8_t ucBluetoothValue;
extern float angle;
extern float CLB_fLeftMotorOut,CLB_fRightMotorOut,CLB_fBluetoothDirectionNew;
extern int16_t CLB_s16LeftMotorPulse,CLB_s16RightMotorPulse;
extern float Distance;
extern 	int y1,z1,y2,z2,flagbt;
extern float CLB_fCarSpeed_I,CLB_fCarSpeed_P,CLB_fCarAngle_P,CLB_fCarAngle_D;

extern void Balancingcarcontrol(void);   	    //ƽ��С������

void delay_nms(uint16_t time);
void BalancingCarParameterInit(void);    // ƽ�⳵������ʼ��
void AngleControl(void)	 ;
void MotorOutput(void);
void SpeedControl(void);
void GetMotorPulse(void);
void UltraSonicProcessing(void);
void InitMPU6050(void);
void Packetparsing(void);  // ��Э�����
#endif
