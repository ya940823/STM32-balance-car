/*************************************************************
 ƽ�⳵������Ҫ����
**************************************************************/
#include "Balancecar.h"
#include "I2C_MPU6050.h"
#include "MOTOR.h"
#include "Serial.h"
#include "MPU6050.h"
#include "Ultrasonic.h"
#include "stm32f10x_gpio.h"
#include "math.h" 
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/*****************************************/
TypeUsart3 MyUsart3;  // ����ģ�����

// ƽ��С���˶�״̬ö��
enum{
  enCARSTOP = 0,
  enCARRUN,
  enCARBACK,
  enCARLEFT,
  enCARRIGHT,
  enCARTLEFT,
  enCARTRIGHT
}enCarState;


#define 	run_balancecar     '1'// ƽ�⳵ǰ��
#define 	back_balancecar    '2'// ƽ�⳵����
#define 	left_balancecar    '4'// ƽ�⳵��ת
#define 	right_balancecar   '3'// ƽ�⳵��ת
#define 	stop_balancecar    '0'// ƽ�⳵ֹͣ


int get_newcarstate = enCARSTOP; //  1ǰ2��3��4��0ֹͣ

/*****************������************************/
uint8_t CLB_u8MainEventCount;						  //��ѭ���жϼ���  ��SysTick_Handler(void)��ʹ�� ÿ1ms��1
uint8_t CLB_u8SpeedControlCount;						  //�ٶȿ���ѭ������  ��SysTick_Handler(void)��ʹ�� ÿ5ms��1
uint8_t CLB_u8SpeedControlPeriod;
uint8_t CLB_u8DirectionControlPeriod;
uint8_t CLB_u8DirectionControlCount;					  //ת�����ѭ������  ��SysTick_Handler(void)��ʹ�� ÿ5ms��1 
uint8_t CLB_u8trig;
uint8_t ucBluetoothValue;                      //������������


/******������Ʋ���******/
float CLB_fSpeedControlOut;						   //�ٶȿ���PWM
float CLB_fSpeedControlOutOld;
float CLB_fSpeedControlOutNew;
float CLB_fAngleControlOut;
float CLB_fLeftMotorOut;
float CLB_fRightMotorOut;

float CLB_fCarAngle;						 //�Ƕȿ���PWM

//-----�ǶȻ����ٶȻ�PID���Ʋ���-----
float  CLB_fCarAngle_P = 91.3;  
float  CLB_fCarAngle_D = 0.21;	

float  CLB_fCarSpeed_P= - 5.1;
float  CLB_fCarSpeed_I= - 0.10;;


/******************************************************
    �ٶȿ��Ʋ���
******************************************************/

int16_t   CLB_s16LeftMotorPulse;					   //����������
int16_t	  CLB_s16RightMotorPulse;					   //�ҵ��������

int32_t   CLB_s32LeftMotorPulseOld;
int32_t   CLB_s32RightMotorPulseOld;
int32_t   CLB_s32LeftMotorPulseSigma;				  //50ms��������ֵ
int32_t   CLB_s32RightMotorPulseSigma;				 //50ms�ҵ������ֵ

float CLB_fCarSpeed;							           //�������̵ó��ĳ���
float CLB_fCarSpeedOld;

float CLB_fCarPosition;						          //��������ͨ������õ���С��λ��

/*-----��ͣ����-----*/
int leftstop=0;
int rightstop=0;
int stopflag=0;

/******������********/
float fultrasonic  = 0;							         //������������
float Distance  = 0;									       //����������

/******�������Ʋ���******/																	
float CLB_fBluetoothSpeed;						     //�������Ƴ���
float CLB_fBluetoothDirectionNew;			     //����ƽ���������ʹ��
float CLB_fBluetoothDirectionSL;			     //��ת��־λ  
float CLB_fBluetoothDirectionSR;			     //��ת��־λ	  
int Ultrasonicflag = 0;
int y1,z1,y2,z2,flagbt;

float dac = 0,dgy = 0;

/************��ת*****************/
float CLB_fBluetoothDirectionL;				   //����ת��־λ  
float CLB_fBluetoothDirectionR;				   //����ת��־λ  
int driectionxco=800;

/**********��ʱ�Ӻ���*******************************************/
void delay_nms(uint16_t time)
{    
   uint16_t i=0;  
   while(time--)
   {
      i=12000;  
      while(i--) ;    
   }
}
/**************************************************************/

/***************************************************************
** ��������: BalancingCarParameterInit
** ��������: ƽ�⳵������ʼ��
***************************************************************/
void BalancingCarParameterInit(void)
{
	CLB_s16LeftMotorPulse = CLB_s16RightMotorPulse = 0;					                  //��������ֵ   ��ʼ��
	CLB_s32LeftMotorPulseSigma = CLB_s32RightMotorPulseSigma = 0;		              //����������	 ��ʼ��

	CLB_fCarSpeed = CLB_fCarSpeedOld = 0;								                          //ƽ��С������	��ʼ��
	CLB_fCarPosition = 0;												                                  //ƽ��С��λ����	��ʼ��
	CLB_fCarAngle    = 0;												                                  //ƽ��С���Ƕ�ֵ	��ʼ��

	CLB_fAngleControlOut = CLB_fSpeedControlOut = CLB_fBluetoothDirectionNew = 0;	//�Ƕ�PWM������PWM����������PWM	 ��ʼ��
	CLB_fLeftMotorOut    = CLB_fRightMotorOut   = 0;								              //���ҳ���PWM���ֵ 			       ��ʼ��
	CLB_fBluetoothSpeed  = 0;														                          //�������Ƴ���ֵ                 ��ʼ��
	CLB_fBluetoothDirectionL =CLB_fBluetoothDirectionR= 0;						            //��������������ת��־λ         ��ʼ��
	CLB_fBluetoothDirectionSL =CLB_fBluetoothDirectionSR= 0;						          //������������ת���־λ         ��ʼ��
	
  CLB_u8MainEventCount=0;															                          //����5ms��ʱ���ӳ���SysTick_Handler(void)�����жϼ���λ
	CLB_u8SpeedControlCount=0;														                        //����5ms��ʱ���ӳ���SysTick_Handler(void)��50ms�ٶ�ƽ���������λ
  CLB_u8SpeedControlPeriod=0;														                        //����5ms��ʱ���ӳ���SysTick_Handler(void)��50ms�ٶ�ƽ���������λ

	fultrasonic=0;											                                          //����5ms��ʱ���ӳ���SysTick_Handler(void)�г�����ƽ���������λ
}


/***************************************************************
** ��������: AngleControl
** ��������: �ǶȻ����ƺ���

***************************************************************/
void AngleControl(void)	 
{
	if(flagbt==1)
	{
		CLB_fCarAngle_P=0;
		CLB_fCarAngle_P=y1*1.71875;
	}
		if(flagbt==2)
	{
		CLB_fCarAngle_D=0;
		CLB_fCarAngle_D=(z1-64)*0.15625;
	}
	dac=accel[2];
	dgy=gyro[2];
	if(Pitch==0||Pitch<-20||Pitch>20)			     //MPU6050״ָ̬ʾ�� STM32���İ� PC13 ��ɫ������Ϊ������
	{
	  GPIO_ResetBits(GPIOC, GPIO_Pin_13); 		 //MPU6050״ָ̬ʾ�� STM32���İ� PC13 ��ɫ������Ϊ������
	}
	else 
	{GPIO_SetBits(GPIOC, GPIO_Pin_13);}			 //MPU6050״̬����ʱLED��Ϩ��
	
	CLB_fCarAngle = Roll - CAR_ZERO_ANGLE;													   //DMP ROLL��������Ƕ���Ԥ��С����б�Ƕ�ֵ�Ĳ�ó��Ƕ�   
	CLB_fAngleControlOut =  CLB_fCarAngle * CLB_fCarAngle_P + gyro[0] * CLB_fCarAngle_D ;	  //�Ƕ�PD����							   
}

/***************************************************************
** ��������: SetMotorVoltageAndDirection
** ��������: ���ת��             
***************************************************************/
void SetMotorVoltageAndDirection(int16_t s16LeftVoltage,int16_t s16RightVoltage)
{
	  uint16_t u16LeftMotorValue;
	  uint16_t u16RightMotorValue;
	
    if(s16LeftVoltage<0)								 //������PWM���Ϊ��ʱ PB14��Ϊ�� PB15��Ϊ��
    {	
	    GPIO_SetBits(GPIOB, GPIO_Pin_14 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_15 );
      s16LeftVoltage = (-s16LeftVoltage);
    }
    else 
    {	
      GPIO_SetBits(GPIOB, GPIO_Pin_15 );  //������PWM���Ϊ��ʱ PB14��Ϊ�� PB15��Ϊ��
      GPIO_ResetBits(GPIOB, GPIO_Pin_14 ); 
      s16LeftVoltage = s16LeftVoltage;
    }

    if(s16RightVoltage<0)
    {															       //���ҵ��PWM���Ϊ��ʱ PB12��Ϊ�� PB13��Ϊ��
      GPIO_SetBits(GPIOB, GPIO_Pin_13 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_12 );
      s16RightVoltage = (-s16RightVoltage);
    }
    else														    //���ҵ��PWM���Ϊ��ʱ PB12��Ϊ�� PB13��Ϊ��
    {
	    GPIO_SetBits(GPIOB, GPIO_Pin_12 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_13 );	
     
      s16RightVoltage = s16RightVoltage;
    }
		
	   u16RightMotorValue= (u16)s16RightVoltage;
	   u16LeftMotorValue = (u16)s16LeftVoltage;


	TIM_SetCompare3(TIM2,u16LeftMotorValue);			  //TIM2�� u16RightMotorValue�Աȣ�����ͬ��ת���Σ�����PWMռ�ձ�
	TIM_SetCompare4(TIM2,u16RightMotorValue);			  //TIM3�� u16LeftMotorValue�Աȣ�����ͬ��ת���Σ�����PWMռ�ձ�
		
	// �ж�ƽ�⳵�Ƿ������ͣ���ߵ���
  if(Pitch>10||Pitch<-10&CLB_fBluetoothDirectionSR==0&CLB_fBluetoothDirectionSL==0)
	{		
		TIM_SetCompare3(TIM2,0);
		TIM_SetCompare4(TIM2,0);
		stopflag=1;		
	}
	else stopflag=0;
	
	if(CLB_fCarAngle > 50 || CLB_fCarAngle < (-50))
	{
		TIM_SetCompare3(TIM2,0);
		TIM_SetCompare4(TIM2,0);  
		stopflag=1;	
	}
	else stopflag=0;
}

// ����������
void MotorOutput(void)																					  
{	   
	  //�ҵ��ת��PWM�����ں�ƽ��Ƕȡ��ٶ����
		CLB_fLeftMotorOut  = CLB_fAngleControlOut +CLB_fSpeedControlOutNew + CLB_fBluetoothDirectionNew;		//����ת��PWM�����ں�ƽ��Ƕȡ��ٶ����	
    CLB_fRightMotorOut = CLB_fAngleControlOut +CLB_fSpeedControlOutNew - CLB_fBluetoothDirectionNew;		//�ҵ��ת��PWM�����ں�ƽ��Ƕȡ��ٶ����

	if((int16_t)CLB_fLeftMotorOut  > MOTOR_OUT_MAX)	CLB_fLeftMotorOut  = MOTOR_OUT_MAX;
	if((int16_t)CLB_fLeftMotorOut  < MOTOR_OUT_MIN)	CLB_fLeftMotorOut  = MOTOR_OUT_MIN;
	if((int16_t)CLB_fRightMotorOut > MOTOR_OUT_MAX)	CLB_fRightMotorOut = MOTOR_OUT_MAX;
	if((int16_t)CLB_fRightMotorOut < MOTOR_OUT_MIN)	CLB_fRightMotorOut = MOTOR_OUT_MIN;
	
    SetMotorVoltageAndDirection((int16_t)CLB_fLeftMotorOut,(int16_t)CLB_fRightMotorOut);
    
}
// ��ȡ���������
void GetMotorPulse(void)              //�ɼ�����ٶ�����
{ 
	uint16_t u16TempLeft;
	uint16_t u16TempRight;
	
	u16TempLeft = TIM_GetCounter(TIM3);   //  TIM3��ʱ���������
 	u16TempRight= TIM_GetCounter(TIM4);	//	 TIM4��ʱ���������
	leftstop=u16TempLeft;
	rightstop=u16TempRight;
	TIM_SetCounter(TIM3,0);
	TIM_SetCounter(TIM4,0);  //����
	CLB_s16LeftMotorPulse=u16TempLeft;
	CLB_s16RightMotorPulse=(-u16TempRight);
		
	CLB_s32LeftMotorPulseSigma  +=CLB_s16LeftMotorPulse;		 //����ֵ���� 40ms����ֵ
	CLB_s32RightMotorPulseSigma +=CLB_s16RightMotorPulse; 	 //����ֵ���� 40ms����ֵ
}

/***************************************************************
** ��������: SpeedControl
** ��������: �ٶȻ����ƺ���
***************************************************************/
void SpeedControl(void)
{
	CLB_fCarSpeed = (CLB_s32LeftMotorPulseSigma  + CLB_s32RightMotorPulseSigma );  //���ҵ��������ƽ��ֵ��ΪС����ǰ����
	CLB_s32LeftMotorPulseSigma = CLB_s32RightMotorPulseSigma = 0;	                //ȫ�ֱ��� ע�⼰ʱ����		
	CLB_fCarSpeedOld *= 0.7;
	CLB_fCarSpeedOld +=CLB_fCarSpeed*0.3;
	
	CLB_fCarPosition += CLB_fCarSpeedOld; 		 //·��  ���ٶȻ���	  
	CLB_fCarPosition += CLB_fBluetoothSpeed;   //�ں�  ���������ٶ�
	CLB_fCarPosition +=	fultrasonic;		       //�ں�  �����������ٶ�
	if(stopflag==1)
	{
		CLB_fCarPosition=0;	
	}
	//������������//
	if((int32_t)CLB_fCarPosition > CAR_POSITION_MAX)    CLB_fCarPosition = CAR_POSITION_MAX;
	if((int32_t)CLB_fCarPosition < CAR_POSITION_MIN)    CLB_fCarPosition = CAR_POSITION_MIN;
	
	if(flagbt==3)
	{
		CLB_fCarSpeed_P=0;
		CLB_fCarSpeed_P=(y2-128)*0.46875;
	}
	if(flagbt==4)
	{
		CLB_fCarSpeed_I=0;
		CLB_fCarSpeed_I=(z2-192)*0.15625;
	}																							  
	CLB_fSpeedControlOutNew = (CLB_fCarSpeedOld -CAR_SPEED_SET ) * CLB_fCarSpeed_P + (CLB_fCarPosition - CAR_POSITION_SET ) * CLB_fCarSpeed_I; //�ٶ�PI�㷨 �ٶ�*P +λ��*I=�ٶ�PWM���
}
/**********************************************************
** �������ܣ�
** ����ģ����������жϺ���
***********************************************************/
void USART3_IRQHandler(void)
{  
   if(USART_GetITStatus(USART3,USART_IT_RXNE)==SET)   //�жϽ����ж�
   {
	   USART_ClearITPendingBit(USART3,USART_IT_RXNE);  //������ձ�־λ
      MyUsart3.buff[MyUsart3.len++]=USART_ReceiveData(USART3);      //��������
   }
	 
   if(USART_GetITStatus(USART3,USART_IT_IDLE)==SET)   //�жϿ����ж�
   { 
      
		  MyUsart3.buff[MyUsart3.len]='\0';
      MyUsart3.flag=1;
      MyUsart3.len=0;
      USART_ReceiveData(USART3);
   }
}	 
//��������ȡ����ֵȻ��Ѹ�ֵ���뵽�ٶ�PI�м���
void UltraSonicProcessing(void)
{  
	if(Ultrasonicflag==0)
	{
    Distance =TIM_GetCounter(TIM1)*5*34/200.0;  //��λת��Ϊcm
		// �������ֵС��5cm��ƽ�⳵������PWMֵ
	  if(Distance <  5.00)							
		{
	    	fultrasonic= (-300);
	    }
		// �������ֵ����5cm��С�ڻ����8cmС��ǰ����׷��
		else if(Distance >= 5 & Distance <= 8)  
		{
			fultrasonic=500;
		}
		// ����ֵ����8cm������������ʧЧ
	  else 
		{
			fultrasonic=0;	
 	  }
  }
 }
// ƽ��С�����Ƴ���
void Balancingcarcontrol(void)
{
	switch (get_newcarstate)
	{
		case enCARSTOP: //ֹͣ
		{
			CLB_fBluetoothSpeed = 0;
			fultrasonic=0;
			CLB_fBluetoothDirectionNew=0;
			Ultrasonicflag=0;
		} break; 					   
		case enCARRUN: //��ǰ�ٶ� -800
		{
			CLB_fBluetoothDirectionNew= 0; 	
			CLB_fBluetoothSpeed =  -800 ;
			Ultrasonicflag=1;
		}break;	   
		case enCARLEFT://��ת 
		{
			CLB_fBluetoothDirectionNew= 300; 
			Ultrasonicflag=1;

		}break;   
		case enCARRIGHT: //��ת
		{
			CLB_fBluetoothDirectionNew= -300; 
			Ultrasonicflag=1;
		}break;	
		case enCARBACK: //�����ٶ� 800
		{
			CLB_fBluetoothDirectionNew= 0; 		
			CLB_fBluetoothSpeed = (800);
			Ultrasonicflag=1;  
		}break;
		case enCARTLEFT: //����
		{
			CLB_fBluetoothDirectionNew = driectionxco; 
			Ultrasonicflag=1; 
		}break;
		case enCARTRIGHT: //����
		{
			CLB_fBluetoothDirectionNew = -driectionxco; 
			Ultrasonicflag=1;
		}break;		
		default: CLB_fBluetoothSpeed = 0; break; 					   //ֹͣ
	}
}
/***************************************************************************
����Э�����ݽ���
***************************************************************************/ 
void Packetparsing(void)
{
		if(strcmp((const char*)MyUsart3.buff,"ONA")==0){printf("go forward!\r\n"); get_newcarstate = enCARRUN;}      // ǰ��
		else if(strcmp((const char*)MyUsart3.buff,"ONB")==0){printf("go back!\r\n");get_newcarstate = enCARBACK;}    // ����
		else if(strcmp((const char*)MyUsart3.buff,"ONC")==0){printf("go left!\r\n"); get_newcarstate = enCARLEFT;}   // ��ת
		else if(strcmp((const char*)MyUsart3.buff,"OND")==0){printf("go right!\r\n");get_newcarstate = enCARRIGHT;}  // ��ת
		else if(strcmp((const char*)MyUsart3.buff,"ONG")==0){printf("Left rotation!\r\n");get_newcarstate = enCARTLEFT;}   // ����
		else if(strcmp((const char*)MyUsart3.buff,"ONH")==0){printf("right rotation!\r\n");get_newcarstate = enCARTRIGHT;} // ����
		else if(strcmp((const char*)MyUsart3.buff,"ONF")==0){printf("Stop!\r\n");get_newcarstate = enCARSTOP;}   // ֹͣ
		else if(strcmp((const char*)MyUsart3.buff,"ONE")==0){printf("Stop!\r\n"); get_newcarstate = enCARSTOP;}  // ֹͣ	
}

 
 
 
 
 
 
 
 
 
