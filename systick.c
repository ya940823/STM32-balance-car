#include "SysTick.h"

static __IO uint32_t TimingDelay;

void SysTick_Init(void)		  //1ms��ʱʱ��
{
	if (SysTick_Config(SystemCoreClock / 200))	// ST3.5.0��汾   1s/x= a ms  
 	{ 
		/* Capture error */ 
		while (1);
	}
		// �رյδ�ʱ��  
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
}

void Delay_us(__IO uint32_t nTime)
{ 
	TimingDelay = nTime;	

	// ʹ�ܵδ�ʱ��  
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;

	while(TimingDelay != 0);
}
 
void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
	{ 
	TimingDelay--;
	}
}
