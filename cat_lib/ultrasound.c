#include "FreeRTOS.h"
#include "timers.h"
#include "ultrasound.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"


#define GPIO_PORT_TRIGGER GPIOB
#define GPIO_PIN_TRIGGER GPIO_Pin_1
#define GPIO_PORT_ECHO GPIOB
#define GPIO_PIN_ECHO GPIO_Pin_0


static ultsound_state_err_t ultsound_state=ULTSOUND_OK;


static unsigned int pulse_width;
xTimerHandle servoTimers;
unsigned char fTrigger=0;
unsigned char fDistance=0;



void init_trigger(void);
void init_echo(void);
	


void issue_sound_pulse(){
	fTrigger=1;
	TIM_SetCounter(TIM3,11);
	
	TIM_Cmd(TIM3, ENABLE);
	GPIO_WriteBit(GPIO_PORT_TRIGGER,GPIO_PIN_TRIGGER,Bit_SET);
	
}
void UltrasoundPolling(){
	
	issue_sound_pulse();
	
}

void ultra_sound_init(){
	
	servoTimers=xTimerCreate("ultrasound",	 ( 10 ), pdTRUE, ( void * ) 1,  UltrasoundPolling	 );
	xTimerStart( servoTimers, 0 );
	///////////////////////////
	init_trigger();
	init_echo();

	
}




void init_trigger(){
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		GPIO_InitStruct.GPIO_Pin =	GPIO_PIN_TRIGGER; //PD12->LED3 PD13->LED4 PD14->LED5 PDa5->LED6
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;			  // Alt Function - Push Pull
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init( GPIO_PORT_TRIGGER, &GPIO_InitStruct ); 
		GPIO_WriteBit(GPIO_PORT_TRIGGER,GPIO_PIN_TRIGGER,Bit_RESET);
		TIM_DeInit(TIM3);
		/* TIM3 clock enable */
		TIM_InternalClockConfig(TIM3);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period=11-1;
		TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock)
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
		/* TIM IT enable */
		TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
		TIM_Cmd(TIM3, DISABLE);
		TIM_SelectOnePulseMode(TIM3,TIM_OPMode_Single);
		TIM_ClearITPendingBit( TIM3, TIM_IT_Update);
	
		/*Added by YINCHEN*/
		//NVIC_InitTypeDef NVIC_InitStructure;
		 /* Enable the TIM3 gloabal Interrupt */
		 NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
		 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		 NVIC_Init(&NVIC_InitStructure);
}
void init_echo(){
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable GPIO C clock. */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	// Setup Blue & Green LED on STM32-Discovery Board to use PWM.
	GPIO_InitStruct.GPIO_Pin =  GPIO_PIN_ECHO ;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;            // Alt Function - Push Pull
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIO_PORT_ECHO, &GPIO_InitStruct ); 
	
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure);

	 
}
void EXTI0_IRQHandler(){
	unsigned long temp;
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET){
		EXTI_ClearITPendingBit(EXTI_Line0);
		if(ultsound_state==ULTSOUND_BUSY){
			//distance update
			temp=TIM_GetCounter(TIM3);

			ultsound_state=ULTSOUND_OK;
		}
	}
	
}


void TIM3_IRQHandler(){
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=RESET){
		
		TIM_Cmd(TIM3, DISABLE);
		GPIO_WriteBit(GPIO_PORT_TRIGGER,GPIO_PIN_ECHO,Bit_RESET);
		TIM_ClearITPendingBit( TIM3, TIM_IT_Update);
		if(fTrigger){
			fDistance=1;
			fTrigger=0;
			TIM_SetCounter(TIM3,0xffff);
	
			TIM_Cmd(TIM3, ENABLE);
			
		}
		else{
			fDistance=0;//shouldnt enter here
			TIM_Cmd(TIM3, DISABLE);
		}
		
	}
	
}

