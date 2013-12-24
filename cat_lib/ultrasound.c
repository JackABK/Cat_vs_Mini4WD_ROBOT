#include "FreeRTOS.h"
#include "timers.h"
#include "ultrasound.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"


#define GPIO_PORT_TRIGGER GPIOA
#define GPIO_PIN_TRIGGER GPIO_Pin_6
#define GPIO_PORT_ECHO GPIOB
#define GPIO_PIN_ECHO GPIO_Pin_0


static ultsound_state_err_t ultsound_state=ULTSOUND_OK;

unsigned long distance;
static unsigned int pulse_width;
xTimerHandle ultrasoundTimers;
unsigned char fTrigger=0;
unsigned char fDistance=0;



void init_trigger(void);
void init_echo(void);
	
	


void issue_sound_pulse(){
	int i;
	
	GPIO_WriteBit(GPIO_PORT_TRIGGER,GPIO_PIN_TRIGGER,Bit_SET);
	for(i=0;i<180;i++)
		__NOP();
	GPIO_WriteBit(GPIO_PORT_TRIGGER,GPIO_PIN_TRIGGER,Bit_RESET);
	
}
void UltrasoundPolling(){
	//if(ultsound_state==ULTSOUND_OK){
		ultsound_state=ULTSOUND_BUSY;
		issue_sound_pulse();
	//}
	
}

void ultra_sound_init(){
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* TIM3 channel 2 pin (PC.07) configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);
	///////////////////////////////////////////////
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period=11-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock)
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	//TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_PrescalerConfig(TIM3,84-1,TIM_PSCReloadMode_Immediate);
	
	/*Added by YINCHEN*/
	
	 /* Enable the TIM3 gloabal Interrupt */
	 NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure);

	 /* TIM3 configuration: PWM Input mode ------------------------
	      The external signal is connected to TIM3 CH2 pin (PA.01), 
	      The Rising edge is used as active edge,
	      The TIM3 CCR2 is used to compute the frequency value 
	      The TIM3 CCR1 is used to compute the duty cycle value
	   ------------------------------------------------------------ */
	 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInit(TIM3,&TIM_ICInitStructure);
#if (0)
	TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);
	/* Select the TIM3 Input Trigger: TI2FP2 */
	TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);
	/* Select the slave Mode: Reset Mode */
	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
	/* Enable the Master/Slave Mode */
	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);
	/* TIM enable counter */
	TIM_Cmd(TIM3, ENABLE);
	/* Enable the CC2 Interrupt Request */
	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
#endif
	TIM_Cmd(TIM3, ENABLE);

	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);

	init_trigger();
	
	ultrasoundTimers=xTimerCreate("ultrasound",( 1000 ), pdTRUE, ( void * ) 1,  UltrasoundPolling	 );
	xTimerStart( ultrasoundTimers, 0 );
	
//	init_echo();

	
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
		
}
#if (0)
void init_echo(){
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	SYSCFG_DeInit();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	
	SYSCFG_EXTILineConfig( EXTI_PortSourceGPIOB,EXTI_PinSource0);
	
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStruct.GPIO_Pin =  GPIO_PIN_ECHO ;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
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
	
	TIM_Cmd(TIM3, DISABLE);
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET){
		EXTI_ClearITPendingBit(EXTI_Line0);
		if(ultsound_state==ULTSOUND_BUSY){
			//distance update
			distance=0xFFFF-TIM_GetCounter(TIM3);
			
			ultsound_state=ULTSOUND_OK;
		}
	}
	
}
#endif

static unsigned long IC3ReadValue1,IC3ReadValue2;
static unsigned char CaptureNumber=0;
void TIM3_IRQHandler(void){
	/* Clear TIM3 Capture compare interrupt pending bit */
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	/* Get the Input Capture value */
	if(CaptureNumber == 0){
		IC3ReadValue1=TIM_GetCapture2(TIM3);
		CaptureNumber=1;
		
	}
	else if(CaptureNumber ==1){
		IC3ReadValue2=TIM_GetCapture2(TIM3);
		if(IC3ReadValue2>IC3ReadValue1){
			distance = (IC3ReadValue2 - IC3ReadValue1); 
		}
		else{
			distance = (0xFFFF-IC3ReadValue1) + IC3ReadValue2; 
		}
		
		CaptureNumber=0;
		
	}
	/* Duty cycle computation */
	//distance = TIM_GetCapture2(TIM3)/(SystemCoreClock/1000000/4);//(unit:us) APB Prescalar = 4
	//pulse_width=TIM_GetCounter(TIM3);
	//__NOP();
}

#if (0)
void TIM3_IRQHandler(){
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=RESET){
		
		TIM_Cmd(TIM3, DISABLE);
		GPIO_WriteBit(GPIO_PORT_TRIGGER,GPIO_PIN_TRIGGER,Bit_RESET);
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
			ultsound_state=ULTSOUND_OK;
		}
		
	}
	
}
#endif
unsigned long Get_Distance(){
	return distance;
}
