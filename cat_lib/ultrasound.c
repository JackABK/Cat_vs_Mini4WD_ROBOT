#include "FreeRTOS.h"
#include "timers.h"
#include "ultrasound.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"


#define GPIO_PORT_TRIGGER GPIOA
#define GPIO_PIN_TRIGGER GPIO_Pin_5
#define GPIO_PORT_ECHO_0 GPIOA
#define GPIO_PIN_ECHO_0 GPIO_Pin_6
#define GPIO_PORT_ECHO_1 GPIOA
#define GPIO_PIN_ECHO_1 GPIO_Pin_7


static unsigned long CH1Distance,CH2Distance;
static xTimerHandle ultrasoundTimers;

static unsigned long IC3CH1ReadValue1,IC3CH1ReadValue2, IC3CH2ReadValue1,IC3CH2ReadValue2;
static unsigned char CH1CaptureNumber=0,CH2CaptureNumber=0;



void init_trigger(void);
void init_echo(void);
	
	


void issue_sound_pulse(){
	int i;
	GPIO_WriteBit(GPIO_PORT_TRIGGER,GPIO_PIN_TRIGGER,Bit_SET);
	/*10us delay*/
	for(i=0;i<180;i++)
		__NOP();
	GPIO_WriteBit(GPIO_PORT_TRIGGER,GPIO_PIN_TRIGGER,Bit_RESET);
	
}

#define UltrasoundPolling issue_sound_pulse
#if (0)
void UltrasoundPolling(){
	issue_sound_pulse();
}
#endif

void ultra_sound_init(){
	
	init_echo();
	init_trigger();
	
	ultrasoundTimers=xTimerCreate("ultrasound",(500 ), pdTRUE, ( void * ) 1,  UltrasoundPolling	 );
	xTimerStart( ultrasoundTimers, 0 );

	
}
void init_echo(){
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	/* TIM3 channel 2 pin (PC.07) configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_ECHO_0|GPIO_PIN_ECHO_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIO_PORT_ECHO_0, &GPIO_InitStructure);
	
	
	
	GPIO_PinAFConfig(GPIO_PORT_ECHO_0,GPIO_PinSource6,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIO_PORT_ECHO_0,GPIO_PinSource7,GPIO_AF_TIM3);
		
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		
	TIM_PrescalerConfig(TIM3,84-1,TIM_PSCReloadMode_Immediate);//24MHz->1Mz(1us)
		
	 /* Enable the TIM3 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInit(TIM3,&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInit(TIM3,&TIM_ICInitStructure);
	TIM_Cmd(TIM3, ENABLE);
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);

	

}



void init_trigger(){
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Pin =	GPIO_PIN_TRIGGER; //PD12->LED3 PD13->LED4 PD14->LED5 PDa5->LED6
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;			  // Alt Function - Push Pull
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIO_PORT_TRIGGER, &GPIO_InitStruct ); 
	GPIO_WriteBit(GPIO_PORT_TRIGGER,GPIO_PIN_TRIGGER,Bit_RESET);
		
}


void TIM3_IRQHandler(void){
	if(TIM_GetITStatus(TIM3,TIM_IT_CC1)==SET){
		if(CH1CaptureNumber == 0){
			IC3CH1ReadValue1=TIM_GetCapture1(TIM3);
			CH1CaptureNumber=1;
		
		}
		else if(CH1CaptureNumber ==1){
			IC3CH1ReadValue2=TIM_GetCapture1(TIM3);
			if(IC3CH1ReadValue2>IC3CH1ReadValue1){
				CH1Distance = (IC3CH1ReadValue2 - IC3CH1ReadValue1); 
			}
			else{
				CH1Distance = (0xFFFF-IC3CH1ReadValue1) + IC3CH1ReadValue2; 
			}
			//CH1Distance/=58;
			CH1CaptureNumber=0;
		
		}
	
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
	}
	else if(TIM_GetITStatus(TIM3,TIM_IT_CC2)==SET){
		if(CH2CaptureNumber == 0){
			IC3CH2ReadValue1=TIM_GetCapture2(TIM3);
			CH2CaptureNumber=1;
		
		}
		else if(CH2CaptureNumber ==1){
			IC3CH2ReadValue2=TIM_GetCapture2(TIM3);
			if(IC3CH2ReadValue2>IC3CH2ReadValue1){
				CH2Distance = (IC3CH2ReadValue2 - IC3CH2ReadValue1); 
			}
			else{
				CH2Distance = (0xFFFF-IC3CH2ReadValue1) + IC3CH2ReadValue2; 
			}
			//CH2Distance/=58;
			CH2CaptureNumber=0;
		
		}
	
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	}
	
	
	
}

unsigned long Get_CH1Distance(){
	return CH1Distance;
}

unsigned long Get_CH2Distance(){
	return CH2Distance;
}
