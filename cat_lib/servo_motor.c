#include "FreeRTOS.h"
#include "timers.h"
#include "servo_motor.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"


#define GPIO_PORT_SERVO GPIOA
#define GPIO_PIN_SERVO GPIO_Pin_1


static servo_err_t servo_state=SERVO_OK;
static int servo_angle=0;
static int target_angle=0;
static unsigned char servo_agility=1;

static unsigned int pulse_width;
xTimerHandle servoTimers;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;


void issue_servo_pulse(int angle){
	
	pulse_width=((angle+90)*11)+500;//us
	
	TIM_SetCounter(TIM2,pulse_width);
	//TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_Cmd(TIM2, ENABLE);
	GPIO_WriteBit(GPIO_PORT_SERVO,GPIO_PIN_SERVO,Bit_SET);
}
void ServoPolling(){
	static unsigned char count=0;
	if(servo_angle==target_angle){
		if(servo_state==SERVO_OK)
			return;
		else if(servo_state==SERVO_BUSY){
			count++;
			if(count==50){
				count=0;
				servo_state=SERVO_OK;
			}
		}
		
	}
	else if(servo_angle<target_angle){
		if(servo_angle+servo_agility < target_angle){
			servo_angle+=servo_agility;
		}
		else{
			servo_angle=target_angle;
		}
	}
	else{
		if(servo_angle-servo_agility > target_angle){
			servo_angle-=servo_agility;
		}
		else{
			servo_angle=target_angle;
		}
	}
	
	issue_servo_pulse(servo_angle);
}
	

void servo_init(){
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	servoTimers=xTimerCreate("Servo",	 ( 20 ), pdTRUE, ( void * ) 1,  ServoPolling	 );
	xTimerStart( servoTimers, 0 );
	///////////////////////////
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
<<<<<<< HEAD
	GPIO_InitStruct.GPIO_Pin =  GPIO_PIN_SERVO ; //PD12->LED3 PD13->LED4 PD14->LED5 PDa5->LED6
=======
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_1 ; //PD12->LED3 PD13->LED4 PD14->LED5 PDa5->LED6
>>>>>>> ed6dcdc62110b74476d724469704d79b30782623
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;            // Alt Function - Push Pull
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
<<<<<<< HEAD
	GPIO_Init( GPIO_PORT_SERVO, &GPIO_InitStruct ); 
	GPIO_WriteBit(GPIO_PORT_SERVO,GPIO_PIN_SERVO,Bit_RESET);
=======
	GPIO_Init( GPIOA, &GPIO_InitStruct ); 
	GPIO_WriteBit(GPIOA,GPIO_Pin_1,Bit_RESET);
>>>>>>> ed6dcdc62110b74476d724469704d79b30782623
	TIM_DeInit(TIM2);
	/* TIM2 clock enable */
	TIM_InternalClockConfig(TIM2);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period=2-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock)
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	/* TIM IT enable */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, DISABLE);
	TIM_SelectOnePulseMode(TIM2,TIM_OPMode_Single);
	TIM_ClearITPendingBit( TIM2, TIM_IT_Update);

	/*Added by YINCHEN*/
	//NVIC_InitTypeDef NVIC_InitStructure;
	 /* Enable the TIM2 gloabal Interrupt */
	 NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure);

	 

	
}

servo_err_t servo_operate(int agility,int angle){
	
	if(servo_state==SERVO_OK)
		servo_state=SERVO_BUSY;
	else
		return servo_state;

	target_angle=angle;
	servo_agility=agility;
} 

void TIM2_IRQHandler(){
	
	
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET){
		
		TIM_Cmd(TIM2, DISABLE);
		GPIO_WriteBit(GPIO_PORT_SERVO,GPIO_PIN_SERVO,Bit_RESET);
		TIM_ClearITPendingBit( TIM2, TIM_IT_Update);
		
	}

}

