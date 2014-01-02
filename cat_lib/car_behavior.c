#include "FreeRTOS.h"
#include "timers.h"

#include "stm32f4xx_gpio.h"



#include "car_behavior.h"
#include "servo_motor.h"
#include "ultrasound.h"


#include "timers.h"


#define MOTOR_INPUT_PORT GPIOA
#define MOTOR_INPUT_PIN1 GPIO_Pin_2
#define MOTOR_INPUT_PIN2 GPIO_Pin_3

#define CAR_POLLING_PERIOD 200 //unit:ms

unsigned char fButton=0;
xTimerHandle carTimers;
static unsigned char state_motor_test=0;

void init_button(void);
#if (0)
void motor_test(void* p){
	
	vTaskDelay(1000);
	servo_operate(3,0);
	vTaskDelay(1000);
		
	
	while(1){
		switch(state_motor_test){
			case 0:
				//forward left
				servo_operate(3,-15);
				vTaskDelay(2000);
				forward_motor();
				state_motor_test=1;
				
				break;
			case 1:
				//back right
				servo_operate(3,15);
				vTaskDelay(2000);
				backward_motor();
				state_motor_test=2;
				break;
			case 2:
				//forward right
				servo_operate(3,15);
				vTaskDelay(2000);
				forward_motor();
				state_motor_test=3;
				break;

			case 3:
				//back left
				servo_operate(3,-15);
				vTaskDelay(2000);
				backward_motor();
				state_motor_test=0;
				break;
		}
		vTaskDelay(2000);
		stop_motor();
		vTaskDelay(4000);
	}
}
#endif

void init_motor(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	/* Enable GPIO C clock. */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// Setup Blue & Green LED on STM32-Discovery Board to use PWM.
	GPIO_InitStruct.GPIO_Pin =  MOTOR_INPUT_PIN1|MOTOR_INPUT_PIN2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( MOTOR_INPUT_PORT, &GPIO_InitStruct ); 
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN1,Bit_RESET);
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN2,Bit_RESET);
	
}
void forward_motor(void){
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN1,Bit_SET);
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN2,Bit_RESET);
	
}
void stop_motor(void){
	servo_operate(3,0);
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN1,Bit_RESET);
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN2,Bit_RESET);
}
void backward_motor(void){
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN1,Bit_RESET);
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN2,Bit_SET);
	
}
#define THRESHOLD_DISTANCE 100

static unsigned char count4car=0;

void CarPolling(){
	unsigned char state=0x00;
	unsigned long temp1,temp2,temp3,temp4;
	
	if(fButton==1){
		temp1=Get_CH1Distance();
		temp2=Get_CH2Distance();
		temp3=Get_CH3Distance();
		temp4=Get_CH4Distance();
		if(temp1==0 || temp2==0)
			goto STOP_CAR;
		state|=((temp1>THRESHOLD_DISTANCE)?0x01:0x00);
		state|=((temp2>THRESHOLD_DISTANCE)?0x02:0x00);
		switch(state){
			case 0x00:
				//deadend
				servo_operate(10,0);
				backward_motor();
				GPIO_SetBits( GPIOD,GPIO_Pin_12);
				break;
			case 0x01:
				//block on left
				servo_operate(3,-15);
				forward_motor();
				GPIO_SetBits( GPIOD,GPIO_Pin_13);
				break;

			case 0x02:
				//block on left
				servo_operate(3,15);
				forward_motor();
				GPIO_SetBits( GPIOD,GPIO_Pin_14);
				break;

			case 0x03:
				//nothing ahead
				servo_operate(3,0);
				forward_motor();
				GPIO_SetBits( GPIOD,GPIO_Pin_15);
				break;

			
		}
STOP_CAR: count4car++;
		if(count4car>4){
			GPIO_ResetBits( GPIOD,GPIO_Pin_12);
			GPIO_ResetBits( GPIOD,GPIO_Pin_13);
			GPIO_ResetBits( GPIOD,GPIO_Pin_14);
			GPIO_ResetBits( GPIOD,GPIO_Pin_15);
			
			stop_motor();
			fButton=0;
		}
			
	}
	
}

void init_car(){
	init_motor();
	servo_init();
	ultra_sound_init();
	
	init_button();
	carTimers=xTimerCreate("Car",	 ( CAR_POLLING_PERIOD), pdTRUE, ( void * ) 1,  CarPolling	 );
	xTimerStart( carTimers, 0 );
}
void init_button(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable GPIO C clock. */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// Setup Blue & Green LED on STM32-Discovery Board to use PWM.
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_0 ;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	
	GPIO_Init( GPIOA, &GPIO_InitStruct ); 
	
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	EXTI_ClearITPendingBit(EXTI_Line0);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	 
}
void EXTI0_IRQHandler(){
	EXTI_ClearITPendingBit(EXTI_Line0);
	
	if(fButton==0){
		fButton=1;
		count4car=0;
	}
}


