#include "FreeRTOS.h"
#include "timers.h"

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_syscfg.h"


#include "car_behavior.h"
#include "servo_motor.h"
#include "ultrasound.h"


#include "timers.h"


#define MOTOR_INPUT_PORT GPIOA
#define MOTOR_INPUT_PIN1 GPIO_Pin_2
#define MOTOR_INPUT_PIN2 GPIO_Pin_3

#define CAR_POLLING_PERIOD 200 //unit:ms

unsigned char fButtonFront=0,fButtonBack=0;
xTimerHandle carTimers;
static unsigned char state_motor_test=0;

void init_button(void);


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
void left_motor(void){
        /*forward and turn left big angle*/
        servo_operate(3,-12);
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN1,Bit_SET);
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN2,Bit_RESET);
}
void right_motor(void){
        /*forward and turn right big angle*/
        servo_operate(3,12);
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN1,Bit_SET);
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN2,Bit_RESET);
}
void right_forward_motor(void){
        /*forward and turn right small angle*/
        servo_operate(3,7);
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN1,Bit_SET);
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN2,Bit_RESET);
}
void left_forward_motor(void){
        /*forward and turn right small angle*/
        servo_operate(3,-7);
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN1,Bit_SET);
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN2,Bit_RESET);
}

void right_backward_motor(void){
        /*forward and turn right small angle*/
        servo_operate(3,7);
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN1,Bit_RESET);
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN2,Bit_SET);
}

void left_backward_motor(void){
        /*forward and turn right small angle*/
        servo_operate(3,-7);
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN1,Bit_RESET);
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN2,Bit_SET);
}






#define THRESHOLD_DISTANCE 100

static unsigned char count4car=0;

void CarPolling(){
	unsigned char state=0x00;
	unsigned long temp1,temp2,temp3,temp4;
	
	if(fButtonBack==1){
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
				backward_motor();
                                  if(temp3>temp4)
                                    servo_operate(3,-7);
                                  else
                                    servo_operate(3,7);
				break;
			case 0x01:
				//need to turn left
				servo_operate(3,-7);
				forward_motor();
				break;

			case 0x02:
				//need to turn right
				servo_operate(3,7);
				forward_motor();
				break;

			case 0x03:
				//nothing ahead
				servo_operate(3,0);
				forward_motor();
				break;

			
		}
STOP_CAR: count4car++;
		if(count4car>4){
			GPIO_ResetBits( GPIOD,GPIO_Pin_12);
			GPIO_ResetBits( GPIOD,GPIO_Pin_13);
			GPIO_ResetBits( GPIOD,GPIO_Pin_14);
			GPIO_ResetBits( GPIOD,GPIO_Pin_15);
			
			stop_motor();
			fButtonBack=0;
		}
			
	}
        else if(fButtonFront==1){
		temp1=Get_CH1Distance();
		temp2=Get_CH2Distance();
		temp3=Get_CH3Distance();
		temp4=Get_CH4Distance();
		if(temp3==0 || temp4==0)
			goto STOP_CAR1;
		state|=((temp3>THRESHOLD_DISTANCE)?0x01:0x00);
		state|=((temp4>THRESHOLD_DISTANCE)?0x02:0x00);
		switch(state){
			case 0x00:
				//deadend
				
				forward_motor();
                                    if(temp1>temp2)
                                         servo_operate(3,-7);
                                      else
                                          servo_operate(3,7);
				break;
			case 0x01:
				//turn left
				servo_operate(3,-7);
				backward_motor();
				break;

			case 0x02:
				//turn right
				servo_operate(3,7);
				backward_motor();
				break;

			case 0x03:
				//nothing ahead
				servo_operate(3,0);
				backward_motor();
				break;

			
		}
STOP_CAR1: count4car++;
		if(count4car>4){
	        		GPIO_ResetBits( GPIOD,GPIO_Pin_12);
			GPIO_ResetBits( GPIOD,GPIO_Pin_13);
			GPIO_ResetBits( GPIOD,GPIO_Pin_14);
			GPIO_ResetBits( GPIOD,GPIO_Pin_15);
			stop_motor();
			fButtonFront=0;
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
	/* Enable GPIO A clock. */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// Setup Blue & Green LED on STM32-Discovery Board to use PWM.
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_0 ;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init( GPIOA, &GPIO_InitStruct ); 
    
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init( GPIOA, &GPIO_InitStruct );

    
    /* Connect EXTI Line0 to PA0 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);
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


    
    /* Connect EXTI Line1 to PA1 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource1);
	EXTI_InitStruct.EXTI_Line = EXTI_Line1;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	EXTI_ClearITPendingBit(EXTI_Line1);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


    

void EXTI0_IRQHandler(){

    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
            GPIO_SetBits( GPIOD,GPIO_Pin_12);
           GPIO_SetBits( GPIOD,GPIO_Pin_13);
           GPIO_SetBits( GPIOD,GPIO_Pin_14);
           GPIO_SetBits( GPIOD,GPIO_Pin_15);

	if(fButtonBack==0){
		fButtonBack=1;
		count4car=0;
	}
	EXTI_ClearITPendingBit(EXTI_Line0);

    }
}

void EXTI1_IRQHandler(){


      if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {
	
            GPIO_ResetBits( GPIOD,GPIO_Pin_12);
           GPIO_ResetBits( GPIOD,GPIO_Pin_13);
           GPIO_ResetBits( GPIOD,GPIO_Pin_14);
           GPIO_ResetBits( GPIOD,GPIO_Pin_15);

        if(fButtonFront==0){
		fButtonFront=1;
		count4car=0;
	}

        EXTI_ClearITPendingBit(EXTI_Line1);

      }
}


void close_CarPolloing(void)
{
    xTimerDelete(carTimers,0);
}






