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

#define CAR_POLLING_PERIOD 50 //unit:ms

#define LEFT_DEGREE  -4
#define CENTRAL_DEGREE 6
#define RIGHT_DEGREE    16


unsigned char fButtonFront=0,fButtonBack=0;
xTimerHandle carTimers;
xTimerHandle PIRTimers_Front;
xTimerHandle PIRTimers_Back;
xTimerHandle PIR_Timers;


static unsigned char state_motor_test=0;
static unsigned char Ultrasonic_Detected_Finished=1;


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
	servo_operate(3,CENTRAL_DEGREE);
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






#define THRESHOLD_DISTANCE 50

static unsigned char count4car=0;

int search_maximum_index(unsigned long* arr,int length){
    unsigned long maxi=0;
    int idx,i;
    for(i=0;i<length;i++){
        if(maxi<arr[i]){
              idx=i;
              maxi=arr[i];
        }
    }
    return idx;
}
void CarPolling(){
	unsigned char state=0x00;
	unsigned long temp[4];

        fButtonBack = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)  ;
        fButtonFront= GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)  ;
          

    
	if(fButtonBack==1 && fButtonFront==0){
                Ultrasonic_Detected_Finished = 0;
		temp[0]=Get_CH1Distance();
		temp[1]=Get_CH2Distance();
		temp[2]=Get_CH3Distance();
		temp[3]=Get_CH4Distance();
		if(temp[0]==0 || temp[1]==0)
			goto STOP_CAR;
		state|=((temp[0]>THRESHOLD_DISTANCE)?0x01:0x00);
		state|=((temp[1]>THRESHOLD_DISTANCE)?0x02:0x00);
		switch(state){
			case 0x00:
				//deadend
                                  if(temp[2]>temp[3])
                                    servo_operate(3,LEFT_DEGREE);
                                  else
                                    servo_operate(3,RIGHT_DEGREE);

                                  backward_motor();
				break;
			case 0x01:
				//need to turn left
				servo_operate(3,LEFT_DEGREE);
				forward_motor();
				break;

			case 0x02:
				//need to turn right
				servo_operate(3,RIGHT_DEGREE);
				forward_motor();
				break;

			case 0x03:
				//nothing ahead
				servo_operate(3,CENTRAL_DEGREE);
				forward_motor();
				break;

			
		}
STOP_CAR: count4car++;
		if(count4car==19){
			GPIO_ResetBits( GPIOD,GPIO_Pin_12);
			GPIO_ResetBits( GPIOD,GPIO_Pin_13);
			GPIO_ResetBits( GPIOD,GPIO_Pin_14);
			GPIO_ResetBits( GPIOD,GPIO_Pin_15);
			
			stop_motor();
			fButtonBack=0;
                          Ultrasonic_Detected_Finished = 1;
		}
			
	}
        else if(fButtonFront==1 && fButtonBack==0){
                   Ultrasonic_Detected_Finished= 0;
		temp[0]=Get_CH1Distance();
		temp[1]=Get_CH2Distance();
		temp[2]=Get_CH3Distance();
		temp[3]=Get_CH4Distance();
		if(temp[2]==0 || temp[3]==0)
			goto STOP_CAR1;
		state|=((temp[2]>THRESHOLD_DISTANCE)?0x01:0x00);
		state|=((temp[3]>THRESHOLD_DISTANCE)?0x02:0x00);
		switch(state){
			case 0x00:
				//deadend
				
                                    if(temp[0]>temp[1])
                                         servo_operate(3,LEFT_DEGREE);
                                      else
                                          servo_operate(3,RIGHT_DEGREE);

                                      forward_motor();
				break;
			case 0x01:
				//turn left
				servo_operate(3,LEFT_DEGREE);
				backward_motor();
				break;

			case 0x02:
				//turn right
				servo_operate(3,RIGHT_DEGREE);
				backward_motor();
				break;

			case 0x03:
				//nothing ahead
				servo_operate(3,CENTRAL_DEGREE);
				backward_motor();
				break;

			
		}
STOP_CAR1: count4car++;
		if(count4car==19){
	        		GPIO_ResetBits( GPIOD,GPIO_Pin_12);
			GPIO_ResetBits( GPIOD,GPIO_Pin_13);
			GPIO_ResetBits( GPIOD,GPIO_Pin_14);
			GPIO_ResetBits( GPIOD,GPIO_Pin_15);
			stop_motor();
			fButtonFront=0;
                          Ultrasonic_Detected_Finished = 1;
		}
			
	}
        else if(fButtonFront==1&&fButtonBack==1){
                    temp[0]=Get_CH1Distance();
		temp[1]=Get_CH2Distance();
		temp[2]=Get_CH3Distance();
		temp[3]=Get_CH4Distance();
                switch( search_maximum_index(temp,4)){
                    case 0://left forward
                                servo_operate(3,LEFT_DEGREE);
		            	forward_motor();
                              break;
                      case 1://right forward
                                servo_operate(3,RIGHT_DEGREE);
		            	forward_motor();
                              break;
                      case 2://left backward
                                servo_operate(3,LEFT_DEGREE);
		            	backward_motor();
                              break;
                      case 3://right backward
                                servo_operate(3,RIGHT_DEGREE);
		            	backward_motor();
                              break;
                     

                }
                count4car++;
		if(count4car==19){
			
			stop_motor();
		}
        }
	
}





void PIR_Polling()
{
        unsigned char Direction_state=0x00;
        uint8_t PIR_Back_Bit; 
        uint8_t PIR_Front_Bit;
        
    if(Ultrasonic_Detected_Finished){

         PIR_Back_Bit = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)  ;
         PIR_Front_Bit = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)  ;
        
        Direction_state|=((PIR_Back_Bit)?0x01:0x00);
        Direction_state|=((PIR_Front_Bit)?0x02:0x00);

        switch(Direction_state){
                case 0x00:
                    //Nothing detected

       
                                    break;
                    case 0x01:
                        //PIR back detected
                GPIO_SetBits( GPIOD,GPIO_Pin_12);
                GPIO_SetBits( GPIOD,GPIO_Pin_13);
                GPIO_SetBits( GPIOD,GPIO_Pin_14);
                GPIO_SetBits( GPIOD,GPIO_Pin_15);
                     if(fButtonBack==0){
                         fButtonBack=1;
                         count4car=0;
                     }
                                    break;

                        case 0x02:

                            //PIR front detected
                                                  GPIO_ResetBits( GPIOD,GPIO_Pin_12);
                                                    GPIO_ResetBits( GPIOD,GPIO_Pin_13);
                                                  GPIO_ResetBits( GPIOD,GPIO_Pin_14);
                                                  GPIO_ResetBits( GPIOD,GPIO_Pin_15);

                   if(fButtonFront==0){
                   fButtonFront=1;
                   count4car=0;
                    }
                                                        break;

        case 0x03:
            //PIR front  and back detected
            GPIO_SetBits( GPIOD,GPIO_Pin_12);
                GPIO_SetBits( GPIOD,GPIO_Pin_13);
                GPIO_SetBits( GPIOD,GPIO_Pin_14);
                GPIO_SetBits( GPIOD,GPIO_Pin_15);
                      if(fButtonFront==0){
                   fButtonFront=1;
                   count4car=0;
                    }
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

    #if 0
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
    #endif
    

    /*using the polling method to trigger the PIR sensors*/

  //  PIR_Timers=xTimerCreate("PIRTimers", ( 100), pdTRUE, ( void * ) 1,  PIR_Polling);
   //  xTimerStart( PIR_Timers, 0 );
        
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






