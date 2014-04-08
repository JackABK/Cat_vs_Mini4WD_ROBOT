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

#define CAR_POLLING_PERIOD 20 //unit:ms

//for calibration
#define SERVO_LEFT_DEGREE  -14
#define SERVO_CENTRAL_DEGREE 6
#define SERVO_RIGHT_DEGREE    26



xTimerHandle carTimers;



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
inline void forward_motor(void){
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN1,Bit_SET);
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN2,Bit_RESET);
	
}
void stop_motor(){
	servo_operate(3,SERVO_CENTRAL_DEGREE);
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN1,Bit_RESET);
	GPIO_WriteBit(MOTOR_INPUT_PORT,MOTOR_INPUT_PIN2,Bit_RESET);
}
inline void backward_motor(){
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

void straight_forward_motor(){
	servo_operate(3,SERVO_CENTRAL_DEGREE);
	forward_motor();
}
void straight_backward_motor(){
	servo_operate(3,SERVO_CENTRAL_DEGREE);
	backward_motor();
}

void left_forward_motor(){
        /*forward and turn right small angle*/
        servo_operate(3,SERVO_LEFT_DEGREE);
	forward_motor();
}

void right_forward_motor(){
        /*forward and turn right small angle*/
	servo_operate(3,SERVO_RIGHT_DEGREE);
	forward_motor();
}


void right_backward_motor(){
        /*forward and turn right small angle*/
	servo_operate(3,SERVO_RIGHT_DEGREE);
	backward_motor();
}

void left_backward_motor(){
        /*forward and turn right small angle*/
        servo_operate(3,SERVO_LEFT_DEGREE);
	backward_motor();
}







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
car_state_t car_state=CAR_STATE_IDLE;

inline unsigned char ReadPIRButton(){
	unsigned char temp;
	temp=(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)<<1  )
		|GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1);
	return temp;
}
inline unsigned char IsFrontButtonAct(unsigned char button){
	return button&0x01;
}
inline unsigned char IsBackButtonAct(unsigned char button){
	return button&0x02;
}



#define PIR_DEBOUNCE_NUM 10
#define CAR_MOVING_PERIOD 40
#define CAR_REST_PERIOD 50

#define THRESHOLD_DISTANCE 100

void CarPolling(){
	unsigned char fButton,state=0;
	unsigned long distance[4];
	static int count4front,count4back,count;
	if(car_state==CAR_STATE_IDLE){
		fButton=ReadPIRButton();
		if(IsFrontButtonAct(fButton))
			count4front++;
		else
			count4front=0;
		if(IsBackButtonAct(fButton))
			count4back++;
		else
			count4back=0;

		if(count4front>=PIR_DEBOUNCE_NUM)
			car_state|=CAR_STATE_MOVE_BACK;
		if(count4back>=PIR_DEBOUNCE_NUM)
			car_state|=CAR_STATE_MOVE_FORWARD;
		if(count4front+count4back>=40)
			car_state=CAR_STATE_MOVE_BOTH;
	}
	else if(car_state==CAR_STATE_REST){
		stop_motor();
		count++;
		if(count>=CAR_REST_PERIOD){
			count=0;
			car_state=CAR_STATE_IDLE;
		}
	}
	else if(car_state==CAR_STATE_MOVING){
		count++;
		if(count>=CAR_MOVING_PERIOD){
			count=0;
			car_state=CAR_STATE_REST;
		}
			
	}
	else{
		distance[0]=Get_CH1Distance();
		distance[1]=Get_CH2Distance();
		distance[2]=Get_CH3Distance();
		distance[3]=Get_CH4Distance();
		
		if(car_state==CAR_STATE_MOVE_FORWARD){
			if(distance[0]==0 || distance[1]==0){
				car_state=CAR_STATE_IDLE;
				return;
			}
			state|=((distance[0]>THRESHOLD_DISTANCE)?0x01:0x00);
			state|=((distance[1]>THRESHOLD_DISTANCE)?0x02:0x00);
			switch(state){
				case 0x00:
					//deadend
                          	        if(distance[2]>distance[3])
						left_backward_motor();
					else
						right_backward_motor();
					break;
				case 0x01:
					//need to turn left
					left_forward_motor();
					break;
				case 0x02:
					//need to turn right
					right_forward_motor();
					break;
				case 0x03:
					//nothing ahead
					straight_forward_motor();
					break;

			
			}
		}
		else if(car_state==CAR_STATE_MOVE_BACK){
			if(distance[2]==0 || distance[3]==0){
				car_state=CAR_STATE_IDLE;
				return;
			}
			state|=((distance[2]>THRESHOLD_DISTANCE)?0x01:0x00);
			state|=((distance[3]>THRESHOLD_DISTANCE)?0x02:0x00);
			switch(state){
				case 0x00:
					//deadend
				
					if(distance[0]>distance[1])
						left_forward_motor();
					else
						right_forward_motor();
					break;
				case 0x01:
					
					left_backward_motor();
					break;

				case 0x02:
					right_backward_motor();
					break;

				case 0x03:
					//nothing ahead
					straight_backward_motor();
					break;

			
			}
		}
		else if(car_state==CAR_STATE_MOVE_BOTH){
			switch( search_maximum_index(distance,4)){
				case 0://left forward
					left_forward_motor();
					break;
				case 1://right forward
					right_forward_motor();
					break;
				case 2://left backward
					left_backward_motor();
					break;
				case 3://right backward
					right_backward_motor();
					break;
			}

		}
		car_state=CAR_STATE_MOVING;
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

    
        
}










