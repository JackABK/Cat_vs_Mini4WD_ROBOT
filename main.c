#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"

#include "example.h"
#include "servo_motor.h"


#define USE_FILELIB_STDIO_COMPAT_NAMES


void init_USART3(void);
void init_LED(void);

void forward_motor(void);
void backward_motor(void);
void stop_motor(void);

void test_FPU_test(void* p);
void motor_test(void* p);


int main(void) {
	uint8_t ret = pdFALSE;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	init_USART3();
	init_LED();
	init_motor();
 	servo_init();
	ultra_sound_init();


	
		
	//init_Button();
	
	xTaskCreate(motor_test, "motor", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	ret = xTaskCreate(test_FPU_test, "FPU", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	if (ret == pdTRUE) {
		printf("System Started!\n");
		vTaskStartScheduler();  // should never return
	} else {
		printf("System Error!\n");
		// --TODO blink some LEDs to indicates fatal system error
	}

	for (;;);
}

void vApplicationTickHook(void) {
}

/* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created.  It is also called by various parts of the
   demo application.  If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
void vApplicationMallocFailedHook(void) {
	taskDISABLE_INTERRUPTS();
	for(;;);
}

/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
   task.  It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()).  If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
void vApplicationIdleHook(void) {
}

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName) {
  (void) pcTaskName;
  (void) pxTask;
  /* Run time stack overflow checking is performed if
     configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
     function is called if a stack overflow is detected. */
  taskDISABLE_INTERRUPTS();
  for(;;);
}


unsigned char fTest=0;

static unsigned char state_motor_test=0;
void motor_test(void* p){
	
	vTaskDelay(1000);
	servo_operate(3,0);
	vTaskDelay(1000);
		
	
	while(1){
		switch(state_motor_test){
			case 0:
				//forward left
				servo_operate(3,-30);
				forward_motor();
				state_motor_test=1;
				
				break;
			case 1:
				//back right
				servo_operate(3,30);
				backward_motor();
				state_motor_test=2;
				break;
			case 2:
				//forward right
				servo_operate(3,30);
				forward_motor();
				state_motor_test=3;
				break;

			case 3:
				//back left
				servo_operate(3,-30);
				backward_motor();
				state_motor_test=0;
				break;
		}
		vTaskDelay(1500);
		stop_motor();
		vTaskDelay(1500);
	}
}
void test_FPU_test(void* p) {
  float ff = 1.0f;
  printf("Start FPU test task.\n");
  for (;;) {
    float s = sinf(ff);
    ff += s;
    // TODO some other test
	GPIO_ToggleBits( GPIOD,GPIO_Pin_13);
	//printf("%ld\n",Get_Distance() );
#if (0)
	if(fTest){
		servo_operate(2,30);
	}
	else{
		servo_operate(2,-30);
	}
	fTest^=1;
#endif
    vTaskDelay(1000);
  }
  vTaskDelete(NULL);
}
#if (0)
void init_Button(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable GPIO C clock. */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// Setup Blue & Green LED on STM32-Discovery Board to use PWM.
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_0 ;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;            // Alt Function - Push Pull
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOA, &GPIO_InitStruct ); 
	
	
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
	EXTI_ClearITPendingBit(EXTI_Line0);
	GPIO_ToggleBits( GPIOD,GPIO_Pin_14);
}
#endif

#define MOTOR_INPUT_PORT GPIOA
#define MOTOR_INPUT_PIN1 GPIO_Pin_2
#define MOTOR_INPUT_PIN2 GPIO_Pin_3

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

void init_LED(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	/* Enable GPIO C clock. */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE);
	// Setup Blue & Green LED on STM32-Discovery Board to use PWM.
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15; //PD12->LED3 PD13->LED4 PD14->LED5 PDa5->LED6
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;            // Alt Function - Push Pull
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOD, &GPIO_InitStruct ); 
	GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_RESET);
	GPIO_WriteBit(GPIOD,GPIO_Pin_13,Bit_RESET);
	GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_RESET);
	GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_RESET);
}
/*
 * Configure USART3(PB10, PB11) to redirect printf data to host PC.
 */
void init_USART3(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  USART_InitTypeDef USART_InitStruct;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

  USART_InitStruct.USART_BaudRate = 115200;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART3, &USART_InitStruct);
  USART_Cmd(USART3, ENABLE);
}
