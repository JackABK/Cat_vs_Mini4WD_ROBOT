#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"

#include "example.h"

#include "car_behavior.h"

#define USE_FILELIB_STDIO_COMPAT_NAMES

/*can accept the remote control from the android smart phone*/
#if 0 
#define USE_REMOTE_CONTROL
#endif 

void init_USART3(void);
void init_LED(void);
void test_FPU_test(void* p);
void motor_test(void* p);

#ifdef USE_REMOTE_CONTROL
void remote_task(void *p);
#endif 

/*global USART receive command*/
char Receive_Command;

int main(void) {
	uint8_t ret = pdFALSE;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	init_USART3();
	init_LED();
        init_car();

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

void test_FPU_test(void* p) {
  printf("Start FPU test task.\n");
  for (;;) {
  
	printf("%ld %ld %ld %ld\n\r",Get_CH1Distance(),Get_CH2Distance() ,Get_CH3Distance(),Get_CH4Distance() );

    vTaskDelay(1000);
  }
  vTaskDelete(NULL);
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


#ifdef USE_REMOTE_CONTROL
    USART_InitStruct.USART_BaudRate = 9600;  /*connecting from bluetooth receiver , both of baudrates are 9600.*/
#else
    USART_InitStruct.USART_BaudRate = 115200; 
#endif

  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART3, &USART_InitStruct);
  USART_Cmd(USART3, ENABLE);
}
void enable_USART3_interrupts(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable transmit and receive interrupts for the USART3. */
    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    /* Enable the USART3 IRQ in the NVIC module (so that the USART3 interrupt
     * handler is enabled). */                                                                                                                                                                                       
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
void USART3_IRQHandler()
{
   
   
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
		/* Receive the byte from the buffer. */
        Receive_Command = USART_ReceiveData(USART3);
    }
    else {
        /* Only transmit and receive interrupts should be enabled.
         * If this is another type of interrupt, freeze.
         */
        while(1);
    }
}


#ifdef USE_REMOTE_CONTROL
#define BACKSPACE (127)
void remote_task(void *p)                                                                                                                                                                                  
{

		char str[30];  
		char *argv[20];  
		char ch;
		int curr_char ,argc;      
		int done;                 
		char immediate_ch[2];

		printf("Hello this is the stm32f4\n");

		while (1) {
				curr_char = 0;        
				done = 0;             

				if(Receive_Command > 0 ){
						/* Receive a byte from the RS232 port (this call will                                                                                                                                                    
						 * block). */      
						switch(Receive_Command){          
								case 'f':
										//forward
										forward_motor();               
										break;
								case 'b':
										//back 
										backward_motor();              
										break;
								case 's':
										//stop
										stop_motor();              
										break;
								case '0':
										//-45 angle
										servo_operate(3,-45);
										break;
								case '1':
										//-30 angle
										servo_operate(3,-30);
										break;
								case '2':
										//-15 angle
										servo_operate(3,-15);
										break;
								case '3':
										//0 angle
										servo_operate(3,0);
										break;
								case '4':
										//15 angle
										servo_operate(3,15);
										break;
								case '5':
										//30 angle
										servo_operate(3,30);
										break;
								case '6':
										//45 angle
										servo_operate(3,45);
										break;
						}

						/*reset the command ,let it cannot re-doing*/
						Receive_Command = 0;
				}
		} 

}
#endif
