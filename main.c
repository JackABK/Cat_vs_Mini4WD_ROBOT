/*Reference*/
/*https://github.com/Torrentula/STM32F4-examples/blob/master/USART/main.c*/


#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "example.h"
#include "car_behavior.h"

#define USE_FILELIB_STDIO_COMPAT_NAMES

/*Setting the USART MAX string lenth */
#define MAX_STRLEN 12 // this is the maximum string length of our string in characters
volatile char received_string[MAX_STRLEN+1]; // this will hold the recieved string

/*global USART receive command*/
char Receive_Command = 0;
/*can accept the remote control from the android smart phone*/
uint8_t USE_REMOTE_CONTROL;




 

void init_USART3(void);
void init_LED(void);
void test_FPU_test(void* p);
void motor_test(void* p);

void remote_task(void *p);
void USART_puts(USART_TypeDef* USARTx, volatile char *s);




int main(void) {
	uint8_t ret = pdFALSE;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    
	init_USART3();
	init_USART1(9600);
	init_LED();
         //init_car();

	ret = xTaskCreate(test_FPU_test, "FPU", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    
        ret = xTaskCreate(remote_task, "remote task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

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

   USART_InitStruct.USART_BaudRate = 115200; 
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


void remote_task(void *p)                                                                                                                                                                                  
{
		while (1) {
				if(Receive_Command > 0 ){
						/* Receive a byte from the RS232 port (this call will                                                                                                                                                    
						 * block). */      

						        switch(Receive_Command){
                                                                      case 'A':
										//Mode A   -->  remote control mode
										/*reset the system*/
										NVIC_SystemReset();              
										break;
                                                                       case 'B':
										//Mode B   --> auto-avoidance mode
										/*enable the auto-avoidance*/
										 init_car();             
										break;
								case 'f':
										//forward
										forward_motor();               
										break;
								case 'b':
										//back 
										backward_motor();              
										break;
                                                                     case 'l':
										//left 
										left_motor();              
										break;
                                                                     case 'r':
										//right
										right_motor();              
										break;
                                                                     case 'N':
                                                                                    //left _forward
                                                                                     left_forward_motor();              
                                                                                     break;
                                                                    case 'E':
                                                                                    //right_forward
                                                                                    right_forward_motor();              
                                                                                    break;
                                                                    case 'W':
                                                                                    //left_backward
                                                                                    left_backward_motor();              
                                                                                    break;
                                                                   case 'S':
                                                                                    //right_backward
                                                                                    right_backward_motor();              
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


void init_USART1(uint32_t baudrate){
        
        /* This is a concept that has to do with the libraries provided by ST
         * to make development easier the have made up something similar to 
         * classes, called TypeDefs, which actually just define the common
         * parameters that every peripheral needs to work correctly
         * 
         * They make our life easier because we don't have to mess around with 
         * the low level stuff of setting bits in the correct registers
         */
        GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
        USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
        NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
        
        /* enable APB2 peripheral clock for USART1 
         * note that only USART1 and USART6 are connected to APB2
         * the other USARTs are connected to APB1
         */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        
        /* enable the peripheral clock for the pins used by 
         * USART1, PB6 for TX and PB7 for RX
         */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
        
        /* This sequence sets up the TX and RX pins 
         * so they work correctly with the USART1 peripheral
         */
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;                         // the pins are configured as alternate function so the USART peripheral has access to them
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;                // this defines the IO speed and has nothing to do with the baudrate!
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;                        // this defines the output type as push pull mode (as opposed to open drain)
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;                        // this activates the pullup resistors on the IO pins
        GPIO_Init(GPIOB, &GPIO_InitStruct);                                        // now all the values are passed to the GPIO_Init() function which sets the GPIO registers
        
        /* The RX and TX pins are now connected to their AF
         * so that the USART1 can take over control of the 
         * pins
         */
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
        
        /* Now the USART_InitStruct is used to define the 
         * properties of USART1 
         */
        USART_InitStruct.USART_BaudRate = baudrate;                                // the baudrate is set to the value we passed into this init function
        USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
        USART_InitStruct.USART_StopBits = USART_StopBits_1;                // we want 1 stop bit (standard)
        USART_InitStruct.USART_Parity = USART_Parity_No;                // we don't want a parity bit (standard)
        USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
        USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
        USART_Init(USART1, &USART_InitStruct);                                        // again all the properties are passed to the USART_Init function which takes care of all the bit setting
        
        
        /* Here the USART1 receive interrupt is enabled
         * and the interrupt controller is configured 
         * to jump to the USART1_IRQHandler() function
         * if the USART1 receive interrupt occurs
         */
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt 
        
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;                 // we want to configure the USART1 interrupts
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                 // this sets the subpriority inside the group
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                         // the USART1 interrupts are globally enabled
        NVIC_Init(&NVIC_InitStructure);                                                         // the properties are passed to the NVIC_Init function which takes care of the low level stuff        

        // finally this enables the complete USART1 peripheral
        USART_Cmd(USART1, ENABLE);
}








// this is the interrupt request handler (IRQ) for ALL USART1 interrupts
void USART1_IRQHandler(void){
        
        // check if the USART1 receive interrupt flag was set
        if( USART_GetITStatus(USART1, USART_IT_RXNE) ){
                
                static uint8_t cnt = 0; // this counter is used to determine the string length
                Receive_Command = USART1->DR; // the character from the USART1 data register is saved in t
                
                /* check if the received character is not the LF character (used to determine end of string) 
                 * or the if the maximum string length has been been reached 
                 */
                if( (Receive_Command != '\n') && (cnt < MAX_STRLEN) ){ 
                        received_string[cnt] = Receive_Command;
                        cnt++;
                }
                else{ // otherwise reset the character counter and print the received string
                        cnt = 0;
                        USART_puts(USART1, received_string);
                }
        }
}


void USART_puts(USART_TypeDef* USARTx, volatile char *s)
{
        while(*s){
                // wait until data register is empty
                while( !(USARTx->SR & 0x00000040) ); 
                USART_SendData(USARTx, *s);
                *s++;
        }
}





