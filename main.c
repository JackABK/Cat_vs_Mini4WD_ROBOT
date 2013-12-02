#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "stdio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_sdio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"
#include "misc.h"
void init_USART3(void);
void init_LED(void);
void init_SDcard(void);
void test_FPU_test(void* p);
void test_SDcard(void* p);
int main(void) {
  uint8_t ret = pdFALSE;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  init_USART3();
  init_LED();
  init_SDcard();
  ret = xTaskCreate(test_SDcard, "FPU", 512, NULL, 1, NULL);
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
void init_SDcard(){
	
	SDIO_InitTypeDef SDIO_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_SDIO);
	
	// Setup Blue & Green LED on STM32-Discovery Board to use PWM.
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9| GPIO_Pin_10| GPIO_Pin_11; //PD12->LED3 PD13->LED4 PD14->LED5 PDa5->LED6
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;            // Alt Function - Push Pull
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOC, &GPIO_InitStruct ); 
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2;
	GPIO_Init( GPIOD, &GPIO_InitStruct ); 
	
	SDIO_InitStruct.SDIO_BusWide=SDIO_BusWide_4b;
	SDIO_InitStruct.SDIO_ClockBypass=SDIO_ClockBypass_Disable;
	SDIO_InitStruct.SDIO_ClockDiv=0x00;
	SDIO_InitStruct.SDIO_ClockEdge=SDIO_ClockEdge_Rising;
	SDIO_InitStruct.SDIO_ClockPowerSave=SDIO_ClockPowerSave_Enable;
	SDIO_InitStruct.SDIO_HardwareFlowControl=SDIO_HardwareFlowControl_Enable;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO,ENABLE);
	SDIO_ClockCmd(ENABLE);
	SDIO_SetPowerState(SDIO_PowerState_ON);
	SDIO_Init(&SDIO_InitStruct);
}

void test_SDcard(void* p){
	SDIO_CmdInitTypeDef SDIO_CmdInitStruct;
	for(;;){
		SDIO_CmdStructInit(&SDIO_CmdInitStruct);
		SDIO_CmdInitStruct.SDIO_Response=SDIO_Response_Short;
		SDIO_CmdInitStruct.SDIO_CPSM=SDIO_CPSM_Enable;
		SDIO_CmdInitStruct.SDIO_Wait=SDIO_Wait_No;
		SDIO_SendCommand(&SDIO_CmdInitStruct);
		while(SDIO_GetFlagStatus(SDIO_FLAG_CMDACT)==SET);
		GPIO_ToggleBits( GPIOD,GPIO_Pin_12);
		vTaskDelay(1000);
	}
	vTaskDelete(NULL);
	
}
void test_FPU_test(void* p) {
  float ff = 1.0f;
  printf("Start FPU test task.\n");
  for (;;) {
    float s = sinf(ff);
    ff += s;
    // TODO some other test
	GPIO_ToggleBits( GPIOD,GPIO_Pin_13);
	vTaskDelay(1000);
  }
  vTaskDelete(NULL);
}
void init_LED(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	/* Enable GPIO C clock. */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
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
