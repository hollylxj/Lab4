/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "lab3.h"
#include "misc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "semphr.h"

// Duty cycle = 0xFFF/0xFFFF (the defined period)
#define CCR_Val1 (uint16_t) 0xFFF
#define CCR_Val2 (uint16_t) 0xFFF
#define CCR_Val3 (uint16_t) 0xFFF
#define CCR_Val4 (uint16_t) 0xFFF

void SetSysClock72(void);
void GPIO_Configuration(void);
void Timer3and4_Config(void);
void RCC_Configuration(void);
void ErrorLoop(void);

/*-------------task declaration------------*/
void DetectEmergency(void* pvParameters);
void RefreshSensorData(void* pvParameters);
void CalculateOrientation(void* pvParameters);
void UpdatePid(void* pvParameters);
void LogDebugInfo(void* pvParameters);
void Ledred(void* pvParameters);
void Ledgreen(void* pvParameters);

/*-------------structure defines-------------*/
GPIO_InitTypeDef GPIO_InitStructure; 
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
RCC_ClocksTypeDef RCC_ClockFreq;
ErrorStatus HSEStartUpStatus;

//declarations
uint16_t Prescaler= 0;
uint16_t PrescalerValue=0;
uint16_t initial = 0;
uint16_t CCR_M1,CCR_M2,CCR_M3,CCR_M4;
xSemaphoreHandle xsnSemaphore;
MotorSpeeds p_motorSpeeds;

int main(void)
{
SetSysClock72();
RCC_Configuration();
GPIO_Configuration();
Timer3and4_Config();

// create tasks
xTaskCreate(DetectEmergency, (portCHAR *) "DetectEmergency",configMINIMAL_STACK_SIZE,NULL,4,NULL);
xTaskCreate(RefreshSensorData, (portCHAR *)  "Res",configMINIMAL_STACK_SIZE,NULL,3,NULL);
xTaskCreate(CalculateOrientation, (portCHAR *)  "CalculateOrientation",configMINIMAL_STACK_SIZE,NULL,2,NULL);
xTaskCreate(UpdatePid, (portCHAR *)  "UpdatePid",configMINIMAL_STACK_SIZE,NULL,2,NULL);
xTaskCreate(LogDebugInfo, (portCHAR *)  "LogDebugInfo",configMINIMAL_STACK_SIZE,NULL,1,NULL);
xTaskCreate(Ledred, (portCHAR *)  "Ledred",configMINIMAL_STACK_SIZE,NULL,1,NULL);
xTaskCreate(Ledgreen, (portCHAR *)  "Ledgreen",configMINIMAL_STACK_SIZE,NULL,1,NULL);

vTaskStartScheduler(); // start the task scheduler

while(1)
{}

return 0;
}

void Timer3and4_Config(void)
{
/* set up timer base struct.
  /* Compute the prescaler value */
  // timer runs at 5MHz. timer frequency= sysclock/prescaler(the counting speed)
  PrescalerValue =72000000 / 0xFFFF;
  // time period: the number counted.(the period)
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);
  TIM_PrescalerConfig(TIM4, PrescalerValue, TIM_PSCReloadMode_Immediate);

  TIM_Cmd(TIM3, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = initial;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* PWM1 Mode configuration: Channel3 */
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
  
  /* PWM1 Mode configuration: Channel4 */
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
  
  /* PWM1 Mode configuration: Channel4 */
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM4, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM4, ENABLE);
}


void RCC_Configuration(void)
{
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
  // RCC_APB2Periph_AFIO is for blinking the red LED
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_AFIO, ENABLE);
}

void GPIO_Configuration(void)
{
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST,ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_WriteBit(GPIOB, GPIO_Pin_5, 0); 
  GPIO_WriteBit(GPIOB, GPIO_Pin_4, 0);

  /* GPIOB Configuration:TIM3,4 Channel3 and 4 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

}



void SetSysClock72(void)
{
  RCC_DeInit();
  RCC_HSEConfig(RCC_HSE_ON);
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if (HSEStartUpStatus != SUCCESS)
  {
  ErrorLoop();
  };
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    FLASH_SetLatency(FLASH_Latency_2);
    // PLLCLK = 8MHz * 9 = 72 MHz 
    RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);
    // Enable PLL 
    RCC_PLLCmd(ENABLE);
    // Wait till PLL is ready 
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){
    }
    // Select PLL as system clock source 
    RCC_SYSCLKConfig    (RCC_SYSCLKSource_PLLCLK);

    // Wait till PLL is used as system clock source 
    while(RCC_GetSYSCLKSource() != 0x08){
    }
      // HCLK = SYSCLK 
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
    // PCLK2 = HCLK 
    RCC_PCLK2Config(RCC_HCLK_Div1); 
    // PCLK1 = HCLK/2 
    RCC_PCLK1Config(RCC_HCLK_Div2);
}


 // Define tasks, in total there are 7 tasks (including blinking red and green LEDs) 
// once 10ms
void DetectEmergency(void* pvParameters){

while (1){
detectEmergency();
vTaskDelay(10);
  }
}

// once 100ms
void RefreshSensorData(void* pvParameters){
// create a binary semaphore for use
vSemaphoreCreateBinary(xsnSemaphore);
while (1){
refreshSensorData();
xSemaphoreGive(xsnSemaphore);  //allow this function to give semaphore 
vTaskDelay(100);
  }
}

//once 100ms
void CalculateOrientation(void* pvParameters){
while (1){
if( xsnSemaphore != NULL ){
// allow this function to take semaphore 
if(xSemaphoreTake(xsnSemaphore,(portTickType)100)==pdTRUE){ 
calculateOrientation();}
      }
   }
}

//once 1s
void UpdatePid(void* pvParameters){
while(1){

  updatePid(&p_motorSpeeds);
  
  CCR_M1=(uint16_t)(p_motorSpeeds.m1*CCR_Val1);
  CCR_M2=(uint16_t)(p_motorSpeeds.m2*CCR_Val2);
  CCR_M3=(uint16_t)(p_motorSpeeds.m3*CCR_Val3);
  CCR_M4=(uint16_t)(p_motorSpeeds.m4*CCR_Val4);
  
  TIM3->CCR4 = (uint16_t) CCR_M1;
  TIM3->CCR3 = (uint16_t) CCR_M2;
  TIM4->CCR4 = (uint16_t) CCR_M3;
  TIM4->CCR3 = (uint16_t) CCR_M4;
  vTaskDelay(1000);
}
}

// once 1s
void LogDebugInfo(void* pvParameters){
while(1){
logDebugInfo();
  vTaskDelay(1000);
}
}

//green led blink 0.5 Hz
void Ledgreen(void* pvParameters){

while(1){
  GPIO_WriteBit(GPIOB,GPIO_Pin_5,0);
  vTaskDelay(1000);
  GPIO_WriteBit(GPIOB,GPIO_Pin_5,1);
  vTaskDelay(1000);
}
}


// red led blink 0.25Hz
void Ledred(void* pvParameters){

while(1){
  GPIO_WriteBit(GPIOB,GPIO_Pin_4,0);
  vTaskDelay(2000);
  GPIO_WriteBit(GPIOB,GPIO_Pin_4,1);
  vTaskDelay(2000);
}
}

void ErrorLoop(void)
{
}



