/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_flash.c"
#include "misc.c"
#include "misc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_tim.c"
#include "lab3.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "semphr.h"

void SetSysClock72(void);
void GPIO_Configuration(void);
void TIM_Config(void);
void Timer3and4_Config(void);

void RCC_Configuration(void);
void ErrorLoop(void);

void DetectEmergency(void * Parameters);
void RefreshSensorData(void * Parameters);
void CalculateOrientation(void * Parameters);
void UpdatePid(void * Parameters);
void LogDebugInfo(void * Parameters);
void SwitchMotor(float M1,float M2,float M3, float M4);

// SET SYSTEM CLOCK
// DEFINE A GLOBAL VARIABLE OF TYPE GPIO_InitTypeDef
GPIO_InitTypeDef GPIO_InitStructure; 
// DEFINE A GLOBAL VARIABLE OF TYPE TIM_TimeBaseInitTypeDef
// for timer 2
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
// for timer 3 and 4

// DEFINE A GLOBAL VARIABLE OF TYPE TIM_OCInitTypeDef
TIM_OCInitTypeDef  TIM_OCInitStructure;

// DEFINE A GLOBAL VARIABLE OF TYPE NVIC_InitTypeDef
NVIC_InitTypeDef  NVIC_InitStructure;
// DEFINE A GLOBAL VARIABLE OF TYPE RCC_ClocksTypeDef
RCC_ClocksTypeDef RCC_ClockFreq;
// DEFINE A GLOBAL VARIABLE OF TYPE ErrorStatus
ErrorStatus HSEStartUpStatus;

int on = 0;
uint16_t Prescaler= 0;
uint16_t PrescalerValue=0;
uint16_t initial = 0;

#define CCR_Val1 (uint16_t) 0xFFF
#define CCR_Val2 (uint16_t) 0xFFF
#define CCR_Val3 (uint16_t) 0xFFF
#define CCR_Val4 (uint16_t) 0xFFF
#define motor_thrust 0.3

// DEFINE COUNTER VARIABLE FOR STATEMACHINE
int status = 0;
// DEFINE A VARIABLE FOR DEBUG POURPOSES
int debug;  
int count =0;

int main(void)
{
// INITIALIZE THE STATEMACHINE COUNTER TO BE ZERO
status = 0; 

SetSysClock72();
RCC_Configuration();
GPIO_Configuration();
Timer3and4_Config();
TIM_Config();// CONFIGURE TIMER TO GENERATE INTERRUPT

xTaskCreate(DetectEmergency,"FDetectEmergency",configMINIMAL_STACK_SIZE,NULL,0,NULL);
  xTaskCreate(RefreshSensorData,"FRefreshSensorData",configMINIMAL_STACK_SIZE,NULL,1,NULL);
  xTaskCreate(CalculateOrientation,"FCalculateOrientation",configMINIMAL_STACK_SIZE,NULL,1,NULL);
  xTaskCreate(UpdatePid,"FUpdatePid",configMINIMAL_STACK_SIZE,NULL,2,NULL);
  xTaskCreate(LogDebugInfo,"FLogDebugInfo",configMINIMAL_STACK_SIZE,NULL,4,NULL);


while(1)
{  }

}


void Timer3and4_Config(void)
{

/* set up timer base struct.
  /* Compute the prescaler value */
  // timer runs at 5MHz. timer frequency= sysclock/prescaler(the counting speed)
  PrescalerValue =72000000 / 0xFFFF;
  // time period: the number counted.(the period)
  // 5KHz= 5MHz/1000
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
  
// configure timer 4


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


void TIM_Config(void)
{ 
  // timer runs at 5MHz. timer frequency= sysclock/prescaler(the counting speed)
  Prescaler= (uint16_t) (72000000 / 0xFFFF);

  /* TIM2 configuration */
  // generate 1Hz interrupts
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
  TIM_TimeBaseStructure.TIM_Prescaler = Prescaler;

  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; /*update every time when counter overflows*/ 
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);
  
  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_Pulse = 0x0;  
  TIM_OC1Init(TIM2, &TIM_OCInitStructure);

TIM_PrescalerConfig(TIM2,Prescaler,TIM_PSCReloadMode_Immediate);

  TIM_ClearFlag(TIM2, TIM_FLAG_Update);


  /* Configure two bits for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  /* Enable the TIM2 Interrupt */
  //NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x0);
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  

  /* Enable TIM2 Update interrupts */
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
   	

  /* TIM2 enable counters */
  TIM_Cmd(TIM2, ENABLE);

}




void RCC_Configuration(void)
{
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
}

void GPIO_Configuration(void)
{
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST,ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_WriteBit(GPIOB, GPIO_Pin_5, 1); 
  GPIO_WriteBit(GPIOB, GPIO_Pin_4, 1);

  /* GPIOB Configuration:TIM3,4 Channel3 and 4 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

}



void SetSysClock72(void)
{
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/   
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if (HSEStartUpStatus != SUCCESS)
  {
  ErrorLoop();
  };
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
 


    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  
  
      /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);
}

void DetectEmergency(void * Parameters)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1){
detectEmergency();
vTaskDelay(10);
  }
}

static xSemaphoreHandle xSemaphore = NULL;
void RefreshSensorData(void * Parameters)
{
vSemaphoreCreateBinary(xSemaphore);
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1){
refreshSensorData();
xSemaphoreGive(xSemaphore);
vTaskDelay(100);
  }
}


void CalculateOrientation(void * Parameters)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1){
if(xSemaphore != NULL)
  {
if(xSemaphoreTake(xSemaphore, 0)==pdTRUE){
calculateOrientation();
}
}

  }
}


void UpdatePid(void * Parameters)
{
static float M1=0,M2=0,M3=0,M4=0;
static MotorSpeeds p_motorSpeeds;

while(1){
  vTaskDelay(1000);
  updatePid(&p_motorSpeeds);
  if(p_motorSpeeds.m1) M1=1;
  else M1=0;
  if(p_motorSpeeds.m2) M2=1;
  else M2=0;
  if(p_motorSpeeds.m3) M3=1;
  else M3=0;
  if(p_motorSpeeds.m4) M4=1;
  else M4=0;

  SwitchMotor(M1,M2,M3,M4);
}
}


void LogDebugInfo(void * Parameters)
{
while(1){
logDebugInfo();
vTaskDelay(1000);
}
}


void vApplicationStackOverflowHook (xTaskHandle *pxTask, signed portCHAR *pcTaskName)
{
    (void) pxTask;
    (void) pcTaskName;

}


void SwitchMotor(float M1,float M2,float M3, float M4)
{

  uint16_t CCR_M1,CCR_M2,CCR_M3,CCR_M4;
  CCR_M1=(uint16_t)(M1*0xFFF);
  CCR_M2=(uint16_t)(M2*0xFFF);
  CCR_M3=(uint16_t)(M3*0xFFF);
  CCR_M4=(uint16_t)(M4*0xFFF);
  
  TIM3->CCR4 =(uint16_t) CCR_M1;
  
  TIM3->CCR3 = (uint16_t) CCR_M2;
 
  TIM4->CCR4 = (uint16_t) CCR_M3;
  
  TIM4->CCR3 = (uint16_t) CCR_M4;
}


void ErrorLoop(void)
{
debug=4;
}

