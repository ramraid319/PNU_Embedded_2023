#include <stdbool.h>
#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "core_cm3.h"
#include "misc.h"
#include "lcd.h"
#include "touch.h"
#include "stm32f10x_dma.h"
#include "misc.h"
#include "bluetooth.h"

int color[12] = {WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY};

/*
0 : IDLE (Wait Coin)
1 : Game Start
2 : Game Pause
3 : Game Over
*/
int gameStatus = 0;

// coin count
int coinCount = 0;

/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void USART2_Init(void);
void USART2_IRQHandler();
void NVIC_Configure(void);
void TIM2_Init(void);
void TIM3_Init(void);
void TIM2_IRQHandler();
void TIM3_IRQHandler();


void RCC_Configure(void)
{  
/* TIM2, TIM3 clock enable */  
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

/* Alternate Function IO clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
        
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure1;
    GPIO_InitTypeDef GPIO_InitStructure2;
    GPIO_InitTypeDef GPIO_InitStructure3;
    GPIO_InitTypeDef GPIO_InitStructure4;

    /* USART1 pin setting */
    // UART pin TX GPIO_Pin_9, UART pin RX GPIO_Pin_10
   
    //TX a9
    GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure1);
   
    //RX a10
    GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IPD | GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure2);
   
    /* USART2 pin setting */
   
    //TX a2
    GPIO_InitStructure3.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure3.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure3.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure3);
   
    //RX a3
    GPIO_InitStructure4.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure4.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure4.GPIO_Mode = GPIO_Mode_IPD | GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure4);
    
    // 3 Servo Motors   OUT
    // 3 Crash Sensors  IN
    // 1 Coin Sensors   IN
    // 3 LED            OUT
}

void TIM2_Init(void) {
/*
  TIM_TimeBaseInitTypeDef TIM2_InitStructure;

  //led timer
  TIM2_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM2_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM2_InitStructure.TIM_Period = 10000;
  TIM2_InitStructure.TIM_Prescaler = 7200;

  TIM_TimeBaseInit(TIM2, &TIM2_InitStructure);
  TIM_ARRPreloadConfig(TIM2, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);*/
}

void TIM3_Init() {

  TIM_TimeBaseInitTypeDef TIM3_InitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  //pwm timer
  TIM3_InitStructure.TIM_Period = 20000;
  TIM3_InitStructure.TIM_Prescaler = 72;
  TIM3_InitStructure.TIM_ClockDivision = 0;
  TIM3_InitStructure.TIM_CounterMode = TIM_CounterMode_Down;

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = 0x00;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0; // todo

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM3, &TIM3_InitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);

}

void USART2_Init(void)
{
    USART_InitTypeDef USART2_InitStructure;

// Enable the USART2 peripheral
USART_Cmd(USART2, ENABLE);

// TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
        // TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
        USART2_InitStructure.USART_BaudRate = 9600;
        USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART2_InitStructure.USART_StopBits = USART_StopBits_1;
        USART2_InitStructure.USART_Parity = USART_Parity_No;
        USART2_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
        USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART2, &USART2_InitStructure);

// TODO: Enable the USART2 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

}

void NVIC_Configure(void) {

    NVIC_InitTypeDef NVIC_InitStructure;

    // TODO: fill the arg you want
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // USART2
    // 'NVIC_EnableIRQ' is only required for USART setting
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_Init(&NVIC_InitStructure);
}


void TIM2_IRQHandler() {

}

void USART2_IRQHandler() {
    uint16_t word;
    if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET){
        // the most recent received data by the USART2 peripheral
        word = USART_ReceiveData(USART2);
        
        if(word == 's')
          gameStatus = 1;
        else if(word == 'p')
          gameStatus = 2;
        else if(word == 'r')
          gameStatus = 1;
        
        // clear 'Read data register not empty' flag
    USART_ClearITPendingBit(USART2,USART_IT_RXNE);
    }
}

int main(void)
{
  char msg[] = "abcde\r\n";
  unsigned int i;
  
  SystemInit();
  RCC_Configure();
  GPIO_Configure();
  USART2_Init();      // bluetooth
  NVIC_Configure();
  TIM2_Init();
  TIM3_Init();
  
  BT_RCC_Configure();
BT_GPIO_Configure();
BT_USART2_Init();
BT_NVIC_Configure();
  
  
  // ------------------------------------
  LCD_Init();
  Touch_Configuration();
  Touch_Adjust();
  LCD_Clear(WHITE);

  LCD_ShowString(20, 60, "MON_Team05", BLACK, WHITE);
  LCD_ShowString(20, 80, "Pinball Game !!", BLACK, WHITE);

/*
    while (1) {
      
      if(gameStatus == 0 && coinCount > 0) {
        
      }.
      
    Touch_GetXY(&x,&y,1);
    Convert_Pos(x,y,&x,&y);
    
    
      
    // show playtime
    if(gameSatus == 1) {
      LCD_ShowNum(100,120,(uint32_t)playtime,4,BLACK,WHITE);
    }
    */
  while(1)
  {

    LCD_ShowNum(20, 100, gameStatus, 10, BLACK, WHITE);

  }
    return 0;
}
