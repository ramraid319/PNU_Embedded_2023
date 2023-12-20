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
//#include "sensor.h"
#include "servo.h"

int color[12] = {WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY};

/*
0 : IDLE (Wait Coin)
1 : Game Start
2 : Game Pause
3 : Game Over
*/
int gameStatus = 1;

// coin count
int coinCount = 0;

//position
uint16_t value, x, y;
volatile uint16_t toggle = 0;

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
void sendDataUART1(uint16_t data);
void sendDataUART2(uint16_t data);
static void pwm_setting();
void change(PWM* pwm, uint16_t per);
void delay(int d);


void SENSOR_RCC_Configure(void);
void SENSOR_GPIO_Configure(void);
void SENSOR_ADC_Configure(void);
void SENSOR_DMA_Configure(void);

PWM pwm, pwm1, pwm2, pwm3, pwm4, pwm5, pwm6;

volatile uint32_t ADC_Value[4];
volatile uint32_t LED[3] = {0, 0, 0};


void delay(int d){
  for (int i = 0; i <= d; i++) {
      ;
  }
} 

/*--------------------Servo Setting------------------------*/
static void pwm_setting(){
    // right, left servo 0,1
    pwm.OCMode     = TIM_OCMode_PWM1;
    pwm.rcc_timer    = RCC_APB1Periph_TIM4;
    pwm.timer           = TIM4;
    pwm.rcc_gpio     = RCC_APB2Periph_GPIOB;
    pwm.gpio_port    = GPIOB;
    pwm.gpio_pin     = GPIO_Pin_8;
    pwm.channel      = 3;
    SERVO_Configure(&pwm);
    //SERVO_Rotate(&pwm, 0);
    
    pwm2.OCMode    = TIM_OCMode_PWM1;
    pwm2.rcc_timer   = RCC_APB1Periph_TIM4;
    pwm2.timer          = TIM4;
    pwm2.rcc_gpio    = RCC_APB2Periph_GPIOB;
    pwm2.gpio_port   = GPIOB;
    pwm2.gpio_pin    = GPIO_Pin_9;
    pwm2.channel     = 4;
    SERVO_Configure(&pwm2);
    SERVO_Rotate(&pwm2, 0);
    
    //start
    pwm5.OCMode      = TIM_OCMode_PWM1;
    pwm5.rcc_timer   = RCC_APB1Periph_TIM3;
    pwm5.timer       = TIM3;
    pwm5.rcc_gpio    = RCC_APB2Periph_GPIOA;
    pwm5.gpio_port   = GPIOA;
    pwm5.gpio_pin    = GPIO_Pin_6;
    pwm5.channel     = 1;
    SERVO_Configure(&pwm5);
    SERVO_Rotate(&pwm5, 210);
    
    //left sidei servo
    pwm6.OCMode      = TIM_OCMode_PWM1;
    pwm6.rcc_timer   = RCC_APB1Periph_TIM3;
    pwm6.timer       = TIM3;
    pwm6.rcc_gpio    = RCC_APB2Periph_GPIOA;
    pwm6.gpio_port   = GPIOA;
    pwm6.gpio_pin    = GPIO_Pin_7;
    pwm6.channel     = 2;
    SERVO_Configure(&pwm6);
    
}

/*--------------------TIM Configure------------------------*/
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

/*--------------------UART Configure------------------------*/
void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* USART1 pin setting */
    // UART pin TX GPIO_Pin_9, UART pin RX GPIO_Pin_10
   
    //TX a9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    //RX a10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD | GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    /* USART2 pin setting */
   
    //TX a2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    //RX a3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD | GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
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

void USART1_Init(void)
{
    USART_InitTypeDef USART1_InitStructure;

// Enable the USART1 peripheral
    USART_Cmd(USART1, ENABLE);

// TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
        USART1_InitStructure.USART_BaudRate = 9600;
        USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART1_InitStructure.USART_StopBits = USART_StopBits_1;
        USART1_InitStructure.USART_Parity = USART_Parity_No;
        USART1_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
        USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART1, &USART1_InitStructure);

   
   // TODO: Enable the USART1 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
   USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

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
    
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
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
/*
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_Init(&NVIC_InitStructure);*/
}

void TIM2_IRQHandler() {

}

void USART1_IRQHandler() {
    uint16_t word;
    if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET){
        // the most recent received data by the USART1 peripheral
        word = USART_ReceiveData(USART1);
        sendDataUART2(word);
        //sendDataUART1(word);
       
        // TODO implement

        // clear 'Read data register not empty' flag
    USART_ClearITPendingBit(USART1,USART_IT_RXNE);
    }
}

void USART2_IRQHandler() {
    uint16_t word;
    if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET){
        // the most recent received data by the USART2 peripheral
        word = USART_ReceiveData(USART2);
        
        if(word == 's' && coinCount > 0)
          gameStatus = 1;
        else if(word == 'p')
          gameStatus = 2;
        else if(word == 'r')
          gameStatus = 1;
        
        // clear 'Read data register not empty' flag
    USART_ClearITPendingBit(USART2,USART_IT_RXNE);
    }
}

void sendDataUART1(uint16_t data) {
  USART_SendData(USART1, data);
}

void sendDataUART2(uint16_t data) {
  USART_SendData(USART2, data);
}

/*--------------------Sensor Configure------------------------*/
void SENSOR_RCC_Configure() {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  // RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  // DMA port clock enable
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

void SENSOR_GPIO_Configure() {
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure_C;
  
  // ADC Port Configure
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_InitStructure_C.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure_C.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure_C);
}

void SENSOR_ADC_Configure() {
  ADC_InitTypeDef ADC_InitStructure;
  
  // ADC Configure
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 4;
  
  ADC_Init(ADC1, &ADC_InitStructure);
  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5); // PC0
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_239Cycles5); // PC1
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_239Cycles5); // PC2
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_239Cycles5); // PC3
  
  // Enable interrupt
  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
  ADC_DMACmd(ADC1, ENABLE);
  
  // Enable ADC1
  ADC_Cmd(ADC1, ENABLE);
  ADC_ResetCalibration(ADC1);
  
  
  while(ADC_GetResetCalibrationStatus(ADC1));
  
  ADC_StartCalibration(ADC1);
  
  while(ADC_GetCalibrationStatus(ADC1));
  
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void SENSOR_DMA_Configure() {
  DMA_InitTypeDef DMA_Instructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  DMA_DeInit(DMA1_Channel1);
  /* DMA Configuration */
  
  DMA_Instructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  DMA_Instructure.DMA_MemoryBaseAddr = (uint32_t)ADC_Value;
  DMA_Instructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  
  DMA_Instructure.DMA_BufferSize = 4;
  
  DMA_Instructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_Instructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_Instructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_Instructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  
  DMA_Instructure.DMA_Mode = DMA_Mode_Circular;
  DMA_Instructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_Instructure.DMA_M2M = DMA_M2M_Disable;  
  
  DMA_Init(DMA1_Channel1, &DMA_Instructure);
  DMA_ITConfig(DMA1_Channel1, DMA1_IT_TC1, ENABLE);
  DMA_Cmd(DMA1_Channel1, ENABLE);
}

void DMA1_Channel1_IRQHandler() {
  if (DMA_GetITStatus(DMA1_IT_TC1) != RESET) {

    // TODO : Change to LED

      /*
      * Check Coin Sensor Values
    if (gameStatus == 0 && ADC_Vlaue[3] <= 10)
    {
        coinCount++;
        LCD_ShowString(20, 240, "Coin In : ", BLACK, WHITE);
        LCD_ShowNum(220, 240, (uint32_t)coinCount, 8, BLACK, WHITE);
    }
    */
    if (gameStatus == 1 && ADC_Value[0] <=10)
    {
        LCD_ShowString(20, 240, "Sensor 1 Pressed !!", BLACK, WHITE);
        LED[0] == 1;
    }
    
    if (gameStatus == 1 && ADC_Value[1] <=10)
    {
        LCD_ShowString(20, 260, "Sensor 2 Pressed !!", BLACK, WHITE);
        LED[1] == 1;
        GPIO_SetBits(GPIOA,GPIO_Pin_0);

    }
    
    if (gameStatus == 1 && ADC_Value[2] <=10)
    {
        LCD_ShowString(20, 280, "Sensor 3 Pressed !!", BLACK, WHITE);
        LED[2] == 1;
    }

    if (LED[0] == 1 && LED[1] == 1 && LED[2] == 1)
    {
        LCD_ShowString(20, 300, "Game Win !!", BLACK, WHITE);
        gameStatus = 3;
    }
    
    DMA_ClearITPendingBit(DMA1_IT_TC1);
  }
}

/*--------------------LED Configure------------------------*/
void LED_RCC_Configure()
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
}

void LED_GPIO_Configure()
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}


int main(void)
{
  char msg[] = "abcde\r\n";
  unsigned int i;
  
  SystemInit();
  RCC_Configure();
  GPIO_Configure(); 
  USART1_Init();      // PC
  USART2_Init();      // bluetooth
  NVIC_Configure();
  TIM2_Init();
  TIM3_Init();
  
  BT_RCC_Configure();
  BT_GPIO_Configure();
  BT_USART1_Init();
  BT_USART2_Init();
  BT_NVIC_Configure();
  
  SENSOR_RCC_Configure();
  SENSOR_GPIO_Configure();
  SENSOR_ADC_Configure();
  SENSOR_DMA_Configure();
  
  LED_RCC_Configure();
  LED_GPIO_Configure();
  
  pwm_setting();

  // ------------------------------------
  
  LCD_Init();
  Touch_Configuration();
  //Touch_Adjust();
  LCD_Clear(WHITE);

  LCD_ShowString(20, 60, "MON_Team05", BLACK, WHITE);
  LCD_ShowString(20, 80, "Pinball Game !!", BLACK, WHITE);
  LCD_DrawRectangle(10, 120, 110, 220);
  LCD_DrawRectangle(130, 120, 230, 220);
  LCD_ShowString(40, 155, "LEFT", RED, WHITE);
  LCD_ShowString(160, 155, "RIGHT", BLUE, WHITE);

  GPIO_ResetBits(GPIOA,GPIO_Pin_0);

  
  while (1) {
      
    if(gameStatus == 0)
      LCD_ShowString(20, 100, "IDLE", BLACK, WHITE);

    else if(gameStatus == 1)
      LCD_ShowString(20, 100, "Game Start !!", BLACK, WHITE);

    else if(gameStatus == 2)
      LCD_ShowString(20, 100, "Game Pause ..", BLACK, WHITE);

    else if (gameStatus == 3)
    {
      LCD_ShowString(20, 100, "Game Over ;;", BLACK, WHITE);
      LCD_ShowNum(20, 120, (uint32_t)gameTime, 16, BLACK, WHITE);
    }
    
    Touch_GetXY(&x,&y,1);
    Convert_Pos(x,y,&x,&y);
    
    if (gameStatus == 1)
    {
        if (x >= 10 && x <= 110 && y >= 120 && y <= 220)
        {
            if (toggle == 0)
            {
                SERVO_Rotate(&pwm, 180);
                toggle = 1;
            }

            delay(10000000);

            if (toggle == 1)
            {
                SERVO_Rotate(&pwm, 0);
                toggle = 0;
            }
        }

        if (x >= 130 && x <= 230 && y >= 120 && y <= 220)
        {
            if (toggle == 0)
            {
                SERVO_Rotate(&pwm2, 0);
                toggle = 1;
            }

            delay(10000000);

            if (toggle == 1)
            {
                SERVO_Rotate(&pwm2, 180);
                toggle = 0;
            }
        }
    }

    /* testing

    LCD_ShowNum(20, 120, (uint32_t)ADC_Value[0], 32, BLACK, WHITE);
    LCD_ShowNum(20, 140, (uint32_t)toggle, 32, BLACK, WHITE);
    */

  }
  return 0;
}
