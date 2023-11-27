#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_adc.h"
#include "lcd.h"
#include "touch.h"
#include "stm32f10x_dma.h"

#define ADC1_DR_Address 0x40012440

volatile uint32_t ADC_Value;

//volatile uint32_t ADC_Value;
int color[12] = {WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY};


void RCC_Configure(void)
{
   // Enable the APB2 peripheral clock using the function 'RCC_APB2PeriphClockCmd'
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}


void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //조도센서 핀
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void ADC_Configure(void){

  ADC_InitTypeDef ADC_12;
  
  ADC_DeInit(ADC1);
  ADC_12.ADC_Mode = ADC_Mode_Independent;
  ADC_12.ADC_ScanConvMode = DISABLE;
  ADC_12.ADC_ContinuousConvMode = ENABLE;
  ADC_12.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_12.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_12.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_12);
  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12,1, ADC_SampleTime_239Cycles5);
  ADC_DMACmd(ADC1,ENABLE);
  ADC_Cmd(ADC1, ENABLE);
  ADC_ResetCalibration(ADC1);
  
  while (ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  
  while (ADC_GetCalibrationStatus(ADC1));
  ADC_SoftwareStartConvCmd(ADC1,ENABLE);
}


void DMA_Configure(){
  DMA_InitTypeDef DMA_InitStructure;
  
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);//ADC1_DR_Address; //어떤 레지스터의 정보를 받을지 메모리 주소를 설정
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &ADC_Value;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //조도센서 주소 증가 하지x
  DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable; //메모리 주소 증가
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //memory to memory disable
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  DMA_Cmd(DMA1_Channel1, ENABLE);
}

int main() {
// LCD 관련 설정은 LCD_Init에 구현되어 있으므로 여기서 할 필요 없음
  SystemInit();
  RCC_Configure();
  GPIO_Configure();
  ADC_Configure();
  DMA_Configure();
  // ------------------------------------
  LCD_Init();
  Touch_Configuration();
  Touch_Adjust();
  LCD_Clear(WHITE);


  while(1){

      //LCD_ShowNum(100,120,1111,4,WHITE,WHITE);

      if((uint32_t)ADC_Value> 1000){ //밝을때);
        LCD_Clear(WHITE);
      }
      else{
        LCD_Clear(GRAY);
      }
    
      if((uint32_t)ADC_Value> 1000){ //밝을때);
        LCD_ShowNum(100,120,(uint32_t)ADC_Value,4,BLACK,GRAY);
      }
      else{
        LCD_ShowNum(100,120,(uint32_t)ADC_Value,4,BLACK,WHITE);
      }
    }
}