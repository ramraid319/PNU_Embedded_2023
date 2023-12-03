#include "bluetooth.h"

void BT_RCC_Configure(void)
{
/* USART1, USART2 TX/RX port clock enable */  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

/* USART1, USART2 clock enable */  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  
/* Alternate Function IO clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void BT_GPIO_Configure(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  //USART2 TX a2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //USART2 RX a3
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD | GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void BT_USART2_Init(void)
{
  USART_InitTypeDef USART2_InitStructure;

// Enable the USART2 peripheral
  USART_Cmd(USART2, ENABLE);

//Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
  USART2_InitStructure.USART_BaudRate = 9600;
  USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART2_InitStructure.USART_StopBits = USART_StopBits_1;
  USART2_InitStructure.USART_Parity = USART_Parity_No;
  USART2_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART2, &USART2_InitStructure);

//Enable the USART2 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

}

void BT_NVIC_Configure(void) {

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // USART2
    // 'NVIC_EnableIRQ' is only required for USART setting
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}