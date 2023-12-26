#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "common.h"

void BT_RCC_Configure(void);
void BT_GPIO_Configure(void);
void BT_USART1_Init(void);
void BT_USART2_Init(void);
void BT_NVIC_Configure(void);

#endif