#include "servo.h"

void SERVO_Configure(PWM* pwm)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    //  APB1 clock (TIM3,4) enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    //  APB2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  
    GPIO_InitTypeDef GPIO_InitStructure; 
    GPIO_InitStructure.GPIO_Pin = pwm->gpio_pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate Function
    GPIO_Init(pwm->gpio_port, &GPIO_InitStructure);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // TIMER
    TIM_TimeBaseStructure.TIM_Period= 20000-1;
    TIM_TimeBaseStructure.TIM_Prescaler= (uint16_t)(SystemCoreClock/1000000)-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseInit(pwm->timer, &TIM_TimeBaseStructure);
      
    // PWM init
    TIM_OCInitStructure.TIM_OCMode      = pwm -> OCMode;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 1500;
    
    switch(pwm->channel){
    case 1:
      TIM_OC1Init(pwm->timer, &TIM_OCInitStructure);
      TIM_OC1PreloadConfig(pwm->timer, TIM_OCPreload_Disable);
      break;
    case 2:
      TIM_OC2Init(pwm->timer, &TIM_OCInitStructure);
      TIM_OC2PreloadConfig(pwm->timer, TIM_OCPreload_Disable);
      break;
    case 3:
      TIM_OC3Init(pwm->timer, &TIM_OCInitStructure);
      TIM_OC3PreloadConfig(pwm->timer, TIM_OCPreload_Disable);
      break;
    case 4:
      TIM_OC4Init(pwm->timer, &TIM_OCInitStructure);
      TIM_OC4PreloadConfig(pwm->timer, TIM_OCPreload_Disable);
      break;
    default:
      break;
    }
    
    TIM_ARRPreloadConfig(pwm->timer, ENABLE);
    TIM_Cmd(pwm->timer,ENABLE);
}

void SERVO_Rotate(PWM* pwm, int degree){
    // 0 ~ 180 degree movement
    int pwm_pulse = ((2300 - 700) / 180) * (degree - 15) + 700;
    TIM_OCInitTypeDef tim_oc_init_struct;
    tim_oc_init_struct.TIM_OCMode      = pwm->OCMode;
    tim_oc_init_struct.TIM_OCPolarity  = TIM_OCPolarity_High;
    tim_oc_init_struct.TIM_OutputState = TIM_OutputState_Enable;
    tim_oc_init_struct.TIM_Pulse       = pwm_pulse;
    // ¥ìo¡¤©öAI ©÷A ©øO¨úiAU ¡ÆI
    switch(pwm->channel){
    case 1:
      TIM_OC1Init(pwm->timer, &tim_oc_init_struct);
      break;
    case 2:
      TIM_OC2Init(pwm->timer, &tim_oc_init_struct);
      break;
    case 3:
      TIM_OC3Init(pwm->timer, &tim_oc_init_struct);
      break;
    case 4:
      TIM_OC4Init(pwm->timer, &tim_oc_init_struct);
      break;
    default:
      break;
    }
}