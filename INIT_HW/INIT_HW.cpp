#include "mbed.h"
#include "FastPWM.h"
#include "setting.h"

void Init_ADC(void){
    // ADC Setup
     RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;                        // clock for ADC3
     RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;                        // clock for ADC2
     RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;                        // clock for ADC1
     
     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;                        // Enable clock for GPIOC
     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;                        // Enable clock for GPIOA
    
     ADC->CCR = 0x00000016;                                     // Regular simultaneous mode only
     ADC1->CR2 |= ADC_CR2_ADON;//0x00000001;                    // ADC1 ON
     ADC1->SQR3 = 0x0000000E;                    //channel      // use PC_4 as input- ADC1_IN14
     ADC2->CR2 |= ADC_CR2_ADON;//0x00000001;                    // ADC2 ON
     ADC2->SQR3 = 0x00000008;                                   // use PB_0 as input - ADC2_IN8
     ADC3->CR2 |= ADC_CR2_ADON;                                 // ADC3 ON
     ADC3->SQR3 = 0x0000000B;                                   // use PC_1, - ADC3_IN11
     GPIOC->MODER |= 0b1100001100;             //each channel   // PC_4, PC_1 are analog inputs 
     GPIOB->MODER |= 0x3;                                       // PB_0 as analog input
     
     ADC1->SMPR1 |= 0x00001000;                                     // 15 cycles on CH_14, 0b0001000000000000
     ADC2->SMPR2 |= 0x01000000;                                     // 15 cycles on CH_8,  0b0000000100000000<<16
     ADC3->SMPR1 |= 0x00000008;                                     // 15 cycles on CH_11, 0b0000000000001000

    }
    
void Init_PWM(){

    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;                         // enable TIM4 clock
    FastPWM pwm_v(PIN_V);
    FastPWM pwm_w(PIN_W);
    
     //ISR Setup     
    
    NVIC_EnableIRQ(TIM4_IRQn);                         //Enable TIM4 IRQ

    TIM4->DIER |= TIM_DIER_UIE;                                 // enable update interrupt
    TIM4->CR1 = 0x40;                                           // CMS = 10, interrupt only when counting up // Center-aligned mode
    TIM4->CR1 |= TIM_CR1_UDIS;
    TIM4->CR1 |= TIM_CR1_ARPE;                                  // autoreload on, 
    TIM4->RCR |= 0x001;                                         // update event once per up/down count of TIM4 
    TIM4->EGR |= TIM_EGR_UG;
 
    //PWM Setup

    TIM4->PSC = 0x0;                                            // no prescaler, timer counts up in sync with the peripheral clock
    TIM4->ARR = PWM_ARR;                                          // set auto reload
    TIM4->CCER |= ~(TIM_CCER_CC1NP);                            // Interupt when low side is on.
    TIM4->CR1 |= TIM_CR1_CEN;                                   // enable TIM4
    
}

void Init_TMR3(){
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;                         // enable TIM3 clock
    
     //ISR Setup     
    
    NVIC_EnableIRQ(TIM3_IRQn);                         //Enable TIM3 IRQ

    TIM3->DIER |= TIM_DIER_UIE;                                 // enable update interrupt
    TIM3->CR1 = 0x40;                                           // CMS = 10, interrupt only when counting up // Center-aligned mode
    TIM3->CR1 |= TIM_CR1_UDIS;
    TIM3->CR1 |= TIM_CR1_ARPE;                                  // autoreload on, 
    TIM3->RCR |= 0x001;                                         // update event once per up/down count of TIM3 
    TIM3->EGR |= TIM_EGR_UG;

    TIM3->PSC = 0x00;                                            // no prescaler, timer counts up in sync with the peripheral clock
    TIM3->ARR = TMR3_COUNT;                                          // set auto reload, 5 khz
    TIM3->CCER |= ~(TIM_CCER_CC1NP);                            // Interupt when low side is on.
    TIM3->CR1 |= TIM_CR1_CEN;                                   // enable TIM4
}

void Init_TMR2(){
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;                         // enable TIM5 clock
    
     //ISR Setup     
    
    NVIC_EnableIRQ(TIM2_IRQn);                         //Enable TIM5 IRQ

    TIM2->DIER |= TIM_DIER_UIE;                                 // enable update interrupt
    TIM2->CR1 = 0x40;                                           // CMS = 10, interrupt only when counting up // Center-aligned mode
    TIM2->CR1 |= TIM_CR1_UDIS;
    TIM2->CR1 |= TIM_CR1_ARPE;                                  // autoreload on, 
    TIM2->RCR |= 0x001;                                         // update event once per up/down count of TIM5
    TIM2->EGR |= TIM_EGR_UG;

    TIM2->PSC = 0x12;                                            // no prescaler, timer counts up in sync with the peripheral clock
    TIM2->ARR = TMR2_COUNT;                                          // set auto reload, 5 khz
    TIM2->CCER |= ~(TIM_CCER_CC1NP);                            // Interupt when low side is on.
    TIM2->CR1 |= TIM_CR1_CEN;                                   // enable TIM5
}