#ifndef TIMER_H
#define TIMER_H
#include "main.h"

extern void TIM1_Init(uint16_t arr, uint16_t psc);
extern void TIM3_Init(uint16_t arr, uint16_t psc);
extern void TIM6_Init(uint16_t arr, uint16_t psc);
extern void TIM12_Init(uint16_t arr, uint16_t psc);

void TIM6_Configuration(void);
void TIM8_Configuration(void);//Ä¦²ÁÂÖ ZÒý½Å

#endif
