#ifndef __CAR_H__
#define __CAR_H__

#include "stm32f1xx_hal.h"

#ifndef uint32_t
#define uint32_t unsigned int
#endif

#define MAX_TIMR_COUNT 10240
typedef struct Motor
{
    TIM_HandleTypeDef *timer_1;
    uint32_t channel_1;

    TIM_HandleTypeDef *timer_2;
    uint32_t channel_2;
} MotorTypeDef;
void Motor_Init(MotorTypeDef *m, TIM_HandleTypeDef *t1, uint32_t c1, TIM_HandleTypeDef *t2, uint32_t c2);
void Motor_Run(MotorTypeDef *m, int32_t speed);

typedef struct Car {
    MotorTypeDef left_front;
    MotorTypeDef right_front;
    MotorTypeDef left_rear;
    MotorTypeDef right_rear;
}CarTypeDef;

void Car_Init(CarTypeDef *car,MotorTypeDef m1,MotorTypeDef m2,MotorTypeDef m3,MotorTypeDef m4);
void Car_Move(CarTypeDef *car, uint32_t cmd);

#endif