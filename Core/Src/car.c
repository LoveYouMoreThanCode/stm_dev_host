#include "car.h"

void Motor_Init(MotorTypeDef *m, TIM_HandleTypeDef *t1, uint32_t c1,TIM_HandleTypeDef *t2,uint32_t c2)
{
    m->timer_1 = t1;
    m->channel_1 = c1;
    m->timer_2 = t2;
    m->channel_2 = c2;
    __HAL_TIM_SET_COMPARE(m->timer_1, m->channel_1, 0);
    __HAL_TIM_SET_COMPARE(m->timer_2, m->channel_2, 0);
    HAL_TIM_PWM_Start(m->timer_1, m->channel_1);
    HAL_TIM_PWM_Start(m->timer_2, m->channel_1);
}
void Motor_Run(MotorTypeDef *m, int32_t speed)
{
    speed = speed < -100 ? -100 : speed;
    speed = speed > 100 ? 100 : speed;
    if (speed > 0)
    {
        uint32_t count = MAX_TIMR_COUNT * speed / 100;
        __HAL_TIM_SET_COMPARE(m->timer_1, m->channel_1, count);
        __HAL_TIM_SET_COMPARE(m->timer_2, m->channel_2, 0);
    }else {
        speed = -1 * speed;
        uint32_t count = MAX_TIMR_COUNT * speed / 100;
        __HAL_TIM_SET_COMPARE(m->timer_1, m->channel_1, 0);
        __HAL_TIM_SET_COMPARE(m->timer_2, m->channel_2, count);
    }
}
void Car_Init(CarTypeDef *car,MotorTypeDef m1,MotorTypeDef m2,MotorTypeDef m3,MotorTypeDef m4) {
    Motor_Init(&car->left_front, m1.timer_1, m1.channel_1, m1.timer_2, m1.channel_2);
    Motor_Init(&car->right_front, m2.timer_1, m2.channel_1, m2.timer_2, m2.channel_2);
    Motor_Init(&car->left_rear, m3.timer_1, m3.channel_1, m3.timer_2, m3.channel_2);
    Motor_Init(&car->right_rear, m4.timer_1, m4.channel_1, m4.timer_2, m4.channel_2);
}

void Car_Move(CarTypeDef *car, uint32_t cmd)
{
    switch (cmd)
    {
    case CAR_CMD_LEFT_FORWARD:
        Motor_Run(&car->left_front, 40);
        Motor_Run(&car->left_rear, 40);
        Motor_Run(&car->right_front, 60);
        Motor_Run(&car->right_rear, 60);
        break;
    case CAR_CMD_FORWARD:
        Motor_Run(&car->left_front, 70);
        Motor_Run(&car->left_rear, 70);
        Motor_Run(&car->right_front, 70);
        Motor_Run(&car->right_rear, 70);
        break;
    case CAR_CMD_RIGHT_FORWARD:
        Motor_Run(&car->left_front, 60);
        Motor_Run(&car->left_rear, 60);
        Motor_Run(&car->right_front, 40);
        Motor_Run(&car->right_rear, 40);
        break;
    case CAR_CMD_LEFT:
        Motor_Run(&car->left_front, -30);
        Motor_Run(&car->left_rear, -30);
        Motor_Run(&car->right_front, 30);
        Motor_Run(&car->right_rear, 30);
        break;
    case CAR_CMD_STOP:
        Motor_Run(&car->left_front, 0);
        Motor_Run(&car->left_rear, 0);
        Motor_Run(&car->right_front, 0);
        Motor_Run(&car->right_rear, 0);
        break;
    case CAR_CMD_RIGHT:
        Motor_Run(&car->left_front, 30);
        Motor_Run(&car->left_rear, 30);
        Motor_Run(&car->right_front, -30);
        Motor_Run(&car->right_rear, -30);
        break;
    case CAR_CMD_LEFT_BACKWARD:
        Motor_Run(&car->left_front, -40);
        Motor_Run(&car->left_rear, -40);
        Motor_Run(&car->right_front, -60);
        Motor_Run(&car->right_rear, -60);
        break;
    case CAR_CMD_BACKWARD:
        Motor_Run(&car->left_front, -70);
        Motor_Run(&car->left_rear, -70);
        Motor_Run(&car->right_front, -70);
        Motor_Run(&car->right_rear, -70);
        break;
    case CAR_CMD_RIGHT_BACKWARD:
        Motor_Run(&car->left_front, -60);
        Motor_Run(&car->left_rear, -60);
        Motor_Run(&car->right_front, -40);
        Motor_Run(&car->right_rear, -40);
    default:
        break;
    }
}