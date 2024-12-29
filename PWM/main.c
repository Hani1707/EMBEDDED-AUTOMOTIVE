#include "stm32f10x.h"

void TIM3_PWM_Init(void);
void Servo_SetAngle(uint16_t angle);

int main(void) {
    // Kh?i t?o PWM cho servo
    TIM3_PWM_Init();

    while (1) {
        // Quay servo t?i góc 0°
        Servo_SetAngle(0);
        for (volatile int i = 0; i < 10000; i++);  // Delay don gi?n

        // Quay servo t?i góc 90°
        Servo_SetAngle(90);
        for (volatile int i = 0; i < 10000; i++);  // Delay don gi?n

//        // Quay servo t?i góc 180°
//        Servo_SetAngle(180);
//        for (volatile int i = 0; i < 20000; i++);  // Delay don gi?n
    }
}

void TIM3_PWM_Init(void) {
    // B?t xung clock cho GPIOA và TIM3
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // C?u hình chân PA6 cho PWM (TIM3 Channel 1)
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // C?u hình Timer 3
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1;  // Chu k? 20ms (50Hz)
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;  // T?n s? 1MHz (1us per tick)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // C?u hình PWM cho TIM3 Channel 1
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1500;  // Giá tr? trung bình 1.5ms cho v? trí trung tâm
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);

    // B?t Timer 3
    TIM_Cmd(TIM3, ENABLE);
}

void Servo_SetAngle(uint16_t angle) {
    // Chuy?n d?i góc thành d? r?ng xung
    uint16_t pulse_length = (1000 + ((angle * 1000) / 180));  // 1000us d?n 2000us cho góc 0° d?n 180°
    TIM_SetCompare1(TIM3, pulse_length);
}
