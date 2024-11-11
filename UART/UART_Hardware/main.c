#include "stm32f10x.h"        // Device header
#include "stm32f10x_rcc.h"    // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"   // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_usart.h"  // Keil::Device:StdPeriph Drivers:USART
#include "stm32f10x_tim.h"    // Keil::Device:StdPeriph Drivers:TIM

void RCC_Config(void) {
    // B?t clock cho GPIOA, USART1 v� TIM2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

void GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    // C?u h�nh ch�n TX (PA9) cho UART
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // C?u h�nh ch�n RX (PA10) cho UART
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void USART_Config(void) {
    USART_InitTypeDef USART_InitStruct;

    // C?u h�nh th�ng s? cho USART
    USART_InitStruct.USART_BaudRate = 9600;                            // Baudrate
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;           // 8 bits
    USART_InitStruct.USART_StopBits = USART_StopBits_1;                // 1 stop bit
    USART_InitStruct.USART_Parity = USART_Parity_No;                   // No parity
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;       // C? truy?n v� nh?n
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStruct);

    // B?t USART1
    USART_Cmd(USART1, ENABLE);
}

void TIM2_Config(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

    // C?u h�nh TIM2 d? d?m m?i 1 �s (1 MHz)
    TIM_TimeBaseInitStruct.TIM_Prescaler = 72 - 1;  // 72 MHz / 72 = 1 MHz (1 �s)
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period = 0xFFFF;     // Max value cho 16-bit counter
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

    // B?t TIM2
    TIM_Cmd(TIM2, ENABLE);
}

void delay_us(uint32_t microseconds) {
    TIM_SetCounter(TIM2, 0);  // �?t l?i b? d?m v? 0
    while (TIM_GetCounter(TIM2) < microseconds);
}

void delay_ms(uint32_t milliseconds) {
    for (uint32_t i = 0; i < milliseconds; i++) {
        delay_us(1000);  // G?i delay_us v?i 1000 �s d? d?t du?c 1 ms
    }
}

/*void UART_Transmit(char *str) {
    while (*str) {
        // �?i cho d?n khi c? TXE (Transmit Data Register Empty) du?c thi?t l?p
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);

        // G?i t?ng k� t? qua USART1
        USART_SendData(USART1, *str++);

        // �?i cho d?n khi qu� tr�nh truy?n ho�n t?t
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    }
}

char UART_Receive(void) {
    // �?i d?n khi c� d? li?u nh?n du?c (c? RXNE du?c thi?t l?p)
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

    // �?c d? li?u t? b? d?m nh?n
    return (char)USART_ReceiveData(USART1);
}*/

uint8_t USART1_ReceiveByte(void) {
    uint8_t temp = 0x00;
    
    // Ch? d?n khi d? li?u du?c nh?n (c? RXNE du?c thi?t l?p)
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
    
    // �?c d? li?u v?a nh?n du?c
    temp = USART_ReceiveData(USART1);
    return temp;
}

void USART1_TransmitByte(uint8_t byte) {
    // Ch? cho d?n khi thanh ghi truy?n d? li?u tr?ng (c? TXE du?c thi?t l?p)
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    
    // Truy?n d? li?u (byte)
    USART_SendData(USART1, byte);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}


/*int main(void) {
    char received_char;
    
    RCC_Config();
    GPIO_Config();
    USART_Config();
    TIM2_Config();

    while (1) {
        
        UART_Transmit("DUNG\n");

        // T?o d? tr? gi?a c�c l?n truy?n (1 gi�y)
        delay_ms(1000);

        // Nh?n m?t k� t? t? UART
        received_char = UART_Receive();

        // Truy?n l?i k� t? v?a nh?n d? ki?m tra
        UART_Transmit(&received_char);

        // T?o d? tr? d? nh?n k� t? ti?p theo
        delay_ms(1000);
    }
}*/

char data[] = {'d', 'u', 'n', 'g', 'x', 'd', '!'};
char buffer[100];

int main() {
    RCC_Config();
    GPIO_Config();
    TIM2_Config();
    USART_Config();

    while (1) {
        // V�ng l?p d? truy?n t?ng k� t? trong m?ng `data`
        for (int i = 0; i < 7; i++) {
            USART1_TransmitByte(data[i]);   // Truy?n t?ng k� t? qua UART
            delay_ms(1000);                 // T?o d? tr? 1 gi�y gi?a c�c k� t?
        }

        // Truy?n k� t? xu?ng d�ng ('\n') d? b�o hi?u k?t th�c chu?i
        USART1_TransmitByte('\n');

        // V�ng l?p kh�c (chua r� ch?c nang, nhung c� th? d? x? l� nh?n d? li?u)
       // for (int i = 0; i < 100; i++) {
            // N?i dung v�ng l?p (chua th?y du?c t? h�nh ?nh, n�n s? c?n x�c d?nh th�m)
        //}
    }
}

