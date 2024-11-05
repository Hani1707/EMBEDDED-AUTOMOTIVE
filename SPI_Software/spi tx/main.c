#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM


#define SPI_SCK_Pin GPIO_Pin_0
#define SPI_MISO_Pin GPIO_Pin_1
#define SPI_MOSI_Pin GPIO_Pin_2
#define SPI_CS_Pin GPIO_Pin_3
#define SPI_GPIO GPIOA
#define SPI_RCC RCC_APB2Periph_GPIOA

void delay_ms(int timedelay)
{
    TIM_SetCounter(TIM2, 0); // Reset the counter of timer TIM2 to 0
    while(TIM_GetCounter(TIM2) < timedelay * 10) {} // Wait until TIM2 counter reaches (timedelay * 10)
    // This effectively creates a delay of 'timedelay' milliseconds (assuming each count is 0.1 ms)
}

void RCC_Config(){
	RCC_APB2PeriphClockCmd(SPI_RCC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}
// Timer configuration
void TIM_Config() {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1; // Set clock division to 1 (72 MHz / 1 = 72 MHz)
    TIM_TimeBaseInitStruct.TIM_Prescaler = 7200 - 1;        // Set prescaler to 7200-1 (prescales the clock by 7200)
    TIM_TimeBaseInitStruct.TIM_Period = 0xFFFF - 1;         // Set the auto-reload period to maximum (16-bit timer)
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; // Set counter mode to up-counting

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct); // Initialize TIM2 with the configuration
    TIM_Cmd(TIM2, ENABLE);                           // Enable TIM2
}


void Clock(){
	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_SET);
	delay_ms(4);
	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_RESET);
	delay_ms(4);
}

void SPI_Init(){
	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_RESET);
	GPIO_WriteBit(SPI_GPIO, SPI_CS_Pin, Bit_SET);
	GPIO_WriteBit(SPI_GPIO, SPI_MISO_Pin, Bit_RESET);
	GPIO_WriteBit(SPI_GPIO, SPI_MOSI_Pin, Bit_RESET);
}


void GPIO_Config(){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = SPI_SCK_Pin| SPI_MOSI_Pin| SPI_CS_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI_MISO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
}

void SPI_Master_Transmit(uint8_t u8Data){//0b10010000
	uint8_t u8Mask = 0x80;// 0b10000000
	uint8_t tempData;
	GPIO_WriteBit(SPI_GPIO, SPI_CS_Pin, Bit_RESET);
	delay_ms(1);
	for(int i=0; i<8; i++){
		tempData = u8Data & u8Mask;
		if(tempData){
			GPIO_WriteBit(SPI_GPIO, SPI_MOSI_Pin, Bit_SET);
			delay_ms(1);
		} else{
			GPIO_WriteBit(SPI_GPIO, SPI_MOSI_Pin, Bit_RESET);
			delay_ms(1);
		}
		u8Data=u8Data<<1;
		Clock();
	}
	GPIO_WriteBit(SPI_GPIO, SPI_CS_Pin, Bit_SET);
	delay_ms(1);
}








uint8_t DataTrans[] = {1,3,9,10,15,19,90};//Data
int main(){
	RCC_Config();
	GPIO_Config();
	TIM_Config();
	SPI_Init();
	while(1){	
		for(int i = 0; i < 7; i++){
			SPI_Master_Transmit(DataTrans[i]);
			delay_ms(1000);
		}
	}
}





