#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f10x_spi.h"              // Keil::Device:StdPeriph Drivers:SPI
               
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
// Timer configuration
void TIM_Config() {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1; // Clock division factor of 1 (72 MHz / 1 = 72 MHz)
    TIM_TimeBaseInitStruct.TIM_Prescaler = 7200 - 1;        // Prescaler value of 7200, effectively reducing clock speed
    TIM_TimeBaseInitStruct.TIM_Period = 0xFFFF - 1;         // Maximum 16-bit period value for the timer
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; // Configure the timer to count upwards

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct); // Initialize TIM2 with the defined settings
    TIM_Cmd(TIM2, ENABLE);                           // Enable TIM2 to start counting
}


void RCC_Config(){
	RCC_APB2PeriphClockCmd(SPI_RCC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

void Clock(){
	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_SET);
	delay_ms(4);
	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_RESET);
	delay_ms(4);
}

void SPI_Config(){
	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_RESET);
	GPIO_WriteBit(SPI_GPIO, SPI_CS_Pin, Bit_SET);
	GPIO_WriteBit(SPI_GPIO, SPI_MISO_Pin, Bit_RESET);
	GPIO_WriteBit(SPI_GPIO, SPI_MOSI_Pin, Bit_RESET);
}

void GPIO_Config() {
    GPIO_InitTypeDef GPIO_InitStructure;

    // Configuring MISO (Master In Slave Out) Pin
    GPIO_InitStructure.GPIO_Pin = SPI_MISO_Pin;       // Select the MISO pin
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // Set as Push-Pull Output mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // Set speed to 50MHz
    GPIO_Init(SPI_GPIO, &GPIO_InitStructure);         // Initialize with above settings

    // Configuring SCK (Clock), MOSI (Master Out Slave In), and CS (Chip Select) Pins
    GPIO_InitStructure.GPIO_Pin = SPI_SCK_Pin | SPI_MOSI_Pin | SPI_CS_Pin; // Select SCK, MOSI, and CS pins
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // Set as Floating Input mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     // Set speed to 50MHz
    GPIO_Init(SPI_GPIO, &GPIO_InitStructure);             // Initialize with above settings
}

uint8_t SPI_Slave_Receive(void){
	uint8_t u8Mask = 0x80;
	uint8_t dataReceive =0x00;//0b11000000
	uint8_t temp = 0x00, i=0;
	while(GPIO_ReadInputDataBit(SPI_GPIO, SPI_CS_Pin));
	while(!GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin));
	for(i=0; i<8;i++)
    	{ 
		if(GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin)){
		while (GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin)) 
			temp = GPIO_ReadInputDataBit(SPI_GPIO, SPI_MOSI_Pin);
		dataReceive=dataReceive<<1;
		dataReceive=dataReceive|temp;
    		}
	while(!GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin));
	}
	return dataReceive;
}



uint8_t DataReceive[7];
uint8_t dat;
int main(){
	RCC_Config();
	GPIO_Config();
	TIM_Config();
	SPI_Config();
	while(1)
		{	
			if(!(GPIO_ReadInputDataBit(SPI_GPIO, SPI_CS_Pin))){
				for(int i=0; i<7; i++)
				dat=SPI_Slave_Receive();
			}
		}
	}
		





