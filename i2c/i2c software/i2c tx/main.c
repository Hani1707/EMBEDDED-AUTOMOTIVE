#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_i2c.h"              // Keil::Device:StdPeriph Drivers:I2C
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC

#define I2C_SCL 		GPIO_Pin_6
#define I2C_SDA		GPIO_Pin_7
#define EEPROM_ADDRESS  0x57       // Dia chi I2C cua AT24C32
#define I2C_SPEED       100000     // Toc do I2C 100kHz
#define I2C_GPIO 	GPIOB

#define WRITE_SDA_0 	GPIO_ResetBits(I2C_GPIO, I2C_SDA)
#define WRITE_SDA_1 	GPIO_SetBits(I2C_GPIO, I2C_SDA)
#define WRITE_SCL_0 	GPIO_ResetBits(I2C_GPIO, I2C_SCL)
#define WRITE_SCL_1 	GPIO_SetBits(I2C_GPIO, I2C_SCL)
#define READ_SDA_VAL 	GPIO_ReadInputDataBit(I2C_GPIO, I2C_SDA)
typedef enum{
	NOT_OK = 0,
	OK = 1
} status;
	
typedef enum{
	NACK = 0,
	ACK = 1
} ACK_Bit;
// Delay function using Timer2
void delay_us(uint32_t Delay)
{
    TIM_SetCounter(TIM2, 0);
    while (TIM_GetCounter(TIM2) < Delay)
        ;
}

void Delay1Ms(void)
{
	TIM_SetCounter(TIM2, 0);
	while (TIM_GetCounter(TIM2) < 1000){
}
	}

void Delay_Ms(uint32_t u32DelayInMs)
{
    while (u32DelayInMs) {
        Delay1Ms();
        --u32DelayInMs;
    }
}


void TIM2_Config()
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1; // Prescale to 1 MHz (1 µs period)
    TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;    // Maximum period for counting
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    TIM_Cmd(TIM2, ENABLE);
}


void RCC_Config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

void GPIO_Config(){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin = I2C_SDA| I2C_SCL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(I2C_GPIO, &GPIO_InitStructure);
}

void I2C_Config(){
	WRITE_SDA_1;
	delay_us(1);
	WRITE_SCL_1;
	delay_us(1);
}

void I2C_Start(){
	
	WRITE_SCL_1;  	
	delay_us(3);	
	WRITE_SDA_1;
	delay_us(3);
	WRITE_SDA_0;	//SDA reset to 0 before SCL.
	delay_us(3);
	WRITE_SCL_0;
	delay_us(3);
}

void I2C_Stop(){
	
	WRITE_SDA_0;
	delay_us(3);
	WRITE_SCL_1; 	//SCL set to 1 before SDA.
	delay_us(3);
	WRITE_SDA_1;
	delay_us(3);
}

status I2C_Write(uint8_t u8Data){	
	uint8_t i;
	status stRet;
	for(int i=0; i< 8; i++){		//Write byte data.
		if (u8Data & 0x80) {
			WRITE_SDA_1;
		} else {
			WRITE_SDA_0;
		}
		delay_us(3); // make sure SDA high complete
		
		//SCL Clock
		WRITE_SCL_1;
		delay_us(5);
		WRITE_SCL_0;
		delay_us(2);
		// shift 1 bit to the left
		u8Data <<= 1;
	}
	WRITE_SDA_1;					//
	delay_us(3);
	WRITE_SCL_1;					//
	delay_us(3);
	
	if (READ_SDA_VAL) {	
		stRet = NOT_OK;				
	} else {
		stRet = OK;					
	}

	delay_us(2);
	WRITE_SCL_0;
	delay_us(5);
	
	return stRet;
}

uint8_t I2C_Read(ACK_Bit _ACK){	
	uint8_t i;		
  status u8set;	
	uint8_t u8Ret = 0x00;
	WRITE_SDA_1;
	delay_us(3);	
	for (i = 0; i < 8; ++i) {
		u8Ret <<= 1;
		
		//SCL Clock
		WRITE_SCL_1;
		delay_us(3);
		if (READ_SDA_VAL) {
			u8Ret |= 0x01;
		}
		delay_us(2);
		WRITE_SCL_0;
		delay_us(5);
	}

	if (_ACK) {	
		WRITE_SDA_0;
	} else {
		WRITE_SDA_1;
	}
	delay_us(3);
	
	WRITE_SCL_1;
	delay_us(5);
	WRITE_SCL_0;
	delay_us(5);

	return u8Ret;
}

status EPROM_Read(uint16_t MemAddr, uint8_t SlaveAddress, uint8_t NumByte, uint8_t *u8Data ){
	uint8_t i;
	I2C_Start();
	if (I2C_Write(SlaveAddress << 1) == NOT_OK) {
		I2C_Stop();
		return NOT_OK;
	}
	if (I2C_Write(MemAddr >> 8) == NOT_OK) {
    I2C_Stop();
    return NOT_OK;
}

if (I2C_Write(MemAddr) == NOT_OK) {
    I2C_Stop();
    return NOT_OK;
}
	I2C_Start();
if (I2C_Write((SlaveAddress << 1) | 1) == NOT_OK) {
    I2C_Stop();
    return NOT_OK;
}

for (i = 0; i < NumByte - 1; ++i) {
    u8Data[i] = I2C_Read(ACK); // Ð?c các byte và g?i ACK cho m?i byte, tr? byte cu?i
}

		u8Data[i] = I2C_Read(NACK); // Ð?c byte cu?i và g?i NACK

I2C_Stop();

return OK;

}
status EPROM_Write(uint16_t MemAddr, uint8_t SlaveAddress, uint8_t NumByte, uint8_t *u8Data ){
uint8_t i;
    I2C_Start();
		
    if (I2C_Write(SlaveAddress << 1) == NOT_OK) {
        I2C_Stop();
        return NOT_OK;
    }

    if (I2C_Write((MemAddr + i) >> 8) == NOT_OK) {
        I2C_Stop();
        return NOT_OK;
    }

    if (I2C_Write(MemAddr + i) == NOT_OK) {
        I2C_Stop();
        return NOT_OK;
    }
for (i = 0; i < NumByte; ++i) {
    if (I2C_Write(u8Data[i]) == NOT_OK) {
        I2C_Stop();
        return NOT_OK;
    }
	}
    I2C_Stop();
		delay_us(10);

 return OK;
}
	

// Khai bao mang du lieu de ghi vao EEPROM
uint8_t Data1[10] = {0x03, 0x05, 0x0E, 0xDA, 0xA6, 0x6F, 0x50, 0x00, 0x00, 0xF0};
uint8_t Data2[10] = {0x19, 0x0A, 0x19, 0x24, 0xFA, 0x10, 0x3C, 0x48, 0x59, 0x77};
uint8_t Rcv[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Ham chinh
int main() {
    // C?u hình h? th?ng
    RCC_Config();
    TIM2_Config();
    GPIO_Config();
    I2C_Config();

    // Ghi d? li?u t? Data1 vào EEPROM t?i d?a ch? 0x0045
    while (EPROM_Write(0x0045, 0x57, 10, Data1) == NOT_OK) {
        // Th? ghi l?i n?u th?t b?i
    }

    // Thêm delay gi?a hai l?n truy?n
    //Delay_Ms(500); // Thêm 500 ms delay, có th? di?u ch?nh theo yêu c?u

    // Sau khi ghi xong Data1, ti?p t?c ghi Data2
    while (EPROM_Write(0x0060, 0x57, 10, Data2) == NOT_OK) {
        // Th? ghi l?i n?u th?t b?i
    }

    // Ð?c l?i d? li?u t? EEPROM d? ki?m tra
    while (1) {
        // Ð?c 10 byte t? d?a ch? 0x0045 vào m?ng Rcv
        while (EPROM_Read(0x0045, 0x57, 10, Rcv) == NOT_OK) {
            // Th? d?c l?i n?u th?t b?i
        }
				Delay_Ms(500);
        // Ð?c 10 byte t? d?a ch? 0x0060 vào m?ng Rcv
        while (EPROM_Read(0x0060, 0x57, 10, Rcv) == NOT_OK) {
            // Th? d?c l?i n?u th?t b?i
        }

        Delay_Ms(1000); // Ð?i 1 giây tru?c khi d?c l?i
    }
}



















