#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#define EEPROM_ADDRESS  0x57       // Dia chi I2C cua AT24C32
#define I2C_SPEED       100000     // Toc do I2C 100kHz
#define I2C_PORT        I2C1       // Su dung I2C1
#define EEPROM_I2C_RCC  RCC_APB1Periph_I2C1
#define EEPROM_GPIO_RCC RCC_APB2Periph_GPIOB
#define EEPROM_GPIO     GPIOB
#define EEPROM_SCL      GPIO_Pin_6
#define EEPROM_SDA      GPIO_Pin_7

void I2C_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    // Bat clock cho I2C va GPIO
    RCC_APB1PeriphClockCmd(EEPROM_I2C_RCC, ENABLE);
    RCC_APB2PeriphClockCmd(EEPROM_GPIO_RCC, ENABLE);

    // Cau hinh chan I2C SDA va SCL
    GPIO_InitStructure.GPIO_Pin = EEPROM_SCL | EEPROM_SDA;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(EEPROM_GPIO, &GPIO_InitStructure);

    // Cau hinh I2C
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;

    I2C_Init(I2C_PORT, &I2C_InitStructure);
    I2C_Cmd(I2C_PORT, ENABLE);
}

// Cau hinh TIM2 de tao do tre
void TIM2_Config(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 72 - 1;  // Chia tan so 72 MHz xuong 1 MHz (1 µs moi dem)
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period = 0xFFFF;     // Dem den gia tri toi da (16-bit)
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

    TIM_Cmd(TIM2, ENABLE);
}

void Delay_Us(uint16_t us) {
    TIM_SetCounter(TIM2, 0);  // Dat bo dem cua TIM2 ve 0
    while (TIM_GetCounter(TIM2) < us);
}

void Delay_Ms(uint16_t ms) {
    for (uint16_t i = 0; i < ms; i++) {
        Delay_Us(1000);  // Moi lan goi Delay_Us(1000) se tao ra do tre 1 ms
    }
}

void EEPROM_Write(uint16_t MemAddress, uint8_t *data, uint8_t length) {
    // Bat dau giao tiep I2C
    I2C_GenerateSTART(I2C_PORT, ENABLE);
    while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT));

    // Gui dia chi cua EEPROM voi bit ghi
    I2C_Send7bitAddress(I2C_PORT, EEPROM_ADDRESS << 1, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    // Gui byte cao cua dia chi bo nho
    I2C_SendData(I2C_PORT, (uint8_t)(MemAddress >> 8));
    while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    // Gui byte thap cua dia chi bo nho
    I2C_SendData(I2C_PORT, (uint8_t)(MemAddress & 0xFF));
    while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    // Ghi du lieu vao EEPROM
    for (uint8_t i = 0; i < length; i++) {
        I2C_SendData(I2C_PORT, data[i]);
        while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    }

    // Ket thuc giao tiep I2C
    I2C_GenerateSTOP(I2C_PORT, ENABLE);
    Delay_Ms(5);  // Doi de EEPROM hoan thanh viec ghi
}

void EEPROM_Read(uint16_t MemAddress, uint8_t *data, uint8_t length) {
    // Bat dau giao tiep I2C
    I2C_GenerateSTART(I2C_PORT, ENABLE);
    while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT));

    // Gui dia chi cua EEPROM voi bit ghi de chi dinh dia chi can doc
    I2C_Send7bitAddress(I2C_PORT, EEPROM_ADDRESS << 1, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    // Gui byte cao cua dia chi bo nho
    I2C_SendData(I2C_PORT, (uint8_t)(MemAddress >> 8));
    while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    // Gui byte thap cua dia chi bo nho
    I2C_SendData(I2C_PORT, (uint8_t)(MemAddress & 0xFF));
    while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    // Bat dau lai giao tiep I2C, chuyen sang che do doc
    I2C_GenerateSTART(I2C_PORT, ENABLE);
    while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT));

    // Gui dia chi cua EEPROM voi bit doc
    I2C_Send7bitAddress(I2C_PORT, EEPROM_ADDRESS << 1, I2C_Direction_Receiver);
    while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    // Doc du lieu tu EEPROM
    for (uint8_t i = 0; i < length - 1; i++) {
        while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_BYTE_RECEIVED));
        data[i] = I2C_ReceiveData(I2C_PORT);
        I2C_AcknowledgeConfig(I2C_PORT, ENABLE);
    }

    // Doc byte cuoi cung va gui NACK
    while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_BYTE_RECEIVED));
    data[length - 1] = I2C_ReceiveData(I2C_PORT);
    I2C_AcknowledgeConfig(I2C_PORT, DISABLE);

    I2C_GenerateSTOP(I2C_PORT, ENABLE);
}

// Khai bao mang du lieu de ghi va doc tu EEPROM
uint8_t Data1[10] = {0x03, 0x05, 0x0E, 0xDA, 0xA6, 0x6F, 0x50, 0x00, 0x00, 0xF0};
uint8_t Data2[10] = {0x05, 0x0A, 0x19, 0x24, 0xFA, 0x10, 0x3C, 0x48, 0x59, 0x77};
uint8_t Rcv[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

int main() {
    // Cau hinh I2C va TIM2
    I2C_Config();
    TIM2_Config();

    // Ghi du lieu tu Data1 vao EEPROM tai dia chi 0x0045
    EEPROM_Write(0x0045, Data1, 10);
    Delay_Ms(10);  // Doi 10 ms de dam bao ghi hoan tat

    // Ghi du lieu tu Data2 vao EEPROM tai dia chi 0x0060
   EEPROM_Write(0x0060, Data2, 10);
    Delay_Ms(10);  // Doi 10 ms de dam bao ghi hoan tat

    // Doc lai du lieu tu EEPROM de kiem tra
   while (1) {
    // Ð?c toàn b? Data1 t? d?a ch? 0x0045 vào m?ng Rcv
    for (int i = 0; i < 10; i++) {
        EEPROM_Read(0x0045 + i, &Rcv[i], 1); // Ð?c t?ng byte t? Data1
        Delay_Ms(10); // Delay nh? gi?a m?i byte d?c
    }

    Delay_Ms(500); // Thêm 500ms delay gi?a d?c Data1 và Data2

    // Ð?c toàn b? Data2 t? d?a ch? 0x0060 vào m?ng Rcv
    for (int i = 0; i < 10; i++) {
        EEPROM_Read(0x0060 + i, &Rcv[i], 1); // Ð?c t?ng byte t? Data2
        Delay_Ms(10); // Delay nh? gi?a m?i byte d?c
    }
		
	

    Delay_Ms(1000); // Ð?i 1 giây tru?c khi d?c l?i
		
}   
	 }
