#ifndef __I2C_INIT_H
#define __I2C_INIT_H

#include "stm32f10x.h"
#include "stm32f10x_i2c.h"

//AD0接地时为0x68  AD0接电源时为0x69
#define SLAVE_ADDRESS   0x68  

#define GPIO_PORT_IIC	GPIOB						
#define RCC_IIC_PORT 	RCC_APB2Periph_GPIOB		
#define IIC_SCL_PIN		GPIO_Pin_6		
#define IIC_SDA_PIN		GPIO_Pin_7

#define IIC_SCL_1()  GPIO_SetBits(GPIO_PORT_IIC,IIC_SCL_PIN)				
#define IIC_SCL_0()  GPIO_ResetBits(GPIO_PORT_IIC,IIC_SCL_PIN)				

#define IIC_SDA_1()  GPIO_SetBits(GPIO_PORT_IIC,IIC_SDA_PIN)				
#define IIC_SDA_0()  GPIO_ResetBits(GPIO_PORT_IIC,IIC_SDA_PIN)				

#define IIC_SDA_READ()  GPIO_ReadInputDataBit(GPIO_PORT_IIC,IIC_SDA_PIN)

#define IIC_WR	0
#define IIC_RD	1	
void iic_Delay(void);
void iic_Start(void);
void iic_Stop(void);
void iic_WriteByte(uint8_t _ucByte);
uint8_t iic_ReadByte(void);
void I2C_Mode_Config(void);
void I2C_GPIO_Config(void);
uint8_t iic_WaitAck(void);
void iic_Ack(void);
void iic_NAck(void);
void iic_gpio_init(void);
uint8_t iic_init(void);
void iic_write_REG(uint8_t REG,uint8_t data);
u16 iic_read_REG(uint8_t REG);


#endif 



