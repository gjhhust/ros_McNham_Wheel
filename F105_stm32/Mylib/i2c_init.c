#include "main.h"
int  I2CTimeOut=1000;  

void iic_Start(void)
{
	IIC_SDA_1();
	IIC_SCL_1();
	delay_us(5);
	IIC_SDA_0();
	delay_us(5);
	IIC_SCL_0();
}


//SCL高 SDA上升沿 --> 停止信号
void iic_Stop(void)
{
	IIC_SDA_0();
	IIC_SCL_1();
  delay_us(5);
	IIC_SDA_1();
	delay_us(5);
}
void iic_WriteByte(uint8_t _ucByte)
{
	uint8_t i;

	//从最高位开始发送
	for(i = 0; i < 8; i ++)
	{
		if(_ucByte & 0x80)
			IIC_SDA_1();

		else
			IIC_SDA_0();

	  delay_us(5);
		IIC_SCL_1();
		delay_us(5);
		IIC_SCL_0();
		
		if (i == 7)
			//释放总线
			IIC_SDA_1(); 

		_ucByte <<= 1;
		
	 delay_us(5);
	}
}


u8 iic_ReadByte()
{
	u8 i;
	u8 value = 0;

	//最先读到的为最高位
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		
		IIC_SCL_1();
		delay_us(5);
		
		if (IIC_SDA_READ())
			value++;

		IIC_SCL_0();
		delay_us(5);
	}
	
		iic_NAck();

	return value;
}

//返回值为1则未应答  0为应答
uint8_t iic_WaitAck(void)
{
	uint8_t re;

	//CPU释放SDA总线
	IIC_SDA_1();	
  delay_us(5);
	
	//CPU驱动SCL=1 
	//此时器件会返回ACK应答
	IIC_SCL_1();
	delay_us(5);
	
	if (IIC_SDA_READ())	
		re = 1;

	else
		re = 0;

	IIC_SCL_0();
	delay_us(5);
	
	return re;
}


void iic_Ack(void)
{
	//CPU驱动SDA=0
	IIC_SDA_0();
  delay_us(5);
	
	//CPU产生1个时钟
	IIC_SCL_1();	
	delay_us(5);
	IIC_SCL_0();
	delay_us(5);
	
	//CPU释放SDA总线
	IIC_SDA_1();	
}


void iic_NAck(void)
{
	//CPU驱动SDA=1
	IIC_SDA_1();	
	delay_us(5);
	
	//CPU产生1个时钟
	IIC_SCL_1();	
	delay_us(5);
	IIC_SCL_0();
	delay_us(5);	
}


void iic_gpio_init(void)
{
	GPIO_InitTypeDef 	gpio;

	RCC_APB2PeriphClockCmd(RCC_IIC_PORT,ENABLE);

	gpio.GPIO_Pin = IIC_SCL_PIN;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIO_PORT_IIC,&gpio);
	
	gpio.GPIO_Pin = IIC_SDA_PIN;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_Out_OD;  
	GPIO_Init(GPIO_PORT_IIC,&gpio);

	//停止信号
	//复位IIC总线上的所有设备到待机模式
	iic_Stop();
}


uint8_t iic_init(void)
{
	uint8_t ucAck;

	iic_gpio_init();
	
	iic_Start();

	//发送设备地址+读写控制位
	//读写控制位：(0 = w ; 1 = r)
	iic_WriteByte((0x68<<1)|IIC_WR);
	
	ucAck = iic_WaitAck();	

	iic_Stop();

	return ucAck;
}

void iic_write_REG(uint8_t REG,uint8_t data)
{
a:
   iic_Start();
	 iic_WriteByte(SLAVE_ADDRESS<<1|0);
	 iic_WaitAck();
   iic_WriteByte(REG);
	 if(iic_WaitAck())
	 {
	   iic_Stop();
		 goto a;
	 }
	iic_WriteByte(data);
	 if(iic_WaitAck())
	 {
	   iic_Stop();
		 goto a;
	 }	 
}

u16 iic_read_REG(uint8_t REG)
{
	iic_Start();
	iic_WriteByte(SLAVE_ADDRESS<<1|0);
	iic_WaitAck();
	iic_WriteByte(REG);
	iic_WaitAck();
	iic_Stop();
	iic_Start();
	iic_WriteByte(SLAVE_ADDRESS<<1|1);
	return iic_ReadByte();
}



