#include "stm32f1xx_hal.h"
#include "IIC.h"

//****************************Micro*******************************/
#define	READ_CW2015				0xc5
#define	WRITE_CW2015			0xc4

#define I2C_DELAY		2

//***************************function*******************************/
void delay_10us()
{
	unsigned char a,b;
	for(b=1;b>0;b--)
		for(a=7;a>0;a--);
}


void start(void)			
{
	SDA_SET_OUT();
	SDA_Pin_OUT(1);				
	SCL_Pin_OUT(1);			
	delay_10us();
	SDA_Pin_OUT(0);				
	delay_10us();
	SCL_Pin_OUT(0);			
}
//-------------------------------------------------------------------
void stop(void)				
{
	SDA_SET_OUT();
	SDA_Pin_OUT(0);			
	delay_10us();
	SCL_Pin_OUT(1);			
	delay_10us();
	SDA_Pin_OUT(1);			
	delay_10us();
	//	SCL_Pin_OUT(0);
	delay_10us();
}
//-------------------------------------------------------------------
unsigned char r_ack(void)				//接收应答信号函数
{
  unsigned char ack;				//定义一个位变量，来暂存应答状态。
  SDA_SET_IN();
	SCL_Pin_OUT(1);				//拉高时钟线。
	delay_10us();
	ack = SDA_Pin_IN();				//读取应答信号的状态。
	delay_10us();
	SCL_Pin_OUT(0);				//拉低时钟线。
	delay_10us();
	return ack;				//返回应答信号的状态，0表示应答，1表示非应答。
}

unsigned char wait_r_ack(void)
{
	unsigned char ack;
	unsigned int i;	
	ack = 1;
	for(i = 0; i < 100; i++)
	{
		ack = r_ack();
		if(!ack)
		{
			break;
		}
		if(i == 99)
		{
			stop();
			return 1;
		}
	}
	return 0;
}
//---------------------------------------`----------------------------
void s_ack(void)				//发送应答信号函数
{
	SDA_SET_OUT();
	SDA_Pin_OUT(1);				//释放数据总线，准备接收应答信号。
	delay_10us();
	SCL_Pin_OUT(1);				//拉高时钟线。
	delay_10us();
	SCL_Pin_OUT(0);				//拉低时钟线。
	delay_10us();

}

void sn_ack(void)				//Master发送准备连读指令
{
	SDA_SET_OUT();
	SDA_Pin_OUT(0);
	delay_10us();
	SCL_Pin_OUT(1);
	delay_10us();
	SCL_Pin_OUT(0);
	SDA_Pin_OUT(1);
	delay_10us();
}

//-------------------------------------------------------------------
void write_byte(unsigned char w_data)		//向IIC总线写入一个字节的数据函数 
{
	unsigned char i;
	SDA_SET_OUT();
	for(i = 0; i < 8; i++)				//有8位数据
	{
		if(w_data & 0x80)
			SDA_Pin_OUT(1);
		else
			SDA_Pin_OUT(0);
		delay_10us();
		SCL_Pin_OUT(1);						//拉高时钟线，将数写入到设备中。
		delay_10us();
		SCL_Pin_OUT(0);						//拉低时钟线，允许改变数据线的状态
		delay_10us();
		w_data = w_data << 1;				//数据左移一位，把次高位放在最高位,为写入次高位做准备
  }
}
//-------------------------------------------------------------------
unsigned char read_byte(void)			//从IIC总线读取一个字节的数据函数
{
	unsigned char i;
	unsigned char r_data;					//定义一个缓冲寄存器。
	SDA_SET_IN();
	for(i = 0; i < 8; i++)				//有8位数据
	{
		SCL_Pin_OUT(1);						//拉高时钟线，为读取下一位数据做准备。
		delay_10us();
		r_data = r_data << 1;				//将缓冲字节的数据左移一位，准备读取数据。
		delay_10us();
		if(SDA_Pin_IN())							//如果数据线为高平电平。
		{
			r_data = r_data | 0x1;			//则给缓冲字节的最低位写1。
		}
		SCL_Pin_OUT(0);						//拉低时钟线，为读取下一位数据做准备。
		delay_10us();
	}	
	return r_data;						//返回读取的一个字节数据。
}

//------------------------IIC读写函数--------------------------------
unsigned char cw_read(unsigned char point_reg,unsigned char *r_pdata)
{
	start();
	write_byte(WRITE_CW2015);
	if(wait_r_ack())
	{
		return 1;
	}
	write_byte(point_reg);
	if(wait_r_ack())
	{
		return 1;
	}
	start();
	write_byte(READ_CW2015);
	if(wait_r_ack())
	{
		return 1;
	}
	*r_pdata = read_byte();
	s_ack();
	stop();
	return 0;
}

unsigned char cw_read_word(unsigned char point_reg,unsigned char *r_pdata, unsigned int length)
{
	unsigned char count;

	start();
	write_byte(WRITE_CW2015);
	if(wait_r_ack())
	{
		return 1;
	}
	write_byte(point_reg);
	if(wait_r_ack())
	{
		return 1;
	}
	start();
	write_byte(READ_CW2015);
	if(wait_r_ack())
	{
		return 1;
	}
	for(count = 0; count < length; count++ ){
		*r_pdata = read_byte();
		if(count + 1 < length){
			r_pdata++;
			sn_ack();
		}
	}
	s_ack();
	stop();
	return 0;
}

unsigned char cw_write(unsigned char point_reg,unsigned char *w_pdata)
{
	start();
	write_byte(WRITE_CW2015);
	if(wait_r_ack())
	{
		return 1;
	}
	write_byte(point_reg);
	if(wait_r_ack())
	{
		return 1;
	}
	write_byte(*w_pdata);
	if(wait_r_ack())
	{
		return 1;
	}
	stop();
	return 0;
}
