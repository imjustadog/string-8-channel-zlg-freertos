#ifndef __I2C_H__
#define __I2C_H__

#define IIC_SDA0    GPIOB->BSRR=(uint32_t)GPIO_PIN_7<<16U
#define IIC_SDA1    GPIOB->BSRR=GPIO_PIN_7
#define IIC_SCL0    GPIOB->BSRR=(uint32_t)GPIO_PIN_6<<16U
#define IIC_SCL1    GPIOB->BSRR=GPIO_PIN_6

#define SDA_SET_IN()	  GPIOB->CRL &= ~(0x03 << 28)
#define SDA_SET_OUT()   GPIOB->CRL |= (0X01 << 28)

#define noACK 0            
#define ACK   1            

#define SCL_Pin_OUT(x) do{if(x){IIC_SCL1;}else{IIC_SCL0;}}while(0)
#define SDA_Pin_OUT(x) do{if(x){IIC_SDA1;}else{IIC_SDA0;}}while(0)
#define SDA_Pin_IN() ((GPIOB->IDR&GPIO_PIN_7)!=0)?1:0

void delay_us(void);

#endif
