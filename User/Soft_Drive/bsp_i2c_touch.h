#ifndef __I2C_TOUCH_H
#define __I2C_TOUCH_H

#include "gd32f4xx.h"

/*使用软件IIC，把SOFT_IIC宏设置为1，硬件IIC则设置为0
!!使用硬件IIC时非常容易出现错误，不推荐*/
#define SOFT_IIC      1

/*设定使用的电容屏IIC设备地址*/
#define GTP_ADDRESS            0xBA

#define I2CT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define I2CT_LONG_TIMEOUT         ((uint32_t)(10 * I2CT_FLAG_TIMEOUT))

/*I2C引脚*/
#define GTP_I2C                          I2C1
#define GTP_I2C_CLK                      RCU_I2C1
#define GTP_I2C_CLK_INIT                 rcu_periph_clock_enable

#define GTP_I2C_SCL_PIN                  GPIO_PIN_10                 
#define GTP_I2C_SCL_GPIO_PORT            GPIOB                       
#define GTP_I2C_SCL_GPIO_CLK             RCU_GPIOB
//#define GTP_I2C_SCL_SOURCE               GPIO_PinSource4
//#define GTP_I2C_SCL_AF                   GPIO_AF_I2C2

#define GTP_I2C_SDA_PIN                  GPIO_PIN_11                  
#define GTP_I2C_SDA_GPIO_PORT            GPIOB                     
#define GTP_I2C_SDA_GPIO_CLK             RCU_GPIOB
//#define GTP_I2C_SDA_SOURCE               GPIO_PinSource5
//#define GTP_I2C_SDA_AF                   GPIO_AF_I2C2

/*复位引脚*/
#define GTP_RST_GPIO_PORT                GPIOA
#define GTP_RST_GPIO_CLK                 RCU_GPIOA
#define GTP_RST_GPIO_PIN                 GPIO_PIN_7
#define GTP_RST_1()                      gpio_bit_set(GTP_RST_GPIO_PORT,GTP_RST_GPIO_PIN)
#define GTP_RST_0()                      gpio_bit_reset(GTP_RST_GPIO_PORT,GTP_RST_GPIO_PIN)

/*中断引脚*/
#define GTP_INT_GPIO_PORT                GPIOA
#define GTP_INT_GPIO_CLK                 RCU_GPIOA
#define GTP_INT_GPIO_PIN                 GPIO_PIN_3
#define GTP_INT_EXTI_PORTSOURCE          EXTI_SOURCE_GPIOA
#define GTP_INT_EXTI_PINSOURCE           EXTI_SOURCE_PIN3
#define GTP_INT_EXTI_LINE                EXTI_3
#define GTP_INT_EXTI_IRQ                 EXTI3_IRQn
#define GTP_INT_1()                      gpio_bit_set(GTP_INT_GPIO_PORT,GTP_INT_GPIO_PIN)
#define GTP_INT_0()                      gpio_bit_reset(GTP_INT_GPIO_PORT,GTP_INT_GPIO_PIN)

/*中断服务函数*/
#define GTP_IRQHandler                   EXTI3_IRQHandler


//软件IIC使用的宏
#define I2C_SCL_1()  gpio_bit_set(GTP_I2C_SCL_GPIO_PORT, GTP_I2C_SCL_PIN)          /* SCL = 1 */
#define I2C_SCL_0()  gpio_bit_reset(GTP_I2C_SCL_GPIO_PORT, GTP_I2C_SCL_PIN)        /* SCL = 0 */

#define I2C_SDA_1()  gpio_bit_set(GTP_I2C_SDA_GPIO_PORT, GTP_I2C_SDA_PIN)          /* SDA = 1 */
#define I2C_SDA_0()  gpio_bit_reset(GTP_I2C_SDA_GPIO_PORT, GTP_I2C_SDA_PIN)        /* SDA = 0 */

#define I2C_SDA_READ()  gpio_input_bit_get(GTP_I2C_SDA_GPIO_PORT, GTP_I2C_SDA_PIN)    /* 读SDA口线状态 */

//函数接口
void I2C_Touch_Init(void);
uint32_t I2C_WriteBytes(uint8_t ClientAddr,uint8_t* pBuffer,  uint8_t NumByteToWrite);
uint32_t I2C_ReadBytes(uint8_t ClientAddr,uint8_t* pBuffer, uint16_t NumByteToRead);
void I2C_ResetChip(void);
void I2C_GTP_IRQDisable(void);
void I2C_GTP_IRQEnable(void);

void GT911_WriteReg(uint16_t _usRegAddr, uint8_t *_pRegBuf, uint8_t _ucLen);
void GT911_ReadReg(uint16_t _usRegAddr, uint8_t *_pRegBuf, uint8_t _ucLen);

uint8_t Touch_PenIRQ(void); 

#endif /* __I2C_TOUCH_H */
