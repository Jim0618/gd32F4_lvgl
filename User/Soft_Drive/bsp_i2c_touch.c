/**
  ******************************************************************************
  * @file    bsp_i2c_touch.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   电容触摸屏的专用iic驱动
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火  STM32 F429 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "bsp_i2c_touch.h"
#include "gt9xx.h"
//#include "lvgl/lv_hal/lv_hal.h"
#include "systick.h"


extern uint32_t g_inputx;
extern uint32_t g_inputy;

/* STM32 I2C 快速模式 */
#define I2C_Speed              400000

/* 这个地址只要与STM32外挂的I2C器件地址不一样即可 */
#define I2C_OWN_ADDRESS7      0x0A

static void Delay(__IO uint32_t nCount)     //简单的延时函数
{
    for(; nCount != 0; nCount--);
}


/**
  * @brief  使能触摸屏中断
  * @param  无
  * @retval 无
  */
void I2C_GTP_IRQEnable(void)
{
    /*配置 INT 为浮空输入 */   
    gpio_mode_set(GTP_INT_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GTP_INT_GPIO_PIN);
    
    /* enable and set EXTI interrupt priority */
    nvic_irq_enable(GTP_INT_EXTI_IRQ, 2U, 0U);
    
    /* 连接 EXTI 中断源 到INT 引脚 */
    rcu_periph_clock_enable(RCU_SYSCFG);
    syscfg_exti_line_config(GTP_INT_EXTI_PORTSOURCE, GTP_INT_EXTI_PINSOURCE);
    
    /* configure key EXTI line */
    exti_init(GTP_INT_EXTI_LINE, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_flag_clear(GTP_INT_EXTI_LINE);
    
    delay_1ms(50);
}

/**
  * @brief  关闭触摸屏中断
  * @param  无
  * @retval 无
  */
void I2C_GTP_IRQDisable(void)
{
    /*配置 INT 为浮空输入 */   
    gpio_mode_set(GTP_INT_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GTP_INT_GPIO_PIN);
    
    rcu_periph_clock_enable(RCU_SYSCFG);
    /* 连接 EXTI 中断源 到INT 引脚 */
    syscfg_exti_line_config(GTP_INT_EXTI_PORTSOURCE, GTP_INT_EXTI_PINSOURCE);
    
    /* configure key EXTI line */
    exti_init(GTP_INT_EXTI_LINE, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_disable(GTP_INT_EXTI_LINE);
    /* disable touch panel EXTI interrupt */
    nvic_irq_disable(GTP_INT_EXTI_IRQ);
}

/**
  * @brief  触摸屏 I/O配置
  * @param  无
  * @retval 无
  */
static void I2C_GPIO_Config(void)
{
#if (SOFT_IIC)   //使用软件IIC
    /*配置SCL引脚 */
    rcu_periph_clock_enable(GTP_I2C_SCL_GPIO_CLK);
    gpio_mode_set(GTP_I2C_SCL_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GTP_I2C_SCL_PIN);
    gpio_output_options_set(GTP_I2C_SCL_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GTP_I2C_SCL_PIN);

    /*配置SDA引脚 */
    rcu_periph_clock_enable(GTP_I2C_SDA_GPIO_CLK);
    gpio_mode_set(GTP_I2C_SDA_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GTP_I2C_SDA_PIN);
    gpio_output_options_set(GTP_I2C_SDA_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GTP_I2C_SDA_PIN);
#endif
 
    /*配置RST引脚，下拉推挽输出 */
    rcu_periph_clock_enable(GTP_RST_GPIO_CLK);
    gpio_mode_set(GTP_RST_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GTP_RST_GPIO_PIN);
    gpio_output_options_set(GTP_RST_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GTP_RST_GPIO_PIN);
  
    /*配置 INT引脚，下拉推挽输出，方便初始化 */   
    //设置为下拉，方便初始化
    rcu_periph_clock_enable(GTP_INT_GPIO_CLK);
    gpio_mode_set(GTP_INT_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GTP_INT_GPIO_PIN);
    gpio_output_options_set(GTP_INT_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GTP_INT_GPIO_PIN);
}


/**
  * @brief  对GT91xx芯片进行复位
  * @param  无
  * @retval 无
  */
void I2C_ResetChip(void)
{
    /*初始化GT9157,rst为高电平，int为低电平，则gt9157的设备地址被配置为0xBA*/
    GTP_RST_0();
    GTP_INT_0();
    delay_1ms(1);

    /*init low >5ms*/
    GTP_RST_1();
    delay_1ms(5);
}

/**
  * @brief  I2C 外设(GT91xx)初始化
  * @param  无
  * @retval 无
  */
void I2C_Touch_Init(void)
{
    I2C_GPIO_Config(); 
 
    I2C_ResetChip();

    I2C_GTP_IRQEnable();
}


#if (SOFT_IIC)   //使用软件IIC     

/*
*********************************************************************************************************
*    函 数 名: i2c_Delay
*    功能说明: I2C总线位延迟，最快400KHz
*    形    参：无
*    返 回 值: 无
*********************************************************************************************************
*/
static void i2c_Delay(void)
{
    uint8_t i;

    /*　
      下面的时间是通过逻辑分析仪测试得到的。
      工作条件：CPU主频180MHz ，MDK编译环境，1级优化
      
      循环次数为50时，SCL频率 = 333KHz 
      循环次数为30时，SCL频率 = 533KHz，  
      循环次数为20时，SCL频率 = 727KHz， 
    */
    for (i = 0; i < 10*5; i++);
}

void i2c_Start(void)
{
    /* 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号 */
    I2C_SDA_1();
    I2C_SCL_1();
    i2c_Delay();
    I2C_SDA_0();
    i2c_Delay();
    I2C_SCL_0();
    i2c_Delay();
}

void i2c_Stop(void)
{
    /* 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号 */
    I2C_SDA_0();
    I2C_SCL_1();
    i2c_Delay();
    I2C_SDA_1();
}

void i2c_SendByte(uint8_t _ucByte)
{
    uint8_t i;

    /* 先发送字节的高位bit7 */
    for (i = 0; i < 8; i++){
        if (_ucByte & 0x80){
            I2C_SDA_1();
        }else{
            I2C_SDA_0();
        }
        i2c_Delay();
        I2C_SCL_1();
        i2c_Delay();
        I2C_SCL_0();
        if (i == 7){
            I2C_SDA_1(); // 释放总线
        }
        _ucByte <<= 1;    /* 左移一个bit */
        i2c_Delay();
    }
}

/*
*********************************************************************************************************
*    函 数 名: i2c_ReadByte
*    功能说明: CPU从I2C总线设备读取8bit数据
*    形    参：无
*    返 回 值: 读到的数据
*********************************************************************************************************
*/
uint8_t i2c_ReadByte(void)
{
    uint8_t i;
    uint8_t value;

    /* 读到第1个bit为数据的bit7 */
    value = 0;
    for (i = 0; i < 8; i++){
        value <<= 1;
        I2C_SCL_1();
        i2c_Delay();
        if (I2C_SDA_READ()){
            value++;
        }
        I2C_SCL_0();
        i2c_Delay();
    }
    return value;
}

/*
*********************************************************************************************************
*    函 数 名: i2c_WaitAck
*    功能说明: CPU产生一个时钟，并读取器件的ACK应答信号
*    形    参：无
*    返 回 值: 返回0表示正确应答，1表示无器件响应
*********************************************************************************************************
*/
uint8_t i2c_WaitAck(void)
{
    uint8_t re;

    I2C_SDA_1();    /* CPU释放SDA总线 */
    i2c_Delay();
    I2C_SCL_1();    /* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
    i2c_Delay();
    if (I2C_SDA_READ())    /* CPU读取SDA口线状态 */{
        re = 1;
    }else{
        re = 0;
    }
    I2C_SCL_0();
    i2c_Delay();
    return re;
}

/*
*********************************************************************************************************
*    函 数 名: i2c_Ack
*    功能说明: CPU产生一个ACK信号
*    形    参：无
*    返 回 值: 无
*********************************************************************************************************
*/
void i2c_Ack(void)
{
    I2C_SDA_0();    /* CPU驱动SDA = 0 */
    i2c_Delay();
    I2C_SCL_1();    /* CPU产生1个时钟 */
    i2c_Delay();
    I2C_SCL_0();
    i2c_Delay();
    I2C_SDA_1();    /* CPU释放SDA总线 */
}

/*
*********************************************************************************************************
*    函 数 名: i2c_NAck
*    功能说明: CPU产生1个NACK信号
*    形    参：无
*    返 回 值: 无
*********************************************************************************************************
*/
void i2c_NAck(void)
{
    I2C_SDA_1();    /* CPU驱动SDA = 1 */
    i2c_Delay();
    I2C_SCL_1();    /* CPU产生1个时钟 */
    i2c_Delay();
    I2C_SCL_0();
    i2c_Delay();    
}

#define I2C_DIR_WR    0        /* 写控制bit */
#define I2C_DIR_RD    1        /* 读控制bit */

/**
  * @brief   使用IIC读取数据
  * @param   
  *     @arg ClientAddr:从设备地址
  *        @arg pBuffer:存放由从机读取的数据的缓冲区指针
  *        @arg NumByteToRead:读取的数据长度
  * @retval  无
  */
uint32_t I2C_ReadBytes(uint8_t ClientAddr,uint8_t* pBuffer, uint16_t NumByteToRead)
{
    /* 第1步：发起I2C总线启动信号 */
    i2c_Start();
    
    /* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
    i2c_SendByte(ClientAddr | I2C_DIR_RD);    /* 此处是读指令 */
    
    /* 第3步：等待ACK */
    if (i2c_WaitAck() != 0)
    {
        goto cmd_fail;    /* 器件无应答 */
    }

    while(NumByteToRead) 
    {
        if(NumByteToRead == 1){
            i2c_NAck();    /* 最后1个字节读完后，CPU产生NACK信号(驱动SDA = 1) */
      
            /* 发送I2C总线停止信号 */
            i2c_Stop();
        }
    
        *pBuffer = i2c_ReadByte();
    
        /* 读指针自增 */
        pBuffer++; 
      
        /*计数器自减 */
        NumByteToRead--;
    
        i2c_Ack();    /* 中间字节读完后，CPU产生ACK信号(驱动SDA = 0) */  
    }

    /* 发送I2C总线停止信号 */
    i2c_Stop();
    return 0;    /* 执行成功 */

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
    /* 发送I2C总线停止信号 */
    i2c_Stop();
    return 1;
}

/**
  * @brief   使用IIC写入数据
  * @param   
  *     @arg ClientAddr:从设备地址
  *        @arg pBuffer:缓冲区指针
  *     @arg NumByteToWrite:写的字节数
  * @retval  无
  */
uint32_t I2C_WriteBytes(uint8_t ClientAddr,uint8_t* pBuffer, uint8_t NumByteToWrite)
{
    uint16_t m;    

    /*　第0步：发停止信号，启动内部写操作　*/
    i2c_Stop();
  
    /* 通过检查器件应答的方式，判断内部写操作是否完成, 一般小于 10ms
        CLK频率为200KHz时，查询次数为30次左右
    */
    for (m = 0; m < 1000; m++){
        /* 第1步：发起I2C总线启动信号 */
        i2c_Start();
        
        /* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
        i2c_SendByte(ClientAddr | I2C_DIR_WR);    /* 此处是写指令 */
        
        /* 第3步：发送一个时钟，判断器件是否正确应答 */
        if (i2c_WaitAck() == 0){
            break;
        }
    }
    if (m  == 1000){
        goto cmd_fail;    /* EEPROM器件写超时 */
    }   
    
    while(NumByteToWrite--){
        /* 第4步：开始写入数据 */
        i2c_SendByte(*pBuffer);

        /* 第5步：检查ACK */
        if (i2c_WaitAck() != 0){
            goto cmd_fail;    /* 器件无应答 */
        }

        pBuffer++;    /* 地址增1 */
    }
    
    /* 命令执行成功，发送I2C总线停止信号 */
    i2c_Stop();
    return 0;

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
    /* 发送I2C总线停止信号 */
    i2c_Stop();
    return 1;
}

#endif

/* 
*********************************************************************************************************
*    函 数 名: GT911_ReadReg 
*    功能说明: 读1个或连续的多个寄存器 
*    形 参: _usRegAddr : 寄存器地址 
*    _pRegBuf : 寄存器数据缓冲区 
*    _ucLen : 数据长度 
*    返 回 值: 无 
*********************************************************************************************************
*/ 
void GT911_ReadReg(uint16_t _usRegAddr, uint8_t *_pRegBuf, uint8_t _ucLen) 
{ 
    uint8_t i; 

    i2c_Start();    /* 总线开始信号 */  

    i2c_SendByte(GTP_ADDRESS | I2C_DIR_WR);    /* 发送设备地址+写信号 */
    i2c_WaitAck(); 

    i2c_SendByte(_usRegAddr >> 8);     /* 地址高8位 */ 
    i2c_WaitAck();                     

    i2c_SendByte(_usRegAddr);          /* 地址低8位 */  
    i2c_WaitAck(); 
    i2c_Stop();    
    i2c_Start(); 
    i2c_SendByte(GTP_ADDRESS | I2C_DIR_RD);  /* 发送设备地址+读信号 */ 
    i2c_WaitAck(); 

    for (i = 0; i < _ucLen - 1; i++) { 
        _pRegBuf[i] = i2c_ReadByte();  /* 读寄存器数据 */ 
        i2c_Ack(); 
    } 

    /* 最后一个数据 */
    _pRegBuf[i] = i2c_ReadByte();      /* 读寄存器数据 */
    i2c_NAck(); 

    i2c_Stop();    /* 总线停止信号 */  
} 

/* 
*********************************************************************************************************
*    函 数 名: GT911_WriteReg 
*    功能说明: 写1个或连续的多个寄存器 
*    形 参: _usRegAddr : 寄存器地址 
*    _pRegBuf : 寄存器数据缓冲区 
*    _ucLen : 数据长度 
*    返 回 值: 无 
*********************************************************************************************************
*/ 
void GT911_WriteReg(uint16_t _usRegAddr, uint8_t *_pRegBuf, uint8_t _ucLen) 
{ 
    uint8_t i; 

    i2c_Start();    /* 总线开始信号 */ 

    i2c_SendByte(GTP_ADDRESS | I2C_DIR_WR);    /* 发送设备地址+写信号 */ 
    i2c_WaitAck(); 

    i2c_SendByte(_usRegAddr >> 8);    /* 地址高8位 */ 
    i2c_WaitAck(); 

    i2c_SendByte(_usRegAddr);         /* 地址低8位 */ 
    i2c_WaitAck(); 

    for (i = 0; i < _ucLen; i++) 
    { 
        i2c_SendByte(_pRegBuf[i]);    /* 寄存器数据 */ 
        i2c_WaitAck(); 
    } 

    i2c_Stop(); /* 总线停止信号 */ 
} 


uint8_t Touch_PenIRQ(void)
{
    return gpio_input_bit_get(GTP_INT_GPIO_PORT, GTP_INT_GPIO_PIN);
}


///**
// * Read an input device
// * @param indev_id id of the input device to read
// * @param x put the x coordinate here
// * @param y put the y coordinate here
// * @return true: the device is pressed, false: released
// */
//static bool touchpad_read(lv_indev_data_t *data)
//{
////    static int16_t last_x = 0;
////    static int16_t last_y = 0;
////    __IO uint32_t xpos = 0, ypos = 0, X, Y;

////    if(Touch_PenIRQ() == 0) {
////        xpos = Touch_MeasurementX();
////        ypos = Touch_MeasurementY();
////        X = _AD2X(xpos);
////        Y = LCD_Y-_AD2Y(ypos);
////        data->point.x = X;
////        data->point.y = Y;
////        last_x = data->point.x;
////        last_y = data->point.y;
////        data->state = LV_INDEV_STATE_PR;
////    } else {
////        data->point.x = last_x;
////        data->point.y = last_y;
////        data->state = LV_INDEV_STATE_REL;
////    }

////    GT911_OnePiontScan();
////    if(0 != GT911_PiontScan()){
////        data->point.x = g_inputx;
////        data->point.y = g_inputy;
////        data->state = LV_INDEV_STATE_PR;
////    }else{
////        data->point.x = g_inputx;
////        data->point.y = g_inputy;
////        data->state = LV_INDEV_STATE_REL;
////    }
//	static int16_t last_x = 0;
//	static int16_t last_y = 0;
//	if(is_touch)//有按键按下时
//	{
//		data->point.x = touch_x;
//		data->point.y = touch_y;
//		data->state = LV_INDEV_STATE_PR;
//		last_x = data->point.x;
//		last_y = data->point.y;
//		is_touch = 0;
//	}
//	else
//	{		
//		data->point.x = last_x;
//		data->point.y = last_y;
//		data->state = LV_INDEV_STATE_REL;
//	}
//    return false;
//}

///**
// * Initialize your input devices here
// */
//void touchpad_init(void)
//{
//    GTP_Init_Panel();

//    lv_indev_drv_t indev_drv;
//    lv_indev_drv_init(&indev_drv);
//    indev_drv.read = touchpad_read;
//    indev_drv.type = LV_INDEV_TYPE_POINTER;
//    lv_indev_drv_register(&indev_drv);
//}

/*********************************************END OF FILE**********************/
