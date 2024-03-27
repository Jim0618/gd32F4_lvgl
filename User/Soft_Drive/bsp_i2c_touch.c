/**
  ******************************************************************************
  * @file    bsp_i2c_touch.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   ���ݴ�������ר��iic����
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:����  STM32 F429 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "bsp_i2c_touch.h"
#include "gt9xx.h"
//#include "lvgl/lv_hal/lv_hal.h"
#include "systick.h"


extern uint32_t g_inputx;
extern uint32_t g_inputy;

/* STM32 I2C ����ģʽ */
#define I2C_Speed              400000

/* �����ַֻҪ��STM32��ҵ�I2C������ַ��һ������ */
#define I2C_OWN_ADDRESS7      0x0A

static void Delay(__IO uint32_t nCount)     //�򵥵���ʱ����
{
    for(; nCount != 0; nCount--);
}


/**
  * @brief  ʹ�ܴ������ж�
  * @param  ��
  * @retval ��
  */
void I2C_GTP_IRQEnable(void)
{
    /*���� INT Ϊ�������� */   
    gpio_mode_set(GTP_INT_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GTP_INT_GPIO_PIN);
    
    /* enable and set EXTI interrupt priority */
    nvic_irq_enable(GTP_INT_EXTI_IRQ, 2U, 0U);
    
    /* ���� EXTI �ж�Դ ��INT ���� */
    rcu_periph_clock_enable(RCU_SYSCFG);
    syscfg_exti_line_config(GTP_INT_EXTI_PORTSOURCE, GTP_INT_EXTI_PINSOURCE);
    
    /* configure key EXTI line */
    exti_init(GTP_INT_EXTI_LINE, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_flag_clear(GTP_INT_EXTI_LINE);
    
    delay_1ms(50);
}

/**
  * @brief  �رմ������ж�
  * @param  ��
  * @retval ��
  */
void I2C_GTP_IRQDisable(void)
{
    /*���� INT Ϊ�������� */   
    gpio_mode_set(GTP_INT_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GTP_INT_GPIO_PIN);
    
    rcu_periph_clock_enable(RCU_SYSCFG);
    /* ���� EXTI �ж�Դ ��INT ���� */
    syscfg_exti_line_config(GTP_INT_EXTI_PORTSOURCE, GTP_INT_EXTI_PINSOURCE);
    
    /* configure key EXTI line */
    exti_init(GTP_INT_EXTI_LINE, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_disable(GTP_INT_EXTI_LINE);
    /* disable touch panel EXTI interrupt */
    nvic_irq_disable(GTP_INT_EXTI_IRQ);
}

/**
  * @brief  ������ I/O����
  * @param  ��
  * @retval ��
  */
static void I2C_GPIO_Config(void)
{
#if (SOFT_IIC)   //ʹ�����IIC
    /*����SCL���� */
    rcu_periph_clock_enable(GTP_I2C_SCL_GPIO_CLK);
    gpio_mode_set(GTP_I2C_SCL_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GTP_I2C_SCL_PIN);
    gpio_output_options_set(GTP_I2C_SCL_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GTP_I2C_SCL_PIN);

    /*����SDA���� */
    rcu_periph_clock_enable(GTP_I2C_SDA_GPIO_CLK);
    gpio_mode_set(GTP_I2C_SDA_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GTP_I2C_SDA_PIN);
    gpio_output_options_set(GTP_I2C_SDA_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GTP_I2C_SDA_PIN);
#endif
 
    /*����RST���ţ������������ */
    rcu_periph_clock_enable(GTP_RST_GPIO_CLK);
    gpio_mode_set(GTP_RST_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GTP_RST_GPIO_PIN);
    gpio_output_options_set(GTP_RST_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GTP_RST_GPIO_PIN);
  
    /*���� INT���ţ�������������������ʼ�� */   
    //����Ϊ�����������ʼ��
    rcu_periph_clock_enable(GTP_INT_GPIO_CLK);
    gpio_mode_set(GTP_INT_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GTP_INT_GPIO_PIN);
    gpio_output_options_set(GTP_INT_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GTP_INT_GPIO_PIN);
}


/**
  * @brief  ��GT91xxоƬ���и�λ
  * @param  ��
  * @retval ��
  */
void I2C_ResetChip(void)
{
    /*��ʼ��GT9157,rstΪ�ߵ�ƽ��intΪ�͵�ƽ����gt9157���豸��ַ������Ϊ0xBA*/
    GTP_RST_0();
    GTP_INT_0();
    delay_1ms(1);

    /*init low >5ms*/
    GTP_RST_1();
    delay_1ms(5);
}

/**
  * @brief  I2C ����(GT91xx)��ʼ��
  * @param  ��
  * @retval ��
  */
void I2C_Touch_Init(void)
{
    I2C_GPIO_Config(); 
 
    I2C_ResetChip();

    I2C_GTP_IRQEnable();
}


#if (SOFT_IIC)   //ʹ�����IIC     

/*
*********************************************************************************************************
*    �� �� ��: i2c_Delay
*    ����˵��: I2C����λ�ӳ٣����400KHz
*    ��    �Σ���
*    �� �� ֵ: ��
*********************************************************************************************************
*/
static void i2c_Delay(void)
{
    uint8_t i;

    /*��
      �����ʱ����ͨ���߼������ǲ��Եõ��ġ�
      ����������CPU��Ƶ180MHz ��MDK���뻷����1���Ż�
      
      ѭ������Ϊ50ʱ��SCLƵ�� = 333KHz 
      ѭ������Ϊ30ʱ��SCLƵ�� = 533KHz��  
      ѭ������Ϊ20ʱ��SCLƵ�� = 727KHz�� 
    */
    for (i = 0; i < 10*5; i++);
}

void i2c_Start(void)
{
    /* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C���������ź� */
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
    /* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C����ֹͣ�ź� */
    I2C_SDA_0();
    I2C_SCL_1();
    i2c_Delay();
    I2C_SDA_1();
}

void i2c_SendByte(uint8_t _ucByte)
{
    uint8_t i;

    /* �ȷ����ֽڵĸ�λbit7 */
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
            I2C_SDA_1(); // �ͷ�����
        }
        _ucByte <<= 1;    /* ����һ��bit */
        i2c_Delay();
    }
}

/*
*********************************************************************************************************
*    �� �� ��: i2c_ReadByte
*    ����˵��: CPU��I2C�����豸��ȡ8bit����
*    ��    �Σ���
*    �� �� ֵ: ����������
*********************************************************************************************************
*/
uint8_t i2c_ReadByte(void)
{
    uint8_t i;
    uint8_t value;

    /* ������1��bitΪ���ݵ�bit7 */
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
*    �� �� ��: i2c_WaitAck
*    ����˵��: CPU����һ��ʱ�ӣ�����ȡ������ACKӦ���ź�
*    ��    �Σ���
*    �� �� ֵ: ����0��ʾ��ȷӦ��1��ʾ��������Ӧ
*********************************************************************************************************
*/
uint8_t i2c_WaitAck(void)
{
    uint8_t re;

    I2C_SDA_1();    /* CPU�ͷ�SDA���� */
    i2c_Delay();
    I2C_SCL_1();    /* CPU����SCL = 1, ��ʱ�����᷵��ACKӦ�� */
    i2c_Delay();
    if (I2C_SDA_READ())    /* CPU��ȡSDA����״̬ */{
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
*    �� �� ��: i2c_Ack
*    ����˵��: CPU����һ��ACK�ź�
*    ��    �Σ���
*    �� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_Ack(void)
{
    I2C_SDA_0();    /* CPU����SDA = 0 */
    i2c_Delay();
    I2C_SCL_1();    /* CPU����1��ʱ�� */
    i2c_Delay();
    I2C_SCL_0();
    i2c_Delay();
    I2C_SDA_1();    /* CPU�ͷ�SDA���� */
}

/*
*********************************************************************************************************
*    �� �� ��: i2c_NAck
*    ����˵��: CPU����1��NACK�ź�
*    ��    �Σ���
*    �� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_NAck(void)
{
    I2C_SDA_1();    /* CPU����SDA = 1 */
    i2c_Delay();
    I2C_SCL_1();    /* CPU����1��ʱ�� */
    i2c_Delay();
    I2C_SCL_0();
    i2c_Delay();    
}

#define I2C_DIR_WR    0        /* д����bit */
#define I2C_DIR_RD    1        /* ������bit */

/**
  * @brief   ʹ��IIC��ȡ����
  * @param   
  *     @arg ClientAddr:���豸��ַ
  *        @arg pBuffer:����ɴӻ���ȡ�����ݵĻ�����ָ��
  *        @arg NumByteToRead:��ȡ�����ݳ���
  * @retval  ��
  */
uint32_t I2C_ReadBytes(uint8_t ClientAddr,uint8_t* pBuffer, uint16_t NumByteToRead)
{
    /* ��1��������I2C���������ź� */
    i2c_Start();
    
    /* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
    i2c_SendByte(ClientAddr | I2C_DIR_RD);    /* �˴��Ƕ�ָ�� */
    
    /* ��3�����ȴ�ACK */
    if (i2c_WaitAck() != 0)
    {
        goto cmd_fail;    /* ������Ӧ�� */
    }

    while(NumByteToRead) 
    {
        if(NumByteToRead == 1){
            i2c_NAck();    /* ���1���ֽڶ����CPU����NACK�ź�(����SDA = 1) */
      
            /* ����I2C����ֹͣ�ź� */
            i2c_Stop();
        }
    
        *pBuffer = i2c_ReadByte();
    
        /* ��ָ������ */
        pBuffer++; 
      
        /*�������Լ� */
        NumByteToRead--;
    
        i2c_Ack();    /* �м��ֽڶ����CPU����ACK�ź�(����SDA = 0) */  
    }

    /* ����I2C����ֹͣ�ź� */
    i2c_Stop();
    return 0;    /* ִ�гɹ� */

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
    /* ����I2C����ֹͣ�ź� */
    i2c_Stop();
    return 1;
}

/**
  * @brief   ʹ��IICд������
  * @param   
  *     @arg ClientAddr:���豸��ַ
  *        @arg pBuffer:������ָ��
  *     @arg NumByteToWrite:д���ֽ���
  * @retval  ��
  */
uint32_t I2C_WriteBytes(uint8_t ClientAddr,uint8_t* pBuffer, uint8_t NumByteToWrite)
{
    uint16_t m;    

    /*����0������ֹͣ�źţ������ڲ�д������*/
    i2c_Stop();
  
    /* ͨ���������Ӧ��ķ�ʽ���ж��ڲ�д�����Ƿ����, һ��С�� 10ms
        CLKƵ��Ϊ200KHzʱ����ѯ����Ϊ30������
    */
    for (m = 0; m < 1000; m++){
        /* ��1��������I2C���������ź� */
        i2c_Start();
        
        /* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
        i2c_SendByte(ClientAddr | I2C_DIR_WR);    /* �˴���дָ�� */
        
        /* ��3��������һ��ʱ�ӣ��ж������Ƿ���ȷӦ�� */
        if (i2c_WaitAck() == 0){
            break;
        }
    }
    if (m  == 1000){
        goto cmd_fail;    /* EEPROM����д��ʱ */
    }   
    
    while(NumByteToWrite--){
        /* ��4������ʼд������ */
        i2c_SendByte(*pBuffer);

        /* ��5�������ACK */
        if (i2c_WaitAck() != 0){
            goto cmd_fail;    /* ������Ӧ�� */
        }

        pBuffer++;    /* ��ַ��1 */
    }
    
    /* ����ִ�гɹ�������I2C����ֹͣ�ź� */
    i2c_Stop();
    return 0;

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
    /* ����I2C����ֹͣ�ź� */
    i2c_Stop();
    return 1;
}

#endif

/* 
*********************************************************************************************************
*    �� �� ��: GT911_ReadReg 
*    ����˵��: ��1���������Ķ���Ĵ��� 
*    �� ��: _usRegAddr : �Ĵ�����ַ 
*    _pRegBuf : �Ĵ������ݻ����� 
*    _ucLen : ���ݳ��� 
*    �� �� ֵ: �� 
*********************************************************************************************************
*/ 
void GT911_ReadReg(uint16_t _usRegAddr, uint8_t *_pRegBuf, uint8_t _ucLen) 
{ 
    uint8_t i; 

    i2c_Start();    /* ���߿�ʼ�ź� */  

    i2c_SendByte(GTP_ADDRESS | I2C_DIR_WR);    /* �����豸��ַ+д�ź� */
    i2c_WaitAck(); 

    i2c_SendByte(_usRegAddr >> 8);     /* ��ַ��8λ */ 
    i2c_WaitAck();                     

    i2c_SendByte(_usRegAddr);          /* ��ַ��8λ */  
    i2c_WaitAck(); 
    i2c_Stop();    
    i2c_Start(); 
    i2c_SendByte(GTP_ADDRESS | I2C_DIR_RD);  /* �����豸��ַ+���ź� */ 
    i2c_WaitAck(); 

    for (i = 0; i < _ucLen - 1; i++) { 
        _pRegBuf[i] = i2c_ReadByte();  /* ���Ĵ������� */ 
        i2c_Ack(); 
    } 

    /* ���һ������ */
    _pRegBuf[i] = i2c_ReadByte();      /* ���Ĵ������� */
    i2c_NAck(); 

    i2c_Stop();    /* ����ֹͣ�ź� */  
} 

/* 
*********************************************************************************************************
*    �� �� ��: GT911_WriteReg 
*    ����˵��: д1���������Ķ���Ĵ��� 
*    �� ��: _usRegAddr : �Ĵ�����ַ 
*    _pRegBuf : �Ĵ������ݻ����� 
*    _ucLen : ���ݳ��� 
*    �� �� ֵ: �� 
*********************************************************************************************************
*/ 
void GT911_WriteReg(uint16_t _usRegAddr, uint8_t *_pRegBuf, uint8_t _ucLen) 
{ 
    uint8_t i; 

    i2c_Start();    /* ���߿�ʼ�ź� */ 

    i2c_SendByte(GTP_ADDRESS | I2C_DIR_WR);    /* �����豸��ַ+д�ź� */ 
    i2c_WaitAck(); 

    i2c_SendByte(_usRegAddr >> 8);    /* ��ַ��8λ */ 
    i2c_WaitAck(); 

    i2c_SendByte(_usRegAddr);         /* ��ַ��8λ */ 
    i2c_WaitAck(); 

    for (i = 0; i < _ucLen; i++) 
    { 
        i2c_SendByte(_pRegBuf[i]);    /* �Ĵ������� */ 
        i2c_WaitAck(); 
    } 

    i2c_Stop(); /* ����ֹͣ�ź� */ 
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
//	if(is_touch)//�а�������ʱ
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
