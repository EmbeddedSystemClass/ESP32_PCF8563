/*******************************************************************************
  * @file       IIC BUS DRIVER APPLICATION       
  * @author 
  * @version
  * @date 
  * @brief
  ******************************************************************************
  * @attention
  *
  *
*******************************************************************************/

/*-------------------------------- Includes ----------------------------------*/
#include "stdint.h"


#define GPIO_SDA    27
#define GPIO_SCL    14

#define RETRY_TIME_OUT  3




#define IIC_SCL_ON()    gpio_set_level(GPIO_SCL, 1) //SCL HIGH
#define IIC_SCL_OFF()   gpio_set_level(GPIO_SCL, 0)        //SCL LOW

#define IIC_SDA_ON()    gpio_set_level(GPIO_SDA, 1)  //SDA HIGH
#define IIC_SDA_OFF()   gpio_set_level(GPIO_SDA, 0)       //SDA LOW

//#define SDA_OUT()       MAP_GPIODirModeSet(iic_sda_gpio, iic_sda_pin, GPIO_DIR_MODE_OUT);  //set sda pin out mode
//#define SDA_IN()        MAP_GPIODirModeSet(iic_sda_gpio, iic_sda_pin, GPIO_DIR_MODE_IN);  //set sda pin in mode


/*******************************************************************************
 FUNCTION PROTOTYPES
*******************************************************************************/

extern void iic_init(void);
extern void delay_us(uint8_t nus);  //delay about nus*3us

extern void IIC_Start(void);  //iic start

extern void IIC_Stop(void);  //iic stop

extern void I2C_Init(void);  //init the iic bus

extern short IIC_Wait_Ack(void);  //iic bus waite ack,acked return 0,no acked return 1

extern void IIC_Send_Byte(uint8_t txd);  //iic send a byte

extern uint8_t IIC_Read_Byte(uint8_t ack);  //iic read a byte,1-send ack,0-no ack

extern void MulTry_IIC_WR_Reg(uint8_t sla_addr,uint8_t reg_addr,uint8_t val);  //write a byte to slave register whit multiple try

extern void MulTry_IIC_RD_Reg(uint8_t sla_addr,uint8_t reg_addr,uint8_t *val);  //Read a byte from slave register whit multiple try

extern void MulTry_I2C_WR_mulReg(uint8_t sla_addr,uint8_t reg_addr,uint8_t *buf,uint8_t len); //write multi byte to slave register whit multiple try

extern void MulTry_I2C_RD_mulReg(uint8_t sla_addr,uint8_t reg_addr,uint8_t *buf,uint8_t len);  //read multiple byte from slave register whit multiple time


/*******************************************************************************
                                      END         
*******************************************************************************/




