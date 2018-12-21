#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "iic.h"

int time_delay=0;

void timer_once_cb(void *arg);//函数声明
esp_timer_handle_t timer_once_handle = 0;//定义单次定时器句柄

//定义一个单次运行的定时器结构体
esp_timer_create_args_t timer_once_arg = { .callback = &timer_once_cb, //设置回调函数
        .arg = NULL, //不携带参数
        .name = "OnceTimer" //定时器名字
        };


void timer_once_cb(void *arg) 
{
    time_delay=1;
    //int64_t tick = esp_timer_get_time();
    //printf("方法回调名字: %s , 距离定时器开启时间间隔 = %lld \r\n", __func__, tick);
}

void delay_us(uint8_t nus)
{
  time_delay=0;
  esp_timer_start_once(timer_once_handle, nus); //启动单次定时器
  while(time_delay==0);
  return;
}

void SCL_OUT(void)
{
    gpio_config_t io_conf;

    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO16
    io_conf.pin_bit_mask = (1<<GPIO_SCL);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);    
}

void SDA_OUT(void)
{
    gpio_config_t io_conf;

    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO16
    io_conf.pin_bit_mask = (1<<GPIO_SDA);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);    
}


void SDA_IN(void)
{
    gpio_config_t io_conf;

    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO16
    io_conf.pin_bit_mask = (1<<GPIO_SDA);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);    
}


void iic_init(void)
{
  SCL_OUT();
  SDA_OUT();
  esp_err_t err  = esp_timer_create(&timer_once_arg, &timer_once_handle);
}


/*******************************************************************************
  iic start:when CLK is high,SDA change form high to low
*******************************************************************************/
void IIC_Start(void)
{
  
  SDA_OUT();  //sda out mode

  IIC_SDA_ON();
  
  IIC_SCL_ON();
  
  delay_us(5);  //delay about 5us
  
  IIC_SDA_OFF();
  
  delay_us(5);  //delay about 5us
  
  IIC_SCL_OFF();
}

/*******************************************************************************
  iic stop:when CLK is high SDA change form low to high
*******************************************************************************/
void IIC_Stop(void)
{
  SDA_OUT();  //sda out mode
  
  IIC_SCL_OFF();
  
  IIC_SDA_OFF(); 
  
  IIC_SCL_ON();
  
  delay_us(5);  //delay about 5us
  
  IIC_SDA_ON();
  
  delay_us(5);	//delay about 5us						   	
}

/*******************************************************************************
  Master send ack
*******************************************************************************/
static void IIC_Ack(void)
{
  SDA_OUT();    //sda out mode
  
  IIC_SCL_OFF();
  
  IIC_SDA_OFF();
  
  delay_us(2);  //delay about 2us
  
  IIC_SCL_ON();
  
  delay_us(5);  //delay about 5us
  
  IIC_SCL_OFF();
}

/*******************************************************************************
  Master send no ack
*******************************************************************************/
static void IIC_NAck(void)
{
  SDA_OUT();  //sda out mode
  
  IIC_SCL_OFF();
  
  IIC_SDA_ON();
  
  delay_us(2);  //delay about 2us
  
  IIC_SCL_ON();
  
  delay_us(5);  //delay about 5us
  
  IIC_SCL_OFF();
}

/*******************************************************************************
//init the iic bus
*******************************************************************************/
void I2C_Init(void)
{
  SDA_OUT();  //sda out mode
  
  IIC_SCL_ON();
  
  IIC_SDA_ON();
}

/*******************************************************************************
  iic bus wait ack,acked return 1,no acked return 0
*******************************************************************************/
short IIC_Wait_Ack(void)
{
  uint8_t retry=0;
  
  SDA_IN();  //SDA IN
  
  IIC_SDA_ON();
  
  delay_us(1);  //delay about 1us
  
  IIC_SCL_ON();
  
  delay_us(1);  //delay about 1us
  
  while(gpio_get_level(GPIO_SDA))  //waite for ack
  {
    if(retry++>200)  //200us time out
    {
      IIC_Stop();  //iic bus stop
      
      return 0;
    }
    delay_us(2);  //delay about 2us
  }
  
  IIC_SCL_OFF();
  
  return 1;  
} 

/*******************************************************************************
  iic send a byte
*******************************************************************************/
void IIC_Send_Byte(uint8_t txd)
{                        
  uint8_t t;   
  
  SDA_OUT(); 	    
  
  IIC_SCL_OFF();
  
  for(t=0;t<8;t++)
  {      
    if(txd&0x80)
    {
      IIC_SDA_ON();
    }
    else
    {
      IIC_SDA_OFF();
    }
    
    txd<<=1; 
    
    IIC_SCL_ON();
    
    delay_us(2);  //delay about 2us
    
    IIC_SCL_OFF();
    
    delay_us(2);  //delay about 2us
  }
} 

/*******************************************************************************
  IIC Read a Byte,ack:1-send ack,0-no ack
*******************************************************************************/
uint8_t IIC_Read_Byte(uint8_t ack)
{
  uint8_t i;
  uint8_t receive=0;
  
  SDA_IN();  //sda in mode
  
  for(i=0;i<8;i++)
  {
    IIC_SCL_ON();
    
    delay_us(2);  //delay about 2us
    
    receive<<=1;
    
    if(gpio_get_level(GPIO_SDA))  //read 1 bit data
    {
      receive++;
    }
    
    IIC_SCL_OFF(); 
    
    delay_us(1);  //delay about 1us
  }
  
  if (ack)
  {
    IIC_Ack();  //send ack
  }
  else
  {
    IIC_NAck();  //send no ack
  }
  
  return receive;
}

/*******************************************************************************
  write a byte to slave register
*******************************************************************************/
static short IIC_WR_Reg(uint8_t sla_addr,uint8_t reg_addr,uint8_t val)
{
  IIC_Start();  //IIC start 	
  
  IIC_Send_Byte(2*sla_addr);  //send write command
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {   
    return 0;
  }
  
  IIC_Send_Byte(reg_addr);  //send register address
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    return 0;
  }
  
  IIC_Send_Byte(val);  //send data value
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  { 
    return 0;
  }
   
  IIC_Stop();   //IIC stop	
  
  return 1;
}

/*******************************************************************************
  write a byte to slave register whit multiple try
*******************************************************************************/
void MulTry_IIC_WR_Reg(uint8_t sla_addr,uint8_t reg_addr,uint8_t val)
{
  uint8_t n_try;
  
  for(n_try=0;n_try<RETRY_TIME_OUT;n_try++)
  {
    if(IIC_WR_Reg(sla_addr,reg_addr,val)==1)
    {
      break;
    }
    vTaskDelay(6 / portTICK_RATE_MS);
    //MAP_UtilsDelay(80000);  //delay about 6ms
  }
}

/*******************************************************************************
  Read a byte from slave register
*******************************************************************************/
static short IIC_RD_Reg(uint8_t sla_addr,uint8_t reg_addr,uint8_t *val)		
{
  IIC_Start();  //IIC start  	
  
  IIC_Send_Byte(2*sla_addr);  //send write command
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    return 0;
  }
  
  IIC_Send_Byte(reg_addr);  //send register address
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    return 0;
  }
  
  IIC_Start();  //IIC start
  
  IIC_Send_Byte(2*sla_addr+1);  //send read command
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    return 0;
  }
  
  *val=IIC_Read_Byte(0);  //read a byte
  
  IIC_Stop();   //IIC stop
  
  return 0;
}

/*******************************************************************************
//Read a byte from slave register whit multiple try
*******************************************************************************/
void MulTry_IIC_RD_Reg(uint8_t sla_addr,uint8_t reg_addr,uint8_t *val)
{
  uint8_t n_try;
  
  for(n_try=0;n_try<RETRY_TIME_OUT;n_try++)
  {
    if(IIC_RD_Reg(sla_addr,reg_addr,val)==1)
    {
      break;
    }
    vTaskDelay(3 / portTICK_RATE_MS);
    //MAP_UtilsDelay(40000);  //delay about 3ms
  }
}

/*******************************************************************************
// write multi byte to slave register
*******************************************************************************/
static short I2C_WR_mulReg(uint8_t sla_addr,uint8_t reg_addr,uint8_t *buf,uint8_t len) 
{

  IIC_Start();  //IIC start 	

  IIC_Send_Byte(2*sla_addr);  //send write command	
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    return 0;
  }
  
  IIC_Send_Byte(reg_addr);  //send register address
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    return 0;
  }
  
  while(len)
  {
    IIC_Send_Byte(*buf);  //send data value
    
    buf++;
    
    len--;
    
    if(IIC_Wait_Ack()<0)  //wait device ack
    {
      IIC_Stop();   //IIC stop
    
      return 0;
    }
  }

  IIC_Stop();   //IIC stop
  
  return 1;
}

/*******************************************************************************
  write multi byte to slave register whit multiple try
*******************************************************************************/
void MulTry_I2C_WR_mulReg(uint8_t sla_addr,uint8_t reg_addr,uint8_t *buf,uint8_t len) 
{
  uint8_t n_try;
  
  for(n_try=0;n_try<RETRY_TIME_OUT;n_try++)
  {
    if(I2C_WR_mulReg(sla_addr,reg_addr,buf,len)==1)
    {
      break;
    }
    vTaskDelay(6 / portTICK_RATE_MS);
    //MAP_UtilsDelay(80000);  //delay about 6ms
  }
}

/*******************************************************************************
// read multiple byte from slave register
*******************************************************************************/
static short I2C_RD_mulReg(uint8_t sla_addr,uint8_t reg_addr,uint8_t *buf,uint8_t len) 		
{
  uint8_t i;
  
  IIC_Start();  //IIC start  	
  
  IIC_Send_Byte(2*sla_addr);  //send write command 
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    return 0;
  }
  
  IIC_Send_Byte(reg_addr);  //send register address
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    return 0;
  }	
  
  IIC_Start();  //IIC start
  
  IIC_Send_Byte(2*sla_addr+1);  //send read command
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    return 0;
  }
  
  for(i=0;i<len;i++)
  {
    if(i==len-1)
    {
      buf[i]=IIC_Read_Byte(0);  //read a byte no ack
    }
    else
    {
      buf[i]=IIC_Read_Byte(1);  //read a byte ack
    }
  }

  IIC_Stop();  //IIC stop
  
  return 1;
}

/*******************************************************************************
  read multiple byte from slave register whit multiple try
*******************************************************************************/
void MulTry_I2C_RD_mulReg(uint8_t sla_addr,uint8_t reg_addr,uint8_t *buf,uint8_t len) 
{
  uint8_t n_try;
  
  for(n_try=0;n_try<RETRY_TIME_OUT;n_try++)
  {
    if(I2C_RD_mulReg(sla_addr,reg_addr,buf,len)==1)
    {
      break;
    }
    vTaskDelay(3 / portTICK_RATE_MS);
    //MAP_UtilsDelay(40000);  //delay about 3ms
  }
}


/*******************************************************************************
                                      END         
*******************************************************************************/




