#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "PCF8563.h"
#include "iic.h"

 int64_t timestamp=0;

void timer_periodic_cb(void *arg);//定时器函数声明
esp_timer_handle_t timer_periodic_handle = 0;//定义重复定时器句柄


//定义一个重复运行的定时器结构体
esp_timer_create_args_t timer_periodic_arg = { .callback =
        &timer_periodic_cb, //设置回调函数
        .arg = NULL, //不携带参数
        .name = "PeriodicTimer" //定时器名字
        };


void timer_periodic_cb(void *arg) //1ms中断一次
{
    static int64_t timer_count=0;


    timer_count++;
    if(timer_count>=1000)//1s
    {
        timer_count=0;


    }
    
}

static void Time_Read_Task(void* arg)
{
    while(1)
    {
        printf("read timestamp\n");  
        ESP_LOGW("gjh:", "版本控制！！！不听话就挨揍！！！"); 
        timestamp=Read_UnixTime();

        printf("timestamp=%lld\n",timestamp);
        Read_UTCtime();
        
        ESP_LOGE("gjh:", "版本控制！！！不听话就挨揍！！！");

        vTaskDelay(1000 / portTICK_RATE_MS);
    }  
}


void app_main()
{




    /*******************************timer 1s init**********************************************/  
    esp_err_t err = esp_timer_create(&timer_periodic_arg, &timer_periodic_handle);
    err = esp_timer_start_periodic(timer_periodic_handle, 1000);//创建定时器，单位us 定时1ms
    if(err != ESP_OK)
    {
        printf("timer periodic create err code:%d\n", err);
    }
    


    vTaskDelay(20 / portTICK_RATE_MS);

    iic_init();
    vTaskDelay(20 / portTICK_RATE_MS);
    Timer_IC_Init();

    printf("read timestamp\n");

    timestamp=Read_UnixTime();

    printf("timestamp=%lld\n",timestamp);


    xTaskCreate(&Time_Read_Task, "Time_Read_Task", 2048, NULL, 10, NULL);



}


