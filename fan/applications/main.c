/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main program body.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "ch32v30x.h"
#include <rtthread.h>
#include <rthw.h>
#include "drivers/pin.h"
#include <board.h>

#include "pwm.h"
#include "sensor_asair_aht10.h"

#include <arpa/inet.h>
#include <netdev.h>
#include <ntp.h>

#define CONTROL_UART_NAME       "uart2"
static rt_device_t serial;

#define AHT10_I2C_BUS            "i2c1"
/* 信号量 */
static struct rt_semaphore mode_sem;
static struct rt_semaphore plus_sem;
static struct rt_semaphore reduce_sem;
static struct rt_semaphore shake_sem;
static struct rt_semaphore hum_sem;
static struct rt_semaphore rx_sem;

/* 邮箱控制块 */
static struct rt_mailbox mb;
/* 用于放邮件的内存池 */
static char mb_pool[128];

/* 邮箱控制块 */
static struct rt_mailbox mb2;
/* 用于放邮件的内存池 */
static char mb_pool2[128];

/* 邮箱控制块 */
static struct rt_mailbox mb3;
/* 用于放邮件的内存池 */
static char mb_pool3[128];

/* AHT10 初始化 */
int rt_hw_aht10_port(void)
{
    struct rt_sensor_config cfg;
    cfg.intf.dev_name = AHT10_I2C_BUS;
    cfg.intf.user_data = (void *)AHT10_I2C_ADDR;
    rt_hw_aht10_init("aht10", &cfg);
    return RT_EOK;
}

/* 接收数据回调函数 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_sem_release(&rx_sem);

    return RT_EOK;
}

ALIGN(RT_ALIGN_SIZE)
static char thread1_stack[512];
static struct rt_thread thread1;
/* 串口数据处理线程入口 */
static void sem_release_entry(void *parameter)
{
    char *str;

    while (1)
    {
        /* 从邮箱中收取邮件 */
        if (rt_mb_recv(&mb, (rt_uint32_t *)&str, RT_WAITING_FOREVER) == RT_EOK)
        {
            switch (str[0])
            {
                case 'm':
                    rt_kprintf("mode change\r\n");
                    rt_sem_release(&mode_sem);
                    break;
                case 'p':
                    rt_kprintf("plus\r\n");
                    rt_sem_release(&plus_sem);
                    break;
                case 'r':
                    rt_kprintf("reduce\r\n");
                    rt_sem_release(&reduce_sem);
                    break;
                case 's':
                    rt_kprintf("Shake the head\r\n");
                    rt_sem_release(&shake_sem);
                    break;
                case 'h':
                    rt_kprintf("hum\r\n");
                    rt_sem_release(&hum_sem);
                    break;
                default:
                    break;
            }
            rt_thread_mdelay(100);
        }
    }
}

ALIGN(RT_ALIGN_SIZE)
static char thread2_stack[512];
static struct rt_thread thread2;
/* 串口数据转存线程入口 */
static void uart_storage_entry(void *args)
{
    char ch;
    while (1)
    {
        /* 从串口读取一个字节的数据，没有读取到则等待接收信号量 */
        while (rt_device_read(serial, -1, &ch, 1) != 1)
        {
            /* 阻塞等待接收信号量，等到信号量后再次读取数据 */
            rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        }
        rt_mb_send(&mb,(rt_uint32_t)&ch);
    }
}

ALIGN(RT_ALIGN_SIZE)
static char thread3_stack[512];
static struct rt_thread thread3;
/* 摇头控制线程入口 */
void Shake_Thread_Entry (void *args)
{
    static rt_err_t result;
    static int shake_flag=0;
    int dir=0;
    int duty=Servo_MID;
    Servo_PWMOut_Init(143,9999,Servo_MID);
    while(1)
    {
        result = rt_sem_take(&shake_sem, 100);
        if(result != RT_EOK)
        {
            rt_sem_delete(&shake_sem);
        }
        else
        {
            rt_kprintf("get sem\r\n");
            if(shake_flag == 0)
            {
                shake_flag = 1;
            }
            else {
                shake_flag = 0;
            }
        }
        while(shake_flag)
        {
            result = rt_sem_take(&shake_sem, 100);
            if(result != RT_EOK)
            {
                rt_sem_delete(&shake_sem);
            }
            else
            {
                rt_kprintf("get sem\r\n");
                if(shake_flag == 0)
                {
                    shake_flag = 1;
                }
                else {
                    shake_flag = 0;
                }
            }
            if(dir)
            {
                duty++;
            }
            else
            {
                duty--;
            }

            if (duty >= Servo_MAX)
            {
                dir = 0;
            }
            if (duty <= Servo_MIN)
            {
                dir = 1;
            }
            Servo_PWMOut_Init(143, 9999, duty);
            rt_thread_mdelay(100);
        }
        rt_thread_mdelay(100);
    }
}

ALIGN(RT_ALIGN_SIZE)
static char thread4_stack[1024];
static struct rt_thread thread4;
/* 电机控制线程入口 */
void Speed_Thread_entry(void *args)
{
    static rt_err_t result1;
    static rt_err_t result2;
    static rt_err_t result3;
    static rt_err_t result4;
    static int speed = 0;
    static int Gear = 0;
    static int mode = 0;
    static int hum_mode=0;

    rt_uint32_t tem,hum;
    int duty=0;

    static char str[11];
    static char str3[12];
    static char str1[20] = "\x74\x34\x2E\x74\x78\x74\x3D\x22\xD7\xD4\xB6\xAF\xC4\xA3\xCA\xBD\x22\xFF\xFF\xFF";   //自动模式
    static char str2[20] = "\x74\x34\x2E\x74\x78\x74\x3D\x22\xCA\xD6\xB6\xAF\xC4\xA3\xCA\xBD\x22\xFF\xFF\xFF";   //手动模式

    Motor_PWMOut_Init(143, 57, 70);
    rt_pin_mode(11, PIN_MODE_OUTPUT);
    rt_pin_write(11, 0);
    rt_pin_mode(6, PIN_MODE_OUTPUT);
    rt_pin_write(6, 0);

    while(1)
    {
        result1 = rt_sem_take(&plus_sem, 100);
        result2 = rt_sem_take(&reduce_sem, 100);
        result4 = rt_sem_take(&hum_sem, 100);

        result3 = rt_sem_take(&mode_sem, 100);
        if(result3 != RT_EOK)
        {
            rt_sem_delete(&mode_sem);
        }
        else
        {
            if(mode)
            {
                mode = 0;
            }
            else
            {
                mode = 1;
            }
        }
        if(mode == 0)
        {
            rt_mb_detach(&mb2);
            rt_mb_detach(&mb3);
            if(result1 != RT_EOK || mode == 1)
            {
                rt_sem_delete(&plus_sem);
            }
            else
            {
                rt_kprintf("PLUS\r\n");
                speed += 14;
                Gear++;
                if(speed >= 140)
                {
                    speed = 140;
                }
                if(Gear >= 10)
                {
                    Gear = 10;
                }
            }

            if(result2 != RT_EOK || mode == 1)
            {
                rt_sem_delete(&reduce_sem);
            }
            else
            {
                rt_kprintf("REDUCE\r\n");
                speed -= 14;
                Gear--;
                if(speed <= 0 )
                {
                    speed = 0;
                }
                if(Gear <= 0)
                {
                    Gear=0;
                }
            }

            if(result4 != RT_EOK || mode == 1)
            {
                rt_sem_delete(&hum_sem);
            }
            else
            {
                if(hum_mode)
                {
                    hum_mode = 0;
                    rt_pin_write(11, 0);
                }
                else
                {
                    hum_mode = 1;
                    rt_pin_write(11, 1);
                }
            }
            Motor_PWMOut_Init(143, 58, speed);
            if(Gear <= 9)
            {
                sprintf(str,"n0.val=%d\xff\xff\xff",Gear);
                rt_device_write(serial,0,str,sizeof(str));

            }
            else
            {
                sprintf(str3,"n0.val=%d\xff\xff\xff",Gear);
                rt_device_write(serial,0,str3,sizeof(str3));
            }
            rt_thread_mdelay(10);
            rt_device_write(serial,0,str2,sizeof(str2));
            rt_thread_mdelay(100);
        }
        else
        {
            rt_sem_delete(&plus_sem);
            rt_sem_delete(&reduce_sem);
            rt_sem_delete(&hum_sem);

           if (rt_mb_recv(&mb3, &tem, RT_WAITING_FOREVER) == RT_EOK)
           {
               rt_kprintf("thread1: get a mail from mailbox, the content:%d\n", tem);
           }

           if (rt_mb_recv(&mb2, &hum, RT_WAITING_FOREVER) == RT_EOK)
           {
               rt_kprintf("thread1: get a mail from mailbox, the content:%d\n", hum);
           }

           if(hum < 50)
           {
               rt_pin_write(11, 1);
           }
           else
           {
               rt_pin_write(11, 0);
           }

           duty = tem * 3.5;
           if(duty >= 140)
           {
               duty = 140;
           }
           if(tem <= 20)
           {
               duty = 0;
           }
           Motor_PWMOut_Init(143, 58, duty);

            sprintf(str,"n0.val=%d\xff\xff\xff",0);
            rt_device_write(serial,0,str,sizeof(str));
            rt_thread_mdelay(10);
            rt_device_write(serial,0,str1,sizeof(str1));
            rt_thread_mdelay(100);
        }
    }
}

ALIGN(RT_ALIGN_SIZE)
static char thread5_stack[1024];
static struct rt_thread thread5;
static void aht10_thread_entry(void *args)
{
    float hum,tem;
    rt_uint32_t hum_int,hum_float;
    rt_uint32_t tem_int,tem_float;
    char str[13];
    aht10_device_t dev;

    rt_hw_aht10_port();
    dev = aht10_init(AHT10_I2C_BUS);
    if(dev == RT_NULL)
    {
        rt_kprintf("The sensor initializes failure\r\n");
        return;
    }
    else
    {
        rt_kprintf("The sensor initializes OK\r\n");
    }

    while(1)
    {
        hum = aht10_read_humidity(dev);
        tem = aht10_read_temperature(dev);
        hum_int = (rt_uint32_t)hum;
        hum_float = ((rt_uint32_t)(hum * 10) % 10);
        tem_int = (rt_uint32_t)tem;
        tem_float = ((rt_uint32_t)(tem * 10) % 10);
        rt_mb_send(&mb2,(rt_uint32_t)hum_int);
        rt_mb_send(&mb3,(rt_uint32_t)tem_int);
        sprintf(str,"x0.val=%d%d\xff\xff\xff",tem_int,tem_float);
        rt_device_write(serial,0,str,sizeof(str));
        sprintf(str,"x1.val=%d%d\xff\xff\xff",hum_int,hum_float);
        rt_device_write(serial,0,str,sizeof(str));

        rt_thread_mdelay(5000);
    }
}

int Thread_Init(void)
{
    static rt_err_t result;
    /* 初始化信号量 */
    rt_sem_init(&rx_sem, "RXSEM", 0, RT_IPC_FLAG_FIFO);
    rt_sem_init(&mode_sem, "MODESEM", 0, RT_IPC_FLAG_FIFO);
    rt_sem_init(&plus_sem, "PLUSSEM", 0, RT_IPC_FLAG_FIFO);
    rt_sem_init(&reduce_sem, "REDUCESEM", 0, RT_IPC_FLAG_FIFO);
    rt_sem_init(&shake_sem, "SHAKESEM", 0, RT_IPC_FLAG_FIFO);
    rt_sem_init(&hum_sem, "HUMSEM", 0, RT_IPC_FLAG_FIFO);

    /* 初始化一个 mailbox */
    result = rt_mb_init(&mb,
                        "mbt",                      /* 名称是 mbt */
                        &mb_pool[0],                /* 邮箱用到的内存池是 mb_pool */
                        sizeof(mb_pool) / 4,        /* 邮箱中的邮件数目，因为一封邮件占 4 字节 */
                        RT_IPC_FLAG_FIFO);          /* 采用 FIFO 方式进行线程等待 */
    if (result != RT_EOK)
    {
        rt_kprintf("init mailbox failed.\n");
        return -1;
    }

    /* 初始化一个 mailbox */
    result = rt_mb_init(&mb2,
                        "mbt2",                      /* 名称是 mbt */
                        &mb_pool2[0],                /* 邮箱用到的内存池是 mb_pool */
                        sizeof(mb_pool2) / 4,        /* 邮箱中的邮件数目，因为一封邮件占 4 字节 */
                        RT_IPC_FLAG_FIFO);          /* 采用 FIFO 方式进行线程等待 */
    if (result != RT_EOK)
    {
        rt_kprintf("init mailbox failed.\n");
        return -1;
    }

    /* 初始化一个 mailbox */
    result = rt_mb_init(&mb3,
                        "mbt3",                      /* 名称是 mbt */
                        &mb_pool3[0],                /* 邮箱用到的内存池是 mb_pool */
                        sizeof(mb_pool3) / 4,        /* 邮箱中的邮件数目，因为一封邮件占 4 字节 */
                        RT_IPC_FLAG_FIFO);          /* 采用 FIFO 方式进行线程等待 */
    if (result != RT_EOK)
    {
        rt_kprintf("init mailbox failed.\n");
        return -1;
    }

    serial = rt_device_find(CONTROL_UART_NAME);
    if (!serial)
    {
        rt_kprintf("find %s failed!\n", CONTROL_UART_NAME);
        return RT_ERROR;
    }
    /* 以中断接收及轮询发送模式打开串口设备 */
    rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, uart_input);

    rt_thread_init(&thread1,
                   "sem_release",
                   sem_release_entry,
                   RT_NULL,
                   &thread1_stack[0],
                   sizeof(thread1_stack),
                   9,
                   5);
    rt_thread_startup(&thread1);

    rt_thread_init(&thread2,
                   "uart_storage",
                   uart_storage_entry,
                   RT_NULL,
                   &thread2_stack[0],
                   sizeof(thread2_stack),
                   9,
                   5);
    rt_thread_startup(&thread2);

    rt_thread_init(&thread3,
                   "Shake",
                   Shake_Thread_Entry,
                   RT_NULL,
                   &thread3_stack[0],
                   sizeof(thread3_stack),
                   10,
                   5);
    rt_thread_startup(&thread3);

    rt_thread_init(&thread4,
                   "Motor",
                   Speed_Thread_entry,
                   RT_NULL,
                   &thread4_stack[0],
                   sizeof(thread4_stack),
                   11,
                   5);
    rt_thread_startup(&thread4);

    rt_thread_init(&thread5,
                   "AHT10",
                   aht10_thread_entry,
                   RT_NULL,
                   &thread5_stack[0],
                   sizeof(thread5_stack),
                   12,
                   5);
    rt_thread_startup(&thread5);

    return 0;
}
/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    //获取网卡对象
    struct netdev* net = netdev_get_by_name("esp0");
    time_t now;     /* 保存获取的当前时间值 */
    char *str;
    char str1[36];
    rt_kprintf("MCU: CH32V307\n");
    rt_kprintf("SysClk: %dHz\n",SystemCoreClock);
    Thread_Init();
    while(netdev_is_internet_up(net) != 1)
    {
       rt_thread_mdelay(200);
    }
    time_t cur_time;
    cur_time = ntp_sync_to_rtc(NULL);
    if (cur_time)
    {
        now = time(RT_NULL);
        str = ctime(&now);
        sprintf(str1,"t7.txt=\"%s\"\xff\xff\xff",str);
        rt_kprintf("%s\r\n",str1);
        rt_device_write(serial,0,str1,sizeof(str1));
    }
    while(1)
    {
        now = time(RT_NULL);
        str = ctime(&now);
        sprintf(str1,"t7.txt=\"%s\"\xff\xff\xff",str);
        rt_device_write(serial,0,str1,sizeof(str1));
        rt_thread_mdelay(1000);
    }
}



