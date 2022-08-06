# AHT10介绍和使用

## 1、AHT10 模块简介

AHT10 是一款性价比极高的温湿度传感器，采用 **I2C **的接口方式 ，i2c通讯方式：SCL --> 时钟线、SDI --> 数据线

优点：高精度，完全校准；极高的可靠性与卓越的长期稳定性；抗干扰能力强；性价比极高。

![aht101](imgs\aht101.png)

![aht102](imgs\aht102.png)



## 2、底层i2c通信协议简介

I2C（Inter Integrated Circuit）总线是 PHILIPS 公司开发的一种 **半双工、双向二线制同步串行总线** 。I2C 总线传输数据时只需两根信号线，一根是双向数据线 SDA（serial data），另一根是双向时钟线 SCL（serial clock）。

根据串行通信及同步通信的特点可知， IIC 总线的数据沿着 SDA 逐位传输。且位的输出通过主机和从机之间共享的时钟信号与位采样同步。

传输过程：

* 主机发送起始信号，将启动信号发送到从机。
* 主机向每个从机发送它想要与之通信的从机的 7 位、 8 位或 10 位地址（地址帧），以及读/写位。
* 每个从机进行寻址，将主机发送的地址与自身地址进行比较，若匹配，则发送 ACK 位应答信号,反之从机将 SDA 线路保持不变维持在高电平。
* 主机发送或接收数据帧。
* 传输完每个数据帧后，接收方向发送方返回 ACK 位应答信号，确认成功接收数据帧。
* 主机发送停止信号。

**RT-Thread IIC 设备驱动框架**

由上到下可分为：设备层、驱动框架层、驱动层。

* 驱动层：分别为硬件 IIC 驱动和软件 IIC 驱动。
* 驱动框架层：RTT 为 IIC 提供的一套抽象接口。
* 设备层：为应用程序提供的统一 API 。

核心结构体。

```c
struct rt_i2c_bit_ops
{
    void *data;            /* private data for lowlevel routines */
    void (*set_sda)(void *data, rt_int32_t state);
    void (*set_scl)(void *data, rt_int32_t state);
    rt_int32_t (*get_sda)(void *data);
    rt_int32_t (*get_scl)(void *data);

    void (*udelay)(rt_uint32_t us);

    rt_uint32_t delay_us;  /* scl and sda line delay */
    rt_uint32_t timeout;   /* in tick */
};
```

| 函数指针                                       | 功能                          |
| ---------------------------------------------- | ----------------------------- |
| void (*set_sda)(void *data, rt_int32_t state); | 设置SDA电平                   |
| void (*set_scl)(void *data, rt_int32_t state); | 设置SCL电平                   |
| rt_int32_t (*get_sda)(void *data);             | 获取SDA电平                   |
| rt_int32_t (*get_scl)(void *data);             | 获取SCL电平                   |
| void (*udelay)(rt_uint32_t us);                | 软件I2C时序所需要的的延时函数 |

## 3、sensor框架的使用

Sensor 设备的作用是：为上层提供统一的操作接口，提高上层代码的可重用性。简化底层驱动开发的难度，只要实现简单的 ops  就可以将传感器注册到系统上。

应用程序通过 RT-Thread 提供的 I/O 设备管理接口来访问传感器设备。

核心结构体：

```c
struct rt_sensor_device
{
    struct rt_device             parent;    /* The standard device */

    struct rt_sensor_info        info;      /* The sensor info data */
    struct rt_sensor_config      config;    /* The sensor config data */

    void                        *data_buf;  /* The buf of the data received */
    rt_size_t                    data_len;  /* The size of the data received */

    const struct rt_sensor_ops  *ops;       /* The sensor ops */

    struct rt_sensor_module     *module;    /* The sensor module */

    rt_err_t (*irq_handle)(rt_sensor_t sensor);             /* Called when an interrupt is generated, registered by the driver */
};
```

## 4、将AHT10对接到sensor框架

**使用步骤：**

* 配置工程
* Sensor框架初始化
* 创建线程

```c
#include "sensor_asair_aht10.h"

#define AHT10_I2C_BUS            "i2c1"
/* AHT10 初始化 */
int rt_hw_aht10_port(void)
{
    struct rt_sensor_config cfg;
    cfg.intf.dev_name = AHT10_I2C_BUS;
    cfg.intf.user_data = (void *)AHT10_I2C_ADDR;
    rt_hw_aht10_init("aht10", &cfg);
    return RT_EOK;
}


ALIGN(RT_ALIGN_SIZE)
static char thread1_stack[1024];
static struct rt_thread thread1;

static void aht10_thread_entry(void *args)
{
    float hum,tem;
    rt_uint32_t hum_int,hum_float;
    rt_uint32_t tem_int,tem_float;

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

        rt_kprintf("tem=%d.%d C\r\nhum=%d.%d%\r\n",tem_int,tem_float,hum_int,hum_float);

        rt_thread_mdelay(5000);
    }
}

int AHT10(void)
{
    rt_thread_init(&thread1,
                   "AHT10",
                   aht10_thread_entry,
                   RT_NULL,
                   &thread1_stack[0],
                   sizeof(thread1_stack),
                   12,
                   5);
    rt_thread_startup(&thread1);
}
MSH_CMD_EXPORT(AHT10,  AHT10 Sample);
```



