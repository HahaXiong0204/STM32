
使用cubemx重新创建项目 必须在middlewares/Rt-thread/RTOS/kernal的board.c中将static int uart_init(void)的串口改成usart1

该项目，可以通过不同设备发送不同主题来区分设备，

已经解决不用接收多个数据

开启了 风扇的pwm 为499 不转

蜂鸣器 0 响 1 不响

一台设备同时将只有一个人可以使用，通过 user_uing 判断是否有人使用




TIM1 定时1s         呼吸灯 LED0
TIM2 定时0.1s
TIM3 PC8 -> 电机 通道3 		PB1 通道4-> led_pwm 
TIM4 点击测速 PE0
TIM14 RTT系统时间不可用


LED0 PB1
LED1 PB0      低电平点亮

PA4 -> 光敏
PA5 ->  电压
PA6 -> 浊度
PA7 -> 空气

PG14 -> DHT11

PI4 -> RC522_SDA
PI5 -> RC522_RST

PE4 PE5 PE6 接薄膜按键  1接 GND

更新了字库

