# DJI M2006 Motor angle control

#### 介绍
此代码用于完成大疆M2006电机搭配大疆C610电调进行的角度位置控制。

#### 软件架构
1. 单片机的配置以及相关代码的编写使用STM32CUBEIDE完成。
2. 电机数据的实时监测使用CUBE Monitor完成。

#### 安装教程

CUBEIDE以及CUBE Monitor的安装请参考ST官网。

#### 使用说明

1.  使用CUBEIDE软件完成单片机配置，具体配置内容请参考.ioc文件
2.  烧录完成后即可运行电机控制程序，目标角度可在BSP_pid.c文件中对target值进行修改，PID相关参数可在pid.c文件中进行修改。
3.  对于电机转速、角度、PID相关计算值的实时监测可以在CUBE Monitor中Variables栏进行选择并实时监测。



