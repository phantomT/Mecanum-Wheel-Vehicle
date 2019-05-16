# 麦克纳姆轮机器人-Mecanum Wheel Robot

## 电路说明-Circuit Description

### 电路连线-Circuit Connection：

![](https://github.com/phantomT/Mecanum-Wheel-Vehicle/blob/master/%E7%94%B5%E8%B7%AF%E5%9B%BE%E6%88%AA%E5%B1%8F.PNG)

### 电路元件-Circuit Components：

1. STM32F429IGT6最小系统板——控制板

   > STM32F429IGT6 Minimum System Board - Control Board(MCU)

2. TJA1050 CAN总线通信模块

   > TJA1050 CAN bus communication Module

3. JDY-08蓝牙模块

   > JDY-08 Bluetooth Module

4. 超声波传感器

   > Ultrasonic Sensor

5. RMDS-107直流电机驱动器

   > RMDS-107 DC Motor Driver

控制板使用UART1和2接口与蓝牙模块通信，发送指令接受信号强度回传。  

The MCU communicates with the Bluetooth Module using the UART1 and 2 interfaces, and sends commands to receive signal strength backhaul.    

UART7接口用于测试调试，PI9和PA12是CAN协议通信引脚，通过TJA1050模块转换接入CAN总线，对电机驱动器进行控制。    

The UART7 interface is used for test debugging. PI9 and PA12 are CAN Bus Communication pins. The TJA1050 Module is connected to the CAN Bus to control the Motor Driver.    

CAN总线两端各接入一个120Ω的电阻，每个驱动器控制一个直流编码电机，Encoder接口接入编码器的AB相信号线和5V电源线和地线，Motor线接入电机的正负极。  

Each **120Ω Resistor** is connected to each end of the CAN Bus. Each Driver controls a DC-coded motor. The `Encoder` interface is connected to the AB-phase signal line of the Encoder and the `5V` and `GND`. The `motor` line is connected to the `+` and `-` of the Motor.   

使用控制板的计时器TIM3的四个通道对应的引脚，接受超声波传感器的回传信号，使用控制板的PD10-PD13四个GPIO引脚向超声波模块传输触发信号。  

Use the corresponding pin of the 4 channels of TIM3 on the MCU to receive the return signal of the Ultrasonic Sensor, and use the four GPIO pins from PD10 to PD13 on the MCU to trigger 4 Ultrasonic Modules.   

**控制板由锂电池接入电源模块降压提供的3.3V电源供电，驱动器电源由锂电池直接接入，使用12V电源，驱动器直接为编码器和电机供电。**传感器使用的5V电源由控制板内置的变压模块提供。  

The MCU is powered by a 3.3V power supply which voltage was reduced by a  Power Module from a Lithium Battery. The Driver's power supply is directly connected to the Lithium Battery. The 12V power supply is used, and the Driver directly supplies power to the Encoder and the Motor. The 5V power supply used by Sensors is provided by the transformer built-in the MCU.    

**程序中只包含CAN总线通信与电机驱动部分代码。**  

**The program only contains the CAN bus communication and motor driver  code. **     

## 机械设计-Mechanical Design

![](https://github.com/phantomT/Mecanum-Wheel-Vehicle/blob/master/%E6%9C%BA%E6%A2%B0%E5%9B%BE%E6%88%AA%E5%B1%8F.png)

机械图纸包含在`Mechanics`文件夹里  

Mechanical drawings are included in the `Mechanics` folder.

## 驱动程序

**程序中只包含CAN总线通信与电机驱动部分代码。**  

**The program only contains the CAN bus communication and motor driver  code. **   

CAN总线程序与电机驱动程序移植自`Robomodule`厂家提供的库函数模板。  

The CAN bus program and the motor driver are ported from the library function template provided by the manufacturer of `Robomodule`.  

程序使用**STM32CUBEMX**配置，使用**HAL**库进行编程。  

The program uses the **STM32CUBEMX** to configure the chip and is programmed using the **HAL** library.  

程序源码包含在`SourceCode`文件夹里。  

The program source code is included in the `SourceCode` folder.  

