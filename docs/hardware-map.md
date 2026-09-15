# 硬件映射

## MCU 与通信接口

| 功能 | MCU 引脚 | 外设/信号 | 连接对象 | 备注 |
|---|---|---|---|---|
| Jetson 发送 | PA9 | USART1_TX / JETSON_TX | Jetson | 新固件侧 USART1 |
| Jetson 接收 | PA10 | USART1_RX / JETSON_RX | Jetson | 新固件侧 USART1 |
| MSSD 发送 | PA2 | USART2_TX / MSSD_TX | TP8485/MSSD | 当前不发送MSSD业务命令 |
| MSSD 接收 | PA3 | USART2_RX / MSSD_RX | TP8485/MSSD | 当前不解析MSSD业务响应 |
| RS485方向 | PD7 | RS485_DE | TP8485 DE和/RE | 低电平为接收；复位窗口风险见待验证清单 |

## 遥控输入

| 遥控通道 | MCU 引脚 | 定时器通道 |
|---|---|---|
| CH1 | PC6 | TIM8_CH1 |
| CH2 | PC7 | TIM8_CH2 |
| CH5 | PC8 | TIM8_CH3 |
| CH6 | PC9 | TIM8_CH4 |
| CH7急停 | PA6 | TIM3_CH1 |

TIM3和TIM8均配置为1 MHz输入捕获计数。当前仅编译RC捕获接口，`main()`没有调用`rc_capture_start()`，因此空工程启动后不会主动开启这些捕获通道。阈值、消抖和失联策略见 [RC输入模块离线设计](rc-input.md)。

## 指示器件

| 功能 | MCU 引脚 | 有效电平 |
|---|---|---|
| BEEP | PB8 | 高电平响 |
| LED0 | PB5 | 低电平亮 |
| LED1 | PE5 | 低电平亮 |

## 板级复用限制

- PC6、PC7同时连接开发板摄像头接口。使用RC CH1/CH2期间禁止同时使用摄像头接口。
- PC8、PC9同时连接TF卡接口，板上存在47 kΩ上拉。当前测试禁止启用SDIO，也禁止插入TF卡参与测试。
- 新固件不包含BMS模块。
