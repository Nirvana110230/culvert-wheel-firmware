# 轮式底盘控制主链路

板级入口 `main()` 选用 `chassis_app_wheel_rc_config()`。两轴量程为 ±500 mm/s、±1500 mrad/s，方向系数均为 +1。轮半径 101 mm、半轮距 250 mm、减速比 5.2。轮式换算保留原车 635 mm 周长近似及逐步整数截断，两个输入的组合峰值为 426 轴 RPM。

| 环节 | 运行行为 |
|---|---|
| 输入 | TIM8 和 TIM3 捕获五路 PWM，ISR 发布带接收时刻的样本。`rc_input_process_sample` 处理轴方向、死区、开关消抖和急停。独立档监测 CH5，CH1/2/6/7 参与驾驶判断。 |
| 冷启动 | MSSD 1 号站，USART2 9600 8N1。写四项闭环配置，再独立读回、读取两侧状态。SWB 在待机、SWC 解除、摇杆回中，切遥控后才接受驾驶输入。 |
| 运动 | `chassis_app_poll` 推进状态机、`motion_supervisor_poll`、`chassis_kinematics_calculate` 以及 MSSD 调度。两侧分别使用 FC10 写有符号轴速，高字在前。处于遥控时两轴回中给双侧零目标，保留松闸。 |
| 停车 | 待机、急停、失联和电机故障撤销运动。停车和机械抱闸使用各自的 FC06 寄存器。写回显仅能确认驱动器收到命令，物理抱闸位置没有反馈。 |
| 总线 | DMA TX 与 UART TC 分开处理。最后一位发完后放低 PD7；接收侧按 RTU 帧、CRC 和当前请求校验应答。 |
| 上位机 | 板用配置选 `CHASSIS_STANDALONE_RC`。只有保留的 `CHASSIS_HOST_INSPECTION` 档启用上位机接收及握手、巡检 ACK 与速度帧。 |
| 监测 | 软件保留当前模式、输入和授权、目标与反馈、故障、超时、CRC、PWM 丢样等诊断。PB8 在急停时驱动蜂鸣器。 |

操作与故障恢复步骤见[正式固件烧录与使用说明](../../../wheel-firmware/正式固件烧录与使用说明.md)。Keil 实选源码和各项变化见[详细对照报告](../../../wheel-firmware/原Keil工程与新正式轮式固件详细对照报告.md)。
