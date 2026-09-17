# 底盘控制主链路

前台数据流为：RC捕获或Jetson帧 → 既有应用状态机 → 运动监督器 → 差速运动学 → MSSD驱动 → RS485事务 → USART2 DMA适配。ISR只发布捕获样本、DMA位置或完成/错误标志；状态转换、协议验证和事务调度均在 `chassis_app_poll()` 中执行。

`chassis_app_default_config()` 是当前唯一的板级应用配置入口。已记录机械基线为轮半径101 mm、半轮距250 mm和减速比5.2，但以下门控默认关闭：

- `direction_mapping_verified=false`；
- `maximum_rpm_verified=false`，最大RPM为0；
- `motion_enable=false`；
- `rc_mapping_verified=false`；
- 左右抱闸 `mapping_valid=false`。
- Jetson自检必须收到部署协议`0x51`成功响应。

因此默认版本即使收到合法非零RC或Jetson命令，也只会保持运动未授权并调度停止事务，不会构造非零RPM目标。组合层没有任何抱闸调用，也不写MSSD持久参数。

启动顺序为软件对象初始化、USART2接收/DE低、USART1接收、RC输入捕获，然后依次读取 `0x007B～0x007E`、右状态和左状态。只有配置为独立闭环且通信控制、两侧反馈有效且故障码为0后，软件自检才可能通过。配置不符只锁止并报告，不自动改写。

安全停止优先于普通目标和轮询。正在进行的普通事务可被非阻塞取消，随后顺序执行右、左停止；部分成功被锁存为 `PARTIAL_COMMIT`。写事务不自动重试。协议写成功只代表驱动器响应匹配，不代表车轮达到目标或机械抱闸到位。

PB8蜂鸣器只实现证据明确的急停提示：进入EMERGENCY立即置为有效电平，之后每
200 ms非阻塞翻转，离开EMERGENCY或停止应用时立即关闭。LED调试闪烁不属于安全链路，未复制旧
工程的阻塞式行为。
