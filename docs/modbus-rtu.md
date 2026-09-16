# Modbus RTU 与 RS485 基础层

## Modbus RTU 纯协议层

本层不包含 MSSD 型号、寄存器地址或命令调度，只提供：

- CRC16-Modbus；CRC 在线路上的低字节先发送。
- `0x03` 读保持寄存器请求，数量限制为 1～125。
- `0x06` 写单寄存器请求。
- `0x10` 写多个寄存器请求，数量限制为 1～123。
- 正常响应以及精确 5 字节异常响应的关联校验。

所有构帧函数都接收目标容量，并在容量不足时返回失败且把 `written` 清零。
响应验证会检查请求本身、从站地址、功能码、精确长度、CRC、`0x03` 字节数、
`0x06` 地址/值回显以及 `0x10` 地址/数量回显。截断帧、附带尾随字节的粘包、
错误 CRC 和错误从站都会被拒绝。

广播地址 0 只允许构造写请求；由于广播没有响应，响应校验只接受 1～247 的
单播请求。当前没有任何代码调用这些构帧函数发送 MSSD 命令。

## RS485 非阻塞事务状态机

```text
IDLE -> SWITCH_TO_TX -> DMA_TX -> WAIT_UART_TC
     -> SWITCH_TO_RX -> WAIT_RESPONSE -> COMPLETE
```

失败路径进入 `TIMEOUT` 或 `ERROR`。`COMPLETE`、`TIMEOUT`、`ERROR` 都必须由
`rs485_transport_acknowledge()` 明确确认后才能回到 `IDLE`，所以新事务不能覆盖
尚未处理的终态。

- `begin()` 只在 `IDLE` 接受一个固定容量请求。
- 每次状态迁移都更新 `state_enter_ms`。
- DMA、UART TC、响应等待分别使用独立超时，达到期限即超时。
- 超时用 `(uint32_t)(now - start) >= limit`，支持 tick 回绕。
- UART TC 成功后先进入 `SWITCH_TO_RX`。
- 只有 `set_de(false)` 成功才进入 `WAIT_RESPONSE`。
- 任一失败出口都会尝试恢复接收方向；恢复失败单独计数，且不会宣称 DE 已拉低。
- DMA 完成、UART 错误和响应完成回调均校验当前状态。
- `abort()` 明确终止活动事务并进入 `ERROR`。

端口抽象约定 `set_de(false)` 为 PD7 低电平接收，`set_de(true)` 为 PD7 高电平发送。
本轮尚未绑定 USART2 HAL 回调、没有在 `main()` 初始化事务，也没有具体 MSSD
寄存器，因此状态机不会自行产生总线传输。
