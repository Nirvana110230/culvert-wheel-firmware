# Jetson 通信协议基础层

## 兼容性证据范围

本定义直接依据旧 STM32 工程的 `User/protocol.c`、`User/protocol.h`、
`User/main.c`、`Drivers/SYSTEM/usart` 和 `Rosmaster_Lib.py` 整理。
当前工作区未找到 Jetson 端 `stm32_node.cpp`，因此这里只能确认与旧 STM32
工程的收发实现兼容，不能宣称已经验证真实 Jetson 部署兼容性。

## 旧帧格式

```text
FF DIRECTION LEN FUNC PAYLOAD... CHECKSUM
```

- `DIRECTION=FC`：Jetson 到 STM32。
- `DIRECTION=FB`：STM32 到 Jetson。
- `LEN` 包含 `LEN` 自身、`FUNC`、全部 `PAYLOAD` 和 `CHECKSUM`。
- 完整帧长度为 `LEN + 2`，载荷长度为 `LEN - 3`。
- `CHECKSUM` 是从 `LEN` 到最后一个载荷字节的无符号 8 位累加和；
  不包含 `FF`、方向字节和校验字节本身。

### 0x12 运动帧

```text
偏移  字段
0     FF
1     FC
2     0A
3     12
4     mode
5     vx_lo
6     vx_hi
7     vy_lo
8     vy_hi
9     vz_lo
10    vz_hi
11    checksum
```

`Vx`、`Vy`、`Vz` 都是小端、有符号 16 位整数。解析成功只产生数据与时间戳，
不刷新全局看门狗、不调用状态机，也不控制电机。

## 旧协议黄金向量

```text
MOTION 极值，Vx=-32768、Vy=0、Vz=32767：
FF FC 0A 12 01 00 80 00 00 FF 7F 1B

SELF_CHECK 响应：
FF FC 05 51 01 00 57

REQUEST_DATA 请求 VERSION：
FF FC 05 50 51 00 A6

SELF_CHECK 请求，STM32 到 Jetson：
FF FB 04 51 00 55

INSPECT_CTRL start，STM32 到 Jetson：
FF FB 04 A3 01 A8

EMERGENCY asserted，STM32 到 Jetson：
FF FB 04 A4 01 A9

MODE_REPORT standby，STM32 到 Jetson：
FF FB 04 A5 01 AA
```

## 功能码占用

| 功能码 | 名称 | 载荷长度 | 说明 |
|---|---|---:|---|
| `0x12` | MOTION | 7 | 旧协议 |
| `0x50` | REQUEST_DATA | 2 | 旧协议，`LEN=5` |
| `0x51` | VERSION / SELF_CHECK | 1 或 2 | 旧协议冲突兼容入口 |
| `0xA3` | LEGACY_INSPECT_CTRL | 1 | 旧协议，`LEN=4` |
| `0xA4` | LEGACY_EMERGENCY | 1 | 旧协议，`LEN=4` |
| `0xA5` | LEGACY_MODE_REPORT | 1 | 旧协议，`LEN=4` |
| `0xB0` | INSPECT_REQUEST | 0 | 新分配 |
| `0xB1` | INSPECT_ACCEPTED | 0 | 新分配 |
| `0xB2` | INSPECT_REJECTED | 0 | 新分配 |
| `0xB3` | INSPECT_FINISHED | 0 | 新分配 |
| `0xB4` | INSPECT_CANCELLED | 0 | 新分配 |
| `0xB5` | STATUS | 4 | 新分配，载荷语义待业务层定义 |
| `0xB6` | HEARTBEAT | 1 | 新分配，载荷语义待业务层定义 |

旧代码扫描未发现 `0xB0`～`0xB6` 被占用。`INSPECT_REQUEST`、接受、拒绝、完成和
取消已接入组合层；STATUS和HEARTBEAT载荷语义仍未接入业务层。

## 流式解析与容量

- 最大载荷容量 32 字节，最大帧缓存 37 字节；每个已知功能码仍执行精确载荷长度校验。
- 解析器支持单字节输入、半帧、连续帧、噪声重同步、错误长度/校验和后的恢复。
- 帧内超时为 100 ms，使用无符号时间差和 `>=`，可处理 32 位 tick 回绕。
- 接收队列有 4 个实际可用槽；满后丢弃新帧并累计溢出计数。

## USART1 DMA 边界

传输层使用当前 HAL 的 `HAL_UARTEx_ReceiveToIdle_DMA()`，配合已配置为 Circular
的 DMA1 Channel 5。HT、TC 或 IDLE 回调只按 DMA 位置把新增字节复制到固定软件环形
缓冲，协议解析在 `poll()` 的主循环上下文执行。

- DMA 位置等于缓冲区大小时按一次尾段处理并归一到 0，重复位置不会重复交付。
- 软件环形缓冲满时统计溢出事件和丢弃字节数。
- UART 错误只在回调中标记恢复；`poll()` 丢弃半帧并停止、重启 DMA。
- DMA 停止或重启失败会计数并保持 STOPPED，不会伪装为恢复成功。
- 若中断被阻塞到 DMA 在两次可见回调之间绕回超过一整圈，硬件不提供可恢复的圈数；
  该情况只能通过保证 HT/TC 中断及时服务及后续硬件压力测试确认。
- `jetson_uart_transport_send_async()` 复制一个不超过37字节的帧并启动USART1 TX DMA；同一时刻只允许一个发送，完成回调只发布事件，由前台清除忙状态。
- 传输层已在 `main()` 中启动。当前只会因状态机动作发送巡检请求/取消帧，不发送MSSD速度；尚未进行真实Jetson兼容性验证。
