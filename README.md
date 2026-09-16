# 涵洞巡检机器人底盘固件

本目录是涵洞巡检机器人底盘的新固件工程，目标 MCU 为 STM32F103ZET6。工程由 STM32CubeMX 6.17.0 生成，使用 STM32CubeF1 V1.8.7、CMake 3.28.1 和 GNU Tools for STM32 13.3.1（`arm-none-eabi-gcc` 13.3.1）。

## 当前阶段

当前仅开展离线开发。RC、Jetson、状态机、运动监督、运动学、MSSD/Modbus 和RS485 DMA已通过非阻塞组合层接入 `main()`。启动后只允许接收通信、读取MSSD配置/状态并执行安全停止；默认配置关闭方向映射、最大RPM和应用运动授权，因此任何RC或Jetson非零命令都不能形成非零RPM事务。

状态机使用明确的 `REMOTE_ARMING` 和 `INSPECT_REQUESTED` 状态。一次性动作只表达清除命令、请求零速、请求抱闸、请求巡检或取消巡检；持续运动权限由当前状态的纯查询接口给出。权限仅代表上层授权，不代表立即松闸或发送速度。

严禁将当前版本直接连接电机试车。进入任何硬件测试前，必须先完成 [待硬件验证清单](docs/hardware-validation-pending.md)，制定接线检查、架空轮测试、急停验证和回滚方案。

## Release 构建

在本目录执行：

```powershell
cmake --preset Release
cmake --build --preset Release
```

主机侧测试可在已有主机 C 编译器的环境中构建运行。测试工程位于 `tests/`，HAL边界使用测试桩，不访问真实串口或硬件。RC模块设计和待验证参数见 [RC输入模块离线设计](docs/rc-input.md)，主链路与默认锁见 [底盘控制主链路](docs/chassis-control.md)。

## CubeMX 重新生成注意事项

- `.ioc` 是外设配置的唯一生成输入，重新生成前先检查 Git 工作树并保存差异。
- 不要把业务代码放入 `Core/` 或 `cmake/stm32cubemx/`；状态机保存在 `Application/`，HAL适配保存在 `Bsp/`，纯服务逻辑保存在 `Services/`，主机测试保存在 `tests/`。
- 不直接修改 `cmake/stm32cubemx/CMakeLists.txt`。业务源文件和包含目录只在顶层 `CMakeLists.txt` 中接入。
- 重新生成后必须审查 `.ioc`、`Core/`、启动文件、链接脚本和生成 CMake 文件的差异。
- 重新生成后必须重新执行主机测试和 STM32 Release 构建，并再次核对 GPIO 初始安全电平。

更多约束见 [硬件映射](docs/hardware-map.md) 和 [安全需求](docs/safety-requirements.md)。
