# 涵洞巡检机器人底盘固件

本目录是涵洞巡检机器人底盘的新固件工程，目标 MCU 为 STM32F103ZET6。工程由 STM32CubeMX 6.17.0 生成，使用 STM32CubeF1 V1.8.7、CMake 3.28.1 和 GNU Tools for STM32 13.3.1（`arm-none-eabi-gcc` 13.3.1）。

## 当前阶段

当前仅开展离线开发。工程包含 CubeMX 外设初始化代码和不依赖 HAL 的纯状态机骨架，但状态机尚未接入 `main()`，也没有 MSSD 写寄存器、Modbus 业务、BMS、松闸或非零速度发送实现。

状态机使用明确的 `REMOTE_ARMING` 和 `INSPECT_REQUESTED` 状态。一次性动作只表达清除命令、请求零速、请求抱闸、请求巡检或取消巡检；持续运动权限由当前状态的纯查询接口给出。权限仅代表上层授权，不代表立即松闸或发送速度。

严禁将当前版本直接连接电机试车。进入任何硬件测试前，必须先完成 [待硬件验证清单](docs/hardware-validation-pending.md)，制定接线检查、架空轮测试、急停验证和回滚方案。

## Release 构建

在本目录执行：

```powershell
cmake --preset Release
cmake --build --preset Release
```

主机侧状态机测试可在已有主机 C 编译器的环境中构建运行。测试工程位于 `tests/`，不依赖 STM32 HAL。

## CubeMX 重新生成注意事项

- `.ioc` 是外设配置的唯一生成输入，重新生成前先检查 Git 工作树并保存差异。
- 不要把业务代码放入 `Core/` 或 `cmake/stm32cubemx/`；业务代码保存在 `Application/`，主机测试保存在 `tests/`。
- 不直接修改 `cmake/stm32cubemx/CMakeLists.txt`。业务源文件和包含目录只在顶层 `CMakeLists.txt` 中接入。
- 重新生成后必须审查 `.ioc`、`Core/`、启动文件、链接脚本和生成 CMake 文件的差异。
- 重新生成后必须重新执行主机测试和 STM32 Release 构建，并再次核对 GPIO 初始安全电平。

更多约束见 [硬件映射](docs/hardware-map.md) 和 [安全需求](docs/safety-requirements.md)。
