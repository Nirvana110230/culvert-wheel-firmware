# 轮式巡检底盘固件 · STM32F103ZET6

本工程维护涵洞巡检机器人轮式底盘的独立遥控固件。板端入口为 `Core/Src/main.c::main`，轮式配置统一由 `Application/Src/chassis_app.c::chassis_app_wheel_rc_config()` 生成。Debug 和 Release 使用同一套轮式参数与控制逻辑。实车烧录使用交付目录中的 Release BIN。[正式固件烧录与使用说明](../../wheel-firmware/正式固件烧录与使用说明.md)

`Bsp` 处理 TIM3/TIM8 捕获、串口 DMA 和 RS485 换向。`Services/rc_input` 解析接收器，`Application` 执行模式切换、运动监督与差速换算，`Protocol` 负责 MSSD 和保留的上位机协议。独立档在本机完成初始化，依据遥控和真实电机反馈行驶。[控制链路](docs/chassis-control.md)

## 构建

在本目录执行下列命令。PATH 应含 CMake、Ninja 和 GNU Arm Embedded 工具。每种构建均输出 ELF、BIN、HEX 和 MAP 到对应的 `build` 子目录。

```powershell
cmake --preset Release
cmake --build --preset Release
cmake --preset Debug
cmake --build --preset Debug
```

在带本机 GCC 的 Linux 或 WSL 中，从本目录运行已有的 Host Tests。

```sh
cmake -S tests -B build/Host -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Host
ctest --test-dir build/Host --output-on-failure --no-tests=error
```

## 维护与历史

`.ioc` 记录 CubeMX 外设配置。保留 `Core` 的 USER CODE 区、启动文件和链接脚本，业务源在顶层 CMake 清单注册。工程文档中的早期离线设计、默认锁和板级核查清单记录当时的开发状态。本版参数、当前运行逻辑、实测范围和交付镜像以本入口及正式报告为准。[原 Keil 对照报告](../../wheel-firmware/原Keil工程与新正式轮式固件详细对照报告.md)
