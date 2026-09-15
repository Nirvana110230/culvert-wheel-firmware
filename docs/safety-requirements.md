# 安全需求

## 离线阶段硬边界

- 禁止烧录当前开发版本。
- 禁止连接或控制电机。
- 禁止实现或发送MSSD写寄存器命令。
- 禁止实现松闸。
- 禁止生成或发送任何非零速度命令。
- 在完成硬件验证前，不得声称急停距离、零速抱闸时序、遥控失联行为或MSSD超时保护已经验证。

## 状态机安全规则

- 上电状态机必须为 `APP_STATE_BOOT_SAFE`，模式选择无效。
- 状态机是纯C决策模块，不得包含HAL头文件或访问GPIO、UART、TIM、DMA、MSSD。
- 一次性动作仅表达清除运动命令、请求零速、请求抱闸、请求巡检和取消巡检；不得用动作位图表达持续运动权限。
- 持续权限必须由状态查询：只有REMOTE允许遥控运动，只有INSPECTING允许Jetson运动，两种权限不得同时成立。
- 权限只是授权，不等于立即松闸或发送速度；任何硬件执行层必须另行设计、审查和测试。
- 未知状态、未知事件或损坏的模式值必须进入FAULT并请求零速和抱闸。
- 已知但当前状态不适用的事件必须安全忽略，不改变状态、模式或产生动作；急停、指定状态RC失联、故障和适用状态下的超时仍必须产生安全动作。
- CH7急停在所有非FAULT状态具有最高状态切换优先级，结果为EMERGENCY并请求零速和抱闸。
- MOTOR_FAULT和自检失败进入FAULT；FAULT不得由普通遥控或Jetson事件解除。
- RC失联从BOOT_SAFE、REMOTE_ARMING、REMOTE、STANDBY、INSPECT_REQUESTED和INSPECTING进入FAULT。
- BOOT_SAFE、STANDBY、INSPECT_REQUESTED、INSPECTING或EMERGENCY切向遥控时，必须先进入REMOTE_ARMING；只有确认CH1/CH2中位后才能进入REMOTE。
- CH5仅在STANDBY有效，并进入明确的INSPECT_REQUESTED状态；只有Jetson接受后才能进入INSPECTING。
- 巡检请求被拒绝、取消或请求超时后回到STANDBY；请求期间切换遥控时取消请求并进入REMOTE_ARMING。
- INSPECTING遇到完成、取消、运动超时、模式切换、急停、RC失联或电机故障时，必须清除运动命令并安全退出。
- 所有控制源切换必须产生 `APP_ACTION_CLEAR_ALL_MOTION_COMMANDS`。
- 状态机上下文不得保存速度命令。

## 后续硬件执行层的准入条件

硬件执行层不得仅依据“允许运动”动作直接驱动车辆。至少还应独立检查命令新鲜度、遥控有效性、急停输入、电机故障、实际零速反馈和刹车时序，并具备复位后的安全默认输出。具体参数均属于待硬件验证项。
