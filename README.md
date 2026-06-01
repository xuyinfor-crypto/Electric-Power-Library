# control_algo 纯算法静态库

本目录是控制算法静态库边界。它不依赖芯片、硬件寄存器、外设驱动、产品型号、产品额定值、CCS 工程、TI `IQmathLib.h` 或工程级 `parameter.h`。

库内只使用标准 C 头文件：

```text
math.h
string.h
```

采样周期、基波频率、PLL 参数、限幅、前馈、开环指令等全部由调用方通过 `CONTROL_ALGO_CONFIG` 显式注入。

## 编译输出

Windows 批处理入口：

```bat
build_control_algo_lib.bat
```

该脚本自动尝试 MSVC、GCC 或 Clang，输出：

```text
build\control_algo.lib
```

CMake 入口：

```bat
cmake -S . -B build
cmake --build build --config Release
```

## 文件职责

### 头文件（include/）

```text
include/types.h                      公共类型定义
include/control_algo.h               总入口聚合头
include/clarke.h                     abc -> alpha/beta
include/park.h                       alpha/beta -> dq
include/inverse_clarke.h             alpha/beta -> abc
include/inverse_park.h               dq -> alpha/beta
include/component.h                  可复用旋转坐标分量提取核心
include/harmonic_detect.h            谐波电流检测（通用，支持任意次数）
include/harmonic_control.h           谐波电流控制（通用，支持任意次数）
include/dq_control.h                 dq 轴双通道控制
include/sequence.h                   正序/负序提取与序分量控制
include/feedforward.h                前馈三相生成
include/pid.h                        PID 单功能计算
include/pi_design.h                  考虑延时的固定 N 形式 PI 参数设计
include/Vbus_volt_ctrl.h             直流母线电压控制
include/fundamental_curr_detect.h    基波电流检测
include/fundamental_curr_ctrl.h      基波电流控制（dq 坐标系）
include/current_loop.h               电流环总控制
include/spwm.h                       SPWM 调制
include/pll.h                        电网 PLL
include/open_loop.h                  开环调制
include/isr.h                        ISR 周期编排
include/mean.h                       均值计算
include/lpf.h                        滑窗低通
include/math.h                       限幅、角度归一化、sin/cos 查表、配置校验、三相量运算
include/init.h                       CONTROL_ALGO 初始化
```

### 源文件（src/）

```text
src/clarke.c                         abc -> alpha/beta
src/park.c                           alpha/beta -> dq
src/inverse_clarke.c                 alpha/beta -> abc
src/inverse_park.c                   dq -> alpha/beta
src/component.c                      可复用旋转坐标分量提取核心
src/harmonic_detect.c                谐波电流检测（通用，支持任意次数）
src/harmonic_control.c               谐波电流控制（通用，支持任意次数）
src/dq_control.c                     dq 轴双通道控制
src/sequence.c                       正序/负序提取与序分量控制
src/feedforward.c                    前馈三相生成
src/pid.c                            PID 单功能计算
src/pi_design.c                      考虑延时的固定 N 形式 PI 参数设计
src/Vbus_volt_ctrl.c                 直流母线电压控制
src/fundamental_curr_detect.c        基波电流检测
src/fundamental_curr_ctrl.c          基波电流控制（dq 坐标系）
src/current_loop.c                   电流环总控制
src/spwm.c                           SPWM 调制
src/pll.c                            电网 PLL
src/open_loop.c                      开环调制
src/isr.c                            ISR 周期编排
src/mean.c                           均值计算
src/lpf.c                            滑窗低通
src/math.c                           限幅、角度归一化、sin/cos 查表、配置校验、三相量运算
src/init.c                           CONTROL_ALGO 初始化
```

硬规则：一个功能模块，一个 `.c` 和一个 `.h`。新增功能时必须先建同名模块头，再建同名实现文件。

## 架构设计

### 控制流程

```
┌─────────────────────────────────────────────────────────────────┐
│                    负载电流 / 电流反馈（abc）                      │
└─────────────────────────────────────────────────────────────────┘
                                │
                ┌───────────────┴───────────────┐
                ↓                               ↓
    ┌───────────────────────┐       ┌───────────────────────┐
    │ fundamental_curr_detect│       │    harmonic_detect    │
    │ （基波电流检测）         │       │   （谐波电流检测）      │
    │                        │       │                       │
    │ 输入：                  │       │ 输入：                 │
    │  - current_fdb (abc)   │       │  - current_fdb (abc)  │
    │  - angle (基波)         │       │  - angle (各次谐波)    │
    │                        │       │                       │
    │ 输出：                  │       │ 处理：                 │
    │  - detected_dq (dq)    │       │  - for循环遍历各次谐波  │
    │                        │       │  - 每次谐波独立滤波器   │
    └───────────┬───────────┘       │  - 提取dq分量          │
                │                    │                       │
                │                    │ 输出：                 │
                │                    │  - components[i].dq   │
                │                    └───────────┬───────────┘
                │                               │
                ↓                               │
    ┌───────────────────────┐                   │
    │  Vbus_volt_ctrl        │                   │
    │ （直流母线电压控制）     │                   │
    │                        │                   │
    │ 输入：                  │                   │
    │  - dc_voltage (ref/fdb)│                   │
    │  - detected_dq.d       │                   │
    │    (有功电流反馈)       │                   │
    │                        │                   │
    │ 输出：                  │                   │
    │  - active_ref (d轴)    │                   │
    └───────────┬───────────┘                   │
                │                               │
                │    ┌─────────────────────┐    │
                │    │    current_loop     │    │
                │    │  （电流环总控制）     │    │
                │    │                     │    │
                │    │ SVG/APF模式：        │    │
                │    │  refs[i] =          │    │
                │    │    detect[i].dq     │←───┘
                │    │                     │
                │    │ 其他模式：           │
                │    │  refs[i] = 外部输入  │
                │    └──────────┬──────────┘
                │               │
                ↓               ↓
    ┌───────────────────────┐
    │ harmonic_control      │
    │ （谐波电流控制）        │
    │                        │
    │ 输入：                  │
    │  - refs[i] (各次dq)    │←── current_loop 设置
    │  - detect[i].dq        │←── harmonic_detect
    │                        │
    │ 处理：                  │
    │  - for循环遍历各次谐波  │
    │  - 每次谐波独立PI控制   │
    │  - dq → abc           │
    │                        │
    │ 输出：                  │
    │  - 谐波电压指令 (abc)   │
    └───────────┬───────────┘
                │
                ↓
    ┌───────────────────────┐
    │ fundamental_curr_ctrl │
    │ （基波电流控制）         │
    │                        │
    │ 输入：                  │
    │  - active_ref (d轴)    │←── Vbus_volt_ctrl
    │  - reactive_ref (q轴)  │←── 外部输入
    │  - detected_dq (dq)    │←── fundamental_curr_detect
    │                        │
    │ 处理：                  │
    │  - dq轴PI控制          │
    │  - dq → abc           │
    │                        │
    │ 输出：                  │
    │  - 基波电压指令 (abc)   │
    └───────────┬───────────┘
                │
                │    ┌─────────────────────┐
                │    │    current_loop     │
                │    │  （电流环总控制）     │
                │    │                     │
                │    │ 叠加：              │
                │    │  - 基波电压指令      │
                │    │  - 谐波电压指令      │
                │    │  - 前馈电压          │
                │    │                     │
                │    │ 输出：              │
                │    │  - 总电压指令 (abc)  │
                │    └──────────┬──────────┘
                │               │
                ↓               ↓
    ┌───────────────────────────────┐
    │             SPWM              │
    │        （SPWM 调制）           │
    │                               │
    │ 输入：                         │
    │  - 总电压指令 (abc)            │
    │                               │
    │ 处理：                         │
    │  - 电压指令归一化              │
    │  - 三相平衡处理                │
    │  - 占空比限幅                  │
    │                               │
    │ 输出：                         │
    │  - 三相占空比 (abc)            │
    └───────────────┬───────────────┘
                    │
                    ↓
    ┌───────────────────────────────┐
    │         三相占空比 (abc)        │
    │              → PWM            │
    └───────────────────────────────┘
```

### 模块职责

| 模块 | 输入 | 输出 | 量纲 |
|------|------|------|------|
| `fundamental_curr_detect` | 电流反馈(abc)、PLL角度 | 基波电流检测值(dq) | 电流 |
| `Vbus_volt_ctrl` | 直流电压ref/fdb、有功电流反馈 | 有功电流参考(d轴) | 电流 |
| `fundamental_curr_ctrl` | 有功参考(d)、无功参考(q)、电流反馈(dq) | 基波电压指令(abc) | 电压 |
| `harmonic_detect` | 电流反馈(abc)、各次谐波角度 | 各次谐波检测值(dq) | 电流 |
| `harmonic_control` | 各次谐波检测值(dq)、谐波参考(dq) | 谐波电压指令(abc) | 电压 |
| `current_loop` | 以上所有 | 总电压指令(abc) | 电压 |
| `spwm` | 总电压指令(abc) | 三相占空比(abc) | 占空比 |

### 通用谐波设计

谐波检测和控制模块采用通用设计，支持任意次数的谐波：

```c
// 配置需要处理的谐波次数
HARMONIC_DETECT_CONFIG detect_config = {
    .num_harmonics = 4,
    .orders = {
        { .order = 5,  .sequence = SEQUENCE_POSITIVE, .gain = 1.0f, .phase_offset = 0.0f, .dq_limit = 0.5f, .power_scale = 1.0f },
        { .order = 7,  .sequence = SEQUENCE_POSITIVE, .gain = 1.0f, .phase_offset = 0.0f, .dq_limit = 0.5f, .power_scale = 1.0f },
        { .order = 11, .sequence = SEQUENCE_POSITIVE, .gain = 1.0f, .phase_offset = 0.0f, .dq_limit = 0.3f, .power_scale = 1.0f },
        { .order = 13, .sequence = SEQUENCE_POSITIVE, .gain = 1.0f, .phase_offset = 0.0f, .dq_limit = 0.3f, .power_scale = 1.0f },
    }
};

// 初始化
harmonic_detect_init(&obj->inter.harmonic_detect, &detect_config);
harmonic_control_init(&obj->inter.harmonic_control, &control_config);

// 运行（自动处理所有配置的谐波次数）
harmonic_detect_run(&obj->inter.harmonic_detect, current_fdb, base_angle);
harmonic_control_run(&obj->inter.harmonic_control, &obj->inter.harmonic_detect, base_angle);
```

## API 设计

主公开头文件是：

```text
include\control_algo.h
```

API 分为三层：

**输入层**（外部调用方写入采样量和命令）：

```c
set_cmd_word()                  /* 写入命令字 */
set_grid_voltage()              /* 写入三相电网电压 */
set_current_feedback()          /* 写入三相电流反馈 */
set_current_reference()         /* 写入三相电流参考 */
set_dc_voltage()                /* 写入直流电压 ref/fdb */
set_angle()                     /* 写入外部角度 */
```

**输出层**（外部调用方读取控制结果）：

```c
get_st_word()                   /* 读取状态字 */
get_fault_word()                /* 读取故障字 */
get_output()                    /* 读取控制输出（const 指针） */
```

**参数整定层**（供初始化和调试流程读写内部参数）：

```c
get_config()                    /* 读写配置 */
get_vbus_ctrl()                 /* 读写直流母线电压控制器 */
get_fundamental_ctrl()          /* 读写基波电流控制器 */
get_harmonic_detect()           /* 读写谐波检测器 */
get_harmonic_control()          /* 读写谐波控制器 */
```

## 迁移边界

已纳入静态库的纯算法：

```text
Clarke
inverse Clarke
Park
inverse Park
PLL
PID
PI parameter design
Harmonic detection（通用，支持任意次数）
Harmonic control（通用，支持任意次数）
Positive/negative sequence extraction
Positive/negative sequence control
Fundamental current detection
Fundamental current control（dq 坐标系）
DC bus voltage control
Ramp
Mean
Moving-window low-pass filter
```

必须留在应用层或适配层的模块：

```text
ADC/PWM/GPIO/SCI/SPI/FRAM/DPRAM/看门狗/中断向量
Modbus 协议收发和寄存器映射
DSPA/DSPB 共享内存地址表
产品额定值换算、工程宏、硬件通道表
接触器、风机、LED、FPGA RUN 引脚等执行动作
```

明确不纳入算法库：

```text
DSP281x / C2000 芯片头文件
PWM / ADC / GPIO / SCI / SPI / FRAM / DPRAM / 看门狗
Modbus / graph / OVCNTimeManager
DSPA / DSPB 产品额定参数和工程宏
Vdc_REF / Vabc_REF / Iabc_REF / Ilabc_REF / SAMPLE_FREQ / CARRIER_FREQ 等产品或工程常量
```

## 性能优化说明

| 优化项 | 原始实现 | 优化后 | 收益 |
|--------|---------|--------|------|
| sin/cos | 标准库 `<math.h>` | 256 点查表 + 线性插值 | ~10x 加速 |
| 角度归一化 | while 循环 | fmod 一次性取模 | 确定性执行时间 |
| PLL 幅值计算 | 标准库 sqrt | 快速逆平方根（位操作 + 牛顿迭代） | ~5x 加速 |
| 电压环配置 | 栈上临时对象 | static const + 运行时覆盖 | 降低栈深度 |

## 编码规范

1. **命名**：`模块名_动词_名词`，如 `harmonic_detect_run`
2. **空指针检查**：所有函数入口使用 `== NULL` 检查
3. **头文件保护**：`#ifndef MODULE_H_ / #define MODULE_H_ / #endif`
4. **注释**：每个 .h 文件包含标准注释块（目的/原理/要点/输入/输出/下游）
5. **ISR 安全**：禁止动态内存分配、递归、标准库浮点运算、while 循环
