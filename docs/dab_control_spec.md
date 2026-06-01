# DAB 控制模块化改造 Spec

## 0. 范围说明

本文基于 [dab_control_principles.md](./dab_control_principles.md)。

默认假设：

1. `DAB = Dual Active Bridge`
2. 第一版目标是“在现有 SVG 并网控制之外，新增独立 DAB 控制链路”
3. 第一版先做 **SPS 单移相**
4. 第一版优先保证**边界清楚、模块清楚、保护清楚**

本文是 spec，不是代码已完成声明。

## 1. 目标

本次改造要达到：

1. 在 `control_algo_lib` 内新增独立的 DAB 纯算法模块。
2. DAB 能基于 `Vdc1/Vdc2` 生成相移命令，而不是侵入现有三相电流环。
3. 代码结构按小模块拆分，每个模块职责单一。
4. 所有硬件相关内容继续留在应用层/适配层。
5. 为后续真正写代码和接硬件提供可执行的文件级蓝图。

## 2. 非目标

这版 spec 不要求：

1. 一次性支持 DPS/TPS
2. 一次性支持所有硬件寄存器映射
3. 一次性打通 Modbus/共享内存地址表
4. 一次性替换现有 DSPA/DSPB 全部流程

## 3. 架构结论

新增 DAB 后，算法库结构变成：

```text
PLL / 三相电流环 / 谐波补偿 / SPWM         -> 现有 SVG 主链路
DAB mode / Vdc2 loop / power FF / phase shift -> 新增 DAB 独立链路
```

二者关系：

1. 共用部分输入采样数据，但输出不同。
2. 三相链路输出 `abc duty`。
3. DAB 链路输出 `phase_shift_cmd`。
4. 最终在应用层分别送到不同 PWM 执行路径。

## 4. 文件级拆分方案

## 4.1 新增头文件

计划新增：

1. `include/dab_mode.h`
2. `include/dab_vdc_ctrl.h`
3. `include/dab_power_feedforward.h`
4. `include/dab_phase_shift.h`
5. `include/dab_softstart.h`
6. `include/dab_protection.h`
7. `include/dab_control.h`

## 4.2 新增源文件

计划新增：

1. `src/dab_mode.c`
2. `src/dab_vdc_ctrl.c`
3. `src/dab_power_feedforward.c`
4. `src/dab_phase_shift.c`
5. `src/dab_softstart.c`
6. `src/dab_protection.c`
7. `src/dab_control.c`

## 4.3 必须修改的现有文件

1. `include/types.h`
2. `include/control_algo.h`
3. `include/init.h`
4. `include/isr.h`
5. `src/init.c`
6. `src/isr.c`
7. `CMakeLists.txt`
8. `README.md`

## 4.4 保持不进算法库的内容

不进入 `control_algo_lib`：

1. `src/f28xpwm.c`
2. `src/hal.c`
3. `src/gcc_dspa.c`
4. `src/gcc_dspb.c`
5. `src/modbus.c`
6. 与 DSPA/DSPB SHM 地址绑定的读写逻辑

这些内容属于硬件适配和工程集成层。

## 5. 数据结构改造

## 5.1 `types.h` 新增枚举和字

建议新增：

```c
typedef enum {
    DAB_ST_STOP = 0,
    DAB_ST_READY,
    DAB_ST_SOFTSTART,
    DAB_ST_RUN,
    DAB_ST_DERATING,
    DAB_ST_FAULT
} DAB_STATE;

typedef enum {
    DAB_DIR_FORWARD = 1,
    DAB_DIR_REVERSE = -1,
    DAB_DIR_AUTO = 0
} DAB_DIRECTION_MODE;
```

建议新增状态/命令/故障字：

```c
typedef union {
    unsigned int raw;
    struct {
        unsigned int enabled:1;
        unsigned int ready:1;
        unsigned int softstart:1;
        unsigned int running:1;
        unsigned int derating:1;
        unsigned int fault:1;
        unsigned int reserved:10;
    } bits;
} DAB_ST_WORD;

typedef union {
    unsigned int raw;
    struct {
        unsigned int run_dab:1;
        unsigned int reset_dab_fault:1;
        unsigned int reserved:14;
    } bits;
} DAB_CMD_WORD;

typedef union {
    unsigned int raw;
    struct {
        unsigned int bad_config:1;
        unsigned int vdc1_ov:1;
        unsigned int vdc1_uv:1;
        unsigned int vdc2_ov:1;
        unsigned int vdc2_uv:1;
        unsigned int over_current:1;
        unsigned int softstart_timeout:1;
        unsigned int phase_shift_sat:1;
        unsigned int reserved:8;
    } bits;
} DAB_FAULT_WORD;
```

## 5.2 `types.h` 新增 DAB 配置

建议新增：

```c
typedef struct {
    real_t switching_frequency_hz;
    real_t leakage_inductance_h;
    real_t turn_ratio;
    real_t vdc2_ref;
    real_t vdc2_kp;
    real_t vdc2_ki;
    real_t power_ref_max;
    real_t power_ref_min;
    real_t phase_shift_max_rad;
    real_t phase_shift_slew_rad_per_s;
    real_t vdc1_ov_limit;
    real_t vdc1_uv_limit;
    real_t vdc2_ov_limit;
    real_t vdc2_uv_limit;
    real_t current_limit;
    unsigned int outer_loop_divider;
} DAB_CONFIG;
```

## 5.3 `types.h` 新增 DAB 输入输出

建议新增：

```c
typedef struct {
    real_t vdc1;
    real_t vdc2;
    real_t idab;
    real_t vdc2_ref;
    int enable;
    DAB_DIRECTION_MODE direction_mode;
} DAB_INPUT;

typedef struct {
    real_t power_ref;
    real_t power_ref_limited;
    real_t phase_shift_ref_rad;
    real_t phase_shift_cmd_rad;
    real_t phase_shift_cmd_pu;
    DAB_STATE state;
    DAB_ST_WORD st_word;
    DAB_FAULT_WORD fault_word;
} DAB_OUTPUT;
```

## 5.4 `types.h` 新增子模块对象

建议新增：

```c
typedef struct {
    DAB_STATE state;
    int direction_sign;
    unsigned int softstart_counter;
} DAB_MODE;

typedef struct {
    PID pid;
    real_t power_ref;
} DAB_VDC_CTRL;

typedef struct {
    real_t power_gain;
    real_t power_ref_limited;
} DAB_POWER_FEEDFORWARD;

typedef struct {
    real_t phase_shift_ref_rad;
    real_t phase_shift_cmd_rad;
    real_t phase_shift_cmd_pu;
} DAB_PHASE_SHIFT;

typedef struct {
    real_t ramp_output_rad;
} DAB_SOFTSTART;

typedef struct {
    DAB_FAULT_WORD fault_word;
    real_t derate_power_limit;
} DAB_PROTECTION;

typedef struct {
    DAB_CONFIG config;
    DAB_INPUT input;
    DAB_OUTPUT output;
    DAB_MODE mode;
    DAB_VDC_CTRL vdc_ctrl;
    DAB_POWER_FEEDFORWARD power_ff;
    DAB_PHASE_SHIFT phase_shift;
    DAB_SOFTSTART softstart;
    DAB_PROTECTION protection;
    unsigned int slow_loop_prescaler;
} DAB_CONTROL;
```

## 5.5 `CONTROL_ALGO` 顶层对象改造

建议在 `INTER` 或 `CONTROL_ALGO` 内加入：

```c
DAB_CONTROL dab;
```

并在 `OUTPUT` 中加入只读回传量：

```c
real_t dab_phase_shift_cmd_pu;
real_t dab_power_ref;
unsigned int dab_state;
unsigned int dab_fault_word;
```

## 6. API 设计

## 6.1 `dab_control.h` 对外接口

建议最少提供：

```c
void dab_control_init(DAB_CONTROL *obj, const DAB_CONFIG *config);
void dab_control_run(DAB_CONTROL *obj);
```

## 6.2 子模块接口

建议提供：

```c
void dab_mode_run(DAB_MODE *obj, const DAB_INPUT *input, const DAB_PROTECTION *protection);
void dab_vdc_ctrl_init(DAB_VDC_CTRL *obj, const DAB_CONFIG *config);
void dab_vdc_ctrl_run(DAB_VDC_CTRL *obj, real_t ref, real_t fdb);
void dab_power_feedforward_run(DAB_POWER_FEEDFORWARD *obj, const DAB_CONFIG *config, real_t vdc1, real_t vdc2, real_t power_ref);
void dab_phase_shift_run(DAB_PHASE_SHIFT *obj, const DAB_CONFIG *config, real_t power_gain, real_t power_ref);
void dab_softstart_run(DAB_SOFTSTART *obj, const DAB_CONFIG *config, DAB_STATE state, real_t target_phase_shift_rad);
void dab_protection_run(DAB_PROTECTION *obj, const DAB_CONFIG *config, const DAB_INPUT *input, real_t phase_shift_cmd_rad);
```

## 6.3 `control_algo.h` 顶层新增接口

建议新增：

```c
static inline void set_dab_input(CONTROL_ALGO *obj, DAB_INPUT input);
static inline DAB_CONTROL *get_dab_ctrl(CONTROL_ALGO *obj);
static inline const DAB_OUTPUT *get_dab_output(const CONTROL_ALGO *obj);
```

可选：

```c
static inline void set_dab_enable(CONTROL_ALGO *obj, int enable);
static inline void set_dab_bus_voltage(CONTROL_ALGO *obj, real_t vdc1, real_t vdc2);
static inline void set_dab_current_feedback(CONTROL_ALGO *obj, real_t idab);
```

## 7. 调度设计

## 7.1 `init.c` 改造要求

`init()` 需要：

1. 初始化 `obj->inter.dab`
2. 配置默认 `DAB_CONFIG`
3. 清零状态、故障、相移输出
4. 不自动强制打开 DAB

注意：

现有 `init.c` 里会默认打开 `run_pll` 和 `run_current_loop`。DAB 不建议默认开。

## 7.2 `isr.c` 改造要求

建议顺序：

1. 校验总配置
2. 跑 PLL
3. 跑原有 `open_loop` 或 `current_loop`
4. 跑 `dab_control_run`

建议伪代码：

```c
if(obj->cmd_word.bits.run_pll) {
    run_grid_pll(obj);
}

if(obj->cmd_word.bits.run_open_loop) {
    run_open_loop(obj);
} else if(obj->cmd_word.bits.run_current_loop) {
    run_current_loop(obj);
}

if(obj->inter.dab.input.enable) {
    dab_control_run(&obj->inter.dab);
}
```

## 7.3 DAB 慢环分频

要求：

1. `dab_control_run()` 每个 ISR 都可调用。
2. 外环 PI 不必每次都更新。
3. `outer_loop_divider` 控制慢环分频。

示意：

```c
if(++obj->slow_loop_prescaler >= obj->config.outer_loop_divider) {
    obj->slow_loop_prescaler = 0;
    // run voltage loop
}
// 每次 ISR 都跑 protection / slew / output update
```

## 8. 模块内职责细化

## 8.1 `dab_mode`

必须负责：

1. `STOP/READY/SOFTSTART/RUN/DERATING/FAULT` 转换
2. 方向切换前先拉回零相移
3. 复位时清故障锁存

不能负责：

1. 直接写 PWM
2. 直接算功率公式

## 8.2 `dab_vdc_ctrl`

必须负责：

1. `Vdc2` 误差转 `P_ref`
2. PI 积分反饱和

不能负责：

1. 相移求解
2. 保护判定

## 8.3 `dab_power_feedforward`

必须负责：

1. 当前电压工况下的功率增益计算
2. 可达功率限制

不能负责：

1. 状态机
2. 输出时序

## 8.4 `dab_phase_shift`

必须负责：

1. `P_ref -> phi_ref`
2. `phi_ref -> phi_cmd`
3. 斜率限制

不能负责：

1. 直接决定是否允许运行

## 8.5 `dab_softstart`

必须负责：

1. 启动时缓升
2. 关断/切向时缓降

## 8.6 `dab_protection`

必须负责：

1. 采样合法性检查
2. 母线阈值检查
3. 电流阈值检查
4. 输出故障字

## 8.7 `dab_control`

必须负责：

1. 串起所有子模块
2. 生成单一对外结果
3. 保存中间调试量

## 9. 输出定义

算法库对应用层至少输出：

1. `phase_shift_cmd_rad`
2. `phase_shift_cmd_pu`
3. `power_ref`
4. `state`
5. `fault_word`

应用层收到后再自己决定：

1. 怎么映射到 ePWM
2. 怎么同步桥臂
3. 怎么写共享内存/Modbus

## 10. 验收标准

## 10.1 结构验收

满足以下条件才算拆分合格：

1. DAB 没有塞进 `current_loop.c`
2. 每个功能点各自有独立 `.h/.c`
3. 硬件寄存器访问不进入 `control_algo_lib`
4. `CMakeLists.txt` 可独立编译新增模块

## 10.2 行为验收

至少满足：

1. 未使能时输出 0 相移
2. `Vdc2` 低于参考时输出正向功率命令
3. `Vdc2` 高于参考且允许双向时输出反向功率命令
4. `Vdc1` 欠压时禁止继续加大向前送能
5. 故障触发时输出立即归零并锁存故障

## 10.3 参数验收

至少保证：

1. 错误参数会置 `bad_config`
2. `phase_shift_max_rad <= pi/2`
3. `outer_loop_divider >= 1`
4. `switching_frequency_hz > 0`
5. `leakage_inductance_h > 0`
6. `turn_ratio > 0`

## 11. 场景测试表

## 11.1 输入 -> 输出

### 场景 1：禁用

输入：

1. `enable = 0`
2. `Vdc1 = 正常`
3. `Vdc2 = 正常`

期望输出：

1. `state = STOP`
2. `phase_shift_cmd = 0`
3. `fault = 0`

### 场景 2：正向送能

输入：

1. `enable = 1`
2. `Vdc1 = 780V`
3. `Vdc2 = 710V`
4. `Vdc2_ref = 760V`

期望输出：

1. `state` 最终进入 `RUN`
2. `power_ref > 0`
3. `phase_shift_cmd > 0`

### 场景 3：反向回能

输入：

1. `enable = 1`
2. `Vdc1 = 760V`
3. `Vdc2 = 800V`
4. `Vdc2_ref = 760V`

期望输出：

1. `power_ref < 0`
2. `phase_shift_cmd < 0`

### 场景 4：一次侧欠压

输入：

1. `Vdc1 < uv_limit`
2. `Vdc2 < ref`

期望输出：

1. 不允许继续扩大正向相移
2. 进入 `DERATING` 或 `FAULT`

### 场景 5：过流

输入：

1. `idab > current_limit`

期望输出：

1. `fault.over_current = 1`
2. `phase_shift_cmd = 0`
3. `state = FAULT`

## 11.2 单次操作链路追踪

操作：

1. 上位机使能 DAB
2. 下发 `Vdc2_ref`
3. 采样返回 `Vdc1/Vdc2`

状态变化：

1. `dab_mode` 从 `STOP -> READY -> SOFTSTART -> RUN`

各渲染/输出函数拿到的数据：

1. `dab_vdc_ctrl` 拿 `ref/fdb`
2. `dab_power_feedforward` 拿 `P_ref + V1/V2`
3. `dab_phase_shift` 拿 `P_ref_limited + gain`
4. `dab_softstart` 拿 `phi_ref`
5. `dab_protection` 拿 `Vdc1/Vdc2/idab`

最终结果：

1. 算法库输出 `phase_shift_cmd_pu`
2. 应用层把它翻译成 DAB PWM 时序
3. 上位机显示 `state = RUN`，`Vdc2` 逼近参考

如果这一条链路在代码实现阶段列不清楚，就说明 DAB 模块还没拆干净。

## 12. 集成步骤

建议按这个顺序落代码：

1. 先改 `types.h` 和 `control_algo.h`，把 DAB 数据边界搭好。
2. 再写 `dab_mode`、`dab_vdc_ctrl`、`dab_phase_shift` 三个最核心模块。
3. 再补 `dab_power_feedforward`、`dab_softstart`、`dab_protection`。
4. 再写 `dab_control` 总编排。
5. 最后才接入 `isr.c`、应用层 PWM 适配、通信映射。

原因：

1. 先立边界，后填实现，能避免把硬件细节反向污染算法库。
2. 先跑通最小闭环，再补软启动和保护，更容易定位问题。

## 13. 风险和待确认项

这些是当前 spec 里明确保留的待确认项，不做假设性承诺：

1. 硬件上的 DAB 电流采样是否存在，采样点在哪里。
2. DAB 主副桥具体由哪个 DSP 或 FPGA 负责输出时序。
3. `Vdc2` 的真实业务目标是稳压、均压、隔离供电，还是双向储能接口。
4. 是否需要双向回能，还是只需要单向 `Vdc1 -> Vdc2`。
5. 共享内存和 Modbus 是否已经预留足够地址。

这些不影响先写纯算法 spec，但会影响最终接线和接口命名。

## 14. 本 spec 的最终落点

实现完成后，仓库应新增一条清晰的新链路：

```text
输入 Vdc1/Vdc2/idab
-> DAB 状态机
-> Vdc2 外环
-> 功率前馈
-> 相移求解
-> 软启动/保护
-> 输出 phase_shift_cmd
```

这条链路必须和现有三相 `current_loop` 并列存在，而不是混成一个“大而全”的控制函数。
