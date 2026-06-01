/*
 * 模块：开环参考生成
 * 设计目的：在不运行闭环电流控制时，根据配置 dq 参考生成三相开环输出。
 * 设计原理：按基波频率和采样周期累加内部角度，再将 open_loop_dq 反变换为 abc 并限幅。
 * 设计要点：开环角度和输出均为归一化算法量；模块不绑定任何执行载体。
 * 输入：CONTROL_ALGO 中的 open_loop_dq、base_frequency_hz、sample_period_s、duty_limit 和命令状态。
 * 输出：output.phase、output.alpha_beta、output.pll_angle 和 open_loop 状态位。
 * 输出给谁用：应用层或测试流程读取三相输出，用于开环调试或模式切换验证。
 */
#ifndef OPEN_LOOP_H_
#define OPEN_LOOP_H_

#include "types.h"

void run_open_loop(CONTROL_ALGO *obj);

#endif
