/*
 * 模块：电网相位锁定
 * 设计目的：从三相电压输入中估算同步角度和频率，作为闭环控制的角度基准。
 * 设计原理：先做 Clarke 变换，再根据当前估计角度计算 q 轴相位误差，通过 PI 型环路更新角度。
 * 设计要点：输入应为归一化三相量；PLL 参数和采样周期由配置提供，不在模块内固化。
 * 输入：CONTROL_ALGO 中的 grid_voltage、pll_kp、pll_ki、限幅、base_frequency_hz 和 sample_period_s。
 * 输出：output.pll_angle、output.pll_freq_hz、output.alpha_beta、pll_last_error 和 pll_ready 状态位。
 * 输出给谁用：电流环、谐波控制、正负序控制和应用层同步逻辑读取角度与频率。
 */
#ifndef PLL_H_
#define PLL_H_

#include "types.h"

void run_grid_pll(CONTROL_ALGO *obj);

#endif
