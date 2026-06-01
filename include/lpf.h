/*
 * 模块：单极点 IIR 低通滤波
 * 设计目的：用 O(1) 内存实现低通滤波，替代滑动窗口均值的 O(N) 内存方案。
 * 设计原理：y[n] = alpha * x[n] + (1 - alpha) * y[n-1]，只需 2 个 float 状态。
 * 设计要点：alpha=1.0 时直通（不过滤）；alpha=0 时保持上一次值；
 *          alpha 可直接设置或从截止频率自动计算。
 * 输入：LPF 对象和当前样本 insert_data。
 * 输出：average 字段为滤波后输出。
 * 输出给谁用：通用分量提取模块读取 average 作为滤波后的 d/q 分量。
 */
#ifndef LPF_H_
#define LPF_H_

#include "types.h"

/*
 * 初始化 LPF：从截止频率和采样频率自动计算 alpha
 * cutoff_hz - 目标截止频率（Hz）
 * sample_hz - 采样频率（Hz）
 * alpha = 2π × fc × Ts / (1 + 2π × fc × Ts)
 */
void lpf_init(LPF *obj, real_t cutoff_hz, real_t sample_hz);

/*
 * 执行一次 IIR 滤波
 * insert_data - 当前样本值
 * 滤波结果写入 obj->average
 */
void lowpass_filter(LPF *obj, real_t insert_data);

#endif
