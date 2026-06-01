/*
 * 模块：单极点 IIR 低通滤波
 * 设计目的：用 O(1) 内存实现低通滤波，替代滑动窗口均值的 O(N) 内存方案。
 * 设计原理：y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 *          alpha = 2π × fc × Ts / (1 + 2π × fc × Ts)
 * 输入：LPF 对象和当前样本。
 * 输出：average 字段为滤波后输出。
 * 输出给谁用：通用分量提取模块。
 */
#include <math.h>
#include "lpf.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/*
 * 模块：LPF 初始化
 * 设计目的：从截止频率和采样频率自动计算 alpha 系数。
 * 设计原理：一阶 IIR 低通的双线性变换近似：
 *          alpha = 2π × fc × Ts / (1 + 2π × fc × Ts)
 * 输入：cutoff_hz - 目标截止频率（Hz）；sample_hz - 采样频率（Hz）。
 * 输出：obj 中 alpha 被设置，y_prev 和 average 清零。
 */
void lpf_init(LPF *obj, real_t cutoff_hz, real_t sample_hz)
{
	real_t ts;
	real_t wc_ts;

	if(obj == NULL) {
		return;
	}

	if(sample_hz <= 0.0f || cutoff_hz <= 0.0f || cutoff_hz >= sample_hz * 0.5f) {
		/* 截止频率无效或超过奈奎斯特频率，设为直通 */
		obj->alpha = 1.0f;
	}
	else {
		ts = 1.0f / sample_hz;
		wc_ts = 2.0f * PI * cutoff_hz * ts;
		obj->alpha = wc_ts / (1.0f + wc_ts);
	}

	obj->y_prev = 0.0f;
	obj->average = 0.0f;
}

/*
 * 模块：IIR 低通滤波计算
 * 设计目的：每周期执行一次一阶 IIR 滤波。
 * 设计原理：y = alpha * x + (1 - alpha) * y_prev
 *          alpha=1.0 时 y=x（直通）；alpha=0 时 y=y_prev（保持）。
 * 输入：insert_data - 当前样本值。
 * 输出：obj->average 为滤波后输出。
 */
void lowpass_filter(LPF *obj, real_t insert_data)
{
	if(obj == NULL) {
		return;
	}

	obj->average = obj->alpha * insert_data + (1.0f - obj->alpha) * obj->y_prev;
	obj->y_prev = obj->average;
}
