/*
 * 模块：锁相环
 * 设计目的：从三相电网电压中提取相位角和频率，为坐标变换提供同步角度。
 * 设计原理：Clarke 变换得到 alpha/beta，通过 PI 控制器调节估算角度使 q 轴分量趋零。
 * 设计要点：使用快速逆平方根替代标准库 sqrt，消除 ISR 路径上的高开销浮点运算。
 * 输入：三相电网电压（归一化量）、PI 参数和基波频率配置。
 * 输出：pll_angle（pu 角度）、pll_freq_hz（频率）和 pll_ready 状态位。
 * 输出给谁用：电流环编排、开环输出等需要同步角度的模块。
 */
#include "pll.h"
#include "clarke.h"
#include "math.h"

/*
 * 模块：快速逆平方根
 * 设计目的：用位操作 + 一次牛顿迭代快速计算 1/sqrt(x)，替代标准库 sqrt。
 * 设计原理：基于 IEEE 754 浮点数位表示的近似公式，单次迭代精度约 1%。
 * 输入：x - 正浮点数。
 * 输出：约等于 1/sqrt(x) 的近似值。
 * 输出给谁用：PLL 模块用于向量归一化。
 */
static real_t fast_inv_sqrt(real_t x)
{
	real_t half_x = 0.5f * x;
	int i = *(int *)&x;
	i = 0x5f3759df - (i >> 1);
	x = *(real_t *)&i;
	x = x * (1.5f - half_x * x * x);
	return x;
}

/*
 * 模块：PLL 主函数
 * 设计目的：每周期更新一次锁相环状态，输出电网角度和频率。
 * 设计原理：计算 alpha/beta 幅值用于归一化，q 轴误差经 PI 调节后叠加到角度累加器。
 * 输入：CONTROL_ALGO 总对象中的电网电压、PI 参数和历史状态。
 * 输出：pll_angle、pll_freq_hz、alpha_beta 和 pll_ready。
 * 输出给谁用：电流环、开环等需要同步角度的模块。
 */
void run_grid_pll(CONTROL_ALGO *obj)
{
	real_t mag_sq;
	real_t inv_mag;
	real_t q_error;
	real_t pll_out;
	real_t nominal_step;

	if(obj == NULL) {
		return;
	}

	obj->inter.alpha_beta = clarke(obj->input.grid_voltage, &obj->inter.zero);

	/* 用幅值平方做阈值判断，避免不必要的 sqrt 调用 */
	mag_sq = obj->inter.alpha_beta.alpha * obj->inter.alpha_beta.alpha
		+ obj->inter.alpha_beta.beta * obj->inter.alpha_beta.beta;

	if(mag_sq > 1.0e-12f) {
		/* 快速逆平方根：1/magnitude ≈ fast_inv_sqrt(mag_sq) */
		inv_mag = fast_inv_sqrt(mag_sq);
		q_error = (obj->inter.alpha_beta.beta * cos_pu(obj->output.pll_angle)
			- obj->inter.alpha_beta.alpha * sin_pu(obj->output.pll_angle)) * inv_mag;
	}
	else {
		q_error = 0.0f;
	}

	obj->inter.pll_ui += obj->config.pll_ki * q_error;
	pll_out = obj->config.pll_kp * q_error + obj->inter.pll_ui;
	pll_out = sat(pll_out, obj->config.pll_out_max, obj->config.pll_out_min);
	nominal_step = obj->config.base_frequency_hz * obj->config.sample_period_s;

	obj->output.pll_angle = wrap_unit(obj->output.pll_angle + nominal_step + pll_out * obj->config.sample_period_s);
	obj->output.pll_freq_hz = obj->config.base_frequency_hz + pll_out;
	obj->output.alpha_beta = obj->inter.alpha_beta;
	obj->inter.pll_last_error = q_error;
	obj->st_word.bits.pll_ready = 1;
}
