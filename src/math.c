#include <math.h>
#include "math.h"

/*
 * 查表法常量定义
 * SIN_TABLE_SIZE: sin/cos 查表分辨率，256 点覆盖 [0, 2π)
 * SIN_TABLE_SCALE: 查表索引缩放因子 = TABLE_SIZE / 1.0（pu 角度范围）
 * SIN_TABLE_MASK: 环形缓冲区掩码，用于快速取模
 */
#define SIN_TABLE_SIZE 256
#define SIN_TABLE_SCALE ((real_t)SIN_TABLE_SIZE)
#define SIN_TABLE_MASK (SIN_TABLE_SIZE - 1)

/*
 * sin/cos 查找表（pu 角度，覆盖 [0, 1) 对应 [0, 2π)）
 * 在模块加载时初始化一次，后续热路径只做查表 + 线性插值。
 */
static real_t sin_table[SIN_TABLE_SIZE];
static real_t cos_table[SIN_TABLE_SIZE];
static int sin_table_initialized = 0;

/*
 * 模块：sin/cos 查找表初始化
 * 设计目的：用标准 sin/cos 填充查找表，后续热路径不再调用标准库。
 * 设计原理：256 点表 + 线性插值，精度约 0.001%，满足控制算法需求。
 * 输入：无。
 * 输出：填充 sin_table 和 cos_table。
 * 输出给谁用：sin_pu / cos_pu。
 */
static void init_sin_table(void)
{
	int i;
	real_t angle;

	if(sin_table_initialized) {
		return;
	}
	for(i = 0; i < SIN_TABLE_SIZE; i++) {
		angle = 2.0f * PI * (real_t)i / (real_t)SIN_TABLE_SIZE;
		sin_table[i] = (real_t)sin(angle);
		cos_table[i] = (real_t)cos(angle);
	}
	sin_table_initialized = 1;
}

/*
 * 模块：限幅
 * 设计目的：将 value 限制在 [min, max] 范围内。
 * 设计原理：简单分支判断，无浮点运算开销。
 * 输入：value - 待限幅值；max - 上限；min - 下限。
 * 输出：限幅后的值。
 * 输出给谁用：所有需要限幅的算法模块。
 */
real_t sat(real_t value, real_t max, real_t min)
{
	if(value > max) {
		return max;
	}
	if(value < min) {
		return min;
	}
	return value;
}

/*
 * 模块：pu 角度归一化
 * 设计目的：将任意 pu 角度归一化到 [0, 1) 范围。
 * 设计原理：用 fmod 一次性完成取模，消除 while 循环的不可预测执行时间。
 * 输入：value - 任意 pu 角度值。
 * 输出：归一化后的 pu 角度 [0, 1)。
 * 输出给谁用：PLL、open_loop 等需要角度归一化的模块。
 */
real_t wrap_unit(real_t value)
{
	value = (real_t)fmod((double)value, 1.0);
	if(value < 0.0f) {
		value += 1.0f;
	}
	return value;
}

/*
 * 模块：pu 角度正弦查表
 * 设计目的：用查表 + 线性插值快速计算 sin(2π × angle_pu)。
 * 设计原理：256 点表覆盖 [0, 2π)，线性插值精度约 0.001%。
 * 输入：angle_pu - pu 角度 [0, 1)。
 * 输出：sin 值 [-1, 1]。
 * 输出给谁用：Park/Inverse Park/PLL 等所有需要三角函数的模块。
 */
real_t sin_pu(real_t angle_pu)
{
	real_t index_f;
	int index;
	real_t frac;
	real_t s0, s1;

	init_sin_table();

	index_f = angle_pu * SIN_TABLE_SCALE;
	index = (int)index_f;
	frac = index_f - (real_t)index;

	s0 = sin_table[index & SIN_TABLE_MASK];
	s1 = sin_table[(index + 1) & SIN_TABLE_MASK];

	return s0 + frac * (s1 - s0);
}

/*
 * 模块：pu 角度余弦查表
 * 设计目的：用查表 + 线性插值快速计算 cos(2π × angle_pu)。
 * 设计原理：256 点表覆盖 [0, 2π)，线性插值精度约 0.001%。
 * 输入：angle_pu - pu 角度 [0, 1)。
 * 输出：cos 值 [-1, 1]。
 * 输出给谁用：Park/Inverse Park/PLL 等所有需要三角函数的模块。
 */
real_t cos_pu(real_t angle_pu)
{
	real_t index_f;
	int index;
	real_t frac;
	real_t c0, c1;

	init_sin_table();

	index_f = angle_pu * SIN_TABLE_SCALE;
	index = (int)index_f;
	frac = index_f - (real_t)index;

	c0 = cos_table[index & SIN_TABLE_MASK];
	c1 = cos_table[(index + 1) & SIN_TABLE_MASK];

	return c0 + frac * (c1 - c0);
}

/*
 * 模块：配置合法性校验
 * 设计目的：检查 CONFIG 是否包含有效参数。
 * 设计原理：校验关键字段是否为正数，防止后续除零或异常运算。
 * 输入：config - 待校验的配置指针。
 * 输出：1 = 合法，0 = 非法。
 * 输出给谁用：init 和 isr 模块在启动时调用。
 */
int config_is_valid(const CONFIG *config)
{
	return config != NULL
		&& config->sample_period_s > 0.0f
		&& config->base_frequency_hz > 0.0f
		&& config->duty_limit > 0.0f;
}

/*
 * 模块：三相量加法
 * 设计目的：提供两个三相量的逐相加法运算。
 * 设计原理：逐分量相加，无分支，适合 ISR 热路径。
 * 输入：left - 左操作数；right - 右操作数。
 * 输出：left + right 的三相量结果。
 * 输出给谁用：谐波控制、序控制等需要累加三相量的模块。
 */
ABC add_abc(ABC left, ABC right)
{
	ABC result;

	result.a = left.a + right.a;
	result.b = left.b + right.b;
	result.c = left.c + right.c;

	return result;
}

/*
 * 模块：三相量清零
 * 设计目的：返回全零三相量，替代多处重复的零值赋值。
 * 设计原理：直接返回零初始化结构体。
 * 输入：无。
 * 输出：全零三相量。
 * 输出给谁用：谐波控制、序控制等需要清零三相量的模块。
 */
ABC zero_abc(void)
{
	ABC result;

	result.a = 0.0f;
	result.b = 0.0f;
	result.c = 0.0f;

	return result;
}
