/*
 * 模块：算法数学工具
 * 设计目的：提供控制算法库内部通用的限幅、角度归一化、三角函数、配置校验和三相量运算工具。
 * 设计原理：用标准 C 数学函数实现 pu 角度的 sin/cos（查表法），把基础边界检查和三相量辅助运算集中在本模块。
 * 设计要点：只依赖标准 C；不得加入平台侧、设备侧或工程侧相关判断。
 * 输入：标量值、pu 角度、CONFIG 指针或三相量。
 * 输出：限幅值、归一化角度、三角函数值、配置是否合法的布尔结果或运算后的三相量。
 * 输出给谁用：所有需要基础数学能力和三相量运算的算法模块。
 */
#ifndef MATH_H_
#define MATH_H_

#include "types.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#ifndef SQRT3
#define SQRT3 1.73205080756887729353f
#endif

/* 限幅：将 value 限制在 [min, max] 范围内 */
real_t sat(real_t value, real_t max, real_t min);

/* pu 角度归一化：将任意 pu 角度归一化到 [0, 1) */
real_t wrap_unit(real_t value);

/* pu 角度正弦查表：sin(2π × angle_pu)，256 点表 + 线性插值 */
real_t sin_pu(real_t angle_pu);

/* pu 角度余弦查表：cos(2π × angle_pu)，256 点表 + 线性插值 */
real_t cos_pu(real_t angle_pu);

/* 配置合法性校验：检查 CONFIG 关键字段是否有效 */
int config_is_valid(const CONFIG *config);

/* 三相量加法：result = left + right（逐相相加） */
ABC add_abc(ABC left, ABC right);

/* 三相量清零：返回全零三相量 */
ABC zero_abc(void);

#endif
