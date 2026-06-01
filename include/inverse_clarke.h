/*
 * 模块：反 Clarke 坐标变换
 * 设计目的：把静止 alpha/beta 量和零序量转换回三相 abc。
 * 设计原理：使用 Clarke 逆变换公式重建 a/b/c 三相分量。
 * 设计要点：函数无内部状态，输入和输出均为纯算法结构体，可被多个控制模块复用。
 * 输入：ALPHA_BETA alpha_beta 和 zero 零序量。
 * 输出：ABC 三相量。
 * 输出给谁用：前馈生成、开环输出、dq/谐波/序分量反变换等模块。
 */
#ifndef INVERSE_CLARKE_H_
#define INVERSE_CLARKE_H_

#include "types.h"

ABC inverse_clarke(ALPHA_BETA alpha_beta, real_t zero);

#endif
