/*
 * 模块：Clarke 坐标变换
 * 设计目的：把三相 abc 量转换到静止 alpha/beta 坐标，供后续 Park、PLL 和分量提取使用。
 * 设计原理：采用幅值一致的三相到两相变换，并可同时计算零序平均量。
 * 设计要点：输入必须是已经归一化的纯算法量；函数无内部状态，便于复用和单元测试。
 * 输入：ABC phase，以及可选的 zero 输出指针。
 * 输出：ALPHA_BETA；zero 指针非空时输出三相平均零序。
 * 输出给谁用：PLL、通用分量提取、正负序提取、谐波提取以及需要 alpha/beta 的控制模块。
 */
#ifndef CLARKE_H_
#define CLARKE_H_

#include "types.h"

ALPHA_BETA clarke(ABC phase, real_t *zero);

#endif
