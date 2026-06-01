/*
 * 模块：Park 坐标变换
 * 设计目的：把静止 alpha/beta 量转换到旋转 dq 坐标，便于在旋转坐标中提取或控制分量。
 * 设计原理：按 pu 角度计算 sin/cos，将静止坐标向量投影到 d/q 轴。
 * 设计要点：函数无内部状态，角度含义由调用方确定，可复用于基波、谐波和序分量。
 * 输入：ALPHA_BETA alpha_beta 和 angle_pu。
 * 输出：DQ。
 * 输出给谁用：PLL 相关计算、通用分量提取、谐波提取和 dq 控制反馈生成流程。
 */
#ifndef PARK_H_
#define PARK_H_

#include "types.h"

DQ park(ALPHA_BETA alpha_beta, real_t angle_pu);

#endif
