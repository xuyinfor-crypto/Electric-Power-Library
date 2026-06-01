/*
 * 模块：反 Park 坐标变换
 * 设计目的：把旋转 dq 坐标中的控制量转换到静止 alpha/beta 坐标。
 * 设计原理：按单位周期角度计算 sin/cos，将 dq 向量旋回静止坐标系。
 * 设计要点：角度使用 pu 单位，调用方负责确定角度对应的基波、谐波或序分量坐标系。
 * 输入：DQ dq 和 angle_pu。
 * 输出：ALPHA_BETA。
 * 输出给谁用：反 Clarke、前馈生成、开环输出、谐波控制和序分量控制模块。
 */
#ifndef INVERSE_PARK_H_
#define INVERSE_PARK_H_

#include "types.h"

ALPHA_BETA inverse_park(DQ dq, real_t angle_pu);

#endif
