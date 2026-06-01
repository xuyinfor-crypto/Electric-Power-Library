/*
 * 模块：通用旋转分量提取
 * 设计目的：为正负序、谐波等周期分量提供统一的 dq 提取和 dq 到 abc 反变换能力。
 * 设计原理：按配置的阶次、序方向和相位偏置建立旋转坐标系，在该坐标系中提取 d/q 分量。
 * 设计要点：正序和负序只通过相序映射区分；限幅、增益、滤波和功率平方统计都保持在算法内部。
 * 输入：COMPONENT_INPUT 中的三相量、基波角度和使能标志，以及组件自身配置。
 * 输出：detected_dq、limited_dq、phase_ref 和 power_square。
 * 输出给谁用：谐波提取、正负序提取、谐波 dq 控制、正负序控制等上层纯算法模块。
 */
#ifndef COMPONENT_H_
#define COMPONENT_H_

#include "types.h"

ABC rotate_dq_to_abc(DQ dq, const COMPONENT_CONFIG *config, real_t angle_pu);
void extract_component(COMPONENT *obj, const COMPONENT_INPUT *input);

#endif
