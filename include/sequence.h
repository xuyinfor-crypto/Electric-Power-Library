/*
 * 模块：正负序提取与控制
 * 设计目的：提取三相量的正序、负序分量，并按配置生成序分量补偿参考。
 * 设计原理：正负序复用通用旋转分量提取；控制输出则把给定 dq 参考按序方向反变换到 abc。
 * 设计要点：提取和控制使用同一套 component 配置语义，便于正序、负序和谐波模块复用。
 * 输入：提取时输入三相量、角度和使能标志；控制时输入序分量 dq_ref、config、angle 和限幅。
 * 输出：positive_ref、negative_ref、phase_ref、power_square 或控制器 phase_ref。
 * 输出给谁用：上层补偿参考合成、电流参考生成和诊断统计流程。
 */
#ifndef SEQUENCE_H_
#define SEQUENCE_H_

#include "types.h"

void extract_sequence(SEQUENCE_EXTRACTOR *obj, const COMPONENT_INPUT *input);
void run_sequence_control(SEQUENCE_CONTROLLER *obj, real_t angle);

#endif
