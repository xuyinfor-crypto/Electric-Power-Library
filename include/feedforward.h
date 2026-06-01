/*
 * 模块：三相前馈生成
 * 设计目的：根据给定角度和配置前馈量生成零序居中的三相前馈。
 * 设计原理：先将 dq 前馈反变换为 abc，再减去三相最大最小值的中点以提升三相对称利用率。
 * 设计要点：只使用归一化算法量和角度，不绑定任何执行载体；输出写入 CONTROL_ALGO 内部中间量。
 * 输入：CONTROL_ALGO 配置中的 feedforward，以及调用方传入的 angle_pu。
 * 输出：inter.feedforward.a/b/c。
 * 输出给谁用：电流控制模块把该三相前馈叠加到三相电流闭环输出。
 */
#ifndef FEEDFORWARD_H_
#define FEEDFORWARD_H_

#include "types.h"

void calc_feedforward(CONTROL_ALGO *obj, real_t angle_pu);

#endif
