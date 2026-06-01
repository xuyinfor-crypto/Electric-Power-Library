/*
 * 模块：周期平均值计算
 * 设计目的：在固定样本数窗口内计算 v 和 i 两个通道的平均值。
 * 设计原理：逐样本累加，计数到 sample_count 后输出平均值并清空累计量。
 * 设计要点：本模块只处理标量统计，不假设 v/i 的物理来源或工程侧含义。
 * 输入：MEAN 对象中的 v、i、sample_count 和累计状态。
 * 输出：vo、io、cnt、vacc、iacc。
 * 输出给谁用：诊断、统计或上层控制逻辑读取平均值结果。
 */
#ifndef MEAN_H_
#define MEAN_H_

#include "types.h"

void calc_mean(MEAN *obj);

#endif
