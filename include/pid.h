/*
 * 模块：PID 控制器
 * 设计目的：提供可复用的单输入单输出 PID 计算核心。
 * 设计原理：按 ref-fdb 计算误差，生成比例、积分、微分项，并用饱和误差做抗积分饱和补偿。
 * 设计要点：所有参数和状态都保存在 PID 中，调用方负责初始化限幅和增益。
 * 输入：PID 中的 ref、fdb、kp、ki、kc、kd、out_max、out_min 和历史状态。
 * 输出：e、up、ui、ud、uprsat、out、saterr 和 up1。
 * 输出给谁用：电压控制、电流控制、dq 控制和其他需要单轴闭环的模块。
 */
#ifndef PID_H_
#define PID_H_

#include "types.h"

void pid_calc(PID *obj);

#endif
