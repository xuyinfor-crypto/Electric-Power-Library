/*
 * 模块：dq 双轴控制
 * 设计目的：在同步旋转 dq 坐标中对 d 轴和 q 轴分别闭环，供谐波、序分量等控制复用。
 * 设计原理：d/q 各自复用 PID 控制器，闭环输出再叠加 dq 前馈并限幅。
 * 设计要点：模块只处理 dq 结构体，不关心坐标系来自基波、谐波还是序分量；坐标变换由调用方负责。
 * 输入：DQ_CONTROL_INPUT 中的 dq ref、dq fdb、dq feedforward，以及控制器 PID 参数。
 * 输出：pi_out 为双轴闭环输出，dq_out 为叠加前馈和限幅后的 dq 输出。
 * 输出给谁用：谐波控制、正负序控制或其他需要 dq 轴闭环的纯算法模块。
 */
#ifndef DQ_CONTROL_H_
#define DQ_CONTROL_H_

#include "types.h"

void run_dq_control(DQ_CONTROLLER *obj, const DQ_CONTROL_INPUT *input);

#endif
