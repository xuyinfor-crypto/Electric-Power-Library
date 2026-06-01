/*
 * 模块：谐波 dq 控制
 * 设计目的：对各次谐波在各自旋转 dq 坐标中闭环控制，避免直接在 abc 中混合控制谐波量。
 * 设计原理：读取谐波提取模块得到的 detected_dq，每个谐波阶次复用 dq 双轴控制，再反变换回 abc 汇总。
 * 设计要点：每个谐波独立控制、独立输出、最后求和；模块内部复用 dq_control 和 component 反变换能力。
 * 输入：HARMONIC_CONTROL_INPUT 中的提取器指针、角度、使能标志和各次谐波 dq 参考。
 * 输出：h5_ref、h7_ref、h11_ref、h13_ref 以及汇总 phase_ref。
 * 输出给谁用：上层电流参考合成或补偿控制流程读取 phase_ref 作为谐波补偿参考。
 */
#ifndef HARMONIC_CONTROL_H_
#define HARMONIC_CONTROL_H_

#include "types.h"

void run_harmonic_control(HARMONIC_CONTROLLER *obj, const HARMONIC_CONTROL_INPUT *input);

#endif
