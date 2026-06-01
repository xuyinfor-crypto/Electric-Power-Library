/*
 * 模块：电流环编排
 * 设计目的：把前馈、电压外环和三相电流内环串成一次完整的电流环计算。
 * 设计原理：优先使用 PLL 角度，未就绪时使用外部输入角度；电压环生成附加电流参考，电流环生成最终三相输出。
 * 设计要点：本模块只编排算法模块，不访问任何平台侧或工程侧信息；状态变化写回 CONTROL_ALGO 对象。
 * 输入：CONTROL_ALGO 中的命令状态、直流电压 ref/fdb、电流 ref/fdb、配置和内部控制器状态。
 * 输出：vdc_pi_out、current_pi_out、output.phase 和 current_loop 状态位。
 * 输出给谁用：ISR 编排模块和应用层总输出读取这些结果，继续交给外部执行层使用。
 */
#ifndef CURRENT_LOOP_H_
#define CURRENT_LOOP_H_

#include "types.h"

void run_current_loop(CONTROL_ALGO *obj);

#endif
