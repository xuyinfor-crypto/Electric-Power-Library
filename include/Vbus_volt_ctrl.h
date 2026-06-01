/*
 * 模块：直流母线电压控制
 * 设计目的：根据直流电压参考和反馈生成有功电流参考（d 轴）。
 * 设计原理：电压误差经 PID 输出 d 轴调节量，作为有功电流参考。
 * 设计要点：模块只处理归一化控制量；直流量的采样、标定和工程侧换算必须由调用方在库外完成。
 * 输入：REF_FDB dc_voltage，以及电压控制器内部 PID 参数。
 * 输出：active_ref（有功电流参考，d 轴）。
 * 输出给谁用：current_loop 读取，作为 fundamental_curr_ctrl 的 d 轴参考。
 */
#ifndef VBUS_VOLT_CTRL_H_
#define VBUS_VOLT_CTRL_H_

#include "types.h"

/* 直流母线电压控制器对象 */
typedef struct {
    PID pid;                /* PID 控制器 */
    real_t q_ref;           /* q 轴参考（无功） */
    real_t q_feedforward;   /* q 轴前馈 */
    real_t output_limit;    /* 输出限幅 */
    DQ dq_ref;              /* dq 参考 */
    real_t active_ref;      /* 有功电流参考（d 轴） */
    ABC phase_ref;          /* 三相参考（abc） */
} VBUS_VOLT_CTRL;

/* 函数原型 */
void Vbus_volt_ctrl_init(VBUS_VOLT_CTRL *obj);
void Vbus_volt_ctrl_run(VBUS_VOLT_CTRL *obj, REF_FDB dc_voltage, real_t active_current_fdb);

#endif /* VBUS_VOLT_CTRL_H_ */
