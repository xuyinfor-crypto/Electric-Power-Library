/*
 * 模块：直流母线电压控制
 * 设计目的：根据直流电压参考和反馈生成有功电流参考（d 轴）。
 * 设计原理：电压误差经 PID 输出 d 轴调节量，作为有功电流参考。
 * 设计要点：模块只处理归一化控制量；直流量的采样、标定和工程侧换算必须由调用方在库外完成。
 * 输入：REF_FDB dc_voltage，以及电压控制器内部 PID 参数。
 * 输出：active_ref（有功电流参考，d 轴）。
 * 输出给谁用：current_loop 读取，作为 fundamental_curr_ctrl 的 d 轴参考。
 */
#include "Vbus_volt_ctrl.h"
#include "pid.h"
#include "math.h"

void Vbus_volt_ctrl_init(VBUS_VOLT_CTRL *obj)
{
    if(obj == NULL) {
        return;
    }

    obj->pid.ref = 0.0f;
    obj->pid.fdb = 0.0f;
    obj->pid.kp = 0.0f;
    obj->pid.ki = 0.0f;
    obj->pid.kc = 0.0f;
    obj->pid.kd = 0.0f;
    obj->pid.out_max = 0.0f;
    obj->pid.out_min = 0.0f;
    obj->pid.e = 0.0f;
    obj->pid.up = 0.0f;
    obj->pid.ui = 0.0f;
    obj->pid.ud = 0.0f;
    obj->pid.uprsat = 0.0f;
    obj->pid.out = 0.0f;
    obj->pid.saterr = 0.0f;
    obj->pid.up1 = 0.0f;

    obj->q_ref = 0.0f;
    obj->q_feedforward = 0.0f;
    obj->output_limit = 0.0f;
    obj->dq_ref.d = 0.0f;
    obj->dq_ref.q = 0.0f;
    obj->active_ref = 0.0f;
    obj->phase_ref = zero_abc();
}

void Vbus_volt_ctrl_run(VBUS_VOLT_CTRL *obj, REF_FDB dc_voltage, real_t active_current_fdb)
{
    if(obj == NULL) {
        return;
    }

    /* PID 控制 */
    obj->pid.ref = dc_voltage.ref;
    obj->pid.fdb = dc_voltage.fdb;
    pid_calc(&obj->pid);

    /* 有功电流参考（d 轴）= -pid.out - q_feedforward */
    obj->active_ref = -obj->pid.out - obj->q_feedforward;

    /* 限幅 */
    if(obj->output_limit > 0.0f) {
        obj->active_ref = sat(obj->active_ref, obj->output_limit, -obj->output_limit);
    }

    /* dq 参考 */
    obj->dq_ref.d = obj->active_ref;
    obj->dq_ref.q = obj->q_ref;
}
