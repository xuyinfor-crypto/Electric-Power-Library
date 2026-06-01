/*
 * 模块：基波电流控制
 * 设计目的：在 dq 坐标系下对基波电流进行闭环控制，生成基波电压指令。
 * 设计原理：d 轴控制有功电流，q 轴控制无功电流；PI 控制后反变换到 abc。
 * 设计要点：输入使用 dq 格式的参考和反馈；输出为 abc 格式的电压指令。
 * 输入：有功电流参考（d 轴）、无功电流参考（q 轴）、基波电流反馈（dq）。
 * 输出：基波电压指令（abc）。
 * 输出给谁用：current_loop 读取，叠加谐波电压指令和前馈后生成总电压指令。
 */
#include "fundamental_curr_ctrl.h"
#include "dq_control.h"
#include "inverse_park.h"
#include "inverse_clarke.h"
#include "math.h"

/* 固定的正序基波配置（order=1, POSITIVE, gain=1, phase_offset=0） */
static const COMPONENT_CONFIG base_config = {
    1,                          /* order */
    SEQUENCE_POSITIVE,          /* sequence */
    1.0f,                       /* gain */
    0.0f,                       /* phase_offset */
    0.0f,                       /* dq_limit（运行时由 output_limit 覆盖） */
    1.0f                        /* power_scale */
};

void fundamental_curr_ctrl_init(FUNDAMENTAL_CURR_CTRL *obj)
{
    if(obj == NULL) {
        return;
    }

    /* 初始化 dq 控制器 */
    obj->dq_ctrl.d_pid.ref = 0.0f;
    obj->dq_ctrl.d_pid.fdb = 0.0f;
    obj->dq_ctrl.d_pid.kp = 0.0f;
    obj->dq_ctrl.d_pid.ki = 0.0f;
    obj->dq_ctrl.d_pid.kc = 0.0f;
    obj->dq_ctrl.d_pid.kd = 0.0f;
    obj->dq_ctrl.d_pid.out_max = 0.0f;
    obj->dq_ctrl.d_pid.out_min = 0.0f;
    obj->dq_ctrl.d_pid.e = 0.0f;
    obj->dq_ctrl.d_pid.up = 0.0f;
    obj->dq_ctrl.d_pid.ui = 0.0f;
    obj->dq_ctrl.d_pid.ud = 0.0f;
    obj->dq_ctrl.d_pid.uprsat = 0.0f;
    obj->dq_ctrl.d_pid.out = 0.0f;
    obj->dq_ctrl.d_pid.saterr = 0.0f;
    obj->dq_ctrl.d_pid.up1 = 0.0f;

    obj->dq_ctrl.q_pid = obj->dq_ctrl.d_pid;

    obj->dq_ctrl.output_limit = 0.0f;
    obj->dq_ctrl.pi_out.d = 0.0f;
    obj->dq_ctrl.pi_out.q = 0.0f;
    obj->dq_ctrl.dq_out.d = 0.0f;
    obj->dq_ctrl.dq_out.q = 0.0f;

    /* 配置 */
    obj->config = base_config;
    obj->output_limit = 0.0f;
}

void fundamental_curr_ctrl_run(
    FUNDAMENTAL_CURR_CTRL *obj,
    const FUNDAMENTAL_CURR_CTRL_INPUT *input,
    FUNDAMENTAL_CURR_CTRL_OUTPUT *output,
    real_t angle
)
{
    DQ_CONTROL_INPUT control_input;
    ALPHA_BETA alpha_beta;

    if(obj == NULL || input == NULL || output == NULL) {
        return;
    }

    /* 构造 dq 控制输入 */
    control_input.ref = input->ref;
    control_input.fdb = input->fdb;
    control_input.feedforward = input->feedforward;

    /* dq 轴 PI 控制 */
    run_dq_control(&obj->dq_ctrl, &control_input);

    /* 输出 dq 电压指令 */
    output->voltage_dq = obj->dq_ctrl.dq_out;

    /* 反变换到 abc */
    /* 先更新配置中的 dq_limit */
    obj->config.dq_limit = obj->output_limit;

    /* inverse Park: dq → alpha/beta */
    alpha_beta = inverse_park(obj->dq_ctrl.dq_out, angle);

    /* inverse Clarke: alpha/beta → abc */
    output->voltage_abc = inverse_clarke(alpha_beta, 0.0f);
}
