/*
 * 模块：谐波电流控制（通用设计）
 * 设计目的：对各次谐波在各自旋转 dq 坐标中闭环控制，生成谐波补偿电压指令。
 * 设计原理：使用配置表指定需要处理的谐波次数，用 for 循环遍历所有配置的谐波次数。
 * 设计要点：每个谐波独立控制、独立输出、最后求和；模块内部复用 dq_control 能力。
 *          输入为 dq 格式的检测值和参考值；输出为 abc 格式的电压指令。
 * 输入：各次谐波 dq 检测值、各次谐波 dq 参考值、基波角度。
 * 输出：各次谐波 abc 参考值和总叠加参考值。
 * 输出给谁用：current_loop 读取 phase_ref，叠加基波电压指令和前馈后生成总电压指令。
 */
#include "harmonic_control.h"
#include "dq_control.h"
#include "component.h"
#include "math.h"

/* 单次谐波控制 */
static ABC run_one_harmonic(
    DQ_CONTROLLER *controller,
    const HARMONIC_COMPONENT *component,
    DQ ref,
    real_t angle
)
{
    DQ_CONTROL_INPUT control_input;

    if(controller == NULL || component == NULL) {
        return zero_abc();
    }

    control_input.ref = ref;
    control_input.fdb = component->detected_dq;
    control_input.feedforward.d = 0.0f;
    control_input.feedforward.q = 0.0f;

    /* dq 轴 PI 控制 */
    run_dq_control(controller, &control_input);

    /* 反变换到 abc */
    return rotate_dq_to_abc(controller->dq_out, &component->config, angle);
}

void harmonic_control_init(HARMONIC_CONTROL *obj, const HARMONIC_CONTROL_CONFIG *config)
{
    int i;

    if(obj == NULL || config == NULL) {
        return;
    }

    obj->config = *config;
    obj->phase_ref = zero_abc();

    /* 初始化各次谐波控制器 */
    for(i = 0; i < config->num_harmonics; i++) {
        /* d 轴控制器 */
        obj->controllers[i].d_pid.ref = 0.0f;
        obj->controllers[i].d_pid.fdb = 0.0f;
        obj->controllers[i].d_pid.kp = config->kp[i];
        obj->controllers[i].d_pid.ki = config->ki[i];
        obj->controllers[i].d_pid.kc = 0.0f;
        obj->controllers[i].d_pid.kd = 0.0f;
        obj->controllers[i].d_pid.out_max = config->output_limit[i];
        obj->controllers[i].d_pid.out_min = -config->output_limit[i];
        obj->controllers[i].d_pid.e = 0.0f;
        obj->controllers[i].d_pid.up = 0.0f;
        obj->controllers[i].d_pid.ui = 0.0f;
        obj->controllers[i].d_pid.ud = 0.0f;
        obj->controllers[i].d_pid.uprsat = 0.0f;
        obj->controllers[i].d_pid.out = 0.0f;
        obj->controllers[i].d_pid.saterr = 0.0f;
        obj->controllers[i].d_pid.up1 = 0.0f;

        /* q 轴控制器（与 d 轴相同） */
        obj->controllers[i].q_pid = obj->controllers[i].d_pid;

        obj->controllers[i].output_limit = config->output_limit[i];
        obj->controllers[i].pi_out.d = 0.0f;
        obj->controllers[i].pi_out.q = 0.0f;
        obj->controllers[i].dq_out.d = 0.0f;
        obj->controllers[i].dq_out.q = 0.0f;

        /* 初始化参考值 */
        obj->refs[i].d = 0.0f;
        obj->refs[i].q = 0.0f;
        obj->h_refs[i] = zero_abc();
    }
}

void harmonic_control_run(
    HARMONIC_CONTROL *obj,
    const HARMONIC_DETECT *detect,
    real_t base_angle
)
{
    int i;
    real_t harmonic_angle;

    if(obj == NULL || detect == NULL) {
        return;
    }

    obj->phase_ref = zero_abc();

    /* 遍历所有配置的谐波次数 */
    for(i = 0; i < obj->config.num_harmonics; i++) {
        /* 计算该次谐波角度 */
        harmonic_angle = base_angle * detect->config.orders[i].order;

        /* 执行单次谐波控制 */
        obj->h_refs[i] = run_one_harmonic(
            &obj->controllers[i],
            &detect->components[i],
            obj->refs[i],
            harmonic_angle
        );

        /* 累加到总参考 */
        obj->phase_ref = add_abc(obj->phase_ref, obj->h_refs[i]);
    }
}
