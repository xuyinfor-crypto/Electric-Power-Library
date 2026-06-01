/*
 * 模块：谐波电流检测（通用设计）
 * 设计目的：从电流反馈中提取指定阶次谐波的 dq 分量，为谐波控制提供反馈量。
 * 设计原理：使用配置表指定需要处理的谐波次数，用 for 循环遍历所有配置的谐波次数。
 * 设计要点：本模块只负责提取和统计，不直接完成谐波闭环控制；控制动作交给 harmonic_control。
 *          每个谐波次数有独立的低通滤波器。
 * 输入：电流反馈（abc）、基波角度、谐波配置表。
 * 输出：各次谐波组件的 detected_dq、limited_dq、power_square，并汇总 power_square。
 * 输出给谁用：谐波 dq 控制模块和诊断统计逻辑读取这些谐波反馈与强度信息。
 */
#include "harmonic_detect.h"
#include "clarke.h"
#include "park.h"
#include "math.h"

/* 单次谐波分量提取 */
static void extract_one_harmonic(
    ABC current_fdb,
    real_t angle,
    HARMONIC_ORDER_CONFIG *cfg,
    HARMONIC_COMPONENT *comp
)
{
    ALPHA_BETA alpha_beta;
    DQ dq;
    real_t harmonic_angle;

    if(cfg == NULL || comp == NULL) {
        return;
    }

    /* 计算该次谐波的角度 */
    harmonic_angle = angle * cfg->order;

    /* Clarke 变换：abc → alpha/beta */
    alpha_beta = clarke(current_fdb);

    /* Park 变换：alpha/beta → dq（各次谐波旋转坐标系） */
    dq = park(alpha_beta, harmonic_angle);

    /* 低通滤波 */
    lowpass_filter(&comp->d_lpf, dq.d);
    lowpass_filter(&comp->q_lpf, dq.q);

    /* 保存检测值 */
    comp->detected_dq.d = comp->d_lpf.average;
    comp->detected_dq.q = comp->q_lpf.average;

    /* 限幅 */
    comp->limited_dq = sat_dq(comp->detected_dq, cfg->dq_limit);

    /* 计算功率 */
    comp->power_square = comp->limited_dq.d * comp->limited_dq.d * cfg->power_scale
                       + comp->limited_dq.q * comp->limited_dq.q * cfg->power_scale;
}

void harmonic_detect_init(HARMONIC_DETECT *obj, const HARMONIC_DETECT_CONFIG *config)
{
    int i;

    if(obj == NULL || config == NULL) {
        return;
    }

    obj->config = *config;
    obj->phase_ref = zero_abc();
    obj->power_square = 0.0f;

    /* 初始化各次谐波组件 */
    for(i = 0; i < config->num_harmonics; i++) {
        obj->components[i].config = config->orders[i];
        obj->components[i].d_lpf.alpha = 1.0f;  /* 默认直通，可后续配置 */
        obj->components[i].d_lpf.y_prev = 0.0f;
        obj->components[i].d_lpf.average = 0.0f;
        obj->components[i].q_lpf.alpha = 1.0f;
        obj->components[i].q_lpf.y_prev = 0.0f;
        obj->components[i].q_lpf.average = 0.0f;
        obj->components[i].detected_dq.d = 0.0f;
        obj->components[i].detected_dq.q = 0.0f;
        obj->components[i].limited_dq.d = 0.0f;
        obj->components[i].limited_dq.q = 0.0f;
        obj->components[i].phase_ref = zero_abc();
        obj->components[i].power_square = 0.0f;
    }
}

void harmonic_detect_run(HARMONIC_DETECT *obj, ABC current_fdb, real_t base_angle)
{
    int i;

    if(obj == NULL) {
        return;
    }

    obj->phase_ref = zero_abc();
    obj->power_square = 0.0f;

    /* 遍历所有配置的谐波次数 */
    for(i = 0; i < obj->config.num_harmonics; i++) {
        /* 提取该次谐波分量 */
        extract_one_harmonic(current_fdb, base_angle,
                            &obj->config.orders[i],
                            &obj->components[i]);

        /* 累加功率 */
        obj->power_square += obj->components[i].power_square;
    }
}
