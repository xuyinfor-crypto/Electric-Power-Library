/*
 * 模块：SPWM 调制
 * 设计目的：将三相电压指令转换为三相占空比，用于 PWM 生成。
 * 设计原理：三相正弦调制波与三角载波比较，生成 PWM 信号。
 *          在数字控制中，直接将电压指令转换为占空比。
 * 设计要点：输入为三相电压指令（abc），范围 [-1, 1]。
 *          输出为三相占空比（abc），范围 [0, 1]。
 *          占空比 = (电压指令 + 1) / 2。
 * 输入：三相电压指令（abc）。
 * 输出：三相占空比（abc）。
 * 输出给谁用：PWM 模块读取占空比，生成 PWM 信号。
 */
#include "spwm.h"
#include "math.h"

void spwm_init(SPWM *obj, const SPWM_CONFIG *config)
{
    if(obj == NULL || config == NULL) {
        return;
    }

    obj->config = *config;
    obj->duty_out = zero_abc();
}

void spwm_run(SPWM *obj, ABC voltage_ref)
{
    real_t duty_a, duty_b, duty_c;

    if(obj == NULL) {
        return;
    }

    /* 电压指令限幅（范围 [-1, 1]） */
    duty_a = sat(voltage_ref.a, 1.0f, -1.0f);
    duty_b = sat(voltage_ref.b, 1.0f, -1.0f);
    duty_c = sat(voltage_ref.c, 1.0f, -1.0f);

    /* 转换为占空比：duty = (voltage + 1) / 2 */
    duty_a = (duty_a + 1.0f) * 0.5f;
    duty_b = (duty_b + 1.0f) * 0.5f;
    duty_c = (duty_c + 1.0f) * 0.5f;

    /* 占空比限幅 */
    duty_a = sat(duty_a, obj->config.duty_max, obj->config.duty_min);
    duty_b = sat(duty_b, obj->config.duty_max, obj->config.duty_min);
    duty_c = sat(duty_c, obj->config.duty_max, obj->config.duty_min);

    /* 最小脉宽限制 */
    if(obj->config.min_pulse_width > 0.0f) {
        /* 如果占空比接近0或1，强制拉到最小/最大值 */
        if(duty_a < obj->config.min_pulse_width) {
            duty_a = obj->config.min_pulse_width;
        } else if(duty_a > (1.0f - obj->config.min_pulse_width)) {
            duty_a = 1.0f - obj->config.min_pulse_width;
        }

        if(duty_b < obj->config.min_pulse_width) {
            duty_b = obj->config.min_pulse_width;
        } else if(duty_b > (1.0f - obj->config.min_pulse_width)) {
            duty_b = 1.0f - obj->config.min_pulse_width;
        }

        if(duty_c < obj->config.min_pulse_width) {
            duty_c = obj->config.min_pulse_width;
        } else if(duty_c > (1.0f - obj->config.min_pulse_width)) {
            duty_c = 1.0f - obj->config.min_pulse_width;
        }
    }

    /* 输出占空比 */
    obj->duty_out.a = duty_a;
    obj->duty_out.b = duty_b;
    obj->duty_out.c = duty_c;
}
