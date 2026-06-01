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
#ifndef SPWM_H_
#define SPWM_H_

#include "types.h"

/* SPWM 配置 */
typedef struct {
    real_t duty_max;          /* 最大占空比 */
    real_t duty_min;          /* 最小占空比 */
    real_t dead_time;         /* 死区时间（归一化） */
    real_t min_pulse_width;   /* 最小脉宽（归一化） */
} SPWM_CONFIG;

/* SPWM 对象 */
typedef struct {
    SPWM_CONFIG config;       /* 配置 */
    ABC duty_out;             /* 三相占空比输出 */
} SPWM;

/* 函数原型 */
void spwm_init(SPWM *obj, const SPWM_CONFIG *config);
void spwm_run(SPWM *obj, ABC voltage_ref);

#endif /* SPWM_H_ */
