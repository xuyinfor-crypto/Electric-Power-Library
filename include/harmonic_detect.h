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
#ifndef HARMONIC_DETECT_H_
#define HARMONIC_DETECT_H_

#include "types.h"

/* 最大支持的谐波次数数量 */
#define MAX_HARMONICS  16

/* 单次谐波配置 */
typedef struct {
    int order;           /* 谐波次数（5, 7, 11, 13, ...） */
    SEQUENCE sequence;   /* 正序/负序 */
    real_t gain;         /* 增益 */
    real_t phase_offset; /* 相位偏移 */
    real_t dq_limit;     /* dq 限幅 */
    real_t power_scale;  /* 功率缩放 */
} HARMONIC_ORDER_CONFIG;

/* 谐波检测器配置 */
typedef struct {
    int num_harmonics;                              /* 谐波次数数量 */
    HARMONIC_ORDER_CONFIG orders[MAX_HARMONICS];    /* 各次谐波配置 */
} HARMONIC_DETECT_CONFIG;

/* 单次谐波检测结果 */
typedef struct {
    HARMONIC_ORDER_CONFIG config;   /* 该次谐波配置 */
    LPF d_lpf;                      /* d 轴低通滤波器 */
    LPF q_lpf;                      /* q 轴低通滤波器 */
    DQ detected_dq;                 /* 检测值（dq） */
    DQ limited_dq;                  /* 限幅后的检测值（dq） */
    ABC phase_ref;                  /* 该次谐波参考（abc） */
    real_t power_square;            /* 该次谐波功率 */
} HARMONIC_COMPONENT;

/* 谐波检测器对象 */
typedef struct {
    HARMONIC_DETECT_CONFIG config;                      /* 配置 */
    HARMONIC_COMPONENT components[MAX_HARMONICS];       /* 各次谐波检测结果 */
    ABC phase_ref;                                      /* 总谐波补偿参考 */
    real_t power_square;                                /* 总谐波功率 */
} HARMONIC_DETECT;

/* 函数原型 */
void harmonic_detect_init(HARMONIC_DETECT *obj, const HARMONIC_DETECT_CONFIG *config);
void harmonic_detect_run(HARMONIC_DETECT *obj, ABC current_fdb, real_t base_angle);

#endif /* HARMONIC_DETECT_H_ */
