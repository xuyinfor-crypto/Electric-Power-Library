/*
 * 模块：基波电流控制
 * 设计目的：在 dq 坐标系下对基波电流进行闭环控制，生成基波电压指令。
 * 设计原理：d 轴控制有功电流，q 轴控制无功电流；PI 控制后反变换到 abc。
 * 设计要点：输入使用 dq 格式的参考和反馈；输出为 abc 格式的电压指令。
 * 输入：有功电流参考（d 轴）、无功电流参考（q 轴）、基波电流反馈（dq）。
 * 输出：基波电压指令（abc）。
 * 输出给谁用：current_loop 读取，叠加谐波电压指令和前馈后生成总电压指令。
 */
#ifndef FUNDAMENTAL_CURR_CTRL_H_
#define FUNDAMENTAL_CURR_CTRL_H_

#include "types.h"

/* 基波电流控制器输入 */
typedef struct {
    DQ ref;               /* 电流参考（dq） */
    DQ fdb;               /* 电流反馈（dq） */
    DQ feedforward;       /* 前馈（dq，可选） */
} FUNDAMENTAL_CURR_CTRL_INPUT;

/* 基波电流控制器输出 */
typedef struct {
    DQ voltage_dq;        /* 基波电压指令（dq） */
    ABC voltage_abc;      /* 基波电压指令（abc） */
} FUNDAMENTAL_CURR_CTRL_OUTPUT;

/* 基波电流控制器对象 */
typedef struct {
    DQ_CONTROLLER dq_ctrl;    /* dq 轴双通道控制器 */
    COMPONENT_CONFIG config;  /* 反变换配置（正序基波） */
    real_t output_limit;      /* 输出限幅 */
} FUNDAMENTAL_CURR_CTRL;

/* 函数原型 */
void fundamental_curr_ctrl_init(FUNDAMENTAL_CURR_CTRL *obj);
void fundamental_curr_ctrl_run(
    FUNDAMENTAL_CURR_CTRL *obj,
    const FUNDAMENTAL_CURR_CTRL_INPUT *input,
    FUNDAMENTAL_CURR_CTRL_OUTPUT *output,
    real_t angle
);

#endif /* FUNDAMENTAL_CURR_CTRL_H_ */
