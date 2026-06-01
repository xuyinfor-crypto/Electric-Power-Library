/*
 * 模块：基波电流检测
 * 设计目的：从电流反馈中提取基波 dq 分量，用于基波电流控制。
 * 设计原理： Clarke → Park（基波旋转坐标系）→ 输出 dq 检测值。
 * 设计要点：只负责检测，不负责计算参考值；参考值由 current_loop 指定。
 * 输入：电流反馈（abc）、PLL 角度（基波）。
 * 输出：基波电流检测值（dq）。
 * 输出给谁用：current_loop 读取，作为 fundamental_curr_ctrl 的反馈。
 */
#ifndef FUNDAMENTAL_CURR_DETECT_H_
#define FUNDAMENTAL_CURR_DETECT_H_

#include "types.h"

/* 输入结构 */
typedef struct {
    ABC current_fdb;      /* 电流反馈（abc） */
    real_t angle;         /* PLL 角度（基波） */
} FUNDAMENTAL_CURR_DETECT_INPUT;

/* 输出结构 */
typedef struct {
    DQ detected_dq;      /* 基波电流检测值（dq） */
} FUNDAMENTAL_CURR_DETECT_OUTPUT;

/* 函数原型 */
void fundamental_curr_detect(
    const FUNDAMENTAL_CURR_DETECT_INPUT *input,
    FUNDAMENTAL_CURR_DETECT_OUTPUT *output
);

#endif /* FUNDAMENTAL_CURR_DETECT_H_ */
