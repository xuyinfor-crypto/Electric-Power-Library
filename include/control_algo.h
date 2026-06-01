/*
 * 模块：控制算法库总入口
 * 设计目的：向外部调用方提供统一的算法库访问入口，聚合各独立功能模块的公开接口。
 * 设计原理：总入口只包含模块头文件和轻量访问函数，不承载具体控制算法实现。
 *          API 分为三层：
 *            1. 输入层：set 函数写入采样量、参考值和命令字
 *            2. 计算层：由 isr 模块内部调度，外部不直接调用
 *            3. 输出层：get 函数读取控制输出、状态字和故障字
 * 设计要点：保持纯算法边界，不保存平台侧、设备侧或工程侧信息；
 *          具体功能仍由各模块各自维护。
 * 输入：CONTROL_ALGO 对象、命令字、三相量、直流量、角度和配置结构体。
 * 输出：状态字、故障字、控制输出结构体以及各子控制器对象指针。
 * 输出给谁用：应用层调度代码读取这些接口，再把算法输出交给外部执行层或上层控制流程使用。
 *
 * API 设计原则：
 * 1. 所有外部输入通过 set 函数写入，禁止直接操作 input 结构体
 * 2. 所有外部输出通过 get 函数读取，返回 const 指针防止意外修改
 * 3. 内部控制器提供非 const 指针用于参数整定
 * 4. 命令字/状态字/故障字通过位域访问，保持类型安全
 */
#ifndef CONTROL_ALGO_H_
#define CONTROL_ALGO_H_

#include "types.h"
#include "clarke.h"
#include "component.h"
#include "current_loop.h"
#include "dq_control.h"
#include "feedforward.h"
#include "fundamental_curr_detect.h"
#include "fundamental_curr_ctrl.h"
#include "harmonic_detect.h"
#include "harmonic_control.h"
#include "init.h"
#include "inverse_clarke.h"
#include "inverse_park.h"
#include "isr.h"
#include "lpf.h"
#include "math.h"
#include "mean.h"
#include "open_loop.h"
#include "park.h"
#include "pi_design.h"
#include "pid.h"
#include "pll.h"
#include "sequence.h"
#include "spwm.h"
#include "Vbus_volt_ctrl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 输入层：外部调用方通过这些函数写入采样量和命令
 * ======================================================================== */

/* 写入命令字（run_pll / run_open_loop / run_current_loop） */
static inline void set_cmd_word(CONTROL_ALGO *obj, unsigned int raw)
{
	obj->cmd_word.raw = raw;
}

/* 写入三相电网电压（归一化量） */
static inline void set_grid_voltage(CONTROL_ALGO *obj, ABC grid_voltage)
{
	obj->input.grid_voltage = grid_voltage;
}

/* 写入三相电流反馈（归一化量） */
static inline void set_current_feedback(CONTROL_ALGO *obj, ABC current_fdb)
{
	obj->input.current_fdb = current_fdb;
}

/* 写入三相电流参考（归一化量） */
static inline void set_current_reference(CONTROL_ALGO *obj, ABC current_ref)
{
	obj->input.current_ref = current_ref;
}

/* 写入直流电压 ref/fdb（归一化量） */
static inline void set_dc_voltage(CONTROL_ALGO *obj, REF_FDB dc_voltage)
{
	obj->input.dc_voltage = dc_voltage;
}

/* 写入外部角度（pu 单位，PLL 未就绪时使用） */
static inline void set_angle(CONTROL_ALGO *obj, real_t angle)
{
	obj->input.angle = angle;
}

/* ========================================================================
 * 输出层：外部调用方通过这些函数读取控制结果
 * ======================================================================== */

/* 读取状态字（pll_ready / open_loop / current_loop） */
static inline unsigned int get_st_word(const CONTROL_ALGO *obj)
{
	return obj->st_word.raw;
}

/* 读取故障字（null_object / bad_config） */
static inline unsigned int get_fault_word(const CONTROL_ALGO *obj)
{
	return obj->fault_word.raw;
}

/* 读取控制输出（三相量、alpha/beta、PLL 角度/频率、PI 输出） */
static inline const OUTPUT *get_output(const CONTROL_ALGO *obj)
{
	return &obj->output;
}

/* ========================================================================
 * 参数整定层：供初始化和调试流程读写内部控制器参数
 * ======================================================================== */

/* 读写配置（采样周期、基波频率、PI 参数、限幅等） */
static inline CONFIG *get_config(CONTROL_ALGO *obj)
{
	return &obj->config;
}

/* 读写直流母线电压控制器（含 PID 参数和输出限幅） */
static inline VBUS_VOLT_CTRL *get_vbus_ctrl(CONTROL_ALGO *obj)
{
	return &obj->inter.vbus_ctrl;
}

/* 读写基波电流控制器（含 dq 控制器参数和输出限幅） */
static inline FUNDAMENTAL_CURR_CTRL *get_fundamental_ctrl(CONTROL_ALGO *obj)
{
	return &obj->inter.fundamental_ctrl;
}

/* 读写谐波检测器（含谐波配置） */
static inline HARMONIC_DETECT *get_harmonic_detect(CONTROL_ALGO *obj)
{
	return &obj->inter.harmonic_detect;
}

/* 读写谐波控制器（含谐波控制参数） */
static inline HARMONIC_CONTROL *get_harmonic_control(CONTROL_ALGO *obj)
{
	return &obj->inter.harmonic_control;
}

/* 读写 SPWM 调制模块（含占空比配置） */
static inline SPWM *get_spwm(CONTROL_ALGO *obj)
{
	return &obj->inter.spwm;
}

/* 读取占空比输出（const 指针） */
static inline const ABC *get_duty_output(const CONTROL_ALGO *obj)
{
	return &obj->inter.spwm.duty_out;
}

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_ALGO_H_ */
