/*
 * 模块：电流环总控制
 * 设计目的：调度所有电流控制模块，生成最终的电压指令。
 * 设计原理：协调基波电流检测、直流母线电压控制、基波电流控制、谐波检测、谐波控制，
 *          将它们的输出叠加后生成总电压指令。
 * 设计要点：本模块只编排算法模块，不访问任何平台侧或工程侧信息；状态变化写回 CONTROL_ALGO 对象。
 *          SVG/APF 模式下，谐波参考值 = 检测值；其他模式下，谐波参考值由外部输入。
 * 输入：CONTROL_ALGO 中的命令状态、直流电压 ref/fdb、电流 ref/fdb、配置和内部控制器状态。
 * 输出：output.phase（总电压指令）、st_word 状态位。
 * 输出给谁用：ISR 编排模块和应用层总输出读取这些结果，继续交给外部执行层使用。
 */
#include "current_loop.h"
#include "fundamental_curr_detect.h"
#include "Vbus_volt_ctrl.h"
#include "fundamental_curr_ctrl.h"
#include "harmonic_detect.h"
#include "harmonic_control.h"
#include "feedforward.h"
#include "spwm.h"
#include "math.h"

void run_current_loop(CONTROL_ALGO *obj)
{
    real_t angle;
    FUNDAMENTAL_CURR_DETECT_INPUT fundamental_input;
    FUNDAMENTAL_CURR_DETECT_OUTPUT fundamental_output;
    FUNDAMENTAL_CURR_CTRL_INPUT ctrl_input;
    FUNDAMENTAL_CURR_CTRL_OUTPUT ctrl_output;
    int i;

    if(obj == NULL) {
        return;
    }

    /* 获取角度：优先使用 PLL 角度，未就绪时使用外部输入角度 */
    angle = obj->st_word.bits.pll_ready ? obj->output.pll_angle : obj->input.angle;

    /* 1. 基波电流检测 */
    fundamental_input.current_fdb = obj->input.current_fdb;
    fundamental_input.angle = angle;
    fundamental_curr_detect(&fundamental_input, &fundamental_output);

    /* 2. 直流母线电压控制 → 有功电流参考（d 轴） */
    Vbus_volt_ctrl_run(&obj->inter.vbus_ctrl,
                      obj->input.dc_voltage,
                      fundamental_output.detected_dq.d);

    /* 3. 指定无功电流参考（q 轴）← 外部输入 */
    /* 注：无功电流参考由外部通过 obj->input 或其他方式设定 */

    /* 4. 基波电流控制 */
    ctrl_input.ref.d = obj->inter.vbus_ctrl.active_ref;     /* d 轴参考 */
    ctrl_input.ref.q = obj->inter.vbus_ctrl.q_ref;          /* q 轴参考 */
    ctrl_input.fdb = fundamental_output.detected_dq;        /* 电流反馈 */
    ctrl_input.feedforward.d = 0.0f;
    ctrl_input.feedforward.q = 0.0f;

    fundamental_curr_ctrl_run(&obj->inter.fundamental_ctrl,
                             &ctrl_input,
                             &ctrl_output,
                             angle);

    /* 5. 谐波电流检测 */
    harmonic_detect_run(&obj->inter.harmonic_detect,
                       obj->input.current_fdb,
                       angle);

    /* 6. 设置谐波参考值 */
    if(obj->cmd_word.bits.run_current_loop) {
        /* SVG/APF 模式：参考值 = 检测值（补偿负载谐波） */
        for(i = 0; i < obj->inter.harmonic_detect.config.num_harmonics; i++) {
            obj->inter.harmonic_control.refs[i] = obj->inter.harmonic_detect.components[i].detected_dq;
        }
    }
    /* 其他模式：参考值由外部输入（已通过 refs 数组设定） */

    /* 7. 谐波电流控制 */
    harmonic_control_run(&obj->inter.harmonic_control,
                        &obj->inter.harmonic_detect,
                        angle);

    /* 8. 前馈生成 */
    calc_feedforward(obj, angle);

    /* 9. 叠加输出：基波电压指令 + 谐波电压指令 + 前馈 */
    obj->output.phase = add_abc(ctrl_output.voltage_abc,
                               obj->inter.harmonic_control.phase_ref);
    obj->output.phase = add_abc(obj->output.phase,
                               obj->inter.feedforward);

    /* 10. SPWM 调制：电压指令 → 占空比 */
    spwm_run(&obj->inter.spwm, obj->output.phase);

    /* 11. 保存占空比输出 */
    obj->output.duty = obj->inter.spwm.duty_out;

    /* 12. 更新状态位 */
    obj->st_word.bits.current_loop = 1;

    /* 13. 保存中间结果（用于调试） */
    obj->output.vdc_pi_out = obj->inter.vbus_ctrl.pid.out;
    obj->output.current_pi_out = ctrl_output.voltage_abc;
}
