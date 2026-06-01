#include <string.h>
#include "init.h"
#include "math.h"
#include "Vbus_volt_ctrl.h"
#include "fundamental_curr_ctrl.h"
#include "harmonic_detect.h"
#include "harmonic_control.h"
#include "spwm.h"

void init(CONTROL_ALGO *obj, const CONFIG *config)
{
    CONFIG default_config;

    if(obj == NULL) {
        return;
    }

    memset(obj, 0, sizeof(*obj));
    default_config.sample_period_s = 0.0001f;
    default_config.base_frequency_hz = 50.0f;
    default_config.pll_kp = 1.0f;
    default_config.pll_ki = 0.0f;
    default_config.pll_out_max = 5.0f;
    default_config.pll_out_min = -5.0f;
    default_config.feedforward = 0.0f;
    default_config.open_loop_dq.d = 0.0f;
    default_config.open_loop_dq.q = 0.0f;
    default_config.duty_limit = 0.99f;

    obj->config = config != NULL ? *config : default_config;
    if(!config_is_valid(&obj->config)) {
        obj->fault_word.bits.bad_config = 1;
    }

    /* 初始化直流母线电压控制器 */
    Vbus_volt_ctrl_init(&obj->inter.vbus_ctrl);
    obj->inter.vbus_ctrl.output_limit = obj->config.duty_limit;
    obj->inter.vbus_ctrl.pid.out_max = obj->config.duty_limit;
    obj->inter.vbus_ctrl.pid.out_min = -obj->config.duty_limit;

    /* 初始化基波电流控制器 */
    fundamental_curr_ctrl_init(&obj->inter.fundamental_ctrl);
    obj->inter.fundamental_ctrl.output_limit = obj->config.duty_limit;
    obj->inter.fundamental_ctrl.dq_ctrl.d_pid.out_max = obj->config.duty_limit;
    obj->inter.fundamental_ctrl.dq_ctrl.d_pid.out_min = -obj->config.duty_limit;
    obj->inter.fundamental_ctrl.dq_ctrl.q_pid.out_max = obj->config.duty_limit;
    obj->inter.fundamental_ctrl.dq_ctrl.q_pid.out_min = -obj->config.duty_limit;

    /* 初始化谐波检测器（默认配置，可后续覆盖） */
    HARMONIC_DETECT_CONFIG default_harmonic_detect_config = {
        .num_harmonics = 0,
    };
    harmonic_detect_init(&obj->inter.harmonic_detect, &default_harmonic_detect_config);

    /* 初始化谐波控制器（默认配置，可后续覆盖） */
    HARMONIC_CONTROL_CONFIG default_harmonic_control_config = {
        .num_harmonics = 0,
    };
    harmonic_control_init(&obj->inter.harmonic_control, &default_harmonic_control_config);

    /* 初始化 SPWM 调制模块 */
    SPWM_CONFIG default_spwm_config = {
        .duty_max = 0.99f,
        .duty_min = 0.01f,
        .dead_time = 0.0f,
        .min_pulse_width = 0.01f,
        .amplitude_limit = 0.99f,
    };
    spwm_init(&obj->inter.spwm, &default_spwm_config);

    obj->cmd_word.bits.run_pll = 1;
    obj->cmd_word.bits.run_current_loop = 1;
}
