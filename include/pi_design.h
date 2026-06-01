/*
 * 模块：PI 参数设计
 * 设计目的：根据被控对象近似模型、延时和目标相位裕度计算 PI 控制器参数。
 * 设计原理：把积分型或一阶型对象的目标穿越频率和零点位置求出，再换算 kp/ki。
 * 设计要点：本模块只做连续域参数计算，不自动写入运行中的 PID；模型参数由调用方明确提供。
 * 输入：PI_DESIGN_CONFIG 中的对象模型、延时、零点比例，以及 phase_margin_deg。
 * 输出：kp、ki、crossover_rad_s、zero_rad_s 和状态码。
 * 输出给谁用：参数整定工具或上层配置流程读取结果，再按需写入具体 PID 控制器。
 */
#ifndef PI_DESIGN_H_
#define PI_DESIGN_H_

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	PI_DESIGN_PLANT_INTEGRATOR = 0,
	PI_DESIGN_PLANT_FIRST_ORDER = 1
} PI_DESIGN_PLANT_TYPE;

typedef enum {
	PI_DESIGN_OK = 0,
	PI_DESIGN_NULL_OBJECT = 1,
	PI_DESIGN_BAD_CONFIG = 2,
	PI_DESIGN_BAD_PHASE_MARGIN = 3,
	PI_DESIGN_NO_CROSSOVER = 4,
	PI_DESIGN_ITERATION_FAILED = 5
} PI_DESIGN_STATUS;

typedef struct {
	PI_DESIGN_PLANT_TYPE type;
	real_t a;
	real_t b;
	real_t c;
} PI_DESIGN_PLANT;

typedef struct {
	PI_DESIGN_PLANT plant;
	real_t delay_s;
	real_t sample_period_s;
	real_t zero_ratio_n;
} PI_DESIGN_CONFIG;

typedef struct {
	real_t phase_margin_deg;
} PI_DESIGN_INPUT;

typedef struct {
	real_t phase_margin_rad;
	real_t target_phase_rad;
	real_t crossover_rad_s;
	real_t zero_rad_s;
} PI_DESIGN_INTER;

typedef struct {
	real_t kp;
	real_t ki_pid;
	real_t ki_cont;
	real_t crossover_rad_s;
	real_t zero_rad_s;
	PI_DESIGN_STATUS status;
} PI_DESIGN_OUTPUT;

typedef struct {
	PI_DESIGN_INPUT input;
	PI_DESIGN_INTER inter;
	PI_DESIGN_OUTPUT output;
	PI_DESIGN_CONFIG config;
} PI_DESIGN;

void pi_design_init(PI_DESIGN *obj, const PI_DESIGN_CONFIG *config);
PI_DESIGN_STATUS pi_design_calc(PI_DESIGN *obj);

static inline void set_pi_design_config(PI_DESIGN *obj, PI_DESIGN_CONFIG config)
{
	obj->config = config;
}

static inline void set_pi_design_phase_margin_deg(PI_DESIGN *obj, real_t phase_margin_deg)
{
	obj->input.phase_margin_deg = phase_margin_deg;
}

static inline const PI_DESIGN_OUTPUT *get_pi_design_output(const PI_DESIGN *obj)
{
	return &obj->output;
}

#ifdef __cplusplus
}
#endif

#endif
