/*
 * 模块：控制算法库公共类型
 * 设计目的：集中定义算法库各模块共享的数据结构，保证 abc、alpha/beta、dq 等坐标量表达一致。
 * 设计原理：用结构体描述单一语义输入输出，减少长形参和散乱标量在模块之间传递。
 * 设计要点：本文件只放类型和状态定义，不放算法实现；类型必须与平台侧、设备侧和工程侧信息解耦。
 * 输入：外部调用方写入的配置、采样归一化量、参考值、反馈值和命令状态。
 * 输出：各模块更新后的中间量、控制量、状态量和故障量。
 * 输出给谁用：所有算法模块共享这些类型，应用层也通过这些结构体读写算法库边界数据。
 */
#ifndef TYPES_H_
#define TYPES_H_

/* 最大支持的谐波次数数量 */
#define MAX_HARMONICS  16

typedef float real_t;

typedef struct {
	real_t a;
	real_t b;
	real_t c;
} ABC;

typedef struct {
	real_t alpha;
	real_t beta;
} ALPHA_BETA;

typedef struct {
	real_t d;
	real_t q;
} DQ;

typedef struct {
	real_t ref;
	real_t fdb;
} REF_FDB;

typedef enum {
	SEQUENCE_POSITIVE = 1,
	SEQUENCE_NEGATIVE = -1
} SEQUENCE;

typedef union {
	unsigned int raw;
	struct {
		unsigned int pll_ready:1;
		unsigned int open_loop:1;
		unsigned int current_loop:1;
		unsigned int reserved:13;
	} bits;
} ST_WORD;

typedef union {
	unsigned int raw;
	struct {
		unsigned int run_pll:1;
		unsigned int run_open_loop:1;
		unsigned int run_current_loop:1;
		unsigned int reserved:13;
	} bits;
} CMD_WORD;

typedef union {
	unsigned int raw;
	struct {
		unsigned int null_object:1;
		unsigned int bad_config:1;
		unsigned int reserved:14;
	} bits;
} FAULT_WORD;

typedef struct {
	real_t sample_period_s;
	real_t base_frequency_hz;
	real_t pll_kp;
	real_t pll_ki;
	real_t pll_out_max;
	real_t pll_out_min;
	real_t feedforward;
	DQ open_loop_dq;
	real_t duty_limit;
} CONFIG;

typedef struct {
	ABC grid_voltage;
	ABC current_ref;
	ABC current_fdb;
	REF_FDB dc_voltage;
	real_t angle;
} INPUT;

typedef struct {
	ABC phase;
	ABC duty;                /* 三相占空比输出 */
	ALPHA_BETA alpha_beta;
	real_t pll_angle;
	real_t pll_freq_hz;
	real_t vdc_pi_out;
	ABC current_pi_out;
} OUTPUT;

typedef struct {
	real_t ref;
	real_t fdb;
	real_t kp;
	real_t ki;
	real_t kc;
	real_t kd;
	real_t out_max;
	real_t out_min;
	real_t e;
	real_t up;
	real_t ui;
	real_t ud;
	real_t uprsat;
	real_t out;
	real_t saterr;
	real_t up1;
} PID;

typedef struct {
	real_t v;
	real_t i;
	real_t vo;
	real_t io;
	real_t vacc;
	real_t iacc;
	int cnt;
	int sample_count;
} MEAN;

/*
 * 单极点 IIR 低通滤波器
 * 设计原理：y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 * 内存占用：2 个 float（alpha + y_prev），无缓冲区
 * alpha = 1.0 时直通（不过滤），alpha = 0 时保持上一次值
 * alpha 可由调用方直接设置，或通过 lpf_init 从截止频率自动计算
 */
typedef struct {
	real_t alpha;    /* 滤波系数 (0, 1]，1.0 = 直通 */
	real_t y_prev;   /* 上一次滤波输出 */
	real_t average;  /* 最新滤波输出（只读） */
} LPF;

typedef struct {
	int order;
	SEQUENCE sequence;
	real_t gain;
	real_t phase_offset;
	real_t dq_limit;
	real_t power_scale;
} COMPONENT_CONFIG;

typedef struct {
	ABC phase;
	real_t angle;
	int enabled;
} COMPONENT_INPUT;

typedef struct {
	COMPONENT_CONFIG config;
	LPF d_lpf;
	LPF q_lpf;
	DQ detected_dq;
	DQ limited_dq;
	ABC phase_ref;
	real_t power_square;
} COMPONENT;

typedef struct {
	COMPONENT h5;
	COMPONENT h7;
	COMPONENT h11;
	COMPONENT h13;
	ABC phase_ref;
	real_t power_square;
} HARMONIC_EXTRACTOR;

typedef struct {
	COMPONENT positive;
	COMPONENT negative;
	ABC positive_ref;
	ABC negative_ref;
	ABC phase_ref;
	real_t power_square;
} SEQUENCE_EXTRACTOR;

/* 直流母线电压控制器对象 */
typedef struct {
	PID pid;                /* PID 控制器 */
	real_t q_ref;           /* q 轴参考（无功） */
	real_t q_feedforward;   /* q 轴前馈 */
	real_t output_limit;    /* 输出限幅 */
	DQ dq_ref;              /* dq 参考 */
	real_t active_ref;      /* 有功电流参考（d 轴） */
	ABC phase_ref;          /* 三相参考（abc） */
} VBUS_VOLT_CTRL;

typedef struct {
	ABC ref;
	ABC fdb;
	ABC feedforward;
} CURRENT_CONTROL_INPUT;

typedef struct {
	DQ ref;
	DQ fdb;
	DQ feedforward;
} DQ_CONTROL_INPUT;

typedef struct {
	PID d_pid;
	PID q_pid;
	real_t output_limit;
	DQ pi_out;
	DQ dq_out;
} DQ_CONTROLLER;

typedef struct {
	const HARMONIC_EXTRACTOR *extractor;
	real_t angle;
	int enabled;
	DQ h5_ref;
	DQ h7_ref;
	DQ h11_ref;
	DQ h13_ref;
} HARMONIC_CONTROL_INPUT;

typedef struct {
	DQ_CONTROLLER h5;
	DQ_CONTROLLER h7;
	DQ_CONTROLLER h11;
	DQ_CONTROLLER h13;
	ABC h5_ref;
	ABC h7_ref;
	ABC h11_ref;
	ABC h13_ref;
	ABC phase_ref;
} HARMONIC_CONTROLLER;

typedef struct {
	PID a_pid;
	PID b_pid;
	PID c_pid;
	real_t output_limit;
	ABC pi_out;
	ABC phase_out;
} CURRENT_CONTROLLER;

typedef struct {
	COMPONENT_CONFIG config;
	DQ dq_ref;
	real_t output_limit;
	ABC phase_ref;
} SEQUENCE_CONTROLLER;

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

/* 谐波控制器配置 */
typedef struct {
	int num_harmonics;                              /* 谐波次数数量 */
	real_t kp[MAX_HARMONICS];                       /* 各次谐波 Kp */
	real_t ki[MAX_HARMONICS];                       /* 各次谐波 Ki */
	real_t output_limit[MAX_HARMONICS];             /* 各次谐波输出限幅 */
} HARMONIC_CONTROL_CONFIG;

/* 谐波控制器对象 */
typedef struct {
	HARMONIC_CONTROL_CONFIG config;                 /* 配置 */
	DQ_CONTROLLER controllers[MAX_HARMONICS];       /* 各次谐波控制器 */
	DQ refs[MAX_HARMONICS];                         /* 各次谐波参考（dq） */
	ABC h_refs[MAX_HARMONICS];                      /* 各次谐波参考（abc） */
	ABC phase_ref;                                  /* 总谐波补偿参考 */
} HARMONIC_CONTROL;

/* 基波电流控制器对象 */
typedef struct {
	DQ_CONTROLLER dq_ctrl;    /* dq 轴双通道控制器 */
	COMPONENT_CONFIG config;  /* 反变换配置（正序基波） */
	real_t output_limit;      /* 输出限幅 */
} FUNDAMENTAL_CURR_CTRL;

/* SPWM 配置 */
typedef struct {
	real_t duty_max;          /* 最大占空比 */
	real_t duty_min;          /* 最小占空比 */
	real_t dead_time;         /* 死区时间（归一化） */
	real_t min_pulse_width;   /* 最小脉宽（归一化） */
	real_t amplitude_limit;   /* 电压指令幅值限幅 */
} SPWM_CONFIG;

/* SPWM 对象 */
typedef struct {
	SPWM_CONFIG config;       /* 配置 */
	ABC duty_out;             /* 三相占空比输出 */
} SPWM;

typedef struct {
	ALPHA_BETA alpha_beta;
	DQ dq;
	real_t zero;
	real_t ramp_angle;
	real_t pll_ui;
	real_t pll_last_error;
	ABC feedforward;
	VBUS_VOLT_CTRL vbus_ctrl;                    /* 直流母线电压控制 */
	FUNDAMENTAL_CURR_CTRL fundamental_ctrl;      /* 基波电流控制 */
	HARMONIC_DETECT harmonic_detect;             /* 谐波电流检测 */
	HARMONIC_CONTROL harmonic_control;           /* 谐波电流控制 */
	SPWM spwm;                                   /* SPWM 调制 */
} INTER;

typedef struct {
	INPUT input;
	INTER inter;
	OUTPUT output;
	CONFIG config;
	ST_WORD st_word;
	CMD_WORD cmd_word;
	FAULT_WORD fault_word;
} CONTROL_ALGO;

/* 辅助函数：零向量 */
static inline ABC zero_abc(void) {
	ABC zero;
	zero.a = 0.0f;
	zero.b = 0.0f;
	zero.c = 0.0f;
	return zero;
}

/* 辅助函数：向量加法 */
static inline ABC add_abc(ABC a, ABC b) {
	ABC result;
	result.a = a.a + b.a;
	result.b = a.b + b.b;
	result.c = a.c + b.c;
	return result;
}

/* 辅助函数：DQ 限幅 */
static inline DQ sat_dq(DQ dq, real_t limit) {
	DQ result;
	if(dq.d > limit) result.d = limit;
	else if(dq.d < -limit) result.d = -limit;
	else result.d = dq.d;

	if(dq.q > limit) result.q = limit;
	else if(dq.q < -limit) result.q = -limit;
	else result.q = dq.q;

	return result;
}

/* 辅助函数：限幅 */
static inline real_t sat(real_t x, real_t max, real_t min) {
	if(x > max) return max;
	if(x < min) return min;
	return x;
}

#endif /* TYPES_H_ */
