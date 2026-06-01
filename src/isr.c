#include "isr.h"
#include "current_loop.h"
#include "math.h"
#include "open_loop.h"
#include "pll.h"

void run_isr(CONTROL_ALGO *obj)
{
	if(obj == NULL) {
		return;
	}

	if(!config_is_valid(&obj->config)) {
		obj->fault_word.bits.bad_config = 1;
		return;
	}

	if(obj->cmd_word.bits.run_pll) {
		run_grid_pll(obj);
	}

	if(obj->cmd_word.bits.run_open_loop) {
		run_open_loop(obj);
	}
	else if(obj->cmd_word.bits.run_current_loop) {
		run_current_loop(obj);
	}
}
