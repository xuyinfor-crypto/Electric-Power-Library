/*
 * 模块：算法对象初始化
 * 设计目的：初始化 CONTROL_ALGO 总对象，建立可运行的默认配置和内部控制器限幅。
 * 设计原理：先清零对象，再拷贝外部配置或使用默认配置，并对基础配置做合法性检查。
 * 设计要点：默认值只用于算法自洽启动，不代表任何工程侧定值；调用方可传入完整配置覆盖。
 * 输入：CONTROL_ALGO 对象指针，以及可选 CONFIG 配置指针。
 * 输出：清零后的对象、配置、故障位、控制器限幅和默认命令位。
 * 输出给谁用：应用层在周期调用算法前先调用本模块，后续 ISR 编排模块基于初始化状态运行。
 */
#ifndef INIT_H_
#define INIT_H_

#include "types.h"

void init(CONTROL_ALGO *obj, const CONFIG *config);

#endif
