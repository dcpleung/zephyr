/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_INPUTMUX_TRIGGER_PORTS_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_INPUTMUX_TRIGGER_PORTS_H_

#define LPC55S69_DMA0_OTRIG_BASE 0x16000000
#define LPC55S69_DMA0_ITRIG_BASE 0x0E00000F
#define LPC55S69_DMA1_OTRIG_BASE 0x24000002
#define LPC55S69_DMA1_ITRIG_BASE 0x20000008

#define RT595_DMA0_OTRIG_BASE 0x30000000
#define RT595_DMA0_ITRIG_BASE 0x2000000E
#define RT595_DMA1_OTRIG_BASE 0x50000000
#define RT595_DMA1_ITRIG_BASE 0x4000000E

#define LPC55S36_DMA0_OTRIG_BASE 0x16000000
#define LPC55S36_DMA0_ITRIG_BASE 0x0E000011
#define LPC55S36_DMA1_OTRIG_BASE 0x24000002
#define LPC55S36_DMA1_ITRIG_BASE 0x20000008

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_INPUTMUX_TRIGGER_PORTS_H_ */
