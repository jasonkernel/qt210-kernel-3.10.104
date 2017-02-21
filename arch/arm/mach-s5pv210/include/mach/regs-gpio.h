/* linux/arch/arm/mach-s5pv210/include/mach/regs-gpio.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * S5PV210 - GPIO (including EINT) register definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_REGS_GPIO_H
#define __ASM_ARCH_REGS_GPIO_H __FILE__

#include <mach/map.h>

#define S5PV210_EINT30CON		(S5P_VA_GPIO + 0xE00)
#define S5P_EINT_CON(x)			(S5PV210_EINT30CON + ((x) * 0x4))

#define S5PV210_EINT30FLTCON0		(S5P_VA_GPIO + 0xE80)
#define S5P_EINT_FLTCON(x)		(S5PV210_EINT30FLTCON0 + ((x) * 0x4))

#define S5PV210_EINT30MASK		(S5P_VA_GPIO + 0xF00)
#define S5P_EINT_MASK(x)		(S5PV210_EINT30MASK + ((x) * 0x4))

#define S5PV210_EINT30PEND		(S5P_VA_GPIO + 0xF40)
#define S5P_EINT_PEND(x)		(S5PV210_EINT30PEND + ((x) * 0x4))

#define EINT_REG_NR(x)			(EINT_OFFSET(x) >> 3)

#define eint_irq_to_bit(irq)		(1 << (EINT_OFFSET(irq) & 0x7))

#define EINT_MODE		S3C_GPIO_SFN(0xf)

#define EINT_GPIO_0(x)		S5PV210_GPH0(x)
#define EINT_GPIO_1(x)		S5PV210_GPH1(x)
#define EINT_GPIO_2(x)		S5PV210_GPH2(x)
#define EINT_GPIO_3(x)		S5PV210_GPH3(x)

#define S5PV210_GPA0_BASE               (S5P_VA_GPIO + 0x000)
#define S5PV210_GPA1_BASE               (S5P_VA_GPIO + 0x020)
#define S5PV210_GPB_BASE                (S5P_VA_GPIO + 0x040)
#define S5PV210_GPC0_BASE               (S5P_VA_GPIO + 0x060)
#define S5PV210_GPC1_BASE               (S5P_VA_GPIO + 0x080)
#define S5PV210_GPD0_BASE               (S5P_VA_GPIO + 0x0A0)
#define S5PV210_GPD1_BASE               (S5P_VA_GPIO + 0x0C0)
#define S5PV210_GPE0_BASE               (S5P_VA_GPIO + 0x0E0)
#define S5PV210_GPE1_BASE               (S5P_VA_GPIO + 0x100)
#define S5PV210_GPF0_BASE               (S5P_VA_GPIO + 0x120)
#define S5PV210_GPF1_BASE               (S5P_VA_GPIO + 0x140)
#define S5PV210_GPF2_BASE               (S5P_VA_GPIO + 0x160)
#define S5PV210_GPF3_BASE               (S5P_VA_GPIO + 0x180)
#define S5PV210_GPG0_BASE               (S5P_VA_GPIO + 0x1A0)
#define S5PV210_GPG1_BASE               (S5P_VA_GPIO + 0x1C0)
#define S5PV210_GPG2_BASE               (S5P_VA_GPIO + 0x1E0)
#define S5PV210_GPG3_BASE               (S5P_VA_GPIO + 0x200)
#define S5PV210_GPH0_BASE               (S5P_VA_GPIO + 0xC00)
#define S5PV210_GPH1_BASE               (S5P_VA_GPIO + 0xC20)
#define S5PV210_GPH2_BASE               (S5P_VA_GPIO + 0xC40)
#define S5PV210_GPH3_BASE               (S5P_VA_GPIO + 0xC60)
#define S5PV210_GPI_BASE                (S5P_VA_GPIO + 0x220)


#endif /* __ASM_ARCH_REGS_GPIO_H */
