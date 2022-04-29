/*
 * Copyright (c) 2014-2015 Wind River Systems, Inc.
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System/hardware module for fsl_frdm_k64f platform
 *
 * This module provides routines to initialize and support board-level
 * hardware for the fsl_frdm_k64f platform.
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <soc.h>
// #include <drivers/uart.h>
// #include <fsl_common.h>
#include <clock_S32K1xx.h>
#include <arch/cpu.h>
#include <arch/arm/aarch32/cortex_m/cmsis.h>

static const clock_manager_user_config_t clockMan1_InitConfig0 = {
    .scgConfig =
    {
        .sircConfig =
        {
            .initialize = true,
            .enableInStop = true,                 /* Enable SIRC in stop mode */
            .enableInLowPower = true,             /* Enable SIRC in low power mode */
            .locked = false,                      /* unlocked */
            .range = SCG_SIRC_RANGE_HIGH,         /* Slow IRC high range clock (8 MHz) */
            .div1 = SCG_ASYNC_CLOCK_DIV_BY_1,     /* Slow IRC Clock Divider 1: divided by 1 */
            .div2 = SCG_ASYNC_CLOCK_DIV_BY_1,     /* Slow IRC Clock Divider 3: divided by 1 */
        },
        .fircConfig =
        {
            .initialize = true,
            .regulator = true,                    /* FIRC regulator is enabled */
            .locked = false,                      /* unlocked */
            .range = SCG_FIRC_RANGE_48M,           /*!< RANGE      */
            .div1 = SCG_ASYNC_CLOCK_DIV_BY_1,     /* Fast IRC Clock Divider 1: divided by 1 */
            .div2 = SCG_ASYNC_CLOCK_DIV_BY_1,     /* Fast IRC Clock Divider 3: divided by 1 */
        },
        .rtcConfig =
        {
            .initialize = false,
        },
        .soscConfig =
        {
            .initialize = true,
            .freq = 8000000U,                     /* System Oscillator frequency: 8000000Hz */
            .monitorMode = SCG_SOSC_MONITOR_DISABLE,/* Monitor disabled */
            .locked = false,                      /* Slow IRC disabled */
            .extRef = SCG_SOSC_REF_OSC,           /* Internal oscillator of OSC requested. */
            .gain = SCG_SOSC_GAIN_LOW,            /* Configure crystal oscillator for low-gain operation */
            .range = SCG_SOSC_RANGE_HIGH,         /* High frequency range selected for the crystal oscillator of 8 MHz to 40 MHz. */
            .div1 = SCG_ASYNC_CLOCK_DIV_BY_1,     /* System OSC Clock Divider 1: divided by 1 */
            .div2 = SCG_ASYNC_CLOCK_DIV_BY_1,     /* System OSC Clock Divider 3: divided by 1 */
        },
        .spllConfig =
        {
            .initialize = true,
            .monitorMode = SCG_SPLL_MONITOR_DISABLE,/* Monitor disabled */
            .locked = false,                      /* unlocked */
            .prediv = (uint8_t)SCG_SPLL_CLOCK_PREDIV_BY_1,/* Divided by 1 */
            .mult = (uint8_t)SCG_SPLL_CLOCK_MULTIPLY_BY_28,/* Multiply Factor is 28 */
            .src = 0U,
            .div1 = SCG_ASYNC_CLOCK_DIV_BY_2,     /* System PLL Clock Divider 1: divided by 2 */
            .div2 = SCG_ASYNC_CLOCK_DIV_BY_4,     /* System PLL Clock Divider 3: divided by 4 */
        },
        .clockOutConfig =
        {
            .initialize = true,
            .source = SCG_CLOCKOUT_SRC_FIRC,      /* Fast IRC. */
        },
        .clockModeConfig =
        {
            .initialize = true,
            .rccrConfig =
            {
                .src = SCG_SYSTEM_CLOCK_SRC_FIRC, /* Fast IRC */
                .divCore = SCG_SYSTEM_CLOCK_DIV_BY_1,/* Core Clock Divider: divided by 1 */
                .divBus = SCG_SYSTEM_CLOCK_DIV_BY_1,/* Bus Clock Divider: divided by 1 */
                .divSlow = SCG_SYSTEM_CLOCK_DIV_BY_2,/* Slow Clock Divider: divided by 2 */
            },
            .vccrConfig =
            {
                .src = SCG_SYSTEM_CLOCK_SRC_SIRC, /* Slow IRC */
                .divCore = SCG_SYSTEM_CLOCK_DIV_BY_2,/* Core Clock Divider: divided by 2 */
                .divBus = SCG_SYSTEM_CLOCK_DIV_BY_1,/* Bus Clock Divider: divided by 1 */
                .divSlow = SCG_SYSTEM_CLOCK_DIV_BY_4,/* Slow Clock Divider: divided by 4 */
            },
            .hccrConfig =
            {
                .src = SCG_SYSTEM_CLOCK_SRC_SYS_PLL,/* System PLL */
                .divCore = SCG_SYSTEM_CLOCK_DIV_BY_1,/* Core Clock Divider: divided by 1 */
                .divBus = SCG_SYSTEM_CLOCK_DIV_BY_2,/* Bus Clock Divider: divided by 2 */
                .divSlow = SCG_SYSTEM_CLOCK_DIV_BY_4,/* Slow Clock Divider: divided by 4 */
            },
        },
    },
    .pccConfig =
    {
        .peripheralClocks = peripheralClockConfig0, /*!< Peripheral clock control configurations  */
        .count = NUM_OF_PERIPHERAL_CLOCKS_0, /*!< Number of the peripheral clock control configurations  */
    },
    .simConfig =
    {
        .clockOutConfig =
        {
           .initialize = true, /*!< Initialize    */
            .enable = true,                       /* enabled */
            .source = SIM_CLKOUT_SEL_SYSTEM_SCG_CLKOUT,/* SCG CLKOUT clock select: SCG slow clock */
            .divider = SIM_CLKOUT_DIV_BY_1, /* Divided by 1 */
        },
        .lpoClockConfig =
        {
            .initialize = true, /*!< Initialize    */
            .enableLpo1k = true, /*!< LPO1KCLKEN    */
            .enableLpo32k = true, /*!< LPO32KCLKEN   */
            .sourceLpoClk = SIM_LPO_CLK_SEL_LPO_128K,/* 128 kHz LPO clock */
            .sourceRtcClk = SIM_RTCCLK_SEL_FIRCDIV1_CLK,/* FIRCDIV1 clock */
        },
        .platGateConfig =
        {
            .initialize = true, /*!< Initialize    */
            .enableEim = true, /*!< CGCEIM        */
            .enableErm = true, /*!< CGCERM        */
            .enableDma = true, /*!< CGCDMA        */
            .enableMpu = true, /*!< CGCMPU        */
            .enableMscm = true, /*!< CGCMSCM       */
        },
        .tclkConfig =
        {
            .initialize = false, /*!< Initialize    */
        },
        .traceClockConfig =
        {
            .initialize = true, /*!< Initialize    */
            .divEnable = true, /*!< TRACEDIVEN    */
            .source = CLOCK_TRACE_SRC_CORE_CLK, /*!< TRACECLK_SEL  */
            .divider = 0U, /*!< TRACEDIV      */
            .divFraction = false, /*!< TRACEFRAC     */
        },
    },
    .pmcConfig =
    {
        .lpoClockConfig =
        {
        .initialize = true,  /*!< Initialize    */
        .enable = true, /*!< Enable/disable LPO     */
        .trimValue = 0, /*!< Trimming value for LPO */
        },
    },
};

/**
 *
 * @brief Initialize the system clock
 *
 * This routine will configure the multipurpose clock generator (MCG) to
 * set up the system clock.
 * The MCG has nine possible modes, including Stop mode.  This routine assumes
 * that the current MCG mode is FLL Engaged Internal (FEI), as from reset.
 * It transitions through the FLL Bypassed External (FBE) and
 * PLL Bypassed External (PBE) modes to get to the desired
 * PLL Engaged External (PEE) mode and generate the maximum 120 MHz system
 * clock.
 *
 */
static ALWAYS_INLINE void clock_init(void)
{
	error = CLOCK_DRV_Init(&clockMan1_InitConfig0);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart0), okay)
	CLOCK_SetLpuartClock(LPUART0SRC_OSCERCLK);
#endif

}

/**
 *
 * @brief Perform basic hardware initialization
 *
 * Initialize the interrupt controller device drivers.
 * Also initialize the timer device driver, if required.
 *
 * @return 0
 */
static int s32k_init(const struct device *arg)
{
	ARG_UNUSED(arg);

	unsigned int oldLevel; /* old interrupt lock level */

	/* disable interrupts */
	oldLevel = irq_lock();

	/* Initialize PLL/system clock up to 180 MHz */
	clock_init();

	/*
	 * install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */
	NMI_INIT();

	/* restore interrupt state */
	irq_unlock(oldLevel);
	return 0;
}

SYS_INIT(s32k_init, PRE_KERNEL_1, 0);
