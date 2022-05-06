/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_S32K_SCG_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_S32K_SCG_H_

/* SCG system oscillator mode */
#define S32K_SCG_SOSC_MODE_EXT        0U
#define S32K_SCG_SOSC_MODE_LOW_POWER  4U
#define S32K_SCG_SOSC_MODE_HIGH_GAIN 12U

/* SCG clock controller clock names */
#define S32K_SCG_CORESYS_CLK          0U
#define S32K_SCG_BUS_CLK              1U
#define S32K_SCG_FLEXBUS_CLK          2U
#define S32K_SCG_FLASH_CLK            3U
#define S32K_SCG_SOSC_CLK             4U
#define S32K_SCG_SIRC_CLK             5U
#define S32K_SCG_FIRC_CLK             6U
#define S32K_SCG_SPLL_CLK             7U
#define S32K_SCG_SOSC_ASYNC_DIV1_CLK  8U
#define S32K_SCG_SOSC_ASYNC_DIV2_CLK  9U
#define S32K_SCG_SIRC_ASYNC_DIV1_CLK 10U
#define S32K_SCG_SIRC_ASYNC_DIV2_CLK 11U
#define S32K_SCG_FIRC_ASYNC_DIV1_CLK 12U
#define S32K_SCG_FIRC_ASYNC_DIV2_CLK 13U
#define S32K_SCG_SPLL_ASYNC_DIV1_CLK 14U
#define S32K_SCG_SPLL_ASYNC_DIV2_CLK 15U

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_S32K_SCG_H_ */
