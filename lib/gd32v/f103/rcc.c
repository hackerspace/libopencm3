/** @defgroup rcc_file RCC peripheral API

@ingroup peripheral_apis

@brief <b>libopencm3 GD32VF103 Reset and Clock Control</b>

@version 1.0.0

@author @htmlonly &copy; @endhtmlonly 2009
Federico Ruiz-Ugalde \<memeruiz at gmail dot com\>
@author @htmlonly &copy; @endhtmlonly 2009 Uwe Hermann <uwe@hermann-uwe.de>
@author @htmlonly &copy; @endhtmlonly 2010 Thomas Otto <tommi@viadmin.org>
@author @htmlonly &copy; @endhtmlonly 2020 Lubomir Rintel <lkundrak@v3.sk>

@date 3 July 2020

This library supports the Reset and Clock Control System in the GD32VF103
series of ARM Cortex Microcontrollers by GigaDevice.

Clock settings and resets for many peripherals are given here rather than in
the corresponding peripheral library.

The library also provides a number of common configurations for the processor
system clock. Not all possible configurations are included.

LGPL License Terms @ref lgpl_license
 */
/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Federico Ruiz-Ugalde <memeruiz at gmail dot com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2010 Thomas Otto <tommi@viadmin.org>
 * Copyright (C) 2019 Icenowy Zheng <icenowy@aosc.io>
 * Copyright (C) 2020 Lubomir Rintel <lkundrak@v3.sk>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/**@{*/

#include <libopencm3/cm3/assert.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>

/** Set the default clock frequencies */
uint32_t rcc_apb1_frequency = 8000000;
uint32_t rcc_apb2_frequency = 8000000;
uint32_t rcc_ahb_frequency = 8000000;

const struct rcc_clock_scale rcc_hsi_configs[] = {
	[RCC_CLOCK_HSI_48MHZ] = {
		.pllsrc = RCC_CFGR_PLLSRC_HSI_CLK_DIV2,
		.pllmul = RCC_CFGR_PLLMUL_PLL_CLK_MUL12,

		.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
		.ppre1 = RCC_CFGR_PPRE1_HCLK_DIV2,
		.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,

		.adcpre = RCC_CFGR_ADCPRE_PCLK2_DIV4,
		.usbpre = RCC_CFGR_USBPRE_PLL_CLK_NODIV, /* Good Luck. */
		/* Really, use an external crystal. */

		.ahb_frequency  = 48000000,
		.apb1_frequency = 24000000,
		.apb2_frequency = 48000000,
	},
	[RCC_CLOCK_HSI_72MHZ] = {
		.pllsrc = RCC_CFGR_PLLSRC_HSI_CLK_DIV2,
		.pllmul = RCC_CFGR_PLLMUL_PLL_CLK_MUL18,

		.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
		.ppre1 = RCC_CFGR_PPRE1_HCLK_DIV2,
		.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,

		.adcpre = RCC_CFGR_ADCPRE_PCLK2_DIV6,
		.usbpre = RCC_CFGR_USBPRE_PLL_CLK_DIV1_5,

		.ahb_frequency  = 72000000,
		.apb1_frequency = 36000000,
		.apb2_frequency = 72000000,
	},
	[RCC_CLOCK_HSI_108MHZ] = {
		.pllsrc = RCC_CFGR_PLLSRC_HSI_CLK_DIV2,
		.pllmul = RCC_CFGR_PLLMUL_PLL_CLK_MUL27,

		.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
		.ppre1 = RCC_CFGR_PPRE1_HCLK_DIV2,
		.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,

		.adcpre = RCC_CFGR_ADCPRE_PCLK2_DIV8,
		/* USB is not going to work. */

		.ahb_frequency  = 108000000,
		.apb1_frequency = 54000000,
		.apb2_frequency = 108000000,
	},
};

const struct rcc_clock_scale rcc_hse8_configs[] = {
	[RCC_CLOCK_HSE8_24MHZ] = {
		.pll_prediv1src = RCC_CFGR2_PREDIV1SRC_HSE_CLK,
		.pll_prediv1 = RCC_CFGR2_PREDIV_DIV2,

		.pllsrc = RCC_CFGR_PLLSRC_HSE_CLK,
		.pllmul = RCC_CFGR_PLLMUL_PLL_CLK_MUL6,

		.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
		.ppre1 = RCC_CFGR_PPRE1_HCLK_DIV2,
		.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,

		.adcpre = RCC_CFGR_ADCPRE_PCLK2_DIV2,
		/* USB is not going to work. */

		.ahb_frequency  = 24000000,
		.apb1_frequency = 12000000,
		.apb2_frequency = 24000000,
	},
	[RCC_CLOCK_HSE8_36MHZ] = {
		.pll_prediv1src = RCC_CFGR2_PREDIV1SRC_HSE_CLK,
		.pll_prediv1 = RCC_CFGR2_PREDIV_DIV2,

		.pllsrc = RCC_CFGR_PLLSRC_HSE_CLK,
		.pllmul = RCC_CFGR_PLLMUL_PLL_CLK_MUL9,

		.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
		.ppre1 = RCC_CFGR_PPRE1_HCLK_DIV2,
		.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,

		.adcpre = RCC_CFGR_ADCPRE_PCLK2_DIV4,
		/* USB is not going to work. */

		.ahb_frequency  = 36000000,
		.apb1_frequency = 18000000,
		.apb2_frequency = 36000000,
	},
	[RCC_CLOCK_HSE8_48MHZ] = {
		.pll_prediv1src = RCC_CFGR2_PREDIV1SRC_HSE_CLK,
		.pll_prediv1 = RCC_CFGR2_PREDIV_DIV2,

		.pllsrc = RCC_CFGR_PLLSRC_HSE_CLK,
		.pllmul = RCC_CFGR_PLLMUL_PLL_CLK_MUL12,

		.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
		.ppre1 = RCC_CFGR_PPRE1_HCLK_DIV2,
		.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,

		.adcpre = RCC_CFGR_ADCPRE_PCLK2_DIV4,
		.usbpre = RCC_CFGR_USBPRE_PLL_CLK_NODIV,

		.ahb_frequency  = 48000000,
		.apb1_frequency = 24000000,
		.apb2_frequency = 48000000,
	},
	[RCC_CLOCK_HSE8_56MHZ] = {
		.pll_prediv1src = RCC_CFGR2_PREDIV1SRC_HSE_CLK,
		.pll_prediv1 = RCC_CFGR2_PREDIV_DIV2,

		.pllsrc = RCC_CFGR_PLLSRC_HSE_CLK,
		.pllmul = RCC_CFGR_PLLMUL_PLL_CLK_MUL14,

		.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
		.ppre1 = RCC_CFGR_PPRE1_HCLK_DIV2,
		.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,

		.adcpre = RCC_CFGR_ADCPRE_PCLK2_DIV4,
		/* USB is not going to work. */

		.ahb_frequency  = 56000000,
		.apb1_frequency = 28000000,
		.apb2_frequency = 56000000,
	},
	[RCC_CLOCK_HSE8_72MHZ] = {
		.pll_prediv1src = RCC_CFGR2_PREDIV1SRC_HSE_CLK,
		.pll_prediv1 = RCC_CFGR2_PREDIV_DIV2,

		.pllsrc = RCC_CFGR_PLLSRC_HSE_CLK,
		.pllmul = RCC_CFGR_PLLMUL_PLL_CLK_MUL18,

		.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
		.ppre1 = RCC_CFGR_PPRE1_HCLK_DIV2,
		.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,

		.adcpre = RCC_CFGR_ADCPRE_PCLK2_DIV6,
		.usbpre = RCC_CFGR_USBPRE_PLL_CLK_DIV1_5,

		.ahb_frequency  = 72000000,
		.apb1_frequency = 36000000,
		.apb2_frequency = 72000000,
	},
	[RCC_CLOCK_HSE8_96MHZ] = {
		.pll_prediv1src = RCC_CFGR2_PREDIV1SRC_HSE_CLK,
		.pll_prediv1 = RCC_CFGR2_PREDIV_DIV2,

		.pllsrc = RCC_CFGR_PLLSRC_HSE_CLK,
		.pllmul = RCC_CFGR_PLLMUL_PLL_CLK_MUL24,

		.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
		.ppre1 = RCC_CFGR_PPRE1_HCLK_DIV2,
		.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,

		.adcpre = RCC_CFGR_ADCPRE_PCLK2_DIV8,
		.usbpre = RCC_CFGR_USBPRE_PLL_CLK_DIV2,

		.ahb_frequency  = 96000000,
		.apb1_frequency = 48000000,
		.apb2_frequency = 96000000,
	},
	[RCC_CLOCK_HSE8_108MHZ] = {
		.pll_prediv1src = RCC_CFGR2_PREDIV1SRC_HSE_CLK,
		.pll_prediv1 = RCC_CFGR2_PREDIV_DIV2,

		.pllsrc = RCC_CFGR_PLLSRC_HSE_CLK,
		.pllmul = RCC_CFGR_PLLMUL_PLL_CLK_MUL27,

		.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
		.ppre1 = RCC_CFGR_PPRE1_HCLK_DIV2,
		.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,

		.adcpre = RCC_CFGR_ADCPRE_PCLK2_DIV8,
		/* USB is not going to work. */

		.ahb_frequency  = 108000000,
		.apb1_frequency = 52000000,
		.apb2_frequency = 108000000,
	},
};

const struct rcc_clock_scale rcc_hse25_configs[] = {
	[RCC_CLOCK_HSE25_24MHZ] = {
		.pll_prediv2 = RCC_CFGR2_PREDIV_DIV5,
		.pll_mul2 = RCC_CFGR2_PLL2MUL_PLL2_CLK_MUL8,

		.pll_prediv1src = RCC_CFGR2_PREDIV1SRC_PLL2_CLK,
		.pll_prediv1 = RCC_CFGR2_PREDIV_DIV2,

		.pllsrc = RCC_CFGR_PLLSRC_HSE_CLK,
		.pllmul = RCC_CFGR_PLLMUL_PLL_CLK_MUL6,

		.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
		.ppre1 = RCC_CFGR_PPRE1_HCLK_DIV2,
		.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,

		.adcpre = RCC_CFGR_ADCPRE_PCLK2_DIV2,
		/* USB is not going to work. */

		.ahb_frequency  = 24000000,
		.apb1_frequency = 12000000,
		.apb2_frequency = 24000000,
	},
	[RCC_CLOCK_HSE25_36MHZ] = {
		.pll_prediv2 = RCC_CFGR2_PREDIV_DIV5,
		.pll_mul2 = RCC_CFGR2_PLL2MUL_PLL2_CLK_MUL8,

		.pll_prediv1src = RCC_CFGR2_PREDIV1SRC_PLL2_CLK,
		.pll_prediv1 = RCC_CFGR2_PREDIV_DIV2,

		.pllsrc = RCC_CFGR_PLLSRC_HSE_CLK,
		.pllmul = RCC_CFGR_PLLMUL_PLL_CLK_MUL9,

		.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
		.ppre1 = RCC_CFGR_PPRE1_HCLK_DIV2,
		.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,

		.adcpre = RCC_CFGR_ADCPRE_PCLK2_DIV4,
		/* USB is not going to work. */

		.ahb_frequency  = 36000000,
		.apb1_frequency = 18000000,
		.apb2_frequency = 36000000,
	},
	[RCC_CLOCK_HSE25_48MHZ] = {
		.pll_prediv2 = RCC_CFGR2_PREDIV_DIV5,
		.pll_mul2 = RCC_CFGR2_PLL2MUL_PLL2_CLK_MUL8,

		.pll_prediv1src = RCC_CFGR2_PREDIV1SRC_PLL2_CLK,
		.pll_prediv1 = RCC_CFGR2_PREDIV_DIV2,

		.pllsrc = RCC_CFGR_PLLSRC_HSE_CLK,
		.pllmul = RCC_CFGR_PLLMUL_PLL_CLK_MUL12,

		.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
		.ppre1 = RCC_CFGR_PPRE1_HCLK_DIV2,
		.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,

		.adcpre = RCC_CFGR_ADCPRE_PCLK2_DIV4,
		.usbpre = RCC_CFGR_USBPRE_PLL_CLK_NODIV,

		.ahb_frequency  = 48000000,
		.apb1_frequency = 24000000,
		.apb2_frequency = 48000000,
	},
	[RCC_CLOCK_HSE25_56MHZ] = {
		.pll_prediv2 = RCC_CFGR2_PREDIV_DIV5,
		.pll_mul2 = RCC_CFGR2_PLL2MUL_PLL2_CLK_MUL8,

		.pll_prediv1src = RCC_CFGR2_PREDIV1SRC_PLL2_CLK,
		.pll_prediv1 = RCC_CFGR2_PREDIV_DIV2,

		.pllsrc = RCC_CFGR_PLLSRC_HSE_CLK,
		.pllmul = RCC_CFGR_PLLMUL_PLL_CLK_MUL14,

		.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
		.ppre1 = RCC_CFGR_PPRE1_HCLK_DIV2,
		.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,

		.adcpre = RCC_CFGR_ADCPRE_PCLK2_DIV4,
		/* USB is not going to work. */

		.ahb_frequency  = 56000000,
		.apb1_frequency = 28000000,
		.apb2_frequency = 56000000,
	},
	[RCC_CLOCK_HSE25_72MHZ] = {
		.pll_prediv2 = RCC_CFGR2_PREDIV_DIV5,
		.pll_mul2 = RCC_CFGR2_PLL2MUL_PLL2_CLK_MUL8,

		.pll_prediv1src = RCC_CFGR2_PREDIV1SRC_PLL2_CLK,
		.pll_prediv1 = RCC_CFGR2_PREDIV_DIV2,

		.pllsrc = RCC_CFGR_PLLSRC_HSE_CLK,
		.pllmul = RCC_CFGR_PLLMUL_PLL_CLK_MUL18,

		.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
		.ppre1 = RCC_CFGR_PPRE1_HCLK_DIV2,
		.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,

		.adcpre = RCC_CFGR_ADCPRE_PCLK2_DIV6,
		.usbpre = RCC_CFGR_USBPRE_PLL_CLK_DIV1_5,

		.ahb_frequency  = 72000000,
		.apb1_frequency = 36000000,
		.apb2_frequency = 72000000,
	},
	[RCC_CLOCK_HSE25_96MHZ] = {
		.pll_prediv2 = RCC_CFGR2_PREDIV_DIV5,
		.pll_mul2 = RCC_CFGR2_PLL2MUL_PLL2_CLK_MUL8,

		.pll_prediv1src = RCC_CFGR2_PREDIV1SRC_PLL2_CLK,
		.pll_prediv1 = RCC_CFGR2_PREDIV_DIV2,

		.pllsrc = RCC_CFGR_PLLSRC_HSE_CLK,
		.pllmul = RCC_CFGR_PLLMUL_PLL_CLK_MUL24,

		.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
		.ppre1 = RCC_CFGR_PPRE1_HCLK_DIV2,
		.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,

		.adcpre = RCC_CFGR_ADCPRE_PCLK2_DIV8,
		.usbpre = RCC_CFGR_USBPRE_PLL_CLK_DIV2,

		.ahb_frequency  = 96000000,
		.apb1_frequency = 48000000,
		.apb2_frequency = 96000000,
	},
	[RCC_CLOCK_HSE25_108MHZ] = {
		.pll_prediv2 = RCC_CFGR2_PREDIV_DIV5,
		.pll_mul2 = RCC_CFGR2_PLL2MUL_PLL2_CLK_MUL8,

		.pll_prediv1src = RCC_CFGR2_PREDIV1SRC_PLL2_CLK,
		.pll_prediv1 = RCC_CFGR2_PREDIV_DIV2,

		.pllsrc = RCC_CFGR_PLLSRC_HSE_CLK,
		.pllmul = RCC_CFGR_PLLMUL_PLL_CLK_MUL27,

		.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
		.ppre1 = RCC_CFGR_PPRE1_HCLK_DIV2,
		.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,

		.adcpre = RCC_CFGR_ADCPRE_PCLK2_DIV8,
		/* USB is not going to work. */

		.ahb_frequency  = 108000000,
		.apb1_frequency = 52000000,
		.apb2_frequency = 108000000,
	},
};

/*---------------------------------------------------------------------------*/
/** @brief RCC Clear the Oscillator Ready Interrupt Flag

Clear the interrupt flag that was set when a clock oscillator became ready to
use.

@param[in] osc Oscillator ID
*/

void rcc_osc_ready_int_clear(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL3:
		RCC_CIR |= RCC_CIR_PLL3RDYC;
		break;
	case RCC_PLL2:
		RCC_CIR |= RCC_CIR_PLL2RDYC;
		break;
	case RCC_PLL:
		RCC_CIR |= RCC_CIR_PLLRDYC;
		break;
	case RCC_HSE:
		RCC_CIR |= RCC_CIR_HSERDYC;
		break;
	case RCC_HSI:
		RCC_CIR |= RCC_CIR_HSIRDYC;
		break;
	case RCC_LSE:
		RCC_CIR |= RCC_CIR_LSERDYC;
		break;
	case RCC_LSI:
		RCC_CIR |= RCC_CIR_LSIRDYC;
		break;
	}
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Enable the Oscillator Ready Interrupt

@param[in] osc Oscillator ID
*/

void rcc_osc_ready_int_enable(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		RCC_CIR |= RCC_CIR_PLLRDYIE;
		break;
	case RCC_PLL2:
		RCC_CIR |= RCC_CIR_PLL2RDYIE;
		break;
	case RCC_PLL3:
		RCC_CIR |= RCC_CIR_PLL3RDYIE;
		break;
	case RCC_HSE:
		RCC_CIR |= RCC_CIR_HSERDYIE;
		break;
	case RCC_HSI:
		RCC_CIR |= RCC_CIR_HSIRDYIE;
		break;
	case RCC_LSE:
		RCC_CIR |= RCC_CIR_LSERDYIE;
		break;
	case RCC_LSI:
		RCC_CIR |= RCC_CIR_LSIRDYIE;
		break;
	}
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Disable the Oscillator Ready Interrupt

@param[in] osc Oscillator ID
*/

void rcc_osc_ready_int_disable(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		RCC_CIR &= ~RCC_CIR_PLLRDYIE;
		break;
	case RCC_PLL2:
		RCC_CIR &= ~RCC_CIR_PLL2RDYIE;
		break;
	case RCC_PLL3:
		RCC_CIR &= ~RCC_CIR_PLL3RDYIE;
		break;
	case RCC_HSE:
		RCC_CIR &= ~RCC_CIR_HSERDYIE;
		break;
	case RCC_HSI:
		RCC_CIR &= ~RCC_CIR_HSIRDYIE;
		break;
	case RCC_LSE:
		RCC_CIR &= ~RCC_CIR_LSERDYIE;
		break;
	case RCC_LSI:
		RCC_CIR &= ~RCC_CIR_LSIRDYIE;
		break;
	}
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Read the Oscillator Ready Interrupt Flag

@param[in] osc Oscillator ID
@returns int. Boolean value for flag set.
*/

int rcc_osc_ready_int_flag(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		return ((RCC_CIR & RCC_CIR_PLLRDYF) != 0);
		break;
	case RCC_PLL2:
		return ((RCC_CIR & RCC_CIR_PLL2RDYF) != 0);
		break;
	case RCC_PLL3:
		return ((RCC_CIR & RCC_CIR_PLL3RDYF) != 0);
		break;
	case RCC_HSE:
		return ((RCC_CIR & RCC_CIR_HSERDYF) != 0);
		break;
	case RCC_HSI:
		return ((RCC_CIR & RCC_CIR_HSIRDYF) != 0);
		break;
	case RCC_LSE:
		return ((RCC_CIR & RCC_CIR_LSERDYF) != 0);
		break;
	case RCC_LSI:
		return ((RCC_CIR & RCC_CIR_LSIRDYF) != 0);
		break;
	}

	cm3_assert_not_reached();
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Clear the Clock Security System Interrupt Flag

*/

void rcc_css_int_clear(void)
{
	RCC_CIR |= RCC_CIR_CSSC;
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Read the Clock Security System Interrupt Flag

@returns int. Boolean value for flag set.
*/

int rcc_css_int_flag(void)
{
	return ((RCC_CIR & RCC_CIR_CSSF) != 0);
}

bool rcc_is_osc_ready(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		return RCC_CR & RCC_CR_PLLRDY;
	case RCC_PLL2:
		return RCC_CR & RCC_CR_PLL2RDY;
	case RCC_PLL3:
		return RCC_CR & RCC_CR_PLL3RDY;
	case RCC_HSE:
		return RCC_CR & RCC_CR_HSERDY;
	case RCC_HSI:
		return RCC_CR & RCC_CR_HSIRDY;
	case RCC_LSE:
		return RCC_BDCR & RCC_BDCR_LSERDY;
	case RCC_LSI:
		return RCC_CSR & RCC_CSR_LSIRDY;
	}
	return false;
}

void rcc_wait_for_osc_ready(enum rcc_osc osc)
{
	while (!rcc_is_osc_ready(osc));
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Turn on an Oscillator.

Enable an oscillator and power on. Each oscillator requires an amount of time
to settle to a usable state. Refer to datasheets for time delay information. A
status flag is available to indicate when the oscillator becomes ready (see
@ref rcc_osc_ready_int_flag and @ref rcc_wait_for_osc_ready).

@note The LSE clock is in the backup domain and cannot be enabled until the
backup domain write protection has been removed (see @ref
pwr_disable_backup_domain_write_protect).

@param[in] osc Oscillator ID
*/

void rcc_osc_on(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		RCC_CR |= RCC_CR_PLLON;
		break;
	case RCC_PLL2:
		RCC_CR |= RCC_CR_PLL2ON;
		break;
	case RCC_PLL3:
		RCC_CR |= RCC_CR_PLL3ON;
		break;
	case RCC_HSE:
		RCC_CR |= RCC_CR_HSEON;
		break;
	case RCC_HSI:
		RCC_CR |= RCC_CR_HSION;
		break;
	case RCC_LSE:
		RCC_BDCR |= RCC_BDCR_LSEON;
		break;
	case RCC_LSI:
		RCC_CSR |= RCC_CSR_LSION;
		break;
	}
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Turn off an Oscillator.

Disable an oscillator and power off.

@note An oscillator cannot be turned off if it is selected as the system clock.
@note The LSE clock is in the backup domain and cannot be disabled until the
backup domain write protection has been removed (see
@ref pwr_disable_backup_domain_write_protect) or the backup domain has been
(see reset @ref rcc_backupdomain_reset).

@param[in] osc Oscillator ID
*/

void rcc_osc_off(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		RCC_CR &= ~RCC_CR_PLLON;
		break;
	case RCC_PLL2:
		RCC_CR &= ~RCC_CR_PLL2ON;
		break;
	case RCC_PLL3:
		RCC_CR &= ~RCC_CR_PLL3ON;
		break;
	case RCC_HSE:
		RCC_CR &= ~RCC_CR_HSEON;
		break;
	case RCC_HSI:
		RCC_CR &= ~RCC_CR_HSION;
		break;
	case RCC_LSE:
		RCC_BDCR &= ~RCC_BDCR_LSEON;
		break;
	case RCC_LSI:
		RCC_CSR &= ~RCC_CSR_LSION;
		break;
	}
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Enable the Clock Security System.

*/

void rcc_css_enable(void)
{
	RCC_CR |= RCC_CR_CSSON;
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Disable the Clock Security System.

*/

void rcc_css_disable(void)
{
	RCC_CR &= ~RCC_CR_CSSON;
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the Source for the System Clock.

@param[in] clk System Clock Selection @ref rcc_cfgr_scs
*/

void rcc_set_sysclk_source(uint32_t clk)
{
	RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_SW) |
			(clk << RCC_CFGR_SW_SHIFT);
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the PLL Multiplication Factor.

@note This only has effect when the PLL is disabled.

@param[in] mul PLL multiplication factor @ref rcc_cfgr_pmf
*/

void rcc_set_pll_multiplication_factor(uint32_t mul)
{
	RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_PLLMUL_0_3 & ~RCC_CFGR_PLLMUL_4) |
			((mul & 0xf) << RCC_CFGR_PLLMUL_0_3_SHIFT) |
			((!!(mul & 0x10)) << RCC_CFGR_PLLMUL_4_SHIFT);
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the PLL2 Multiplication Factor.

@note This only has effect when the PLL is disabled.

@param[in] mul Unsigned int32. PLL multiplication factor @ref rcc_cfgr_pmf
*/

void rcc_set_pll2_multiplication_factor(uint32_t mul)
{
	RCC_CFGR2 = (RCC_CFGR2 & ~RCC_CFGR2_PLL2MUL) |
			(mul << RCC_CFGR2_PLL2MUL_SHIFT);
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the PLL3 Multiplication Factor.

@note This only has effect when the PLL is disabled.

@param[in] mul Unsigned int32. PLL multiplication factor @ref rcc_cfgr_pmf
*/

void rcc_set_pll3_multiplication_factor(uint32_t mul)
{
	RCC_CFGR2 = (RCC_CFGR2 & ~RCC_CFGR2_PLL3MUL) |
			(mul << RCC_CFGR2_PLL3MUL_SHIFT);
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the PLL Clock Source.

@note This only has effect when the PLL is disabled.

@param[in] pllsrc PLL clock source @ref rcc_cfgr_pcs
*/

void rcc_set_pll_source(uint32_t pllsrc)
{
	RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_PLLSRC) |
			(pllsrc << 16);
}

/*---------------------------------------------------------------------------*/
/** @brief RCC RTC Clock Enabled Flag

@returns uint32_t. Nonzero if the RTC Clock is enabled.
*/

uint32_t rcc_rtc_clock_enabled_flag(void)
{
	return RCC_BDCR & RCC_BDCR_RTCEN;
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Enable the RTC clock

*/

void rcc_enable_rtc_clock(void)
{
	RCC_BDCR |= RCC_BDCR_RTCEN;
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the Source for the RTC clock

@param[in] clock_source RTC clock source. Only HSE/128, LSE and LSI.
*/

void rcc_set_rtc_clock_source(enum rcc_osc clock_source)
{
	uint32_t reg32;

	switch (clock_source) {
	case RCC_LSE:
		/* Turn the LSE on and wait while it stabilises. */
		RCC_BDCR |= RCC_BDCR_LSEON;
		while ((reg32 = (RCC_BDCR & RCC_BDCR_LSERDY)) == 0);

		/* Choose LSE as the RTC clock source. */
		RCC_BDCR &= ~((1 << 8) | (1 << 9));
		RCC_BDCR |= (1 << 8);
		break;
	case RCC_LSI:
		/* Turn the LSI on and wait while it stabilises. */
		RCC_CSR |= RCC_CSR_LSION;
		while ((reg32 = (RCC_CSR & RCC_CSR_LSIRDY)) == 0);

		/* Choose LSI as the RTC clock source. */
		RCC_BDCR &= ~((1 << 8) | (1 << 9));
		RCC_BDCR |= (1 << 9);
		break;
	case RCC_HSE:
		/* Turn the HSE on and wait while it stabilises. */
		RCC_CR |= RCC_CR_HSEON;
		while ((reg32 = (RCC_CR & RCC_CR_HSERDY)) == 0);

		/* Choose HSE as the RTC clock source. */
		RCC_BDCR &= ~((1 << 8) | (1 << 9));
		RCC_BDCR |= (1 << 9) | (1 << 8);
		break;
	case RCC_PLL:
	case RCC_PLL2:
	case RCC_PLL3:
	case RCC_HSI:
		/* Unusable clock source, here to prevent warnings. */
		/* Turn off clock sources to RTC. */
		RCC_BDCR &= ~((1 << 8) | (1 << 9));
		break;
	}
}

/*---------------------------------------------------------------------------*/
/** @brief ADC Setup the A/D Clock

The ADC's have a common clock prescale setting.

@param[in] adcpre Prescale divider taken from @ref rcc_cfgr_adcpre
*/

void rcc_set_adcpre(uint32_t adcpre)
{
	RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_ADCPRE_0_1 & ~RCC_CFGR_ADCPRE_2) |
			((adcpre & 0x3) << RCC_CFGR_ADCPRE_0_1_SHIFT) |
			((!!(adcpre & 0x4)) << RCC_CFGR_ADCPRE_2_SHIFT);
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the APB2 Prescale Factor.

@param[in] ppre2 APB2 prescale factor @ref rcc_cfgr_apb2pre
*/

void rcc_set_ppre2(uint32_t ppre2)
{
	RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_PPRE2) |
			(ppre2 << RCC_CFGR_PPRE2_SHIFT);
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the APB1 Prescale Factor.

@note The APB1 clock frequency must not exceed 36MHz.

@param[in] ppre1 APB1 prescale factor @ref rcc_cfgr_apb1pre
*/

void rcc_set_ppre1(uint32_t ppre1)
{
	RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_PPRE1) |
			(ppre1 << RCC_CFGR_PPRE1_SHIFT);

}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the AHB Prescale Factor.

@param[in] hpre AHB prescale factor @ref rcc_cfgr_ahbpre
*/

void rcc_set_hpre(uint32_t hpre)
{
	RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_HPRE) |
			(hpre << RCC_CFGR_HPRE_SHIFT);

}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the USB Prescale Factor.

The prescale factor can be set to 1 (no prescale) for use when the PLL clock is
48MHz, or 1.5 to generate the 48MHz USB clock from a 72MHz PLL clock.

@note This bit cannot be reset while the USB clock is enabled.

@param[in] usbpre USB prescale factor @ref rcc_cfgr_usbpre
*/

void rcc_set_usbpre(uint32_t usbpre)
{
	RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_USBPRE) |
			(usbpre << RCC_CFGR_USBPRE_SHIFT);
}

/*---------------------------------------------------------------------------*/
/** @brief * Set PLL Source pre-divider
 *
 * @param[in] prediv division by prediv @ref rcc_cfgr2_prediv
 */

void rcc_set_prediv1(uint32_t prediv)
{
	RCC_CFGR2 = (RCC_CFGR2 & ~RCC_CFGR2_PREDIV1) |
			(prediv << RCC_CFGR2_PREDIV1_SHIFT);
}

/*---------------------------------------------------------------------------*/
/** @brief * Set PLL2 and PLL3 Source pre-divider
 *
 * @param[in] prediv division by prediv @ref rcc_cfgr2_prediv
 */

void rcc_set_prediv2(uint32_t prediv)
{
	RCC_CFGR2 = (RCC_CFGR2 & ~RCC_CFGR2_PREDIV2) |
			(prediv << RCC_CFGR2_PREDIV2_SHIFT);
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the PLL pre-divider Clock Source.

@param[in] pllsrc PLL pre-divider source @ref rcc_cfgr2_prediv1src
*/

void rcc_set_prediv1_source(uint32_t pllsrc)
{
	RCC_CFGR2 = (RCC_CFGR2 & ~RCC_CFGR2_PREDIV1SRC) |
			(pllsrc << 16);
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Get the System Clock Source.

@returns Unsigned int32. System clock source:
@li 00 indicates HSE
@li 01 indicates LSE
@li 02 indicates PLL
*/

uint32_t rcc_system_clock_source(void)
{
	/* Return the clock source which is used as system clock. */
	return (RCC_CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_SHIFT;
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Wait for System Clock Ready.

@param[in] clk System Clock Selection @ref rcc_cfgr_scs
*/

void rcc_wait_for_sysclk_source(uint32_t clk)
{
	while (rcc_system_clock_source() != clk);
}

/*---------------------------------------------------------------------------*/
/*
 * These functions are setting up the whole clock system for the most common
 * input clock and output clock configurations.
 */
/*---------------------------------------------------------------------------*/
/**
 * Setup clocks to run from PLL.
 * The arguments provide the pll source, multipliers, dividers, all that's
 * needed to establish a system clock.
 * @param clock clock information structure
 */
void rcc_clock_setup_pll(const struct rcc_clock_scale *clock)
{
	if (clock->pllsrc == RCC_CFGR_PLLSRC_HSE_CLK) {
		/* Enable external high-speed oscillator. */
		rcc_osc_on(RCC_HSE);
		rcc_wait_for_osc_ready(RCC_HSE);
	} else if (clock->pllsrc == RCC_CFGR_PLLSRC_HSI_CLK_DIV2) {
		/* Enable internal high-speed oscillator. */
		rcc_osc_on(RCC_HSI);
		rcc_wait_for_osc_ready(RCC_HSI);
	}

	/*
	 * Set prescalers for AHB, ADC, APB1, APB2 and USB.
	 * Do this before touching the PLL (TODO: why?).
	 */
	rcc_set_hpre(clock->hpre);
	rcc_set_ppre1(clock->ppre1);
	rcc_set_ppre2(clock->ppre2);

	rcc_set_adcpre(clock->adcpre);
	rcc_set_usbpre(clock->usbpre);

	/* Set the PLL multiplication factor. */
	rcc_set_pll_multiplication_factor(clock->pllmul);

	/* Set the PLL source. */
	rcc_set_pll_source(clock->pllsrc);

	if (clock->pllsrc == RCC_CFGR_PLLSRC_HSE_CLK) {
		rcc_set_prediv1(clock->pll_prediv1);
		rcc_set_prediv1_source(clock->pll_prediv1src);

		if (clock->pll_prediv1src == RCC_CFGR2_PREDIV1SRC_PLL2_CLK) {
			rcc_set_prediv2(clock->pll_prediv2);
			rcc_set_pll2_multiplication_factor(clock->pll_mul2);
			rcc_osc_on(RCC_PLL2);
			rcc_wait_for_osc_ready(RCC_PLL2);
		}
	}

	/* Enable PLL oscillator and wait for it to stabilize. */
	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);

	/* Select PLL as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);
	rcc_wait_for_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);

	/* Set the peripheral clock frequencies used */
	rcc_ahb_frequency = clock->ahb_frequency;
	rcc_apb1_frequency = clock->apb1_frequency;
	rcc_apb2_frequency = clock->apb2_frequency;
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Reset the Backup Domain

The backup domain registers are reset to disable RTC controls and clear user
data.
*/

void rcc_backupdomain_reset(void)
{
	/* Set the backup domain software reset. */
	RCC_BDCR |= RCC_BDCR_BDRST;

	/* Clear the backup domain software reset. */
	RCC_BDCR &= ~RCC_BDCR_BDRST;
}

/**@}*/
