/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#ifndef __SX9500_SPECIFICS_H__
#define __SX9500_SPECIFICS_H__

#include <linux/input/smtc/misc/sx9500_platform_data.h>
#include <linux/input/smtc/misc/sx9500_i2c_reg.h>

static int sx9500_get_nirq_state(void)
{
	return !gpio_get_value(GPIO_SX9500_NIRQ);
}

/* Define Registers that need to be initialized to values different than
 * default
 */
static struct smtc_reg_data sx9500_i2c_reg_setup[] = {
	{
		.reg = SX9500_IRQ_ENABLE_REG,
		.val = 0xFF,
	},
	{
		.reg = SX9500_CPS_CTRL1_REG,
		.val = 0x43,
	},
	{
		.reg = SX9500_CPS_CTRL2_REG,
		.val = 0x77,
	},
	{
		.reg = SX9500_CPS_CTRL3_REG,
		.val = 0x01,
	},
	{
		.reg = SX9500_CPS_CTRL4_REG,
		.val = 0x20,
	},
	{
		.reg = SX9500_CPS_CTRL5_REG,
		.val = 0x16,
	},
	{
		.reg = SX9500_CPS_CTRL6_REG,
		.val = 0x04,
	},
	{
		.reg = SX9500_CPS_CTRL7_REG,
		.val = 0x40,
	},
	{
		.reg = SX9500_CPS_CTRL8_REG,
		.val = 0x00,
	},
	{
		.reg = SX9500_CPS_CTRL0_REG,
		.val = 0x2C,
	},
};


static struct _buttonInfo psmtcButtons[] = {
	{
		.keycode = KEY_0,
		.mask = SX9500_TCHCMPSTAT_TCHSTAT0_FLAG,
	},
	{
		.keycode = KEY_1,
		.mask = SX9500_TCHCMPSTAT_TCHSTAT1_FLAG,
	},
	{
		.keycode = KEY_2,
		.mask = SX9500_TCHCMPSTAT_TCHSTAT2_FLAG,
	},
	{
		.keycode = KEY_3,
		.mask = SX9500_TCHCMPSTAT_TCHSTAT3_FLAG,
	},
};

static struct _totalButtonInformation smtcButtonInformation = {
	.buttons = psmtcButtons,
	.buttonSize = ARRAY_SIZE(psmtcButtons),
};

static sx9500_platform_data_t sx9500_config = {
	/* Function pointer to get the NIRQ state (1->NIRQ-low, 0->NIRQ-high) */
	.get_is_nirq_low = sx9500_get_nirq_state,
	/* pointer to an initializer function. Here in case needed in the future */
	//.init_platform_hw = sx9500_init_ts,
	.init_platform_hw = NULL,
	/* pointer to an exit function. Here in case needed in the future */
	//.exit_platform_hw = sx9500_exit_ts,
	.exit_platform_hw = NULL,
	
	.pi2c_reg = sx9500_i2c_reg_setup,
	.i2c_reg_num = ARRAY_SIZE(sx9500_i2c_reg_setup),

	.pbuttonInformation = &smtcButtonInformation,
};

#endif /* __SX9500_SPECIFICS_H__ */
