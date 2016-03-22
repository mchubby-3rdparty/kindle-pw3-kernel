/*
 * Copyright 2014 Amazon Technologies, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307  USA
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <net/bluetooth/bt_pwr_ctrl.h>

extern void brcm_gpio_bt_power_enable(int enable);
static unsigned enable = 0;

static ssize_t
btpwrctrl_enable_show(struct device* dev, struct device_attribute* attr, char* buf)
{
	return sprintf(buf, "%d\n", enable);
}

static ssize_t
btpwrctrl_enable_store(struct device* dev,
	struct device_attribute* attr,
	const char* buf,
	size_t size)
{
	if (sscanf(buf, "%d", (unsigned*)&enable) <= 0)
		return -EINVAL;
	printk(KERN_ERR "\nEntered value %d, turning BT chip %s\n", enable, (enable?"ON":"OFF"));

	brcm_gpio_bt_power_enable(!!enable);

	return size;
}
static DEVICE_ATTR(btenable, 0644, btpwrctrl_enable_show, btpwrctrl_enable_store);


static int bt_pwrctrl_probe(struct platform_device *pdev)
{
	int ret=0;
	struct bt_pwr_data *pdata = pdev->dev.platform_data;

	if(pdata && pdata->uart_pdev)
		printk(KERN_ERR "uart pdev is not NULL");
	else if(pdata)
		printk(KERN_ERR "uart pdev IS NULL");
	else
		printk(KERN_ERR "pdata itself is NULL");

        /* Create sys entry : mode */
	if ((ret = device_create_file(&pdev->dev, &dev_attr_btenable) < 0)) {
		printk(KERN_ERR "sys entry creation failed!");
		goto out_error;
	}

	return 0;

out_error:
	return ret;
}

static int bt_pwrctrl_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_btenable);
	return 0;
}

static struct platform_driver bt_pwrctrl_drv = {
	.driver = {
			.name = "bt_pwr_ctrl",
		   },
	.probe = bt_pwrctrl_probe,
	.remove = bt_pwrctrl_remove,
};

static __init int bt_pwrctrl_init(void)
{
	return platform_driver_register(&bt_pwrctrl_drv);
}

static void __exit bt_pwrctrl_exit(void)
{
	platform_driver_unregister(&bt_pwrctrl_drv);
}

module_init(bt_pwrctrl_init);
module_exit(bt_pwrctrl_exit);
MODULE_AUTHOR("Sandeep Marathe msandeep@lab126.com");
MODULE_DESCRIPTION("Amazon Bluetooth Power control driver");
MODULE_LICENSE("GPL v2");
