/*
 * LED Kernel Timer Trigger
 *
 * Copyright 2005-2006 Openedhand Ltd.
 *
 * Author: Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/leds.h>

static ssize_t led_delay_on_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	return sprintf(buf, "%lu\n", led_cdev->blink_delay_on);
}

static ssize_t led_delay_on_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	unsigned long state;
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &state);
	if (ret)
		return ret;

    led_blink_set(led_cdev, &state, &led_cdev->blink_delay_off, &led_cdev->blink_raise_duration, &led_cdev->blink_fall_duration);
	led_cdev->blink_delay_on = state;

	return size;
}

static ssize_t led_delay_off_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	return sprintf(buf, "%lu\n", led_cdev->blink_delay_off);
}

static ssize_t led_delay_off_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	unsigned long state;
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &state);
	if (ret)
		return ret;

    led_blink_set(led_cdev, &led_cdev->blink_delay_on, &state, &led_cdev->blink_raise_duration, &led_cdev->blink_fall_duration);
	led_cdev->blink_delay_off = state;

	return size;
}

static ssize_t led_raise_duration_show(struct device* dev,
                                       struct device_attribute* attr, char* buf)
{
    struct led_classdev* led_cdev = dev_get_drvdata(dev);

    return sprintf(buf, "%lu\n", led_cdev->blink_raise_duration);
}

static ssize_t led_raise_duration_store(struct device* dev,
                                        struct device_attribute* attr, const char* buf, size_t size)
{
    struct led_classdev* led_cdev = dev_get_drvdata(dev);
    unsigned long state;
    ssize_t ret = -EINVAL;

    ret = kstrtoul(buf, 10, &state);

    if (ret)
    { return ret; }

    led_blink_set(led_cdev, &led_cdev->blink_delay_on, &led_cdev->blink_delay_off, &state, &led_cdev->blink_fall_duration);
    led_cdev->blink_raise_duration = state;

    return size;
}

static ssize_t led_fall_duration_show(struct device* dev,
                                      struct device_attribute* attr, char* buf)
{
    struct led_classdev* led_cdev = dev_get_drvdata(dev);

    return sprintf(buf, "%lu\n", led_cdev->blink_fall_duration);
}

static ssize_t led_fall_duration_store(struct device* dev,
                                       struct device_attribute* attr, const char* buf, size_t size)
{
    struct led_classdev* led_cdev = dev_get_drvdata(dev);
    unsigned long state;
    ssize_t ret = -EINVAL;

    ret = kstrtoul(buf, 10, &state);

    if (ret)
    { return ret; }

    led_blink_set(led_cdev, &led_cdev->blink_delay_on, &led_cdev->blink_delay_off, &led_cdev->blink_raise_duration, &state);
    led_cdev->blink_fall_duration = state;

    return size;
}
static DEVICE_ATTR(delay_on, 0644, led_delay_on_show, led_delay_on_store);
static DEVICE_ATTR(delay_off, 0644, led_delay_off_show, led_delay_off_store);
static DEVICE_ATTR(raise_duration, 0644, led_raise_duration_show, led_raise_duration_store);
static DEVICE_ATTR(fall_duration, 0644, led_fall_duration_show, led_fall_duration_store);

static void timer_trig_activate(struct led_classdev *led_cdev)
{
	int rc;

	led_cdev->trigger_data = NULL;

	rc = device_create_file(led_cdev->dev, &dev_attr_delay_on);
	if (rc)
		return;
	rc = device_create_file(led_cdev->dev, &dev_attr_delay_off);
	if (rc)
		goto err_out_delayon;
    rc = device_create_file(led_cdev->dev, &dev_attr_raise_duration);
    if (rc)
        goto err_out_raiseduration;
    rc = device_create_file(led_cdev->dev, &dev_attr_fall_duration);
    if (rc)
        goto err_out_fallduration;

    led_blink_set(led_cdev, &led_cdev->blink_delay_on,
              &led_cdev->blink_delay_off, &led_cdev->blink_raise_duration, &led_cdev->blink_fall_duration);
    led_cdev->activated = true;

	return;

err_out_fallduration:
    device_remove_file(led_cdev->dev, &dev_attr_raise_duration);
err_out_raiseduration:
    device_remove_file(led_cdev->dev, &dev_attr_delay_off);
err_out_delayon:
	device_remove_file(led_cdev->dev, &dev_attr_delay_on);
}

static void timer_trig_deactivate(struct led_classdev *led_cdev)
{
	if (led_cdev->activated) {
		device_remove_file(led_cdev->dev, &dev_attr_delay_on);
		device_remove_file(led_cdev->dev, &dev_attr_delay_off);
        device_remove_file(led_cdev->dev, &dev_attr_raise_duration);
        device_remove_file(led_cdev->dev, &dev_attr_fall_duration);
		led_cdev->activated = false;
	}

	/* Stop blinking */
	led_set_brightness(led_cdev, LED_OFF);
}

static struct led_trigger timer_led_trigger = {
	.name     = "timer",
	.activate = timer_trig_activate,
	.deactivate = timer_trig_deactivate,
};

static int __init timer_trig_init(void)
{
	return led_trigger_register(&timer_led_trigger);
}

static void __exit timer_trig_exit(void)
{
	led_trigger_unregister(&timer_led_trigger);
}

module_init(timer_trig_init);
module_exit(timer_trig_exit);

MODULE_AUTHOR("Richard Purdie <rpurdie@openedhand.com>");
MODULE_DESCRIPTION("Timer LED trigger");
MODULE_LICENSE("GPL");
