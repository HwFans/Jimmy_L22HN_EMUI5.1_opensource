/*
 * driver/base/topology.c - Populate sysfs with cpu topology information
 *
 * Written by: Zhang Yanmin, Intel Corporation
 *
 * Copyright (C) 2006, Intel Corp.
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 * NON INFRINGEMENT.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include <linux/mm.h>
#include <linux/cpu.h>
#include <linux/module.h>
#include <linux/hardirq.h>
#include <linux/topology.h>

#define define_one_ro_named(_name, _func)				\
	static DEVICE_ATTR(_name, 0444, _func, NULL)

#define define_one_ro(_name)				\
	static DEVICE_ATTR(_name, 0444, show_##_name, NULL)

#define define_id_show_func(name)				\
static ssize_t show_##name(struct device *dev,			\
		struct device_attribute *attr, char *buf)	\
{								\
	return sprintf(buf, "%d\n", topology_##name(dev->id));	\
}

#if defined(topology_thread_cpumask) || defined(topology_core_cpumask) || \
    defined(topology_book_cpumask)
static ssize_t show_cpumap(int type, const struct cpumask *mask, char *buf)
{
	ptrdiff_t len = PTR_ALIGN(buf + PAGE_SIZE - 1, PAGE_SIZE) - buf;
	int n = 0;

	if (len > 1) {
		n = type?
			cpulist_scnprintf(buf, len-2, mask) :
			cpumask_scnprintf(buf, len-2, mask);
		buf[n++] = '\n';
		buf[n] = '\0';
	}
	return n;
}
#endif

#define define_siblings_show_map(name)					\
static ssize_t show_##name(struct device *dev,				\
			   struct device_attribute *attr, char *buf)	\
{									\
	return show_cpumap(0, topology_##name(dev->id), buf);		\
}

#define define_siblings_show_list(name)					\
static ssize_t show_##name##_list(struct device *dev,			\
				  struct device_attribute *attr,	\
				  char *buf)				\
{									\
	return show_cpumap(1, topology_##name(dev->id), buf);		\
}

#define define_siblings_show_func(name)		\
	define_siblings_show_map(name); define_siblings_show_list(name)

define_id_show_func(physical_package_id);
define_one_ro(physical_package_id);

define_id_show_func(core_id);
define_one_ro(core_id);

define_siblings_show_func(thread_cpumask);
define_one_ro_named(thread_siblings, show_thread_cpumask);
define_one_ro_named(thread_siblings_list, show_thread_cpumask_list);

define_siblings_show_func(core_cpumask);
define_one_ro_named(core_siblings, show_core_cpumask);
define_one_ro_named(core_siblings_list, show_core_cpumask_list);

#ifdef CONFIG_SCHED_BOOK
define_id_show_func(book_id);
define_one_ro(book_id);
define_siblings_show_func(book_cpumask);
define_one_ro_named(book_siblings, show_book_cpumask);
define_one_ro_named(book_siblings_list, show_book_cpumask_list);
#endif

static struct attribute *default_attrs[] = {
	&dev_attr_physical_package_id.attr,
	&dev_attr_core_id.attr,
	&dev_attr_thread_siblings.attr,
	&dev_attr_thread_siblings_list.attr,
	&dev_attr_core_siblings.attr,
	&dev_attr_core_siblings_list.attr,
#ifdef CONFIG_SCHED_BOOK
	&dev_attr_book_id.attr,
	&dev_attr_book_siblings.attr,
	&dev_attr_book_siblings_list.attr,
#endif
	NULL
};

static struct attribute_group topology_attr_group = {
	.attrs = default_attrs,
	.name = "topology"
};

/* Add/Remove cpu_topology interface for CPU device */
static int topology_add_dev(unsigned int cpu)
{
	struct device *dev = get_cpu_device(cpu);

    if (dev)
        return sysfs_create_group(&dev->kobj, &topology_attr_group);
    else
        return -EINVAL;
}

static void topology_remove_dev(unsigned int cpu)
{
	struct device *dev = get_cpu_device(cpu);

    if (dev)
        sysfs_remove_group(&dev->kobj, &topology_attr_group);
}

static int topology_cpu_callback(struct notifier_block *nfb,
				 unsigned long action, void *hcpu)
{
	unsigned int cpu = (unsigned long)hcpu;
	int rc = 0;

	switch (action) {
	case CPU_ONLINE:
	case CPU_ONLINE_FROZEN:
		rc = topology_add_dev(cpu);
		break;
	case CPU_UP_PREPARE:
	case CPU_UP_PREPARE_FROZEN:
	case CPU_UP_CANCELED:
	case CPU_UP_CANCELED_FROZEN:
		break;
	case CPU_DEAD:
	case CPU_DEAD_FROZEN:
		topology_remove_dev(cpu);
		break;
	}
	return notifier_from_errno(rc);
}

static int topology_sysfs_init(void)
{
	int cpu;
	int rc = 0;

	cpu_notifier_register_begin();

	for_each_online_cpu(cpu) {
		rc = topology_add_dev(cpu);
		if (rc)
			goto out;
	}
	__hotcpu_notifier(topology_cpu_callback, 0);

out:
	cpu_notifier_register_done();
	return rc;
}

device_initcall(topology_sysfs_init);
