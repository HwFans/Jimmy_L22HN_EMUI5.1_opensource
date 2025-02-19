/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include "mtk_ion.h"
#include "ion_drv.h"
#include <linux/interrupt.h>
#include "mt_idle.h"
#include "mt_spm_reg.h"
/* #include "pcm_def.h" */
#include "mt_spm_idle.h"
#include "mt_smi.h"
#include "m4u.h"

#include "disp_drv_platform.h"
#include "disp_debug.h"
#include "disp_debug.h"
#include "disp_lcm.h"
#include "disp_utils.h"
#include "disp_session.h"
#include "primary_display.h"
#include "disp_helper.h"
#include "cmdq_def.h"
#include "cmdq_record.h"
#include "cmdq_reg.h"
#include "cmdq_core.h"
#include "ddp_manager.h"
#include "disp_lcm.h"
#include "ddp_clkmgr.h"
#include "mt_smi.h"
/* #include "mmdvfs_mgr.h" */
#include "disp_log.h"
#include "disp_lowpower.h"
/* device tree */
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/io.h>
/*#include "mach/eint.h"*/
#if defined(CONFIG_MTK_LEGACY)
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include <cust_eint.h>
#else
#include "disp_dts_gpio.h"
#endif

#ifdef CONFIG_HUAWEI_LCD_DSM//lcd
#include 	<dsm/dsm_pub.h>
int lcd_report_dsm_errno(int errno);
#endif

#include "disp_recovery.h"

extern unsigned int esd_recovery_level;

static struct task_struct *primary_display_check_task; /* For abnormal check */
static wait_queue_head_t _check_task_wq;	/* used for blocking check task  */
static atomic_t _check_task_wakeup = ATOMIC_INIT(0);	/* For  Check Task */

static wait_queue_head_t esd_ext_te_wq;	/* For EXT TE EINT Check */
static atomic_t esd_ext_te_event = ATOMIC_INIT(0);	/* For EXT TE EINT Check */
static unsigned int esd_check_mode;
static unsigned int esd_check_enable;
static unsigned int esd_init;

unsigned int get_esd_check_mode(void)
{
	return esd_check_mode;
}

void set_esd_check_mode(unsigned int mode)
{
	esd_check_mode = mode;
}

unsigned int _can_switch_check_mode(void)
{
	int ret = 0;

	if (primary_get_lcm()->params->dsi.customization_esd_check_enable == 0
		&& primary_get_lcm()->params->dsi.lcm_esd_check_table[0].cmd != 0)
		ret = 1;
	return ret;
}

bool _can_do_read_reg(void)
{
	bool ret = false;

	if (primary_display_is_video_mode())
		ret = true;
	else if (!disp_helper_get_option(DISP_OPT_IDLE_MGR))
		ret = true;
	if (primary_get_lcm()->params->dsi.lcm_esd_check_table[0].cmd == 0)
		ret = false;

	return ret;
}

static unsigned int _need_do_esd_check(void)
{
	int ret = 0;
#ifdef CONFIG_OF
	if ((primary_get_lcm()->params->dsi.esd_check_enable == 1) && (islcmconnected == 1))
		ret = 1;

#else
	if (primary_get_lcm()->params->dsi.esd_check_enable == 1)
		ret = 1;

#endif
	return ret;
}

/* For Cmd Mode Read LCM Check */
/* Config cmdq_handle_config_esd */
int _esd_check_config_handle_cmd(cmdqRecHandle handle)
{
	int ret = 0;		/* 0:success */

	/* 1.reset */
	cmdqRecReset(handle);

	primary_display_manual_lock();

	/* 2.write first instruction */
	/* cmd mode: wait CMDQ_SYNC_TOKEN_STREAM_EOF(wait trigger thread done) */
	cmdqRecWaitNoClear(handle, CMDQ_SYNC_TOKEN_STREAM_EOF);

	/* 3.clear CMDQ_SYNC_TOKEN_ESD_EOF(trigger thread need wait this sync token) */
	cmdqRecClearEventToken(handle, CMDQ_SYNC_TOKEN_ESD_EOF);

	/* 4.write instruction(read from lcm) */
	dpmgr_path_build_cmdq(primary_get_dpmgr_handle(), handle,
				CMDQ_ESD_CHECK_READ, 0);

	/* 5.set CMDQ_SYNC_TOKE_ESD_EOF(trigger thread can work now) */
	cmdqRecSetEventToken(handle, CMDQ_SYNC_TOKEN_ESD_EOF);

	primary_display_manual_unlock();

	/* 6.flush instruction */
	dprec_logger_start(DPREC_LOGGER_ESD_CMDQ, 0, 0);
	ret = cmdqRecFlush(handle);
	dprec_logger_done(DPREC_LOGGER_ESD_CMDQ, 0, 0);


	DISPMSG("[ESD]_esd_check_config_handle_cmd ret=%d\n", ret);


	if (ret)
		ret = 1;
	return ret;
}

/* For Vdo Mode Read LCM Check */
/* Config cmdq_handle_config_esd */
int _esd_check_config_handle_vdo(cmdqRecHandle handle)
{
	int ret = 0;		/* 0:success , 1:fail */

	/* 0.reset */
	cmdqRecReset(handle);

	primary_display_manual_lock();

	/* 1.wait stream eof first */
	cmdqRecWaitNoClear(handle, CMDQ_EVENT_DISP_RDMA0_EOF);
	cmdqRecWaitNoClear(handle, CMDQ_EVENT_MUTEX0_STREAM_EOF);

	/* 2.stop dsi vdo mode */
	dpmgr_path_build_cmdq(primary_get_dpmgr_handle(), handle, CMDQ_STOP_VDO_MODE,
			      0);

	/* 3.write instruction(read from lcm) */
	dpmgr_path_build_cmdq(primary_get_dpmgr_handle(), handle, CMDQ_ESD_CHECK_READ,
			      0);

	/* 4.start dsi vdo mode */
	dpmgr_path_build_cmdq(primary_get_dpmgr_handle(), handle, CMDQ_START_VDO_MODE,
			      0);
	cmdqRecClearEventToken(handle, CMDQ_EVENT_MUTEX0_STREAM_EOF);
	cmdqRecClearEventToken(handle, CMDQ_EVENT_DISP_RDMA0_EOF);
	/* 5. trigger path */
	dpmgr_path_trigger(primary_get_dpmgr_handle(), handle, CMDQ_ENABLE);
	/*	mutex sof wait*/
	ddp_mutex_set_sof_wait(dpmgr_path_get_mutex(primary_get_dpmgr_handle()),
				handle, 0);

	primary_display_manual_unlock();

	/* 6.flush instruction */
	primary_display_manual_lock();
	dprec_logger_start(DPREC_LOGGER_ESD_CMDQ, 0, 0);
	ret = cmdqRecFlush(handle);
	dprec_logger_done(DPREC_LOGGER_ESD_CMDQ, 0, 0);
	primary_display_manual_unlock();

	DISPMSG("[ESD]_esd_check_config_handle_vdo ret=%d\n", ret);


	if (ret)
		ret = 1;
	return ret;
}

/* For EXT TE EINT Check */
static irqreturn_t _esd_check_ext_te_irq_handler(int irq, void *data)
{
	MMProfileLogEx(ddp_mmp_get_events()->esd_vdo_eint, MMProfileFlagPulse, 0, 0);
	atomic_set(&esd_ext_te_event, 1);
	wake_up_interruptible(&esd_ext_te_wq);
	return IRQ_HANDLED;
}

#if defined(CONFIG_MTK_LEGACY)
int primary_display_switch_esd_mode(int mode)
{
	int ret = 0;
	int gpio_mode = 0;
	struct device_node *node;
	int irq;
	u32 ints[2] = { 0, 0 };

#ifndef GPIO_DSI_TE_PIN
	return ret;
#endif

	gpio_mode = mt_get_gpio_mode(GPIO_DSI_TE_PIN);

	/* switch to eint mode */
	if (mode == GPIO_EINT_MODE) {
		/* 1. set mode0 */
		/* keep dsi te mode */
		/* mt_set_gpio_mode(GPIO_DSI_TE_PIN, GPIO_DSI_TE_PIN_M_GPIO); */

		/* 2.register irq handler */
		node = of_find_compatible_node(NULL, NULL, "mediatek, DSI_TE-eint");
		if (node) {
			of_property_read_u32_array(node,
						   "debounce",
						   ints,
						   ARRAY_SIZE(ints));
			mt_gpio_set_debounce(ints[0], ints[1]);
			irq = irq_of_parse_and_map(node, 0);
			if (primary_get_lcm()->params->dsi.ext_te_edge == LCM_POLARITY_FALLING) {
				if (request_irq(irq, _esd_check_ext_te_irq_handler,
						IRQF_TRIGGER_FALLING, "DSI_TE-eint", NULL))
					/*IRQF_TRIGGER_NONE*/
					DISPERR("[ESD]EINT IRQ LINE NOT AVAILABLE!!\n");
			} else {
				if (request_irq(irq, _esd_check_ext_te_irq_handler,
						IRQF_TRIGGER_RISING, "DSI_TE-eint", NULL))
					/*IRQF_TRIGGER_NONE*/
					DISPERR("[ESD]EINT IRQ LINE NOT AVAILABLE!!\n");
			}
		} else
			DISPERR("[ESD][%s] can't find DSI_TE eint compatible node\n",  __func__);

	}

	if (mode == GPIO_DSI_MODE) {

		/* 1. unregister irq handler */
		node = of_find_compatible_node(NULL, NULL, "mediatek, DSI_TE-eint");
		if (node) {
			irq = irq_of_parse_and_map(node, 0);
			free_irq(irq, NULL);
		} else
			DISPERR("[ESD][%s] can't find DSI_TE eint compatible node\n",  __func__);

		/* 2. set dsi te mode */
		/* mt_set_gpio_mode(GPIO_DSI_TE_PIN, GPIO_DSI_TE_PIN_M_DSI_TE); */

	}

	return ret;
}
#else
int primary_display_switch_esd_mode(int mode)
{
	int ret = 0;
	/* int gpio_mode = 0; */
	struct device_node *node;
	int irq;
	u32 ints[2] = { 0, 0 };

	if (mode == GPIO_EINT_MODE) {

		/* 1. set mode0 */
		/* disp_dts_gpio_select_state(DTS_GPIO_STATE_TE_MODE_GPIO); */

		/* 2.register irq handler */
		node = of_find_compatible_node(NULL, NULL, "mediatek, DSI_TE-eint");
		if (node) {
			of_property_read_u32_array(node,
						   "debounce",
						   ints,
						   ARRAY_SIZE(ints));
			/* mt_gpio_set_debounce(ints[0], ints[1]); */
			irq = irq_of_parse_and_map(node, 0);
			if (primary_get_lcm()->params->dsi.ext_te_edge == LCM_POLARITY_FALLING) {
				if (request_irq(irq, _esd_check_ext_te_irq_handler,
						IRQF_TRIGGER_FALLING, "DSI_TE-eint", NULL))
					/*IRQF_TRIGGER_NONE*/
					DISPERR("[ESD]EINT IRQ LINE NOT AVAILABLE!!\n");
			} else {
				if (request_irq(irq, _esd_check_ext_te_irq_handler,
						IRQF_TRIGGER_RISING, "DSI_TE-eint", NULL))
					/*IRQF_TRIGGER_NONE*/
					DISPERR("[ESD]EINT IRQ LINE NOT AVAILABLE!!\n");
			}

		} else
			DISPERR("[ESD][%s] can't find DSI_TE eint compatible node\n", __func__);



	} else if (mode == GPIO_DSI_MODE) {
		/* 1. unregister irq handler */
		node = of_find_compatible_node(NULL, NULL, "mediatek, DSI_TE-eint");

		if (node) {
			irq = irq_of_parse_and_map(node, 0);
			free_irq(irq, NULL);
		} else
			DISPERR("[ESD][%s] can't find DSI_TE eint compatible node\n",  __func__);

		/* disp_dts_gpio_select_state(DTS_GPIO_STATE_TE_MODE_TE); */


	}

	return ret;
}
#endif

int do_esd_check_eint(void)
{
	int ret = 0;

	if (wait_event_interruptible_timeout(esd_ext_te_wq,
		atomic_read(&esd_ext_te_event), HZ / 2) > 0)
		ret = 0;	/* esd check pass */
	else
		ret = 1;	/* esd check fail */

	atomic_set(&esd_ext_te_event, 0);

	return ret;
}

int do_esd_check_dsi_te(void)
{
	int ret = 0;

	if (dpmgr_wait_event_timeout(primary_get_dpmgr_handle(), DISP_PATH_EVENT_IF_VSYNC, HZ / 2) > 0)
		ret = 0;	/* esd check pass */
	else
		ret = 1;	/* esd check fail */

	return ret;
}

int do_esd_check_read(void)
{
	int ret = 0;
	cmdqRecHandle handle = NULL;

	/* 0.create esd check cmdq */
	cmdqRecCreate(CMDQ_SCENARIO_DISP_ESD_CHECK, &handle);

	primary_display_manual_lock();
	dpmgr_path_build_cmdq(primary_get_dpmgr_handle(), handle, CMDQ_ESD_ALLC_SLOT, 0);
	primary_display_manual_unlock();

	DISPMSG("[ESD]ESD config thread=%p\n", handle);

	/* 1.use cmdq to read from lcm */
	if (primary_display_is_video_mode())
		ret = _esd_check_config_handle_vdo(handle);
	else
		ret = _esd_check_config_handle_cmd(handle);

	primary_display_manual_lock();

	if (ret == 1) {	/* cmdq fail */
		if (need_wait_esd_eof()) {
			/* Need set esd check eof synctoken to let trigger loop go. */
			cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_ESD_EOF);
		}
		/* do dsi reset */
		dpmgr_path_build_cmdq(primary_get_dpmgr_handle(), handle,
				      CMDQ_DSI_RESET, 0);
		goto destroy_cmdq;
	}

	DISPMSG("[ESD]ESD config thread done~\n");

	/* 2.check data(*cpu check now) */
	ret = dpmgr_path_build_cmdq(primary_get_dpmgr_handle(), handle,
				  CMDQ_ESD_CHECK_CMP, 0);
	if (ret)
		ret = 1;	/* esd check fail */


destroy_cmdq:
	dpmgr_path_build_cmdq(primary_get_dpmgr_handle(), handle,
			      CMDQ_ESD_FREE_SLOT, 0);
	primary_display_manual_unlock();

	/* 3.destroy esd config thread */
	cmdqRecDestroy(handle);

	return ret;
}
/* ESD CHECK FUNCTION */
/* return 1: esd check fail */
/* return 0: esd check pass */
int primary_display_esd_check(void)
{
	int ret = 0;
	unsigned int mode;

	dprec_logger_start(DPREC_LOGGER_ESD_CHECK, 0, 0);
	MMProfileLogEx(ddp_mmp_get_events()->esd_check_t, MMProfileFlagStart, 0, 0);
	DISPMSG("[ESD]ESD check begin\n");

	primary_display_manual_lock();
	if (primary_get_state() == DISP_SLEPT) {
		MMProfileLogEx(ddp_mmp_get_events()->esd_check_t, MMProfileFlagPulse, 1, 0);
		DISPMSG("[ESD]primary display path is slept?? -- skip esd check\n");
		primary_display_manual_unlock();
		goto done;
	}
	primary_display_manual_unlock();

	/*  Esd Check : EXT TE */
	if (primary_get_lcm()->params->dsi.customization_esd_check_enable == 0) {
		/* use te for esd check */
		MMProfileLogEx(ddp_mmp_get_events()->esd_extte, MMProfileFlagStart, 0, 0);

		mode = get_esd_check_mode();
		if (GPIO_EINT_MODE == mode) {
			DISPMSG("[ESD]ESD check eint\n");
			MMProfileLogEx(ddp_mmp_get_events()->esd_extte, MMProfileFlagPulse,
				primary_display_is_video_mode(), mode);
			primary_display_switch_esd_mode(mode);
			ret = do_esd_check_eint();
			mode = GPIO_DSI_MODE; /* used for mode switch */
			primary_display_switch_esd_mode(mode);
		} else if (GPIO_DSI_MODE == mode) {
			MMProfileLogEx(ddp_mmp_get_events()->esd_extte, MMProfileFlagPulse,
				primary_display_is_video_mode(), mode);
#if 0
			/* use eint do esd check instead of dsi te irq for lowpower */
			ret = do_esd_check_dsi_te();
#else
			DISPMSG("[ESD]ESD check read\n");
			if (_can_do_read_reg())
				ret = do_esd_check_read();
#endif
			mode = GPIO_EINT_MODE; /* used for mode switch */
		}
		if (disp_helper_get_option(DISP_OPT_ESD_CHECK_SWITCH))
			if (primary_display_is_video_mode()) {
				/* try eint & read switch on vdo mode */
				if (_can_switch_check_mode())
					set_esd_check_mode(mode);
			}

		MMProfileLogEx(ddp_mmp_get_events()->esd_extte, MMProfileFlagEnd, 0, ret);

		goto done;
	}

	/*  Esd Check : Read from lcm */
	MMProfileLogEx(ddp_mmp_get_events()->esd_rdlcm, MMProfileFlagStart, 0,
		       primary_display_cmdq_enabled());

	if (primary_display_cmdq_enabled() == 0) {
		DISPMSG("[ESD]not support cpu read do esd check\n");
		MMProfileLogEx(ddp_mmp_get_events()->esd_rdlcm, MMProfileFlagEnd, 0, ret);
		goto done;
	}

	MMProfileLogEx(ddp_mmp_get_events()->esd_rdlcm, MMProfileFlagPulse, 0,
		primary_display_is_video_mode());

	/* only cmd mode read & with disable mmsys clk will kick */
	if (disp_helper_get_option(DISP_OPT_IDLEMGR_ENTER_ULPS) && !primary_display_is_video_mode())
		primary_display_idlemgr_kick((char *)__func__, 1);
	ret = do_esd_check_read();


	MMProfileLogEx(ddp_mmp_get_events()->esd_rdlcm, MMProfileFlagEnd, 0, ret);

done:
	DISPMSG("[ESD]ESD check end, ret = %d\n", ret);
	MMProfileLogEx(ddp_mmp_get_events()->esd_check_t, MMProfileFlagEnd, 0, ret);
	dprec_logger_done(DPREC_LOGGER_ESD_CHECK, 0, 0);
	return ret;

}


static int primary_display_check_recovery_worker_kthread(void *data)
{
	struct sched_param param = {.sched_priority = RTPM_PRIO_FB_THREAD };
	int ret = 0;
	int i = 0;
	int esd_try_cnt = 5;	/* 20; */
	int recovery_done = 0;

	DISPFUNC();
	sched_setscheduler(current, SCHED_RR, &param);

	while (1) {
		msleep(2000);/*2s*/
		mmsys_enable = 4;
		ret = wait_event_interruptible(_check_task_wq, atomic_read(&_check_task_wakeup));
		if (ret < 0) {
			DISPMSG("[disp_check]check thread waked up accidently\n");
			continue;
		}

		_primary_path_switch_dst_lock();

		/* 1. esd check & recovery */
		if (esd_check_enable) {

			i = 0;/*repeat*/
			do {
				ret = primary_display_esd_check();

				if (ret == 1) {
					DISPERR("[ESD]esd check fail, will do esd recovery. try=%d\n", i);
					primary_display_esd_recovery();
					recovery_done = 1;
				} else
					break;

			} while (++i < esd_try_cnt);

			if (ret == 1) {
				DISPERR("[ESD]after esd recovery %d times, still fail, disable esd check\n",
					esd_try_cnt);
				primary_display_esd_check_enable(0);
			} else if (recovery_done == 1) {
                #ifdef CONFIG_HUAWEI_LCD_DSM//lcd
                lcd_report_dsm_errno( DSM_LCD_ESD_STATUS_ERROR_NO);
                #endif

				DISPMSG("[ESD]esd recovery success\n");
				recovery_done = 0;
			}
		}

		_primary_path_switch_dst_unlock();

		/* 2. other check & recovery */

		if (kthread_should_stop())
			break;
	}
	return 0;
}


/* ESD RECOVERY */
int primary_display_esd_recovery(void)
{
	DISP_STATUS ret = DISP_STATUS_OK;
	LCM_PARAMS *lcm_param = NULL;

	DISPFUNC();
	dprec_logger_start(DPREC_LOGGER_ESD_RECOVERY, 0, 0);
	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagStart, 0, 0);
	DISPMSG("[ESD]ESD recovery begin\n");
	primary_display_manual_lock();
	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse,
		       primary_display_is_video_mode(), 1);


	lcm_param = disp_lcm_get_params(primary_get_lcm());
	if (primary_get_state() == DISP_SLEPT) {
		DISPMSG("[ESD]esd recovery but primary display path is slept??\n");
		goto done;
	}
	primary_display_idlemgr_kick((char *)__func__, 0);
	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse, 0, 2);

	/* blocking flush before stop trigger loop */
	_blocking_flush();

	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse, 0, 3);

	DISPDBG("[ESD]display cmdq trigger loop stop[begin]\n");
	_cmdq_stop_trigger_loop();
	DISPMSG("[ESD]display cmdq trigger loop stop[end]\n");

	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse, 0, 4);

	DISPDBG("[ESD]stop dpmgr path[begin]\n");
	dpmgr_path_stop(primary_get_dpmgr_handle(), CMDQ_DISABLE);
	if (primary_display_is_decouple_mode())
			dpmgr_path_stop(primary_get_ovl2mem_handle(), CMDQ_DISABLE);
	DISPMSG("[ESD]stop dpmgr path[end]\n");

	if (dpmgr_path_is_busy(primary_get_dpmgr_handle())) {
		DISPMSG("[ESD]primary display path is busy after stop\n");
		dpmgr_wait_event_timeout(primary_get_dpmgr_handle(), DISP_PATH_EVENT_FRAME_DONE, HZ * 1);
		DISPMSG("[ESD]wait frame done ret:%d\n", ret);
	}
	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse, 0, 5);

	DISPDBG("[ESD]reset display path[begin]\n");
	dpmgr_path_reset(primary_get_dpmgr_handle(), CMDQ_DISABLE);
	if (primary_display_is_decouple_mode())
			dpmgr_path_reset(primary_get_ovl2mem_handle(), CMDQ_DISABLE);
	DISPMSG("[ESD]reset display path[end]\n");

	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse, 0, 6);
	{
		LCM_PARAMS *lcm_param = NULL;
		disp_ddp_path_config *data_config;
		struct ddp_io_golden_setting_arg gset_arg;

		ddp_disconnect_path(DDP_SCENARIO_PRIMARY_ALL, NULL);
		ddp_disconnect_path(DDP_SCENARIO_PRIMARY_RDMA0_COLOR0_DISP, NULL);
		DISPMSG("cmd/video mode=%d\n", primary_display_is_video_mode());
		dpmgr_path_set_video_mode(primary_get_dpmgr_handle(), primary_display_is_video_mode());

		dpmgr_path_connect(primary_get_dpmgr_handle(), CMDQ_DISABLE);
		if (primary_display_is_decouple_mode()) {
			if (primary_get_ovl2mem_handle())
				dpmgr_path_connect(primary_get_ovl2mem_handle(), CMDQ_DISABLE);
			else
				DISPERR("in decouple_mode but no ovl2mem_path_handle\n");
		}

		MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 1, 2);
		lcm_param = disp_lcm_get_params(primary_get_lcm());
		if (lcm_param == NULL) {
			DISPERR("[ESD]Fatal error, lcm_param is NULL\n");
			ret = -1;
		}

		data_config = dpmgr_path_get_last_config(primary_get_dpmgr_handle());
		memcpy(&(data_config->dispif_config), lcm_param, sizeof(LCM_PARAMS));

		data_config->dst_w = disp_helper_get_option(DISP_OPT_FAKE_LCM_WIDTH);
		data_config->dst_h = disp_helper_get_option(DISP_OPT_FAKE_LCM_HEIGHT);
		if (lcm_param->type == LCM_TYPE_DSI) {
			if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB888)
				data_config->lcm_bpp = 24;
			else if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB565)
				data_config->lcm_bpp = 16;
			else if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB666)
				data_config->lcm_bpp = 18;
		} else if (lcm_param->type == LCM_TYPE_DPI) {
			if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB888)
				data_config->lcm_bpp = 24;
			else if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB565)
				data_config->lcm_bpp = 16;
			if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB666)
				data_config->lcm_bpp = 18;
		}

		data_config->fps = primary_display_get_fps_nolock();
		data_config->dst_dirty = 1;

		data_config->ovl_dirty = 1;

		ret = dpmgr_path_config(primary_get_dpmgr_handle(), data_config, NULL);
		MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 2, 2);
		data_config->dst_dirty = 0;

		memset(&gset_arg, 0, sizeof(gset_arg));
		gset_arg.dst_mod_type = dpmgr_path_get_dst_module_type(primary_get_dpmgr_handle());
		gset_arg.is_decouple_mode = primary_display_is_decouple_mode();
		dpmgr_path_ioctl(primary_get_dpmgr_handle(), NULL, DDP_OVL_GOLDEN_SETTING, &gset_arg);

	}
	DISPDBG("[POWER]lcm suspend[begin]\n");
	disp_lcm_suspend(primary_get_lcm());
	DISPMSG("[POWER]lcm suspend[end]\n");
	DISPDBG("[ESD]lcm force init[begin]\n");
	disp_lcm_init(primary_get_lcm(), 1);
	DISPMSG("[ESD]lcm force init[end]\n");

        disp_lcm_set_backlight(primary_get_lcm(), NULL, esd_recovery_level);
	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse, 0, 8);

	DISPDBG("[ESD]start dpmgr path[begin]\n");
	dpmgr_path_start(primary_get_dpmgr_handle(), CMDQ_DISABLE);
	if (primary_display_is_decouple_mode())
			dpmgr_path_start(primary_get_ovl2mem_handle(), CMDQ_DISABLE);
	DISPMSG("[ESD]start dpmgr path[end]\n");
	if (dpmgr_path_is_busy(primary_get_dpmgr_handle())) {
		DISPERR("[ESD]Fatal error, we didn't trigger display path but it's already busy\n");
		ret = -1;
		/* goto done; */
	}

	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse, 0, 9);
	DISPDBG("[ESD]start cmdq trigger loop[begin]\n");
	_cmdq_start_trigger_loop();
	DISPMSG("[ESD]start cmdq trigger loop[end]\n");
	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse, 0, 10);
	if (primary_display_is_video_mode()) {
		/* for video mode, we need to force trigger here */
		/* for cmd mode, just set DPREC_EVENT_CMDQ_SET_EVENT_ALLOW when trigger loop start */
		dpmgr_path_trigger(primary_get_dpmgr_handle(), NULL, CMDQ_DISABLE);
	}
	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse, 0, 11);

	/* (in suspend) when we stop trigger loop
	* if no other thread is running, cmdq may disable its clock
	* all cmdq event will be cleared after suspend */
	cmdqCoreSetEvent(CMDQ_EVENT_DISP_WDMA0_EOF);

	/* set dirty to trigger one frame -- cmd mode */
	if (!primary_display_is_video_mode()) {
		cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
		mdelay(40);
	}
done:
	primary_display_manual_unlock();



	DISPMSG("[ESD]ESD recovery end\n");
	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagEnd, 0, 0);
	dprec_logger_done(DPREC_LOGGER_ESD_RECOVERY, 0, 0);
	return ret;
}




void primary_display_check_recovery_init(void)
{
	if (!esd_init) {
		/* primary display check thread init */
		primary_display_check_task =
			kthread_create(primary_display_check_recovery_worker_kthread, NULL,
			"disp_check");
		init_waitqueue_head(&_check_task_wq);

		esd_init = 1;
		if (disp_helper_get_option(DISP_OPT_ESD_CHECK_RECOVERY))
			wake_up_process(primary_display_check_task);
		init_waitqueue_head(&esd_ext_te_wq);
	}
	if (disp_helper_get_option(DISP_OPT_ESD_CHECK_RECOVERY)) {
		if (_need_do_esd_check()) {
			/* esd check init */
			set_esd_check_mode(GPIO_EINT_MODE);
			primary_display_esd_check_enable(1);
		} else {
			atomic_set(&_check_task_wakeup, 1);
			wake_up_interruptible(&_check_task_wq);
		}
	}
}

void primary_display_esd_check_enable(int enable)
{
	if (_need_do_esd_check()) {

		if (enable) {
			esd_check_enable = 1;
			DISPMSG("[ESD]enable esd check\n");
			atomic_set(&_check_task_wakeup, 1);
			wake_up_interruptible(&_check_task_wq);
		} else {
			esd_check_enable = 0;
			DISPMSG("[ESD]disable esd check\n");

		}
	} else
		DISPMSG("[ESD]do not support esd check\n");

}

unsigned int need_wait_esd_eof(void)
{
	int ret = 1;

	/* 1.esd check disable */
	/* 2.vdo mode */
	/* 3.cmd mode te */
	if (_need_do_esd_check() == 0)
		ret = 0;

	if (primary_display_is_video_mode())
		ret = 0;

	if (primary_get_lcm()->params->dsi.customization_esd_check_enable == 0)
		ret = 0;

	return ret;
}

