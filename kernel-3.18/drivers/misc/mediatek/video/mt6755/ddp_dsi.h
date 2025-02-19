/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __DSI_DRV_H__
#define __DSI_DRV_H__


#include <linux/types.h>

#include "lcm_drv.h"
#include "ddp_hal.h"
#include "fbconfig_kdebug.h"
#include "cmdq_record.h"
#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------- */

#define DSI_CHECK_RET(expr)             \
	do {                                \
		DSI_STATUS ret = (expr);        \
		ASSERT(DSI_STATUS_OK == ret);   \
	} while (0)

/* --------------------------------------------------------------------------- */

#define		DSI_DCS_SHORT_PACKET_ID_0			0x05
#define		DSI_DCS_SHORT_PACKET_ID_1			0x15
#define		DSI_DCS_LONG_PACKET_ID				0x39
#define		DSI_DCS_READ_PACKET_ID				0x06

#define		DSI_GERNERIC_SHORT_PACKET_ID_1		0x13
#define		DSI_GERNERIC_SHORT_PACKET_ID_2		0x23
#define		DSI_GERNERIC_LONG_PACKET_ID			0x29
#define		DSI_GERNERIC_READ_LONG_PACKET_ID	0x14


#define		DSI_WMEM_CONTI						(0x3C)
#define		DSI_RMEM_CONTI						(0x3E)

/* ESD recovery method for video mode LCM */
#define		METHOD_NONCONTINUOUS_CLK			(0x1)
#define		METHOD_BUS_TURN_AROUND				(0x2)

/* State of DSI engine */
#define		DSI_VDO_VSA_VS_STATE				(0x008)
#define		DSI_VDO_VSA_HS_STATE				(0x010)
#define		DSI_VDO_VSA_VE_STATE				(0x020)
#define		DSI_VDO_VBP_STATE					(0x040)
#define		DSI_VDO_VACT_STATE					(0x080)
#define		DSI_VDO_VFP_STATE					(0x100)

/* --------------------------------------------------------------------------- */

	typedef enum {
		DSI_STATUS_OK = 0,

		DSI_STATUS_ERROR,
	} DSI_STATUS;


	typedef enum {
		SHORT_PACKET_RW = 0,
		FB_WRITE = 1,
		LONG_PACKET_W = 2,
		FB_READ = 3,
	} DSI_INS_TYPE;


	typedef enum {
		DISABLE_BTA = 0,
		ENABLE_BTA = 1,
	} DSI_CMDQ_BTA;


	typedef enum {
		LOW_POWER = 0,
		HIGH_SPEED = 1,
	} DSI_CMDQ_HS;


	typedef enum {
		CL_8BITS = 0,
		CL_16BITS = 1,
	} DSI_CMDQ_CL;


	typedef enum {
		DISABLE_TE = 0,
		ENABLE_TE = 1,
	} DSI_CMDQ_TE;


	typedef enum {
		DISABLE_RPT = 0,
		ENABLE_RPT = 1,
	} DSI_CMDQ_RPT;


	typedef struct {
		unsigned type:2;
		unsigned BTA:1;
		unsigned HS:1;
		unsigned CL:1;
		unsigned TE:1;
		unsigned Rsv:1;
		unsigned RPT:1;
	} DSI_CMDQ_CONFG, *PDSI_CMDQ_CONFIG;


	typedef struct {
		unsigned CONFG:8;
		unsigned Data_ID:8;
		unsigned Data0:8;
		unsigned Data1:8;
	} DSI_T0_INS, *PDSI_T0_INS;

	typedef struct {
		unsigned CONFG:8;
		unsigned Data_ID:8;
		unsigned mem_start0:8;
		unsigned mem_start1:8;
	} DSI_T1_INS, *PDSI_T1_INS;

	typedef struct {
		unsigned CONFG:8;
		unsigned Data_ID:8;
		unsigned WC16:16;
		unsigned int *pdata;
	} DSI_T2_INS, *PDSI_T2_INS;

	typedef struct {
		unsigned CONFG:8;
		unsigned Data_ID:8;
		unsigned mem_start0:8;
		unsigned mem_start1:8;
	} DSI_T3_INS, *PDSI_T3_INS;

	typedef struct {
		uint8_t TXDIV0;
		uint8_t TXDIV1;
		uint32_t SDM_PCW;
		uint8_t SSC_PH_INIT;
		uint16_t SSC_PRD;
		uint16_t SSC_DELTA1;
		uint16_t SSC_DELTA;
	} DSI_PLL_CONFIG;

	typedef enum {
		DSI_INTERFACE_0 = 0,
		DSI_INTERFACE_1,
		DSI_INTERFACE_DUAL,
		DSI_INTERFACE_NUM,
	} DSI_INTERFACE_ID;

	typedef enum {
		DSI_VFP = 0,
		DSI_VSA,
		DSI_VBP,
		DSI_VACT,
		DSI_HFP,
		DSI_HSA,
		DSI_HBP,
		DSI_BLLP,
		DSI_PORCH_NUM,
	} DSI_PORCH_TYPE;

	extern const LCM_UTIL_FUNCS PM_lcm_utils_dsi0;
	/* defined in mtkfb.c */
	extern bool is_ipoh_bootup;


	void DSI_ChangeClk(DISP_MODULE_ENUM module, uint32_t clk);
	int32_t DSI_ssc_enable(uint32_t dsi_idx, uint32_t en);
	uint32_t PanelMaster_get_CC(uint32_t dsi_idx);
	void PanelMaster_set_CC(uint32_t dsi_index, uint32_t enable);
	uint32_t PanelMaster_get_dsi_timing(uint32_t dsi_index, MIPI_SETTING_TYPE type);
	uint32_t PanelMaster_get_TE_status(uint32_t dsi_idx);
	void PanelMaster_DSI_set_timing(uint32_t dsi_index, MIPI_TIMING timing);
	unsigned int PanelMaster_set_PM_enable(unsigned int value);
	uint32_t DSI_dcs_read_lcm_reg_v2(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, uint8_t cmd,
				       uint8_t *buffer, uint8_t buffer_size);
	void *get_dsi_params_handle(uint32_t dsi_idx);
	void dsi_analysis(DISP_MODULE_ENUM module);
	void DSI_LFR_UPDATE(DISP_MODULE_ENUM module, cmdqRecHandle cmdq);
	void DSI_Set_LFR(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, unsigned int mode,
			 unsigned int type, unsigned int enable, unsigned int skip_num);
	DSI_STATUS DSI_BIST_Pattern_Test(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, bool enable, unsigned int color);
	int ddp_dsi_start(DISP_MODULE_ENUM module, void *cmdq);
	DSI_STATUS DSI_DumpRegisters(DISP_MODULE_ENUM module, int level);
	void DSI_ForceConfig(int forceconfig);
	int DSI_set_roi(int x, int y);
	int DSI_check_roi(void);
	int dsi_enable_irq(DISP_MODULE_ENUM module, void *handle, unsigned int enable);
    unsigned int DSI_dcs_read_lcm_reg_v2_wrapper_DSI0(uint8_t cmd, uint8_t *buffer, uint8_t buffer_size);
    void DSI_set_cmdq_V2_DSI0(void *cmdq, unsigned cmd, unsigned char count, unsigned char *para_list,unsigned char force_update);
    #ifdef CONFIG_HUAWEI_LCD_DSM
    int lcd_report_dsm_err(int type,int err_value,int add_value);
    int lcd_report_dsm_errno(int errno);
    #endif
#ifdef __cplusplus
}
#endif
#endif				/* __DPI_DRV_H__ */
