/*
 * goodix_ts.h - Touch driver header file of Goodix 
 * 
 * Copyright (C) 2015 - 2016 Goodix Technology Incorporated   
 * Copyright (C) 2015 - 2016 Yulong Cai <caiyulong@goodix.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the GOODiX's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 *
 */

#ifndef _GOODIX_TS_H_
#define _GOODIX_TS_H_
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/gpio.h>
#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#endif
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <huawei_platform/log/hw_log.h>
#include "../../huawei_ts_kit.h"
#include "../../huawei_ts_kit_algo.h"
#if defined(CONFIG_HUAWEI_DSM)
#include <dsm/dsm_pub.h>
#endif

#define GTP_DRIVER_VERSION          "v1.6<2016/07/08>"
#define IIC_DMA_MAX_TRANSFER_SIZE     250
#define IIC_MAX_TRANSFER_SIZE         8//250
#define GTP_CONFIG_ORG_LENGTH       228
#define GTP_CONFIG_MAX_LENGTH       240
#define GTP_CONFIG_EXT_LENGTH       128
#define GTP_CONFIG_MIN_LENGTH       186
#define SWITCH_OFF                  0
#define SWITCH_ON                   1
#define GTP_AUTO_UPDATE_CFG 0

/* request type */
#define GTP_RQST_CONFIG                 0x01
#define GTP_RQST_RESET                  0x03
#define GTP_RQST_NOISE_CFG				0x10
#define GTP_RQST_NORMA_CFG				0x11
#define GTP_RQST_RESPONDED              0x00
#define GTP_RQST_IDLE                   0xFF

/* Register define */
#define GTP_READ_COOR_ADDR          0x814E
#define GTP_REG_CMD                 0x8040
#define GTP_REG_CMD_ADDR1		0x80
#define GTP_REG_CMD_ADDR2		0x40
#define GTP_REG_SENSOR_ID           0x814A
#define GTP_REG_CONFIG_DATA         0x8047
#define GTP_REG_CONFIG_RESOLUTION   0x8048
#define GTP_REG_CONFIG_TRIGGER      0x8053
#define GTP_REG_EXT_CONFIG          0xBF7B
#define GTP_REG_VERSION             0x8140
/* need check */
#define GTP_REG_ESD_CHECK           0x8043
#define GTP_REG_FLASH_PASSBY        0x8006
#define GTP_REG_PROJECT_ID          0x81A0
#define GTP_REG_HN_PAIRED           0x81AA
#define GTP_REG_HN_MODE             0x81A8
#define GTP_REG_MODULE_SWITCH3      0x8058
#define GTP_REG_FW_CHK_MAINSYS      0x41E4
#define GTP_REG_FW_CHK_SUBSYS       0x5095
#define GTP_REG_WAKEUP_GESTURE		0x814B
#define GTP_REG_RQST                    0x8043
#define GTP_REG_HAVE_KEY                0x804E

/* cmd define */
#define GTP_CMD_SLEEP               0x05
#define GTP_CMD_CHARGER_ON          0x06
#define GTP_CMD_CHARGER_OFF         0x07
#define GTP_CMD_GESTURE_WAKEUP      0x08
#define GTP_CMD_CLEAR_CFG           0x10

/* cmd esd check*/
#define GTP_CMD_ESD_CHECK		0xAA
#define GTP_ESD_RESET_VALUE1		0x42
#define GTP_ESD_RESET_VALUE2		0x26
#define GTP_ESD_RESET_VALUE3		0x01
#define GTP_ESD_RESET_REG           0x4226

#define GTP_MAX_TOUCH    			TS_MAX_FINGER
#define GTP_MAX_KEY_NUM  			4
#define GTP_ADDR_LENGTH  			2
#define GTP_MAX_TOUCH_DATA_LEN		1 + 8 * GTP_MAX_TOUCH + 2
#define I2C_MASTER_CLOCK            300
#define GTP_REG_HW_INFO             0x4220

#define SLEEP_MODE					0x01
#define GESTURE_MODE				0x02

/* define offset in the config*/
#define RESOLUTION_LOC         1    // (GTP_REG_CONFIG_RESOLUTION - GTP_REG_CONFIG_DATA)
#define TRIGGER_LOC               6   //(GTP_REG_CONFIG_TRIGGER - GTP_REG_CONFIG_DATA)
#define MODULE_SWITCH3_LOC			(GTP_REG_MODULE_SWITCH3 - GTP_REG_CONFIG_DATA)
#define EXTERN_CFG_OFFSET			(0x805A - GTP_REG_CONFIG_DATA)

#define GTP_PROJECT_ID_LEN			11
#define GTP_VENDOR_NAME_LEN		8

#ifdef ROI
#define ROI_STA_REG     0xA6A2
#define ROI_HEAD_LEN    4  
#define ROI_DATA_REG    (ROI_STA_REG + ROI_HEAD_LEN)
#define ROI_READY_MASK    0x10
#define ROI_TRACKID_MASK  0x0f
#endif
#define PINCTRL_STATE_ACTIVE	"state_rst_output1"//"pmx_ts_active"
#define PINCTRL_STATE_SUSPEND	"state_rst_output0"
#define PINCTRL_STATE_RELEASE	"state_rst_output0"
#define PINCTRL_STATE_INT_HIGH	"state_eint_output1"//"ts_int_high"
#define PINCTRL_STATE_INT_LOW	"state_eint_output0"//"ts_int_low"
#define PINCTRL_STATE_AS_INT      "state_eint_as_int"
#define RAWDATA_TEST_START_LABEL	"SepcialTestNode="
#define GOODIX_RAW_DUMP_FILE		"/data/log/tp_self_test.txt"
#define FAIL			0
#define SUCCESS			1
#define CHECK_HW_STATUS_RETRY 		3
#define NO_NEED_UPDATE   4

extern u16 show_len;
extern u16 total_len;
#pragma pack(1)
struct goodix_hw_info {
	u8 product_id[5];
	u32 patch_id;
	u32 mask_id;
	u8 sensor_id;
	u8 match_opt;
	u8 vendor_name[8];
	u8 config_ver;
};
struct goodix_coordinate {
	u16 x;
	u16 y;
};
#pragma pack()

/*
 * fw_update_info - firmware update information
 * @update_type: should be the value below
 *	 UPDATE_TYPE_HEADER - using requset_firmware, fw_update_boot
 *	 UPDATE_TYPE_FILE - using filp_open, fw_update_sd
 * @statue: update status
 * @progress: update progress
 * @max_progress: max progress
 * @force_update: force update firmware
 */
struct fw_update_info {
	int update_type;
	int status;
	int progress;
	int max_progress;
	int force_update;
	struct fw_info *firmware_info;
	u32 fw_length;
	const struct firmware *fw;

	// file update
	char *fw_name;
	u8 *buffer;
	mm_segment_t old_fs;
	struct file *fw_file;

	// header update
	u8 *fw_data;
};

enum goodix_ts_feature {
	TS_FEATURE_NONE = 0,
	TS_FEATURE_GLOVE,
	TS_FEATURE_HOLSTER,
	TS_FEATURE_POCKET,
};

struct goodix_ts_config {
	bool initialized;
	char *name;
	u8 data[GTP_CONFIG_ORG_LENGTH];
	int size;
	int delay_ms;
};

#ifdef ROI
/*
 * goodix_ts_roi - finger sense data structure
 * @enabled: enable/disable finger sense feature
 * @data_ready: flag to indicate data ready state
 * @roi_rows: rows of roi data
 * @roi_clos: columes of roi data
 * @mutex: mutex
 * @rawdata: rawdata buffer
 */
struct goodix_ts_roi {
	bool enabled;
	bool data_ready;
	int track_id;
	unsigned int roi_rows;
	unsigned int roi_cols;
	struct mutex mutex;
	u16 *rawdata;
};
#endif

struct goodix_ts_data;

/*
 * goodix_ts_ops - touch driver operations stub
 */
struct goodix_ts_ops {
	int (*i2c_write)(u16 addr, u8 * buffer, s32 len);
	int (*i2c_read)(u16 addr, u8 * buffer, s32 len);
	int (*chip_reset)(void);
	int (*send_cmd)(u8 cmd, u8 data);
	int (*send_cfg)(struct goodix_ts_config *cfg_ptr);
	int (*i2c_read_dbl_check)(u16 addr, u8 * buffer, s32 len);
	int (*read_version)(struct goodix_hw_info * hw_info);
	int (*parse_cfg_data)(struct goodix_ts_data *ts,
			char *cfg_type, u8 *cfg, int *cfg_len, u8 sid);
};

/*
 * goodix_ts_data - main structure of touch driver
 * @pdev: pointer to huawei_touch platform device
 * @vdd_ana: analog regulator
 * @vcc_i2c: digital regulator
 * @pinctrl: pinctrl
 * @pins_default: pin configuration in default state
 * @pins_suspend: pin configuration in suspend state
 * @pins_gesture: pin configuration in gesture mode
 * @dev_data: pointer to huawei ts_device_data
 * @ops: driver operations stub
 * @hw_info: chip hardware infomation
 * @normal_config: normal config data, normal config data will be
 *		be used by firmware in normal working state
 * @glove_config: glove config data, glove config data will be
 *		be used by firmware in glove state
 * @holster_config: holster config data, holster config data will be
 *		be used by firmware in holster state
 * @roi: finger sense data structure
 * @max_x/@max_y: max resolution in x/y direction
 * @vdd_value: analog voltage value
 * @vio_value: digital voltage value
 * @flip_x/@flip_y: coordinates transformation
 * @rawdiff_mode: rawdiff mode, this mode is mainly used by
 *		goodix debug tools.
 * @tools_support: goodix debug tools supprots
 */
 struct goodix_test_threshold {
	int raw_data_min;
	int raw_data_max;

	int cb_test_min;
	int cb_test_max;

	int short_circuit_min;
	int  lcd_noise_max;
	int open_test_cb_min;
};
 struct goodix_test_params {

	struct goodix_test_threshold threshold;

	u8 channel_x_num;
	u8 channel_y_num;
	u8 key_num;
};
 struct goodix_delay_time {
	int hard_reset_delay;
	int erase_min_delay;
	int calc_crc_delay;
	int reboot_delay;
	int erase_query_delay;

	int write_flash_query_times;
	int read_ecc_query_times;
	int erase_flash_query_times;
	int upgrade_loop_times;
};
struct goodix_ts_data {
	struct platform_device *pdev;
	struct goodix_delay_time *delay_time;
	struct regulator *vdd_ana;
	struct regulator *vcc_i2c;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;
	struct pinctrl_state *pinctrl_state_int_high;
	struct pinctrl_state *pinctrl_state_int_low;
	struct pinctrl_state *pinctrl_state_as_int;
	struct mutex mutex_cmd;
	struct mutex mutex_cfg;
	struct ts_kit_device_data *goodix_device_data;
	struct ts_kit_platform_data* ts_platform_data;
	struct goodix_ts_ops ops;
	struct goodix_hw_info hw_info;
	struct goodix_ts_config normal_config;
	struct goodix_ts_config glove_config;
	struct goodix_ts_config holster_config;
	char project_id[GTP_PROJECT_ID_LEN];
	char vendor_name[GTP_VENDOR_NAME_LEN];
#ifdef ROI
	struct goodix_ts_roi roi;
#endif
	int max_x;
	int max_y;
	u32 vdd_value;
	u32 vio_value;
	bool flip_x;
	bool flip_y;

	char firmware_name[64];
	volatile bool rawdiff_mode;
	bool noise_env;
	bool tools_support;
// debug+
	bool sensor_id_valid;
	bool fw_update_ok;
	bool enter_suspend;
	bool enter_resume;
	bool enter_rawtest;
// debug-
	u8  fw_error;
	u8  enter_update;
};
extern struct dsm_client *ts_dclient;
enum FW_uptate_state
{
	GOODIX_CHECK_FILE_FAIL = 0,
	GOODIX_GET_IC_FW_MSG_FAIL,
	GOODIX_ENTER_UPDATE_MODE_FAIL,
	GOODIX_UPDATE_DSP_ISP_FAIL,
	GOODIX_BURN_DSP_ISP_FAIL,
	GOODIX_BURN_APP_ISP_FAIL,
	GOODIX_BURN_SS51_ISP_FAIL,
	GOODIX_BURN_DSP_FW_FAIL,
	GOODIX_BURN_FW_BOOT_FAIL,
	GOODIX_BURN_FW_BOOT_ISP_FAIL,
	GOODIX_BURN_FW_LINK_FAIL,
	GOODIX_BURN_FW_FINISH_FAIL,
	TS_UPDATE_STATE_UNDEFINE = 255,
};
#define GT9XX_FW_NAME                "jim_goodix_Helitec.bin"
#define AUTO_TEST_FW_NAME                    "touch_screen_firmware.bin"
extern bool update_from_sd;
extern struct goodix_ts_data *goodix_ts;
extern struct fw_update_info update_info;
extern struct ts_kit_device_data 	*g_goodix_dev_data;
#define GOODIX_NORMAL_CFG	0
#define GOODIX_GLOVE_CFG	1
#define GOODIX_HOLSTER_CFG	2

#define UPDATE_TYPE_HEADER 0
#define UPDATE_TYPE_FILE   1

/* Log define */
#define  GTP_FW_UPDATE_VERIFY   0 /* verify fw when updating*/
/*#define INPUT_TYPE_B_PROTOCOL*/

#define GTP_DEBUG_FUNC_ON 	1
#define GTP_DEBUG_ARRAY_ON	0
#define GTP_INFO(fmt,arg...)           printk("[GTP-INF][%s:%d] "fmt"\n", __func__, __LINE__, ##arg)
#define GTP_ERROR(fmt,arg...)          printk("[GTP-ERR][%s:%d] "fmt"\n", __func__, __LINE__, ##arg)
#define GTP_DEBUG(fmt,arg...)          printk("[GTP-DBG][%s:%d]"fmt"\n",__func__, __LINE__, ##arg)
#define GTP_DEBUG_FUNC()               do{\
										 if(GTP_DEBUG_FUNC_ON)\
										 printk("[GTP-FUNC] Func:%s@Line:%d\n",__func__,__LINE__);\
									   }while(0)
#define IS_NUM_OR_CHAR(x)    (((x) >= 'A' && (x) <= 'Z') || ((x) >= '0' && (x) <= '9'))

#define GTP_DEBUG_ARRAY(array, num)    do{\                                         
											u32 i;\
											u8* a = array;\
											if(GTP_DEBUG_ARRAY_ON)\
											{\
												printk("<<-GTP-DEBUG-ARRAY->>\n");\
												for (i = 0; i < (num); i++)\
												{\
													printk("%02x   ", (a)[i]);\
													if ((i + 1 ) %10 == 0)\
													{\
														printk("\n");\ 
													}\ 
												}\
												printk("\n");\
											}\
										}while(0)

int goodix_i2c_write(u16 addr, u8 * buffer, u16 len);
int goodix_i2c_read(u16 addr, u8 * buffer, u16 len);
int goodix_chip_reset(void);
int goodix_read_version(struct goodix_hw_info * hw_info);
int goodix_i2c_read_dbl_check(u16 addr, u8 * buffer, u16 len);
int goodix_send_cfg(struct goodix_ts_config *cfg_ptr);
extern s32 gt1x_init_panel(void);
int goodix_init_configs(struct goodix_ts_data *ts);
int goodix_pinctrl_select_normal(struct goodix_ts_data *ts);
int goodix_pinctrl_select_suspend(struct goodix_ts_data *ts);

extern int goodix_get_rawdata(struct ts_rawdata_info *info,
				struct ts_cmd_node *out_cmd);
extern struct ts_kit_device_data *goodix_get_device_data(void);
extern struct goodix_ts_data *goodix_get_platform_data(void);
extern s32 gup_enter_update_mode(void);
extern void gup_leave_update_mode(void);
extern int init_wr_node(void);
extern s32 gup_update_proc(int type);
int goodix_sleep_mode_out(void);
int goodix_put_device_outof_easy_wakeup(void);
#endif /* _GOODIX_TS_H_ */
