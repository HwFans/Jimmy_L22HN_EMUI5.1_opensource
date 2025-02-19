/*****************************************************************************
 *
 * Copyright (c) 2013 mCube, Inc.  All rights reserved.
 *
 * This source is subject to the mCube Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of mCube Inc.
 *
 * All other rights reserved.
 *
 * This code and information are provided "as is" without warranty of any
 * kind, either expressed or implied, including but not limited to the
 * implied warranties of merchantability and/or fitness for a
 * particular purpose.
 *
 * The following software/firmware and/or related documentation ("mCube Software")
 * have been modified by mCube Inc. All revisions are subject to any receiver's
 * applicable license agreements with mCube Inc.
 *
 * Accelerometer Sensor Driver
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *****************************************************************************/

#ifndef _MC3XXX_H_
    #define _MC3XXX_H_

#include <linux/ioctl.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
/***********************************************
 *** REGISTER MAP
 ***********************************************/
#define MC3XXX_REG_XOUT                    0x00
#define MC3XXX_REG_YOUT                    0x01
#define MC3XXX_REG_ZOUT                    0x02
#define MC3XXX_REG_TILT_STATUS             0x03
#define MC3XXX_REG_SAMPLE_RATE_STATUS      0x04
#define MC3XXX_REG_SLEEP_COUNT             0x05
#define MC3XXX_REG_INTERRUPT_ENABLE        0x06
#define MC3XXX_REG_MODE_FEATURE            0x07
#define MC3XXX_REG_SAMPLE_RATE             0x08
#define MC3XXX_REG_TAP_DETECTION_ENABLE    0x09
#define MC3XXX_REG_TAP_DWELL_REJECT        0x0A
#define MC3XXX_REG_DROP_CONTROL            0x0B
#define MC3XXX_REG_SHAKE_DEBOUNCE          0x0C
#define MC3XXX_REG_XOUT_EX_L               0x0D
#define MC3XXX_REG_XOUT_EX_H               0x0E
#define MC3XXX_REG_YOUT_EX_L               0x0F
#define MC3XXX_REG_YOUT_EX_H               0x10
#define MC3XXX_REG_ZOUT_EX_L               0x11
#define MC3XXX_REG_ZOUT_EX_H               0x12
#define MC3XXX_REG_RANGE_CONTROL           0x20
#define MC3XXX_REG_SHAKE_THRESHOLD         0x2B
#define MC3XXX_REG_UD_Z_TH                 0x2C
#define MC3XXX_REG_UD_X_TH                 0x2D
#define MC3XXX_REG_RL_Z_TH                 0x2E
#define MC3XXX_REG_RL_Y_TH                 0x2F
#define MC3XXX_REG_FB_Z_TH                 0x30
#define MC3XXX_REG_DROP_THRESHOLD          0x31
#define MC3XXX_REG_TAP_THRESHOLD           0x32
#define MC3XXX_REG_PRODUCT_CODE            0x3B


//#define MC3451_REG_BYPASS                   0x01
#define MC3451_REG_DOWNLOAD_ADDR           0x01
#define MC3451_REG_SRAM_ADDR               0x02
#define MC3451_REG_ANY_MOTION               0x03
#define MC3451_REG_PEDOMETER_CONTROL       0x04
#define MC3451_REG_SLEEP                   0x05
#define MC3451_REG_READ_XYZ                   0x06
#define MC3451_REG_PEDOMETER               0x07
#define MC3451_REG_VERSION                   0x08
#define MC3451_REG_RTB                       0x10//RTB-->RETURN BOOTLOADER
#define MOTION_THM_COMMAND                   0x11
#define MOTION_DUM_COMMAND                   0x12
#define MOTION_THS_COMMAND                   0x13
#define MOTION_DUS_COMMAND                   0x14
#define STEP_WAITING_COMMAND               0x15
#define THRETH_DYN_COMMAND                   0x16
#define MINCOUNT_CON_COMMAND               0x17
#define TIMEOUT_CON_COMMAND                0x18
#define WRITE_PEDOCOUNT_COMMAND            0x19




/***********************************************
 *** RETURN CODE
 ***********************************************/
#define MC3451_RETCODE_SUCCESS                 (0)
#define MC3451_RETCODE_ERROR_I2C               (-1)
#define MC3451_RETCODE_ERROR_NULL_POINTER      (-2)
#define MC3451_RETCODE_ERROR_STATUS            (-3)
#define MC3451_RETCODE_ERROR_SETUP             (-4)
#define MC3451_RETCODE_ERROR_GET_DATA          (-5)
#define MC3451_RETCODE_ERROR_IDENTIFICATION    (-6)

/***********************************************
 *** CONFIGURATION
 ***********************************************/
#define MC3451_BUF_SIZE    256

char pedo10_sram_hex[] = {
    #include "./firmware/MC3451_V0A_FW.i"
};

char pedo20_sram_hex[] = {
    #include "./firmware/MC3451_V0C_FW.i"
};

#endif    // END OF _MC3XXX_H_

