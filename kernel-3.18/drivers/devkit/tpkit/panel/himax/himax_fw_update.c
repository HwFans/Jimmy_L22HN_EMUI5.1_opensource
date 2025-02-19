/* Himax Android Driver Sample Code for HMX852xF chipset
*
* Copyright (C) 2014 Himax Corporation.
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
*/

#include "himax_ic.h"
#ifdef CONFIG_APP_INFO
#include <misc/app_info.h>
#endif

static int iref_number = 11;
unsigned int HX_UPDATE_FLAG = 0;
unsigned int HX_RESET_COUNT = 0;
unsigned int HX_ESD_RESET_COUNT = 0;
unsigned int FW_VER_MAJ_FLASH_ADDR = 0;
unsigned int FW_VER_MIN_FLASH_ADDR = 0;
unsigned int FW_CFG_VER_FLASH_ADDR = 0;
unsigned int CFG_VER_MAJ_FLASH_ADDR = 0;
unsigned int CFG_VER_MIN_FLASH_ADDR = 0;
static bool iref_found = false;
extern char himax_product_id[];
extern struct himax_ts_data *g_himax_ts_data;

#define BOOT_UPDATE_FIRMWARE_FLAG_FILENAME	"/system/etc/tp_test_parameters/boot_update_firmware.flag"
#define BOOT_UPDATE_FIRMWARE_FLAG "boot_update_firmware_flag:"
//used in firmware upgrade
//1uA
static unsigned char E_IrefTable_1[16][2] = { {0x20,0x0F},{0x20,0x1F},{0x20,0x2F},{0x20,0x3F},
											{0x20,0x4F},{0x20,0x5F},{0x20,0x6F},{0x20,0x7F},
											{0x20,0x8F},{0x20,0x9F},{0x20,0xAF},{0x20,0xBF},
											{0x20,0xCF},{0x20,0xDF},{0x20,0xEF},{0x20,0xFF}};

//2uA
static unsigned char E_IrefTable_2[16][2] = { {0xA0,0x0E},{0xA0,0x1E},{0xA0,0x2E},{0xA0,0x3E},
											{0xA0,0x4E},{0xA0,0x5E},{0xA0,0x6E},{0xA0,0x7E},
											{0xA0,0x8E},{0xA0,0x9E},{0xA0,0xAE},{0xA0,0xBE},
											{0xA0,0xCE},{0xA0,0xDE},{0xA0,0xEE},{0xA0,0xFE}};

//3uA
static unsigned char E_IrefTable_3[16][2] = { {0x20,0x0E},{0x20,0x1E},{0x20,0x2E},{0x20,0x3E},
											{0x20,0x4E},{0x20,0x5E},{0x20,0x6E},{0x20,0x7E},
											{0x20,0x8E},{0x20,0x9E},{0x20,0xAE},{0x20,0xBE},
											{0x20,0xCE},{0x20,0xDE},{0x20,0xEE},{0x20,0xFE}};

//4uA
static unsigned char E_IrefTable_4[16][2] = { {0xA0,0x0D},{0xA0,0x1D},{0xA0,0x2D},{0xA0,0x3D},
											{0xA0,0x4D},{0xA0,0x5D},{0xA0,0x6D},{0xA0,0x7D},
											{0xA0,0x8D},{0xA0,0x9D},{0xA0,0xAD},{0xA0,0xBD},
											{0xA0,0xCD},{0xA0,0xDD},{0xA0,0xED},{0xA0,0xFD}};

//5uA
static unsigned char E_IrefTable_5[16][2] = { {0x20,0x0D},{0x20,0x1D},{0x20,0x2D},{0x20,0x3D},
											{0x20,0x4D},{0x20,0x5D},{0x20,0x6D},{0x20,0x7D},
											{0x20,0x8D},{0x20,0x9D},{0x20,0xAD},{0x20,0xBD},
											{0x20,0xCD},{0x20,0xDD},{0x20,0xED},{0x20,0xFD}};

//6uA
static unsigned char E_IrefTable_6[16][2] = { {0xA0,0x0C},{0xA0,0x1C},{0xA0,0x2C},{0xA0,0x3C},
											{0xA0,0x4C},{0xA0,0x5C},{0xA0,0x6C},{0xA0,0x7C},
											{0xA0,0x8C},{0xA0,0x9C},{0xA0,0xAC},{0xA0,0xBC},
											{0xA0,0xCC},{0xA0,0xDC},{0xA0,0xEC},{0xA0,0xFC}};

//7uA
static unsigned char E_IrefTable_7[16][2] = { {0x20,0x0C},{0x20,0x1C},{0x20,0x2C},{0x20,0x3C},
											{0x20,0x4C},{0x20,0x5C},{0x20,0x6C},{0x20,0x7C},
											{0x20,0x8C},{0x20,0x9C},{0x20,0xAC},{0x20,0xBC},
											{0x20,0xCC},{0x20,0xDC},{0x20,0xEC},{0x20,0xFC}};
int HX_RX_NUM = 0;
int HX_TX_NUM = 0;
int HX_BT_NUM = 0;
int HX_X_RES = 0;
int HX_Y_RES = 0;
int HX_MAX_PT = 0;
unsigned char IC_CHECKSUM = 0;
unsigned char IC_TYPE = 0;
bool HX_XY_REVERSE = false;
bool upgrade_from_sd = false;
static int himax_ManualMode(int enter)
{
	uint8_t cmd[2] = {0};
	cmd[0] = enter;
	if ( i2c_himax_write(HX_REG_FLASH_MANUAL_MODE ,&cmd[0], 1, sizeof(cmd), DEFAULT_RETRY_CNT) < 0) {
		TS_LOG_ERR("%s: i2c access fail!\n", __func__);
		return I2C_ACCESS_FAIL;
	}
	return NO_ERR;
}

static int himax_FlashMode(int enter)
{
	uint8_t cmd[2] = {0};
	cmd[0] = enter;
	if ( i2c_himax_write( HX_REG_SET_FLASH_EN ,&cmd[0], 1, sizeof(cmd), DEFAULT_RETRY_CNT) < 0) {
		TS_LOG_ERR("%s: i2c access fail!\n", __func__);
		return I2C_ACCESS_FAIL;
	}
	return NO_ERR;
}
int himax_lock_flash(int enable)
{
	uint8_t cmd[5];
	if (i2c_himax_write(0xAA ,&cmd[0], 0,sizeof(cmd), 3) < 0) {
		TS_LOG_ERR("%s: i2c access failX!\n", __func__);
			return 0;
	}
	/* lock sequence start */
	cmd[0] = 0x01;cmd[1] = 0x00;cmd[2] = 0x06;
	if (i2c_himax_write(HX_CMD_FLASH_ENABLE ,&cmd[0], 3,sizeof(cmd), 3) < 0) {
		TS_LOG_ERR("%s: i2c access fail1!\n", __func__);
		return 0;
	}

	cmd[0] = 0x03;cmd[1] = 0x00;cmd[2] = 0x00;
	if (i2c_himax_write(HX_CMD_FLASH_SET_ADDRESS ,&cmd[0], 3,sizeof(cmd), 3) < 0) {
		TS_LOG_ERR("%s: i2c access fail2!\n", __func__);
		return 0;
	}
	if(enable!=0){
			cmd[0] = 0x63;cmd[1] = 0x02;cmd[2] = 0x70;cmd[3] = 0x03;
		}
	else{
			cmd[0] = 0x63;cmd[1] = 0x02;cmd[2] = 0x30;cmd[3] = 0x00;
		}
	if (i2c_himax_write(HX_CMD_FLASH_WRITE_REGISTER ,&cmd[0], 4, sizeof(cmd),3) < 0) {
		TS_LOG_ERR("%s: i2c access fail3!\n", __func__);
		return 0;
	}
      cmd[0] = 0x4A;
	if (i2c_himax_master_write(&cmd[0],1,sizeof(cmd), 3) < 0) {
		TS_LOG_ERR("%s: i2c access fail4!\n", __func__);
		return 0;
	}
	msleep(50);

	if (i2c_himax_write(0xA9 ,&cmd[0], 0,sizeof(cmd), 3) < 0) {
		TS_LOG_ERR("%s: i2c access fail5!\n", __func__);
		return 0;
	}
	return 0;
	/* lock sequence stop */
}
/*change 1~7 MA */
static void himax_changeIref(int selected_iref){

	unsigned char temp_iref[16][2] = {	{0x00,0x00},{0x00,0x00},{0x00,0x00},{0x00,0x00},
									{0x00,0x00},{0x00,0x00},{0x00,0x00},{0x00,0x00},
									{0x00,0x00},{0x00,0x00},{0x00,0x00},{0x00,0x00},
									{0x00,0x00},{0x00,0x00},{0x00,0x00},{0x00,0x00}};
	int i = 0;
	int j = 0;
	uint8_t cmd[10] = {0};
	if (i2c_himax_write(0xAA ,&cmd[0], 0,sizeof(cmd), 3) < 0) {
		TS_LOG_ERR("%s: i2c access fail write 0xAA!\n", __func__);
		return;
	}

	TS_LOG_INFO("%s: start to check iref,selected_iref number = %d\n",__func__,selected_iref);

	for(i=0; i<16; i++){
		for(j=0; j<2; j++){
			if(selected_iref == 1){
				temp_iref[i][j] = E_IrefTable_1[i][j];
			}
			else if(selected_iref == 2){
				temp_iref[i][j] = E_IrefTable_2[i][j];
			}
			else if(selected_iref == 3){
				temp_iref[i][j] = E_IrefTable_3[i][j];
			}
			else if(selected_iref == 4){
				temp_iref[i][j] = E_IrefTable_4[i][j];
			}
			else if(selected_iref == 5){
				temp_iref[i][j] = E_IrefTable_5[i][j];
			}
			else if(selected_iref == 6){
				temp_iref[i][j] = E_IrefTable_6[i][j];
			}
			else if(selected_iref == 7){
				temp_iref[i][j] = E_IrefTable_7[i][j];
			}
		}
	}

	if(!iref_found){

#if defined(HMX852xF)
		cmd[0] = 0x01;
		if (i2c_himax_write(HX_REG_SET_FLASH_EN ,&cmd[0], 1, sizeof(cmd), DEFAULT_RETRY_CNT) < 0) {
			TS_LOG_ERR("%s: i2c access fail!\n", __func__);
			return ;
		}

		cmd[0] = 0x00;
		if (i2c_himax_write(HX_REG_SET_FLASH_MANUAL_0 ,&cmd[0], 1, sizeof(cmd), DEFAULT_RETRY_CNT) < 0) {
			TS_LOG_ERR("%s: i2c access fail!\n", __func__);
			return ;
		}

		cmd[0] = 0x0A;
		if (i2c_himax_write(HX_REG_SET_FLASH_MANUAL_1 ,&cmd[0], 1, sizeof(cmd), DEFAULT_RETRY_CNT) < 0) {
			TS_LOG_ERR("%s: i2c access fail!\n", __func__);
			return ;
		}
#endif
#ifdef HMX852xE
		cmd[0] = 0x01; cmd[1] = 0x00; cmd[2] = 0x0A;
		if (i2c_himax_write(HX_REG_SET_FLASH_EN ,&cmd[0], 3, sizeof(cmd), DEFAULT_RETRY_CNT) < 0) {
			TS_LOG_ERR("%s: i2c access fail!\n", __func__);
			return ;
		}
#endif


		//Register 0x44
		cmd[0] = 0x00;
		cmd[1] = 0x00;
		cmd[2] = 0x00;
		if( i2c_himax_write(HX_REG_SET_FLASH_ADDR ,&cmd[0], 3, sizeof(cmd), DEFAULT_RETRY_CNT) < 0){
			TS_LOG_ERR("%s: i2c access fail!\n", __func__);
			return ;
		}

		//Register 0x46
		if( i2c_himax_write(HX_REG_FLASH_TRASFER ,&cmd[0], 0, sizeof(cmd), DEFAULT_RETRY_CNT) < 0){
			TS_LOG_ERR("%s: i2c access fail!\n", __func__);
			return ;
		}

		//Register 0x59
		if( i2c_himax_read(0x59, cmd, 4, sizeof(cmd), DEFAULT_RETRY_CNT) < 0){
			TS_LOG_ERR("%s: i2c access fail!\n", __func__);
			return ;
		}

		//find iref group , default is iref 3
		for (i = 0; i < 16; i++){
			if ((cmd[0] == temp_iref[i][0]) &&
					(cmd[1] == temp_iref[i][1])){
				iref_number = i;
				iref_found = true;
				break;
			}
		}
		if(!iref_found ){
			TS_LOG_ERR("%s: Can't find iref number!\n", __func__);
			return ;
		}
		else{
			TS_LOG_INFO("%s: iref_number=%d, cmd[0]=0x%x, cmd[1]=0x%x\n", __func__, iref_number, cmd[0], cmd[1]);
		}
	}
	msleep(HX_SLEEP_5MS);
#if defined(HMX852xF)
	cmd[0] = 0x01;
	if (i2c_himax_write(HX_REG_SET_FLASH_EN ,&cmd[0], 1, sizeof(cmd), DEFAULT_RETRY_CNT) < 0) {
		TS_LOG_ERR("%s: i2c access fail!\n", __func__);
		return ;
	}
	cmd[0] = 0x00;
	if (i2c_himax_write(HX_REG_SET_FLASH_MANUAL_0 ,&cmd[0], 1, sizeof(cmd), DEFAULT_RETRY_CNT) < 0) {
		TS_LOG_ERR("%s: i2c access fail!\n", __func__);
		return ;
	}
	cmd[0] = 0x06;
	if (i2c_himax_write(HX_REG_SET_FLASH_MANUAL_1 ,&cmd[0], 1, sizeof(cmd), DEFAULT_RETRY_CNT) < 0) {
		TS_LOG_ERR("%s: i2c access fail!\n", __func__);
		return ;
	}
#endif
#ifdef HMX852xE
	cmd[0] = 0x01; cmd[1] = 0x00; cmd[2] = 0x06;
		if (i2c_himax_write(HX_REG_SET_FLASH_EN ,&cmd[0], 3, sizeof(cmd), DEFAULT_RETRY_CNT) < 0) {
			TS_LOG_ERR("%s: i2c access fail!\n", __func__);
			return ;
		}
#endif
	//Register 0x44
	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	if( i2c_himax_write(HX_REG_SET_FLASH_ADDR ,&cmd[0], 3, sizeof(cmd), DEFAULT_RETRY_CNT) < 0){
		TS_LOG_ERR("%s: i2c access fail!\n", __func__);
		return ;
	}
	//Register 0x45
	cmd[0] = temp_iref[iref_number][0];
	cmd[1] = temp_iref[iref_number][1];
	cmd[2] = 0x17;
	cmd[3] = 0x28;
	if( i2c_himax_write(HX_REG_SET_FLASH_DATA ,&cmd[0], 4, sizeof(cmd), DEFAULT_RETRY_CNT) < 0){
		TS_LOG_ERR("%s: i2c access fail!\n", __func__);
		return ;
	}
	//Register 0x4A
	if( i2c_himax_write(HX_REG_FLASH_BPW_START ,&cmd[0], 0, sizeof(cmd), DEFAULT_RETRY_CNT) < 0){
		TS_LOG_ERR("%s: i2c access fail!\n", __func__);
		return ;
	}
#if defined(HMX852xF)
	cmd[0] = 0x01;
	if (i2c_himax_write(HX_REG_SET_FLASH_EN ,&cmd[0], 1, sizeof(cmd), DEFAULT_RETRY_CNT) < 0) {
		TS_LOG_ERR("%s: i2c access fail!\n", __func__);
		return ;
	}

	cmd[0] = 0x00;
	if (i2c_himax_write(HX_REG_SET_FLASH_MANUAL_0 ,&cmd[0], 1, sizeof(cmd), DEFAULT_RETRY_CNT) < 0) {
		TS_LOG_ERR("%s: i2c access fail!\n", __func__);
		return ;
	}
	cmd[0] = 0x0A;
	if (i2c_himax_write(HX_REG_SET_FLASH_MANUAL_1 ,&cmd[0], 1, sizeof(cmd), DEFAULT_RETRY_CNT) < 0) {
		TS_LOG_ERR("%s: i2c access fail!\n", __func__);
		return ;
	}
#endif
#ifdef HMX852xE
		cmd[0] = 0x01; cmd[1] = 0x00; cmd[2] = 0x0A;
		if (i2c_himax_write(HX_REG_SET_FLASH_EN ,&cmd[0], 3, sizeof(cmd), DEFAULT_RETRY_CNT) < 0) {
			TS_LOG_ERR("%s: i2c access fail!write 0x43\n", __func__);
			return ;
		}
#endif

	//Register 0x44
	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	if( i2c_himax_write(HX_REG_SET_FLASH_ADDR ,&cmd[0], 3, sizeof(cmd), DEFAULT_RETRY_CNT) < 0){
		TS_LOG_ERR("%s: i2c access fail!wrtite 0x44\n", __func__);
		return ;
	}
	//Register 0x46
	if( i2c_himax_write(HX_REG_FLASH_TRASFER ,&cmd[0], 0, sizeof(cmd), DEFAULT_RETRY_CNT) < 0){
		TS_LOG_ERR("%s: i2c access fail!write 0x46\n", __func__);
		return ;
	}
	//Register 0x59
	if( i2c_himax_read(0x59, cmd, 4, sizeof(cmd), 3) < 0){
		TS_LOG_ERR("%s: i2c access fail! read 0x59\n", __func__);
		return ;
	}
	TS_LOG_INFO("%s:cmd[0]=%d,cmd[1]=%d,temp_iref_1=%d,temp_iref_2=%d\n",__func__, cmd[0], cmd[1], temp_iref[iref_number][0], temp_iref[iref_number][1]);

	if(cmd[0] != temp_iref[iref_number][0] || cmd[1] != temp_iref[iref_number][1]){
		TS_LOG_ERR("%s: IREF Read Back is not match.\n", __func__);
		TS_LOG_ERR("%s: Iref [0]=%d,[1]=%d\n", __func__,cmd[0],cmd[1]);
	}
	else{
		TS_LOG_INFO("%s: IREF Pass",__func__);
	}
#ifdef HMX852xE
	if (i2c_himax_write(0xA9 ,&cmd[0], 0,sizeof(cmd), 3) < 0) {
		TS_LOG_ERR("%s: i2c access fail write 0xA9!\n", __func__);
		return;
	}
#endif
}

uint8_t himax_calculateChecksum(bool change_iref)
{
	int iref_flag = 0;
	uint8_t cmd[10];
	memset(cmd, 0x00, sizeof(cmd));
	if( i2c_himax_write(HX_CMD_TSSLPOUT ,&cmd[0], 0,sizeof(cmd), DEFAULT_RETRY_CNT) < 0)
	{
		TS_LOG_ERR("%s: i2c access fail1!\n", __func__);
		return 0;
	}
	msleep(120);
	while(true){

		if(change_iref)
		{
			if(iref_flag == 0){
				himax_changeIref(2); //iref 2
			}
			else if(iref_flag == 1){
				himax_changeIref(5); //iref 5
			}
			else if(iref_flag == 2){
				himax_changeIref(1); //iref 1
			}
			else{
				goto CHECK_FAIL;
			}
			iref_flag ++;
		}
		cmd[0] = 0x00;
		cmd[1] = 0x04;
		cmd[2] = 0x0A;
		cmd[3] = 0x02;
		if (i2c_himax_write(0xED ,&cmd[0], 4, sizeof(cmd),DEFAULT_RETRY_CNT) < 0) {
			TS_LOG_ERR("%s: i2c access fail2!\n", __func__);
			return 0;
		}
		//Enable Flash
		cmd[0] = 0x01;
		cmd[1] = 0x00;
		cmd[2] = 0x02;
		if (i2c_himax_write(HX_CMD_FLASH_ENABLE ,&cmd[0], 3,sizeof(cmd), DEFAULT_RETRY_CNT) < 0) {
			TS_LOG_ERR("%s: i2c access fail3!\n", __func__);
			return 0;
		}
		cmd[0] = 0x05;
		if (i2c_himax_write(0xD2 ,&cmd[0], 1,sizeof(cmd), DEFAULT_RETRY_CNT) < 0) {
			TS_LOG_ERR("%s: i2c access fail4!\n", __func__);
			return 0;
		}
		cmd[0] = 0x01;
		if (i2c_himax_write(0x53 ,&cmd[0], 1, sizeof(cmd),DEFAULT_RETRY_CNT) < 0) {
			TS_LOG_ERR("%s: i2c access fail5!\n", __func__);
			return 0;
		}
		msleep(200);
		if (i2c_himax_read(0xAD, cmd, 4,sizeof(cmd), DEFAULT_RETRY_CNT) < 0) {
			TS_LOG_ERR("%s: i2c access fail6!\n", __func__);
			return -1;
		}
		TS_LOG_INFO("%s 0xAD[0,1,2,3] = %d,%d,%d,%d \n",__func__,cmd[0],cmd[1],cmd[2],cmd[3]);
		if (cmd[0] == 0 && cmd[1] == 0 && cmd[2] == 0 && cmd[3] == 0 ) {
			himax_FlashMode(0);
			goto CHECK_PASS;
		} else {
			himax_FlashMode(0);
			goto CHECK_FAIL;
		}
		CHECK_PASS:
			if(change_iref)
			{
				if(iref_flag < 3){
					continue;
				}
				else {
					return 1;
				}
			}
			else
			{
				return 1;
			}

		CHECK_FAIL:
			return 0;
	}
	return 0;
}

#define FLASH_56K_column 	14336 //(1024 x 56) /4
#define FLASH_63K_column 	16128 //(1024 x 63) /4
#define FLASH_56K_addr 		57344 //(1024 x 56)
#define FLASH_63K_addr 		64512 //(1024 x 63)

/*fw upgrade flow*/
int fts_ctpm_fw_upgrade_with_fs(const unsigned char *fw, int len, bool change_iref)
{
	unsigned char* ImageBuffer = fw;
	int fullFileLength = len;
	int i;
	uint8_t cmd[5], last_byte, prePage;
	int FileLength;
	uint8_t checksumResult = 0;

	FileLength = fullFileLength;
	HX_UPDATE_FLAG = 1;
	if ( i2c_himax_write(HX_CMD_TSSLPOUT ,&cmd[0], 0,sizeof(cmd), DEFAULT_RETRY_CNT) < 0)
	{
		TS_LOG_ERR("%s: i2c access fail0!\n", __func__);
		return 0;
	}

	msleep(120);

	himax_lock_flash(0);

	cmd[0] = 0x05;cmd[1] = 0x00;cmd[2] = 0x02;
	if ( i2c_himax_write(HX_CMD_FLASH_ENABLE ,&cmd[0], 3, sizeof(cmd),DEFAULT_RETRY_CNT) < 0)
	{
		TS_LOG_ERR("%s: i2c access fail1!\n", __func__);
		return 0;
	}

	if ( i2c_himax_write(0x4F ,&cmd[0], 0,sizeof(cmd), DEFAULT_RETRY_CNT) < 0)
	{
		TS_LOG_ERR("%s: i2c access fail2!\n", __func__);
		return 0;
	}
	msleep(50);

	himax_ManualMode(1);
	himax_FlashMode(1);

	FileLength = (FileLength + 3) / 4;
	for (i = 0, prePage = 0; i < FileLength; i++)
	{
		last_byte = 0;
		cmd[0] = i & 0x1F;
		if (cmd[0] == 0x1F || i == FileLength - 1)
		{
			last_byte = 1;
		}
		cmd[1] = (i >> 5) & 0x1F;
		cmd[2] = (i >> 10) & 0x1F;
		if ( i2c_himax_write(HX_CMD_FLASH_SET_ADDRESS ,&cmd[0], 3,sizeof(cmd), DEFAULT_RETRY_CNT) < 0)
		{
			TS_LOG_ERR("%s: i2c access fail3!\n", __func__);
			return 0;
		}

		if (prePage != cmd[1] || i == 0)
		{
			prePage = cmd[1];
			cmd[0] = 0x01;cmd[1] = 0x09;//cmd[2] = 0x02;
			if ( i2c_himax_write(HX_CMD_FLASH_ENABLE ,&cmd[0], 2,sizeof(cmd), DEFAULT_RETRY_CNT) < 0)
			{
				TS_LOG_ERR("%s: i2c access fail4!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;cmd[1] = 0x0D;//cmd[2] = 0x02;
			if ( i2c_himax_write(HX_CMD_FLASH_ENABLE ,&cmd[0], 2,sizeof(cmd), DEFAULT_RETRY_CNT) < 0)
			{
				TS_LOG_ERR("%s: i2c access fail5!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;cmd[1] = 0x09;//cmd[2] = 0x02;
			if ( i2c_himax_write(HX_CMD_FLASH_ENABLE ,&cmd[0], 2,sizeof(cmd), DEFAULT_RETRY_CNT) < 0)
			{
				TS_LOG_ERR("%s: i2c access fai6l!\n", __func__);
				return 0;
			}
		}

		memcpy(&cmd[0], &ImageBuffer[4*i], 4);
		if ( i2c_himax_write(HX_CMD_FLASH_WRITE_REGISTER ,&cmd[0], 4, sizeof(cmd),DEFAULT_RETRY_CNT) < 0)
		{
			TS_LOG_ERR("%s: i2c access fail7!\n", __func__);
			return 0;
		}

		cmd[0] = 0x01;cmd[1] = 0x0D;//cmd[2] = 0x02;
		if ( i2c_himax_write(HX_CMD_FLASH_ENABLE ,&cmd[0], 2,sizeof(cmd), DEFAULT_RETRY_CNT) < 0)
		{
			TS_LOG_ERR("%s: i2c access fail8!\n", __func__);
			return 0;
		}

		cmd[0] = 0x01;cmd[1] = 0x09;//cmd[2] = 0x02;
		if ( i2c_himax_write(HX_CMD_FLASH_ENABLE ,&cmd[0], 2,sizeof(cmd), DEFAULT_RETRY_CNT) < 0)
		{
			TS_LOG_ERR("%s: i2c access faill9!\n", __func__);
			return 0;
		}

		if (last_byte == 1)
		{
			cmd[0] = 0x01;cmd[1] = 0x01;//cmd[2] = 0x02;
			if ( i2c_himax_write(HX_CMD_FLASH_ENABLE ,&cmd[0], 2,sizeof(cmd), DEFAULT_RETRY_CNT) < 0)
			{
				TS_LOG_ERR("%s: i2c access fail10!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;cmd[1] = 0x05;//cmd[2] = 0x02;
			if ( i2c_himax_write(HX_CMD_FLASH_ENABLE ,&cmd[0], 2, sizeof(cmd),DEFAULT_RETRY_CNT) < 0)
			{
				TS_LOG_ERR("%s: i2c access fail11!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;cmd[1] = 0x01;//cmd[2] = 0x02;
			if ( i2c_himax_write(HX_CMD_FLASH_ENABLE ,&cmd[0], 2,sizeof(cmd), DEFAULT_RETRY_CNT) < 0)
			{
				TS_LOG_ERR("%s: i2c access fail12!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;cmd[1] = 0x00;//cmd[2] = 0x02;
			if ( i2c_himax_write(HX_CMD_FLASH_ENABLE ,&cmd[0], 2,sizeof(cmd), DEFAULT_RETRY_CNT) < 0)
			{
				TS_LOG_ERR("%s: i2c access fail13!\n", __func__);
				return 0;
			}

			msleep(10);
			if (i == (FileLength - 1))
			{
				himax_FlashMode(0);
				himax_ManualMode(0);
				checksumResult = himax_calculateChecksum(change_iref);//
				himax_lock_flash(1);
				if (checksumResult) //Success
				{
					TS_LOG_INFO("%s: checksumResult success!\n", __func__);
					HX_UPDATE_FLAG = 0;
					return 1;
				}
				else //Fail
				{
					HX_UPDATE_FLAG = 0;
					TS_LOG_ERR("%s: checksumResult fail!\n", __func__);
					return 0;
				}
			}
		}
	}
	HX_UPDATE_FLAG = 0;
	return 0;
}
static int check_firmware_version(const struct firmware *fw)
{
	if(NULL == fw) {
		return FW_NO_EXIST;
	}
	if(!fw->data)
	{
		TS_LOG_INFO(" himax ======== fw requst fail =====");
		return FW_NO_EXIST;
	}
	TS_LOG_INFO("himax IMAGE FW_VER=%x,%x.\n", fw->data[FW_VER_MAJ_FLASH_ADDR], fw->data[FW_VER_MIN_FLASH_ADDR]);
	TS_LOG_INFO("himax IMAGE CFG_VER=%x.\n", fw->data[FW_CFG_VER_FLASH_ADDR]);

	TS_LOG_INFO("himax curr FW_VER=%x,%x.\n", g_himax_ts_data->vendor_fw_ver_H, g_himax_ts_data->vendor_fw_ver_L);
	TS_LOG_INFO("himax curr CFG_VER=%x.\n", g_himax_ts_data->vendor_config_ver);

	if (( g_himax_ts_data->vendor_fw_ver_H < fw->data[FW_VER_MAJ_FLASH_ADDR] )
		|| ( g_himax_ts_data->vendor_fw_ver_L < fw->data[FW_VER_MIN_FLASH_ADDR] )
		|| ( g_himax_ts_data->vendor_config_ver < fw->data[FW_CFG_VER_FLASH_ADDR] ))
	{
		TS_LOG_INFO("firmware is lower, must upgrade.\n");
		return FW_NEED_TO_UPDATE;
	}
	else {
		TS_LOG_INFO("firmware not lower.\n");
		return FW_NO_NEED_TO_UPDATE;
	}
}

static int check_need_firmware_upgrade(const struct firmware *fw)
{
	/*first, check firmware version.*/
	int retval = 0;
	if ( himax_calculateChecksum(false) == 0 )
	{
		TS_LOG_INFO("checksum failed, must upgrade.\n");
		#ifdef CONFIG_HUAWEI_DSM
			hmx_tp_dsm_info.UPDATE_status = FWU_FW_CRC_ERROR;
			hmx_tp_report_dsm_err(DSM_TP_FWUPDATE_ERROR_NO, retval);
		#endif
		return CAL_CHECKSUM_RUN_FAIL;
	}
	else {
		TS_LOG_INFO("checksum success.\n");
	}
	if (check_firmware_version(fw) || upgrade_from_sd==true)
	{
		retval = himax_check_update_firmware_flag();
		if(0 == retval){
			TS_LOG_INFO("now is factory test mode, not update firmware\n");
			return 0;
		}
		if(upgrade_from_sd==true) upgrade_from_sd = false;
		return FW_NEED_TO_UPDATE;
	}

	return FW_NO_NEED_TO_UPDATE;
}

static void  firmware_update(const struct firmware *fw)
{
	int retval = -1;
	int fullFileLength = 0;

	if(!fw->data||!fw->size)
	{
		TS_LOG_INFO(" himax fw requst fail\n");
		return;
	}
	g_himax_ts_data->firmware_updating = true;
	#ifdef HX_CHIP_STATUS_MONITOR
				if(himax_chip_monitor_hand_shaking()!=NO_ERR)
				{
					TS_LOG_INFO("himax_handshaking timeout\n");
					return;
				}
				HX_CHIP_POLLING_COUNT = 0;
				cancel_delayed_work_sync(&g_himax_ts_data->himax_chip_monitor);
	#endif
	himax_HW_reset(HX_LOADCONFIG_DISABLE,HX_INT_EN);

	TS_LOG_INFO("himax fw size =%u \n",(unsigned int )fw->size);

	fullFileLength=(unsigned int) fw->size;
	if (check_need_firmware_upgrade(fw))
	{
			wake_lock(&g_himax_ts_data->ts_flash_wake_lock);
			/*#ifdef HX_CHIP_STATUS_MONITOR
				if(himax_chip_monitor_hand_shaking()!=NO_ERR)
				{
					TS_LOG_INFO("himax_handshaking timeout\n");
					return;
				}
				HX_CHIP_POLLING_COUNT = 0;
				cancel_delayed_work_sync(&g_himax_ts_data->himax_chip_monitor);
			#endif*/
			TS_LOG_INFO("himax flash write start");
			retval = fts_ctpm_fw_upgrade_with_fs(fw->data,fullFileLength,true);
			if( retval == 0 )
			{
				TS_LOG_ERR("%s:himax  TP upgrade error\n", __func__);
				TS_LOG_INFO("himax  update function end\n");
				#ifdef CONFIG_HUAWEI_DSM
					hmx_tp_dsm_info.UPDATE_status = FWU_FW_WRITE_ERROR;
					hmx_tp_report_dsm_err(DSM_TP_FWUPDATE_ERROR_NO, retval);
				#endif
			}
			else
			{

				TS_LOG_INFO("%s: himax TP upgrade OK\n", __func__);
#ifdef HX_UPDATE_WITH_BIN_BUILDIN
				g_himax_ts_data->vendor_fw_ver_H = fw->data[FW_VER_MAJ_FLASH_ADDR];
				g_himax_ts_data->vendor_fw_ver_L = fw->data[FW_VER_MIN_FLASH_ADDR];
				g_himax_ts_data->vendor_config_ver = fw->data[FW_CFG_VER_FLASH_ADDR];

				TS_LOG_INFO("himax upgraded IMAGE FW_VER=%x,%x.\n",fw->data[FW_VER_MAJ_FLASH_ADDR],fw->data[FW_VER_MIN_FLASH_ADDR]);
				TS_LOG_INFO("himax upgraded IMAGE CFG_VER=%x.\n",fw->data[FW_CFG_VER_FLASH_ADDR]);
#endif
			}
			TS_LOG_INFO("himax flash write end");
			wake_unlock(&g_himax_ts_data->ts_flash_wake_lock);

/*#ifdef HX_CHIP_STATUS_MONITOR
				HX_CHIP_POLLING_COUNT = 0;
				queue_delayed_work(g_himax_ts_data->himax_chip_monitor_wq, &g_himax_ts_data->himax_chip_monitor, HX_POLLING_TIMES*HZ);
#endif*/
			TS_LOG_INFO("himax i_update function end ");
	}
	else {
		TS_LOG_INFO("himax don't need upgrade firmware");
	}
	himax_HW_reset(HX_LOADCONFIG_EN,HX_INT_EN);
	#ifdef HX_CHIP_STATUS_MONITOR
		HX_CHIP_POLLING_COUNT = 0;
		queue_delayed_work(g_himax_ts_data->himax_chip_monitor_wq, &g_himax_ts_data->himax_chip_monitor, HX_POLLING_TIMES*HZ);
	#endif
	g_himax_ts_data->firmware_updating = false;
	return ;
}

static void himax_set_app_info(struct himax_ts_data *ts)
{
	char touch_info[40] = {0};
	sprintf(touch_info,
				"HX8527-E [vendor]%s [FW]0x%x",
				ts->vendor_name,
				ts->vendor_config_ver);
#ifdef CONFIG_APP_INFO
	app_info_set("touch_panel", touch_info);
#endif
}

int himax_loadSensorConfig(void)
{
	uint8_t data[12] = {0};
	int retval = 0;
#if defined(HMX852xF)
	//set clock status
	data[0] = HX_REG_SET_CLK_ADDR;
	data[1] = 0x06;
	data[2] = 0x03;
	retval = i2c_himax_master_write(&data[0],SET_CLK_DATA_LEN, sizeof(data),DEFAULT_RETRY_CNT);
	if(retval <0) {
		TS_LOG_ERR("himax_loadSensorConfig write err %d!\n",__LINE__);
		return retval;
	}
	mdelay(HX_SLEEP_1MS);
	//set crystal status
	data[0] = HX_REG_SET_OSC_4_PUMP;
	data[1] = 0x11;
	data[2] = 0x00;
	retval = i2c_himax_master_write(&data[0],SET_CRYSTAL_DATA_LEN, sizeof(data),DEFAULT_RETRY_CNT);
	if(retval <0) {
		TS_LOG_ERR("himax_loadSensorConfig write err %d!\n",__LINE__);
		return retval;
	}
	mdelay(HX_SLEEP_1MS);
#endif

#ifdef HX_ESD_WORKAROUND
	/*Check R36 to check IC Status*/
	if(g_check_r36h_flag != 0){
		retval = i2c_himax_read(HX_REG_RELOAD_FLAG, data, R36H_REG_DATA_LEN, sizeof(data), MAX_RETRY_CNT);
		if(retval <0) {
			TS_LOG_ERR("himax_loadSensorConfig read err %d!\n",__LINE__);
			return retval;
		}
		if(data[0] != 0x0F || data[1] != 0x53)//0x0F and 0x53 are correct status value
		{
			/*IC is abnormal*/
			TS_LOG_ERR("[HimaxError] %s R36 Fail : R36[0]=%d,R36[1]=%d,R36 Counter=%d \n",__func__,data[0],data[1],ESD_R36_FAIL);
			return R36H_FAIL;
		}
	}
#endif
	/*retval = himax_power_on_initCMD();
	if(retval <0) {
		TS_LOG_ERR("himax_power_on_initCMD fail %d!\n",__LINE__);
		return retval;
	}*/
	himax_get_information();
	TS_LOG_INFO("%s: initialization complete\n", __func__);

	return LOAD_SENSORCONFIG_OK;
}

static int himax_file_open_firmware(uint8_t *file_path, uint8_t *databuf,
                int *file_size)
{
    int retval = 0;
    unsigned int file_len = 0;
    struct file *filp = NULL;
    struct inode *inode = NULL;
    mm_segment_t oldfs;

    if(file_path == NULL || databuf == NULL){
        TS_LOG_ERR("%s: path || buf is NULL.\n", __func__);
        return -EINVAL;
    }

    TS_LOG_INFO("%s: path = %s.\n",__func__, file_path);

    // open file
    oldfs = get_fs();
	/*lint -save -e* */
	set_fs(KERNEL_DS);
	/*lint -restore*/

    filp = filp_open(file_path, O_RDONLY, S_IRUSR);
    if (IS_ERR(filp)){
        TS_LOG_ERR("%s: open %s error.\n", __func__, file_path);
        retval = -EIO;
        goto err;
    }

    if (filp->f_op == NULL) {
        TS_LOG_ERR("%s: File Operation Method Error\n", __func__);
        retval = -EINVAL;
        goto exit;
    }

    inode = filp->f_path.dentry->d_inode;
    if (inode == NULL) {
        TS_LOG_ERR("%s: Get inode from filp failed\n", __func__);
        retval = -EINVAL;
        goto exit;
    }

    //Get file size
    file_len = i_size_read(inode->i_mapping->host);
    if (file_len == 0){
        TS_LOG_ERR("%s: file size error,file_len = %d\n",  __func__, file_len);
        retval = -EINVAL;
        goto exit;
    }

    // read image data to kernel */
    if (filp->f_op->read(filp, databuf, file_len, &filp->f_pos) != file_len) {
        TS_LOG_ERR("%s: file read error.\n", __func__);
        retval = -EINVAL;
        goto exit;
    }

    *file_size = file_len;

exit:
    filp_close(filp, NULL);
err:
    set_fs(oldfs);
    return retval;
}


int himax_check_update_firmware_flag(void)
{
	int retval = 0;
	int file_size = 0;
	int update_flag = 0;
	uint8_t *tmp = NULL;
	uint8_t *file_data = NULL;
	uint8_t *file_path = BOOT_UPDATE_FIRMWARE_FLAG_FILENAME;

	file_data = kzalloc(128, GFP_KERNEL);
	if(file_data == NULL){
       TS_LOG_ERR("%s: kzalloc error.\n", __func__);
       return -EINVAL;
	}
	retval = himax_file_open_firmware(file_path, file_data, &file_size);
	if (retval != 0) {
        TS_LOG_ERR("%s, file_open error, code = %d\n", __func__, retval);
        goto exit;
	}
	tmp = strstr(file_data, BOOT_UPDATE_FIRMWARE_FLAG);
	if (tmp == NULL) {
		TS_LOG_ERR( "%s not found\n", BOOT_UPDATE_FIRMWARE_FLAG);
		retval = -1;
		goto exit;
	} else {
		tmp = tmp + strlen(BOOT_UPDATE_FIRMWARE_FLAG);
		sscanf(tmp, "%d", &update_flag);
	}
	if (update_flag == 1) {
		retval = 0;
		TS_LOG_INFO("%s ,check success, flag = %d\n", __func__, update_flag);
		goto exit;
	} else {
		retval = -1;
		TS_LOG_INFO("%s ,check failed, flag = %d\n", __func__, update_flag);
		goto exit;
	}
exit:
	kfree(file_data);
	return retval;
}
static void himax_get_fw_name(char *file_name)
{
	char *firmware_form = ".bin";
	char *ic_name = "himax";
	char   joint_chr = '_';
	strncat(file_name,ic_name,strlen(ic_name)+1);
	strncat(file_name,&joint_chr,1);
	strncat(file_name,g_himax_ts_data->vendor_name,strlen(g_himax_ts_data->vendor_name)+1);
	strncat(file_name,firmware_form,strlen(firmware_form)+1);
	TS_LOG_INFO("%s, firmware name: %s\n",__func__,file_name);
}
int himax_fw_update_boot(char *file_name)
{
	int err = NO_ERR;
	const  struct firmware *fw_entry = NULL;
	char firmware_name[64] = "";

	TS_LOG_INFO("%s: enter!\n", __func__);
	TS_LOG_INFO("himax start to request firmware  %s", file_name);
	#ifdef HX_CHIP_STATUS_MONITOR
		HX_CHIP_POLLING_COUNT = 0;
		cancel_delayed_work_sync(&g_himax_ts_data->himax_chip_monitor);
	#endif
	himax_get_fw_name(file_name);
	snprintf(firmware_name, PAGE_SIZE, "%s", file_name);
	TS_LOG_INFO("%s, request_firmware name: %s\n",__func__,firmware_name);
	err = request_firmware(&fw_entry, firmware_name, &g_himax_ts_data->tskit_himax_data->ts_platform_data->ts_dev->dev);
	if (err < 0) {
		TS_LOG_ERR("himax %s %d: Fail request firmware %s, retval = %d\n",
					__func__, __LINE__, file_name, err);
		#ifdef CONFIG_HUAWEI_DSM
			hmx_tp_dsm_info.UPDATE_status = FWU_REQUEST_FW_FAIL;
			hmx_tp_report_dsm_err(DSM_TP_FWUPDATE_ERROR_NO, err);
		#endif
		goto err_request_firmware;
	}
	firmware_update(fw_entry);
	release_firmware(fw_entry);
	TS_LOG_INFO("%s: end!\n", __func__);
err_request_firmware:
	himax_set_app_info(g_himax_ts_data);
	return err;
}

int himax_fw_update_sd(void)
{
	int retval = 0;
	char firmware_name[100] = "";
	const struct firmware *fw_entry = NULL;
	upgrade_from_sd = true;
	TS_LOG_INFO("%s: enter!\n", __func__);
	sprintf(firmware_name, HX_FW_NAME);

	TS_LOG_INFO("himax start to request firmware %s", firmware_name);
	retval = request_firmware(&fw_entry, firmware_name, &g_himax_ts_data->tskit_himax_data->ts_platform_data->client->dev);
	if (retval < 0) {
		TS_LOG_ERR("himax %s %d: Fail request firmware %s, retval = %d\n",
		__func__, __LINE__, firmware_name, retval);
#ifdef CONFIG_HUAWEI_DSM
		hmx_tp_dsm_info.UPDATE_status = FWU_REQUEST_FW_FAIL;
		hmx_tp_report_dsm_err(DSM_TP_FWUPDATE_ERROR_NO, retval);
#endif
		return retval;
	}

	firmware_update(fw_entry);

	release_firmware(fw_entry);

	himax_set_app_info(g_himax_ts_data);
	//PowerOnSeq(g_himax_ts_data);

	TS_LOG_INFO("%s: end!\n", __func__);
	return retval;
}
