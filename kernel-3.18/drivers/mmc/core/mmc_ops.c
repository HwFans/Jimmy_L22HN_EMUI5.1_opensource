/*
 *  linux/drivers/mmc/core/mmc_ops.h
 *
 *  Copyright 2006-2007 Pierre Ossman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/slab.h>
#include <linux/export.h>
#include <linux/types.h>
#include <linux/scatterlist.h>

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>

#ifdef CONFIG_MMC_PASSWORDS
#include "lock.h"
#endif

#include "core.h"
#include "host.h"
#include "mmc_ops.h"
#ifdef CONFIG_HUAWEI_SDCARD_DSM
#include <linux/mmc/dsm_sdcard.h>
#endif
#ifdef CONFIG_HUAWEI_EMMC_DSM
#include <linux/mmc/dsm_emmc.h>
extern u64 device_index;
#endif
#define MMC_OPS_TIMEOUT_MS	(10 * 60 * 1000) /* 10 minute timeout */

static const u8 tuning_blk_pattern_4bit[] = {
	0xff, 0x0f, 0xff, 0x00, 0xff, 0xcc, 0xc3, 0xcc,
	0xc3, 0x3c, 0xcc, 0xff, 0xfe, 0xff, 0xfe, 0xef,
	0xff, 0xdf, 0xff, 0xdd, 0xff, 0xfb, 0xff, 0xfb,
	0xbf, 0xff, 0x7f, 0xff, 0x77, 0xf7, 0xbd, 0xef,
	0xff, 0xf0, 0xff, 0xf0, 0x0f, 0xfc, 0xcc, 0x3c,
	0xcc, 0x33, 0xcc, 0xcf, 0xff, 0xef, 0xff, 0xee,
	0xff, 0xfd, 0xff, 0xfd, 0xdf, 0xff, 0xbf, 0xff,
	0xbb, 0xff, 0xf7, 0xff, 0xf7, 0x7f, 0x7b, 0xde,
};

static const u8 tuning_blk_pattern_8bit[] = {
	0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00,
	0xff, 0xff, 0xcc, 0xcc, 0xcc, 0x33, 0xcc, 0xcc,
	0xcc, 0x33, 0x33, 0xcc, 0xcc, 0xcc, 0xff, 0xff,
	0xff, 0xee, 0xff, 0xff, 0xff, 0xee, 0xee, 0xff,
	0xff, 0xff, 0xdd, 0xff, 0xff, 0xff, 0xdd, 0xdd,
	0xff, 0xff, 0xff, 0xbb, 0xff, 0xff, 0xff, 0xbb,
	0xbb, 0xff, 0xff, 0xff, 0x77, 0xff, 0xff, 0xff,
	0x77, 0x77, 0xff, 0x77, 0xbb, 0xdd, 0xee, 0xff,
	0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00,
	0x00, 0xff, 0xff, 0xcc, 0xcc, 0xcc, 0x33, 0xcc,
	0xcc, 0xcc, 0x33, 0x33, 0xcc, 0xcc, 0xcc, 0xff,
	0xff, 0xff, 0xee, 0xff, 0xff, 0xff, 0xee, 0xee,
	0xff, 0xff, 0xff, 0xdd, 0xff, 0xff, 0xff, 0xdd,
	0xdd, 0xff, 0xff, 0xff, 0xbb, 0xff, 0xff, 0xff,
	0xbb, 0xbb, 0xff, 0xff, 0xff, 0x77, 0xff, 0xff,
	0xff, 0x77, 0x77, 0xff, 0x77, 0xbb, 0xdd, 0xee,
};

static inline int __mmc_send_status(struct mmc_card *card, u32 *status,
				    bool ignore_crc)
{
	int err;
	struct mmc_command cmd = {0};

	BUG_ON(!card);
	BUG_ON(!card->host);

	cmd.opcode = MMC_SEND_STATUS;
	if (!mmc_host_is_spi(card->host))
		cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;
	if (ignore_crc)
		cmd.flags &= ~MMC_RSP_CRC;

	err = mmc_wait_for_cmd(card->host, &cmd, MMC_CMD_RETRIES);
	if (err)
		return err;

	/* NOTE: callers are required to understand the difference
	 * between "native" and SPI format status words!
	 */
	if (status)
		*status = cmd.resp[0];

	return 0;
}

int mmc_send_status(struct mmc_card *card, u32 *status)
{
	return __mmc_send_status(card, status, false);
}

static int _mmc_select_card(struct mmc_host *host, struct mmc_card *card)
{
	int err;
	struct mmc_command cmd = {0};
#ifdef CONFIG_HUAWEI_SDCARD_DSM
	char *log_buff;
	int   buff_len;
#endif
	BUG_ON(!host);

	cmd.opcode = MMC_SELECT_CARD;

	if (card) {
		cmd.arg = card->rca << 16;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
	} else {
		cmd.arg = 0;
		cmd.flags = MMC_RSP_NONE | MMC_CMD_AC;
	}

	err = mmc_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
#ifdef CONFIG_HUAWEI_SDCARD_DSM
	if(!strcmp(mmc_hostname(host), "mmc1"))
	{
		dsm_sdcard_cmd_logs[DSM_SDCARD_CMD7].value = cmd.resp[0];
	}

	if (err)
	{
		if(-ENOMEDIUM != err && -ETIMEDOUT != err
		&& !strcmp(mmc_hostname(host), "mmc1") && !dsm_client_ocuppy(sdcard_dclient))
		{
			log_buff = dsm_sdcard_get_log(DSM_SDCARD_CMD7,err);
			buff_len = strlen(log_buff);
			dsm_client_copy(sdcard_dclient,log_buff,buff_len + 1);
			dsm_client_notify(sdcard_dclient, DSM_SDCARD_CMD7_RESP_ERR);
		}
		return err;
	}
#else
	if (err)
		return err;
#endif
	return 0;
}

int mmc_select_card(struct mmc_card *card)
{
	BUG_ON(!card);

	return _mmc_select_card(card->host, card);
}

int mmc_deselect_cards(struct mmc_host *host)
{
	return _mmc_select_card(host, NULL);
}

/*
 * Write the value specified in the device tree or board code into the optional
 * 16 bit Driver Stage Register. This can be used to tune raise/fall times and
 * drive strength of the DAT and CMD outputs. The actual meaning of a given
 * value is hardware dependant.
 * The presence of the DSR register can be determined from the CSD register,
 * bit 76.
 */
int mmc_set_dsr(struct mmc_host *host)
{
	struct mmc_command cmd = {0};

	cmd.opcode = MMC_SET_DSR;

	cmd.arg = (host->dsr << 16) | 0xffff;
	cmd.flags = MMC_RSP_NONE | MMC_CMD_AC;

	return mmc_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
}

int mmc_go_idle(struct mmc_host *host)
{
	int err;
	struct mmc_command cmd = {0};

	/*
	 * Non-SPI hosts need to prevent chipselect going active during
	 * GO_IDLE; that would put chips into SPI mode.  Remind them of
	 * that in case of hardware that won't pull up DAT3/nCS otherwise.
	 *
	 * SPI hosts ignore ios.chip_select; it's managed according to
	 * rules that must accommodate non-MMC slaves which this layer
	 * won't even know about.
	 */
	if (!mmc_host_is_spi(host)) {
		mmc_set_chip_select(host, MMC_CS_HIGH);
		mmc_delay(1);
	}

	cmd.opcode = MMC_GO_IDLE_STATE;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_NONE | MMC_CMD_BC;

	err = mmc_wait_for_cmd(host, &cmd, 0);

	mmc_delay(1);

	if (!mmc_host_is_spi(host)) {
		mmc_set_chip_select(host, MMC_CS_DONTCARE);
		mmc_delay(1);
	}

	host->use_spi_crc = 0;

	return err;
}

int mmc_send_op_cond(struct mmc_host *host, u32 ocr, u32 *rocr)
{
	struct mmc_command cmd = {0};
	int i, err = 0;

	BUG_ON(!host);

	cmd.opcode = MMC_SEND_OP_COND;
	cmd.arg = mmc_host_is_spi(host) ? 0 : ocr;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R3 | MMC_CMD_BCR;

	for (i = 100; i; i--) {
		err = mmc_wait_for_cmd(host, &cmd, 0);
		if (err)
			break;

		/* if we're just probing, do a single pass */
		if (ocr == 0)
			break;

		/* otherwise wait until reset completes */
		if (mmc_host_is_spi(host)) {
			if (!(cmd.resp[0] & R1_SPI_IDLE))
				break;
		} else {
			if (cmd.resp[0] & MMC_CARD_BUSY)
				break;
		}

		err = -ETIMEDOUT;

		mmc_delay(10);
	}

	if (rocr && !mmc_host_is_spi(host))
		*rocr = cmd.resp[0];

	return err;
}

int mmc_all_send_cid(struct mmc_host *host, u32 *cid)
{
	int err;
	struct mmc_command cmd = {0};
#ifdef CONFIG_HUAWEI_SDCARD_DSM
	char *log_buff;
	int   buff_len;
#endif
	BUG_ON(!host);
	BUG_ON(!cid);

	cmd.opcode = MMC_ALL_SEND_CID;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R2 | MMC_CMD_BCR;

	err = mmc_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
#ifdef CONFIG_HUAWEI_SDCARD_DSM
	if(!strcmp(mmc_hostname(host), "mmc1"))
	{
		 dsm_sdcard_cmd_logs[DSM_SDCARD_CMD2_R0].value = cmd.resp[0];
		 dsm_sdcard_cmd_logs[DSM_SDCARD_CMD2_R1].value = cmd.resp[1];
		 dsm_sdcard_cmd_logs[DSM_SDCARD_CMD2_R2].value = cmd.resp[2];
		 dsm_sdcard_cmd_logs[DSM_SDCARD_CMD2_R3].value = cmd.resp[3];
	}
#endif
	if (err)
	{
#ifdef CONFIG_HUAWEI_SDCARD_DSM
		if(-ENOMEDIUM != err && -ETIMEDOUT != err
		&& !strcmp(mmc_hostname(host), "mmc1") && !dsm_client_ocuppy(sdcard_dclient))
		{
			log_buff = dsm_sdcard_get_log(DSM_SDCARD_CMD2_R3, err);
			buff_len = strlen(log_buff);
			dsm_client_copy(sdcard_dclient,log_buff,buff_len + 1);
			dsm_client_notify(sdcard_dclient, DSM_SDCARD_CMD2_RESP_ERR);
		}
#endif
		return err;
	}
	memcpy(cid, cmd.resp, sizeof(u32) * 4);

	return 0;
}

int mmc_set_relative_addr(struct mmc_card *card)
{
	int err;
	struct mmc_command cmd = {0};

	BUG_ON(!card);
	BUG_ON(!card->host);

	cmd.opcode = MMC_SET_RELATIVE_ADDR;
	cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	err = mmc_wait_for_cmd(card->host, &cmd, MMC_CMD_RETRIES);
	if (err)
		return err;

	return 0;
}

static int
mmc_send_cxd_native(struct mmc_host *host, u32 arg, u32 *cxd, int opcode)
{
	int err;
	struct mmc_command cmd = {0};

	BUG_ON(!host);
	BUG_ON(!cxd);

	cmd.opcode = opcode;
	cmd.arg = arg;
	cmd.flags = MMC_RSP_R2 | MMC_CMD_AC;

	err = mmc_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
	if (err)
		return err;

	memcpy(cxd, cmd.resp, sizeof(u32) * 4);

	return 0;
}

/*
 * NOTE: void *buf, caller for the buf is required to use DMA-capable
 * buffer or on-stack buffer (with some overhead in callee).
 */
static int
mmc_send_cxd_data(struct mmc_card *card, struct mmc_host *host,
		u32 opcode, void *buf, unsigned len)
{
	struct mmc_request mrq = {NULL};
	struct mmc_command cmd = {0};
	struct mmc_data data = {0};
	struct scatterlist sg;
	void *data_buf;
	int is_on_stack;

	is_on_stack = object_is_on_stack(buf);
	if (is_on_stack) {
		/*
		 * dma onto stack is unsafe/nonportable, but callers to this
		 * routine normally provide temporary on-stack buffers ...
		 */
		data_buf = kmalloc(len, GFP_KERNEL);
		if (!data_buf)
			return -ENOMEM;
	} else
		data_buf = buf;

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = opcode;
	cmd.arg = 0;

	/* NOTE HACK:  the MMC_RSP_SPI_R1 is always correct here, but we
	 * rely on callers to never use this with "native" calls for reading
	 * CSD or CID.  Native versions of those commands use the R2 type,
	 * not R1 plus a data block.
	 */
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = len;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	sg_init_one(&sg, data_buf, len);

	if (opcode == MMC_SEND_CSD || opcode == MMC_SEND_CID) {
		/*
		 * The spec states that CSR and CID accesses have a timeout
		 * of 64 clock cycles.
		 */
		data.timeout_ns = 0;
		data.timeout_clks = 64;
	} else
		mmc_set_data_timeout(&data, card);

	mmc_wait_for_req(host, &mrq);

	if (is_on_stack) {
		memcpy(buf, data_buf, len);
		kfree(data_buf);
	}
#ifdef CONFIG_HUAWEI_EMMC_DSM
	 if (cmd.error || data.error)
		if (host->index == device_index)
			{
			DSM_EMMC_LOG(card, DSM_EMMC_SEND_CXD_ERR,
				"opcode:%d failed, cmd.error:%d, data.error:%d\n",
				opcode, cmd.error, data.error);
		}
#endif
	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;

	return 0;
}

int mmc_send_csd(struct mmc_card *card, u32 *csd)
{
	int ret, i;
	u32 *csd_tmp;

	if (!mmc_host_is_spi(card->host))
		return mmc_send_cxd_native(card->host, card->rca << 16,
				csd, MMC_SEND_CSD);

	csd_tmp = kmalloc(16, GFP_KERNEL);
	if (!csd_tmp)
		return -ENOMEM;

	ret = mmc_send_cxd_data(card, card->host, MMC_SEND_CSD, csd_tmp, 16);
	if (ret)
		goto err;

	for (i = 0;i < 4;i++)
		csd[i] = be32_to_cpu(csd_tmp[i]);

err:
	kfree(csd_tmp);
	return ret;
}

int mmc_send_cid(struct mmc_host *host, u32 *cid)
{
	int ret, i;
	u32 *cid_tmp;

	if (!mmc_host_is_spi(host)) {
		if (!host->card)
			return -EINVAL;
		return mmc_send_cxd_native(host, host->card->rca << 16,
				cid, MMC_SEND_CID);
	}

	cid_tmp = kmalloc(16, GFP_KERNEL);
	if (!cid_tmp)
		return -ENOMEM;

	ret = mmc_send_cxd_data(NULL, host, MMC_SEND_CID, cid_tmp, 16);
	if (ret)
		goto err;

	for (i = 0;i < 4;i++)
		cid[i] = be32_to_cpu(cid_tmp[i]);

err:
	kfree(cid_tmp);
	return ret;
}

int mmc_send_ext_csd(struct mmc_card *card, u8 *ext_csd)
{
	return mmc_send_cxd_data(card, card->host, MMC_SEND_EXT_CSD,
			ext_csd, 512);
}
EXPORT_SYMBOL_GPL(mmc_send_ext_csd);

int mmc_get_ext_csd(struct mmc_card *card, u8 **new_ext_csd)
{
	int err;
	u8 *ext_csd;

	if (!card || !new_ext_csd)
		return -EINVAL;

	if (!mmc_can_ext_csd(card))
		return -EOPNOTSUPP;

	/*
	 * As the ext_csd is so large and mostly unused, we don't store the
	 * raw block in mmc_card.
	 */
	ext_csd = kmalloc(512, GFP_KERNEL);
	if (!ext_csd)
		return -ENOMEM;

	err = mmc_send_ext_csd(card, ext_csd);
	if (err)
		kfree(ext_csd);
	else
		*new_ext_csd = ext_csd;

	return err;
}
EXPORT_SYMBOL_GPL(mmc_get_ext_csd);

int mmc_spi_read_ocr(struct mmc_host *host, int highcap, u32 *ocrp)
{
	struct mmc_command cmd = {0};
	int err;

	cmd.opcode = MMC_SPI_READ_OCR;
	cmd.arg = highcap ? (1 << 30) : 0;
	cmd.flags = MMC_RSP_SPI_R3;

	err = mmc_wait_for_cmd(host, &cmd, 0);

	*ocrp = cmd.resp[1];
	return err;
}

int mmc_spi_set_crc(struct mmc_host *host, int use_crc)
{
	struct mmc_command cmd = {0};
	int err;

	cmd.opcode = MMC_SPI_CRC_ON_OFF;
	cmd.flags = MMC_RSP_SPI_R1;
	cmd.arg = use_crc;

	err = mmc_wait_for_cmd(host, &cmd, 0);
	if (!err)
		host->use_spi_crc = use_crc;
	return err;
}

int mmc_switch_status_error(struct mmc_host *host, u32 status)
{
	if (mmc_host_is_spi(host)) {
		if (status & R1_SPI_ILLEGAL_COMMAND)
			return -EBADMSG;
	} else {
		if (status & 0xFDFFA000)
			pr_warn("%s: unexpected status %#x after switch\n",
				mmc_hostname(host), status);
		if (status & R1_SWITCH_ERROR)
			return -EBADMSG;
	}
	return 0;
}

/**
 *	__mmc_switch - modify EXT_CSD register
 *	@card: the MMC card associated with the data transfer
 *	@set: cmd set values
 *	@index: EXT_CSD register index
 *	@value: value to program into EXT_CSD register
 *	@timeout_ms: timeout (ms) for operation performed by register write,
 *                   timeout of zero implies maximum possible timeout
 *	@use_busy_signal: use the busy signal as response type
 *	@send_status: send status cmd to poll for busy
 *	@ignore_crc: ignore CRC errors when sending status cmd to poll for busy
 *
 *	Modifies the EXT_CSD register for selected card.
 */
int __mmc_switch(struct mmc_card *card, u8 set, u8 index, u8 value,
		unsigned int timeout_ms, bool use_busy_signal, bool send_status,
		bool ignore_crc)
{
	struct mmc_host *host = card->host;
	int err;
	struct mmc_command cmd = {0};
	unsigned long timeout;
	u32 status = 0;
	bool use_r1b_resp = use_busy_signal, busy = false;
	bool expired = false;

	mmc_retune_hold(host);

	/*
	 * If the cmd timeout and the max_busy_timeout of the host are both
	 * specified, let's validate them. A failure means we need to prevent
	 * the host from doing hw busy detection, which is done by converting
	 * to a R1 response instead of a R1B.
	 */
	if (timeout_ms && host->max_busy_timeout &&
		(timeout_ms > host->max_busy_timeout))
		use_r1b_resp = false;

	cmd.opcode = MMC_SWITCH;
	cmd.arg = (MMC_SWITCH_MODE_WRITE_BYTE << 24) |
		  (index << 16) |
		  (value << 8) |
		  set;
	cmd.flags = MMC_CMD_AC;
	if (use_r1b_resp) {
		cmd.flags |= MMC_RSP_SPI_R1B | MMC_RSP_R1B;
		/*
		 * A busy_timeout of zero means the host can decide to use
		 * whatever value it finds suitable.
		 */
		cmd.busy_timeout = timeout_ms;
	} else {
		cmd.flags |= MMC_RSP_SPI_R1 | MMC_RSP_R1;
	}

	if (index == EXT_CSD_SANITIZE_START)
		cmd.sanitize_busy = true;

	err = mmc_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
	if (err)
		goto out;

	/* No need to check card status in case of unblocking command */
	if (!use_busy_signal)
		goto out;

	/*
	 * CRC errors shall only be ignored in cases were CMD13 is used to poll
	 * to detect busy completion.
	 */
	if ((host->caps & MMC_CAP_WAIT_WHILE_BUSY) && use_r1b_resp)
		ignore_crc = false;

	/* We have an unspecified cmd timeout, use the fallback value. */
	if (!timeout_ms)
		timeout_ms = MMC_OPS_TIMEOUT_MS;

	/* Must check status to be sure of no errors. */
	timeout = jiffies + msecs_to_jiffies(timeout_ms);
	do {
		if (send_status) {
			/*
			 * Due to the possibility of being preempted after
			 * sending the status command, check the expiration
			 * time first.
			 */
			expired = time_after(jiffies, timeout);
			err = __mmc_send_status(card, &status, ignore_crc);
			if (err)
				goto out;
		}
		if ((host->caps & MMC_CAP_WAIT_WHILE_BUSY) && use_r1b_resp)
			break;
		if (mmc_host_is_spi(host))
			break;

		/*
		 * We are not allowed to issue a status command and the host
		 * does'nt support MMC_CAP_WAIT_WHILE_BUSY, then we can only
		 * rely on waiting for the stated timeout to be sufficient.
		 */
		if (!send_status) {
			if (use_r1b_resp && host->ops->card_busy) {
				if (!card->host->ops->card_busy(host)) {
					err = 0;
					goto out;
				} else {
					busy = true;
				}
			} else {
				mmc_delay(timeout_ms);
				goto out;
			}
		}

		/* Timeout if the device never leaves the program state. */
		if (expired && R1_CURRENT_STATE(status) == R1_STATE_PRG) {
			pr_err("%s: Card stuck in programming state! %s\n",
				mmc_hostname(host), __func__);
			err = -ETIMEDOUT;
			goto out;
		}
	} while ((R1_CURRENT_STATE(status) == R1_STATE_PRG) || busy);

	err = mmc_switch_status_error(host, status);
out:
	mmc_retune_release(host);

	return err;
}
EXPORT_SYMBOL_GPL(__mmc_switch);

int mmc_switch(struct mmc_card *card, u8 set, u8 index, u8 value,
		unsigned int timeout_ms)
{
	return __mmc_switch(card, set, index, value, timeout_ms, true, true,
				false);
}
EXPORT_SYMBOL_GPL(mmc_switch);

int mmc_send_tuning(struct mmc_host *host)
{
	struct mmc_request mrq = {NULL};
	struct mmc_command cmd = {0};
	struct mmc_data data = {0};
	struct scatterlist sg;
	struct mmc_ios *ios = &host->ios;
	const u8 *tuning_block_pattern;
	int size, err = 0;
	u8 *data_buf;
	u32 opcode;

	if (ios->bus_width == MMC_BUS_WIDTH_8) {
		tuning_block_pattern = tuning_blk_pattern_8bit;
		size = sizeof(tuning_blk_pattern_8bit);
		opcode = MMC_SEND_TUNING_BLOCK_HS200;
	} else if (ios->bus_width == MMC_BUS_WIDTH_4) {
		tuning_block_pattern = tuning_blk_pattern_4bit;
		size = sizeof(tuning_blk_pattern_4bit);
		opcode = MMC_SEND_TUNING_BLOCK;
	} else
		return -EINVAL;

	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf)
		return -ENOMEM;

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = opcode;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = size;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;

	/*
	 * According to the tuning specs, Tuning process
	 * is normally shorter 40 executions of CMD19,
	 * and timeout value should be shorter than 150 ms
	 */
	data.timeout_ns = 150 * NSEC_PER_MSEC;

	data.sg = &sg;
	data.sg_len = 1;
	sg_init_one(&sg, data_buf, size);

	mmc_wait_for_req(host, &mrq);

	if (cmd.error) {
		err = cmd.error;
		goto out;
	}

	if (data.error) {
		err = data.error;
		goto out;
	}

	if (memcmp(data_buf, tuning_block_pattern, size))
		err = -EIO;

out:
	kfree(data_buf);
	return err;
}
EXPORT_SYMBOL_GPL(mmc_send_tuning);

static int
mmc_send_bus_test(struct mmc_card *card, struct mmc_host *host, u8 opcode,
		  u8 len)
{
	struct mmc_request mrq = {NULL};
	struct mmc_command cmd = {0};
	struct mmc_data data = {0};
	struct scatterlist sg;
	u8 *data_buf;
	u8 *test_buf;
	int i, err;
	static u8 testdata_8bit[8] = { 0x55, 0xaa, 0, 0, 0, 0, 0, 0 };
	static u8 testdata_4bit[4] = { 0x5a, 0, 0, 0 };

	/* dma onto stack is unsafe/nonportable, but callers to this
	 * routine normally provide temporary on-stack buffers ...
	 */
	data_buf = kmalloc(len, GFP_KERNEL);
	if (!data_buf)
		return -ENOMEM;

	if (len == 8)
		test_buf = testdata_8bit;
	else if (len == 4)
		test_buf = testdata_4bit;
	else {
		pr_err("%s: Invalid bus_width %d\n",
		       mmc_hostname(host), len);
		kfree(data_buf);
		return -EINVAL;
	}

	if (opcode == MMC_BUS_TEST_W)
		memcpy(data_buf, test_buf, len);

	mrq.cmd = &cmd;
	mrq.data = &data;
	cmd.opcode = opcode;
	cmd.arg = 0;

	/* NOTE HACK:  the MMC_RSP_SPI_R1 is always correct here, but we
	 * rely on callers to never use this with "native" calls for reading
	 * CSD or CID.  Native versions of those commands use the R2 type,
	 * not R1 plus a data block.
	 */
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = len;
	data.blocks = 1;
	if (opcode == MMC_BUS_TEST_R)
		data.flags = MMC_DATA_READ;
	else
		data.flags = MMC_DATA_WRITE;

	data.sg = &sg;
	data.sg_len = 1;
	mmc_set_data_timeout(&data, card);
	sg_init_one(&sg, data_buf, len);
	mmc_wait_for_req(host, &mrq);
	err = 0;
	if (opcode == MMC_BUS_TEST_R) {
		for (i = 0; i < len / 4; i++)
			if ((test_buf[i] ^ data_buf[i]) != 0xff) {
				err = -EIO;
				break;
			}
	}
	kfree(data_buf);

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;

	return err;
}

int mmc_bus_test(struct mmc_card *card, u8 bus_width)
{
	int err, width;

	if (bus_width == MMC_BUS_WIDTH_8)
		width = 8;
	else if (bus_width == MMC_BUS_WIDTH_4)
		width = 4;
	else if (bus_width == MMC_BUS_WIDTH_1)
		return 0; /* no need for test */
	else
		return -EINVAL;

	/*
	 * Ignore errors from BUS_TEST_W.  BUS_TEST_R will fail if there
	 * is a problem.  This improves chances that the test will work.
	 */
	mmc_send_bus_test(card, card->host, MMC_BUS_TEST_W, width);
	err = mmc_send_bus_test(card, card->host, MMC_BUS_TEST_R, width);
	return err;
}

int mmc_send_hpi_cmd(struct mmc_card *card, u32 *status)
{
	struct mmc_command cmd = {0};
	unsigned int opcode;
	int err;

	if (!card->ext_csd.hpi) {
		pr_warn("%s: Card didn't support HPI command\n",
			mmc_hostname(card->host));
		return -EINVAL;
	}

	opcode = card->ext_csd.hpi_cmd;
	if (opcode == MMC_STOP_TRANSMISSION)
		cmd.flags = MMC_RSP_R1B | MMC_CMD_AC;
	else if (opcode == MMC_SEND_STATUS)
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	cmd.opcode = opcode;
	cmd.arg = card->rca << 16 | 1;

	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err) {
		pr_warn("%s: error %d interrupting operation. "
			"HPI command response %#x\n", mmc_hostname(card->host),
			err, cmd.resp[0]);
		return err;
	}
	if (status)
		*status = cmd.resp[0];

	return 0;
}

int mmc_can_ext_csd(struct mmc_card *card)
{
	return (card && card->csd.mmca_vsn > CSD_SPEC_VER_3);
}
#ifdef CONFIG_MMC_PASSWORDS
int sd_send_status(struct mmc_card *card)
{
	int err;
	u32 status;
	do {
		err = mmc_send_status(card, &status);
		if (err)
		{
			break;
		}
		if (card->host->caps & MMC_CAP_WAIT_WHILE_BUSY)
			break;
		if (mmc_host_is_spi(card->host))
			break;
	} while (R1_CURRENT_STATE(status) == 7);

	return err;
}

int sd_send_blocklen(struct mmc_card *card,unsigned int blocklen)
{
	int err;
	struct mmc_command cmd = {0};

	cmd.opcode = MMC_SET_BLOCKLEN;
	cmd.arg = blocklen;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
	printk("[SDLOCK]  %s start  blocklen=%d \n",__func__,blocklen);
	err = mmc_wait_for_cmd(card->host, &cmd, MMC_CMD_RETRIES);
	if (err)
	{
		printk("[SDLOCK] %s failed blocklen=%d \n",__func__,blocklen);
	}
	return err;
}

int sd_send_lock_unlock_cmd(struct mmc_card *card,u8* data_buf,int data_size,int max_buf_size)
{
	int err;
	struct mmc_request mrq = {0};
	struct mmc_command cmd = {0};
	struct mmc_data data = {0};
	struct scatterlist sg;

	cmd.opcode = MMC_LOCK_UNLOCK;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = data_size;
	data.blocks = 1;
	data.flags = MMC_DATA_WRITE;
	data.sg = &sg;
	data.sg_len = 1;
	mmc_set_data_timeout(&data, card);
	data.timeout_ns = (4*1000*1000*1000u);

	mrq.cmd = &cmd;
	mrq.data = &data;

	sg_init_one(&sg, data_buf, max_buf_size);

	printk("[SDLOCK] %s  begin \n",__func__);
	mmc_wait_for_req(card->host, &mrq);
	printk("[SDLOCK] %s  end \n",__func__);

	err = cmd.error;
	if (err) {
		printk("[SDLOCK] %s: lock unlock cmd error %d\n", __func__, cmd.error);
		return err;
	}

	err = data.error;
	if (err) {
		printk("[SDLOCK] %s: lock unlock data error %d\n", __func__,data.error);
		dev_err(mmc_dev(card->host), "[SDLOCK] %s: data error %d\n",__func__, data.error);
	}
	return err;
}

int sd_wait_lock_unlock_cmd(struct mmc_card *card,int mode)
{
	int err;
	struct mmc_command cmd = {0};
	unsigned long erase_timeout;
	unsigned long normal_timeout;


	cmd.opcode = MMC_SEND_STATUS;
	cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	/* set timeout for forced erase operation to 3 min. (see MMC spec) */
	erase_timeout = jiffies + 180 * HZ;
	normal_timeout = jiffies + 10 * HZ;
	if(mode & MMC_LOCK_MODE_ERASE) {
		do {
			/* we cannot use "retries" here because the
			 * R1_LOCK_UNLOCK_FAILED bit is cleared by subsequent reads to
			 * the status register, hiding the error condition */
			err = mmc_wait_for_cmd(card->host, &cmd, 0);
			if (err) {
				printk("[SDLOCK] %s mmc_wait_for_cmd err=%d resp[0] =%x \n",__func__,err,cmd.resp[0]);
				break;
			}

			/* the other modes don't need timeout checking */
			if (!(mode & MMC_LOCK_MODE_ERASE))
				continue;
			if (time_after(jiffies, erase_timeout)) {
				dev_dbg(&card->dev, "forced erase timed out\n");
				err = -ETIMEDOUT;
				break;
			}
		} while (!(cmd.resp[0] & R1_READY_FOR_DATA) || (cmd.resp[0] & R1_CARD_IS_LOCKED));
	} else {
		do {
			/* we cannot use "retries" here because the
			 * R1_LOCK_UNLOCK_FAILED bit is cleared by subsequent reads to
			 * the status register, hiding the error condition */
			err = mmc_wait_for_cmd(card->host, &cmd, 0);
			if (err) {
				printk("[SDLOCK] %s mmc_wait_for_cmd err=%d resp[0] =%x \n",__func__,err,cmd.resp[0]);
				break;
			}

			if (time_after(jiffies, normal_timeout)) {
				dev_dbg(&card->dev, "normal timed out\n");
				err = -ETIMEDOUT;
				printk("[SDLOCK] %s normal timed out err=%d resp[0] =%x \n",__func__,err,cmd.resp[0]);
				break;
			}
		} while (!(cmd.resp[0] & R1_READY_FOR_DATA));
	}

	printk("[SDLOCK] %s MMC_SEND_STATUS and cmd.resp[0] = 0x%x. \r\n",__func__, cmd.resp[0]);

	if (cmd.resp[0] & R1_LOCK_UNLOCK_FAILED) {
		printk("%s: LOCK_UNLOCK operation failed\n", __func__);
		err = -EIO;
		return err;
	}

	if (cmd.resp[0] & R1_CARD_IS_LOCKED)
	{
		printk("%s: R1_CARD_IS_LOCKED\n", __func__);
		mmc_card_set_locked(card);
	}
	else
	{
		printk("%s: R1_CARD_IS_UNLOCKED\n", __func__);
		card->state &= ~MMC_STATE_LOCKED;
	}

	return err;
}

int mmc_lock_unlock_by_buf(struct mmc_card *card, u8* key_buf,int key_len, int mode)
{

	int err, data_size;
	int max_buf_size ;
	u8 *data_buf;
	max_buf_size = 32;
	data_size = 1;
	//max password(16byte) max_key = max_password(old) + max_password(new) + 0xFF + 0xFF = 34
	if(key_len > 30)
	{
		return -EINVAL;
	}

	printk("[SDLOCK] %s begin mode=%d  \n",__func__, mode);

	if (!(mode & MMC_LOCK_MODE_ERASE)) {
//		data_size =  key_len ;
		data_size = max_buf_size;
	}

	data_buf = kzalloc(max_buf_size, GFP_KERNEL);
	if (!data_buf)
	{
		printk("[SDLOCK] %s kzalloc failed\n",__func__);
		return -ENOMEM;
	}
	memset(data_buf, 0, max_buf_size);
	data_buf[0] |= mode;
	if (mode & MMC_LOCK_MODE_UNLOCK)
		data_buf[0] &= ~MMC_LOCK_MODE_UNLOCK;

	if (!(mode & MMC_LOCK_MODE_ERASE)) {
		data_buf[1] = key_len-2; //exclude end 2 chars (0xFF 0xFF)
		memcpy(data_buf + 2, key_buf, key_len);
	} else {
		data_buf[1] =0xff;
		data_buf[2] = 0xff;
	}

	//mmc_rpm_hold(card->host, &card->dev);
#ifdef CONFIG_MMC_BLOCK_DEFERRED_RESUME
	if (mmc_bus_needs_resume(card->host)) {
		mmc_resume_bus(card->host);
	}
#endif
	//mmc_rpm_release(card->host, &card->dev);

	/*-----------Set mmc Status Command-----------------------------*/
	err = sd_send_status(card);
	if (err) {
		goto out;
	}
	printk("[SDLOCK] %s STATUS data_size=%d\r\n",__func__,data_size);

	/*------------Set Block Length Command--------------------------*/
	err = sd_send_blocklen(card,data_size);
	if (err) {
		goto out;
	}
	printk("[SDLOCK] %s MMC_SET_BLOCKLEN \r\n",__func__);

	/*-----------Set Lock/Unlock Command---------------------------*/
	err = sd_send_lock_unlock_cmd(card,data_buf,data_size,max_buf_size);
	printk("[SDLOCK] %s  %d  1058 \r\n",__func__,err);
	 if(err != 0) {
             if (err==(-ETIMEDOUT)&&(mode==MMC_LOCK_MODE_ERASE)) {
                   printk("[SDLOCK] %s :data timeout error[%d] \r\n",__func__,err);
             }
             else {
                  goto out;
             }
          }
	printk("[SDLOCK] %s MMC_LOCK_UNLOCK \r\n",__func__);

	/*-------------Set mmc Status Command--------------------*/
	err = sd_wait_lock_unlock_cmd(card,mode);
	if (err) {
		mmc_set_blocklen(card, 512);
		goto out;
	}

	err = mmc_set_blocklen(card, 512);

out:
	kfree(data_buf);

	printk("[SDLOCK] %s end \n",__func__);

	return err;
}

#endif /* CONFIG_MMC_PASSWORDS */
