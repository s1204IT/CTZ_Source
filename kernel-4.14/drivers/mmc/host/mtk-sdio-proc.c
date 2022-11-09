/*
 * Copyright (c) 2014-2015 MediaTek Inc.
 * Author: Chaotian.Jing <chaotian.jing@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/random.h>

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>

#include <linux/mmc/card.h>
#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>

#define SDIO_CCCR_MTK_DDR208    0xF2
#define SUCCESS		0
#define FAIL		1

enum {
	Read = 0,
	Write = 1,
	Reset = 2,
	Stress_read = 3,
	Stress_write = 4,
	re_init = 5,
	Sress_test = 6
};

static struct mmc_host *host;

/**
 * define some count timer.
 */
#define KAL_TIME_INTERVAL_DECLARATION()  struct timeval __rTs, __rTe
#define KAL_REC_TIME_START()             do_gettimeofday(&__rTs)
#define KAL_REC_TIME_END()	             do_gettimeofday(&__rTe)
#define KAL_GET_TIME_INTERVAL() \
((__rTe.tv_sec * USEC_PER_SEC + __rTe.tv_usec) - \
(__rTs.tv_sec * USEC_PER_SEC + __rTs.tv_usec))

/**
 * sdio_proc_show dispaly the common cccr and cis.
 */
static int sdio_proc_show(struct seq_file *m, void *v)
{
	seq_puts(m, "\n=========================================\n");
	seq_puts(m, "read cmd format:\n");
	seq_puts(m, "echo 0 0xReg 0xfunction> /proc/sdio\n");

	seq_puts(m, "write cmd format:\n");
	seq_puts(m, "echo 1 0xReg 0xfunction 0xValue> /proc/sdio\n");

	seq_puts(m, "tune cmd format:\n");
	seq_puts(m, "echo 2 0x0 0x0 > /proc/sdio\n");

	seq_puts(m, "throughput read cmd format:\n");
	seq_puts(m, "echo 3 0xb0 0x1 > /proc/sdio\n");

	seq_puts(m, "throughtput write cmd format:\n");
	seq_puts(m, "echo 4 0xb0 0x1 0x0 0x0 > /proc/sdio\n");
	seq_puts(m, "throughtput write de-sense format:\n");
	seq_puts(m, "echo 4 0xb0 0x1 0x0 0x1 > /proc/sdio\n");

	seq_puts(m, "re init cmd format:\n");
	seq_puts(m, "echo 5 0x0 0x0 > /proc/sdio\n");
	seq_puts(m, "r/w stress format:\n");
	seq_puts(m, "echo 6 0xb0 0x1 > /proc/sdio\n");
	seq_puts(m, "=========================================\n");

	return SUCCESS;
}

/**
 * This function can be used to do HQA、SDIO SHPA for read.
 * HQA: for SDR104 mode, it is better to set 0xb8(func1) to 0x5a.
 * HQA: for SD3.0 plus, it is better to set 0xb8(func1) to 0x33cc.
 */
static int multi_read(struct sdio_func *func, struct mmc_host *host)
{
	int err, i;
	u32 data, value, count_rw = 0;
	unsigned long total = 0;
	unsigned char *fac_buf = NULL;

	KAL_TIME_INTERVAL_DECLARATION();
	__kernel_suseconds_t usec;

	func->cur_blksize = 0x200;
	fac_buf = kmalloc(0x40000, GFP_KERNEL);
	if (!fac_buf)
		return FAIL;

	/* This function can be used to do HQA、SDIO SHPA */
	data = sdio_f0_readb(func, SDIO_CCCR_MTK_DDR208, &err);
	if (err) {
		kfree(fac_buf);
		return FAIL;
	} else if ((data & 0x3) == 0x3) {
		value = 0x33cc33cc;
		err = sdio_writesb(func, 0xb8, &value, 0x4);
		dev_info(mmc_dev(host), "[sd3.0+]: value is 0x33cc\n");
	} else {
		value = 0x5a5a5a5a;
		err = sdio_writesb(func, 0xb8, &value, 0x4);
		dev_info(mmc_dev(host), "[sd3.0]: value is 0x5a5a\n");
	}
	value = 0;
	err = sdio_readsb(func, &value, 0xb8, 0x4);

	i = 0;
	/* byte to bit */
	total = 0x6400000 * 8;
	do {
		if (i % 0x190 == 0)
			KAL_REC_TIME_START();
		err = sdio_readsb(func, fac_buf, 0xb0, 0x40000);
		if (err)
			count_rw = count_rw + 1;
		i = i + 1;
		if ((i / 0x190) && (i % 0x190 == 0)) {
			KAL_REC_TIME_END();
			usec = KAL_GET_TIME_INTERVAL();
			dev_info(mmc_dev(host), "read: %lu Kbps, err:%x.\n",
				total / (usec / USEC_PER_MSEC), count_rw);
		}
	} while (i < 0x190 * 0x100);

	kfree(fac_buf);

	return SUCCESS;
}

/**
 * This function can be used to do de-sense、HQA、SHPA for write.
 * HQA: for SDR104 mode, it is better to set 0xb8(func1) to 0x5a.
 * HQA: for SD3.0 plus, it is better to set 0xb8(func1) to 0x33cc.
 * de_sense: data is random or not.
 */
static int multi_write(struct sdio_func *func, struct mmc_host *host,
		int de_sense)
{
	int err, i;
	u32 data, value, count_rw = 0;
	unsigned long total = 0;
	unsigned char *fac_buf = NULL;

	KAL_TIME_INTERVAL_DECLARATION();
	__kernel_suseconds_t usec;

	func->cur_blksize = 0x200;
	fac_buf = kmalloc(0x40000, GFP_KERNEL);
	if (!fac_buf)
		return FAIL;

	/* This function can be used to do de-sense、HQA、SHPA */
	data = sdio_f0_readb(func, SDIO_CCCR_MTK_DDR208, &err);
	if (err) {
		kfree(fac_buf);
		return FAIL;
	}
	if (de_sense) {
		//err = wait_for_random_bytes();
		if (!err)
			dev_info(mmc_dev(host), "wait random bytes success.\n");
		//get_random_bytes(fac_buf, 0x10000);
	} else if ((err & 0x3) == 0x3) {
		value = 0x33cc33cc;
		err = sdio_writesb(func, 0xb8, &value, 0x4);
		err = sdio_readsb(func, fac_buf, 0xb0, 0x40000);
		dev_info(mmc_dev(host), "[sd3.0+]: value is 0x33cc\n");
	} else {
		memset(fac_buf, 0x5a, 0x40000);
		dev_info(mmc_dev(host), "[sd3.0]: value is 0x5a5a\n");
	}

	i = 0;
	/* byte to bit */
	total = 0x6400000 * 8;
	do {
		if (i % 0x190 == 0)
			KAL_REC_TIME_START();
		err = sdio_writesb(func, 0xb0, fac_buf, 0x40000);
		if (err)
			count_rw = count_rw + 1;
		i = i + 1;
		if ((i / 0x190) && (i % 0x190 == 0)) {
			KAL_REC_TIME_END();
			usec = KAL_GET_TIME_INTERVAL();
			dev_info(mmc_dev(host), "write: %lu Kbps, err:%x\n",
				total / (usec / USEC_PER_MSEC), count_rw);
		}
	} while (i < 0x190 * 0x100);

	kfree(fac_buf);

	return SUCCESS;
}

static int sdio_stress_test(struct sdio_func *func, struct mmc_host *host)
{
	int err;
	unsigned int count;
	unsigned char *fac_buf = NULL;

	count = 0;
	func->cur_blksize = 0x200;
	fac_buf = kmalloc(0x40000, GFP_KERNEL);
	if (!fac_buf)
		return FAIL;

	while (1) {
		get_random_bytes(fac_buf, 0x10000);
		err = sdio_writesb(func, 0xb0, fac_buf, 0x40000);
		if (err) {
			kfree(fac_buf);
			return FAIL;
		}
		err = sdio_readsb(func, fac_buf, 0xb0, 0x40000);
		if (err) {
			kfree(fac_buf);
			return FAIL;
		}
		sdio_release_host(func);
		count++;
		if ((count % 1000 == 0))
			dev_info(mmc_dev(host), "SDIO R/W count:%d\n", count);
		udelay(300);
		sdio_claim_host(func);
	}
}

/* create struct sdio_func variable for test */
static int create_func(struct sdio_func *func, struct mmc_card *card,
		int fn, int cmd)
{
	unsigned int max_blk;

	/* Judge whether request fn is over the max functions. */
	if (fn > card->sdio_funcs) {
		dev_info(mmc_dev(host), "the fn is over the max sdio funcs.\n");
		return FAIL;
	}

	if (fn) {
		/**
		 * The test read/write api don't need more func
		 * information. So we just use the card & func->num
		 * to the new struct func.
		 */
		if (card->sdio_func[fn - 1]) {
			func->card = card;
			func->num = card->sdio_func[fn - 1]->num;
			func->tuples = card->sdio_func[fn - 1]->tuples;
			func->tmpbuf = card->sdio_func[fn - 1]->tmpbuf;
			max_blk = card->sdio_func[fn - 1]->max_blksize;
			func->max_blksize = max_blk;
			func->cur_blksize = 1;
		} else
			dev_info(mmc_dev(host), "func %d is null,.\n", fn);
	} else if (cmd != re_init) {
		/**
		 * function 0 should not need struct func.
		 * but the api need the parameter, so we create
		 * the a new func for function 0.
		 * re_init cmd do not need struct func.
		 */
		func->card = card;
		func->tuples = card->tuples;
		func->num = 0;
		func->max_blksize = 0x16;
		func->cur_blksize = 0x1;

		func->tmpbuf = kmalloc(func->cur_blksize, GFP_KERNEL);
		if (!func->tmpbuf)
			return FAIL;
		memset(func->tmpbuf, 0, func->cur_blksize);
	}

	return SUCCESS;
}

/**
 * sdio_proc_write - read/write sdio function register.
 */
static ssize_t sdio_proc_write(struct file *file, const char *buf,
		size_t count, loff_t *f_pos)
{
	struct mmc_card *card;
	struct sdio_func *func;
	char *cmd_buf, *str_hand;
	unsigned int cmd, addr, fn, value, de_sense;
	unsigned char result;
	int ret;

	WARN_ON(!host);
	card = host->card;

	cmd_buf = kzalloc((count + 1), GFP_KERNEL);
	if (!cmd_buf)
		return count;

	str_hand = kzalloc(2, GFP_KERNEL);
	if (!str_hand) {
		kfree(cmd_buf);
		return count;
	}

	func = kzalloc(sizeof(struct sdio_func), GFP_KERNEL);
	if (!func) {
		kfree(cmd_buf);
		kfree(str_hand);
		return count;
	}

	ret = copy_from_user(cmd_buf, buf, count);
	if (ret < 0)
		goto end;

	*(cmd_buf + count) = '\0';
	ret = sscanf(cmd_buf, "%x %x %x %x %x",
			&cmd, &addr, &fn, &value, &de_sense);
	if (ret == 0) {
		ret = sscanf(cmd_buf, "%s", str_hand);
		if (ret == 0)
			dev_info(mmc_dev(host), "please enter cmd.\n");

		goto end;
	}

	/* create func struct for test */
	ret = create_func(func, card, fn, cmd);
	if (ret)
		goto end;

	if (cmd != re_init)
		sdio_claim_host(func);

	switch (cmd) {
	case Read:
		dev_info(mmc_dev(host), "read addr:%x, fn:%d.\n", addr, fn);
		ret = 0;
		result = sdio_readb(func, addr, &ret);

		if (ret)
			dev_info(mmc_dev(host), "Read fail(%d).\n", ret);
		else
			dev_info(mmc_dev(host), "f%d reg(%x) is 0x%02x.\n",
					func->num, addr, result);
		break;
	case Write:
		dev_info(mmc_dev(host), "write addr:%x, value:%x, fn:%d.\n",
				addr, (u8)value, fn);
		ret = 0;
		sdio_writeb(func, (u8)value, addr, &ret);

		if (ret)
			dev_info(mmc_dev(host), "write fail(%d).\n", ret);
		else
			dev_info(mmc_dev(host), "write success(%d).\n", ret);

		break;
	case Reset:
		mmc_hw_reset(host);
		break;
	case Stress_read:
		ret = multi_read(func, host);
		if (ret)
			goto end;
		break;
	case Stress_write:
		ret = multi_write(func, host, de_sense);
		if (ret)
			goto end;
		break;
	case re_init:
		/**
		 * this function can reset sdio host & sdio device.
		 * if non-removable is removed, the suspend/resume will fail.
		 * so it need set host->rescan_disable be zero before it does
		 * mmc_add_host(). After it is done, it need set
		 * host->rescan_disable back.
		 */
		dev_info(mmc_dev(host), "start remove host.\n");
		mmc_remove_host(host);
		dev_info(mmc_dev(host), "remove host done, then add host.\n");
		host->rescan_disable = 0;
		mmc_add_host(host);
		dev_info(mmc_dev(host), "add host done.\n");
		host->rescan_disable = 1;
		break;
	case Sress_test:
		ret = sdio_stress_test(func, host);
		if (ret) {
			dev_info(mmc_dev(host), "IO Stress Test Fail!!!\n");
			sdio_release_host(func);
			goto end;
		}
		break;
	default:
		dev_info(mmc_dev(host), "cmd is not valid.\n");
		break;
	}

	if (cmd != re_init)
		sdio_release_host(func);

end:
	kfree(str_hand);
	kfree(cmd_buf);
	if (cmd != re_init)
		kfree(func);

	return count;
}

/**
 * open function show some stable information.
 */
static int sdio_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sdio_proc_show, inode->i_private);
}

/**
 * sdio pre is our own function.
 * seq or single pre is the kernel function.
 */
static const struct file_operations sdio_proc_fops = {
	.owner = THIS_MODULE,
	.open = sdio_proc_open,
	.release = single_release,
	.write = sdio_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,
};

int sdio_proc_init(struct mmc_host *host_init)
{
	struct proc_dir_entry *prEntry;

	host = host_init;

	prEntry = proc_create("sdio", 0660, NULL, &sdio_proc_fops);
	if (prEntry)
		dev_info(mmc_dev(host), "/proc/sdio is created.\n");
	else
		dev_info(mmc_dev(host), "create /proc/sdio failed.\n");

	return 0;
}
EXPORT_SYMBOL(sdio_proc_init);
