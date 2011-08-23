/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
  $
 */

/**
 *  @defgroup   ACCELDL (Motion Library - Accelerometer Driver Layer)
 *  @brief      Provides the interface to setup and handle an accelerometers
 *              connected to the secondary I2C interface of the gyroscope.
 *
 *  @{
 *      @file   kxtf9.c
 *      @brief  Accelerometer setup and handling methods.
*/

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#undef MPL_LOG_NDEBUG
#define MPL_LOG_NDEBUG 1

#ifdef __KERNEL__
#include <linux/module.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <mach/board-ventana-misc.h>
#endif

#include "mpu.h"
#include "mlsl.h"
#include "mlos.h"

#include <log.h>
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-acc"

#define KXTF9_XOUT_HPF_L                (0x00) /* 0000 0000 */
#define KXTF9_XOUT_HPF_H                (0x01) /* 0000 0001 */
#define KXTF9_YOUT_HPF_L                (0x02) /* 0000 0010 */
#define KXTF9_YOUT_HPF_H                (0x03) /* 0000 0011 */
#define KXTF9_ZOUT_HPF_L                (0x04) /* 0001 0100 */
#define KXTF9_ZOUT_HPF_H                (0x05) /* 0001 0101 */
#define KXTF9_XOUT_L                    (0x06) /* 0000 0110 */
#define KXTF9_XOUT_H                    (0x07) /* 0000 0111 */
#define KXTF9_YOUT_L                    (0x08) /* 0000 1000 */
#define KXTF9_YOUT_H                    (0x09) /* 0000 1001 */
#define KXTF9_ZOUT_L                    (0x0A) /* 0001 1010 */
#define KXTF9_ZOUT_H                    (0x0B) /* 0001 1011 */
#define KXTF9_ST_RESP                   (0x0C) /* 0000 1100 */
#define KXTF9_WHO_AM_I                  (0x0F) /* 0000 1111 */
#define KXTF9_TILT_POS_CUR              (0x10) /* 0001 0000 */
#define KXTF9_TILT_POS_PRE              (0x11) /* 0001 0001 */
#define KXTF9_INT_SRC_REG1              (0x15) /* 0001 0101 */
#define KXTF9_INT_SRC_REG2              (0x16) /* 0001 0110 */
#define KXTF9_STATUS_REG                (0x18) /* 0001 1000 */
#define KXTF9_INT_REL                   (0x1A) /* 0001 1010 */
#define KXTF9_CTRL_REG1                 (0x1B) /* 0001 1011 */
#define KXTF9_CTRL_REG2                 (0x1C) /* 0001 1100 */
#define KXTF9_CTRL_REG3                 (0x1D) /* 0001 1101 */
#define KXTF9_INT_CTRL_REG1             (0x1E) /* 0001 1110 */
#define KXTF9_INT_CTRL_REG2             (0x1F) /* 0001 1111 */
#define KXTF9_INT_CTRL_REG3             (0x20) /* 0010 0000 */
#define KXTF9_DATA_CTRL_REG             (0x21) /* 0010 0001 */
#define KXTF9_TILT_TIMER                (0x28) /* 0010 1000 */
#define KXTF9_WUF_TIMER                 (0x29) /* 0010 1001 */
#define KXTF9_TDT_TIMER                 (0x2B) /* 0010 1011 */
#define KXTF9_TDT_H_THRESH              (0x2C) /* 0010 1100 */
#define KXTF9_TDT_L_THRESH              (0x2D) /* 0010 1101 */
#define KXTF9_TDT_TAP_TIMER             (0x2E) /* 0010 1110 */
#define KXTF9_TDT_TOTAL_TIMER           (0x2F) /* 0010 1111 */
#define KXTF9_TDT_LATENCY_TIMER         (0x30) /* 0011 0000 */
#define KXTF9_TDT_WINDOW_TIMER          (0x31) /* 0011 0001 */
#define KXTF9_WUF_THRESH                (0x5A) /* 0101 1010 */
#define KXTF9_TILT_ANGLE                (0x5C) /* 0101 1100 */
#define KXTF9_HYST_SET                  (0x5F) /* 0101 1111 */

#define KXTF9_MAX_DUR (0xFF)
#define KXTF9_MAX_THS (0xFF)
#define KXTF9_THS_COUNTS_P_G (32)

#define KXTF9_CALIBRATION_PATH "/data/sensors/KXTF9_Calibration.ini"
bool flagLoadAccelConfig = false;
static int offset_x = 0, offset_y = 0, offset_z =0;
/* --------------------- */
/* -    Variables.     - */
/* --------------------- */

struct kxtf9_config {
	unsigned int odr; /* Output data rate mHz */
	unsigned int fsr; /* full scale range mg */
	unsigned int ths; /* Motion no-motion thseshold mg */
	unsigned int dur; /* Motion no-motion duration ms */
	unsigned int irq_type;
	unsigned char reg_ths;
	unsigned char reg_dur;
	unsigned char reg_odr;
	unsigned char reg_int_cfg1;
	unsigned char reg_int_cfg2;
	unsigned char ctrl_reg1;
};

struct kxtf9_private_data {
	struct kxtf9_config suspend;
	struct kxtf9_config resume;
};
static int access_calibration_file(void)
{
	char buf[256];
	int ret = 0;
	struct file *fp = NULL;
	mm_segment_t oldfs;
	int max_x = 0, max_y = 0, max_z = 0;
	int min_x = 0, min_y = 0, min_z = 0;

	oldfs=get_fs();
	set_fs(get_ds());
	memset(buf, 0, sizeof(u8)*256);

	fp=filp_open(KXTF9_CALIBRATION_PATH, O_RDONLY, 0);
	if (!IS_ERR(fp)) {
		printk("kxtf9 open config file success\n");
		ret = fp->f_op->read(fp, buf, sizeof(buf), &fp->f_pos);
		printk("kxtf9 config content is :%s\n", buf);
		sscanf(buf,"%6d %6d %6d %6d %6d %6d\n",
			&max_x, &min_x, &max_y,	&min_y, &max_z, &min_z);

		offset_x = min_x + (max_x - min_x)/2;
		offset_y = min_y + (max_y - min_y)/2;
		offset_z = min_z + (max_z - min_z)/2;
		if( ASUSGetProjectID() == 102){
			offset_x = offset_x * (-1);
			offset_y = offset_x * (1);
			offset_z = offset_x * (-1);
		}

		printk("kxtf9: offset: %d %d %d\n", offset_x, offset_y, offset_z);
		filp_close(fp, NULL);
		set_fs(oldfs);
		return 0;
	}
	else{
		offset_x = 0;
		offset_y = 0;
		offset_z = 0;
		printk("No kxtf9 calibration file\n");
		set_fs(oldfs);
		return -1;
	}

}

/*****************************************
    Accelerometer Initialization Functions
*****************************************/

static int kxtf9_set_ths(void *mlsl_handle,
			struct ext_slave_platform_data *pdata,
			struct kxtf9_config *config,
			int apply,
			long ths)
{
	int result = ML_SUCCESS;
	if ((ths * KXTF9_THS_COUNTS_P_G / 1000) > KXTF9_MAX_THS)
		ths = (KXTF9_MAX_THS * 1000) / KXTF9_THS_COUNTS_P_G;

	if (ths < 0)
		ths = 0;

	config->ths = ths;
	config->reg_ths = (unsigned char)
		((long)(ths * KXTF9_THS_COUNTS_P_G) / 1000);
	MPL_LOGV("THS: %d, 0x%02x\n", config->ths, (int)config->reg_ths);
	if (apply)
		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
					KXTF9_WUF_THRESH,
					config->reg_ths);
	return result;
}

static int kxtf9_set_dur(void *mlsl_handle,
			struct ext_slave_platform_data *pdata,
			struct kxtf9_config *config,
			int apply,
			long dur)
{
	int result = ML_SUCCESS;
	long reg_dur = (dur * config->odr) / 1000000;
	config->dur = dur;

	if (reg_dur > KXTF9_MAX_DUR)
		reg_dur = KXTF9_MAX_DUR;

	config->reg_dur = (unsigned char) reg_dur;
	MPL_LOGV("DUR: %d, 0x%02x\n", config->dur, (int)config->reg_dur);
	if (apply)
		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
					KXTF9_WUF_TIMER,
					(unsigned char)reg_dur);
	return result;
}

/**
 * Sets the IRQ to fire when one of the IRQ events occur.  Threshold and
 * duration will not be used uless the type is MOT or NMOT.
 *
 * @param config configuration to apply to, suspend or resume
 * @param irq_type The type of IRQ.  Valid values are
 * - MPU_SLAVE_IRQ_TYPE_NONE
 * - MPU_SLAVE_IRQ_TYPE_MOTION
 * - MPU_SLAVE_IRQ_TYPE_DATA_READY
 */
static int kxtf9_set_irq(void *mlsl_handle,
			struct ext_slave_platform_data *pdata,
			struct kxtf9_config *config,
			int apply,
			long irq_type)
{
	int result = ML_SUCCESS;
	struct kxtf9_private_data *private_data = pdata->private_data;

	config->irq_type = (unsigned char)irq_type;
	config->ctrl_reg1 &= ~0x22;
	if (irq_type == MPU_SLAVE_IRQ_TYPE_DATA_READY) {
		config->ctrl_reg1 |= 0x20;
		config->reg_int_cfg1 = 0x38;
		config->reg_int_cfg2 = 0x00;
	} else if (irq_type == MPU_SLAVE_IRQ_TYPE_MOTION) {
		config->ctrl_reg1 |= 0x02;
		if ((unsigned long) config ==
			(unsigned long) &private_data->suspend)
			config->reg_int_cfg1 = 0x34;
		else
			config->reg_int_cfg1 = 0x24;
		config->reg_int_cfg2 = 0xE0;
	} else {
		config->reg_int_cfg1 = 0x00;
		config->reg_int_cfg2 = 0x00;
	}

	if (apply) {
		/* Must clear bit 7 before writing new configuration */
		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
					KXTF9_CTRL_REG1, 0x40);
		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
					KXTF9_INT_CTRL_REG1,
					config->reg_int_cfg1);
		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
					KXTF9_INT_CTRL_REG2,
					config->reg_int_cfg2);
		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
					KXTF9_CTRL_REG1,
					config->ctrl_reg1);
	}
	MPL_LOGV("CTRL_REG1: %lx, INT_CFG1: %lx, INT_CFG2: %lx\n",
		(unsigned long)config->ctrl_reg1,
		(unsigned long)config->reg_int_cfg1,
		(unsigned long)config->reg_int_cfg2);

	return result;
}

/**
 * Set the Output data rate for the particular configuration
 *
 * @param config Config to modify with new ODR
 * @param odr Output data rate in units of 1/1000Hz
 */
static int kxtf9_set_odr(void *mlsl_handle,
			struct ext_slave_platform_data *pdata,
			struct kxtf9_config *config,
			int apply,
			long odr)
{
	unsigned char bits;
	int result = ML_SUCCESS;

	/* Data sheet says there is 12.5 hz, but that seems to produce a single
	 * correct data value, thus we remove it from the table */
	if (odr > 400000) {
		config->odr = 800000;
		bits = 0x06;
	} else if (odr > 200000) {
		config->odr = 400000;
		bits = 0x05;
	} else if (odr > 100000) {
		config->odr = 200000;
		bits = 0x04;
	} else if (odr > 50000) {
		config->odr = 100000;
		bits = 0x03;
	} else if (odr > 25000) {
		config->odr = 50000;
		bits = 0x02;
	} else if (odr != 0) {
		config->odr = 25000;
		bits = 0x01;
	} else {
		config->odr = 0;
		bits = 0;
	}

	if (odr != 0)
		config->ctrl_reg1 |= 0x80;

	config->reg_odr = bits;
	kxtf9_set_dur(mlsl_handle, pdata,
		config, apply, config->dur);
	MPL_LOGV("ODR: %d, 0x%02x\n", config->odr, (int)config->ctrl_reg1);
	if (apply) {
		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
					KXTF9_DATA_CTRL_REG,
					config->reg_odr);
		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
					KXTF9_CTRL_REG1,
					0x40);
		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
					KXTF9_CTRL_REG1,
					config->ctrl_reg1);
	}
	return result;
}

/**
 * Set the full scale range of the accels
 *
 * @param config pointer to configuration
 * @param fsr requested full scale range
 */
static int kxtf9_set_fsr(void *mlsl_handle,
			struct ext_slave_platform_data *pdata,
			struct kxtf9_config *config,
			int apply,
			long fsr)
{
	int result = ML_SUCCESS;

	config->ctrl_reg1 = (config->ctrl_reg1 & 0xE7);
	if (fsr <= 2000) {
		config->fsr = 2000;
		config->ctrl_reg1 |= 0x00;
	} else if (fsr <= 4000) {
		config->fsr = 4000;
		config->ctrl_reg1 |= 0x08;
	} else {
		config->fsr = 8000;
		config->ctrl_reg1 |= 0x10;
	}

	MPL_LOGV("FSR: %d\n", config->fsr);
	if (apply) {
		/* Must clear bit 7 before writing new configuration */
		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
					KXTF9_CTRL_REG1, 0x40);
		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
					KXTF9_CTRL_REG1, config->ctrl_reg1);
	}
	return result;
}

static int kxtf9_suspend(void *mlsl_handle,
			 struct ext_slave_descr *slave,
			 struct ext_slave_platform_data *pdata)
{
	int result;
	unsigned char data;
	struct kxtf9_private_data *private_data = pdata->private_data;

	printk("%s+\n", __func__);
	/* Wake up */
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				KXTF9_CTRL_REG1, 0x40);
	ERROR_CHECK(result);
	/* INT_CTRL_REG1: */
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				KXTF9_INT_CTRL_REG1,
				private_data->suspend.reg_int_cfg1);
	ERROR_CHECK(result);
	/* WUF_THRESH: */
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				KXTF9_WUF_THRESH,
				private_data->suspend.reg_ths);
	ERROR_CHECK(result);
	/* DATA_CTRL_REG */
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				KXTF9_DATA_CTRL_REG,
				private_data->suspend.reg_odr);
	ERROR_CHECK(result);
	/* WUF_TIMER */
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				KXTF9_WUF_TIMER, private_data->suspend.reg_dur);
	ERROR_CHECK(result);

	/* Normal operation  */
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				KXTF9_CTRL_REG1,
				private_data->suspend.ctrl_reg1);
	ERROR_CHECK(result);
	result = MLSLSerialRead(mlsl_handle, pdata->address,
				KXTF9_INT_REL, 1, &data);
	ERROR_CHECK(result);
	printk("%s-\n", __func__);

	return result;
}

/* full scale setting - register and mask */
#define ACCEL_KIONIX_CTRL_REG      (0x1b)
#define ACCEL_KIONIX_CTRL_MASK     (0x18)

static int kxtf9_resume(void *mlsl_handle,
			struct ext_slave_descr *slave,
			struct ext_slave_platform_data *pdata)
{
	int result = ML_SUCCESS;
	unsigned char data;
	struct kxtf9_private_data *private_data = pdata->private_data;

	printk("%s+\n", __func__);
	/* Wake up */
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				KXTF9_CTRL_REG1, 0x40);
	ERROR_CHECK(result);
	/* INT_CTRL_REG1: */
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				KXTF9_INT_CTRL_REG1,
				private_data->resume.reg_int_cfg1);
	ERROR_CHECK(result);
	/* WUF_THRESH: */
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				KXTF9_WUF_THRESH, private_data->resume.reg_ths);
	ERROR_CHECK(result);
	/* DATA_CTRL_REG */
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				KXTF9_DATA_CTRL_REG,
				private_data->resume.reg_odr);
	ERROR_CHECK(result);
	/* WUF_TIMER */
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				KXTF9_WUF_TIMER, private_data->resume.reg_dur);
	ERROR_CHECK(result);

	/* Normal operation  */
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				KXTF9_CTRL_REG1,
				private_data->resume.ctrl_reg1);
	ERROR_CHECK(result);
	result = MLSLSerialRead(mlsl_handle, pdata->address,
				KXTF9_INT_REL, 1, &data);
	ERROR_CHECK(result);
	printk("%s-\n", __func__);
	return ML_SUCCESS;
}

static int kxtf9_init(void *mlsl_handle,
		struct ext_slave_descr *slave,
		struct ext_slave_platform_data *pdata)
{

	struct kxtf9_private_data *private_data;
	int result = ML_SUCCESS;

	private_data = (struct kxtf9_private_data *)
		MLOSMalloc(sizeof(struct kxtf9_private_data));

	if (!private_data)
		return ML_ERROR_MEMORY_EXAUSTED;

	/* RAM reset */
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				KXTF9_CTRL_REG1,
				0x40); /* Fastest Reset */
	ERROR_CHECK(result);
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				KXTF9_DATA_CTRL_REG,
				0x36); /* Fastest Reset */
	ERROR_CHECK(result);
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				KXTF9_CTRL_REG3, 0xcd); /* Reset */
	ERROR_CHECK(result);
	MLOSSleep(2);

	pdata->private_data = private_data;

	private_data->resume.ctrl_reg1 = 0xC0;
	private_data->suspend.ctrl_reg1 = 0x40;

	result = kxtf9_set_dur(mlsl_handle, pdata, &private_data->suspend,
			FALSE, 1000);
	ERROR_CHECK(result);
	result = kxtf9_set_dur(mlsl_handle, pdata, &private_data->resume,
			FALSE,  2540);
	ERROR_CHECK(result);

	result = kxtf9_set_odr(mlsl_handle, pdata, &private_data->suspend,
			FALSE, 50000);
	ERROR_CHECK(result);
	result = kxtf9_set_odr(mlsl_handle, pdata, &private_data->resume,
			FALSE, 200000);

	result = kxtf9_set_fsr(mlsl_handle, pdata, &private_data->suspend,
			FALSE, 2000);
	ERROR_CHECK(result);
	result = kxtf9_set_fsr(mlsl_handle, pdata, &private_data->resume,
			FALSE, 2000);
	ERROR_CHECK(result);

	result = kxtf9_set_ths(mlsl_handle, pdata, &private_data->suspend,
			FALSE, 80);
	ERROR_CHECK(result);
	result = kxtf9_set_ths(mlsl_handle, pdata, &private_data->resume,
			FALSE, 40);
	ERROR_CHECK(result);

	result = kxtf9_set_irq(mlsl_handle, pdata, &private_data->suspend,
			FALSE,
			MPU_SLAVE_IRQ_TYPE_NONE);
	ERROR_CHECK(result);
	result = kxtf9_set_irq(mlsl_handle, pdata, &private_data->resume,
			FALSE,
			MPU_SLAVE_IRQ_TYPE_NONE);
	ERROR_CHECK(result);
	return result;
}

static int kxtf9_exit(void *mlsl_handle,
			  struct ext_slave_descr *slave,
			  struct ext_slave_platform_data *pdata)
{
	if (pdata->private_data)
		return MLOSFree(pdata->private_data);
	else
		return ML_SUCCESS;
}

static int kxtf9_config(void *mlsl_handle,
			struct ext_slave_descr *slave,
			struct ext_slave_platform_data *pdata,
			struct ext_slave_config *data)
{
	struct kxtf9_private_data *private_data = pdata->private_data;
	if (!data->data)
		return ML_ERROR_INVALID_PARAMETER;

	switch (data->key) {
	case MPU_SLAVE_CONFIG_ODR_SUSPEND:
		return kxtf9_set_odr(mlsl_handle, pdata,
					&private_data->suspend,
					data->apply,
					*((long *)data->data));
	case MPU_SLAVE_CONFIG_ODR_RESUME:
		return kxtf9_set_odr(mlsl_handle, pdata,
					&private_data->resume,
					data->apply,
					*((long *)data->data));
	case MPU_SLAVE_CONFIG_FSR_SUSPEND:
		return kxtf9_set_fsr(mlsl_handle, pdata,
					&private_data->suspend,
					data->apply,
					*((long *)data->data));
	case MPU_SLAVE_CONFIG_FSR_RESUME:
		return kxtf9_set_fsr(mlsl_handle, pdata,
					&private_data->resume,
					data->apply,
					*((long *)data->data));
	case MPU_SLAVE_CONFIG_MOT_THS:
		return kxtf9_set_ths(mlsl_handle, pdata,
					&private_data->suspend,
					data->apply,
					*((long *)data->data));
	case MPU_SLAVE_CONFIG_NMOT_THS:
		return kxtf9_set_ths(mlsl_handle, pdata,
					&private_data->resume,
					data->apply,
					*((long *)data->data));
	case MPU_SLAVE_CONFIG_MOT_DUR:
		return kxtf9_set_dur(mlsl_handle, pdata,
					&private_data->suspend,
					data->apply,
					*((long *)data->data));
	case MPU_SLAVE_CONFIG_NMOT_DUR:
		return kxtf9_set_dur(mlsl_handle, pdata,
					&private_data->resume,
					data->apply,
					*((long *)data->data));
	case MPU_SLAVE_CONFIG_IRQ_SUSPEND:
		return kxtf9_set_irq(mlsl_handle, pdata,
					&private_data->suspend,
					data->apply,
					*((long *)data->data));
	case MPU_SLAVE_CONFIG_IRQ_RESUME:
		return kxtf9_set_irq(mlsl_handle, pdata,
					&private_data->resume,
					data->apply,
					*((long *)data->data));
	default:
		return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
	};

	return ML_SUCCESS;
}

static int kxtf9_get_config(void *mlsl_handle,
				struct ext_slave_descr *slave,
				struct ext_slave_platform_data *pdata,
				struct ext_slave_config *data)
{
	struct kxtf9_private_data *private_data = pdata->private_data;
	if (!data->data)
		return ML_ERROR_INVALID_PARAMETER;

	switch (data->key) {
	case MPU_SLAVE_CONFIG_ODR_SUSPEND:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->suspend.odr;
		break;
	case MPU_SLAVE_CONFIG_ODR_RESUME:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->resume.odr;
		break;
	case MPU_SLAVE_CONFIG_FSR_SUSPEND:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->suspend.fsr;
		break;
	case MPU_SLAVE_CONFIG_FSR_RESUME:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->resume.fsr;
		break;
	case MPU_SLAVE_CONFIG_MOT_THS:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->suspend.ths;
		break;
	case MPU_SLAVE_CONFIG_NMOT_THS:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->resume.ths;
		break;
	case MPU_SLAVE_CONFIG_MOT_DUR:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->suspend.dur;
		break;
	case MPU_SLAVE_CONFIG_NMOT_DUR:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->resume.dur;
		break;
	case MPU_SLAVE_CONFIG_IRQ_SUSPEND:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->suspend.irq_type;
		break;
	case MPU_SLAVE_CONFIG_IRQ_RESUME:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->resume.irq_type;
		break;
	default:
		return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
	};

	return ML_SUCCESS;
}

static int kxtf9_read(void *mlsl_handle,
		      struct ext_slave_descr *slave,
		      struct ext_slave_platform_data *pdata,
		      unsigned char *data)
{
	int result;
	unsigned char reg;
	int x, y, z;

	if(!flagLoadAccelConfig){
		access_calibration_file();
		flagLoadAccelConfig = true;
	}
	result = MLSLSerialRead(mlsl_handle, pdata->address,
				KXTF9_INT_SRC_REG2, 1, &reg);
	ERROR_CHECK(result);

	if (!(reg & 0x10))
		return ML_ERROR_ACCEL_DATA_NOT_READY;

	result = MLSLSerialRead(mlsl_handle, pdata->address,
				slave->reg, slave->len, data);
	x = ((data[1] << 4) | (data[0] >> 4));
	y = ((data[3] << 4) | (data[2] >> 4));
	z = ((data[5] << 4) | (data[4] >> 4));

	if (x & 0x800)
		x |= 0xFFFFF000;
	if (y & 0x800)
		y |= 0xFFFFF000;
	if (z & 0x800)
		z |= 0xFFFFF000;

	x -= offset_x;
	y -= offset_y;
	z -= offset_z;

	data[0] = (x << 4) & 0x000000F0;
	data[1] = (x >> 4) & 0x000000FF;
	data[2] = (y << 4) & 0x000000F0;
	data[3] = (y >> 4) & 0x000000FF;
	data[4] = (z << 4) & 0x000000F0;
	data[5] = (z >> 4) & 0x000000FF;
	ERROR_CHECK(result);
	return result;
}

static struct ext_slave_descr kxtf9_descr = {
	/*.init             = */ kxtf9_init,
	/*.exit             = */ kxtf9_exit,
	/*.suspend          = */ kxtf9_suspend,
	/*.resume           = */ kxtf9_resume,
	/*.read             = */ kxtf9_read,
	/*.config           = */ kxtf9_config,
	/*.get_config       = */ kxtf9_get_config,
	/*.name             = */ "kxtf9",
	/*.type             = */ EXT_SLAVE_TYPE_ACCELEROMETER,
	/*.id               = */ ACCEL_ID_KXTF9,
	/*.reg              = */ 0x06,
	/*.len              = */ 6,
	/*.endian           = */ EXT_SLAVE_LITTLE_ENDIAN,
	/*.range            = */ {2, 0},
};

struct ext_slave_descr *kxtf9_get_slave_descr(void)
{
	return &kxtf9_descr;
}
EXPORT_SYMBOL(kxtf9_get_slave_descr);

/**
 *  @}
**/
