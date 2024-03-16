/*
 * Copyright (c) 2024 Erik Andersson <erian747@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Based on the NT35510 driver provided by ST-Microelectronics at
 * https://github.com/STMicroelectronics/stm32-nt35510/blob/main/nt35510.c
 */

#define DT_DRV_COMPAT frida_nt35510

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/mipi_dsi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include "display_nt35510.h"

LOG_MODULE_REGISTER(nt35510, CONFIG_DISPLAY_LOG_LEVEL);

/**
 * @brief  Possible values of COLMOD parameter corresponding to used pixel formats
 */
#define NT35510_COLMOD_RGB565 0x55
#define NT35510_COLMOD_RGB888 0x77

/**
 * @brief  NT35510_480X800 Timing parameters for Portrait orientation mode
 */
#define NT35510_480X800_HSYNC ((uint16_t)2)  /* Horizontal synchronization */
#define NT35510_480X800_HBP   ((uint16_t)34) /* Horizontal back porch      */
#define NT35510_480X800_HFP   ((uint16_t)34) /* Horizontal front porch     */

#define NT35510_480X800_VSYNC ((uint16_t)120) /* Vertical synchronization   */
#define NT35510_480X800_VBP   ((uint16_t)150) /* Vertical back porch        */
#define NT35510_480X800_VFP   ((uint16_t)150) /* Vertical front porch       */

/**
 * @brief  NT35510_800X480 Timing parameters for Landscape orientation mode
 *         Same values as for Portrait mode in fact.
 */
#define NT35510_800X480_HSYNC NT35510_480X800_VSYNC /* Horizontal synchronization */
#define NT35510_800X480_HBP   NT35510_480X800_VBP   /* Horizontal back porch      */
#define NT35510_800X480_HFP   NT35510_480X800_VFP   /* Horizontal front porch     */
#define NT35510_800X480_VSYNC NT35510_480X800_HSYNC /* Vertical synchronization   */
#define NT35510_800X480_VBP   NT35510_480X800_HBP   /* Vertical back porch        */
#define NT35510_800X480_VFP   NT35510_480X800_HFP   /* Vertical front porch       */

struct nt35510_config {
	const struct device *mipi_dsi;
	const struct gpio_dt_spec reset;
	const struct gpio_dt_spec backlight;
	uint8_t data_lanes;
	uint16_t width;
	uint16_t height;
	uint8_t channel;
	uint16_t rotation;
};

struct nt35510_data {
	enum display_pixel_format dsi_pixel_format;
	enum display_orientation orientation;
	uint16_t xres;
	uint16_t yres;
};

/* Write data buffer to LCD register */
static int nt35510_write_reg(const struct device *dev, uint8_t reg, const uint8_t *buf, size_t len)
{
	int ret;
	const struct nt35510_config *cfg = dev->config;

	ret = mipi_dsi_dcs_write(cfg->mipi_dsi, cfg->channel, reg, buf, len);
	if (ret < 0) {
		LOG_ERR("Failed writing reg: 0x%x result: (%d)", reg, ret);
		return ret;
	}
	return 0;
}

/* Write an 8-bit value to a register */
static int nt35510_write_reg_val(const struct device *dev, uint8_t reg, uint8_t value)
{
	return nt35510_write_reg(dev, reg, &value, 1);
}

/* Initialization and configuration sequence */
static int nt35510_config(const struct device *dev)
{
	struct nt35510_data *data = dev->data;
	int ret = 0;

	static const uint8_t nt35510_reg1[] = {0x03, 0x03, 0x03};
	static const uint8_t nt35510_reg2[] = {0x46, 0x46, 0x46};
	static const uint8_t nt35510_reg[] = {0x55, 0xAA, 0x52, 0x08, 0x01};
	static const uint8_t nt35510_reg3[] = {0x03, 0x03, 0x03};
	static const uint8_t nt35510_reg4[] = {0x36, 0x36, 0x36};
	static const uint8_t nt35510_reg5[] = {0x00, 0x00, 0x02};
	static const uint8_t nt35510_reg6[] = {0x26, 0x26, 0x26};
	static const uint8_t nt35510_reg7[] = {0x01, 0x01};
	static const uint8_t nt35510_reg8[] = {0x09, 0x09, 0x09};
	static const uint8_t nt35510_reg9[] = {0x36, 0x36, 0x36};
	static const uint8_t nt35510_reg10[] = {0x08, 0x08, 0x08};
	static const uint8_t nt35510_reg12[] = {0x26, 0x26, 0x26};
	static const uint8_t nt35510_reg13[] = {0x00, 0x80, 0x00};
	static const uint8_t nt35510_reg14[] = {0x00, 0x80, 0x00};
	static const uint8_t nt35510_reg15[] = {0x00, 0x50};
	static const uint8_t nt35510_reg16[] = {0x55, 0xAA, 0x52, 0x08, 0x00};
	static const uint8_t nt35510_reg17[] = {0xFC, 0x00};
	static const uint8_t nt35510_reg18[] = {0x03, 0x03};
	static const uint8_t nt35510_reg19[] = {0x51, 0x51};
	static const uint8_t nt35510_reg20[] = {0x00, 0x00};

	static const uint8_t nt35510_reg21[] = {0x01, 0x02, 0x02, 0x02};
	static const uint8_t nt35510_reg22[] = {0x00, 0x00, 0x00};
	static const uint8_t nt35510_reg23[] = {0x03, 0x00, 0x00};
	static const uint8_t nt35510_reg24[] = {0x01, 0x01};

	static const uint8_t nt35510_caset_portrait[] = {0x00, 0x00, 0x01, 0xDF};
	static const uint8_t nt35510_raset_portrait[] = {0x00, 0x00, 0x03, 0x1F};
	static const uint8_t nt35510_caset_landscape[] = {0x00, 0x00, 0x03, 0x1F};
	static const uint8_t nt35510_raset_landscape[] = {0x00, 0x00, 0x01, 0xDF};

	ret = nt35510_write_reg(dev, 0xF0, nt35510_reg, 5);   /* LV2:  Page 1 enable */
	ret += nt35510_write_reg(dev, 0xB0, nt35510_reg1, 3); /* AVDD: 5.2V */
	ret += nt35510_write_reg(dev, 0xB6, nt35510_reg2, 3); /* AVDD: Ratio */
	ret += nt35510_write_reg(dev, 0xB1, nt35510_reg3, 3); /* AVEE: -5.2V */
	ret += nt35510_write_reg(dev, 0xB7, nt35510_reg4, 3); /* AVEE: Ratio */
	ret += nt35510_write_reg(dev, 0xB2, nt35510_reg5, 3); /* VCL: -2.5V */
	ret += nt35510_write_reg(dev, 0xB8, nt35510_reg6, 3); /* VCL: Ratio */
	ret += nt35510_write_reg(dev, 0xBF, nt35510_reg7, 1); /* VGH: 15V (Free Pump) */
	ret += nt35510_write_reg(dev, 0xB3, nt35510_reg8, 3);
	ret += nt35510_write_reg(dev, 0xB9, nt35510_reg9, 3);  /* VGH: Ratio */
	ret += nt35510_write_reg(dev, 0xB5, nt35510_reg10, 3); /* VGL_REG: -10V */
	ret += nt35510_write_reg(dev, 0xBA, nt35510_reg12, 3); /* VGLX: Ratio */
	ret += nt35510_write_reg(dev, 0xBC, nt35510_reg13, 3); /* VGMP/VGSP: 4.5V/0V */
	ret += nt35510_write_reg(dev, 0xBD, nt35510_reg14, 3); /* VGMN/VGSN:-4.5V/0V */
	ret += nt35510_write_reg(dev, 0xBE, nt35510_reg15, 2); /* VCOM: -1.325V */

	/* ************************************************************************** */
	/* Proprietary DCS Initialization                                             */
	/* ************************************************************************** */

	ret += nt35510_write_reg(dev, 0xF0, nt35510_reg16, 5); /* LV2: Page 0 enable */
	ret += nt35510_write_reg(dev, 0xB1, nt35510_reg17, 2); /* Display optional control */
	ret += nt35510_write_reg(dev, 0xB6, nt35510_reg18,
				 1); /* Set source output data hold time */
	ret += nt35510_write_reg(dev, 0xB5, nt35510_reg19, 1); /* Display resolution control */
	ret += nt35510_write_reg(dev, 0xB7, nt35510_reg20, 2); /* Gate EQ control */
	ret += nt35510_write_reg(dev, 0xB8, nt35510_reg21, 4); /* Src EQ control(Mode2) */
	ret += nt35510_write_reg(dev, 0xBC, nt35510_reg22, 3);
	ret += nt35510_write_reg(dev, 0xCC, nt35510_reg23, 3);
	ret += nt35510_write_reg(dev, 0xBA, nt35510_reg24, 1);

	/* Add a delay, otherwise MADCTL not taken */
	k_msleep(200);

	/* Configure orientation */
	if (data->orientation == DISPLAY_ORIENTATION_NORMAL) {
		ret += nt35510_write_reg_val(dev, NT35510_CMD_MADCTL, 0x00);
		ret += nt35510_write_reg(dev, NT35510_CMD_CASET, nt35510_caset_portrait, 4);
		ret += nt35510_write_reg(dev, NT35510_CMD_RASET, nt35510_raset_portrait, 4);
	} else {
		ret += nt35510_write_reg_val(dev, NT35510_CMD_MADCTL, 0x60);
		ret += nt35510_write_reg(dev, NT35510_CMD_CASET, nt35510_caset_landscape, 4);
		ret += nt35510_write_reg(dev, NT35510_CMD_RASET, nt35510_raset_landscape, 4);
	}

	/* Exit sleep mode */
	ret += nt35510_write_reg(dev, NT35510_CMD_SLPOUT, NULL, 0);

	/* Wait for sleep out exit */
	k_msleep(20);

	/* Set color mode */
	ret += nt35510_write_reg_val(dev, NT35510_CMD_COLMOD,
				     data->dsi_pixel_format == PIXEL_FORMAT_RGB_565
					     ? NT35510_COLMOD_RGB565
					     : NT35510_COLMOD_RGB888);

	/** CABC : Content Adaptive Backlight Control section start >> */
	/* Note : defaut is 0 (lowest Brightness], 0xFF is highest Brightness, try 0x7F :
	 * intermediate value */
	// ret += nt35510_write_reg(dev, NT35510_CMD_WRDISBV, &nt35510_reg31[1], 1);
	ret += nt35510_write_reg_val(dev, NT35510_CMD_WRDISBV, 0x7f);

	/* defaut is 0, try 0x2C - Brightness Control Block, Display Dimming & BackLight on */
	// ret += nt35510_write_reg(dev, NT35510_CMD_WRCTRLD, &nt35510_reg32[1], 1);
	ret += nt35510_write_reg_val(dev, NT35510_CMD_WRCTRLD, 0x2c);

	/* defaut is 0, try 0x02 - image Content based Adaptive Brightness [Still Picture] */
	// ret += nt35510_write_reg(dev, NT35510_CMD_WRCABC, &nt35510_reg33[1], 1);
	ret += nt35510_write_reg_val(dev, NT35510_CMD_WRCABC, 0x2);

	/* defaut is 0 (lowest Brightness], 0xFF is highest Brightness */
	// ret += nt35510_write_reg(dev, NT35510_CMD_WRCABCMB, &nt35510_reg34[1], 1);
	ret += nt35510_write_reg_val(dev, NT35510_CMD_WRCABCMB, 0xff);

	/* Send Command Display On */
	ret += nt35510_write_reg(dev, MIPI_DCS_SET_DISPLAY_ON, NULL, 0);

	/* Send Command GRAM memory write (no parameters)
	   this initiates frame write via other DSI commands sent by
	   DSI host from LTDC incoming pixels in video mode */
	ret += nt35510_write_reg(dev, NT35510_CMD_RAMWR, NULL, 0);

	return ret;
}

static int nt35510_write(const struct device *dev, uint16_t x, uint16_t y,
			 const struct display_buffer_descriptor *desc, const void *buf)
{
	return -ENOTSUP;
}

static int nt35510_blanking_on(const struct device *dev)
{
	const struct nt35510_config *cfg = dev->config;
	int ret;

	if (cfg->backlight.port != NULL) {
		ret = gpio_pin_set_dt(&cfg->backlight, 0);
		if (ret) {
			LOG_ERR("Disable backlight failed! (%d)", ret);
			return ret;
		}
	}
	return nt35510_write_reg(dev, MIPI_DCS_SET_DISPLAY_OFF, NULL, 0);
}

static int nt35510_blanking_off(const struct device *dev)
{

	const struct nt35510_config *cfg = dev->config;
	int ret;
	if (cfg->backlight.port != NULL) {
		ret = gpio_pin_set_dt(&cfg->backlight, 1);
		if (ret) {
			LOG_ERR("Enable backlight failed! (%d)", ret);
			return ret;
		}
	}
	return nt35510_write_reg(dev, MIPI_DCS_SET_DISPLAY_ON, NULL, 0);
}

static int nt35510_set_brightness(const struct device *dev, const uint8_t brightness)
{
	return nt35510_write_reg(dev, MIPI_DCS_SET_DISPLAY_BRIGHTNESS, &brightness, 1);
	return 0;
}

static void nt35510_get_capabilities(const struct device *dev,
				     struct display_capabilities *capabilities)
{
	const struct nt35510_config *cfg = dev->config;
	struct nt35510_data *data = dev->data;

	memset(capabilities, 0, sizeof(struct display_capabilities));
	capabilities->x_resolution = cfg->width;
	capabilities->y_resolution = cfg->height;
	capabilities->supported_pixel_formats = data->dsi_pixel_format;
	capabilities->current_pixel_format = data->dsi_pixel_format;
	capabilities->current_orientation = data->orientation;
}

static int nt35510_set_pixel_format(const struct device *dev,
				    const enum display_pixel_format pixel_format)
{
	struct nt35510_data *data = dev->data;

	data->dsi_pixel_format = pixel_format;
	return 0;
}

static int nt35510_check_id(const struct device *dev)
{
	const struct nt35510_config *cfg = dev->config;
	uint8_t id = 0;
	int ret;

	ret = mipi_dsi_dcs_read(cfg->mipi_dsi, cfg->channel, NT35510_CMD_RDID2, &id, 1);
	if (ret != sizeof(id)) {
		LOG_ERR("Failed reading ID (%d)", ret);
		return -EIO;
	}

	if (id != NT35510_ID) {
		LOG_ERR("ID 0x%x, expected: 0x%x)", id, NT35510_ID);
		return -EINVAL;
	}
	return 0;
}

static int nt35510_init(const struct device *dev)
{
	const struct nt35510_config *cfg = dev->config;
	struct nt35510_data *data = dev->data;
	struct mipi_dsi_device mdev;
	int ret;

	if (cfg->reset.port) {
		if (!gpio_is_ready_dt(&cfg->reset)) {
			LOG_ERR("Reset GPIO device is not ready!");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("Reset display failed! (%d)", ret);
			return ret;
		}
		k_msleep(20);
		ret = gpio_pin_set_dt(&cfg->reset, 1);
		if (ret < 0) {
			LOG_ERR("Enable display failed! (%d)", ret);
			return ret;
		}
		k_msleep(200);
	}

	/* store x/y resolution & rotation */
	if (cfg->rotation == 0) {
		data->xres = cfg->width;
		data->yres = cfg->height;
		data->orientation = DISPLAY_ORIENTATION_NORMAL;
	} else if (cfg->rotation == 90) {
		data->xres = cfg->height;
		data->yres = cfg->width;
		data->orientation = DISPLAY_ORIENTATION_ROTATED_90;
	} else if (cfg->rotation == 180) {
		data->xres = cfg->width;
		data->yres = cfg->height;
		data->orientation = DISPLAY_ORIENTATION_ROTATED_180;
	} else if (cfg->rotation == 270) {
		data->xres = cfg->height;
		data->yres = cfg->width;
		data->orientation = DISPLAY_ORIENTATION_ROTATED_270;
	}

	/* Attach to MIPI-DSI host */
	mdev.data_lanes = cfg->data_lanes;
	mdev.pixfmt = data->dsi_pixel_format;
	mdev.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST | MIPI_DSI_MODE_LPM;

	mdev.timings.hactive = data->xres;
	mdev.timings.hbp = NT35510_480X800_HBP;
	mdev.timings.hfp = NT35510_480X800_HFP;
	mdev.timings.hsync = NT35510_480X800_HSYNC;
	mdev.timings.vactive = data->yres;
	mdev.timings.vbp = NT35510_480X800_VBP;
	mdev.timings.vfp = NT35510_480X800_VFP;
	mdev.timings.vsync = NT35510_480X800_VSYNC;

	ret = mipi_dsi_attach(cfg->mipi_dsi, cfg->channel, &mdev);
	if (ret < 0) {
		LOG_ERR("MIPI-DSI attach failed! (%d)", ret);
		return ret;
	}

	ret = nt35510_check_id(dev);
	if (ret) {
		LOG_ERR("Panel ID check failed! (%d)", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&cfg->backlight, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Backligt pin init fail (%d)", ret);
		return ret;
	}

	ret = nt35510_config(dev);
	if (ret) {
		LOG_ERR("DSI init sequence failed! (%d)", ret);
		return ret;
	}

	ret = nt35510_blanking_off(dev);
	if (ret) {
		LOG_ERR("Display blanking off failed! (%d)", ret);
		return ret;
	}

	LOG_INF("Init complete(%d)", ret);
	return 0;
}

static const struct display_driver_api nt35510_api = {
	.blanking_on = nt35510_blanking_on,
	.blanking_off = nt35510_blanking_off,
	.write = nt35510_write,
	.set_brightness = nt35510_set_brightness,
	.get_capabilities = nt35510_get_capabilities,
	.set_pixel_format = nt35510_set_pixel_format,
};

#define NT35510_DEFINE(n)                                                                          \
	static const struct nt35510_config nt35510_config_##n = {                                  \
		.mipi_dsi = DEVICE_DT_GET(DT_INST_BUS(n)),                                         \
		.reset = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),                            \
		.backlight = GPIO_DT_SPEC_INST_GET_OR(n, bl_gpios, {0}),                           \
		.data_lanes = DT_INST_PROP_BY_IDX(n, data_lanes, 0),                               \
		.width = DT_INST_PROP(n, width),                                                   \
		.height = DT_INST_PROP(n, height),                                                 \
		.channel = DT_INST_REG_ADDR(n),                                                    \
		.rotation = DT_INST_PROP(n, rotation),                                             \
	};                                                                                         \
                                                                                                   \
	static struct nt35510_data nt35510_data_##n = {                                            \
		.dsi_pixel_format = DT_INST_PROP(n, pixel_format),                                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, &nt35510_init, NULL, &nt35510_data_##n, &nt35510_config_##n,      \
			      POST_KERNEL, CONFIG_DISPLAY_NT35510_INIT_PRIORITY, &nt35510_api);

DT_INST_FOREACH_STATUS_OKAY(NT35510_DEFINE)
