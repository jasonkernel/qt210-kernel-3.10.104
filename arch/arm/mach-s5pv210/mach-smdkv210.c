/* linux/arch/arm/mach-s5pv210/mach-smdkv210.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/device.h>
#include <linux/dm9000.h>
#include <linux/smsc911x.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/pwm_backlight.h>
#include <linux/platform_data/s3c-hsotg.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>
#include <asm/mach-types.h>

#include <video/platform_lcd.h>
#include <video/samsung_fimd.h>

#include <mach/map.h>
#include <mach/regs-clock.h>

#include <plat/regs-serial.h>
#include <plat/regs-srom.h>
#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/adc.h>
#include <linux/platform_data/touchscreen-s3c2410.h>
#include <linux/platform_data/ata-samsung_cf.h>
#include <linux/platform_data/i2c-s3c2410.h>
#include <plat/keypad.h>
#include <plat/pm.h>
#include <plat/fb.h>
#include <plat/samsung-time.h>
#include <plat/backlight.h>
#include <plat/mfc.h>
#include <plat/clock.h>

#include "common.h"

#include <../../../drivers/video/samsung/s3cfb.h>
#include <mach/regs-gpio.h>
#include <plat/media.h>
#include <mach/media.h>
#include <mach/power-domain.h>

#define S5PV210_LCD_WIDTH  800
#define S5PV210_LCD_HEIGHT 480
/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDKV210_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDKV210_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDKV210_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMC0 (6144 * SZ_1K)
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMC1 (9900 * SZ_1K)
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMC2 (6144 * SZ_1K)
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_MFC0 (36864 * SZ_1K)
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_MFC1 (36864 * SZ_1K)
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMD (S5PV210_LCD_WIDTH * \
                                             S5PV210_LCD_HEIGHT * 4 * \
                                             (CONFIG_FB_S3C_NR_BUFFERS + \
                                                 (CONFIG_FB_S3C_NUM_OVLY_WIN * \
                                                  CONFIG_FB_S3C_NUM_BUF_OVLY_WIN))) //sayanta macro values need to be set
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_JPEG (8192 * SZ_1K)

/* 1920 * 1080 * 4 (RGBA)
 * - framesize == 1080p : 1920 * 1080 * 2(16bpp) * 2(double buffer) = 8MB
 * - framesize <  1080p : 1080 *  720 * 4(32bpp) * 2(double buffer) = under 8MB
 **/
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_G2D (8192 * SZ_1K)
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_TEXSTREAM (3000 * SZ_1K)
#define  S5PV210_ANDROID_PMEM_MEMSIZE_PMEM_GPU1 (3300 * SZ_1K)


static struct s3c2410_uartcfg smdkv210_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= SMDKV210_UCON_DEFAULT,
		.ulcon		= SMDKV210_ULCON_DEFAULT,
		.ufcon		= SMDKV210_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= SMDKV210_UCON_DEFAULT,
		.ulcon		= SMDKV210_ULCON_DEFAULT,
		.ufcon		= SMDKV210_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= SMDKV210_UCON_DEFAULT,
		.ulcon		= SMDKV210_ULCON_DEFAULT,
		.ufcon		= SMDKV210_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= SMDKV210_UCON_DEFAULT,
		.ulcon		= SMDKV210_ULCON_DEFAULT,
		.ufcon		= SMDKV210_UFCON_DEFAULT,
	},
};

static struct s3c_ide_platdata smdkv210_ide_pdata __initdata = {
	.setup_gpio	= s5pv210_ide_setup_gpio,
};

static uint32_t smdkv210_keymap[] __initdata = {
	/* KEY(row, col, keycode) */
	KEY(0, 3, KEY_1), KEY(0, 4, KEY_2), KEY(0, 5, KEY_3),
	KEY(0, 6, KEY_4), KEY(0, 7, KEY_5),
	KEY(1, 3, KEY_A), KEY(1, 4, KEY_B), KEY(1, 5, KEY_C),
	KEY(1, 6, KEY_D), KEY(1, 7, KEY_E)
};

static struct matrix_keymap_data smdkv210_keymap_data __initdata = {
	.keymap		= smdkv210_keymap,
	.keymap_size	= ARRAY_SIZE(smdkv210_keymap),
};

static struct samsung_keypad_platdata smdkv210_keypad_data __initdata = {
	.keymap_data	= &smdkv210_keymap_data,
	.rows		= 8,
	.cols		= 8,
};

#ifdef CONFIG_DM9000
static struct resource smdkv210_dm9000_resources[] = {
	[0] = DEFINE_RES_MEM(S5PV210_PA_SROM_BANK5, 1),
	[1] = DEFINE_RES_MEM(S5PV210_PA_SROM_BANK5 + 2, 1),
	[2] = DEFINE_RES_NAMED(IRQ_EINT(9), 1, NULL, IORESOURCE_IRQ \
				| IORESOURCE_IRQ_HIGHLEVEL),
};


static struct dm9000_plat_data smdkv210_dm9000_platdata = {
	.flags		= DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM,
	.dev_addr	= { 0x00, 0x09, 0xc0, 0xff, 0xec, 0x48 },
};

static struct platform_device smdkv210_dm9000 = {
	.name		= "dm9000",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(smdkv210_dm9000_resources),
	.resource	= smdkv210_dm9000_resources,
	.dev		= {
		.platform_data	= &smdkv210_dm9000_platdata,
	},
};
#endif

#ifdef CONFIG_SMSC911X
static struct resource smdkv210_smsc911x_resources[] = {
	[0] = {
		.start = S5PV210_PA_SMSC9220,
		.end   = S5PV210_PA_SMSC9220 + 0xFC,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_EINT(9),
		.end   = IRQ_EINT(9),
		.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL | IRQF_TRIGGER_LOW,
	},
};

static struct smsc911x_platform_config smdkv210_smsc911x_config = {
        .phy_interface  = PHY_INTERFACE_MODE_MII,
        .irq_polarity   = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
        .irq_type       = SMSC911X_IRQ_TYPE_PUSH_PULL,
        .flags          = SMSC911X_USE_16BIT ,
/* you can edit the mac address below instead of the random one   */
/*      .mac            = { 0x00, 0x09, 0xc0, 0xff, 0xec, 0x48 }, */
        };

static struct platform_device s5pv210_device_smsc911x = {
	.name           = "smsc911x",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(smdkv210_smsc911x_resources),
	.resource       = smdkv210_smsc911x_resources,
	.dev            = {
		.platform_data  =  &smdkv210_smsc911x_config,
	},
};
#endif

static struct s5p_media_device s5pv210_media_devs[] = {
        [0] = {
                .id = S5P_MDEV_MFC,
                .name = "mfc",
                .bank = 0,
                .memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_MFC0,
                .paddr = 0,
        },
        [1] = {
                .id = S5P_MDEV_MFC,
                .name = "mfc",
                .bank = 1,
                .memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_MFC1,
                .paddr = 0,
        },
        [2] = {
                .id = S5P_MDEV_FIMC0,
                .name = "fimc0",
                .bank = 1,
                .memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMC0,
                .paddr = 0,
        },
        [3] = {
                .id = S5P_MDEV_FIMC1,
                .name = "fimc1",
                .bank = 1,
                .memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMC1,
                .paddr = 0,
        },
        [4] = {
                .id = S5P_MDEV_FIMC2,
                .name = "fimc2",
                .bank = 1,
                .memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMC2,
                .paddr = 0,
        },
        [5] = {
                .id = S5P_MDEV_JPEG,
                .name = "jpeg",
                .bank = 0,
                .memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_JPEG,
                .paddr = 0,
        },
        [6] = {
                .id = S5P_MDEV_FIMD,
                .name = "fimd",
                .bank = 1,
                .memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMD,
                .paddr = 0,
        },
        [7] = {
                .id = S5P_MDEV_TEXSTREAM,
                                .name = "texstream",
                                .bank = 1,
                                .memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_TEXSTREAM,
                                .paddr = 0,
         },
         [8] = {
                                .id = S5P_MDEV_PMEM_GPU1,
                                .name = "pmem_gpu1",
                                .bank = 0, /* OneDRAM */
                                .memsize = S5PV210_ANDROID_PMEM_MEMSIZE_PMEM_GPU1,
                                .paddr = 0,
         },
         [9] = {
                                .id = S5P_MDEV_G2D,
                                .name = "g2d",
                                .bank = 0,
                                .memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_G2D,
                                .paddr = 0,
         },
};

static void smdkv210_lte480wv_set_power(struct plat_lcd_data *pd,
					unsigned int power)
{

	printk("smdkv210_lte480wv_set_power was called ..............\n");
	if (power) {
#if !defined(CONFIG_BACKLIGHT_PWM)
		gpio_request_one(S5PV210_GPD0(3), GPIOF_OUT_INIT_HIGH, "GPD0");
		gpio_free(S5PV210_GPD0(3));
#endif

		/* fire nRESET on power up */
		gpio_request_one(S5PV210_GPH0(6), GPIOF_OUT_INIT_HIGH, "GPH0");

		gpio_set_value(S5PV210_GPH0(6), 0);
		mdelay(10);

		gpio_set_value(S5PV210_GPH0(6), 1);
		mdelay(10);

		gpio_free(S5PV210_GPH0(6));
	} else {
#if !defined(CONFIG_BACKLIGHT_PWM)
		gpio_request_one(S5PV210_GPD0(3), GPIOF_OUT_INIT_LOW, "GPD0");
		gpio_free(S5PV210_GPD0(3));
#endif
	}
}

static struct plat_lcd_data smdkv210_lcd_lte480wv_data = {
	.set_power	= smdkv210_lte480wv_set_power,
};

static struct platform_device smdkv210_lcd_lte480wv = {
	.name			= "platform-lcd",
	.dev.parent		= &s3c_device_fb.dev,
	.dev.platform_data	= &smdkv210_lcd_lte480wv_data,
};
#if 0
static struct s3c_fb_pd_win smdkv210_fb_win0 = {
	.max_bpp	= 32,
	.default_bpp	= 24,
	.xres		= 800,
	.yres		= 480,
};

static struct fb_videomode smdkv210_lcd_timing = {
	.left_margin	= 13,
	.right_margin	= 8,
	.upper_margin	= 7,
	.lower_margin	= 5,
	.hsync_len	= 3,
	.vsync_len	= 1,
	.xres		= 800,
	.yres		= 480,
};

static struct s3c_fb_platdata smdkv210_lcd0_pdata __initdata = {
	.win[0]		= &smdkv210_fb_win0,
	.vtiming	= &smdkv210_lcd_timing,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
	.setup_gpio	= s5pv210_fb_gpio_setup_24bpp,
};
#endif

static struct s3cfb_lcd lte480wv = {
        .width = S5PV210_LCD_WIDTH,
        .height = S5PV210_LCD_HEIGHT,
        .bpp = 32,
        .freq = 60,

        .timing = {
                .h_fp = 120,
                .h_bp = 13,
                .h_sw = 3,
                .v_fp = 5,
                .v_fpe = 1,
                .v_bp = 7,
                .v_bpe = 1,
                .v_sw = 1,
        },
        .polarity = {
                .rise_vclk = 0,
                .inv_hsync = 1,
                .inv_vsync = 1,
                .inv_vden = 0,
        },
};

static void lte480wv_cfg_gpio(struct platform_device *pdev)
{
        int i;
	printk("lte480wv_cfg_gpio was called ..............\n");

        for (i = 0; i < 8; i++) {
                s3c_gpio_cfgpin(S5PV210_GPF0(i), S3C_GPIO_SFN(2));
                s3c_gpio_setpull(S5PV210_GPF0(i), S3C_GPIO_PULL_NONE);
        }

        for (i = 0; i < 8; i++) {
                s3c_gpio_cfgpin(S5PV210_GPF1(i), S3C_GPIO_SFN(2));
                s3c_gpio_setpull(S5PV210_GPF1(i), S3C_GPIO_PULL_NONE);
        }

        for (i = 0; i < 8; i++) {
                s3c_gpio_cfgpin(S5PV210_GPF2(i), S3C_GPIO_SFN(2));
                s3c_gpio_setpull(S5PV210_GPF2(i), S3C_GPIO_PULL_NONE);
        }
        for (i = 0; i < 6; i++) {
                s3c_gpio_cfgpin(S5PV210_GPF3(i), S3C_GPIO_SFN(2));
                s3c_gpio_setpull(S5PV210_GPF3(i), S3C_GPIO_PULL_NONE);
        }

        /* mDNIe SEL: why we shall write 0x2 ? */
        writel(0x2, S5P_MDNIE_SEL);

#ifndef CONFIG_FB_S3C_TL2796
        /* drive strength to max */
        writel(0xffffffff, S5PV210_GPF0_BASE + 0xc);
        writel(0xffffffff, S5PV210_GPF1_BASE + 0xc);
        writel(0xffffffff, S5PV210_GPF2_BASE + 0xc);
        writel(0x000000ff, S5PV210_GPF3_BASE + 0xc);
#else
        writel(0xC0, S5PV210_GPF0_BASE + 0xc);
#endif
}



#define S5PV210_GPD_0_0_TOUT_0  (0x2)
#define S5PV210_GPD_0_1_TOUT_1  (0x2 << 4)
#define S5PV210_GPD_0_2_TOUT_2  (0x2 << 8)
#define S5PV210_GPD_0_3_TOUT_3  (0x2 << 12)
static int lte480wv_backlight_on(struct platform_device *pdev)
{
        int err;
	printk("lte480wv_backlight_on  was called ..............\n");
#if defined (CONFIG_FB_S3C_TL2796)
        err = gpio_request(S5PV210_GPB(4), "GPB");
        if (err) {
                printk(KERN_ERR "failed to request GPB(4) for "
                "LVDS PWDN pin\n");
                return err;
        }
        gpio_direction_output(S5PV210_GPB(4), 1);
        gpio_set_value(S5PV210_GPB(4), 1);
        gpio_free(S5PV210_GPB(4));
        mdelay(100);
#endif
        err = gpio_request(S5PV210_GPD0(3), "GPD0");

        if (err) {
                printk(KERN_ERR "failed to request GPD0 for "
                        "lcd backlight control\n");
                return err;
        }

        gpio_direction_output(S5PV210_GPD0(3), 1);

        s3c_gpio_cfgpin(S5PV210_GPD0(3), S5PV210_GPD_0_3_TOUT_3);

        gpio_free(S5PV210_GPD0(3));

#if defined (CONFIG_FB_S3C_TL2796)
        err = gpio_request(S5PV210_GPB(5), "GPB");
        if (err) {
                printk(KERN_ERR "failed to request GPB(5) for "
                "LED_EN pin\n");
                return err;
        }
        gpio_direction_output(S5PV210_GPB(5), 1);
        gpio_set_value(S5PV210_GPB(5), 1);
        gpio_free(S5PV210_GPB(5));
#endif
        return 0;
}


static int lte480wv_backlight_off(struct platform_device *pdev, int onoff)
{
        int err;

#if defined (CONFIG_FB_S3C_TL2796)
        err = gpio_request(S5PV210_GPB(5), "GPB");
        if (err) {
                printk(KERN_ERR "failed to request GPB(5) for "
                "LED_EN pin\n");
                return err;
        }
        gpio_direction_output(S5PV210_GPB(5), 0);
        gpio_set_value(S5PV210_GPB(5), 0);
        gpio_free(S5PV210_GPB(5));
#endif

        err = gpio_request(S5PV210_GPD0(3), "GPD0");

        if (err) {
                printk(KERN_ERR "failed to request GPD0 for "
                                "lcd backlight control\n");
                return err;
        }

        gpio_direction_output(S5PV210_GPD0(3), 0);
        gpio_free(S5PV210_GPD0(3));

#if defined (CONFIG_FB_S3C_TL2796)
        err = gpio_request(S5PV210_GPB(4), "GPB");
        if (err) {
                printk(KERN_ERR "failed to request GPB(4) for "
                "LVDS PWDN pin\n");
                return err;
        }
        gpio_direction_output(S5PV210_GPB(4), 0);
        gpio_set_value(S5PV210_GPB(4), 0);
        gpio_free(S5PV210_GPB(4));
#endif
        return 0;
}


static int lte480wv_reset_lcd(struct platform_device *pdev)
{

	printk("lte480wv_reset_lcd was called.................\n");
#ifndef CONFIG_FB_S3C_TL2796
        int err;

        err = gpio_request(S5PV210_GPH0(6), "GPH0");
        if (err) {
                printk(KERN_ERR "failed to request GPH0 for "
                                "lcd reset control\n");
                return err;
        }

        gpio_direction_output(S5PV210_GPH0(6), 1);
        mdelay(100);

        gpio_set_value(S5PV210_GPH0(6), 0);
        mdelay(10);

        gpio_set_value(S5PV210_GPH0(6), 1);
        mdelay(10);

        gpio_free(S5PV210_GPH0(6));
#endif
        return 0;
}


static struct s3c_platform_fb lte480wv_fb_data __initdata = {
        .hw_ver = 0x62,
        .clk_name       = "sclk_fimd",
        .nr_wins = 5,
        .default_win = CONFIG_FB_S3C_DEFAULT_WINDOW,
        .swap = FB_SWAP_WORD | FB_SWAP_HWORD,

        .lcd = &lte480wv,
        .cfg_gpio       = lte480wv_cfg_gpio,
        .backlight_on   = lte480wv_backlight_on,
        .backlight_onoff    = lte480wv_backlight_off,
        .reset_lcd      = lte480wv_reset_lcd,
};

/* USB OTG */
static struct s3c_hsotg_plat smdkv210_hsotg_pdata;

static struct platform_device *smdkv210_devices[] __initdata = {
	&s3c_device_adc,
	&s3c_device_cfcon,
	&s3c_device_fb,
	&s3c_device_hsmmc0,
	&s3c_device_hsmmc1,
	&s3c_device_hsmmc2,
	&s3c_device_hsmmc3,
	&s3c_device_i2c0,
	&s3c_device_i2c1,
	&s3c_device_i2c2,
	&s3c_device_rtc,
	&s3c_device_ts,
	&s3c_device_usb_hsotg,
	&s3c_device_wdt,
	&s5p_device_fimc0,
	&s5p_device_fimc1,
	&s5p_device_fimc2,
	&s5p_device_fimc_md,
	&s5p_device_jpeg,
	&s5p_device_mfc,
	&s5p_device_mfc_l,
	&s5p_device_mfc_r,
	&s5pv210_device_ac97,
	&s5pv210_device_iis0,
	&s5pv210_device_spdif,
	&samsung_asoc_idma,
	&samsung_device_keypad,
#ifdef CONFIG_DM9000
	&smdkv210_dm9000,
#endif
#ifdef CONFIG_SMSC911X
	&s5pv210_device_smsc911x,
#endif
//	&smdkv210_lcd_lte480wv,
#ifdef CONFIG_VIDEO_FIMC
        &s3c_device_fimc0,
        &s3c_device_fimc1,
        &s3c_device_fimc2,
#endif
#ifdef CONFIG_S5PV210_POWER_DOMAIN
        &s5pv210_pd_audio,
        //&s5pv210_pd_cam,
        &s5pv210_pd_tv,
        &s5pv210_pd_lcd,
        &s5pv210_pd_g3d,
        &s5pv210_pd_mfc,
#endif
};

static void __init smdkv210_dm9000_init(void)
{
	unsigned int tmp;

	gpio_request(S5PV210_MP01(5), "nCS5");
	s3c_gpio_cfgpin(S5PV210_MP01(5), S3C_GPIO_SFN(2));
	gpio_free(S5PV210_MP01(5));

	tmp = (5 << S5P_SROM_BCX__TACC__SHIFT);
	__raw_writel(tmp, S5P_SROM_BC5);

	tmp = __raw_readl(S5P_SROM_BW);
	tmp &= (S5P_SROM_BW__CS_MASK << S5P_SROM_BW__NCS5__SHIFT);
	tmp |= (1 << S5P_SROM_BW__NCS5__SHIFT);
	__raw_writel(tmp, S5P_SROM_BW);
}

static struct i2c_board_info smdkv210_i2c_devs0[] __initdata = {
	{ I2C_BOARD_INFO("24c08", 0x50), },     /* Samsung S524AD0XD1 */
	{ I2C_BOARD_INFO("wm8580", 0x1b), },
};

static struct i2c_board_info smdkv210_i2c_devs1[] __initdata = {
	/* To Be Updated */
};

static struct i2c_board_info smdkv210_i2c_devs2[] __initdata = {
	/* To Be Updated */
};

/* LCD Backlight data */
static struct samsung_bl_gpio_info smdkv210_bl_gpio_info = {
	.no = S5PV210_GPD0(3),
	.func = S3C_GPIO_SFN(2),
};

static struct platform_pwm_backlight_data smdkv210_bl_data = {
	.pwm_id = 3,
	.pwm_period_ns = 1000,
};

static void __init smdkv210_map_io(void)
{
	s5pv210_init_io(NULL, 0);
	s3c24xx_init_clocks(clk_xusbxti.rate);
	s3c24xx_init_uarts(smdkv210_uartcfgs, ARRAY_SIZE(smdkv210_uartcfgs));
	samsung_set_timer_source(SAMSUNG_PWM2, SAMSUNG_PWM4);
        /* hcj: change from S5P_RANGE_MFC to 0 */
        s5p_reserve_bootmem(s5pv210_media_devs,
                        ARRAY_SIZE(s5pv210_media_devs), 0);
}

static void __init smdkv210_reserve(void)
{
	//s5p_mfc_reserve_mem(0x43000000, 8 << 20, 0x51000000, 8 << 20);
}

static void __init smdkv210_machine_init(void)
{
	s3c_pm_init();

#ifdef CONFIG_DM9000
	smdkv210_dm9000_init();
#endif

	samsung_keypad_set_platdata(&smdkv210_keypad_data);
	s3c24xx_ts_set_platdata(NULL);

	s3c_i2c0_set_platdata(NULL);
	s3c_i2c1_set_platdata(NULL);
	s3c_i2c2_set_platdata(NULL);
	i2c_register_board_info(0, smdkv210_i2c_devs0,
			ARRAY_SIZE(smdkv210_i2c_devs0));
	i2c_register_board_info(1, smdkv210_i2c_devs1,
			ARRAY_SIZE(smdkv210_i2c_devs1));
	i2c_register_board_info(2, smdkv210_i2c_devs2,
			ARRAY_SIZE(smdkv210_i2c_devs2));

	s3c_ide_set_platdata(&smdkv210_ide_pdata);

//	s3c_fb_set_platdata(&smdkv210_lcd0_pdata);
	s3c_fb_set_platdata(&lte480wv_fb_data);

	samsung_bl_set(&smdkv210_bl_gpio_info, &smdkv210_bl_data);

	s3c_hsotg_set_platdata(&smdkv210_hsotg_pdata);

	platform_add_devices(smdkv210_devices, ARRAY_SIZE(smdkv210_devices));
}

MACHINE_START(SMDKV210, "SMDKV210")
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	.atag_offset	= 0x100,
	.init_irq	= s5pv210_init_irq,
	.map_io		= smdkv210_map_io,
	.init_machine	= smdkv210_machine_init,
	.init_time	= samsung_timer_init,
	.restart	= s5pv210_restart,
	.reserve	= &smdkv210_reserve,
MACHINE_END
