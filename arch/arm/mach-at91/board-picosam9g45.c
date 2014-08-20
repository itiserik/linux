/*
 *  Board-specific setup code for the picoSAM9G45 board
 *
 *  http://www.mini-box.com/pico-SAM9G45-X
 *
 *  Copyright (C) 2011 Nicu Pavel <npavel@mini-box.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/fb.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/atmel-mci.h>
#include <linux/delay.h>
#include <linux/pwm.h>
#include <linux/leds_pwm.h>

#include <linux/platform_data/at91_adc.h>

#include <mach/hardware.h>
#include <video/atmel_lcdc.h>
#include <media/soc_camera.h>
#include <media/atmel-isi.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/at91sam9_smc.h>
#include <mach/system_rev.h>

#include "at91_aic.h"
#include "at91_shdwc.h"
#include "board.h"
#include "sam9_smc.h"
#include "generic.h"
#include "gpio.h"

static void __init picosam9g45_init_early(void)
{
	/* Initialize processor: 12.000 MHz crystal */
	at91_initialize(12000000);


}

/*
 * USB HS Host port (common to OHCI & EHCI)
 */
static struct at91_usbh_data __initdata picosam9g45_usbh_hs_data = {
        .ports          = 2,
        .vbus_pin       = {AT91_PIN_PD1, AT91_PIN_PD3},
        .vbus_pin_active_low = {1, 1},
        .overcurrent_pin= {-EINVAL, -EINVAL},
};


/*
 * USB HS Device port
 */
static struct usba_platform_data __initdata picosam9g45_usba_udc_data = {
	.vbus_pin	= AT91_PIN_PB19,
};


/*
 * SPI devices.
 */
static struct spi_board_info picosam9g45_spi_devices[] = {
	[0] = {/* SPI0 CS0 on right side connector J7*/
		.modalias= "spidev",
		.max_speed_hz= 15 * 1000 * 1000,
		.bus_num= 0,
		.chip_select= 0,
	},
	[1] = {/* SPI1 CS0 on left side connector J9*/
		.modalias= "spidev",
		.max_speed_hz= 15 * 1000 * 1000,
		.bus_num= 1,
		.chip_select= 0,
	},
};


/*
 * MCI (SD/MMC)
 */
static struct mci_platform_data __initdata mci0_data = {
	.slot[0] = {
		.bus_width	= 4,
		.detect_pin	= AT91_PIN_PD10,
                .wp_pin         = -EINVAL,
	},
};

static struct mci_platform_data __initdata mci1_data = {
	.slot[0] = {
		.bus_width	= 4,
		.detect_pin	= AT91_PIN_PD11,
		.wp_pin		= AT91_PIN_PD29,
	},
};


/*
 * MACB Ethernet device
 */
static struct macb_platform_data __initdata picosam9g45_macb_data = {
	.phy_irq_pin	= AT91_PIN_PD5,
	.is_rmii	= 1,
};


/*
 * NAND flash
 */
static struct mtd_partition __initdata picosam9g45_nand_partition[] = {
	{
		.name   = "Bootstrap",
		.offset = 0,
		.size   = SZ_4M
	},
	{
		.name= "RootFS",
		.offset= MTDPART_OFS_NXTBLK,
		.size= 60 * SZ_1M,
	},
	{
		.name= "Space",
		.offset= MTDPART_OFS_NXTBLK,
		.size= MTDPART_SIZ_FULL,
	},
};

/* det_pin is not connected */
static struct atmel_nand_data __initdata picosam9g45_nand_data = {
	.ale		= 21,
	.cle		= 22,
	.rdy_pin	= AT91_PIN_PC8,
	.enable_pin	= AT91_PIN_PC14,
	.parts          = picosam9g45_nand_partition,
        .num_parts      = ARRAY_SIZE(picosam9g45_nand_partition),
};

static struct sam9_smc_config __initdata picosam9g45_nand_smc_config = {
	.ncs_read_setup		= 0,
	.nrd_setup		= 2,
	.ncs_write_setup	= 0,
	.nwe_setup		= 2,

	.ncs_read_pulse		= 4,
	.nrd_pulse		= 4,
	.ncs_write_pulse	= 4,
	.nwe_pulse		= 4,

	.read_cycle		= 7,
	.write_cycle		= 7,

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE,
	.tdf_cycles		= 3,
};

static void __init picosam9g45_add_device_nand(void)
{
	picosam9g45_nand_data.bus_width_16 = board_have_nand_16bit();
	/* setup bus-width (8 or 16) */
	if (picosam9g45_nand_data.bus_width_16)
		picosam9g45_nand_smc_config.mode |= AT91_SMC_DBW_16;
	else
		picosam9g45_nand_smc_config.mode |= AT91_SMC_DBW_8;

	/* configure chip-select 3 (NAND) */
	sam9_smc_configure(0, 3, &picosam9g45_nand_smc_config);

	at91_add_device_nand(&picosam9g45_nand_data);
}


/*
 * LCD Controller
 */
#undef PICOSAM9G45_BIG_DISPLAY
#if defined(CONFIG_FB_ATMEL) || defined(CONFIG_FB_ATMEL_MODULE)
#ifdef PICOSAM9G45_BIG_DISPLAY
static struct fb_videomode at91_tft_vga_modes[] = {
	{
		.name           = "Feigeda",
		.refresh	= 50,
		.xres		= 800,		.yres		= 480,
		.pixclock	= KHZ2PICOS(24525),

		.left_margin	= 88,		.right_margin	= 40,
		.upper_margin	= 32,		.lower_margin	= 13,
		.hsync_len	= 1,		.vsync_len	= 3,

		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};

static struct fb_monspecs at91fb_default_monspecs = {
	.manufacturer	= "FGD",
	.monitor        = "FGD700C4001",

	.modedb		= at91_tft_vga_modes,
	.modedb_len	= ARRAY_SIZE(at91_tft_vga_modes),
	.hfmin		= 15000,
	.hfmax		= 17640,
	.vfmin		= 57,
	.vfmax		= 67,
};
#else
static struct fb_videomode at91_tft_vga_modes[] = {
	{
		.name           = "HannStar",
		.refresh	= 60,
		.xres		= 480,		.yres		= 272,
		/*.pixclock	= KHZ2PICOS(8642),*/
		.pixclock	= KHZ2PICOS(9000),

		.left_margin	= 36,		.right_margin	= 4,
		.upper_margin	= 3,		.lower_margin	= 2,
		.hsync_len	= 0,		.vsync_len	= 0,

		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};

static struct fb_monspecs at91fb_default_monspecs = {
	.manufacturer	= "HNS",
	.monitor        = "HSD043I9W1",

	.modedb		= at91_tft_vga_modes,
	.modedb_len	= ARRAY_SIZE(at91_tft_vga_modes),
	.hfmin		= 15000,
	.hfmax		= 17640,
	.vfmin		= 50,
	.vfmax		= 70,
};
#endif

#define AT91SAM9G45_DEFAULT_LCDCON2 	(ATMEL_LCDC_MEMOR_LITTLE \
					| ATMEL_LCDC_DISTYPE_TFT \
					| ATMEL_LCDC_CLKMOD_ALWAYSACTIVE)

/* Driver datas */
static struct atmel_lcdfb_pdata __initdata picosam9g45_lcdc_data = {
	.lcdcon_is_backlight		= true,
	.default_bpp			= 16,
	.default_dmacon			= ATMEL_LCDC_DMAEN,
	.default_lcdcon2		= AT91SAM9G45_DEFAULT_LCDCON2,
	.default_monspecs		= &at91fb_default_monspecs,
	.guard_time			= 9,
	.lcd_wiring_mode		= ATMEL_LCDC_WIRING_RGB,
};

#else
static struct atmel_lcdfb_info __initdata picosam9g45_lcdc_data;
#endif

/*
 * ADCs and touchscreen
 */
static struct at91_adc_data picosam9g45_adc_data = {
        .channels_used = BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6) | BIT(7),
        .use_external_triggers = true,
        .vref = 3300,
        .touchscreen_type = ATMEL_ADC_TOUCHSCREEN_4WIRE,
};

/*
 * GPIO Buttons
 */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
static struct gpio_keys_button picosam9g45_buttons[] = {
#ifndef CONFIG_PICOSAM9G45_HAS_UART2
	{	/* J9 pin 5 gnd + pin 11 */
		.code		= KEY_BACK,
		.gpio		= AT91_PIN_PB6,
		.active_low	= 1,
		.desc		= "Back",
		.wakeup		= 1,
	},
	{	/* J9 pin 5 gnd + pin 13*/
		.code		= KEY_MENU,
		.gpio		= AT91_PIN_PB7,
		.active_low	= 1,
		.desc		= "Menu",
		.wakeup		= 1,
	},
#endif
	{	/* J9 pin 5 gnd + pin 12 */
		.code		= KEY_HOME,
		.gpio		= AT91_PIN_PB16,
		.active_low	= 1,
		.desc		= "Home",
	},

};

static struct gpio_keys_platform_data picosam9g45_button_data = {
	.buttons	= picosam9g45_buttons,
	.nbuttons	= ARRAY_SIZE(picosam9g45_buttons),
};

static struct platform_device picosam9g45_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &picosam9g45_button_data,
	}
};

static void __init picosam9g45_add_device_buttons(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(picosam9g45_buttons); i++) {
		at91_set_GPIO_periph(picosam9g45_buttons[i].gpio, 1);
		at91_set_deglitch(picosam9g45_buttons[i].gpio, 1);
	}

	platform_device_register(&picosam9g45_button_device);
}
#else
static void __init picosam9g45_add_device_buttons(void) {}
#endif


/*
 * LEDs ... these could all be PWM-driven, for variable brightness
 */
static struct gpio_led picosam9g45_leds[] = {
#ifndef CONFIG_PICOSAM9G45_HAS_UART2
	{	/* "pwr" led */
		.name			= "pwr",
		.gpio			= AT91_PIN_PD30,
		.default_trigger	= "heartbeat",
	},
#endif
	{	/* "u1" led */
		.name			= "u1",
		.gpio			= AT91_PIN_PD0,
		.active_low		= 1,
		.default_trigger	= "mmc0",
	},
#if !(defined(CONFIG_LEDS_ATMEL_PWM) || defined(CONFIG_LEDS_ATMEL_PWM_MODULE))
	{	/* "u2" led */
		.name			= "u2",
		.gpio			= AT91_PIN_PD31,
		.active_low		= 1,
		.default_trigger	= "none",
	},
#endif
};


/*
 * PWM Leds
 */
static struct pwm_lookup pwm_lookup[] = {
        PWM_LOOKUP("at91sam9rl-pwm", 1, "leds_pwm", "d7",
                   5000, PWM_POLARITY_INVERSED),
        PWM_LOOKUP("at91sam9rl-pwm", 2, "leds_pwm", "buzzer",
                   5000, PWM_POLARITY_INVERSED),
};

#if IS_ENABLED(CONFIG_LEDS_PWM)
static struct led_pwm pwm_leds[] = {
        {       /* "right" led, green, userled1, pwm1 */
                .name = "d7",
                .max_brightness = 255,
        },
        {       /* picopc buzzer */
                .name = "buzzer",
                .max_brightness = 255,
        },
};

static struct led_pwm_platform_data pwm_data = {
        .num_leds       = ARRAY_SIZE(pwm_leds),
        .leds           = pwm_leds,
};

static struct platform_device leds_pwm = {
        .name   = "leds_pwm",
        .id     = -1,
        .dev    = {
                .platform_data = &pwm_data,
        },
};
#endif

static struct platform_device *devices[] __initdata = {
#if defined(CONFIG_SOC_CAMERA_OV2640) || \
        defined(CONFIG_SOC_CAMERA_OV2640_MODULE)
        &isi_ov2640,
#endif
#if IS_ENABLED(CONFIG_LEDS_PWM)
        &leds_pwm,
#endif
};


/*
 * PWM buzzer
 */
static void picosam9g45_setup_device_buzzer(void)
{
    at91_set_A_periph(AT91_PIN_PE31, 1);
}


static void __init picosam9g45_board_init(void)
{
	/* Serial */
	/* DGBU on ttyS0. (Rx & Tx only) J7 pins 2,4*/
	at91_register_uart(0, 0, 0);

	/* USART1 on ttyS1. (Rx, Tx, RTS, CTS) J7 pins 11,13,15,17 */
	at91_register_uart(AT91SAM9G45_ID_US1, 1, ATMEL_UART_CTS | ATMEL_UART_RTS);

#ifdef CONFIG_PICOSAM9G45_HAS_UART2
	/* USART2 on ttyS2. (Rx, Tx, RTS, CTS) J9 pins 11,13,15,17*/
	at91_register_uart(AT91SAM9G45_ID_US2, 2, ATMEL_UART_CTS | ATMEL_UART_RTS);
#endif
	at91_add_device_serial();
	/* Buzzer PWM pin */
	picosam9g45_setup_device_buzzer();
	/* USB HS Host */
	at91_add_device_usbh_ohci(&picosam9g45_usbh_hs_data);
	at91_add_device_usbh_ehci(&picosam9g45_usbh_hs_data);
	/* USB HS Device */
	at91_add_device_usba(&picosam9g45_usba_udc_data);
	/* SPI */
	at91_add_device_spi(picosam9g45_spi_devices, ARRAY_SIZE(picosam9g45_spi_devices));
	/* MMC */
	at91_add_device_mci(0, &mci0_data);
	at91_add_device_mci(1, &mci1_data);
	/* Ethernet */
	at91_add_device_eth(&picosam9g45_macb_data);
	/* NAND */
	picosam9g45_add_device_nand();
	/* I2C */
	at91_add_device_i2c(0, NULL, 0);
	at91_add_device_i2c(1, NULL, 0);
	/* LCD Controller */
	at91_add_device_lcdc(&picosam9g45_lcdc_data);
	/* ADC and touchscreen */
        at91_add_device_adc(&picosam9g45_adc_data);
	/* Push Buttons */
	picosam9g45_add_device_buttons();
	/* LEDs */
	at91_gpio_leds(picosam9g45_leds, ARRAY_SIZE(picosam9g45_leds));
	pwm_add_table(pwm_lookup, ARRAY_SIZE(pwm_lookup));
#if IS_ENABLED(CONFIG_LEDS_PWM)
        at91_add_device_pwm(1 << AT91_PWM1 | 1 << AT91_PWM2);
#endif
	/* Other platform devices */
        platform_add_devices(devices, ARRAY_SIZE(devices));
}

MACHINE_START(MINIBOXPICOSAM9G45, "Mini Box picoSAM9 G45 Board")
	/* Maintainer: Nicu Pavel */
	.init_time      = at91sam926x_pit_init,
	.map_io		= at91_map_io,
	.handle_irq     = at91_aic_handle_irq,
	.init_early	= picosam9g45_init_early,
	.init_irq	= at91_init_irq_default,
	.init_machine	= picosam9g45_board_init,
MACHINE_END
