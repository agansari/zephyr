/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System/hardware module for nxp_lpc55s69 platform
 *
 * This module provides routines to initialize and support board-level
 * hardware for the nxp_lpc55s69 platform.
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <soc.h>
#include <drivers/uart.h>
#include <linker/sections.h>
#include <arch/cpu.h>
#include <aarch32/cortex_m/exc.h>
#include <fsl_power.h>
#include <fsl_clock.h>
#include <fsl_common.h>
#include <fsl_device_registers.h>
#include <fsl_pint.h>
// #include <cortex_m/tz.h>

/**
 *
 * @brief Initialize the system clock
 *
 * @return N/A
 *
 */

static ALWAYS_INLINE void clock_init(void)
{
#if defined(CONFIG_SOC_LPC55S16) || defined(CONFIG_SOC_LPC55S69_CPU0)
    /*!< Set up the clock sources */
    /*!< Configure FRO192M */
	/*!< Ensure FRO is on  */
	POWER_DisablePD(kPDRUNCFG_PD_FRO192M);
	/*!< Set up FRO to the 12 MHz, just for sure */
	CLOCK_SetupFROClocking(12000000U);
	/*!< Switch to FRO 12MHz first to ensure we can change the clock */
	CLOCK_AttachClk(kFRO12M_to_MAIN_CLK);

	/* Enable FRO HF(96MHz) output */
	CLOCK_SetupFROClocking(96000000U);

	/*!< Set FLASH wait states for core */
	CLOCK_SetFLASHAccessCyclesForFreq(96000000U);

    /*!< Set up dividers */
	CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U, false);

    /*!< Set up clock selectors - Attach clocks to the peripheries */
	CLOCK_AttachClk(kFRO_HF_to_MAIN_CLK);

	/* Enables the clock for the I/O controller.: Enable Clock. */
    CLOCK_EnableClock(kCLOCK_Iocon);

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(flexcomm4), nxp_lpc_i2c, okay)
	/* attach 12 MHz clock to FLEXCOMM4 */
	CLOCK_AttachClk(kFRO12M_to_FLEXCOMM4);

	/* reset FLEXCOMM for I2C */
	RESET_PeripheralReset(kFC4_RST_SHIFT_RSTn);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(hs_lspi), okay)
	/* Attach 12 MHz clock to HSLSPI */
	CLOCK_AttachClk(kFRO_HF_DIV_to_HSLSPI);

	/* reset HSLSPI for SPI */
	RESET_PeripheralReset(kHSLSPI_RST_SHIFT_RSTn);
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(wwdt0), nxp_lpc_wwdt, okay)
	/* Enable 1 MHz FRO clock for WWDT */
	SYSCON->CLOCK_CTRL |= SYSCON_CLOCK_CTRL_FRO1MHZ_CLK_ENA_MASK;
#endif

    CLOCK_EnableClock(kCLOCK_Mailbox);
    /* Reset the MAILBOX module */
    RESET_PeripheralReset(kMAILBOX_RST_SHIFT_RSTn);

    CLOCK_EnableClock(kCLOCK_Sram1);
    CLOCK_EnableClock(kCLOCK_Sram2);
    CLOCK_EnableClock(kCLOCK_Sram3);
    CLOCK_EnableClock(kCLOCK_Sram4);

	RESET_PeripheralReset(kSRAM1_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kSRAM2_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kSRAM3_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kSRAM4_RST_SHIFT_RSTn);

	SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL0_SRAM_CTRL1_MASK | SYSCON_AHBCLKCTRL0_SRAM_CTRL2_MASK |
							SYSCON_AHBCLKCTRL0_SRAM_CTRL3_MASK | SYSCON_AHBCLKCTRL0_SRAM_CTRL4_MASK;

	printk("early>>>\n CPSTAT = 0x%x\n CPUCTRL = 0x%x\n",SYSCON->CPSTAT,SYSCON->CPUCTRL);

#endif /* CONFIG_SOC_LPC55S69_CPU0 */
}

void z_platform_init(void)
{
// #if ((__FPU_PRESENT == 1) && (__FPU_USED == 1))
//     SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10, CP11 Full Access in Secure mode */
// #if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
//     SCB_NS->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10, CP11 Full Access in Normal mode */
// #endif                                                    /* (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U) */
// #endif                                                    /* ((__FPU_PRESENT == 1) && (__FPU_USED == 1)) */

//     SCB->CPACR |= ((3UL << 0 * 2) | (3UL << 1 * 2)); /* set CP0, CP1 Full Access in Secure mode (enable PowerQuad) */
// #if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
//     SCB_NS->CPACR |= ((3UL << 0 * 2) | (3UL << 1 * 2)); /* set CP0, CP1 Full Access in Normal mode (enable PowerQuad) */
// #endif                                                  /* (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U) */

    // SCB->NSACR |= ((3UL << 0) | (3UL << 10)); /* enable CP0, CP1, CP10, CP11 Non-secure Access */

// #if defined(__MCUXPRESSO)
//     extern void (*const g_pfnVectors[])(void);
//     SCB->VTOR = (uint32_t)&g_pfnVectors;
// #else
//     extern void *__Vectors;
//     SCB->VTOR = (uint32_t)&__Vectors;
// #endif
    // SYSCON->TRACECLKDIV = 0;
/* Optionally enable RAM banks that may be off by default at reset */
// #if !defined(DONT_ENABLE_DISABLED_RAMBANKS)
//     SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL0_SRAM_CTRL1_MASK | SYSCON_AHBCLKCTRL0_SRAM_CTRL2_MASK |
//                                SYSCON_AHBCLKCTRL0_SRAM_CTRL3_MASK | SYSCON_AHBCLKCTRL0_SRAM_CTRL4_MASK;
// #endif
}

/**
 *
 * @brief Perform basic hardware initialization
 *
 * Initialize the interrupt controller device drivers.
 * Also initialize the timer device driver, if required.
 *
 * @return 0
 */

static int nxp_lpc55xxx_init(const struct device *arg)
{
	ARG_UNUSED(arg);

	/* old interrupt lock level */
	unsigned int oldLevel;

	/* disable interrupts */
	oldLevel = irq_lock();

	z_arm_clear_faults();

	/* Initialize FRO/system clock to 96 MHz */
	clock_init();

#ifdef CONFIG_GPIO_MCUX_LPC
	/* Turn on PINT device*/
	PINT_Init(PINT);
#endif

#ifdef CONFIG_CPU_HAS_ARM_SAU
	/* Disable SAU device until security is properly configuration */
	// TZ_SAU_Disable();
	// SAU->CTRL &= ~(1);
	// tz_sau_configure(0,1);
#endif

// #if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
// 	SAU->CTRL |= (1 << SAU_CTRL_ALLNS_Pos);
// #endif

// #if ((__FPU_PRESENT == 1) && (__FPU_USED == 1))
//     SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10, CP11 Full Access in Secure mode */
// #if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
//     SCB_NS->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10, CP11 Full Access in Normal mode */
// #endif                                                    /* (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U) */
// #endif                                                    /* ((__FPU_PRESENT == 1) && (__FPU_USED == 1)) */

//     SCB->CPACR |= ((3UL << 0 * 2) | (3UL << 1 * 2)); /* set CP0, CP1 Full Access in Secure mode (enable PowerQuad) */
// #if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
//     SCB_NS->CPACR |= ((3UL << 0 * 2) | (3UL << 1 * 2)); /* set CP0, CP1 Full Access in Normal mode (enable PowerQuad) */
// #endif                                                  /* (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U) */

//     SCB->NSACR |= ((3UL << 0) | (3UL << 10)); /* enable CP0, CP1, CP10, CP11 Non-secure Access */

// 	SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL0_SRAM_CTRL1_MASK | SYSCON_AHBCLKCTRL0_SRAM_CTRL2_MASK |
//                                SYSCON_AHBCLKCTRL0_SRAM_CTRL3_MASK | SYSCON_AHBCLKCTRL0_SRAM_CTRL4_MASK;

	/*
	 * install default handler that simply resets the CPU if configured in
	 * the kernel, NOP otherwise
	 */
	NMI_INIT();

	/* restore interrupt state */
	irq_unlock(oldLevel);

	return 0;
}

SYS_INIT(nxp_lpc55xxx_init, PRE_KERNEL_1, 0);

#ifdef CONFIG_SLAVE_CORE_MCUX

#define SLAVE_CORE_BOOT_ADDRESS (void *)CONFIG_SLAVE_BOOT_ADDRESS_MCUX

static const char slave_core[] = {
#include "slave-core.inc"
};

/**
 *
 * @brief Slave Init
 *
 * This routine boots the secondary core
 * @return N/A
 */
/* This function is also called at deep sleep resume. */
int _slave_init(struct device *arg)
{
	ARG_UNUSED(arg);

	/* Setup the reset handler pointer (PC) and stack pointer value.
	 * This is used once the second core runs its startup code.
	 * The second core first boots from flash (address 0x00000000)
	 * and then detects its identity (Core no. 1, slave) and checks
	 * registers CPBOOT and use them to continue the boot process.
	 * Make sure the startup code for master core is
	 * appropriate and shareable with the slave core!
	 */
    SYSCON->CPUCFG |= SYSCON_CPUCFG_CPU1ENABLE_MASK;

    /* Boot source for Core 1 from RAM */
    SYSCON->CPBOOT = SYSCON_CPBOOT_CPBOOT(*(uint32_t *)DT_REG_ADDR(DT_CHOSEN(zephyr_code_cpu1_partition)));

	// printk("\n\n*(uint32_t *)DT_REG_ADDR(DT_CHOSEN(zephyr_code_cpu1_partition)) = 0x%x\n\n",*(uint32_t *)DT_REG_ADDR(DT_CHOSEN(zephyr_code_cpu1_partition)));

    int32_t temp = SYSCON->CPUCTRL;
    temp |= 0xc0c40000;
    SYSCON->CPUCTRL = temp | SYSCON_CPUCTRL_CPU1RSTEN_MASK | SYSCON_CPUCTRL_CPU1CLKEN_MASK;
    SYSCON->CPUCTRL = (temp | SYSCON_CPUCTRL_CPU1CLKEN_MASK) & (~SYSCON_CPUCTRL_CPU1RSTEN_MASK);

	printk("after>>>\n CPSTAT = 0x%x\n CPUCTRL = 0x%x\n",SYSCON->CPSTAT,SYSCON->CPUCTRL);

	// configure_nonsecure_msp()

	return 0;
}

SYS_INIT(_slave_init, PRE_KERNEL_2, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

#endif /*CONFIG_SLAVE_CORE_MCUX*/

#if defined(CONFIG_SOC_LPC55S69_CPU1)

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led2)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#if DT_PHA_HAS_CELL(LED0_NODE, gpios, flags)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#endif
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#endif

#ifndef FLAGS
#define FLAGS	0
#endif

int _slave_blink(struct device *arg)
{
	struct device *dev;
	bool led_is_on = false;
	int ret;

	dev = device_get_binding(LED0);
	if (dev == NULL) {
		return 0;
	}

	ret = gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return 0;
	}

	gpio_pin_set(dev, PIN, (int)led_is_on);
	
	// while (1) {
	// 	gpio_pin_set(dev, PIN, (int)led_is_on);
	// 	led_is_on = !led_is_on;
	// 	k_msleep(SLEEP_TIME_MS);
	// }

	return 0;
}

SYS_INIT(_slave_blink, PRE_KERNEL_2, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

#endif
