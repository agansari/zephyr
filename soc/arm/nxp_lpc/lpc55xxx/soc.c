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
#ifdef CONFIG_TRUSTED_EXECUTION_SECURE
#include <cortex_m/tz.h>
#endif

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

#endif /* CONFIG_SOC_LPC55S69_CPU0 */
}

// void BOARD_InitTrustZone()
// {
//     //####################################################################
//     //### SAU configuration ##############################################
//     //####################################################################

//     /* Set SAU Control register: Disable SAU and All Secure */
//     SAU->CTRL = 0;

//     /* Set SAU region number */
//     SAU->RNR = 0;
//     /* Region base address */
//     SAU->RBAR = REGION_0_BASE & SAU_RBAR_BADDR_Msk;
//     /* Region end address */
//     SAU->RLAR = ((REGION_0_END & SAU_RLAR_LADDR_Msk) | ((0U << SAU_RLAR_NSC_Pos) & SAU_RLAR_NSC_Msk)) |
//                 ((1U << SAU_RLAR_ENABLE_Pos) & SAU_RLAR_ENABLE_Msk);

//     /* Set SAU region number */
//     SAU->RNR = 0x00000001U;
//     /* Region base address */
//     SAU->RBAR = REGION_1_BASE & SAU_RBAR_BADDR_Msk;
//     /* Region end address */
//     SAU->RLAR = ((REGION_1_END & SAU_RLAR_LADDR_Msk) | ((0U << SAU_RLAR_NSC_Pos) & SAU_RLAR_NSC_Msk)) |
//                 ((1U << SAU_RLAR_ENABLE_Pos) & SAU_RLAR_ENABLE_Msk);

//     /* Set SAU region number */
//     SAU->RNR = 0x00000002U;
//     /* Region base address */
//     SAU->RBAR = REGION_2_BASE & SAU_RBAR_BADDR_Msk;
//     /* Region end address */
//     SAU->RLAR = ((REGION_2_END & SAU_RLAR_LADDR_Msk) | ((1U << SAU_RLAR_NSC_Pos) & SAU_RLAR_NSC_Msk)) |
//                 ((1U << SAU_RLAR_ENABLE_Pos) & SAU_RLAR_ENABLE_Msk);

//     /* Force memory writes before continuing */
//     __DSB();
//     /* Flush and refill pipeline with updated permissions */
//     __ISB();
//     /* Set SAU Control register: Enable SAU and All Secure (applied only if disabled) */
//     SAU->CTRL = 0x00000001U;

//     //####################################################################
//     //### AHB Configurations #############################################
//     //####################################################################

//     //--------------------------------------------------------------------
//     //--- AHB Security Level Configurations ------------------------------
//     //--------------------------------------------------------------------
//     /* Configuration of AHB Secure Controller
//      * Possible values for every memory sector or peripheral rule:
//      *  0    Non-secure, user access allowed.
//      *  1    Non-secure, privileged access allowed.
//      *  2    Secure, user access allowed.
//      *  3    Secure, privileged access allowed. */

//     //--- Security level configuration of memories -----------------------
//     AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_FLASH_MEM_RULE[0] = 0x00000033U;
//     AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_FLASH_MEM_RULE[1] = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_FLASH_MEM_RULE[2] = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_ROM_MEM_RULE[0]   = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_ROM_MEM_RULE[1]   = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_ROM_MEM_RULE[2]   = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_ROM_MEM_RULE[3]   = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_RAMX[0].MEM_RULE[0]                     = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_RAM0[0].MEM_RULE[0]                     = 0x33333333U;
//     AHB_SECURE_CTRL->SEC_CTRL_RAM0[0].MEM_RULE[1]                     = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_RAM1[0].MEM_RULE[0]                     = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_RAM1[0].MEM_RULE[1]                     = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_RAM2[0].MEM_RULE[0]                     = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_RAM2[0].MEM_RULE[1]                     = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_RAM3[0].MEM_RULE[0]                     = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_RAM3[0].MEM_RULE[1]                     = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_RAM4[0].MEM_RULE[0]                     = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_USB_HS[0].MEM_RULE[0]                   = 0;

//     //--- Security level configuration of peripherals --------------------
//     AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE0_MEM_CTRL0 = 0x00000033U;
//     AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE0_MEM_CTRL1 = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE0_MEM_CTRL2 = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE1_MEM_CTRL0 = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE1_MEM_CTRL1 = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE1_MEM_CTRL2 = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE1_MEM_CTRL3 = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_AHB_PORT8_SLAVE0_RULE                        = 0x03000000U;
//     AHB_SECURE_CTRL->SEC_CTRL_AHB_PORT8_SLAVE1_RULE                        = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_AHB_PORT9_SLAVE0_RULE                        = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_AHB_PORT9_SLAVE1_RULE                        = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_AHB_PORT10[0].SLAVE0_RULE                    = 0;
//     AHB_SECURE_CTRL->SEC_CTRL_AHB_PORT10[0].SLAVE1_RULE                    = 0;

//     //--- Security level configuration of masters ------------------------
//     AHB_SECURE_CTRL->MASTER_SEC_LEVEL        = 0;
//     AHB_SECURE_CTRL->MASTER_SEC_ANTI_POL_REG = 0x3FFFFFFFU;

//     //--------------------------------------------------------------------
//     //--- Pins: Reading GPIO state ---------------------------------------
//     //--------------------------------------------------------------------
//     // Possible values for every pin:
//     //  0b0    Deny
//     //  0b1    Allow
//     //--------------------------------------------------------------------
//     AHB_SECURE_CTRL->SEC_GPIO_MASK0 = 0xFFFFFFFFU;
//     AHB_SECURE_CTRL->SEC_GPIO_MASK1 = 0xFFFFFFFFU;

//     //--------------------------------------------------------------------
//     //--- Interrupts: Interrupt handling by Core1 ------------------------
//     //--------------------------------------------------------------------
//     // Possible values for every interrupt:
//     //  0b0    Deny
//     //  0b1    Allow
//     //--------------------------------------------------------------------
//     AHB_SECURE_CTRL->SEC_CPU_INT_MASK0 = 0xFFFFFFFFU;
//     AHB_SECURE_CTRL->SEC_CPU_INT_MASK1 = 0xFFFFFFFFU;

//     //--------------------------------------------------------------------
//     //--- Interrupts: Interrupt security configuration -------------------
//     //--------------------------------------------------------------------
//     // Possible values for every interrupt:
//     //  0b0    Secure
//     //  0b1    Non-secure
//     //--------------------------------------------------------------------
//     NVIC->ITNS[0] = 0;
//     NVIC->ITNS[1] = 0;

//     //--------------------------------------------------------------------
//     //--- Global Options -------------------------------------------------
//     //--------------------------------------------------------------------
//     SCB->AIRCR = (SCB->AIRCR & 0x000009FF7U) | 0x005FA0000U;
//     SCB->SCR &= 0x0FFFFFFF7U;
//     SCB->SHCSR &= 0x0FFF7FFFFU;
//     SCB->NSACR                               = 0x00000C03U;
//     SCnSCB->CPPWR                            = 0;
//     AHB_SECURE_CTRL->SEC_MASK_LOCK           = 0x00000AAAU;
//     AHB_SECURE_CTRL->MASTER_SEC_LEVEL        = (AHB_SECURE_CTRL->MASTER_SEC_LEVEL & 0x03FFFFFFFU) | 0x080000000U;
//     AHB_SECURE_CTRL->MASTER_SEC_ANTI_POL_REG = (AHB_SECURE_CTRL->MASTER_SEC_ANTI_POL_REG & 0x03FFFFFFFU) | 0x080000000U;
//     AHB_SECURE_CTRL->CPU0_LOCK_REG           = 0x800002AAU;
//     AHB_SECURE_CTRL->CPU1_LOCK_REG           = 0x8000000AU;
//     AHB_SECURE_CTRL->MISC_CTRL_REG           = (AHB_SECURE_CTRL->MISC_CTRL_REG & 0x0FFFF0003U) | 0x00000AAA4U;
//     AHB_SECURE_CTRL->MISC_CTRL_DP_REG        = 0x0000AAA5U;
// }

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

	// SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL0_SRAM_CTRL1_MASK | SYSCON_AHBCLKCTRL0_SRAM_CTRL2_MASK |
    //                            SYSCON_AHBCLKCTRL0_SRAM_CTRL3_MASK | SYSCON_AHBCLKCTRL0_SRAM_CTRL4_MASK;

	// BOARD_InitTrustZone();

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

#define THE_BOOT_ADDR (DT_REG_ADDR(DT_CHOSEN(zephyr_code_cpu1_partition)))
// #define THE_BOOT_ADDR (char*)(CONFIG_SLAVE_BOOT_ADDRESS_MCUX)

	// memcpy((void *)THE_BOOT_ADDR, (void *)slave_core, sizeof(slave_core));

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
	SYSCON->CPBOOT = SYSCON_CPBOOT_CPBOOT(*(uint32_t *)((char*)(THE_BOOT_ADDR+0x0)));

    /* Boot source for Core 1 from flash */
    // SYSCON->CPBOOT = SYSCON_CPBOOT_CPBOOT(*(uint32_t *)DT_REG_ADDR(DT_CHOSEN(zephyr_code_cpu1_partition)));

    int32_t temp = SYSCON->CPUCTRL;
    temp |= 0xc0c40000;
    SYSCON->CPUCTRL = temp | SYSCON_CPUCTRL_CPU1RSTEN_MASK | SYSCON_CPUCTRL_CPU1CLKEN_MASK;
    SYSCON->CPUCTRL = (temp | SYSCON_CPUCTRL_CPU1CLKEN_MASK) & (~SYSCON_CPUCTRL_CPU1RSTEN_MASK);

	__DSB();

	return 0;
}

SYS_INIT(_slave_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

#endif /*CONFIG_SLAVE_CORE_MCUX*/

#ifdef CONFIG_NS_IMAGE_BOOT

#ifdef CONFIG_TRUSTED_EXECUTION_SECURE

#define NS_STACK_ADDRESS (DT_REG_ADDR(DT_CHOSEN(zephyr_code_non_secure_partition)))
#define NS_BOOT_ADDRESS  ((uint8_t*)NS_STACK_ADDRESS+0x04)

int _ns_init(struct device *arg)
{
	ARG_UNUSED(arg);
	// tz_ns_func_ptr_t ResetHandler_ns;

	printk("Hello my baby!...");

	__TZ_set_MSP_NS(*((uint32_t *)(NS_STACK_ADDRESS)));

	SCB_NS->VTOR = (uint32_t)NS_STACK_ADDRESS;

	// ResetHandler_ns = (tz_ns_func_ptr_t)(*((uint32_t *)((NS_STACK_ADDRESS) + 4U)));

	printk("Over to you!\n");

	// ResetHandler_ns();

	return 0;
}
#endif //CONFIG_TRUSTED_EXECUTION_SECURE


#ifdef CONFIG_TRUSTED_EXECUTION_NONSECURE
int _ns_init(struct device *arg)
{
	ARG_UNUSED(arg);

	printk("Hello my darling!\n");

	return 0;
}
#endif //CONFIG_TRUSTED_EXECUTION_NONSECURE


SYS_INIT(_ns_init, POST_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

#endif /*CONFIG_NS_IMAGE_BOOT*/


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

	return 0;
}

SYS_INIT(_slave_blink, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

#endif
