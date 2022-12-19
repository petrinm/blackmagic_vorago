/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2014 Allen Ibara <aibara>
 * Copyright (C) 2015 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2022 1BitSquared <info@1bitsquared.com>
 * Rewritten by Rachel Mant <git@dragonmux.network>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "cortexm.h"
#include "lpc_common.h"
#include "sfdp.h"

#define LPC43xx_CHIPID                0x40043200U
#define LPC43xx_CHIPID_FAMILY_MASK    0x0fffffffU
#define LPC43xx_CHIPID_FAMILY_CODE    0x0906002bU
#define LPC43xx_CHIPID_CHIP_MASK      0xf0000000U
#define LPC43xx_CHIPID_CHIP_SHIFT     28U
#define LPC43xx_CHIPID_CORE_TYPE_MASK 0xff0ffff0U
#define LPC43xx_CHIPID_CORE_TYPE_M0   0x4100c200U
#define LPC43xx_CHIPID_CORE_TYPE_M4   0x4100c240U

#define LPC43xx_PARTID_LOW     0x40045000U
#define LPC43xx_PARTID_HIGH    0x40045004U
#define LPC43xx_PARTID_INVALID 0x00000000U

/* Flashless parts */
#define LPC43xx_PARTID_LPC4310 0xa00acb3fU
#define LPC43xx_PARTID_LPC4320 0xa000cb3cU
#define LPC43xx_PARTID_LPC4330 0xa0000a30U
#define LPC43xx_PARTID_LPC4350 0xa0000830U
#define LPC43xx_PARTID_LPC4370 0x00000230U

/* Errata values for the part codes */
#define LPC43xx_PARTID_LPC4370_ERRATA 0x00000030U

/* On-chip Flash parts */
#define LPC43xx_PARTID_LPC4312 0xa00bcb3fU
#define LPC43xx_PARTID_LPC4315 0xa001cb3fU
#define LPC43xx_PARTID_LPC4322 0xa00bcb3cU
#define LPC43xx_PARTID_LPC4325 0xa001cb3cU
#define LPC43xx_PARTID_LPC433x 0xa0010a30U
#define LPC43xx_PARTID_LPC435x 0xa0010830U

/* Flash configurations */
#define LPC43xx_PARTID_FLASH_CONFIG_MASK 0x000000ffU
#define LPC43xx_PARTID_FLASH_CONFIG_NONE 0xffU
#define LPC43xx_PARTID_FLASH_CONFIG_43x2 0x80U
#define LPC43xx_PARTID_FLASH_CONFIG_43x3 0x44U
#define LPC43xx_PARTID_FLASH_CONFIG_43x5 0x22U
#define LPC43xx_PARTID_FLASH_CONFIG_43x7 0x00U

#define IAP_ENTRYPOINT_LOCATION 0x10400100U

#define LPC43xx_SHADOW_BASE      0x00000000U
#define LPC43xx_SHADOW_SIZE      0x10000000U
#define LPC43xx_LOCAL_SRAM1_BASE 0x10000000U
#define LPC43xx_LOCAL_SRAM1_SIZE (32U * 1024U)
#define LPC4310_LOCAL_SRAM1_SIZE (96U * 1024U)
#define LPC4330_LOCAL_SRAM1_SIZE (128U * 1024U)
#define LPC43xx_LOCAL_SRAM2_BASE 0x10080000U
#define LPC43xx_LOCAL_SRAM2_SIZE (40U * 1024U)
#define LPC43x0_LOCAL_SRAM2_SIZE (72U * 1024U)
#define LPC4370_M0_SRAM_BASE     0x18000000U
#define LPC4370_M0_SRAM_SIZE     (18U * 1024U)
#define LPC43xx_AHB_SRAM_BASE    0x20000000U
#define LPC43x2_AHB_SRAM_SIZE    (16U * 1024U)
#define LPC43x5_AHB_SRAM_SIZE    (48U * 1024U)
#define LPC43xx_ETBAHB_SRAM_BASE 0x2000c000U
#define LPC43xx_ETBAHB_SRAM_SIZE (16U * 1024U)

#define LPC43xx_SCU_BASE       0x40086000U
#define LPC43xx_SCU_BANK3_PIN3 (LPC43xx_SCU_BASE + 0x18cU)
#define LPC43xx_SCU_CLK0       (LPC43xx_SCU_BASE + 0xc00U)
#define LPC43xx_SCU_CLK1       (LPC43xx_SCU_BASE + 0xc04U)
#define LPC43xx_SCU_CLK2       (LPC43xx_SCU_BASE + 0xc08U)
#define LPC43xx_SCU_CLK3       (LPC43xx_SCU_BASE + 0xc0cU)

#define LPC43xx_SCU_PIN_MODE_MASK    0x00000007U
#define LPC43xx_SCU_PIN_MODE_SSP0    0x00000002U
#define LPC43xx_SCU_PIN_MODE_SPIFI   0x00000003U
#define LPC43xx_SCU_PIN_MODE_EMC_CLK 0x00000001U

#define LPC43xx_CGU_BASE               0x40050000U
#define LPC43xx_CGU_CPU_CLK            (LPC43xx_CGU_BASE + 0x06cU)
#define LPC43xx_CGU_BASE_CLK_AUTOBLOCK (1U << 11U)
#define LPC43xx_CGU_BASE_CLK_SEL_IRC   (1U << 24U)

#define LPC43xx_EMC_BASE                    0x40005100U
#define LPC43xx_EMC_DYN_CONFIG0             (LPC43xx_EMC_BASE + 0xc00U)
#define LPC43xx_EMC_DYN_CONFIG_MAPPING_MASK 0x00005000U
#define LPC43xx_EMC_DYN_CONFIG_MAPPING_8    0x00000000U
#define LPC43xx_EMC_DYN_CONFIG_MAPPING_16   0x00001000U

/* Cortex-M4 Application Interrupt and Reset Control Register */
#define LPC43xx_AIRCR 0xe000ed0cU
/* Magic value reset key */
#define LPC43xx_AIRCR_RESET 0x05fa0004U

#define LPC43xx_WDT_MODE       0x40080000U
#define LPC43xx_WDT_CNT        0x40080004U
#define LPC43xx_WDT_FEED       0x40080008U
#define LPC43xx_WDT_PERIOD_MAX 0xffffffU
#define LPC43xx_WDT_PROTECT    (1U << 4U)

#define IAP_RAM_SIZE LPC43xx_ETBAHB_SRAM_SIZE
#define IAP_RAM_BASE LPC43xx_ETBAHB_SRAM_BASE

#define IAP_PGM_CHUNKSIZE 4096U

#define FLASH_NUM_SECTOR 15U

#define LPC43xx_FLASH_BANK_A        0U
#define LPC43xx_FLASH_BANK_A_BASE   0x1a000000U
#define LPC43xx_FLASH_BANK_B        1U
#define LPC43xx_FLASH_BANK_B_BASE   0x1b000000U
#define LPC43xx_FLASH_8kiB          (8U * 1024U)
#define LPC43xx_FLASH_64kiB         (64U * 1024U)
#define LPC43xx_FLASH_128kiB        (128U * 1024U)
#define LPC43xx_FLASH_192kiB        (192U * 1024U)
#define LPC43xx_FLASH_256kiB        (256U * 1024U)
#define LPC43x0_SPI_FLASH_LOW_BASE  0x14000000U
#define LPC43x0_SPI_FLASH_LOW_SIZE  0x04000000U
#define LPC43x0_SPI_FLASH_HIGH_BASE 0x80000000U
#define LPC43x0_SPI_FLASH_HIGH_SIZE 0x08000000U

#define LPC43x0_SPIFI_BASE 0x40003000U
#define LPC43x0_SPIFI_CTRL (LPC43x0_SPIFI_BASE + 0x000U)
#define LPC43x0_SPIFI_CMD  (LPC43x0_SPIFI_BASE + 0x004U)
#define LPC43x0_SPIFI_ADDR (LPC43x0_SPIFI_BASE + 0x008U)
#define LPC43x0_SPIFI_IDAT (LPC43x0_SPIFI_BASE + 0x00cU)
#define LPC43x0_SPIFI_DATA (LPC43x0_SPIFI_BASE + 0x014U)
#define LPC43x0_SPIFI_MCMD (LPC43x0_SPIFI_BASE + 0x018U)
#define LPC43x0_SPIFI_STAT (LPC43x0_SPIFI_BASE + 0x01cU)

#define LPC43x0_SPIFI_DATA_LENGTH(x)       ((x)&0x00003fffU)
#define LPC43x0_SPIFI_DATA_IN              (0U << 15U)
#define LPC43x0_SPIFI_DATA_OUT             (1U << 15U)
#define LPC43x0_SPIFI_INTER_LENGTH(x)      (((x)&7U) << 16U)
#define LPC43x0_SPIFI_CMD_SERIAL           (0U << 19U)
#define LPC43x0_SPIFI_CMD_QUAD_OPCODE      (1U << 19U)
#define LPC43x0_SPIFI_CMD_SERIAL_OPCODE    (2U << 19U)
#define LPC43x0_SPIFI_CMD_QUAD             (3U << 19U)
#define LPC43x0_SPIFI_FRAME_OPCODE_ONLY    (1U << 21U)
#define LPC43x0_SPIFI_FRAME_OPCODE_1B_ADDR (2U << 21U)
#define LPC43x0_SPIFI_FRAME_OPCODE_2B_ADDR (3U << 21U)
#define LPC43x0_SPIFI_FRAME_OPCODE_3B_ADDR (4U << 21U)
#define LPC43x0_SPIFI_FRAME_OPCODE_4B_ADDR (5U << 21U)
#define LPC43x0_SPIFI_FRAME_MASK           0x00e00000U
#define LPC43x0_SPIFI_OPCODE(x)            ((x) << 24U)
#define LPC43x0_SPIFI_STATUS_CMD_ACTIVE    (1U << 1U)
#define LPC43x0_SPIFI_STATUS_RESET         (1U << 4U)

#define SPI_FLASH_CMD_WRITE_ENABLE                                                              \
	(LPC43x0_SPIFI_CMD_SERIAL | LPC43x0_SPIFI_FRAME_OPCODE_ONLY | LPC43x0_SPIFI_OPCODE(0x06U) | \
		LPC43x0_SPIFI_INTER_LENGTH(0))
#define SPI_FLASH_CMD_PAGE_PROGRAM                                                                \
	(LPC43x0_SPIFI_CMD_SERIAL | LPC43x0_SPIFI_FRAME_OPCODE_3B_ADDR | LPC43x0_SPIFI_OPCODE(0x02) | \
		LPC43x0_SPIFI_DATA_OUT | LPC43x0_SPIFI_INTER_LENGTH(0))
#define SPI_FLASH_CMD_SECTOR_ERASE                                                                 \
	(LPC43x0_SPIFI_CMD_SERIAL | LPC43x0_SPIFI_FRAME_OPCODE_3B_ADDR | LPC43x0_SPIFI_OPCODE(0x20U) | \
		LPC43x0_SPIFI_INTER_LENGTH(0))
#define SPI_FLASH_CMD_CHIP_ERASE                                                                \
	(LPC43x0_SPIFI_CMD_SERIAL | LPC43x0_SPIFI_FRAME_OPCODE_ONLY | LPC43x0_SPIFI_OPCODE(0x60U) | \
		LPC43x0_SPIFI_INTER_LENGTH(0))
#define SPI_FLASH_CMD_READ_STATUS                                                               \
	(LPC43x0_SPIFI_CMD_SERIAL | LPC43x0_SPIFI_FRAME_OPCODE_ONLY | LPC43x0_SPIFI_OPCODE(0x05U) | \
		LPC43x0_SPIFI_DATA_IN | LPC43x0_SPIFI_INTER_LENGTH(0))
#define SPI_FLASH_CMD_READ_JEDEC_ID                                                             \
	(LPC43x0_SPIFI_CMD_SERIAL | LPC43x0_SPIFI_FRAME_OPCODE_ONLY | LPC43x0_SPIFI_OPCODE(0x9fU) | \
		LPC43x0_SPIFI_DATA_IN | LPC43x0_SPIFI_INTER_LENGTH(0))
#define SPI_FLASH_CMD_READ_SFDP                                                                    \
	(LPC43x0_SPIFI_CMD_SERIAL | LPC43x0_SPIFI_FRAME_OPCODE_3B_ADDR | LPC43x0_SPIFI_OPCODE(0x5aU) | \
		LPC43x0_SPIFI_DATA_IN | LPC43x0_SPIFI_INTER_LENGTH(1))

#define SPI_FLASH_STATUS_BUSY          0x01U
#define SPI_FLASH_STATUS_WRITE_ENABLED 0x02U

typedef enum lpc43x0_flash_interface {
	FLASH_NONE,
	FLASH_SPIFI,
	FLASH_EMC8,
	FLASH_EMC16,
	FLASH_EMC32,
	FLASH_SPI
} lpc43x0_flash_interface_e;

typedef struct lpc43xx_partid {
	uint32_t part;
	uint8_t flash_config;
} lpc43xx_partid_s;

typedef struct lpc43xx_spi_flash {
	target_flash_s flash_low;
	target_flash_s flash_high;
	uint32_t read_command;
	uint32_t page_size;
	uint8_t sector_erase_opcode;
} lpc43xx_spi_flash_s;

typedef struct lpc43xx_priv {
	uint8_t flash_banks;
} lpc43xx_priv_s;

typedef struct lpc43x0_priv {
	lpc43xx_spi_flash_s *flash;
	lpc43x0_flash_interface_e interface;
} lpc43x0_priv_s;

static bool lpc43xx_cmd_reset(target_s *t, int argc, const char **argv);
static bool lpc43xx_cmd_mkboot(target_s *t, int argc, const char **argv);

static lpc43xx_partid_s lpc43x0_spi_read_partid(target_s *t);
static bool lpc43x0_attach(target_s *t);
static void lpc43x0_detach(target_s *t);
static void lpc43x0_spi_abort(target_s *t);
static void lpc43x0_spi_read(target_s *t, uint32_t command, target_addr_t address, void *buffer, size_t length);
static void lpc43x0_spi_write(target_s *t, uint32_t command, target_addr_t address, const void *buffer, size_t length);
static bool lpc43x0_spi_mass_erase(target_s *t);
static bool lpc43x0_spi_flash_erase(target_flash_s *f, target_addr_t addr, size_t length);
static bool lpc43x0_spi_flash_write(target_flash_s *f, target_addr_t dest, const void *src, size_t length);

static bool lpc43xx_iap_init(target_flash_s *flash);
static lpc43xx_partid_s lpc43xx_iap_read_partid(target_s *t);
static bool lpc43xx_iap_flash_erase(target_flash_s *f, target_addr_t addr, size_t len);
static bool lpc43xx_iap_mass_erase(target_s *t);
static void lpc43xx_wdt_set_period(target_s *t);
static void lpc43xx_wdt_kick(target_s *t);

const command_s lpc43xx_cmd_list[] = {
	{"reset", lpc43xx_cmd_reset, "Reset target"},
	{"mkboot", lpc43xx_cmd_mkboot, "Make flash bank bootable"},
	{NULL, NULL, NULL},
};

static void lpc43xx_add_iap_flash(
	target_s *t, uint32_t iap_entry, uint8_t bank, uint8_t base_sector, uint32_t addr, size_t len, size_t erasesize)
{
	lpc_flash_s *lf = lpc_add_flash(t, addr, len);
	lf->f.erase = lpc43xx_iap_flash_erase;
	lf->f.blocksize = erasesize;
	lf->f.writesize = IAP_PGM_CHUNKSIZE;
	lf->bank = bank;
	lf->base_sector = base_sector;
	lf->iap_entry = iap_entry;
	lf->iap_ram = IAP_RAM_BASE;
	lf->iap_msp = IAP_RAM_BASE + IAP_RAM_SIZE;
	lf->wdt_kick = lpc43xx_wdt_kick;
}

static void lpc43xx_detect(target_s *const t, const lpc43xx_partid_s part_id)
{
	lpc43xx_priv_s *const priv = (lpc43xx_priv_s *)t->target_storage;
	const uint32_t iap_entry = target_mem_read32(t, IAP_ENTRYPOINT_LOCATION);

	switch (part_id.part) {
	case LPC43xx_PARTID_LPC4312:
		t->driver = "LPC4312/3";
		target_add_ram(t, LPC43xx_AHB_SRAM_BASE, LPC43x2_AHB_SRAM_SIZE);
		break;
	case LPC43xx_PARTID_LPC4315:
		t->driver = "LPC4315/7";
		target_add_ram(t, LPC43xx_AHB_SRAM_BASE, LPC43x5_AHB_SRAM_SIZE);
		break;
	case LPC43xx_PARTID_LPC4322:
		t->driver = "LPC4322/3";
		target_add_ram(t, LPC43xx_AHB_SRAM_BASE, LPC43x2_AHB_SRAM_SIZE);
		break;
	case LPC43xx_PARTID_LPC4325:
		t->driver = "LPC4325/7";
		target_add_ram(t, LPC43xx_AHB_SRAM_BASE, LPC43x5_AHB_SRAM_SIZE);
		break;
	case LPC43xx_PARTID_LPC433x:
		t->driver = "LPC433x";
		target_add_ram(t, LPC43xx_AHB_SRAM_BASE, LPC43x5_AHB_SRAM_SIZE);
		break;
	case LPC43xx_PARTID_LPC435x:
		t->driver = "LPC435x";
		target_add_ram(t, LPC43xx_AHB_SRAM_BASE, LPC43x5_AHB_SRAM_SIZE);
		break;
	}
	target_add_ram(t, LPC43xx_SHADOW_BASE, LPC43xx_SHADOW_SIZE);
	target_add_ram(t, LPC43xx_LOCAL_SRAM1_BASE, LPC43xx_LOCAL_SRAM1_SIZE);
	target_add_ram(t, LPC43xx_LOCAL_SRAM2_BASE, LPC43xx_LOCAL_SRAM2_SIZE);
	target_add_ram(t, LPC43xx_ETBAHB_SRAM_BASE, LPC43xx_ETBAHB_SRAM_SIZE);

	/* All parts with Flash have the first 64kiB bank A region */
	lpc43xx_add_iap_flash(
		t, iap_entry, LPC43xx_FLASH_BANK_A, 0U, LPC43xx_FLASH_BANK_A_BASE, LPC43xx_FLASH_64kiB, LPC43xx_FLASH_8kiB);
	/* All parts other than LP43x2 with Flash have the first 64kiB bank B region */
	if (part_id.flash_config != LPC43xx_PARTID_FLASH_CONFIG_43x2) {
		lpc43xx_add_iap_flash(
			t, iap_entry, LPC43xx_FLASH_BANK_B, 0U, LPC43xx_FLASH_BANK_B_BASE, LPC43xx_FLASH_64kiB, LPC43xx_FLASH_8kiB);
		priv->flash_banks = 2;
	} else
		priv->flash_banks = 1;

	switch (part_id.flash_config) {
	case LPC43xx_PARTID_FLASH_CONFIG_43x2:
		/* LP43x2 parts have a full bank A but not bank B */
		lpc43xx_add_iap_flash(t, iap_entry, LPC43xx_FLASH_BANK_A, 8U, LPC43xx_FLASH_BANK_A_BASE + LPC43xx_FLASH_64kiB,
			LPC43xx_FLASH_192kiB + LPC43xx_FLASH_256kiB, LPC43xx_FLASH_64kiB);
		break;
	case LPC43xx_PARTID_FLASH_CONFIG_43x3:
		/* LP43x3 parts have the first 256kiB of both bank A and bank B */
		lpc43xx_add_iap_flash(t, iap_entry, LPC43xx_FLASH_BANK_A, 8U, LPC43xx_FLASH_BANK_A_BASE + LPC43xx_FLASH_64kiB,
			LPC43xx_FLASH_192kiB, LPC43xx_FLASH_64kiB);
		lpc43xx_add_iap_flash(t, iap_entry, LPC43xx_FLASH_BANK_B, 8U, LPC43xx_FLASH_BANK_B_BASE + LPC43xx_FLASH_64kiB,
			LPC43xx_FLASH_192kiB, LPC43xx_FLASH_64kiB);
		break;
	case LPC43xx_PARTID_FLASH_CONFIG_43x5:
		/* LP43x3 parts have the first 256kiB and an additional 128kiB of both bank A and bank B */
		lpc43xx_add_iap_flash(t, iap_entry, LPC43xx_FLASH_BANK_A, 8U, LPC43xx_FLASH_BANK_A_BASE + LPC43xx_FLASH_64kiB,
			LPC43xx_FLASH_192kiB + LPC43xx_FLASH_128kiB, LPC43xx_FLASH_64kiB);
		lpc43xx_add_iap_flash(t, iap_entry, LPC43xx_FLASH_BANK_B, 8U, LPC43xx_FLASH_BANK_B_BASE + LPC43xx_FLASH_64kiB,
			LPC43xx_FLASH_192kiB + LPC43xx_FLASH_128kiB, LPC43xx_FLASH_64kiB);
		break;
	case LPC43xx_PARTID_FLASH_CONFIG_43x7:
		/* LP43x3 parts have the full 512kiB each of both bank A and bank B */
		lpc43xx_add_iap_flash(t, iap_entry, LPC43xx_FLASH_BANK_A, 8U, LPC43xx_FLASH_BANK_A_BASE + LPC43xx_FLASH_64kiB,
			LPC43xx_FLASH_192kiB + LPC43xx_FLASH_256kiB, LPC43xx_FLASH_64kiB);
		lpc43xx_add_iap_flash(t, iap_entry, LPC43xx_FLASH_BANK_B, 8U, LPC43xx_FLASH_BANK_B_BASE + LPC43xx_FLASH_64kiB,
			LPC43xx_FLASH_192kiB + LPC43xx_FLASH_256kiB, LPC43xx_FLASH_64kiB);
		break;
	}

	target_add_commands(t, lpc43xx_cmd_list, "LPC43xx");
}

static void lpc43x0_spi_read_sfdp(target_s *const t, const uint32_t address, void *const buffer, const size_t length)
{
	lpc43x0_spi_read(t, SPI_FLASH_CMD_READ_SFDP, address, buffer, length);
}

static void lpc43x0_add_spi_flash(target_s *const t, const size_t length, const uint32_t read_command)
{
	lpc43xx_spi_flash_s *const flash = calloc(1, sizeof(*flash));
	if (!flash) {
		DEBUG_WARN("calloc: failed in %s\n", __func__);
		return;
	}
	lpc43x0_priv_s *const priv = (lpc43x0_priv_s *)t->target_storage;
	priv->flash = flash;

	spi_parameters_s spi_parameters;
	if (!sfdp_read_parameters(t, &spi_parameters, lpc43x0_spi_read_sfdp)) {
		/* SFDP readout failed, so make some assumptions and hope for the best. */
		spi_parameters.page_size = 256U;
		spi_parameters.sector_size = 4096U;
		spi_parameters.capacity = length;
		spi_parameters.sector_erase_opcode = 0x20U;
	}

	/* Add the high region first so it appears second in the map */
	target_flash_s *const flash_high = &flash->flash_high;
	flash_high->start = LPC43x0_SPI_FLASH_HIGH_BASE;
	flash_high->length = MIN(spi_parameters.capacity, LPC43x0_SPI_FLASH_HIGH_SIZE);
	flash_high->blocksize = spi_parameters.sector_size;
	flash_high->write = lpc43x0_spi_flash_write;
	flash_high->erase = lpc43x0_spi_flash_erase;
	flash_high->erased = 0xffU;
	target_add_flash(t, flash_high);

	/*
	 * Then add the low region - the reason for this is that
	 * target_add_flash inserts new entries to the beginning of the
	 * Flash linked-list in the target structure, so this becomes t->flash.
	 */
	target_flash_s *const flash_low = &flash->flash_low;
	flash_low->start = LPC43x0_SPI_FLASH_LOW_BASE;
	flash_low->length = MIN(spi_parameters.capacity, LPC43x0_SPI_FLASH_LOW_SIZE);
	flash_low->blocksize = spi_parameters.sector_size;
	flash_low->write = lpc43x0_spi_flash_write;
	flash_low->erase = lpc43x0_spi_flash_erase;
	flash_low->erased = 0xffU;
	target_add_flash(t, flash_low);

	flash->read_command = read_command;
	flash->page_size = spi_parameters.page_size;
	flash->sector_erase_opcode = spi_parameters.sector_erase_opcode;
}

static void lpc43x0_detect(target_s *const t, const lpc43xx_partid_s part_id)
{
	switch (part_id.part) {
	case LPC43xx_PARTID_LPC4310:
		t->driver = "LPC4310";
		target_add_ram(t, LPC43xx_LOCAL_SRAM1_BASE, LPC4310_LOCAL_SRAM1_SIZE);
		target_add_ram(t, LPC43xx_LOCAL_SRAM2_BASE, LPC43xx_LOCAL_SRAM2_SIZE);
		target_add_ram(t, LPC43xx_AHB_SRAM_BASE, LPC43x2_AHB_SRAM_SIZE);
		break;
	case LPC43xx_PARTID_LPC4320:
		t->driver = "LPC4320";
		target_add_ram(t, LPC43xx_LOCAL_SRAM1_BASE, LPC4310_LOCAL_SRAM1_SIZE);
		target_add_ram(t, LPC43xx_LOCAL_SRAM2_BASE, LPC43xx_LOCAL_SRAM2_SIZE);
		target_add_ram(t, LPC43xx_AHB_SRAM_BASE, LPC43x5_AHB_SRAM_SIZE);
		break;
	case LPC43xx_PARTID_LPC4330:
	case LPC43xx_PARTID_LPC4350:
		t->driver = "LPC4330/50";
		target_add_ram(t, LPC43xx_LOCAL_SRAM1_BASE, LPC4330_LOCAL_SRAM1_SIZE);
		target_add_ram(t, LPC43xx_LOCAL_SRAM2_BASE, LPC43x0_LOCAL_SRAM2_SIZE);
		target_add_ram(t, LPC43xx_AHB_SRAM_BASE, LPC43x5_AHB_SRAM_SIZE);
		break;
	case LPC43xx_PARTID_LPC4370:
	case LPC43xx_PARTID_LPC4370_ERRATA:
		t->driver = "LPC4370";
		target_add_ram(t, LPC43xx_LOCAL_SRAM1_BASE, LPC4330_LOCAL_SRAM1_SIZE);
		target_add_ram(t, LPC43xx_LOCAL_SRAM2_BASE, LPC43x0_LOCAL_SRAM2_SIZE);
		target_add_ram(t, LPC4370_M0_SRAM_BASE, LPC4370_M0_SRAM_SIZE);
		target_add_ram(t, LPC43xx_AHB_SRAM_BASE, LPC43x5_AHB_SRAM_SIZE);
		break;
	default:
		DEBUG_WARN("Probable LPC43x0 with ID errata: %08" PRIx32 "\n", part_id.part);
		break;
	}
	t->attach = lpc43x0_attach;
	t->detach = lpc43x0_detach;
}

bool lpc43xx_probe(target_s *const t)
{
	const uint32_t chipid = target_mem_read32(t, LPC43xx_CHIPID);
	if ((chipid & LPC43xx_CHIPID_FAMILY_MASK) != LPC43xx_CHIPID_FAMILY_CODE)
		return false;

	const uint32_t chip_code = (chipid & LPC43xx_CHIPID_CHIP_MASK) >> LPC43xx_CHIPID_CHIP_SHIFT;
	t->target_options |= CORTEXM_TOPT_INHIBIT_NRST;

	/* 4 is for rev '-' parts with on-chip Flash, 7 is for rev 'A' parts with on-chip Flash */
	if (chip_code == 4U || chip_code == 7U) {
		const lpc43xx_partid_s part_id = lpc43xx_iap_read_partid(t);
		DEBUG_WARN("LPC43xx part ID: 0x%08" PRIx32 ":%02x\n", part_id.part, part_id.flash_config);
		if (part_id.part == LPC43xx_PARTID_INVALID)
			return false;

		lpc43xx_priv_s *priv = calloc(1, sizeof(lpc43xx_priv_s));
		if (!priv) { /* calloc failed: heap exhaustion */
			DEBUG_WARN("calloc: failed in %s\n", __func__);
			return false;
		}
		t->target_storage = priv;

		t->mass_erase = lpc43xx_iap_mass_erase;
		lpc43xx_detect(t, part_id);
	} else if (chip_code == 5U || chip_code == 6U) {
		const lpc43xx_partid_s part_id = lpc43x0_spi_read_partid(t);
		DEBUG_WARN("LPC43xx part ID: 0x%08" PRIx32 ":%02x\n", part_id.part, part_id.flash_config);
		if (part_id.part == LPC43xx_PARTID_INVALID)
			return false;

		t->mass_erase = lpc43x0_spi_mass_erase;
		lpc43x0_detect(t, part_id);
	} else
		return false;
	return true;
}

/* LPC43xx Flashless part routines */

static lpc43x0_flash_interface_e lpc43x0_determine_flash_interface(target_s *const t)
{
	const uint32_t clk_pin_mode = target_mem_read32(t, LPC43xx_SCU_BANK3_PIN3) & LPC43xx_SCU_PIN_MODE_MASK;
	if (clk_pin_mode == LPC43xx_SCU_PIN_MODE_SPIFI)
		return FLASH_SPIFI;
	if (clk_pin_mode == LPC43xx_SCU_PIN_MODE_SSP0)
		return FLASH_SPI;
	if ((target_mem_read32(t, LPC43xx_SCU_CLK0) & LPC43xx_SCU_PIN_MODE_MASK) == LPC43xx_SCU_PIN_MODE_EMC_CLK) {
		const uint32_t emc_config = target_mem_read32(t, LPC43xx_EMC_DYN_CONFIG0) & LPC43xx_EMC_DYN_CONFIG_MAPPING_MASK;
		if (emc_config == LPC43xx_EMC_DYN_CONFIG_MAPPING_8)
			return FLASH_EMC8;
		if (emc_config == LPC43xx_EMC_DYN_CONFIG_MAPPING_16)
			return FLASH_EMC16;
		return FLASH_EMC32;
	}
	return FLASH_NONE;
}

static bool lpc43x0_attach(target_s *const t)
{
	if (!cortexm_attach(t))
		return false;

	if (!t->target_storage) {
		lpc43x0_priv_s *priv = calloc(1, sizeof(lpc43x0_priv_s));
		if (!priv) { /* calloc failed: heap exhaustion */
			DEBUG_WARN("calloc: failed in %s\n", __func__);
			return false;
		}
		t->target_storage = priv;

		/*
		* Before we can go down a specific route here, we first have to figure out how the device was booted:
		* - Was it bought up on the SPIFI interface
		* - Was it bought up on SSP0
		* - Was it bought up on the EMC interface
		*
		* Once this is ascertained, we can pick how to proceed.
		*
		* Start by reading 0x40045030 - OTP[3,0], Customer control data.
		* If bits 25:28 read as 0, boot is controlled by the external pins, otherwise
		* this determines the boot source. 2 for SPIFI, 3 through 5 for EMC, and 8 for SPI
		* For external pins, P1_1, P1_2, P2_8 and P2_9 control the process.
		*
		* When assembled as [P2_9, P2_9, P1_2, P1_1] and interpreted as a bitvector, the following holds:
		* - 0b0001 -> SPIFI
		* - 0b0010 -> EMC (8-bit)
		* - 0b0011 -> EMC (16-bit)
		* - 0b0100 -> EMC (32-bit)
		* - 0b0111 -> SPI (SSP0)
		*
		* We don't actually care about any of the other modes as they're inconsequential.
		* If the boot source contains a header prior to the image or is SPI boot, the header
		* is validated and the image copied to SRAM at 0x10000000, then executed from there.
		* If the boot source is anything other than SPI and the image contains no header,
		* the chip sets switches execution to that boot source.
		*
		* This process is laid out in Chaper 5 of UM10503. See Fig 16 on pg 59 for a more detailed view.
		*/
		const lpc43x0_flash_interface_e interface = lpc43x0_determine_flash_interface(t);
		priv->interface = interface;
	}

	const uint32_t read_command = target_mem_read32(t, LPC43x0_SPIFI_MCMD);
	lpc43x0_spi_abort(t);
	spi_flash_id_s flash_id;
	lpc43x0_spi_read(t, SPI_FLASH_CMD_READ_JEDEC_ID, 0, &flash_id, sizeof(flash_id));

	/* If we read out valid Flash information, set up a region for it */
	if (flash_id.manufacturer != 0xffU && flash_id.type != 0xffU && flash_id.capacity != 0xffU) {
		const uint32_t capacity = 1U << flash_id.capacity;
		DEBUG_WARN("SPI Flash: mfr = %02x, type = %02x, capacity = %08" PRIx32 "\n", flash_id.manufacturer,
			flash_id.type, capacity);
		lpc43x0_add_spi_flash(t, capacity, read_command);
	}
	/* Restore the read command as using the other command register clobbers this. */
	target_mem_write32(t, LPC43x0_SPIFI_MCMD, read_command);
	return true;
}

static void lpc43x0_detach(target_s *const t)
{
	lpc43x0_priv_s *const priv = (lpc43x0_priv_s *)t->target_storage;
	free(priv->flash);
	priv->flash = NULL;
	t->flash = NULL;
	cortexm_detach(t);
}

/*
 * It is for reasons of errata that we don't use the IAP device identification mechanism here.
 * Instead, we have to read out the bank 0 OTP bytes to fetch the part identification code.
 * Unfortunately it appears this itself has errata and doesn't line up with the values in the datasheet.
 */
static lpc43xx_partid_s lpc43x0_spi_read_partid(target_s *const t)
{
	lpc43xx_partid_s result;
	result.part = target_mem_read32(t, LPC43xx_PARTID_LOW);
	result.flash_config = LPC43xx_PARTID_FLASH_CONFIG_NONE;
	return result;
}

static void lpc43x0_spi_abort(target_s *const t)
{
	target_mem_write32(t, LPC43x0_SPIFI_STAT, LPC43x0_SPIFI_STATUS_RESET);
	while (target_mem_read32(t, LPC43x0_SPIFI_STAT) & LPC43x0_SPIFI_STATUS_RESET)
		continue;
}

static inline void lpc43x0_spi_wait_complete(target_s *const t)
{
	while (target_mem_read32(t, LPC43x0_SPIFI_STAT) & LPC43x0_SPIFI_STATUS_CMD_ACTIVE)
		continue;
}

static void lpc43x0_spi_read(
	target_s *const t, const uint32_t command, const target_addr_t address, void *const buffer, const size_t length)
{
	if ((command & LPC43x0_SPIFI_FRAME_MASK) != LPC43x0_SPIFI_FRAME_OPCODE_ONLY)
		target_mem_write32(t, LPC43x0_SPIFI_ADDR, address);
	target_mem_write32(t, LPC43x0_SPIFI_CMD, command | LPC43x0_SPIFI_DATA_LENGTH(length));
	uint8_t *const data = (uint8_t *)buffer;
	for (size_t i = 0; i < length; ++i)
		data[i] = target_mem_read8(t, LPC43x0_SPIFI_DATA);
	lpc43x0_spi_wait_complete(t);
}

static void lpc43x0_spi_write(target_s *const t, const uint32_t command, const target_addr_t address,
	const void *const buffer, const size_t length)
{
	if ((command & LPC43x0_SPIFI_FRAME_MASK) != LPC43x0_SPIFI_FRAME_OPCODE_ONLY)
		target_mem_write32(t, LPC43x0_SPIFI_ADDR, address);
	target_mem_write32(t, LPC43x0_SPIFI_CMD, command | LPC43x0_SPIFI_DATA_LENGTH(length));
	const uint8_t *const data = (const uint8_t *)buffer;
	for (size_t i = 0; i < length; ++i)
		target_mem_write8(t, LPC43x0_SPIFI_DATA, data[i]);
	lpc43x0_spi_wait_complete(t);
}

static inline uint8_t lpc43x0_spi_read_status(target_s *const t)
{
	uint8_t status;
	lpc43x0_spi_read(t, SPI_FLASH_CMD_READ_STATUS, 0, &status, sizeof(status));
	return status;
}

static void lpc43x0_spi_run_command(target_s *const t, const uint32_t command)
{
	target_mem_write32(t, LPC43x0_SPIFI_CMD, command);
	lpc43x0_spi_wait_complete(t);
}

static bool lpc43x0_spi_mass_erase(target_s *const t)
{
	platform_timeout_s timeout;
	platform_timeout_set(&timeout, 500);
	lpc43x0_spi_abort(t);
	lpc43x0_spi_run_command(t, SPI_FLASH_CMD_WRITE_ENABLE);
	if (!(lpc43x0_spi_read_status(t) & SPI_FLASH_STATUS_WRITE_ENABLED))
		return false;

	lpc43x0_spi_run_command(t, SPI_FLASH_CMD_CHIP_ERASE);
	while (lpc43x0_spi_read_status(t) & SPI_FLASH_STATUS_BUSY)
		target_print_progress(&timeout);

	const lpc43xx_spi_flash_s *const flash = (lpc43xx_spi_flash_s *)t->flash;
	target_mem_write32(t, LPC43x0_SPIFI_MCMD, flash->read_command);
	return true;
}

static bool lpc43x0_spi_flash_erase(target_flash_s *f, target_addr_t addr, size_t length)
{
	target_s *const t = f->t;
	const target_addr_t begin = addr - f->start;
	lpc43x0_spi_abort(t);
	for (size_t offset = 0; offset < length; offset += f->blocksize) {
		lpc43x0_spi_run_command(t, SPI_FLASH_CMD_WRITE_ENABLE);
		if (!(lpc43x0_spi_read_status(t) & SPI_FLASH_STATUS_WRITE_ENABLED))
			return false;

		lpc43x0_spi_write(t, SPI_FLASH_CMD_SECTOR_ERASE, begin + offset, NULL, 0);
		while (lpc43x0_spi_read_status(t) & SPI_FLASH_STATUS_BUSY)
			continue;
	}

	const lpc43xx_spi_flash_s *const flash = (lpc43xx_spi_flash_s *)t->flash;
	target_mem_write32(t, LPC43x0_SPIFI_MCMD, flash->read_command);
	return true;
}

static bool lpc43x0_spi_flash_write(target_flash_s *f, target_addr_t dest, const void *src, size_t length)
{
	target_s *const t = f->t;
	const target_addr_t begin = dest - f->start;
	const char *buffer = src;
	lpc43x0_spi_abort(t);
	for (size_t offset = 0; offset < length; offset += 256U) {
		lpc43x0_spi_run_command(t, SPI_FLASH_CMD_WRITE_ENABLE);
		if (!(lpc43x0_spi_read_status(t) & SPI_FLASH_STATUS_WRITE_ENABLED))
			return false;

		const size_t amount = MIN(length - offset, 256U);
		lpc43x0_spi_write(t, SPI_FLASH_CMD_PAGE_PROGRAM, begin + offset, buffer + offset, amount);
		while (lpc43x0_spi_read_status(t) & SPI_FLASH_STATUS_BUSY)
			continue;
	}

	const lpc43xx_spi_flash_s *const flash = (lpc43xx_spi_flash_s *)t->flash;
	target_mem_write32(t, LPC43x0_SPIFI_MCMD, flash->read_command);
	return true;
}

/* LPC43xx IAP On-board Flash part routines */

static bool lpc43xx_iap_init(target_flash_s *const flash)
{
	target_s *const t = flash->t;
	lpc_flash_s *const f = (lpc_flash_s *)flash;
	/* Deal with WDT */
	lpc43xx_wdt_set_period(t);

	/* Force internal clock */
	target_mem_write32(t, LPC43xx_CGU_CPU_CLK, LPC43xx_CGU_BASE_CLK_AUTOBLOCK | LPC43xx_CGU_BASE_CLK_SEL_IRC);

	/* Initialize flash IAP */
	return lpc_iap_call(f, NULL, IAP_CMD_INIT) == IAP_STATUS_CMD_SUCCESS;
}

/*
 * We can for the on-chip Flash parts use the IAP, so do so as this way the ID codes line up with
 * the ones in the datasheet.
 */
static lpc43xx_partid_s lpc43xx_iap_read_partid(target_s *const t)
{
	/* Define a fake Flash structure so we can invoke the IAP system */
	lpc_flash_s flash;
	flash.f.t = t;
	flash.wdt_kick = lpc43xx_wdt_kick;
	flash.iap_entry = target_mem_read32(t, IAP_ENTRYPOINT_LOCATION);
	flash.iap_ram = IAP_RAM_BASE;
	flash.iap_msp = IAP_RAM_BASE + IAP_RAM_SIZE;

	/* Prepare a failure result in case readback fails */
	lpc43xx_partid_s result;
	result.part = LPC43xx_PARTID_INVALID;
	result.flash_config = LPC43xx_PARTID_FLASH_CONFIG_NONE;

	/* Read back the part ID
	 * XXX: We only use the first 2 values but because of limitations in lpc_iap_call,
	 * we have to declare an array of 4
	 */
	uint32_t part_id[4];
	if (!lpc43xx_iap_init(&flash.f) || lpc_iap_call(&flash, part_id, IAP_CMD_PARTID) != IAP_STATUS_CMD_SUCCESS)
		return result;

	/* Prepare the result and return it */
	result.part = part_id[0];
	result.flash_config = part_id[1] & LPC43xx_PARTID_FLASH_CONFIG_MASK;
	return result;
}

static bool lpc43xx_iap_flash_erase(target_flash_s *f, const target_addr_t addr, const size_t len)
{
	if (!lpc43xx_iap_init(f))
		return false;
	return lpc_flash_erase(f, addr, len);
}

static bool lpc43xx_iap_mass_erase(target_s *t)
{
	lpc43xx_priv_s *const priv = (lpc43xx_priv_s *)t->target_storage;
	platform_timeout_s timeout;
	platform_timeout_set(&timeout, 500);
	lpc43xx_iap_init(t->flash);

	for (size_t bank = 0; bank < priv->flash_banks; ++bank) {
		lpc_flash_s *const f = (lpc_flash_s *)t->flash;
		if (lpc_iap_call(f, NULL, IAP_CMD_PREPARE, 0, FLASH_NUM_SECTOR - 1U, bank) ||
			lpc_iap_call(f, NULL, IAP_CMD_ERASE, 0, FLASH_NUM_SECTOR - 1U, CPU_CLK_KHZ, bank))
			return false;
		target_print_progress(&timeout);
	}

	return true;
}

/* Reset all major systems _except_ debug */
static bool lpc43xx_cmd_reset(target_s *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	/* System reset on target */
	target_mem_write32(t, LPC43xx_AIRCR, LPC43xx_AIRCR_RESET);
	return true;
}

/*
 * Call Boot ROM code to make a flash bank bootable by computing and writing the
 * correct signature into the exception table near the start of the bank.
 *
 * This is done independently of writing to give the user a chance to verify flash
 * before changing it.
 */
static bool lpc43xx_cmd_mkboot(target_s *t, int argc, const char **argv)
{
	/* Usage: mkboot 0 or mkboot 1 */
	if (argc != 2) {
		tc_printf(t, "Expected bank argument 0 or 1.\n");
		return false;
	}

	const uint32_t bank = strtoul(argv[1], NULL, 0);
	if (bank > 1) {
		tc_printf(t, "Unexpected bank number, should be 0 or 1.\n");
		return false;
	}

	lpc43xx_iap_init(t->flash);

	/* special command to compute/write magic vector for signature */
	lpc_flash_s *f = (lpc_flash_s *)t->flash;
	if (lpc_iap_call(f, NULL, IAP_CMD_SET_ACTIVE_BANK, bank, CPU_CLK_KHZ)) {
		tc_printf(t, "Set bootable failed.\n");
		return false;
	}

	tc_printf(t, "Set bootable OK.\n");
	return true;
}

static void lpc43xx_wdt_set_period(target_s *t)
{
	/* Check if WDT is on */
	uint32_t wdt_mode = target_mem_read32(t, LPC43xx_WDT_MODE);

	/* If WDT on, we can't disable it, but we may be able to set a long period */
	if (wdt_mode && !(wdt_mode & LPC43xx_WDT_PROTECT))
		target_mem_write32(t, LPC43xx_WDT_CNT, LPC43xx_WDT_PERIOD_MAX);
}

static void lpc43xx_wdt_kick(target_s *t)
{
	/* Check if WDT is on */
	uint32_t wdt_mode = target_mem_read32(t, LPC43xx_WDT_MODE);

	/* If WDT on, kick it so we don't get the target reset */
	if (wdt_mode) {
		target_mem_write32(t, LPC43xx_WDT_FEED, 0xaa);
		target_mem_write32(t, LPC43xx_WDT_FEED, 0xff);
	}
}
