#include <stdint.h>

#include "general.h"
#include "hex_utils.h"
#include "target.h"
#include "target_probe.h"
#include "target_internal.h"
#include "cortexm.h"


static bool vorago_cmd_efuse(target_s *target, int argc, const char **argv);
static bool vorago_cmd_fram(target_s *target, int argc, const char **argv);

const command_s vorago_cmd_list[] = {
	{"efuse", vorago_cmd_efuse, "Manipulate eFuses"},
	{"fram", vorago_cmd_fram, "Manipulate FRAM"},
	{NULL, NULL, NULL},
};


/* Custom JTAG commands for Vorago JTAG Controller */
#define EF_ADDR            	0x14
#define EF_WDATA            0x15
#define EF_RDATA            0x16
#define EF_TIMING           0x17
#define EF_CMD	            0x18
#define EF_STATUS           0x19
#define EF_SPI_CONFIG       0x1A
#define EF_SPI_ENCAP        0x1B

/* Generic SPI memory opcodes */
#define SPI_MEMORY_WREN     0x06 // Set write enable latch
#define SPI_MEMORY_WRDI     0x04 // Reset write enable latch
#define SPI_MEMORY_WRSR     0x01 // Write status register
#define SPI_MEMORY_RDSR     0x05 // Read status register
#define SPI_MEMORY_READ     0x03 // Read memory data
#define SPI_MEMORY_WRITE    0x02 // Write memory data

/* Look-up table for reversing bits */
#define R2(n) n, n + 2 * 64, n + 1 * 64, n + 3 * 64
#define R4(n) R2(n), R2(n + 2 * 16), R2(n + 1 * 16), R2(n + 3 * 16)
#define R6(n) R4(n), R4(n + 2 * 4), R4(n + 1 * 4), R4(n + 3 * 4)
static const uint8_t reverse[256] = { R6(0), R6(2), R6(1), R6(3)};


static uint8_t vorago_dev_index = 255;
static uint8_t fram_burn = 0;


/* Read efuse register */
static uint32_t read_efuse_register(uint8_t addr)
{
	uint8_t tdi[4], tdo[4];

	// a) Load EF_WDATA register with 0 (Clear Test Read Data)
	tdi[0] = 0x0; tdi[1] = 0x0; tdi[2] = 0x0; tdi[3] = 0x0;
	jtag_dev_write_ir(vorago_dev_index, EF_WDATA);
	jtag_dev_shift_dr(vorago_dev_index, tdi, NULL, 32);

	// b) Load EF_ADDR register with the desired read address (Load Address)
	tdi[0] = addr;
	jtag_dev_write_ir(vorago_dev_index, EF_ADDR);
	jtag_dev_shift_dr(vorago_dev_index, tdi, NULL, 5);

	// c) Load EF_CMD register with 0x1 (Interface Enable)
	tdi[0] = 0x1;
	jtag_dev_write_ir(vorago_dev_index, EF_CMD);
	jtag_dev_shift_dr(vorago_dev_index, tdi, NULL, 3);

	// d) Poll/Read EF_STATUS for Bit 0 being 1 (Oscillator is running)
	int i = 0;
	for (; i < 5; i++) { // Max 5 tries
		tdi[0] = 0x1;
		jtag_dev_write_ir(vorago_dev_index, EF_STATUS);
		jtag_dev_shift_dr(vorago_dev_index, tdi, tdo, 2);
		if (tdo[0] & 0x1)
			break;
	}
	if (i == 5) {
		DEBUG_ERROR("eFurse read timeout! Oscillator is not running!\n");
		return 0;
	}

	// e) Load EF_CMD register with 0x5 (Issue Read Command)
	tdi[0] = 0x5;
	jtag_dev_write_ir(vorago_dev_index, EF_CMD);
	jtag_dev_shift_dr(vorago_dev_index, tdi, NULL, 3);

	// f) Read EF_RDATA register (Result of EFuse read)
	jtag_dev_write_ir(vorago_dev_index, EF_RDATA);
	jtag_dev_shift_dr(vorago_dev_index, tdi, tdo, 32);
	uint32_t val = (tdo[0] << 24) | (tdo[1] << 16) | (tdo[2] << 8) | tdo[3];

	// g) Load EF_CMD register with 0x00 (Interface Disable)
	tdi[0] = 0x0; tdi[1] = 0x0; tdi[2] = 0x0; tdi[3] = 0x0;
	jtag_dev_write_ir(vorago_dev_index, EF_CMD);
	jtag_dev_shift_dr(vorago_dev_index, tdi, NULL, 32);

	return val;
}

/* Program eFuse register */
void program_efuse_register(uint8_t addr, uint32_t value) {
	
	uint8_t tdi[4], tdo[4];

	// a) Load EF_ADDR register with the desired write address (Load Address)
	tdi[0] = addr;
	jtag_dev_write_ir(vorago_dev_index, EF_ADDR);
	jtag_dev_shift_dr(vorago_dev_index, tdi, NULL, 5);

	// b) Load EF_WDATA register with data (Load Data)
	tdi[0] = 0xFF & (value >> 24);
	tdi[1] = 0xFF & (value >> 16);
	tdi[2] = 0xFF & (value >> 8);
	tdi[3] = 0xFF & (value >> 0);
	jtag_dev_write_ir(vorago_dev_index, EF_WDATA);
	jtag_dev_shift_dr(vorago_dev_index, tdi, NULL, 32);

	// c) Load EF_TIMING register with proper timing data
	tdi[0] = 0x84;
	tdi[1] = 0x00;
	tdi[2] = 0x22;
	tdi[3] = 0x00;
	jtag_dev_write_ir(vorago_dev_index, EF_TIMING);
	jtag_dev_shift_dr(vorago_dev_index, tdi, NULL, 25);

	// d) Load EF_CMD register with 0x3 (Interface Enable with Write Mode)
	tdi[0] = 0x03;
	jtag_dev_write_ir(vorago_dev_index, EF_CMD);
	jtag_dev_shift_dr(vorago_dev_index, tdi, NULL, 3);

	// e) Poll/Read EF_STATUS for Bit 0 being 1 (Oscillator is running)
	int i = 0;
	for (; i < 5; i++) { // Max 5 tries
		tdi[0] = 0x01;
		jtag_dev_write_ir(vorago_dev_index, EF_STATUS);
		jtag_dev_shift_dr(vorago_dev_index, tdi, tdo, 2);
		if (tdo[0] & 0x1)
			break;
	}
	if (i == 5) {
		DEBUG_ERROR("eFuse writing timeout! Oscillator is not running!\n");
		return;
	}

	// f) Bring EFUSE_WRITE_ENn pin low (Enable Write)

	// g) Load EF_CMD register with 0x7 (Issue Write Command)
	jtag_dev_write_ir(vorago_dev_index, EF_CMD);
	tdi[0] = 0x7;
	jtag_dev_shift_dr(vorago_dev_index, tdi, NULL, 3); // Start the write operation
	tdi[0] = 0x3;
	jtag_dev_shift_dr(vorago_dev_index, tdi, NULL, 3); // Clear the start operation bit

	// h) Poll/Read EF_STATUS register Bit 1 being 0 (Check/wait for write complete)
	for (i = 0; i < 5; i++) { // Max 5 tries
		tdi[0] = 0x01;
		jtag_dev_write_ir(vorago_dev_index, EF_STATUS);
		jtag_dev_shift_dr(vorago_dev_index, tdi, tdo, 2);
		if (tdo[0] & 0x2)
			break;
	}
	if (i == 5) {
		DEBUG_ERROR("eFuse writing timeout! Write not completed\n");
		return;
	}

	// i) Bring EFUSE_WRITE_ENn pin high (Disable Write)

	// j) Load EF_CMD register with 0x00 (Interface Disable)
	tdi[0] = 0x0;
	tdi[1] = 0x0;
	tdi[2] = 0x0;
	tdi[3] = 0x0;
	jtag_dev_write_ir(vorago_dev_index, EF_CMD);
	jtag_dev_shift_dr(vorago_dev_index, tdi, tdo, 32);

	// k) Do eFuse Read procedure to verify result
	uint32_t readback = read_efuse_register(addr);
	if (value != readback)
		DEBUG_ERROR("eFuse verifying failed! wrote = 0x%08x, read = 0x%08x\n", value, readback);
}

/* Set write latch */
static void spi_set_write_latch()
{
	// Write SPI config
	uint8_t spi_config[3] = {0x01, 0, 8};
	jtag_dev_write_ir(vorago_dev_index, EF_SPI_CONFIG);
	jtag_dev_shift_dr(vorago_dev_index, spi_config, NULL, 24);

	// SPI transfer
	uint8_t spi_write[2] = { reverse[SPI_MEMORY_WREN], 0x00 };
	jtag_dev_write_ir(vorago_dev_index, EF_SPI_ENCAP);
	jtag_dev_shift_dr(vorago_dev_index, spi_write, NULL, 8 + 3);
}

#if 0
/* Reset write latch */
static void spi_reset_write_latch()
{
	// Write SPI config
	uint8_t spi_config[3] = {0x01, 0, 8};
	jtag_dev_write_ir(vorago_dev_index, EF_SPI_CONFIG);
	jtag_dev_shift_dr(vorago_dev_index, spi_config, NULL, 24);

	// SPI transfer
	uint8_t spi_write[2] = { reverse[SPI_MEMORY_WRDI], 0x00};
	jtag_dev_write_ir(vorago_dev_index, EF_SPI_ENCAP);
	jtag_dev_shift_dr(vorago_dev_index, spi_write, NULL, 8 + 3);
}

/* Enable write protection */
static void spi_enable_write_protection()
{
	// Write SPI config
	uint8_t spi_config[3] = {0x01, 0, 16};
	jtag_dev_write_ir(vorago_dev_index, EF_SPI_CONFIG);
	jtag_dev_shift_dr(vorago_dev_index, spi_config, NULL, 24);

	// SPI transfer
	uint8_t spi_write[3] = {reverse[SPI_MEMORY_WRSR], reverse[0x8C], 0x00};
	jtag_dev_write_ir(vorago_dev_index, EF_SPI_ENCAP);
	jtag_dev_shift_dr(vorago_dev_index, spi_write, NULL, 16 + 3);
}

/* Disable write protection */
static void spi_disable_write_protection()
{
	// Write SPI config
	uint8_t spi_config[3] = {0x01, 0, 16};
	jtag_dev_write_ir(vorago_dev_index, EF_SPI_CONFIG);
	jtag_dev_shift_dr(vorago_dev_index, spi_config, NULL, 24);

	// SPI transfer
	uint8_t spi_write[3] = {reverse[SPI_MEMORY_WRSR], reverse[0x02], 0x00};
	jtag_dev_write_ir(vorago_dev_index, EF_SPI_ENCAP);
	jtag_dev_shift_dr(vorago_dev_index, spi_write, NULL, 16 + 3);
}

/* Read SPI memory status register*/
static uint8_t spi_status()
{
	// Write SPI config
	uint8_t spi_config[3] = {0x01, 0, 16};
	jtag_dev_write_ir(vorago_dev_index, EF_SPI_CONFIG);
	jtag_dev_shift_dr(vorago_dev_index, spi_config, NULL, 24);

	// SPI transfer
	uint8_t spi_write[3] = {reverse[SPI_MEMORY_RDSR], reverse[0x02], 0x00};
	uint8_t spi_read[3];
	jtag_dev_write_ir(vorago_dev_index, EF_SPI_ENCAP);
	jtag_dev_shift_dr(vorago_dev_index, spi_write, spi_read, 16 + 3);

	return (reverse[spi_read[1]] << 3) | (reverse[spi_read[2]] >> 5);
}
#endif

/* Read SPI memory */
static void spi_read(uint32_t addr, uint8_t *data, unsigned int data_len)
{
	// Write SPI config
	unsigned int transfer_len = 32 + 8 * data_len;
	uint8_t spi_config[3] = {0x01, transfer_len >> 8, transfer_len};
	jtag_dev_write_ir(vorago_dev_index, EF_SPI_CONFIG);
	jtag_dev_shift_dr(vorago_dev_index, spi_config, NULL, 24);

	// Transfer buffers
	uint8_t spi_write[4 + data_len + 1];
	uint8_t spi_read[4 + data_len + 1];
	spi_write[0] = reverse[SPI_MEMORY_READ];
	spi_write[1] = reverse[0xFF & (addr >> 16)];
	spi_write[2] = reverse[0xFF & (addr >> 8)];
	spi_write[3] = reverse[0xFF & (addr >> 0)];
	memset(&spi_write[4], 0xFF, data_len + 1);

	// Perform the JTAG transfer
	jtag_dev_write_ir(vorago_dev_index, EF_SPI_ENCAP);
	jtag_dev_shift_dr(vorago_dev_index, spi_write, spi_read, 32 + 8 * data_len + 3);

	// Scramble the bits to correct order
	uint8_t *spi_ptr = &spi_read[4];
	for (unsigned int i = 0; i < data_len; i++) {
		*spi_ptr = reverse[*spi_ptr];
		spi_ptr++;
	}
	spi_ptr = &spi_read[4];
	for (unsigned int i = 0; i < data_len; i++) {
		data[i] = (spi_ptr[0] << 3) | (spi_ptr[1] >> 5);
		spi_ptr++;
	}

}

/* Write SPI memory */
static void spi_write(uint32_t addr, const uint8_t *data, unsigned int data_len)
{
	// Write SPI config
	unsigned int transfer_len = 32 + 8 * data_len;
	uint8_t spi_config[3] = {0x01, transfer_len >> 8, transfer_len};
	jtag_dev_write_ir(vorago_dev_index, EF_SPI_CONFIG);
	jtag_dev_shift_dr(vorago_dev_index, spi_config, NULL, 24);

	// Transfer buffer
	uint8_t spi_write[4 + data_len];
	spi_write[0] = reverse[SPI_MEMORY_WRITE];
	spi_write[1] = reverse[0xFF & (addr >> 16)];
	spi_write[2] = reverse[0xFF & (addr >> 8)];
	spi_write[3] = reverse[0xFF & (addr >> 0)];

	// Scramble the bits to correct order
	for (unsigned int i = 0; i < data_len; i++)
		spi_write[4+i] = reverse[data[i]];

	// Perform the JTAG transfer
	jtag_dev_write_ir(vorago_dev_index, EF_SPI_ENCAP);
	jtag_dev_shift_dr(vorago_dev_index, spi_write, NULL, 32 + (8 * data_len));
}


static bool va08xx_attach(target_s *t)
{
	if (!cortexm_attach(t))
		return false;

	uint32_t val = 0x1;
	cortexm_mem_write_sized(t, 0x4000100C, &val,  sizeof(uint32_t), ALIGN_WORD); // ROM_PROT = 0x1

	return true;
}


static void va108xx_mem_write(target_s *t, target_addr_t dest, const void *src, size_t len)
{
	if (dest < 128 * 1024U) {
		if (fram_burn && vorago_dev_index != 255) {
			spi_set_write_latch();
			spi_write(dest, src, len);
		}

		uint32_t val = 0x1;
		cortexm_mem_write_sized(t, 0x4000100C, &val, sizeof(uint32_t), ALIGN_WORD); // ROM_PROT = 0x1

		cortexm_mem_write_sized(t, dest, src, len, ALIGN_BYTE);

		val = 0x0;
		cortexm_mem_write_sized(t, 0x4000100C, &val, sizeof(uint32_t), ALIGN_WORD); // ROM_PROT = 0x0
	}
	cortexm_mem_write_sized(t, dest, src, len, ALIGN_BYTE);
}


static bool va416xx_attach(target_s *t)
{
	if (!cortexm_attach(t))
		return false;
	
	// Disable watchdog and unlock code RAM for write
	uint32_t val = 0x1ACCE551;
	cortexm_mem_write_sized(t, 0x400210C0, &val, sizeof(uint32_t), ALIGN_WORD); // WDOGLOCK = 0x1ACCE551
	val = 0x0;
	cortexm_mem_write_sized(t, 0x40021008, &val, sizeof(uint32_t), ALIGN_WORD); // WDOGCONTROL = 0x0 (disabled)
	val = 0x1;
	cortexm_mem_write_sized(t, 0x40010010, &val, sizeof(uint32_t), ALIGN_WORD); // ROM_PROT = 0x1

	return true;
}

static void va416xx_mem_write(target_s *t, target_addr_t dest, const void *src, size_t len)
{
	if (dest < 128 * 1024U) {
		if (fram_burn && vorago_dev_index != 255) {
			spi_set_write_latch();
			spi_write(dest, src, len);
		}

		uint32_t val = 0x1;
		cortexm_mem_write_sized(t, 0x40010010, &val, sizeof(uint32_t), ALIGN_WORD); // ROM_PROT = 0x1

		cortexm_mem_write_sized(t, dest, src, len, ALIGN_BYTE);

		val = 0x0;
		cortexm_mem_write_sized(t, 0x40010010, &val, sizeof(uint32_t), ALIGN_WORD); // ROM_PROT = 0x0
	}
	cortexm_mem_write_sized(t, dest, src, len, ALIGN_BYTE);
}



void vorago_jtag_dp_handler(const uint8_t dev_index)
{
	DEBUG_INFO("Found Vorago Controller %d\n", dev_index);
	vorago_dev_index = dev_index;
}


bool vorago_probe(target_s *target)
{
	const uint32_t proc_id = target_mem_read32(target, 0x40000FF8);
	DEBUG_INFO("vorago_mcu_probe cpuid=%08lx proc_id=%08lx\n", target->cpuid, proc_id);

	switch (proc_id) {
	case 0x040017E3: /* Vorago VA108xx */
		target->attach = va08xx_attach; // cortexm_attach;
		target->mem_write = va108xx_mem_write;
		break;

	case 0x040057e3: /* Vorago VA416xx */
		target->attach = va416xx_attach;
		target->mem_write = va416xx_mem_write;
		break;
	default:
		return false;
	}

	target->detach = cortexm_detach;
	target->driver = "Vorago";
	target->part_id = proc_id;

	target_add_ram(target, 0x00000000, 128 * 1024U); // Program SRAM
	target_add_ram(target, 0x1FFF8000, 32 * 1024U);  // Data SRAM (DataMem0)
	target_add_ram(target, 0x20000000, 32 * 1024U);  // Data SRAM (DataMem1)
	target_add_commands(target, vorago_cmd_list, target->driver);

	return true;
}


static bool vorago_cmd_efuse(target_s *target, int argc, const char **argv)
{
	if (vorago_dev_index == 255) {
		tc_printf(target, "No Vorago JTAG Controller found!\n");
		return false;
	}
	
	if (argc == 2 && strcmp(argv[1], "dump") == 0)
	{
		// argv[0] = "efuse";
		// argv[1] = "dump";
		tc_printf(target, "eFuse register dump:\n");

		// Read eFuse_ID index register
		uint32_t eFuse_ID_idx = read_efuse_register(0);
		uint32_t eFuse_ID_addr = 31 - __builtin_clz(eFuse_ID_idx | 1) + 2; // CLZ = Count leading zeros
		tc_printf(target, "  eFuse_ID index: 0x%08X (%d)\n", eFuse_ID_idx, eFuse_ID_addr);

		// Read eFuse_Config index register
		uint32_t eFuse_Config_idx = read_efuse_register(1);
		uint32_t eFuse_Config_addr = 31 - __builtin_clz(eFuse_Config_idx | 1) + 2;
		tc_printf(target, "  eFuse_Config index: 0x%08X (%d)\n", eFuse_Config_idx, eFuse_Config_addr);

		// Read rest of registers
		for (uint32_t addr = 2; addr < 32; addr++) {
			const char* note = "";
			if (addr == eFuse_ID_addr)
				note = "<<< eFuse_ID";
			else if (addr == eFuse_Config_addr)
				note = "<<< eFuse_Config";
			tc_printf(target, "  Register %d: 0x%08x %s", addr, read_efuse_register(addr), note);
		}

		return true;
	}
	else if (argc == 3 && strcmp(argv[1], "read") == 0)
	{
		// argv[0] = "efuse";
		// argv[1] = "read";
		// argv[2] = "4";
		const uint32_t addr = strtoul(argv[2], NULL, 0);
		if (addr > 32) {
			tc_printf(target, "Invalid address\n");
			return false;
		}
		tc_printf(target, "Register %d: 0x%08x", addr, read_efuse_register(addr));
		return true;
	}
	else if (argc == 4 && strcmp(argv[1], "burn") == 0)
	{
		// argv[0] = "efuse";
		// argv[1] = "read";
		// argv[2] = "4";
		// argv[3] = "0xdeadbeef";
		const uint32_t addr = strtoul(argv[2], NULL, 0);
		const uint32_t val = strtoul(argv[3], NULL, 0);
		if (addr > 32) {
			tc_printf(target, "Invalid address\n");
			return false;
		}
		program_efuse_register(addr, val);
		return true;
	}
	else {
		tc_printf(target,
			"usage:\n"
			"  monitor efuse dump\n"
			"  monitor efuse read <addr>\n"
			"  monitor efuse burn <addr> <value>\n");
	}

	return true;
}

static bool vorago_cmd_fram(target_s *target, int argc, const char **argv)
{
	if (vorago_dev_index == 255) {
		tc_printf(target, "No Vorago JTAG Controller found!\n");
		return false;
	}
	
	if (argc == 4 && strcmp(argv[1], "read") == 0)
	{
		// argv[0] = "fram";
		// argv[1] = "read";
		// argv[2] = "0x4000";
		// argv[3] = "32";
		uint32_t addr = strtoul(argv[2], NULL, 0);
		uint32_t len = strtoul(argv[3], NULL, 0);
		uint8_t data[16];
		char hex[32 + 1];

		while (len > 0) {
			uint32_t rlen = (len < 16 ? len : 16);
			spi_read(addr, data, rlen);

			hexify(hex, data, rlen);
			tc_printf(target, "%s\n", hex);

			addr += rlen;
			len -= rlen;
		}
		return true;
	}
	else if (argc >= 4 && strcmp(argv[1], "write") == 0)
	{
		// argv[0] = "efuse";
		// argv[1] = "write";
		// argv[2] = "0x4000";
		// argv[3] = "deadbeef";
		uint32_t addr = strtoul(argv[2], NULL, 0);
		const char* hexstr = argv[3];
		uint32_t len = strlen(hexstr) / 2;
		uint8_t data[16];

		while (1) {
			int rlen = (len < 16) ? len : 16;
			unhexify(data, hexstr, rlen);

			spi_set_write_latch();
			spi_write(addr, data, rlen);

			addr += rlen;
			len -= rlen;
		}
		return true;
	}
	else if (argc == 2 && strcmp(argv[1], "enable_burn") == 0)
	{
		fram_burn = 1;
	}
	else if (argc == 2 && strcmp(argv[1], "disable_burn") == 0)
	{
		fram_burn = 0;
	}
	else {
		tc_printf(target,
			"usage:\n"
			"  monitor fram read <addr> <len>\n"
			"  monitor fram write <addr> <hexadecimal data>\n"
			"  monitor fram enable_burn\n"
			"  monitor fram disable_burn\n");
	}
	return false;
}
