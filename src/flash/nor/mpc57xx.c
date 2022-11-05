/***************************************************************************
 *   Copyright (C) 2022 by James Turton                                    *
 *   james.turton@gmx.com                                                  *
 *                                                                         *
 *   Copyright (C) 2015 by James Murray                                    *
 * 	 james@nscc.info                                                       *
 *                                                                         *
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by John McCarthy                                   *
 *   jgmcc@magma.ca                                                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/jtag.h>
#include "imp.h"
#include <target/algorithm.h>
#include <target/mpc57xx_jtag.h>
#include <target/mpc57xx.h>

#define MPC57XX_MANUF_ID	0x029 /* will be wrong */

struct mpc57xx_flash_bank { /* all unknown at the moment */
	uint32_t disable_bit;
	uint32_t busy_bits;
	uint32_t *lock_sel_reg;
	uint32_t *lock_sel_bit;
	int probed;
};

struct mpc57xx_mem_layout {
	uint32_t block_start;
	uint32_t block_size;
	uint32_t rww_part;
	uint32_t lock_sel_reg;
	uint32_t lock_sel_bit;
};

static const struct mpc57xx_mem_layout mem_layout_array[] = {
	{0x00000000, 0x04000, 0x02, 0x00, 0x00000001},
	{0x00004000, 0x04000, 0x02, 0x00, 0x00000002},
	{0x00008000, 0x04000, 0x02, 0x00, 0x00000004},
	{0x0000C000, 0x04000, 0x02, 0x00, 0x00000008},
	{0x00010000, 0x04000, 0x03, 0x00, 0x00000010},
	{0x00014000, 0x04000, 0x03, 0x00, 0x00000020},
	{0x00018000, 0x04000, 0x03, 0x00, 0x00000040},
	{0x0001C000, 0x04000, 0x03, 0x00, 0x00000080},
	{0x00020000, 0x08000, 0x02, 0x00, 0x00000100},
	{0x00028000, 0x08000, 0x03, 0x00, 0x00000200},
	{0x00030000, 0x08000, 0x00, 0x00, 0x00040000},
	{0x00038000, 0x08000, 0x00, 0x00, 0x00080000},
	{0x00040000, 0x08000, 0x01, 0x00, 0x00100000},
	{0x00048000, 0x08000, 0x01, 0x00, 0x00200000},
	{0x00050000, 0x10000, 0x00, 0x00, 0x00400000},
	{0x00060000, 0x10000, 0x01, 0x00, 0x01000000},
	{0x00070000, 0x40000, 0x06, 0x02, 0x00000001},
	{0x000B0000, 0x40000, 0x06, 0x02, 0x00000002},
	{0x000F0000, 0x40000, 0x06, 0x02, 0x00000004},
	{0x00130000, 0x40000, 0x06, 0x02, 0x00000008},
	{0x00170000, 0x40000, 0x06, 0x02, 0x00000010},
	{0x001B0000, 0x40000, 0x06, 0x02, 0x00000020},
	{0x001F0000, 0x40000, 0x06, 0x02, 0x00000040},
	{0x00230000, 0x40000, 0x06, 0x02, 0x00000080},
	{0x00270000, 0x40000, 0x07, 0x02, 0x00000100},
	{0x002B0000, 0x40000, 0x07, 0x02, 0x00000200},
	{0x002F0000, 0x40000, 0x07, 0x02, 0x00000400},
	{0x00330000, 0x40000, 0x07, 0x02, 0x00000800},
	{0x00370000, 0x40000, 0x07, 0x02, 0x00001000},
	{0x003B0000, 0x40000, 0x07, 0x02, 0x00002000},
	{0x003F0000, 0x40000, 0x07, 0x02, 0x00004000},
	{0x00430000, 0x40000, 0x07, 0x02, 0x00008000},
	{0x00470000, 0x40000, 0x08, 0x02, 0x00010000},
	{0x004B0000, 0x40000, 0x08, 0x02, 0x00020000},
	{0x004F0000, 0x40000, 0x08, 0x02, 0x00040000},
	{0x00530000, 0x40000, 0x09, 0x02, 0x00080000},
	{0x00570000, 0x40000, 0x09, 0x02, 0x00100000},
	{0x005B0000, 0x40000, 0x09, 0x02, 0x00200000}
};

static const uint32_t mpc57xx_flash_lock_regs[4] = {
	0xFFFE0010,
	0xFFFE0014,
	0xFFFE0018,
	0xFFFE001c
};

static const uint32_t mpc57xx_flash_sel_regs[4] = {
	0xFFFE0038,
	0xFFFE003C,
	0xFFFE0040,
	0xFFFE0044
};

#define MPC57XX_REG_MCR			0xFFFE0000

#define MPC57XX_REG_MCR_RVE		(1<<(31-0))
#define MPC57XX_REG_MCR_RRE		(1<<(31-1))
#define MPC57XX_REG_MCR_AEE		(1<<(31-2))
#define MPC57XX_REG_MCR_EEE		(1<<(31-3))
#define MPC57XX_REG_MCR_EER		(1<<(31-16))
#define MPC57XX_REG_MCR_RWE		(1<<(31-17))
#define MPC57XX_REG_MCR_SBC		(1<<(31-18))
#define MPC57XX_REG_MCR_PEAS	(1<<(31-20))
#define MPC57XX_REG_MCR_DONE	(1<<(31-21))
#define MPC57XX_REG_MCR_PEG		(1<<(31-22))
#define MPC57XX_REG_MCR_PECIE	(1<<(31-23))
#define MPC57XX_REG_MCR_PGM		(1<<(31-27))
#define MPC57XX_REG_MCR_PSUS	(1<<(31-28))
#define MPC57XX_REG_MCR_ERS		(1<<(31-29))
#define MPC57XX_REG_MCR_ESUS	(1<<(31-30))
#define MPC57XX_REG_MCR_EHV		(1<<(31-31))

/*
 * DEVID values
 */

static const struct mpc57xx_devs_s {
	uint32_t devid;
	const char *name;
} mpc57xx_devs[] = {
	{0x1834601d, "MPC5748G JTAG"}, /* Developed against this chip only. */
	{0x00000000, NULL}
};

static int mpc57xx_build_block_list(struct flash_bank *bank)
{
	struct mpc57xx_flash_bank *mpc57xx_info = bank->driver_priv;

	unsigned int i;
	unsigned int num_blocks = 38;

	bank->num_sectors = num_blocks;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_blocks);
	mpc57xx_info->lock_sel_reg = malloc(sizeof(uint32_t) * num_blocks);
	mpc57xx_info->lock_sel_bit = malloc(sizeof(uint32_t) * num_blocks);

	for (i = 0; i < num_blocks; i++) {
		bank->sectors[i].offset = mem_layout_array[i].block_start;
		bank->sectors[i].size = mem_layout_array[i].block_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1; /* protection bits likely set by hardware */
		mpc57xx_info->lock_sel_reg[i] = mem_layout_array[i].lock_sel_reg;
		mpc57xx_info->lock_sel_bit[i] = mem_layout_array[i].lock_sel_bit;
	}

	return ERROR_OK;
}

/* flash bank mpc57xx 0 0 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(mpc57xx_flash_bank_command)
{
	struct mpc57xx_flash_bank *mpc57xx_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	mpc57xx_info = malloc(sizeof(struct mpc57xx_flash_bank));
	bank->driver_priv = mpc57xx_info;

	mpc57xx_info->probed = 1; /* ensure probe does not occur */

	mpc57xx_build_block_list(bank);

	return ERROR_OK;
}

static int mpc57xx_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct mpc57xx_flash_bank *mpc57xx_info = bank->driver_priv;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint32_t val[4];
	for (unsigned int i = 0; i < 4; i++) {
		retval = target_read_u32(target, mpc57xx_flash_lock_regs[i], val + i);
		if (retval != ERROR_OK)
			return retval;
	}

	for (unsigned int s = 0; s < bank->num_sectors; s++) {
		if (mpc57xx_info->lock_sel_bit[s] & val[mpc57xx_info->lock_sel_reg[s]])
			bank->sectors[s].is_protected = 1;
		else
			bank->sectors[s].is_protected = 0;
	}

	return ERROR_OK;
}

static int mpc57xx_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;
	struct mpc57xx_flash_bank *mpc57xx_info = bank->driver_priv;
	int retval;
	uint32_t val[4];
	unsigned int i;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (set)
		printf("Locking blocks: %d - %d\n", first, last);
	else
		printf("Unlocking blocks: %d - %d\n", first, last);

	// Read LOCK regs first
	for (i = 0; i < 4; i++) {
		retval = target_read_u32(target, mpc57xx_flash_lock_regs[i], val + i);
		if (retval != ERROR_OK)
			return retval;
	}

	// Set appropriate bits to lock/unlock
	for (i = first; i <= last; i++) {
		if (set) {
			val[mpc57xx_info->lock_sel_reg[i]] |= mpc57xx_info->lock_sel_bit[i];
			bank->sectors[i].is_protected = 1;
		}
		else {
			val[mpc57xx_info->lock_sel_reg[i]] &= ~mpc57xx_info->lock_sel_bit[i];
			bank->sectors[i].is_protected = 0;
		}
	}

	// Write back LOCK regs
	for (i = 0; i < 4; i++) {
		retval = target_write_u32(target, mpc57xx_flash_lock_regs[i], val[i]);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int mpc57xx_flash_set_sel(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;
	struct mpc57xx_flash_bank *mpc57xx_info = bank->driver_priv;
	int retval;
	uint32_t val[4];
	unsigned int i;

	// Set all SEL registers to zero
	for (i = 0; i < 4; i++)
		val[i] = 0;

	// Set appropriate bits to one
	for (i = first; i <= last; i++)
		val[mpc57xx_info->lock_sel_reg[i]] |= mpc57xx_info->lock_sel_bit[i];

	// Write SEL regs
	for (i = 0; i < 4; i++) {
		retval = target_write_u32(target, mpc57xx_flash_sel_regs[i], val[i]);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int mpc57xx_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;
	int retval, retry;
	uint32_t val;

	printf("Got a call to erase!\n");

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = mpc57xx_protect(bank, 0, first, last);
	if (retval != ERROR_OK)
		return retval;

	printf("Erasing blocks: %d - %d\n", first, last);

	retval = target_write_u32(target, MPC57XX_REG_MCR, 0); /* Clear MCR */
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, MPC57XX_REG_MCR, &val);
	if (retval != ERROR_OK)
		return retval;
	printf("MPC57XX_REG_MCR = 0x%08x\n", val);

	retval = target_write_u32(target, MPC57XX_REG_MCR, MPC57XX_REG_MCR_ERS); /* select ERS operation */
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, MPC57XX_REG_MCR, &val);
	if (retval != ERROR_OK)
		return retval;
	printf("MPC57XX_REG_MCR = 0x%08x\n", val);

	retval = mpc57xx_flash_set_sel(bank, first, last); /* Set SEL register */
	if (retval != ERROR_OK)
		return retval;

	/* write something to the first address in that flash bank */
	printf("writing to 0x%08x\n", (uint32_t)(bank->base + bank->sectors[first].offset));
	retval = target_write_u32(target, bank->base + bank->sectors[first].offset, 0xffffffff);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, MPC57XX_REG_MCR, MPC57XX_REG_MCR_ERS | MPC57XX_REG_MCR_EHV); /* select ERS+EHV operation */
	if (retval != ERROR_OK)
		return retval;

	/* Wait for Done=1 */
	retry = 10000; /* arbitrary limit, can take some time */
	val = 0;
	while (retry && ((val & MPC57XX_REG_MCR_DONE) == 0)) {
		retval = target_read_u32(target, MPC57XX_REG_MCR, &val);
		if (retval != ERROR_OK)
			return retval;
		retry--;
	}
	if (retry == 0) {
		LOG_ERROR("Timeout waiting for Done");
		return ERROR_TIMEOUT_REACHED;
	}

	retval = target_write_u32(target, MPC57XX_REG_MCR, MPC57XX_REG_MCR_ERS); /* clear EHV */
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, MPC57XX_REG_MCR, 0); /* clear ERS */
	if (retval != ERROR_OK)
		return retval;

	if ((val & MPC57XX_REG_MCR_PEG) == 0) { /* Confirm PEG set */
		LOG_ERROR("Received error on flash erase");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/* Determines sector details give a flash address */
static int mpc57xx_get_sect(struct flash_bank *bank, uint32_t address, uint32_t *start, uint32_t *end)
{
	unsigned int i;
	int block_num = -1;
	uint32_t sec_start = 0;
	uint32_t sec_end = 0;

	/* Validate address */
	for (i = 0; i < bank->num_sectors; i++) {
		sec_start = bank->base + bank->sectors[i].offset;
		sec_end = sec_start + bank->sectors[i].size;

		/* check if destination falls within the current sector */
		if ((address >= sec_start) && (address < sec_end)) {
			block_num = i;
			break;
		}
	}

	*start = sec_start;
	*end = sec_end - 1;

	return block_num;
}

static int mpc57xx_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t words_remaining = (count / sizeof(uint32_t));
	uint32_t bytes_written = 0;
	uint32_t bytes_remaining = (count & 0x00000007);
	uint32_t address = bank->base + offset;
	int retval, retry;
	uint32_t val;
	uint32_t cur_sec_start, cur_sec_end;
	int block_num, tail_end = 0;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("writing to flash at address " TARGET_ADDR_FMT " at offset 0x%8.8" PRIx32
			" count: 0x%8.8" PRIx32 "", bank->base, offset, count);

	if (offset & 0x7) {
		LOG_WARNING("offset 0x%" PRIx32 "breaks required 8-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
		/* TODO: Could workaround this, but not presently. */
	}

	printf("Entered write_block code!\n");

	block_num = -1;
	cur_sec_start = 1; /* intentionally invalid */
	cur_sec_end = 0;

	while (words_remaining) {

		if ((address < cur_sec_start) || (address > cur_sec_end)) {
			printf("Address = 0x%08x\n", address);

			block_num = mpc57xx_get_sect(bank, address, &cur_sec_start, &cur_sec_end);

			if (block_num < 0)
				return ERROR_FLASH_DST_OUT_OF_BANK;

			retval = target_write_u32(target, MPC57XX_REG_MCR, 0); /* Clear MCR */
			if (retval != ERROR_OK)
				return retval;

			retval = mpc57xx_protect(bank, 0, block_num, block_num);
			if (retval != ERROR_OK)
				return retval;

			retval = target_write_u32(target, MPC57XX_REG_MCR, MPC57XX_REG_MCR_PGM); /* Select PGM */
			if (retval != ERROR_OK)
				return retval;
		}

		/* Skip writes if the data is the same as erased flash value */
		if ((target_buffer_get_u32(target, buffer + bytes_written) != 0xffffffff) &&
			(target_buffer_get_u32(target, buffer + bytes_written + sizeof(uint32_t)) != 0xffffffff)) {
			retval = target_write_memory(target, address, sizeof(uint32_t), 1, buffer + bytes_written); /* Write first 32bits */
			if (retval != ERROR_OK)
				return retval;

			if (tail_end) {
				LOG_INFO("Tail end data flash, not sure this is correct. Please check.");
				printf("Handling the stragglers %d\n", bytes_remaining);
				words_remaining = 2;
				bytes_remaining = 0;
			}
			else {
				retval = target_write_memory(target, address + sizeof(uint32_t), sizeof(uint32_t), 1, buffer + bytes_written + sizeof(uint32_t)); /* Write second 32bits */
				if (retval != ERROR_OK)
					return retval;
			}

			retval = target_write_u32(target, MPC57XX_REG_MCR, MPC57XX_REG_MCR_PGM | MPC57XX_REG_MCR_EHV); /* Set EHV */
			if (retval != ERROR_OK)
				return retval;

			/* Wait for Done=1 */
			retry = 10000; /* arbitrary limit */
			val = 0;
			while (retry && ((val & MPC57XX_REG_MCR_DONE) == 0)) {
				retval = target_read_u32(target, MPC57XX_REG_MCR, &val);
				if (retval != ERROR_OK)
					return retval;
				retry--;
			}
			if (retry == 0) {
				LOG_ERROR("Timeout waiting for Done");
				return ERROR_TIMEOUT_REACHED;
			}

			/* Clear EHV for each 64bit programmed */
			retval = target_write_u32(target, MPC57XX_REG_MCR, MPC57XX_REG_MCR_PGM);
			if (retval != ERROR_OK)
				return retval;

			if ((val & MPC57XX_REG_MCR_PEG) == 0) { /* Confirm PEG set */
				LOG_ERROR("Received error on flash erase");
				return ERROR_FAIL;
			}
		}

		if ((words_remaining <= 2) && (bytes_remaining))
			tail_end = 1;

		bytes_written += 8;
		words_remaining -= 2;
		address += 8;
	}

	/* Ensure MCR for existing sector is cleared */
	retval = target_write_u32(target, MPC57XX_REG_MCR, 0);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int mpc57xx_probe(struct flash_bank *bank)
{
	struct mpc57xx_flash_bank *mpc57xx_info = bank->driver_priv;

	if (mpc57xx_info->probed == 0) {
		printf("Probe bypassed.\n");
		LOG_DEBUG("Probe bypassed");
	}

	return ERROR_OK;
}

static int mpc57xx_auto_probe(struct flash_bank *bank)
{
	struct mpc57xx_flash_bank *mpc57xx_info = bank->driver_priv;
	if (mpc57xx_info->probed)
		return ERROR_OK;
	return mpc57xx_probe(bank);
}

static int mpc57xx_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	uint32_t device_id;
	int i;

	/*device_id = jtag_info->idcode;*/
	/* TODO fix this */
	device_id = 0x87654321;

	if (((device_id >> 1) & 0x7ff) != MPC57XX_MANUF_ID) {
		command_print_sameline(cmd,
				 "Cannot identify target as a MPC57XX family (manufacturer 0x%03d != 0x%03d)\n",
				 (unsigned)((device_id >> 1) & 0x7ff),
				 MPC57XX_MANUF_ID);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	for (i = 0; mpc57xx_devs[i].name != NULL; i++) {
		if (mpc57xx_devs[i].devid == (device_id & 0x0fffffff)) {
			command_print_sameline(cmd, "MPC57XX%s", mpc57xx_devs[i].name);
			break;
		}
	}

	if (mpc57xx_devs[i].name == NULL)
		command_print_sameline(cmd, "Unknown");

	command_print_sameline(cmd, " Ver: 0x%02x",
			(unsigned)((device_id >> 28) & 0xf));

	return ERROR_OK;
}

COMMAND_HANDLER(mpc57xx_handle_unlock_command)
{
	if (CMD_ARGC < 1) {
		command_print(CMD, "mpc57xx unlock <bank>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	// uint32_t addr;
	// COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], addr);
	// uint32_t foo;
	// uint32_t block = mpc57xx_get_sect(bank, addr, &foo, &foo);
	// printf("Address: 0x%08x in block: 0x%08x\n", addr, block);
	// mpc57xx_erase(bank, block, block);

	// uint32_t block;
	// COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], block);
	// for (block = 0; block < 38; block++)
	// mpc57xx_unlock_block(bank, block);

	uint8_t buf[] = {0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8};
	mpc57xx_write(bank, buf, 0x005B0000, 8);

	printf("Not handled!\n");
	return ERROR_FAIL;
}

static const struct command_registration mpc57xx_exec_command_handlers[] = {
	{
		.name = "unlock",
		.handler = mpc57xx_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "[bank_id]",
		.help = "Unlock/Erase entire device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration mpc57xx_command_handlers[] = {
	{
		.name = "mpc57xx",
		.mode = COMMAND_ANY,
		.help = "mpc57xx flash command group",
		.usage = "",
		.chain = mpc57xx_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver mpc57xx_flash = {
	.name = "mpc57xx",
	.commands = mpc57xx_command_handlers,
	.flash_bank_command = mpc57xx_flash_bank_command,
	.erase = mpc57xx_erase,
	.protect = mpc57xx_protect,
	.write = mpc57xx_write,
	.read = default_flash_read,
	.probe = mpc57xx_probe,
	.auto_probe = mpc57xx_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = mpc57xx_protect_check,
	.info = mpc57xx_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
