/***************************************************************************
 *   Copyright (C) 2017 by James Murray <james@nscc.info                   *
 *   Based on code:                                                        *
 *       Copyright (C) 2010 by Oleksandr Tymoshenko <gonzo@bluezbox.com>   *
 *   Based on mips_m4k code:                                               *
 *       Copyright (C) 2008 by Spencer Oliver <spen@spen-soft.co.uk>       *
 *       Copyright (C) 2008 by David T.L. Wong                             *
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

#include "jtag/jtag.h"
#include "register.h"
#include "algorithm.h"
#include "target.h"
#include "breakpoints.h"
#include "target_type.h"
#include "mpc5xxx_jtag.h"
#include "mpc5xxx.h"
#include "jtag/interface.h"

const char * const mpc5xxx_core_reg_list[] = {
	"r0",  "r1",  "r2",  "r3",  "r4",  "r5",  "r6",  "r7",
	"r8",  "r9",  "r10", "r11", "r12", "r13", "r14", "r15",
	"r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23",
	"r24", "r25", "r26", "r27", "r28", "r29", "r30", "r31",
	"r32", "r33", "r34", "r35", "r36", "r37", "r38", "r39",
	"r40", "r41", "r42", "r43", "r44", "r45", "r46", "r47",
	"r48", "r49", "r50", "r51", "r52", "r53", "r54", "r55",
	"r56", "r57", "r58", "r59", "r60", "r61", "r62", "r63",
	"pc",  "msr", "cnd", "lr",  "cnt", "xer", "mq",  "r71",
};

const struct mpc5xxx_core_reg
	mpc5xxx_core_reg_list_arch_info[MPC5XXX_NUMCOREREGS] = {
	{0, NULL, NULL},
	{1, NULL, NULL},
	{2, NULL, NULL},
	{3, NULL, NULL},
	{4, NULL, NULL},
	{5, NULL, NULL},
	{6, NULL, NULL},
	{7, NULL, NULL},
	{8, NULL, NULL},
	{9, NULL, NULL},
	{10, NULL, NULL},
	{11, NULL, NULL},
	{12, NULL, NULL},
	{13, NULL, NULL},
	{14, NULL, NULL},
	{15, NULL, NULL},
	{16, NULL, NULL},
	{17, NULL, NULL},
	{18, NULL, NULL},
	{19, NULL, NULL},
	{20, NULL, NULL},
	{21, NULL, NULL},
	{22, NULL, NULL},
	{23, NULL, NULL},
	{24, NULL, NULL},
	{25, NULL, NULL},
	{26, NULL, NULL},
	{27, NULL, NULL},
	{28, NULL, NULL},
	{29, NULL, NULL},
	{30, NULL, NULL},
	{31, NULL, NULL},
	{32, NULL, NULL},
	{33, NULL, NULL},
	{34, NULL, NULL},
	{35, NULL, NULL},
	{36, NULL, NULL},
	{37, NULL, NULL},
	{38, NULL, NULL},
	{39, NULL, NULL},
	{40, NULL, NULL},
	{41, NULL, NULL},
	{42, NULL, NULL},
	{43, NULL, NULL},
	{44, NULL, NULL},
	{45, NULL, NULL},
	{46, NULL, NULL},
	{47, NULL, NULL},
	{48, NULL, NULL},
	{49, NULL, NULL},
	{50, NULL, NULL},
	{51, NULL, NULL},
	{52, NULL, NULL},
	{53, NULL, NULL},
	{54, NULL, NULL},
	{55, NULL, NULL},
	{56, NULL, NULL},
	{57, NULL, NULL},
	{58, NULL, NULL},
	{59, NULL, NULL},
	{60, NULL, NULL},
	{61, NULL, NULL},
	{62, NULL, NULL},
	{63, NULL, NULL},
	{64, NULL, NULL},
	{65, NULL, NULL},
	{66, NULL, NULL},
	{67, NULL, NULL},
	{68, NULL, NULL},
	{69, NULL, NULL},
	{70, NULL, NULL},
	{71, NULL, NULL},
};

int mpc5xxx_read_core_reg(struct target *target, int num)
{
	uint32_t reg_value;

	/* get pointers to arch-specific information */
	struct mpc5xxx_common *mpc5xxx = target_to_mpc5xxx(target);

	if ((num < 0) || (num >= MPC5XXX_NUMCOREREGS))
		return ERROR_COMMAND_SYNTAX_ERROR;

	reg_value = mpc5xxx->core_regs[num];
	buf_set_u32(mpc5xxx->core_cache->reg_list[num].value, 0, 32, reg_value);
	mpc5xxx->core_cache->reg_list[num].valid = 1;
	mpc5xxx->core_cache->reg_list[num].dirty = 0;

	return ERROR_OK;
}

int mpc5xxx_write_core_reg(struct target *target, int num)
{
	uint32_t reg_value;

	/* get pointers to arch-specific information */
	struct mpc5xxx_common *mpc5xxx = target_to_mpc5xxx(target);

	if ((num < 0) || (num >= MPC5XXX_NUMCOREREGS))
		return ERROR_COMMAND_SYNTAX_ERROR;

	reg_value = buf_get_u32(mpc5xxx->core_cache->reg_list[num].value, 0, 32);
	mpc5xxx->core_regs[num] = reg_value;
	LOG_DEBUG("write core reg %i value 0x%" PRIx32 "", num, reg_value);
	mpc5xxx->core_cache->reg_list[num].valid = 1;
	mpc5xxx->core_cache->reg_list[num].dirty = 0;

	return ERROR_OK;
}

int mpc5xxx_get_core_reg(struct reg *reg)
{
	int retval;
	struct mpc5xxx_core_reg *mpc5xxx_reg = reg->arch_info;
	struct target *target = mpc5xxx_reg->target;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	retval = mpc5xxx_read_core_reg(target, mpc5xxx_reg->num);

	return retval;
}

int mpc5xxx_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct mpc5xxx_core_reg *mpc5xxx_reg = reg->arch_info;
	struct target *target = mpc5xxx_reg->target;
	uint32_t value = buf_get_u32(buf, 0, 32);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	buf_set_u32(reg->value, 0, 32, value);
	reg->dirty = 1;
	reg->valid = 1;

	return ERROR_OK;
}

const struct reg_arch_type mpc5xxx_reg_type = {
	.get = mpc5xxx_get_core_reg,
	.set = mpc5xxx_set_core_reg,
};

struct reg_cache *mpc5xxx_build_reg_cache(struct target *target)
{
	int num_regs = MPC5XXX_NUMCOREREGS;
	struct mpc5xxx_common *mpc5xxx = target_to_mpc5xxx(target);
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(num_regs, sizeof(struct reg));
	struct mpc5xxx_core_reg *arch_info =
		malloc(sizeof(struct mpc5xxx_core_reg) * num_regs);
	int i;

	/* Build the process context cache */
	cache->name = "mpc5xxx registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = num_regs;
	(*cache_p) = cache;
	mpc5xxx->core_cache = cache;

	for (i = 0; i < num_regs; i++) {
		arch_info[i] = mpc5xxx_core_reg_list_arch_info[i];
		arch_info[i].target = target;
		arch_info[i].mpc5xxx_common = mpc5xxx;
		reg_list[i].name = mpc5xxx_core_reg_list[i];
		reg_list[i].size = 32;
		reg_list[i].value = calloc(1, sizeof(uint32_t));
		reg_list[i].dirty = false;
		reg_list[i].valid = false;
		reg_list[i].exist = true;
		reg_list[i].type = &mpc5xxx_reg_type;
		reg_list[i].arch_info = &arch_info[i];
	}

	return cache;
}

int mpc5xxx_assert_reset(struct target *target)
{
	struct mpc5xxx_common *mpc5xxx = target_to_mpc5xxx(target);
	int retval;

	printf("mpc5xxx_assert_reset\n");

	adapter_assert_reset();

	/* Init code wants to be talking to JTAGC. */
	retval = mpc5xxx_jtag_access_jtagc(&mpc5xxx->jtag);
	if (retval)
		return retval;

	return ERROR_OK;
}

int mpc5xxx_read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct mpc5xxx_common *mpc5xxx = target_to_mpc5xxx(target);

	LOG_DEBUG("address: 0x%8.8" PRIx32 ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32 "",
		(unsigned int)address,
		size,
		count);

	/* sanitize arguments */
	if (((size != sizeof(uint32_t)) && (size != sizeof(uint16_t)) && (size != sizeof(uint8_t))) || (count == 0) || !(buffer))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (((size == sizeof(uint32_t)) && (address & 0x3u)) || ((size == sizeof(uint16_t)) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	switch (size) {
		case sizeof(uint32_t):
			return mpc5xxx_jtag_read_memory32(&mpc5xxx->jtag, address, count, (uint32_t *)(void *)buffer);
			break;
		case sizeof(uint16_t):
			return mpc5xxx_jtag_read_memory16(&mpc5xxx->jtag, address, count, (uint16_t *)(void *)buffer);
			break;
		case sizeof(uint8_t):
			return mpc5xxx_jtag_read_memory8(&mpc5xxx->jtag, address, count, buffer);
			break;
		default:
			break;
	}

	return ERROR_OK;
}

int mpc5xxx_write_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct mpc5xxx_common *mpc5xxx = target_to_mpc5xxx(target);

	LOG_DEBUG("address: 0x%8.8" PRIx32 ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32 "",
		(unsigned int)address,
		size,
		count);

	/* sanitize arguments */
	if (((size != sizeof(uint32_t)) && (size != sizeof(uint16_t)) && (size != sizeof(uint8_t))) || (count == 0) || !(buffer))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (((size == sizeof(uint32_t)) && (address & 0x3u)) || ((size == sizeof(uint16_t)) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	switch (size) {
		case sizeof(uint32_t):
			return mpc5xxx_jtag_write_memory32(&mpc5xxx->jtag, address, count, (uint32_t *)(void *)buffer);
			break;
		case sizeof(uint16_t):
			return mpc5xxx_jtag_write_memory16(&mpc5xxx->jtag, address, count, (uint16_t *)(void *)buffer);
			break;
		case sizeof(uint8_t):
			return mpc5xxx_jtag_write_memory8(&mpc5xxx->jtag, address, count, buffer);
			break;
		default:
			break;
	}

	return ERROR_OK;
}

int mpc5xxx_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	struct mpc5xxx_common *mpc5xxx = target_to_mpc5xxx(target);
	printf("init_target\n");
	mpc5xxx->jtag.tap = target->tap; /* why? */
	mpc5xxx_build_reg_cache(target);
	/* mpc5xxx->num_inst_bpoints_avail = 2; */
	return ERROR_OK;
}

int mpc5xxx_examine(struct target *target)
{
	uint32_t osr;
	struct mpc5xxx_common *mpc5xxx = target_to_mpc5xxx(target);
	printf("examine\n");
	if (!target_was_examined(target)) {

		target_set_examined(target);

		/* we will configure later */
		mpc5xxx->bp_scanned = 0;
		mpc5xxx->num_inst_bpoints = 0;
		mpc5xxx->num_data_bpoints = 0;
		mpc5xxx->num_inst_bpoints_avail = 0;
		mpc5xxx->num_data_bpoints_avail = 0;

		/* Check if processor halted. */
		mpc5xxx_once_osr_read(&mpc5xxx->jtag, &osr);
		if (osr & MPC5XXX_OSR_HALT_DEBUG) { /* FIXME say halted if in DEBUG mode */
			LOG_INFO("target is halted/debug");
			target->state = TARGET_HALTED;
		} else
			target->state = TARGET_RUNNING;
	}

	printf("examine done!\n");

	return ERROR_OK;
}

int mpc5xxx_arch_state(struct target *target)
{
	struct mpc5xxx_common *mpc5xxx = target_to_mpc5xxx(target);

	LOG_USER("target halted due to %s, pc: 0x%8.8" PRIx32 "",
		debug_reason_name(target), mpc5xxx->jtag.dpc);

	return ERROR_OK;
}

/* This next function derived from mips32.c */
int mpc5xxx_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
		int *reg_list_size, enum target_register_class reg_class)
{
	/* get pointers to arch-specific information */
	struct mpc5xxx_common *mpc5xxx = target_to_mpc5xxx(target);
	unsigned int i;

	/* include floating point registers */
	*reg_list_size = MPC5XXX_NUMCOREREGS;
	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

	for (i = 0; i < MPC5XXX_NUMCOREREGS; i++)
		(*reg_list)[i] = &mpc5xxx->core_cache->reg_list[i];

	return ERROR_OK;
}

int mpc5xxx_jtag_read_memory32(struct mpc5xxx_jtag *jtag_info,
	uint32_t addr, int count, uint32_t *buffer)
{
	int i, retval;

	for (i = 0; i < count; i++) {
		retval = mpc5xxx_once_nexus_read(jtag_info, addr + (i * sizeof(uint32_t)), buffer + i, 32);

		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

int mpc5xxx_jtag_read_memory16(struct mpc5xxx_jtag *jtag_info,
	uint32_t addr, int count, uint16_t *buffer)
{
	int i = 0;
	int retval;
	uint32_t data;

	while (i < count) {
		retval = mpc5xxx_once_nexus_read(jtag_info, addr + (i * sizeof(uint16_t)), &data, 32);

		if (retval != ERROR_OK)
			return retval;

		/* any unaligned halfwords? */
		if (i == 0 && addr & 3) {
			buffer[i++] = (data >> 16) & 0xffff;
		}
		/* last halfword */
		else if (count - i == 1) {
			buffer[i++] = data & 0xffff;
		}
		/* read all complete words */
		else {
			buffer[i++] = data & 0xffff;
			buffer[i++] = (data >> 16) & 0xffff;
		}
	}

	return ERROR_OK;
}

int mpc5xxx_jtag_read_memory8(struct mpc5xxx_jtag *jtag_info,
	uint32_t addr, int count, uint8_t *buffer)
{
	int i = 0;
	int retval;
	uint8_t data[4];

	while (i < count) {
		retval = mpc5xxx_once_nexus_read(jtag_info, addr + (i * sizeof(uint8_t)), (uint32_t *)(void *)data, 32);

		if (retval != ERROR_OK)
			return retval;

		/* Do we have non-aligned bytes at start? */
		if (i == 0 && addr & 3) {
			int len = MIN(4 - (addr & 3), count);
			memcpy(buffer, data + (addr & 3), len);
			i += len;
		}
		/* remaining bytes */
		else if ((count - i) < 4) {
			memcpy(buffer + i, data, count - i);
			i += count - i;
		}
		/* read all complete words */
		else {
			memcpy(buffer + i, data, 4);
			i += 4;
		}
	}

	return ERROR_OK;
}

int mpc5xxx_jtag_write_memory32(struct mpc5xxx_jtag *jtag_info,
	uint32_t addr, int count, const uint32_t *buffer)
{
	int i, retval;

	for (i = 0; i < count; i++) {
		retval = mpc5xxx_once_nexus_write(jtag_info, addr + (i * sizeof(uint32_t)), buffer[i], 32);

		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

int mpc5xxx_jtag_write_memory16(struct mpc5xxx_jtag *jtag_info,
	uint32_t addr, int count, const uint16_t *buffer)
{
	int i = 0;
	int retval;
	uint32_t data;

	while (i < count) {
		retval = mpc5xxx_once_nexus_read(jtag_info, addr + (i * sizeof(uint16_t)), &data, 32);

		if (retval != ERROR_OK)
			return retval;

		/* Do we have non-aligned bytes at start? */
		if (i == 0 && addr & 3) {
			data &= 0xffff;
			data |= buffer[i++] << 16;
		}
		/* last halfword */
		else if (count - i == 1) {
			data &= ~0xffff;
			data |= buffer[i++];
		}
		/* write all complete words */
		else {
			data = buffer[i++];
			data |= buffer[i++] << 16;
		}

		retval = mpc5xxx_once_nexus_write(jtag_info, addr + (i * sizeof(uint16_t)), data, 32);

		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

int mpc5xxx_jtag_write_memory8(struct mpc5xxx_jtag *jtag_info,
	uint32_t addr, int count, const uint8_t *buffer)
{
	int i = 0;
	int j;
	int retval;
	uint32_t data;

	while (i < count) {
		retval = mpc5xxx_once_nexus_read(jtag_info, addr + (i * sizeof(uint8_t)), &data, 32);

		if (retval != ERROR_OK)
			return retval;

		/* Do we have non-aligned bytes at start? */
		if (i == 0 && addr & 3) {
			for (j = addr & 3; (j < 4) && (i < count); j++, i++) {
				data &= ~(0xff << j*8);
				data |= (buffer[i] << j*8);
			}
		}
		/* Write trailing bytes */
		else if ((count - i) < 4) {
			for (j = 0; i < count; j++, i++) {
				data &= ~(0xff << j*8);
				data |= (buffer[i] << j*8);
			}
		}
		/* Write all complete words */
		else {
			data = 0;
			for (j = 0; j < 4; j++, i++)
				data |= (buffer[i] << j*8);
		}

		retval = mpc5xxx_once_nexus_write(jtag_info, addr + (i * sizeof(uint8_t)), data, 32);

		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}
