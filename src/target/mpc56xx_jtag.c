/***************************************************************************
 *   Copyright (C) 2017 by James Murray <james@nscc.info                   *
 *   Based on code:                                                        *
 *       Copyright (C) 2010 by Oleksandr Tymoshenko <gonzo@bluezbox.com>   *
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

#include "target.h"
#include "jtag/jtag.h"
#include "mpc5xxx_jtag.h"
#include "mpc5xxx.h"
#include "mpc56xx_jtag.h"
#include "jtag/interface.h"

#define READ_MAXLOOPS 500

/* eTPU, eDMA, FLEXRAY would be similar */


int mpc56xx_enter_debug(struct mpc5xxx_jtag *jtag_info, int async_flag)
{
	int res;
	printf("got call to enter_debug\n");

	res = mpc56xx_enable_once(jtag_info);
	if (res)
		return res;

	if (async_flag) {
		/* WKUP, DR */
		res = mpc5xxx_once_write(jtag_info, MPC5XXX_ONCE_OCR, MPC5XXX_OCR_DEBUG1, 32);
		if (res)
			return res;
	}

	/* was WKUP, FDB only */
	res = mpc5xxx_once_write(jtag_info, MPC5XXX_ONCE_OCR, MPC5XXX_OCR_DEBUG2 | 1, 32);
	if (res)
		return res;
	res = mpc5xxx_once_write(jtag_info, MPC5XXX_ONCE_DBCR0, MPC5XXX_ONCE_DBCR0_EDM, 32);
	if (res)
		return res;

	/* res = mpc56xx_once_write(jtag_info, MPC56XX_ONCE_DBSR, 0xffffffff, 32);*/
	/* if (res)*/
	/*	return res;*/

	/* Take control of all debug resources */
	res = mpc5xxx_once_write(jtag_info, MPC56XX_ONCE_DBERC0, 0, 32);
	if (res)
		return res;


	return ERROR_OK;
}

/* Sends the enable_once command to the OnCE controller.
 * Freescale datasheets very unclear on whether this is required or how to
 * use it.
 */
int mpc56xx_enable_once(struct mpc5xxx_jtag *jtag_info)
{
	int res;
	uint32_t val;

	res = mpc5xxx_jtag_access_once(jtag_info);
	if (res)
		return res;

	res = mpc5xxx_jtag_set_instr(jtag_info, MPC56XX_ONCE_ENABLE);
	if (res)
		return res;

	/* Then scan through DR stage like exec */

	/* dummy read of one bit to force update-DR */
	res = mpc5xxx_jtag_read_data(jtag_info, &val, 1);
	if (res)
		return res;

	return ERROR_OK;
}

/* In debug mode only, loads an instruction to CPUSCR and executes
 * it. Optionally using alternate data if flag set.
 */
int mpc56xx_exec_inst_nowait(struct mpc5xxx_jtag *jtag_info, uint32_t inst, uint32_t in,
		uint32_t *out, uint32_t flag)
{
	int retval;

	uint32_t osr;
	struct mpc5xxx_cpuscr scr;
	uint32_t val;

	retval = mpc5xxx_once_cpuscr_read(jtag_info, &scr);
	if (retval != ERROR_OK)
		return retval;

	/*
	 * write the appropriate information into the CPU scan chain
	 * register (CPUSCR), followed by writing to OCMD to set the OCMD[GO] bit and clear the OCMD[EX]
	 * bit with the OCMD[RS] field indicating either the CPUSCR or No Register Selected.
	 */
	scr.wbbrl = in;
	scr.wbbrh = 0; /* only for 64 bit */
	/* leave scr.msr unchanged */
	scr.ir = inst;

	/*
	 * FILTHY HACK:
	 * If we a are single stepping as part of normal execution flow, we need to ensure the
	 * top sixteen bits of ctl are the same as when we entered debug mode.
	 * If this is the case, the upper sixteen bits of flag should contain this ...
	 */

	scr.ctl = flag & 0xFFFF0000 ;


	if ((flag & MPC56XX_EI_VAL))
		scr.ctl |= MPC5XXX_CPUSCR_CTL_FFRA;

	/* May point to invalid instruction? */
	if ((flag & MPC56XX_EI_INC) == 0)
		scr.pc -= 4; /* post-decrement by one instruction */

	retval = mpc5xxx_once_cpuscr_write(jtag_info, &scr);
	if (retval != ERROR_OK)
		return retval;

	retval = mpc5xxx_jtag_set_instr(jtag_info, MPC5XXX_ONCE_GO | MPC56XX_ONCE_NOREG); /* was MPC56XX_ONCE_CPUSCR */
	if (retval)
		return retval;

	/* Freescale e200z3 RM has this CRITICAL tidbit:
	 * "In addition, the Update-DR state must also be transitioned through
	 * in order for the single-step and/or exit functionality to be performed"
	 */
	retval = mpc5xxx_jtag_read_data(jtag_info, &val, 1); /* dummy read of one bit to force update-DR */
	if (retval)
		return retval;

	/* After single-step, "the external tool
	 * should read the OnCE Status Register (OSR) to verify that the CPU has returned to debug mode with no
	 * error by verifying that the OSR[DEBUG] bit is set and OSR[ERR] bit is cleared."
	 */

	retval = mpc5xxx_once_osr_read(jtag_info, &osr);
	if (retval != ERROR_OK)
		return retval;

	if (((osr & MPC5XXX_OSR_DEBUG) == 0) || (osr & MPC5XXX_OSR_ERR)) {
		printf("2. OSR indicates failure of some kind. OSR = 0x%08x\n", osr);
		printf("The inst =0x%08x, PC=0x%08x\n", inst, scr.pc);
		if ((osr & MPC5XXX_OSR_ERR))
			return ERROR_WAIT;
		else
			return ERROR_FAIL;
	}

	retval = mpc5xxx_once_cpuscr_read(jtag_info, &scr);
	if (retval != ERROR_OK)
		return retval;

	*out = scr.wbbrl;

	return ERROR_OK;
}

int mpc56xx_exec_inst(struct mpc5xxx_jtag *jtag_info, uint32_t inst, uint32_t in,
		uint32_t *out, uint32_t flag){
	int retval ;
	int retries = 100 ;

	do {
		retval = mpc56xx_exec_inst_nowait(jtag_info, inst, in, out, flag);
	} while (retval == ERROR_WAIT && --retries) ;

	if (retval) {
		if (!retries)
			printf("Failed to execute single step.\n");
		return retval;
	}
	return retval ;
}


/* In debug mode only, exits debug mode.
 * Single-steps over NOP and sets EX bit to exit debug.
 * ctl_on_entry unused here
 */
int mpc56xx_exit_debug(struct mpc5xxx_jtag *jtag_info, uint32_t addr, int sw_bp, uint32_t ctl_on_entry)
{
	int retval;

	uint32_t osr;
	struct mpc5xxx_cpuscr scr;
	uint32_t val;

	if (sw_bp)
		retval = mpc5xxx_once_write(jtag_info, MPC5XXX_ONCE_OCR, MPC5XXX_OCR_DEBUG2, 32); /* leave SW BPs on */
	else
		retval = mpc5xxx_once_write(jtag_info, MPC5XXX_ONCE_OCR, MPC5XXX_OCR_DEBUG_OFF, 32);
	if (retval != ERROR_OK)
		return retval;

	/* debug */
		/* see what's in IACs */
		retval = mpc5xxx_once_read(jtag_info, MPC56XX_ONCE_IAC1, &val, 32);
		if (retval != ERROR_OK)
			return retval;
		printf("IAC1=0x%08x, ", val);
		retval = mpc5xxx_once_read(jtag_info, MPC56XX_ONCE_IAC2, &val, 32);
		if (retval != ERROR_OK)
			return retval;
		printf("IAC2=0x%08x, ", val);
		retval = mpc5xxx_once_read(jtag_info, MPC56XX_ONCE_IAC3, &val, 32);
		if (retval != ERROR_OK)
			return retval;
		printf("IAC3=0x%08x, ", val);
		retval = mpc5xxx_once_read(jtag_info, MPC56XX_ONCE_IAC4, &val, 32);
		if (retval != ERROR_OK)
			return retval;
		printf("IAC4=0x%08x\n", val);
		/* end IACs */

		retval = mpc5xxx_once_read(jtag_info, MPC5XXX_ONCE_OCR, &val, 32);
		if (retval != ERROR_OK)
			return retval;
		printf("OCR=0x%08x\n", val);
	/* end debug */

	retval = mpc5xxx_once_cpuscr_read(jtag_info, &scr);
	if (retval != ERROR_OK)
		return retval;

	scr.wbbrl = 0;
	scr.wbbrh = 0;
	scr.ir = MPC56XX_NOP ;
	scr.pc = addr - 4 ;

	retval = mpc5xxx_once_cpuscr_write(jtag_info, &scr);
	if (retval != ERROR_OK)
		return retval;

	printf("Continuing @ 0x%08x...\n", addr);
	retval = mpc5xxx_jtag_set_instr(jtag_info, MPC5XXX_ONCE_GO | MPC5XXX_ONCE_EX | MPC56XX_ONCE_NOREG);
	if (retval)
		return retval;

	/* Freescale e200z3 RM has this CRITICAL tidbit:
	 * "In addition, the Update-DR state must also be transitioned through
	 * in order for the single-step and/or exit functionality to be performed"
	 */
	retval = mpc5xxx_jtag_read_data(jtag_info, &val, 1); /* dummy read of one bit to force update-DR */
	if (retval)
		return retval;

	if (sw_bp)
		retval = mpc5xxx_once_write(jtag_info, MPC5XXX_ONCE_OCR, MPC5XXX_OCR_FDB, 32); /* leave SW BPs on */
	else
		retval = mpc5xxx_once_write(jtag_info, MPC5XXX_ONCE_OCR, MPC5XXX_OCR_DEBUG_OFF, 32);
	if (retval != ERROR_OK)
		return retval;

	retval = mpc5xxx_once_osr_read(jtag_info, &osr);
	if (retval != ERROR_OK)
		return retval;

	/* Check no error */
	if (osr & MPC5XXX_OSR_ERR) {
		printf("1. OSR indicates failure of some kind. OSR = 0x%08x\n", osr);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static uint32_t bp_reg[MPC56XX_NUM_BPS] = {MPC56XX_ONCE_IAC1, MPC56XX_ONCE_IAC2, MPC56XX_ONCE_IAC3, MPC56XX_ONCE_IAC4};
static uint32_t bp_mask[MPC56XX_NUM_BPS] = {MPC56XX_DB_IAC1, MPC56XX_DB_IAC2, MPC56XX_DB_IAC3, MPC56XX_DB_IAC4};

int mpc56xx_jtag_set_bp(struct mpc5xxx_jtag *jtag_info, int bp_num, uint32_t bp_addr)
{
	int retval;
	uint32_t dbcr0;

	if ((bp_num < 0) || (bp_num >= MPC56XX_NUM_BPS))
		return ERROR_FAIL;
	retval = mpc5xxx_once_read(jtag_info, MPC5XXX_ONCE_DBCR0, &dbcr0, 32);
	if (retval)
		return retval;

	/* Write address to IAC register */
	retval = mpc5xxx_once_write(jtag_info, bp_reg[bp_num], bp_addr, 32);
	if (retval)
		return retval;

	/* clear status register for that channel (write a 1 to that bit) */
	retval = mpc5xxx_once_write(jtag_info, MPC56XX_ONCE_DBSR, bp_mask[bp_num], 32);
	if (retval)
		return retval;

	/* Enable that breakpoint channel */
	dbcr0 |= MPC5XXX_ONCE_DBCR0_EDM | bp_mask[bp_num];

	retval = mpc5xxx_once_write(jtag_info, MPC5XXX_ONCE_DBCR0, dbcr0, 32);
	if (retval)
		return retval;
	/* Set other three DBCRx to all zeros */
	retval = mpc5xxx_once_write(jtag_info, MPC56XX_ONCE_DBCR1, 0, 32);
	if (retval)
		return retval;
	retval = mpc5xxx_once_write(jtag_info, MPC56XX_ONCE_DBCR2, 0, 32);
	if (retval)
		return retval;
	retval = mpc5xxx_once_write(jtag_info, MPC56XX_ONCE_DBCR3, 0, 32);
	if (retval)
		return retval;

	return ERROR_OK;
}

/* Clear breakpoint and watchpoint enable bits.
 * Use this before enabling to ensure only active BP/WP get used.
 */
int mpc56xx_jtag_clr_bps_wps(struct mpc5xxx_jtag *jtag_info)
{
	int retval;
	uint32_t dbcr0, mask;

	mask = MPC56XX_DB_IAC1 | MPC56XX_DB_IAC2 | MPC56XX_DB_IAC3 | MPC56XX_DB_IAC4
			| MPC56XX_DB_DAC1R | MPC56XX_DB_DAC1W | MPC56XX_DB_DAC2R | MPC56XX_DB_DAC2W;
	/* Disable that breakpoint channel */
	retval = mpc5xxx_once_read(jtag_info, MPC5XXX_ONCE_DBCR0, &dbcr0, 32);
	if (retval)
		return retval;
	dbcr0 &= ~mask;
	retval = mpc5xxx_once_write(jtag_info, MPC5XXX_ONCE_DBCR0, dbcr0, 32);
	if (retval)
		return retval;

	/* Clear all of those bits */
	retval = mpc5xxx_once_write(jtag_info, MPC56XX_ONCE_DBSR, mask, 32);
	if (retval)
		return retval;

	return ERROR_OK;
}

int mpc56xx_jtag_clr_bp(struct mpc5xxx_jtag *jtag_info, int bp_num, uint32_t addr)
{
	int retval;
	uint32_t dbcr0;

	if ((bp_num < 0) || (bp_num >= MPC56XX_NUM_BPS))
		return ERROR_FAIL;

	/* Disable that breakpoint channel */
	retval = mpc5xxx_once_read(jtag_info, MPC5XXX_ONCE_DBCR0, &dbcr0, 32);
	if (retval)
		return retval;
	dbcr0 &= ~bp_mask[bp_num];
	retval = mpc5xxx_once_write(jtag_info, MPC5XXX_ONCE_DBCR0, dbcr0, 32);
	if (retval)
		return retval;

	/* Write a 1 to that bit to clear it */
	retval = mpc5xxx_once_write(jtag_info, MPC56XX_ONCE_DBSR, bp_mask[bp_num], 32);
	if (retval)
		return retval;

	return ERROR_OK;
}

static uint32_t wp_reg[MPC56XX_NUM_BPS] = {MPC56XX_ONCE_DAC1, MPC56XX_ONCE_DAC2};
static uint32_t wp_mask[MPC56XX_NUM_BPS] = {MPC56XX_DB_DAC1R | MPC56XX_DB_DAC1W, MPC56XX_DB_DAC2R | MPC56XX_DB_DAC2W};

int mpc56xx_jtag_set_wp(struct mpc5xxx_jtag *jtag_info, int wp_num, uint32_t wp_addr)
{
	int retval;
	uint32_t dbcr0;

	if ((wp_num < 0) || (wp_num >= MPC56XX_NUM_WPS))
		return ERROR_FAIL;
	retval = mpc5xxx_once_read(jtag_info, MPC5XXX_ONCE_DBCR0, &dbcr0, 32);
	if (retval)
		return retval;

	/* Write address to IAC register */
	retval = mpc5xxx_once_write(jtag_info, wp_reg[wp_num], wp_addr, 32);
	if (retval)
		return retval;

	/* clear status register for that channel (write a 1 to that bit) */
	retval = mpc5xxx_once_write(jtag_info, MPC56XX_ONCE_DBSR, wp_mask[wp_num], 32);
	if (retval)
		return retval;

	/* Enable that breakpoint channel */
	dbcr0 |= MPC5XXX_ONCE_DBCR0_EDM | wp_mask[wp_num];

	retval = mpc5xxx_once_write(jtag_info, MPC5XXX_ONCE_DBCR0, dbcr0, 32);
	if (retval)
		return retval;
	/* Set other three DBCRx to all zeros */
	retval = mpc5xxx_once_write(jtag_info, MPC56XX_ONCE_DBCR1, 0, 32);
	if (retval)
		return retval;
	retval = mpc5xxx_once_write(jtag_info, MPC56XX_ONCE_DBCR2, 0, 32);
	if (retval)
		return retval;
	retval = mpc5xxx_once_write(jtag_info, MPC56XX_ONCE_DBCR3, 0, 32);
	if (retval)
		return retval;

	return ERROR_OK;
}

int mpc56xx_jtag_clr_wp(struct mpc5xxx_jtag *jtag_info, int wp_num, uint32_t addr)
{
	int retval;
	uint32_t dbsr0;

	if ((wp_num < 0) || (wp_num >= MPC56XX_NUM_WPS))
		return ERROR_FAIL;

	/* Disable that watchpoint channel */
	retval = mpc5xxx_once_read(jtag_info, MPC5XXX_ONCE_DBCR0, &dbsr0, 32);
	if (retval)
		return retval;
	dbsr0 &= ~wp_mask[wp_num];
	retval = mpc5xxx_once_write(jtag_info, MPC5XXX_ONCE_DBCR0, dbsr0, 32);
	if (retval)
		return retval;

	/* Write a 1 to that bit to clear it */
	retval = mpc5xxx_once_write(jtag_info, MPC56XX_ONCE_DBSR, wp_mask[wp_num], 32);
	if (retval)
		return retval;

	return ERROR_OK;
}

/* The following are not currently used */
#if 0
int mpc56xx_jtag_access_nexus(struct mpc5xxx_jtag *jtag_info)
{
	int res;

	if (jtag_info->current_tap == MPC56XX_TAP_NEXUS)
			return ERROR_OK;

	printf("Switching to NEXUS\n");
	if (jtag_info->current_tap != MPC56XX_TAP_JTAG) {
		res = mpc56xx_jtag_access_jtagc(jtag_info); /* Revert to JTAGC first */
		if (res)
			return res;
	}

	res = mpc5xxx_jtag_set_instr(jtag_info, MPC56XX_TAP_NEXUS);
	if (res)
		return res;

	jtag_info->current_tap = MPC56XX_TAP_NEXUS;
	jtag_info->tap->ir_length = 4; /* Nexus uses 4 bits */

	return ERROR_OK;
}

int mpc56xx_nexus_read(struct mpc5xxx_jtag *jtag_info,
		uint32_t addr, uint32_t *value, uint32_t size)
{
	int res;

	res = mpc56xx_jtag_access_nexus(jtag_info);
	if (res)
		return res;

	res = mpc5xxx_jtag_set_instr(jtag_info, addr);
	if (res)
		return res;

	res = mpc5xxx_jtag_read_data(jtag_info, value, size);
	if (res)
		return res;

	return ERROR_OK;
}

int mpc56xx_nexus_write(struct mpc5xxx_jtag *jtag_info,
		uint32_t addr, uint32_t value, uint32_t size)
{
	int res;

	res = mpc56xx_jtag_access_nexus(jtag_info);
	if (res)
		return res;

	res = mpc5xxx_jtag_set_instr(jtag_info, addr);
	if (res)
		return res;

	res = mpc5xxx_jtag_write_data(jtag_info, value, size);
	if (res)
		return res;

	return ERROR_OK;
}

#endif