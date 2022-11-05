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

#ifndef MPC57XX
#define MPC57XX

struct target;

#define MPC57XX_COMMON_MAGIC	0x1834601d /* Set as MPC5746R JTAG ID - is this acceptable? */

#define MPC57XX_START_OF_SRAM	0x40000000 /* This needs to come from a config var */
#define MPC57XX_SIZE_OF_SRAM	0x000C0000 /* 768k This needs to come from a config var */

#define MPC57XX_FLASH_PFCR1	0xfc030000
#define MPC57XX_FLASH_PFCR2	0xfc030004

#define MPC57XX_PLLDIG_PLLCAL3	0xFFFB0098
#define MPC57XX_PLLDIG_PLLCR	0xFFFB00A0
#define MPC57XX_PLLDIG_PLLSR	0xFFFB00A4
#define MPC57XX_PLLDIG_PLLDV	0xFFFB00A8
#define MPC57XX_PLLDIG_PLLFM	0xFFFB00AC
#define MPC57XX_PLLDIG_PLLFD	0xFFFB00B0
#define MPC57XX_PLLDIG_PLLCAL1	0xFFFB00B8

int mpc57xx_save_context(struct target *target);
int mpc57xx_restore_context(struct target *target);

#endif	/*MPC5634*/
