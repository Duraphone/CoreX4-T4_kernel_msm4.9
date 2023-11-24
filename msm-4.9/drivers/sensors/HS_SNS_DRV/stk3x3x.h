/*
*
* $Id: stk3x3x.h
*
* Copyright (C) 2012~2015 Lex Hsieh     <lex_hsieh@sensortek.com.tw>
*
* This file is subject to the terms and conditions of the GNU General Public
* License.  See the file COPYING in the main directory of this archive for
* more details.
*
*/
#ifndef __STK3X3X_H__
#define __STK3X3X_H__

/* platform data */
struct stk3x3x_platform_data
{
	uint8_t state_reg;
	uint8_t psctrl_reg;
	uint8_t alsctrl_reg;
	uint8_t ledctrl_reg;
	uint8_t	wait_reg;
	uint16_t ps_thd_h;
	uint16_t ps_thd_l;
	uint16_t ps_crosstalk_max;
	bool ps_choose_poll;  // 1 for poll while 0  for int
	int int_pin;
	uint32_t transmittance;
	int als_root;
	int als_numerator;
	uint32_t int_flags;
	int	use_fir;
};


#endif // __STK3X1X_H__
