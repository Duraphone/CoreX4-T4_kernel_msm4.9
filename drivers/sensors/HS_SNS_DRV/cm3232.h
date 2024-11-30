#ifndef __LINUX_CM3232_H
#define __LINUX_CM3232_H

#include <linux/bitops.h>

#define CM3232_I2C_NAME "cm3232"

#define CM3232_VDD_MIN_UV	2750000
#define CM3232_VDD_MAX_UV	2950000
#define CM3232_VI2C_MIN_UV	1750000
#define CM3232_VI2C_MAX_UV	1950000


/* cm3232 polling rate in ms */
#define CM3232_LS_IT_100_POLL_DELAY 160
#define CM3232_LS_IT_400_POLL_DELAY 500
#define CM3232_LS_MIN_POLL_DELAY	1
#define CM3232_LS_MAX_POLL_DELAY	1000


/* Define Slave Address*/
#define	CM3232_slave_add	0x10

#define CM3232_IT_TIME_SHIFTER	2
#define CM3232_SENS_SHIFTER		1

#define CM3232_CHIP_RESET		(1 << 6)
#define CM3232_FUNC_OFF			(1 << 0)

#define SUPPORT_LIGHT_COUNT 3
const int light_2_lux_table[2][6] = {
	{14, 28, 56, 112, 224, 448},
	{48, 96, 192, 384, 768, 1536}
};

#endif
