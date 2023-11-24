/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_SENSORS_H_INCLUDED
#define __LINUX_SENSORS_H_INCLUDED

#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/rwsem.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/rtc.h>

enum {
    SENSOR_HANDLE_META_DATA = 0,
    SENSOR_HANDLE_ACCELEROMETER = 1,
    SENSOR_HANDLE_MAGNETIC_FIELD = 2,
    SENSOR_HANDLE_GYROSCOPE = 4,
    SENSOR_HANDLE_LIGHT = 5,
    SENSOR_HANDLE_PRESSURE = 6,
    SENSOR_HANDLE_TEMPERATURE = 7,
    SENSOR_HANDLE_PROXIMITY = 8,
    SENSOR_HANDLE_RELATIVE_HUMIDITY = 12,
    SENSOR_HANDLE_AMBIENT_TEMPERATURE = 13,
    SENSOR_HANDLE_MAGNETIC_FIELD_UNCALIBRATED = 14,
    SENSOR_HANDLE_SIGNIFICANT_MOTION = 17,
    SENSOR_HANDLE_STEP_DETECTOR = 18,
    SENSOR_HANDLE_STEP_COUNTER = 19,
    SENSOR_HANDLE_HEART_RATE = 21,
    SENSOR_HANDLE_STATIONARY_DETECT = 29,
    SENSOR_HANDLE_MOTION_DETECT = 30,
    SENSOR_HANDLE_HEART_BEAT = 31,
    SENSOR_HANDLE_DYNAMIC_SENSOR_META = 32,
    SENSOR_HANDLE_LOW_LATENCY_OFFBODY_DETECT = 34,
	SENSOR_HANDLE_IR = 36,
	SENSOR_HANDLE_UV = 40,
	SENSOR_HANDLE_HALL = 41,
};

#define SENSORS_HANDLE_EXTEND_BASE		128

enum {
    SENSOR_TYPE_META_DATA				= 0,
    SENSOR_TYPE_ACCELEROMETER			= 1,
    SENSOR_TYPE_MAGNETIC_FIELD			= 2,
    SENSOR_TYPE_ORIENTATION				= 3,
    SENSOR_TYPE_GYROSCOPE				= 4,
    SENSOR_TYPE_LIGHT					= 5,
    SENSOR_TYPE_PRESSURE				= 6,
    SENSOR_TYPE_TEMPERATURE				= 7,
    SENSOR_TYPE_PROXIMITY				= 8,
    SENSOR_TYPE_GRAVITY					= 9,
    SENSOR_TYPE_LINEAR_ACCELERATION		= 10,
    SENSOR_TYPE_ROTATION_VECTOR			= 11,
    SENSOR_TYPE_RELATIVE_HUMIDITY		= 12,
    SENSOR_TYPE_AMBIENT_TEMPERATURE		= 13,
    SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED = 14,
    SENSOR_TYPE_GAME_ROTATION_VECTOR	= 15,
    SENSOR_TYPE_GYROSCOPE_UNCALIBRATED	= 16,
    SENSOR_TYPE_SIGNIFICANT_MOTION		= 17,
    SENSOR_TYPE_STEP_DETECTOR			= 18,
    SENSOR_TYPE_STEP_COUNTER			= 19,
    SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR = 20,
    SENSOR_TYPE_HEART_RATE				= 21,
    SENSOR_TYPE_TILT_DETECTOR			= 22,
    SENSOR_TYPE_WAKE_GESTURE			= 23,
    SENSOR_TYPE_GLANCE_GESTURE			= 24,
    SENSOR_TYPE_PICK_UP_GESTURE			= 25,
    SENSOR_TYPE_WRIST_TILT_GESTURE		= 26,
    SENSOR_TYPE_DEVICE_ORIENTATION		= 27,
    SENSOR_TYPE_POSE_6DOF				= 28,
    SENSOR_TYPE_STATIONARY_DETECT		= 29,
    SENSOR_TYPE_MOTION_DETECT			= 30,
    SENSOR_TYPE_HEART_BEAT				= 31,
    SENSOR_TYPE_DYNAMIC_SENSOR_META		= 32,
    SENSOR_TYPE_ADDITIONAL_INFO			= 33,
    SENSOR_TYPE_LOW_LATENCY_OFFBODY_DETECT = 34,
    SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED = 35,
	PYSUEDO_SENSOR_TYPE_IR				= 36,
	PYSUEDO_SENSOR_TYPE_UV				= 40,
	PYSUEDO_SENSOR_TYPE_HALL			= 41,
	SENSOR_TYPE_FROM_KERNEL_MAX			= 0xFF,
    SENSOR_TYPE_DEVICE_PRIVATE_BASE = 65536, // 0x10000
};

enum sensor_extend_position_index {
	POSI_FRONT = 0x1000,
	POSI_BACK = 0x2000,
	POSI_TOP = 0x3000,
	POSI_BUTT = 0x4000,
	POSI_LEFT = 0x5000,
	POSI_RIGHT = 0x6000,
};

enum {
    SENSOR_FLAG_WAKE_UP = 1u,			// 1
    SENSOR_FLAG_CONTINUOUS_MODE = 0u,	// 0
    SENSOR_FLAG_ON_CHANGE_MODE = 2u,	// 2
    SENSOR_FLAG_ONE_SHOT_MODE = 4u,		// 4
    SENSOR_FLAG_SPECIAL_REPORTING_MODE = 6u, // 6
    SENSOR_FLAG_DATA_INJECTION = 16u,	// 0x10
    SENSOR_FLAG_DYNAMIC_SENSOR = 32u,	// 0x20
    SENSOR_FLAG_ADDITIONAL_INFO = 64u,	// 0x40
    SENSOR_FLAG_DIRECT_CHANNEL_ASHMEM = 1024u,	// 0x400
    SENSOR_FLAG_DIRECT_CHANNEL_GRALLOC = 2048u,	// 0x800
    SENSOR_FLAG_MASK_REPORTING_MODE = 14u,	// 0xE
    SENSOR_FLAG_MASK_DIRECT_REPORT = 896u,	// 0x380
    SENSOR_FLAG_MASK_DIRECT_CHANNEL = 3072u,	// 0xC00
};

#define ACCELEROMETER_NAME	"accelerometer"
#define COMPASS_NAME		"compass"
#define GYROSCOPE_NAME		"gyroscope"
#define LIGHT_NAME			"light"
#define PRESSURE_NAME		"pressure"
#define TEMPERATURE_NAME	"temperature"
#define PROXIMITY_NAME		"proximity"
#define RELATIVE_HUMIDITY_NAME	"relative_humidity"
#define AMBIENT_TEMPERATURE_NAME	"ambient_temperature"
#define SIGNIFICANT_MOTION_NAME		"significant_motion"
#define STEP_COUNTER_NAME		"step_counter"
#define STEP_DETECTOR_NAME		"step_detector"
#define UV_NAME			"ultra_violet"
#define HALL_NAME			"hall"

#define EXTEND_SENSOR_NAME_PREFIX	"extend_"

#define AXIS_FACTOR		0
#define AXIS_OFFSET		1

struct cal_result_t {
	union {
		struct {
			int offset_x; /*axis offset of x axis*/
			int offset_y; /*axis offset of x axis*/
			int offset_z; /*axis offset of x axis*/
		};
		struct {
			int threshold_h; /*proximity threshold_h*/
			int threshold_l; /*proximity threshold_l*/
			int bias; /*proximity measure data noise*/
		};
		int offset[3];
	};
	int factor; /*light sensor factor for real ligt strength*/
	int range;
	struct cal_result_t *node;
};

/**
 * struct sensors_classdev - hold the sensor general parameters and APIs
 * @dev:		The device to register.
 * @node:		The list for the all the sensor drivers.
 * @name:		Name of this sensor.
 * @vendor:		The vendor of the hardware part.
 * @handle:		The handle that identifies this sensors.
 * @type:		The sensor type.
 * @max_range:		The maximum range of this sensor's value in SI units.
 * @resolution:		The smallest difference between two values reported by
 *			this sensor.
 * @sensor_power:	The rough estimate of this sensor's power consumption
 *			in mA.
 * @min_delay:		This value depends on the trigger mode:
 *			continuous: minimum period allowed in microseconds
 *			on-change : 0
 *			one-shot :-1
 *			special : 0, unless otherwise noted
 * @fifo_reserved_event_count:	The number of events reserved for this sensor
 *				in the batch mode FIFO.
 * @fifo_max_event_count:	The maximum number of events of this sensor
 *				that could be batched.
 * @max_delay:		The slowest rate the sensor supports in millisecond.
 * @flags:		Should be '1' if the sensor is a wake up sensor.
 *			set it to '0' otherwise.
 * @enabled:		Store the sensor driver enable status.
 * @delay_msec:		Store the sensor driver delay value. The data unit is
 *			millisecond.
 * @wakeup:		Indicate if the wake up interrupt has been enabled.
 * @max_latency:	Max report latency in millisecond
 * @sensors_enable:	The handle for enable and disable sensor.
 * @sensors_poll_delay:	The handle for set the sensor polling delay time.
 * @sensors_set_latency:Set the max report latency of the sensor.
 * @sensors_flush:	Flush sensor events in FIFO and report it to user space.
 * @position: indicate the chip location on the phone.
 * @params		The sensor calibrate string format params up to userspace.
 * @cal_result		The sensor calibrate parameters, cal_result is a struct for sensor.
 */
struct sensors_classdev {
	struct device		*dev;
	struct list_head	node;
	const char		*name;
	const char		*vendor;
	int			version;
	int			handle;
	int			type;
	const char		*max_range;
	const char		*resolution;
	const char		*sensor_power;
	int			min_delay;
	int			fifo_reserved_event_count;
	int			fifo_max_event_count;
	int32_t			max_delay;
	uint32_t		flags;

	unsigned int		enabled;
	unsigned int		delay_msec;
	unsigned int		wakeup;
	unsigned int		max_latency;
	char *position;
	char *extend_i;
	char			*params;
	struct cal_result_t	cal_result;
	/* enable and disable the sensor handle*/
	int	(*sensors_enable)(struct sensors_classdev *sensors_cdev,
					unsigned int enabled);
	int	(*sensors_poll_delay)(struct sensors_classdev *sensors_cdev,
					unsigned int delay_msec);
	int	(*sensors_self_test)(struct sensors_classdev *sensors_cdev,int *pTest);
	int	(*sensors_set_latency)(struct sensors_classdev *sensor_cdev,
					unsigned int max_latency);
	int	(*sensors_enable_wakeup)(struct sensors_classdev *sensor_cdev,
					unsigned int enable);
	int	(*sensors_flush)(struct sensors_classdev *sensors_cdev);
	int	(*sensors_calibrate)(struct sensors_classdev *sensor_cdev,
					int axis, int apply_now);
	int	(*sensors_write_cal_params)(struct sensors_classdev
				*sensor_cdev, struct cal_result_t *cal_result);
};

extern int sensors_classdev_register(struct device *parent,
				 struct sensors_classdev *sensors_cdev);
extern void sensors_classdev_unregister(struct sensors_classdev *sensors_cdev);
extern void sensors_construct_extend_name(char *base,
		char *type_name, const char *position, int index);
extern int sensors_construct_extend_type_num(int base_type_id,
		const char *position, int index);
extern struct input_dev *sensors_construct_input_dev(struct device *parent_dev,
			char *name, int axes_num, void *priv_data);
extern void sensors_marker(char *src_func,
	char *action, char *object, char *para);
extern void sensors_report_2(struct input_dev *idev,
		int val_1, int val_2, int sensi, uint8_t *c_i);
extern void sensors_report_1(struct input_dev *idev, int val_1);

#endif		/* __LINUX_SENSORS_H_INCLUDED */
