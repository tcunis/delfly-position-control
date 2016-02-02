/*
 * Copyright (C) 2015 Torbjoern Cunis <t.cunis@tudelft.nl>
 *
 * Speed/thrust Control submodule controls commanded horizontal and
 * vertical acceleration.
 *
 * This file is part of paparazzi:
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *//*
 * Copyright (C) 2015 Torbjoern Cunis <t.cunis@tudelft.nl>
 *
 * Speed/thrust Control submodule controls commanded horizontal and
 * vertical acceleration.
 *
 * This file is part of paparazzi:
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file /modules/delfly_control/speed_control.h
 * @author Torbjoern Cunis
 */
/**
 * @file /modules/delfly_control/speed_control.h
 * @author Torbjoern Cunis
 */

#ifndef SPEED_CONTROL_H_
#define SPEED_CONTROL_H_


#include "delfly_state.h"
#include "math/pprz_algebra_int.h"


#define SPEED_CONTROL_MODE_OFF		    0
#define SPEED_CONTROL_MODE_ENTER	    1
#define SPEED_CONTROL_MODE_CONTROL	  2
#define SPEED_CONTROL_TYPE_THROTTLE   3
#define SPEED_CONTROL_TYPE_FLAPFREQ   4

/**
 * Command to speed/thrust control.
 */
struct SpeedControlSetPoint {
	/** h/v acceleration sp
	 * .x = h, .y = v
	 * in m/s2, with #INT32_ACCEL_FRACstatic union Int32VectLong vel_now;// = delfly_state.fv.air;
static union Int32VectLong acc_now;// = delfly_state.fv.acc;
	 *  */
	union Int32VectLong acceleration;

	/** heading s
	 * in rad, with #INT32_ANGLE_FRAC  */
	int32_t heading;
};

/**
 * Speed/thrust control feed-back gains.
 */
struct SpeedControlFeedBackGains {
	struct Int32VectL p;
	struct Int32VectL i;
};

/**
 * Speed/thrust control feed-forward gains.
 */
struct SpeedControlFeedForwardGains {
  int32_t pitch;
  int32_t throttle;
};


struct SpeedControl {
	uint8_t mode;
	uint8_t type;

	struct SpeedControlSetPoint sp;
	struct SpeedControlFeedBackGains fb_gains;
	struct SpeedControlFeedForwardGains ff_gains;

	/* constant pitch offset
	 * in rad, with #INT32_ANGLE_FRAC */
	int32_t pitch_offset;
};


extern struct SpeedControl speed_control;

extern void speed_control_is_running(void);
extern void speed_control_start_running(void);
extern void speed_control_stop_running(void);

extern void speed_control_set_cmd_h( int32_t cmd_h_acceleration, int32_t cmd_heading );
extern void speed_control_set_cmd_v( int32_t cmd_v_acceleration );

/* sets the pitch offset
 * parameter in deg, without FRAC! */
extern void speed_control_set_pitch_offset( int32_t pitch_offset_deg );

#endif /* SPEED_CONTROL_H_ */
