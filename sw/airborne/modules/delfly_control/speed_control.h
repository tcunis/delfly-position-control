/*
 * Copyright (C) 2015 Torbjoern Cunis <t.cunis@tudelft.nl>
 *
 * The DelFly Control module provides submodules and functions necessary
 * for control of the DelFly:
 *
 *  -	Guidance h/v submodule implements guidance_module.h in order to
 *    	control vertical and horizontal position and velocity;
 *  - 	Speed/thrust Control submodule controls commanded horizontal and
 *    	vertical acceleration.
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

#ifndef SPEED_CONTROL_H_
#define SPEED_CONTROL_H_


union Int32Vect2hv {
	struct Int32Vect2 xy;
	struct {
		int32_t horizontal;
		int32_t vertical;
	} hv;
};

/**
 * Command to speed/thrust control.
 */
struct SpeedControlCmd {
	/** h/v acceleration cmd
	 * .x = h, .y = v
	 * in m/s2, with #INT32_ACCEL_FRAC */
	struct Int32Vect2 acceleration;

	/** heading cmd
	 * in rad, with #INT32_ANGLE_FRAC  */
	int32_t heading;
};

/**
 * Speed/thrust control feed-back gains.
 */
struct SpeedControlFeedBackGains {
	int32_t p;
	int32_t i;
};


struct SpeedControl {
	struct SpeedControlCmd cmd;
	struct SpeedControlFeedBackGains fb_gains;
};


extern struct SpeedControl speed_control;


extern void speed_control_set_cmd_h( int32_t cmd_h_acceleration, int32_t cmd_heading );
extern void speed_control_set_cmd_v( int32_t cmd_v_acceleration );


#endif /* SPEED_CONTROL_H_ */
