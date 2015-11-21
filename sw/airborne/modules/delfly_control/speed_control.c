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
 */
/**
 * @file modules/delfly_control/speed_control.c
 * @author Torbjoern Cunis
 */

#include "generated/airframe.h"

#include "speed_control.h"
#include "delfly_control.h"


struct SpeedControl speed_control;


/* acceleration error
 * in m/s2, with #INT32_ACCEL_FRAC	*/
struct Int32Vect2 speed_control_acceleration_error;

/* integrated acceleration error
 * in m/s, with #INT32_SPEED_FRAC 	*/
struct Int32Vect2 speed_control_integral_error;

/* last speed in h/v direction
 * in m/s, with #INT32_SPEED_FRAC	*/
union Int32Vect2hv speed_control_last_speed;

int32_t speed_control_pitch_cmd;
int32_t speed_control_throttle_cmd;


void speed_control_init (void) {

  VECT2_ASSIGN(speed_control.cmd.acceleration, 0, 0);
  speed_control.cmd.heading = 0;

  speed_control.fb_gains.p = SPEED_CONTROL_FB_PGAIN;
  speed_control.fb_gains.i = SPEED_CONTROL_FB_IGAIN;

  speed_control_pitch_cmd = 0;
  speed_control_throttle_cmd = 0;
}


void speed_control_enter (void) {
  VECT2_ASSIGN(speed_control_integral_error, 0, 0);

  speed_control_integral_error = 0;
  speed_control_last_speed.hv.horizontal = stateGetHorizontalSpeedNorm_i();
  speed_control_last_speed.hv.vertical = 0; //TODO;
}


void speed_control_run (bool_t in_flight) {

  if (!in_flight)
	  return; //nothing to do


}
