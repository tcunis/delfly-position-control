/*
 * Copyright (C) 2015 Torbjoern Cunis <t.cunis@tudelft.nl>
 *
 * Speed/thrust Control internal variables (for telemetry only).
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
 * @file /modules/delfly_control/speed_control_var.h
 * @author Torbjoern Cunis
 */


#ifndef SPEED_CONTROL_VAR_H_
#define SPEED_CONTROL_VAR_H_


#include "speed_control.h"

#include "math/pprz_algebra_float.h"


struct SpeedControlRef {
  /* (estimated) position reference
   * in m, with #INT32_POS_FRAC     */
  union Int32VectLong position;

	/* (estimated) velocity reference
	 * in m/s, with #INT32_SPEED_FRAC  */
	union Int32VectLong velocity;

	union Int32VectLong velocity_diff;

	/* (commanded) acceleration reference
	 * in m/s2, with #INT32_ACCEL_FRAC */
	union Int32VectLong acceleration;
};

struct SpeedControlNow {
	/* current air-speed
	 * in m/s, with #INT32_SPEED_FRAC  */
	int32_t air_speed;

	/* current position
	 * in m, with #INT32_POS_FRAC   	 */
	union Int32VectLong position;

	/* current velocity
	 * in m/s, with #INT32_SPEED_FRAC  */
	union Int32VectLong velocity;

	union Int32VectLong velocity_diff;

	/* current acceleration
	 * in m/s2, with #INT32_ACCEL_FRAC */
	union Int32VectLong acceleration;
};

struct SpeedControlError {
  /* 2nd-order integrated acceleration error
   * in m, with #INT32_POS_FRAC      */
  union Int32VectLong position;

	/* integrated acceleration error
	 * in m/s, with #INT32_SPEED_FRAC  */
	union Int32VectLong velocity;

	union Int32VectLong velocity_diff;

	/* acceleration error
	 * in m/s2, with #INT32_ACCEL_FRAC */
	union Int32VectLong acceleration;
};

struct SpeedControlCmd {
	/* acceleration command
	 * (before feeding to model)
	 * in m/s2, with #INT32_ACCEL_FRAC */
	union Int32VectLong acceleration;

	/* pitch command
	 * in rad, with #INT32_ANGLE_FRAC  */
	int32_t pitch;

	/* throttle command
	 * 0:MAX_PPRZ					             */
	int32_t throttle;

	/* flapping frequency command
	 * in Hz                           */
	float flapfreq;
};

struct SpeedControlEquilibrium {
  /* air speed at equilibrium
   * in m/s, with #INT32_MATLAB_FRAC */
  int32_t air_speed;

	/* pitch angle at equilibrium
	 * in rad, with #INT32_MATLAB_FRAC */
	int32_t pitch;

	/* throttle command at equilibrium
	 * in %, with #INT32_MATLAB_FRAC/2 */
	int32_t throttle;

	/* flapping frequency at equilibrium
	 * in Hz                           */
	float flapfreq;
};

struct SpeedControlGainScheduling {
	/* pitch gain matrix
	 * dTheta = [pgm1 pgm2] * [h_acc v_acc]^T
	 * with #INT32_MATLAB_FRAC		   */
	struct Int32Vect2 pitch;

	/* throttle gain matrix
	 * dF 	  = [tgm1 tgm2] * [h_acc v_acc]^T
	 * with #INT32_MATLAB_FRAC/2     */
	struct Int32Vect2 throttle;

  /* flapping frequency gain matrix
   * dff    = [fgm1 fgm2] * [h_acc v_acc]^T
   *                               */
	struct FloatVect2 flapfreq;
};

struct SpeedControlAdaptive {

  struct Int32VectL xi;
};


struct SpeedControlVariables {
	struct SpeedControlRef ref;
	struct SpeedControlNow now;
	struct SpeedControlError err;
	struct SpeedControlCmd ff_cmd;
	struct SpeedControlCmd fb_cmd;
	struct SpeedControlCmd cmd;
	struct SpeedControlEquilibrium eq;
	struct SpeedControlGainScheduling mat;
	struct SpeedControlAdaptive adapt;
};


extern struct SpeedControlVariables speed_control_var;

#endif //SPEED_CONTROL_VAR_H_
