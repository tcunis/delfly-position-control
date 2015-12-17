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


struct SpeedControlRef {
	/* (estimated) velocity reference
	 * in m/s, with #INT32_SPEED_FRAC  */
	union Int32VectLong velocity;

	/* (commanded) acceleration reference
	 * in m/s2, with #INT32_ACCEL_FRAC */
	union Int32VectLong acceleration;
};

struct SpeedControlNow {
	/* current air-speed
	 * in m/s, with #INT32_SPEED_FRAC  */
	int32_t air_speed;

	/* current velocity
	 * in m/s, with #INT32_SPEED_FRAC  */
	union Int32VectLong velocity;

	/* current acceleration
	 * in m/s2, with #INT32_ACCEL_FRAC */
	union Int32VectLong acceleration;
};

struct SpeedControlError {
	/* integrated acceleration error
	 * in m/s, with #INT32_SPEED_FRAC  */
	union Int32VectLong velocity;

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
	 * 0:MAX_PPRZ					   */
	int32_t throttle;
};

struct SpeedControlEquilibrium {
	/* pitch angle at equilibrium
	 * in rad, with #INT32_MATLAB_FRAC */
	int32_t pitch;

	/* throttle frequency at equilibrium
	 * in %, with #INT32_MATLAB_FRAC   */
	int32_t throttle;
};

struct SpeedControlGainScheduling {
	/* pitch gain matrix
	 * dTheta = [pgm1 pgm2] * [h_acc v_acc]^T
	 * with #INT32_MATLAB_FRAC		   */
	struct Int32Vect2 pitch;

	/* throttle gain matrix
	 * dF 	  = [tgm1 tgm2] * [h_acc v_acc]^T
	 * with #INT32_MATLAB_FRAC		   */
	struct Int32Vect2 throttle;
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
};


extern struct SpeedControlVariables speed_control_var;

#endif //SPEED_CONTROL_VAR_H_
