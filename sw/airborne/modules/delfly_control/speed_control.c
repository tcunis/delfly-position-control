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

#include "paparazzi.h"
#include "generated/airframe.h"

#include "speed_control.h"
#include "delfly_control.h"

#include "delfly_state.h"

#include "matlab_include.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"

#include "math/pprz_algebra.h"


/* a += b*s */
#define VECT2_ADD_SCALED(_a, _b, _s) {   \
    (_a).x += ((_b).x * (_s));           \
    (_a).y += ((_b).y * (_s));           \
  }

#define VECT2_DIFF_SCALED(_c, _a, _b, _s) { 	\
	(_c).x = ((_a).x - (_b).x)*(_s);			\
	(_c).y = ((_a).y - (_b).y)*(_s);			\
  }


struct SpeedControl speed_control;


/* acceleration error
 * in m/s2, with #INT32_ACCEL_FRAC	*/
union Int32VectLong speed_control_acceleration_error;

/* integrated acceleration error
 * in m/s, with #INT32_SPEED_FRAC 	*/
union Int32VectLong speed_control_velocity_error;

/* velocity set-point
 * in m/s, with #INT32_SPEED_FRAC   */
union Int32VectLong speed_control_velocity_sp;


/* pitch command
 * in rad, with #INT32_ANGLE_FRAC   */
int32_t speed_control_pitch_cmd;
/* throttle cmd
 * in */
int32_t speed_control_throttle_cmd;


static void speed_control_estimate_error (void);


void speed_control_init (void) {

  VECT2_ZERO(speed_control.sp.acceleration.xy);

  speed_control.sp.heading = 0;

  speed_control.fb_gains.p = SPEED_CONTROL_FB_PGAIN;
  speed_control.fb_gains.i = SPEED_CONTROL_FB_IGAIN;

  speed_control.ff_gains.pitch    = SPEED_CONTROL_FF_PITCH_GAIN;
  speed_control.ff_gains.throttle = SPEED_CONTROL_FF_THROTTLE_GAIN;

  speed_control_pitch_cmd = 0;
  speed_control_throttle_cmd = 0;

  VECT2_ZERO(speed_control_acceleration_error.xy);
  VECT2_ZERO(speed_control_velocity_error.xy);
  VECT2_ZERO(speed_control_velocity_sp.xy);
}

void speed_control_set_cmd_h( int32_t cmd_h_acceleration, int32_t cmd_heading ) {

  speed_control.sp.acceleration.fv.fwd = cmd_h_acceleration;
  speed_control.sp.heading = cmd_heading;
}

void speed_control_set_cmd_v( int32_t cmd_v_acceleration ) {

  speed_control.sp.acceleration.fv.ver = cmd_v_acceleration;
}


void speed_control_enter (void) {
  VECT2_ZERO(speed_control_velocity_error.xy);

  speed_control_velocity_sp = delfly_state.fv.air;
}


void speed_control_estimate_error (void) {

  union Int32VectLong vel_now = delfly_state.fv.air;
  union Int32VectLong acc_now = delfly_state.fv.acc;

  VECT2_ADD_SCALED(speed_control_velocity_sp.xy,
		  	  	   speed_control.sp.acceleration.xy,
				   SPEED_CONTROL_RUN_PERIOD*(1<<(INT32_SPEED_FRAC-INT32_ACCEL_FRAC)));

  union Int32VectLong velocity_error_new;
  VECT2_DIFF(velocity_error_new.xy, speed_control_velocity_sp.xy, vel_now.xy);

  VECT2_DIFF_SCALED(speed_control_acceleration_error.xy,
		  	  	    velocity_error_new.xy,
					speed_control_velocity_error.xy,
					SPEED_CONTROL_RUN_FREQ/(1<<(INT32_SPEED_FRAC-INT32_ACCEL_FRAC)));
  VECT2_COPY(speed_control_velocity_error.xy, velocity_error_new.xy);
}


void speed_control_run (bool_t in_flight) {

  if (!in_flight || guidance_v_mode != GUIDANCE_V_MODE_MODULE)
	  return speed_control_enter(); //nothing to do

  /* feed-forward */
  union Int32VectLong acceleration_cmd;
  VECT2_COPY(acceleration_cmd.xy, speed_control.sp.acceleration.xy);

  /* feed-back */
  speed_control_estimate_error();
  //speed_control.fb_gains.p*
  VECT2_ADD_SCALED(acceleration_cmd.xy, speed_control_acceleration_error.xy, speed_control.fb_gains.p*1.0/100);
  //speed_control.fb_gains.i*
  VECT2_ADD_SCALED(acceleration_cmd.xy, speed_control_velocity_error.xy, speed_control.fb_gains.i*1.0/100);


  /* pitch and throttle command */
  static struct Int32Vect2 pitch_matrix;
  static struct Int32Vect2 throttle_matrix;
  static int32_t pitch_at_equilibrium;
  static int32_t throttle_at_equilibrium;

  VECT2_COPY(pitch_matrix, matlab_pitch_matrix_v08);
  VECT2_COPY(throttle_matrix, matlab_throttle_matrix_v08);
  pitch_at_equilibrium = matlab_pitch_equilibrium_v08;        // pitch at eq in rad, with #INT32_MATLAB_FRAC
  throttle_at_equilibrium = matlab_throttle_equilibrium_v08;  // throttle at eq in %, with #INT32_MATLAB_FRAC

  // commanded pitch in rad, with #INT32_ANGLE_FRAC
  speed_control_pitch_cmd    = ( pitch_at_equilibrium * (1<<INT32_ACCEL_FRAC)
                                 + VECT2_DOT_PRODUCT(pitch_matrix, acceleration_cmd.xy)
                               )/(1<<(INT32_ACCEL_FRAC+INT32_MATLAB_FRAC-INT32_ANGLE_FRAC));
  // commanded throttle in 0:MAX_PPRZ
  speed_control_throttle_cmd = ( throttle_at_equilibrium * (1<<INT32_ACCEL_FRAC)
                                 + VECT2_DOT_PRODUCT(throttle_matrix, acceleration_cmd.xy)
                               )*MAX_PPRZ/(100*(1<<(INT32_MATLAB_FRAC+INT32_ACCEL_FRAC)));

  static struct Int32Eulers orientation_cmd;
  orientation_cmd.phi   = 0;
  orientation_cmd.theta = speed_control_pitch_cmd;
  orientation_cmd.psi   = speed_control.sp.heading;

  stabilization_attitude_set_rpy_setpoint_i( &orientation_cmd );
  stabilization_cmd[COMMAND_THRUST] = speed_control_throttle_cmd;

  stabilization_attitude_run(in_flight);
}
