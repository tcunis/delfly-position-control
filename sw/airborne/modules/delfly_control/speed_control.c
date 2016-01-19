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
#include "speed_control_var.h"
#include "delfly_control.h"

#include "flap_control.h"

#include "delfly_model.h"
#include "delfly_state.h"

#include "matlab_include.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"

#include "delfly_algebra_int.h"
#include "speed_control_var.h"



struct SpeedControl speed_control;
struct SpeedControlVariables speed_control_var;

struct SpeedControlEquilibrium speed_control_eq_zero = {.pitch = 0, .throttle = 0};



static void speed_control_estimate_error (void);
static void speed_control_calculate_cmd( struct SpeedControlCmd* cmd, struct SpeedControlEquilibrium* eq, struct SpeedControlGainScheduling* mat );



void speed_control_init (void) {

  speed_control.mode = SPEED_CONTROL_MODE_OFF;

  VECT2_ZERO(speed_control.sp.acceleration.xy);
  speed_control.sp.heading = 0;

  speed_control.fb_gains.p = SPEED_CONTROL_FB_PGAIN;
  speed_control.fb_gains.i = SPEED_CONTROL_FB_IGAIN;

  speed_control.ff_gains.pitch    = SPEED_CONTROL_FF_PITCH_GAIN;
  speed_control.ff_gains.throttle = SPEED_CONTROL_FF_THROTTLE_GAIN;

  speed_control_set_pitch_offset(SPEED_CONTROL_PITCH_OFFSET);

  VECT2_ZERO(speed_control_var.ref.velocity.xy);
  VECT2_ZERO(speed_control_var.ref.acceleration.xy);

  VECT2_ZERO(speed_control_var.now.acceleration.xy);
  VECT2_ZERO(speed_control_var.now.velocity.xy);
  VECT2_ZERO(speed_control_var.now.acceleration.xy);

  VECT2_ZERO(speed_control_var.err.acceleration.xy);
  VECT2_ZERO(speed_control_var.err.velocity.xy);

  VECT2_ZERO(speed_control_var.cmd.acceleration.xy);
  speed_control_var.cmd.pitch = 0;
  speed_control_var.cmd.throttle = 0;

  VECT2_ZERO(speed_control_var.ff_cmd.acceleration.xy);
  speed_control_var.ff_cmd.pitch = 0;
  speed_control_var.ff_cmd.throttle = 0;

  VECT2_ZERO(speed_control_var.fb_cmd.acceleration.xy);
  speed_control_var.fb_cmd.pitch = 0;
  speed_control_var.fb_cmd.throttle = 0;
}

void speed_control_set_cmd_h( int32_t cmd_h_acceleration, int32_t cmd_heading ) {

  speed_control.sp.acceleration.fv.fwd = cmd_h_acceleration;
  speed_control.sp.heading = cmd_heading;
}

void speed_control_set_cmd_v( int32_t cmd_v_acceleration ) {

  speed_control.sp.acceleration.fv.ver = cmd_v_acceleration;
}

void speed_control_set_pitch_offset( int32_t pitch_offset_deg ) {
  speed_control.pitch_offset = INT32_RAD_OF_DEG(ANGLE_BFP_OF_REAL(pitch_offset_deg));
}


void speed_control_enter (void) {

  speed_control.mode = SPEED_CONTROL_MODE_ENTER;

  VECT2_ZERO(speed_control_var.err.velocity.xy);

  //initial velocity ref: v_ref(0) = v0
  VECT2_COPY(speed_control_var.ref.velocity.xy, delfly_state.fv.air.xy);

  flap_control_enter();

  delfly_model_enter();
  delfly_model_set_cmd( speed_control.sp.acceleration.fv.fwd, speed_control.sp.acceleration.fv.ver );

  /* TODO: gain scheduling w.r.t. air speed */
  VECT2_COPY(speed_control_var.mat.pitch, matlab_pitch_matrix_v08);
  VECT2_COPY(speed_control_var.mat.throttle, matlab_throttle_matrix_v08);
  speed_control_var.eq.pitch 	= matlab_pitch_equilibrium_v08;     // pitch at eq in rad, with #INT32_MATLAB_FRAC
  speed_control_var.eq.throttle = matlab_throttle_equilibrium_v08;  // throttle at eq in %, with #INT32_MATLAB_FRAC
}


void speed_control_estimate_error (void) {

  VECT2_COPY(speed_control_var.now.velocity.xy, delfly_state.fv.air.xy);
  VECT2_COPY(speed_control_var.now.acceleration.xy, delfly_state.fv.acc.xy);

  //update velocity ref: v_ref(k) = v_ref(k-1) + T*a_cmd(k-1)
  //TODO: use reference model!
  VECT2_ADD_SCALED(speed_control_var.ref.velocity.xy,
		  	  	       speed_control_var.ref.acceleration.xy,
				           SPEED_CONTROL_RUN_PERIOD*(1<<(INT32_SPEED_FRAC-INT32_ACCEL_FRAC)));

  //velocity error: v_err = v_ref - v_now
  union Int32VectLong velocity_error_new;
  VECT2_DIFF(velocity_error_new.xy,
		  	 speed_control_var.ref.velocity.xy,
			 speed_control_var.now.velocity.xy);

  //estimated acceleration error: a_err(k-1) = (v_err(k) - v_err(k-1))/T
  VECT2_DIFF_SCALED2(speed_control_var.err.acceleration.xy,
		  	  	    velocity_error_new.xy,
					speed_control_var.err.velocity.xy,
					SPEED_CONTROL_RUN_FREQ,(1<<(INT32_SPEED_FRAC-INT32_ACCEL_FRAC)));
  VECT2_COPY(speed_control_var.err.velocity.xy, velocity_error_new.xy);

  //for telemetry: a_now = a_ref - a_err
  VECT2_DIFF(speed_control_var.now.acceleration.xy,
		  	 speed_control.sp.acceleration.xy,
			 speed_control_var.err.acceleration.xy);
}


void speed_control_run (bool_t in_flight) {

  speed_control_var.now.air_speed = delfly_state.h.speed_air;

  if (!in_flight || guidance_v_mode != GUIDANCE_V_MODE_MODULE)
	  return speed_control_enter(); //nothing to do

  //else:
  speed_control.mode = SPEED_CONTROL_MODE_CONTROL;

  /* feed-forward */
  //union Int32VectLong acceleration_cmd;
  VECT2_COPY(speed_control_var.ff_cmd.acceleration.xy, speed_control.sp.acceleration.xy);

  /* feed-back */
  speed_control_estimate_error();
  VECT2_ZERO(speed_control_var.fb_cmd.acceleration.xy);
  //if p = 100 %, 1 m/s2 acceleration error equals to +1 m/s2 acceleration command
  VECT2_ADD_SCALED2( speed_control_var.fb_cmd.acceleration.xy,
                     speed_control_var.err.acceleration.xy,
                     speed_control.fb_gains.p, 100 );
  //if i = 100 %, 1 m/s velocity error equals to +1 m/s2 acceleration command
  VECT2_ADD_SCALED2( speed_control_var.fb_cmd.acceleration.xy,
		  	  	     speed_control_var.err.velocity.xy,
		  	  	     speed_control.fb_gains.i,
					 (100<<(INT32_SPEED_FRAC-INT32_ACCEL_FRAC)) );

  /* pitch and throttle command */
  //cmd = ff_cmd + fb_cmd
  VECT2_SUM(speed_control_var.cmd.acceleration.xy,
		        speed_control_var.ff_cmd.acceleration.xy,
		        speed_control_var.fb_cmd.acceleration.xy);

  speed_control_calculate_cmd(&speed_control_var.cmd, &speed_control_var.eq, &speed_control_var.mat);

  int32_t cmd_throttle    = TRIM_UPPRZ(speed_control_var.cmd.throttle);

  //for telemetry:
  speed_control_calculate_cmd(&speed_control_var.ff_cmd, &speed_control_eq_zero, &speed_control_var.mat);
  speed_control_calculate_cmd(&speed_control_var.fb_cmd, &speed_control_eq_zero, &speed_control_var.mat);
  speed_control_var.fb_cmd.throttle = TRIM_UPPRZ(speed_control_var.fb_cmd.throttle);

  static struct Int32Eulers orientation_cmd;
  orientation_cmd.phi   = 0;
  orientation_cmd.theta = speed_control_var.cmd.pitch - speed_control.pitch_offset;
  orientation_cmd.psi   = speed_control.sp.heading;

  stabilization_attitude_set_rpy_setpoint_i( &orientation_cmd );
  stabilization_cmd[COMMAND_THRUST] = cmd_throttle;
  //flap_control_run();

  stabilization_attitude_run(in_flight);

  if ( cmd_throttle == speed_control_var.cmd.throttle ) {
	  // throttle command un-saturated
    delfly_model_set_cmd( speed_control.sp.acceleration.fv.fwd, speed_control.sp.acceleration.fv.ver );
    VECT2_COPY(speed_control_var.ref.acceleration.xy, speed_control.sp.acceleration.xy);
  } else {
	  // throttle command saturated
	  speed_control_var.cmd.throttle = cmd_throttle;
	  delfly_model_set_cmd( 0, 0 );
	  VECT2_ZERO(speed_control_var.ref.acceleration.xy);
	}

}



void speed_control_calculate_cmd( struct SpeedControlCmd* cmd, struct SpeedControlEquilibrium* eq, struct SpeedControlGainScheduling* mat ) {
	//commanded pitch in rad, with #INT32_ANGLE_FRAC
	cmd->pitch 	  = (eq->pitch * (1 << INT32_ACCEL_FRAC)
			             + VECT2_DOT_PRODUCT(mat->pitch, cmd->acceleration.xy))
			          / (1 << (INT32_ACCEL_FRAC + INT32_MATLAB_FRAC - INT32_ANGLE_FRAC));
	//commanded throttle in 0:MAX_PPRZ
	cmd->throttle = (eq->throttle //* (1 << INT32_ACCEL_FRAC)
					         + VECT2_DOT_PRODUCT(mat->throttle, cmd->acceleration.xy)/(1 << INT32_ACCEL_FRAC))
				        * (MAX_PPRZ*speed_control.ff_gains.throttle/100) / (100 << INT32_MATLAB_FRAC/2);
}
