/*
 * Copyright (C) Torbjoern Cunis <t.cunis@tudelft.nl>
 *
 * Guidance h/v submodule implements guidance_module.h in order to
 * control vertical and horizontal position and velocity.
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
 * @file "modules/delfly_control/guidance_h_module.c"
 * @author Torbjoern Cunis
 */

#include "delfly_control.h"

#include "delfly_guidance.h"

#include "paparazzi.h"
#include "generated/airframe.h"

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

#include "subsystems/radio_control.h"

#include "delfly_state.h"
#include "speed_control.h"

#include "guidance/guidance_h.h"
#include "guidance/guidance_lat.h"

#include "matlab_include.h"


/* horizontal acceleration command
 * in m/s2, with #INT32_ACCEL_FRAC */
//int32_t guidance_cmd_h_accelerate;

/* heading command
 * in rad, with #INT32_ANGLE_FRAC  */
//int32_t guidance_cmd_heading;

/* horizontal position and velocity error
 * in m, with #INT32_POS_FRAC;
 * in m/s, with #INT32_VEL_FRAC;
 */
//struct Int32Vect2 guidance_h_module_pos_err;
//struct Int32Vect2 guidance_h_module_vel_err;

/* forward position and velocity error
 * in m, with #INT32_POS_FRAC;
 * in m/s, with #INT32_SPEED_FRAC;
 */
//union Int32VectState2 guidance_h_module_fwd_err;

/* forward state gain matrix
 * in 1/s2, with #INT32_MATLAB_FRAC;
 * in 1/s, with #INT32_MATLAB_FRAC;
 */
//union Int32VectState2 guidance_h_module_fwd_gain;


//union Int32VectLong guidance_h_module_vel_rc_sp;


static void guidance_h_module_run_traj( bool_t );


void guidance_h_module_init() {
  //nothing to do yet
  INT32_ZERO(delfly_guidance.cmd.h_acc);
  INT32_ZERO(delfly_guidance.cmd.heading);
  VECT2_ZERO(delfly_guidance.err.fwd.xy);
  VECT2_ZERO(delfly_guidance.err.lat.xy);
  VECT2_ZERO(delfly_guidance.sp.vel_rc.xy);

  VECT2_COPY(delfly_guidance.gains.fwd.xy, matlab_guidance_gain_fwd);
//  guidance_h_module_fwd_gain.states.pos /= (1 << INT32_POS_FRAC);
//  guidance_h_module_fwd_gain.states.vel /= (1 << INT32_SPEED_FRAC);
}


void guidance_h_module_enter() {

  INT32_ZERO(delfly_guidance.cmd.h_acc);
  INT32_ZERO(delfly_guidance.cmd.heading);
  VECT2_ZERO(delfly_guidance.sp.vel_rc.xy);

  guidance_h.rc_sp.psi = delfly_state.h.heading;
}


#define GUIDANCE_H_MODULE_REF_MAX_SPEED   0.3


void guidance_h_module_read_rc(void) {

  stabilization_attitude_read_rc_setpoint_eulers(&guidance_h.rc_sp, TRUE, FALSE, FALSE);
//#if GUIDANCE_H_USE_SPEED_REF
  // negative pitch is forward
  int64_t rc_x = -radio_control.values[RADIO_PITCH];
  int64_t rc_y = radio_control.values[RADIO_ROLL];
  DeadBand(rc_x, MAX_PPRZ / 20);
  DeadBand(rc_y, MAX_PPRZ / 20);

  // convert input from MAX_PPRZ range to SPEED_BFP
  int32_t max_speed = SPEED_BFP_OF_REAL(GUIDANCE_H_MODULE_REF_MAX_SPEED);
  delfly_guidance.sp.vel_rc.fv.fwd = rc_x * max_speed / MAX_PPRZ;
  delfly_guidance.sp.vel_rc.fv.ver = 0; //rc_y * max_speed / MAX_PPRZ;
//#endif
}


void guidance_h_module_run(bool_t in_flight) {

  guidance_h_module_run_traj(in_flight);

  speed_control_set_cmd_h(delfly_guidance.cmd.h_acc, delfly_guidance.cmd.heading);
}


void guidance_h_module_run_traj( bool_t in_flight ) {

	VECT2_DIFF(delfly_guidance.err.pos, guidance_h.sp.pos, delfly_state.h.pos);
	VECT2_DIFF(delfly_guidance.err.vel, guidance_h.sp.speed, delfly_state.h.vel);

	delfly_guidance.err.fwd.states.pos = 0;
	delfly_guidance.err.lat.states.vel = delfly_guidance.sp.vel_rc.fv.fwd;

//	delfly_guidance.cmd.h_acc = delfly_guidance.gains.fwd.states.pos * delfly_guidance.err.fwd.states.pos
//	                  	  	    / (1<<(INT32_MATLAB_FRAC+INT32_POS_FRAC-INT32_ACCEL_FRAC))
//	                          + delfly_guidance.gains.fwd.states.vel * delfly_guidance.err.fwd.states.vel
//	                            / (1<<(INT32_MATLAB_FRAC+INT32_SPEED_FRAC-INT32_ACCEL_FRAC));
	delfly_guidance.cmd.h_acc = delfly_guidance.sp.vel_rc.fv.fwd/(1<<(INT32_SPEED_FRAC-INT32_ACCEL_FRAC));
	delfly_guidance.cmd.heading = guidance_h.rc_sp.psi; //delfly_state.h.heading

	//guidance_lat_adjust_heading( in_flight, cmd_heading, guidance_h_module_pos_err );
}
