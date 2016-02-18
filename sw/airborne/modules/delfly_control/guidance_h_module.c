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



#ifndef DELFLY_GUIDANCE_COMPLEMENTARY_HEADING_GAIN
//use heading pseudo command directly
#define DELFLY_GUIDANCE_COMPLEMENTARY_HEADING_GAIN    100
#endif

#define MAX_HEADING_DELTA   INT32_ANGLE_PI_4


static void guidance_h_module_run_traj( bool_t );


void guidance_h_module_init() {

  delfly_guidance_init();

  delfly_guidance.gains.h.lateral_ratio      = DELFLY_GUIDANCE_LATERAL_RATIO;
  delfly_guidance.gains.h.complementary_gain = DELFLY_GUIDANCE_COMPLEMENTARY_HEADING_GAIN;

//  VECT2_COPY(delfly_guidance.gains.fwd.xy, matlab_guidance_gain_fwd);
  VECT2_ASSIGN(delfly_guidance.gains.fwd.xy, 1, 2);

//  VECT2_COPY(delfly_guidance.gains.lat.xy, matlab_guidance_gain_fwd);
  VECT2_ASSIGN(delfly_guidance.gains.lat.xy, 1, 2);

  delfly_guidance.gains.lat_i = DELFLY_GUIDANCE_LATERAL_IGAIN;


//  guidance_h_module_fwd_gain.states.pos /= (1 << INT32_POS_FRAC);
//  guidance_h_module_fwd_gain.states.vel /= (1 << INT32_SPEED_FRAC);
}


void guidance_h_module_enter() {

  delfly_guidance_enter();

  INT32_ZERO(delfly_guidance.cmd.h_acc);
  INT32_ZERO(delfly_guidance.cmd.heading);
  VECT2_ZERO(delfly_guidance.sp.vel_rc.xy);

  INT32_ZERO(delfly_guidance.err.lat_pos_int);

  switch (delfly_guidance.mode) {

  case DELFLY_GUIDANCE_MODE_MODULE:
    delfly_guidance.sp.att_rc.psi = delfly_state.h.heading;
    break;

  case DELFLY_GUIDANCE_MODE_NAV:
  default:
  {} //nothing to do
  }
}


#define DELFLY_GUIDANCE_REF_MAX_SPEED   0.3


void guidance_h_module_read_rc(void) {

//  switch (delfly_guidance.mode) {
//
//  case DELFLY_GUIDANCE_MODE_MODULE:
//  {
	stabilization_attitude_read_rc_setpoint_eulers(&delfly_guidance.sp.att_rc, TRUE, FALSE, FALSE);
//#if GUIDANCE_H_USE_SPEED_REF
	// negative pitch is forward
	int64_t rc_x = -radio_control.values[RADIO_PITCH];
	int64_t rc_y = radio_control.values[RADIO_ROLL];
	DeadBand(rc_x, MAX_PPRZ / 20);
	DeadBand(rc_y, MAX_PPRZ / 20);

	// convert input from MAX_PPRZ range to SPEED_BFP
	int32_t max_speed = SPEED_BFP_OF_REAL(DELFLY_GUIDANCE_REF_MAX_SPEED);
	delfly_guidance.sp.vel_rc.fv.fwd = rc_x * max_speed / MAX_PPRZ;
	delfly_guidance.sp.vel_rc.fv.ver = 0; //rc_y * max_speed / MAX_PPRZ;
//#endif
//  } break;
//
//  case DELFLY_GUIDANCE_MODE_NAV:
//  default:
//  {} //nothing to do
//  }
}


void guidance_h_module_run(bool_t in_flight) {

  delfly_guidance_run();

  guidance_h_module_run_traj(in_flight);

  speed_control_set_cmd_h(delfly_guidance.cmd.h_acc, delfly_guidance.cmd.heading);
}


void guidance_h_module_run_traj( bool_t in_flight ) {


  switch (delfly_guidance.mode) {

  case DELFLY_GUIDANCE_MODE_MODULE:
    {
      delfly_guidance.err.fwd.states.pos = 0;
      delfly_guidance.err.fwd.states.vel = delfly_guidance.sp.vel_rc.fv.fwd;

//    	delfly_guidance.cmd.h_acc = delfly_guidance.gains.fwd.states.pos * delfly_guidance.err.fwd.states.pos
//    	                  	  	    / (1<<(INT32_MATLAB_FRAC+INT32_POS_FRAC-INT32_ACCEL_FRAC))
//    	                          + delfly_guidance.gains.fwd.states.vel * delfly_guidance.err.fwd.states.vel
//    	                            / (1<<(INT32_MATLAB_FRAC+INT32_SPEED_FRAC-INT32_ACCEL_FRAC));

      delfly_guidance.cmd.h_acc = delfly_guidance.sp.vel_rc.fv.fwd/(1<<(INT32_SPEED_FRAC-INT32_ACCEL_FRAC));
      delfly_guidance.cmd.heading = delfly_guidance.sp.heading; //delfly_state.h.heading
    }
    break;

  case DELFLY_GUIDANCE_MODE_NAV:
    {
      VECT2_DIFF(delfly_guidance.err.pos, delfly_guidance.sp.pos, delfly_state.h.pos);
      VECT2_DIFF(delfly_guidance.err.vel, delfly_guidance.sp.vel, delfly_state.h.vel);

      delfly_guidance.err.fwd.states.pos = VECT2_GET_FWD(delfly_guidance.err.pos, delfly_guidance.sp.heading);
      delfly_guidance.err.fwd.states.vel = VECT2_GET_FWD(delfly_guidance.err.vel, delfly_guidance.sp.heading);

      delfly_guidance.err.lat.states.pos = VECT2_GET_LAT(delfly_guidance.err.pos, delfly_guidance.sp.heading);
      delfly_guidance.err.lat.states.vel = VECT2_GET_LAT(delfly_guidance.err.vel, delfly_guidance.sp.heading);

      delfly_guidance.err.lat_pos_int += delfly_guidance.err.lat.states.pos*(1.0/PERIODIC_FREQUENCY)*(1<<(INT32_SPEED_FRAC-INT32_POS_FRAC));

//      delfly_guidance.cmd.h_acc = 0;
    	delfly_guidance.cmd.h_acc = delfly_guidance.gains.fwd.states.pos * delfly_guidance.err.fwd.states.pos
    	                  	  	    / (1<<(/*INT32_MATLAB_FRAC+*/INT32_POS_FRAC-INT32_ACCEL_FRAC))
    	                          + delfly_guidance.gains.fwd.states.vel * delfly_guidance.err.fwd.states.vel
    	                            / (1<<(/*INT32_MATLAB_FRAC+*/INT32_SPEED_FRAC-INT32_ACCEL_FRAC));

      /* lateral guidance pseudo acceleration command
       * in m/s2, with #INT32_ACCEL_FRAC                  */
      int64_t pseudo_cmd_lat_acc = delfly_guidance.err.lat.states.pos   * delfly_guidance.gains.lat.states.pos / (1<<(/*INT32_MATLAB_FRAC+*/INT32_POS_FRAC-INT32_ACCEL_FRAC))
                                   + delfly_guidance.err.lat.states.vel * delfly_guidance.gains.lat.states.vel / (1<<(/*INT32_MATLAB_FRAC+*/INT32_SPEED_FRAC-INT32_ACCEL_FRAC))
                                   + delfly_guidance.err.lat_pos_int    * delfly_guidance.gains.lat_i          / (100*(1<<(INT32_SPEED_FRAC-INT32_ACCEL_FRAC)));
      /* lateral guidance pseudo heading difference
       * i.e. additional heading to set-point
       * in [-MAX_HEADING_DELTA, +MAX_HEADING_DELTA]
       * in rad, with #INT32_ANGLE_FRAC                   */
      int32_t pseudo_heading_d   = delfly_guidance.gains.h.lateral_ratio*pseudo_cmd_lat_acc*INT32_ANGLE_PI_4/(100*(1<<INT32_ACCEL_FRAC));
      INT32_STRIM(pseudo_heading_d, MAX_HEADING_DELTA);

      /* lateral guidance pseudo heading command
       * i.e. desired heading set-point to attitude control
       * in rad, with #INT32_ANGLE_FRAC                   */
      delfly_guidance.cmd.pseudo_heading = delfly_guidance.sp.heading + pseudo_heading_d;
      /* lateral guidance complementary heading command
       * i.e. heading reference to attitude control
       * in rad, with #INT32_ANGLE_FRAC                   */
      delfly_guidance.cmd.heading = (delfly_guidance.gains.h.complementary_gain*delfly_guidance.cmd.pseudo_heading
                                     + (100 - delfly_guidance.gains.h.complementary_gain)*delfly_state.h.heading)
                                    /100;
    }
    break;

  default:
    {} //nothing to do
  }

}
