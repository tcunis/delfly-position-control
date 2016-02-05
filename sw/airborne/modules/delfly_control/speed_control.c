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
#include "state_estimation.h"

#include "delfly_guidance.h"

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

  //default speed-control type
  speed_control.type = SPEED_CONTROL_TYPE_THROTTLE;

  //default speed-control mode
  speed_control.control_mode = SPEED_CONTROL_MODE_CONTROL;

  VECT2_ZERO(speed_control.sp.acceleration.xy);
  speed_control.sp.heading = 0;

  speed_control.fb_gains.p.fwd = SPEED_CONTROL_FB_FWD_PGAIN;
  speed_control.fb_gains.p.ver = SPEED_CONTROL_FB_VER_PGAIN;
  speed_control.fb_gains.i.fwd = SPEED_CONTROL_FB_FWD_IGAIN;
  speed_control.fb_gains.i.ver = SPEED_CONTROL_FB_VER_IGAIN;
  speed_control.fb_gains.i2    = SPEED_CONTROL_FB_I2GAIN;

  speed_control.ff_gains.pitch    = SPEED_CONTROL_FF_PITCH_GAIN;
  speed_control.ff_gains.throttle = SPEED_CONTROL_FF_THROTTLE_GAIN;

  speed_control.fb_gains.adapt.fwd = SPEED_CONTROL_ADAPT_FWD_GAIN;
  speed_control.fb_gains.adapt.ver = SPEED_CONTROL_ADAPT_VER_GAIN;

  speed_control_set_pitch_offset(SPEED_CONTROL_PITCH_OFFSET);

  VECT2_ZERO(speed_control_var.ref.velocity.xy);
  VECT2_ZERO(speed_control_var.ref.acceleration.xy);

  VECT2_ZERO(speed_control_var.now.acceleration.xy);
  VECT2_ZERO(speed_control_var.now.velocity.xy);
  VECT2_ZERO(speed_control_var.now.position.xy);

  VECT2_ZERO(speed_control_var.err.acceleration.xy);
  VECT2_ZERO(speed_control_var.err.velocity.xy);
  VECT2_ZERO(speed_control_var.err.position.xy);

  speed_control_var.adapt.Xi.fwd = (1<<INT32_MATLAB_FRAC);
  speed_control_var.adapt.Xi.ver = (1<<INT32_MATLAB_FRAC);

  VECT2_ZERO(speed_control_var.cmd.acceleration.xy);
  speed_control_var.cmd.pitch = 0;
  speed_control_var.cmd.throttle = 0;

  VECT2_ZERO(speed_control_var.ff_cmd.acceleration.xy);
  speed_control_var.ff_cmd.pitch = 0;
  speed_control_var.ff_cmd.throttle = 0;

  VECT2_ZERO(speed_control_var.fb_cmd.acceleration.xy);
  speed_control_var.fb_cmd.pitch = 0;
  speed_control_var.fb_cmd.throttle = 0;

  delfly_model_init();
}

void speed_control_set_cmd_h( int32_t cmd_h_acceleration, int32_t cmd_heading ) {

  // saturate acceleration command by maximal acceleration
  speed_control.sp.acceleration.fv.fwd = INT32_SAT(cmd_h_acceleration, STATE_ESTIMATION_ACC_MAX);
//  speed_control.sp.acceleration.fv.fwd = cmd_h_acceleration;
  speed_control.sp.heading = cmd_heading;
}

void speed_control_set_cmd_v( int32_t cmd_v_acceleration ) {

  speed_control.sp.acceleration.fv.ver = INT32_SAT(cmd_v_acceleration, STATE_ESTIMATION_ACC_MAX);
//  speed_control.sp.acceleration.fv.ver = cmd_v_acceleration;
}

void speed_control_set_pitch_offset( int32_t pitch_offset_deg ) {
  speed_control.pitch_offset = INT32_RAD_OF_DEG(ANGLE_BFP_OF_REAL(pitch_offset_deg));
}


static inline void speed_control_gain_scheduling (int32_t air_speed) {

  int32_t max_v04 = (matlab_airspeed_v04 + matlab_airspeed_v08)*(1<<(INT32_SPEED_FRAC-INT32_MATLAB_FRAC))/2,
          max_v08 = (matlab_airspeed_v08 + matlab_airspeed_v12)*(1<<(INT32_SPEED_FRAC-INT32_MATLAB_FRAC))/2,
          max_v12 = (matlab_airspeed_v12 + matlab_airspeed_v25)*(1<<(INT32_SPEED_FRAC-INT32_MATLAB_FRAC))/2,
          max_v25 = (matlab_airspeed_v25 + matlab_airspeed_v50)*(1<<(INT32_SPEED_FRAC-INT32_MATLAB_FRAC))/2;

  if ( air_speed < max_v04 ) {
    speed_control_var.eq.air_speed           = matlab_airspeed_v04;
    VECT2_COPY(speed_control_var.mat.pitch,    matlab_pitch_matrix_v04);
    VECT2_COPY(speed_control_var.mat.throttle, matlab_throttle_matrix_v04);
    VECT2_COPY(speed_control_var.mat.flapfreq, matlab_flapfreq_matrix_v04);
    speed_control_var.eq.pitch               = matlab_pitch_equilibrium_v04;     // pitch at eq in rad, with #INT32_MATLAB_FRAC
    speed_control_var.eq.throttle            = matlab_throttle_equilibrium_v04;  // throttle at eq in %, with #INT32_MATLAB_FRAC
    speed_control_var.eq.flapfreq            = matlab_flapfreq_equilibrium_v04;
  } else if ( air_speed <= max_v08 ) {
    speed_control_var.eq.air_speed           = matlab_airspeed_v08;
    VECT2_COPY(speed_control_var.mat.pitch,    matlab_pitch_matrix_v08);
    VECT2_COPY(speed_control_var.mat.throttle, matlab_throttle_matrix_v08);
    VECT2_COPY(speed_control_var.mat.flapfreq, matlab_flapfreq_matrix_v08);
    speed_control_var.eq.pitch               = matlab_pitch_equilibrium_v08;     // pitch at eq in rad, with #INT32_MATLAB_FRAC
    speed_control_var.eq.throttle            = matlab_throttle_equilibrium_v08;  // throttle at eq in %, with #INT32_MATLAB_FRAC
    speed_control_var.eq.flapfreq            = matlab_flapfreq_equilibrium_v08;
  } else if ( air_speed <= max_v12 ) {
    speed_control_var.eq.air_speed           = matlab_airspeed_v12;
    VECT2_COPY(speed_control_var.mat.pitch,    matlab_pitch_matrix_v12);
    VECT2_COPY(speed_control_var.mat.throttle, matlab_throttle_matrix_v12);
    VECT2_COPY(speed_control_var.mat.flapfreq, matlab_flapfreq_matrix_v12);
    speed_control_var.eq.pitch               = matlab_pitch_equilibrium_v12;     // pitch at eq in rad, with #INT32_MATLAB_FRAC
    speed_control_var.eq.throttle            = matlab_throttle_equilibrium_v12;  // throttle at eq in %, with #INT32_MATLAB_FRAC
    speed_control_var.eq.flapfreq            = matlab_flapfreq_equilibrium_v12;
  } else if ( air_speed <= max_v25 ) {
    speed_control_var.eq.air_speed           = matlab_airspeed_v25;
    VECT2_COPY(speed_control_var.mat.pitch,    matlab_pitch_matrix_v25);
    VECT2_COPY(speed_control_var.mat.throttle, matlab_throttle_matrix_v25);
    VECT2_COPY(speed_control_var.mat.flapfreq, matlab_flapfreq_matrix_v25);
    speed_control_var.eq.pitch               = matlab_pitch_equilibrium_v25;     // pitch at eq in rad, with #INT32_MATLAB_FRAC
    speed_control_var.eq.throttle            = matlab_throttle_equilibrium_v25;  // throttle at eq in %, with #INT32_MATLAB_FRAC
    speed_control_var.eq.flapfreq            = matlab_flapfreq_equilibrium_v25;
  } else {
    speed_control_var.eq.air_speed           = matlab_airspeed_v50;
    VECT2_COPY(speed_control_var.mat.pitch,    matlab_pitch_matrix_v50);
    VECT2_COPY(speed_control_var.mat.throttle, matlab_throttle_matrix_v50);
    VECT2_COPY(speed_control_var.mat.flapfreq, matlab_flapfreq_matrix_v50);
    speed_control_var.eq.pitch               = matlab_pitch_equilibrium_v50;     // pitch at eq in rad, with #INT32_MATLAB_FRAC
    speed_control_var.eq.throttle            = matlab_throttle_equilibrium_v50;  // throttle at eq in %, with #INT32_MATLAB_FRAC
    speed_control_var.eq.flapfreq            = matlab_flapfreq_equilibrium_v50;
  }
}

static inline void speed_control_set_airspeed (int32_t air_speed) {
  if ( speed_control_var.now.air_speed == air_speed ) {
    return; //nothing to do
  }

  speed_control_gain_scheduling(air_speed);
  speed_control_var.now.air_speed = air_speed;
}

void speed_control_enter (void) {

  speed_control.mode = SPEED_CONTROL_MODE_ENTER;

  VECT2_ZERO(speed_control_var.err.velocity.xy);
  VECT2_ZERO(speed_control_var.err.position.xy);

  //initial velocity ref: v_ref(0) = v0
  VECT2_COPY(speed_control_var.ref.velocity.xy, delfly_state.fv.air.xy);
  VECT2_COPY(speed_control_var.now.velocity.xy, delfly_state.fv.air.xy);

  speed_control_var.ref.position.fv.fwd = VECT2_GET_FWD(delfly_state.h.pos, delfly_guidance.sp.heading);
  speed_control_var.ref.position.fv.ver = delfly_state.v.pos;

  flap_control_enter();

  delfly_model_enter();
  delfly_model_set_cmd( 0, 0 );

  speed_control_gain_scheduling(speed_control_var.now.air_speed);

  /* TODO: gain scheduling w.r.t. air speed */
//  VECT2_COPY(speed_control_var.mat.pitch, matlab_pitch_matrix_v08);
//  VECT2_COPY(speed_control_var.mat.throttle, matlab_throttle_matrix_v08);
//  VECT2_COPY(speed_control_var.mat.flapfreq, matlab_flapfreq_matrix_v08);
//  speed_control_var.eq.pitch 	= matlab_pitch_equilibrium_v08;     // pitch at eq in rad, with #INT32_MATLAB_FRAC
//  speed_control_var.eq.throttle = matlab_throttle_equilibrium_v08;  // throttle at eq in %, with #INT32_MATLAB_FRAC
//  speed_control_var.eq.flapfreq = matlab_flapfreq_equilibrium_v08;
}


void speed_control_estimate_error (void) {

  union Int32VectLong last_velocity_ref, last_velocity_now;
  VECT2_COPY( last_velocity_ref.xy, speed_control_var.ref.velocity.xy );
  VECT2_COPY( last_velocity_now.xy, speed_control_var.now.velocity.xy );

  VECT2_COPY(speed_control_var.now.velocity.xy, delfly_state.fv.air.xy);
  VECT2_COPY(speed_control_var.now.acceleration.xy, delfly_state.fv.acc.xy);

  switch (speed_control.control_mode) {

  /* speed-control adaption mode: adapt to actual equilibrium.
   * ff_cmd = 0, fb_cmd = i2*(p_ref(k) - p(k))
   *
   * where p_ref(k) = p(0) f.a. k
   *
   * let v_ref = v(k), a_ref = a(k)
   * i.e. v_err = a_err = 0                                                   */
  case SPEED_CONTROL_MODE_ADAPT:
    {
      VECT2_ZERO( speed_control_var.ff_cmd.acceleration.xy );

      speed_control_var.now.position.fv.fwd = VECT2_GET_FWD(delfly_state.h.pos, delfly_guidance.sp.heading);
      speed_control_var.now.position.fv.ver = delfly_state.v.pos;

//      VECT2_ADD_SCALED2(speed_control_var.ref.position.xy,
//                       speed_control_var.ref.velocity.xy,
//                       SPEED_CONTROL_RUN_PERIOD, (1<<(INT32_SPEED_FRAC-INT32_POS_FRAC)));
////      VECT2_COPY(speed_control_var.ref.position.xy,         delfly_model.states.pos_fv.xy);

      VECT2_COPY(speed_control_var.ref.velocity.xy,     speed_control_var.now.velocity.xy);
      VECT2_COPY(speed_control_var.ref.acceleration.xy, speed_control_var.now.acceleration.xy);

      VECT2_DIFF(speed_control_var.err.position.xy, speed_control_var.ref.position.xy, speed_control_var.now.position.xy);
    }
    break;

  /* speed-control control mode:
   * ff_cmd = a_sp, fb_cmd = p*(a_ref(k) - a(k)) + i*(v_ref(k) - v(k)) + adapt_cmd
   *
   * where a_ref(k) = a_sp(k-1), v_ref(k) = v_ref(k-1) + dt*a_sp(k-1),
   *       v_ref(0) = v(0)
   *
   * and adapt_cmd = i2*p_err                                                   */
  case SPEED_CONTROL_MODE_CONTROL:
    {
      //update velocity ref: v_ref(k) = v_ref(k-1) + T*a_cmd(k-1)
      //TODO: use reference model!
      VECT2_ADD_SCALED(speed_control_var.ref.velocity.xy, speed_control_var.ref.acceleration.xy, SPEED_CONTROL_RUN_PERIOD*(1<<(INT32_SPEED_FRAC-INT32_ACCEL_FRAC)));

//      VECT2_COPY(speed_control_var.ref.velocity.xy,     delfly_model.states.vel_fv.xy);
//      VECT2_COPY(speed_control_var.ref.acceleration.xy, delfly_model.states.acc_fv.xy);

//      //velocity error: v_err = v_ref - v_now
//      union Int32VectLong velocity_error_new;
//      VECT2_DIFF(velocity_error_new.xy, speed_control_var.ref.velocity.xy, speed_control_var.now.velocity.xy);
//
//      //estimated acceleration error: a_err(k-1) = (v_err(k) - v_err(k-1))/T
//      VECT2_DIFF_SCALED2(speed_control_var.err.acceleration.xy,
//                    velocity_error_new.xy,
//              speed_control_var.err.velocity.xy,
//              SPEED_CONTROL_RUN_FREQ,(1<<(INT32_SPEED_FRAC-INT32_ACCEL_FRAC)));
//      VECT2_COPY(speed_control_var.err.velocity.xy, velocity_error_new.xy);

//      //for telemetry: a_now = a_ref - a_err
//      VECT2_DIFF(speed_control_var.now.acceleration.xy,
//    		  	 speed_control.sp.acceleration.xy,
//    			 speed_control_var.err.acceleration.xy);
    }
    /* no break */
  default:
    VECT2_COPY(speed_control_var.ff_cmd.acceleration.xy, speed_control.sp.acceleration.xy);
    break;
  }

  VECT2_DIFF(speed_control_var.ref.velocity_diff.xy, speed_control_var.ref.velocity.xy,     last_velocity_ref.xy);
  VECT2_DIFF(speed_control_var.now.velocity_diff.xy, speed_control_var.now.velocity.xy,     last_velocity_now.xy);

  VECT2_DIFF(speed_control_var.err.velocity.xy,     speed_control_var.ref.velocity.xy,     speed_control_var.now.velocity.xy);
  VECT2_DIFF(speed_control_var.err.acceleration.xy, speed_control_var.ref.acceleration.xy, speed_control_var.now.acceleration.xy);

  VECT2_DIFF(speed_control_var.err.velocity_diff.xy, speed_control_var.ref.velocity_diff.xy, speed_control_var.now.velocity_diff.xy);
}


void speed_control_run (bool_t in_flight) {

//  speed_control_var.now.air_speed = delfly_state.h.speed_air;
  speed_control_set_airspeed( delfly_state.h.speed_wind /*delfly_state.h.speed_air*/ );

  if (!in_flight || guidance_v_mode != GUIDANCE_V_MODE_MODULE)
	  return speed_control_enter(); //nothing to do

  //else:
  speed_control.mode = speed_control.control_mode;

  speed_control_estimate_error();

  /* feed-forward */
//  VECT2_COPY(speed_control_var.ff_cmd.acceleration.xy, speed_control.sp.acceleration.xy);

  /* feed-back */
  VECT2_ZERO(speed_control_var.fb_cmd.acceleration.xy);
  union Int32VectLong fb_p_cmd, fb_i_cmd, fb_i2_cmd;

  switch (speed_control.type)
  {
  default:
  case SPEED_CONTROL_TYPE_THROTTLE:
  case SPEED_CONTROL_TYPE_FLAPFREQ:
    {
      //if p = 100 %, 1 m/s2 acceleration error equals to +1 m/s2 acceleration command
      fb_p_cmd.fv.fwd = speed_control.fb_gains.p.fwd*speed_control_var.err.acceleration.fv.fwd/100;
      fb_p_cmd.fv.ver = speed_control.fb_gains.p.ver*speed_control_var.err.acceleration.fv.ver/100;
    //  VECT2_ADD_SCALED2( speed_control_var.fb_cmd.acceleration.xy,
    //                     speed_control_var.err.acceleration.xy,
    //                     speed_control.fb_gains.p, 100 );
      VECT2_ADD( speed_control_var.fb_cmd.acceleration.xy, fb_p_cmd.xy );

      //if i = 100 %, 1 m/s velocity error equals to +1 m/s2 acceleration command
      fb_i_cmd.fv.fwd = speed_control.fb_gains.i.fwd*speed_control_var.err.velocity.fv.fwd/(100<<(INT32_SPEED_FRAC-INT32_ACCEL_FRAC));
      fb_i_cmd.fv.ver = speed_control.fb_gains.i.ver*speed_control_var.err.velocity.fv.ver/(100<<(INT32_SPEED_FRAC-INT32_ACCEL_FRAC));
    //  VECT2_ADD_SCALED2( speed_control_var.fb_cmd.acceleration.xy,
    //		  	  	         speed_control_var.err.velocity.xy,
    //		  	  	         speed_control.fb_gains.i,
    //		  	  	         (100<<(INT32_SPEED_FRAC-INT32_ACCEL_FRAC)) );
      VECT2_ADD( speed_control_var.fb_cmd.acceleration.xy, fb_i_cmd.xy );
    } break;

  case SPEED_CONTROL_TYPE_ADAPTIVE:
    {
      if ( !VECT2_EQUALS_ZERO(speed_control_var.ref.acceleration.xy) ) {
        /* adapt ratio Xi if not saturated
         * dXi/dt = sign(v_ref)*(gamma/100)*(v_ref - v_now)
         * where gamma is the adaption gain in percent. */
        struct Int32VectL dXi;
        dXi.fwd = INT32_SIGN(speed_control_var.ref.velocity.fv.fwd)*speed_control.fb_gains.adapt.fwd*speed_control_var.err.velocity.fv.fwd/(100*(1<<(INT32_SPEED_FRAC-INT32_MATLAB_FRAC)));
        dXi.ver = INT32_SIGN(speed_control_var.ref.velocity.fv.ver)*speed_control.fb_gains.adapt.ver*speed_control_var.err.velocity.fv.ver/(100*(1<<(INT32_SPEED_FRAC-INT32_MATLAB_FRAC)));

        speed_control_var.adapt.Xi.fwd += dXi.fwd*SPEED_CONTROL_RUN_PERIOD;
        speed_control_var.adapt.Xi.ver += dXi.ver*SPEED_CONTROL_RUN_PERIOD;
      }

      speed_control_var.ff_cmd.acceleration.fv.fwd = speed_control_var.ff_cmd.acceleration.fv.fwd*speed_control_var.adapt.Xi.fwd/(1<<INT32_MATLAB_FRAC);
      speed_control_var.ff_cmd.acceleration.fv.ver = speed_control_var.ff_cmd.acceleration.fv.ver*speed_control_var.adapt.Xi.ver/(1<<INT32_MATLAB_FRAC);
    } break;
  }

  //if i2 = 100 %, 1 m position error equals to +1 m/s2 acceleration command
  fb_i2_cmd.fv.fwd = speed_control.fb_gains.i2*speed_control_var.err.position.fv.fwd/(100<<(INT32_POS_FRAC-INT32_ACCEL_FRAC));
  fb_i2_cmd.fv.ver = speed_control.fb_gains.i2*speed_control_var.err.position.fv.ver/(100<<(INT32_POS_FRAC-INT32_ACCEL_FRAC));
//  VECT2_ADD_SCALED2( speed_control_var.fb_cmd.acceleration.xy,
//                     speed_control_var.err.position.xy,
//                     speed_control.fb_gains.i2,
//                     100*(1<<(INT32_POS_FRAC-INT32_ACCEL_FRAC)) );
  VECT2_ADD( speed_control_var.fb_cmd.acceleration.xy, fb_i2_cmd.xy );


  /* pitch and throttle command */
  //cmd = ff_cmd + fb_cmd
  VECT2_SUM(speed_control_var.cmd.acceleration.xy,
		        speed_control_var.ff_cmd.acceleration.xy,
		        speed_control_var.fb_cmd.acceleration.xy);

  speed_control_calculate_cmd(&speed_control_var.cmd, &speed_control_var.eq, &speed_control_var.mat);

  int32_t cmd_pitch       = INT32_SAT_2(speed_control_var.cmd.pitch, 0, INT32_RAD_OF_DEG(ANGLE_BFP_OF_REAL(75)));
  int32_t cmd_throttle    = TRIM_UPPRZ(speed_control_var.cmd.throttle);

  //for telemetry:
  speed_control_calculate_cmd(&speed_control_var.ff_cmd, &speed_control_eq_zero, &speed_control_var.mat);
  speed_control_calculate_cmd(&speed_control_var.fb_cmd, &speed_control_eq_zero, &speed_control_var.mat);
  speed_control_var.fb_cmd.throttle = TRIM_UPPRZ(speed_control_var.fb_cmd.throttle);

  static struct Int32Eulers orientation_cmd;
  orientation_cmd.phi   = 0;
  orientation_cmd.theta = cmd_pitch - speed_control.pitch_offset;
  orientation_cmd.psi   = speed_control.sp.heading;

  stabilization_attitude_set_rpy_setpoint_i( &orientation_cmd );
  switch (speed_control.type)
  {
  case SPEED_CONTROL_TYPE_FLAPFREQ:
    {
      flap_control_set(speed_control_var.cmd.flapfreq);
      flap_control_run();
    }
    break;
  case SPEED_CONTROL_TYPE_THROTTLE:
  default:
    stabilization_cmd[COMMAND_THRUST] = cmd_throttle;
    break;
  }
  stabilization_attitude_run(in_flight);

  if ( ( (speed_control.mode != SPEED_CONTROL_TYPE_FLAPFREQ && cmd_throttle == speed_control_var.cmd.throttle)
          || (speed_control.mode == SPEED_CONTROL_TYPE_FLAPFREQ && flap_control_reachable()) )
        && cmd_pitch == speed_control_var.cmd.pitch )                   {
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
	//commanded flapping frequency in Hz
	cmd->flapfreq = (eq->flapfreq * (1 << INT32_ACCEL_FRAC)
	                   + VECT2_DOT_PRODUCT(mat->flapfreq, cmd->acceleration.xy))
	                / (1 << (INT32_ACCEL_FRAC));
}
