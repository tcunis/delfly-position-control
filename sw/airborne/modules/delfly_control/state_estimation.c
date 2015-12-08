/*
 * Copyright (C) 2015 Torbjoern Cunis <t.cunis@tudelft.nl>
 *
 * State estimation submodule filters and estimates delfly state
 * estimation.
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
 * @file /modules/delfly_control/state_estimation.h
 * @author Torbjoern Cunis
 */

#include "delfly_state.h"
#include "state_estimation.h"

#include "delfly_control.h"

#include "matlab_include.h"
#include "delfly_algebra_int.h"

#include "state.h"
#include "subsystems/sensors/rpm_sensor.h"



struct DelflyState delfly_state;

struct StateEstimation state_estimation;


struct DelflyModelCovariance delfly_model_process_covariance;
struct Int32Mat33 state_estimation_noise_covariance;


static void state_estimation_getoutput (void);
static void state_estimation_aftermath (void);


void state_estimation_init (void) {

  delfly_model_init_states( &state_estimation.states );
  delfly_model_init_states( &state_estimation.out );

  VECT3_ZERO( state_estimation.err );

  state_estimation.period = BFP_OF_REAL(STATE_ESTIMATION_RUN_PERIOD, INT32_TIME_FRAC);

  INT32_MAT33_DIAG(state_estimation_noise_covariance,
		  	  	  	  SQUARE(matlab_noise_distribution.x)/(1<<INT32_MATLAB_FRAC),
					  SQUARE(matlab_noise_distribution.y)/(1<<INT32_MATLAB_FRAC),
					  SQUARE(matlab_noise_distribution.z)/(1<<INT32_MATLAB_FRAC)
  );
  delfly_model_assign_covariance( &delfly_model_process_covariance, state_estimation.period, matlab_disturbance_distribution );

  //TODO: use OptiTrack mean error as well
  delfly_model_init_covariance( &state_estimation.covariance.estimate );

  VECT2_ZERO( delfly_state.h.pos );
  VECT2_ZERO( delfly_state.h.vel );
  VECT2_ZERO( delfly_state.h.air );
  VECT2_ZERO( delfly_state.h.acc );
  INT32_ZERO( delfly_state.h.speed_air );
  INT32_ZERO( delfly_state.h.speed_vel );
  INT32_ZERO( delfly_state.h.heading );
  INT32_ZERO( delfly_state.h.azimuth );
  INT32_ZERO( delfly_state.h.head_rate );

  INT32_ZERO( delfly_state.v.pos );
  INT32_ZERO( delfly_state.v.vel );
  INT32_ZERO( delfly_state.v.air );
  INT32_ZERO( delfly_state.v.acc );

  VECT2_ZERO( delfly_state.fv.vel.xy );
  VECT2_ZERO( delfly_state.fv.air.xy );
  VECT2_ZERO( delfly_state.fv.acc.xy );

  INT32_ZERO( delfly_state.flap_freq );
}


void state_estimation_getoutput (void) {

  VECT3_COPY(state_estimation.out.pos, *stateGetPositionNed_i());
  VECT3_COPY(state_estimation.out.vel, *stateGetSpeedNed_i());
  VECT3_COPY(state_estimation.out.acc, *stateGetAccelNed_i());

  delfly_model_assign_eulers( &state_estimation.out,
		  	  	  	  	  	  	  *stateGetNedToBodyEulers_i(),
								  *stateGetBodyRates_i()
  );
}


void state_estimation_enter (void) {

  state_estimation_getoutput();

  delfly_model_assign_states( &state_estimation.states,
		  	  	  	  	  	  	  state_estimation.out.pos,
								  state_estimation.out.vel,
								  state_estimation.out.acc
  );
  delfly_model_assign_eulers( &state_estimation.states,
		  	  	  	  	  	  	  state_estimation.out.att,
								  state_estimation.out.rot
  );
}


static inline void state_estimation_update_covariance ( struct DelflyModelCovariance* cov, struct StateEstimationGain gain ) {

  struct DelflyModelCovariance covK;
  MAT33_MULT2(covK.pos_pos, gain.pos_err, cov->pos_pos, -1);
  MAT33_MULT2(covK.pos_vel, gain.pos_err, cov->pos_vel, -1);
  MAT33_MULT2(covK.pos_acc, gain.pos_err, cov->pos_acc, -1);
  MAT33_MULT2(covK.vel_pos, gain.vel_err, cov->pos_pos, -1);
  MAT33_MULT2(covK.vel_vel, gain.vel_err, cov->pos_vel, -1);
  MAT33_MULT2(covK.vel_acc, gain.vel_err, cov->pos_acc, -1);
  MAT33_MULT2(covK.acc_pos, gain.acc_err, cov->pos_pos, -1);
  MAT33_MULT2(covK.acc_vel, gain.acc_err, cov->pos_vel, -1);
  MAT33_MULT2(covK.acc_acc, gain.acc_err, cov->pos_acc, -1);

  MAT33_ADD(covK.pos_pos, cov->pos_pos);
  MAT33_ADD(covK.pos_pos, cov->vel_pos);
  MAT33_ADD(covK.pos_pos, cov->acc_pos);
  MAT33_ADD(covK.vel_pos, cov->pos_pos);
  MAT33_ADD(covK.vel_pos, cov->vel_pos);
  MAT33_ADD(covK.vel_pos, cov->acc_pos);
  MAT33_ADD(covK.acc_pos, cov->pos_pos);
  MAT33_ADD(covK.acc_pos, cov->vel_pos);
  MAT33_ADD(covK.acc_pos, cov->acc_pos);

  MAT33_ADD(covK.pos_vel, cov->pos_vel);
  MAT33_ADD(covK.pos_vel, cov->vel_vel);
  MAT33_ADD(covK.pos_vel, cov->acc_vel);
  MAT33_ADD(covK.vel_vel, cov->pos_vel);
  MAT33_ADD(covK.vel_vel, cov->vel_vel);
  MAT33_ADD(covK.vel_vel, cov->acc_vel);
  MAT33_ADD(covK.acc_vel, cov->pos_vel);
  MAT33_ADD(covK.acc_vel, cov->vel_vel);
  MAT33_ADD(covK.acc_vel, cov->acc_vel);

  MAT33_ADD(covK.pos_acc, cov->pos_acc);
  MAT33_ADD(covK.pos_acc, cov->vel_acc);
  MAT33_ADD(covK.pos_acc, cov->acc_acc);
  MAT33_ADD(covK.vel_acc, cov->pos_acc);
  MAT33_ADD(covK.vel_acc, cov->vel_acc);
  MAT33_ADD(covK.vel_acc, cov->acc_acc);
  MAT33_ADD(covK.acc_acc, cov->pos_acc);
  MAT33_ADD(covK.acc_acc, cov->vel_acc);
  MAT33_ADD(covK.acc_acc, cov->acc_acc);

  MAT33_COPY(cov->pos_pos, covK.pos_pos);
  MAT33_COPY(cov->pos_vel, covK.pos_vel);
  MAT33_COPY(cov->pos_acc, covK.pos_acc);
  MAT33_COPY(cov->vel_pos, covK.vel_pos);
  MAT33_COPY(cov->vel_vel, covK.vel_vel);
  MAT33_COPY(cov->vel_acc, covK.vel_acc);
  MAT33_COPY(cov->acc_pos, covK.acc_pos);
  MAT33_COPY(cov->acc_vel, covK.acc_vel);
  MAT33_COPY(cov->acc_acc, covK.acc_acc);
}


void state_estimation_run (void) {

  struct Int32Mat33 residual_inv;
  struct Int32Vect3 offset_pos, offset_vel, offset_acc;

  MAT33_ZERO( residual_inv );

  state_estimation_getoutput();

  /* Kalman filter -- predict */
  // a-priori state estimate 			x_(k|k-1) = A x_(k-1|k-1)
  delfly_model_update_states( &state_estimation.states, state_estimation.period );
  // a-priori estimate co-variance		P_(k|k-1) = A P_(k-1|k-1) A' + Q
  delfly_model_update_covariance( &state_estimation.covariance.estimate, state_estimation.period );

  /* Kalman filter -- calculate gain */
  // measurement residual				y_k = z_k - H x_(k|k-1)
  VECT3_DIFF( state_estimation.err, state_estimation.out.pos, state_estimation.states.pos );
  // residual co-variance				S_k = H P_(k|k-1) H' + R
  MAT33_COPY( state_estimation.covariance.residual, state_estimation.covariance.estimate.pos_pos );
  MAT33_ADD( state_estimation.covariance.residual, state_estimation_noise_covariance );
  // optimal Kalman gain				K_k = P_(k|k-1) H' inv(S_k)
  MAT33_INV( residual_inv, state_estimation.covariance.residual );
  MAT33_MULT( state_estimation.gain.pos_err, state_estimation.covariance.estimate.pos_pos, residual_inv );
  MAT33_MULT( state_estimation.gain.vel_err, state_estimation.covariance.estimate.vel_pos, residual_inv );
  MAT33_MULT( state_estimation.gain.acc_err, state_estimation.covariance.estimate.acc_pos, residual_inv );

  /* Kalman filter -- update */
  // update states
  MAT33_VECT3_MUL( offset_pos, state_estimation.gain.pos_err, state_estimation.err );
  MAT33_VECT3_MUL( offset_vel, state_estimation.gain.vel_err, state_estimation.err );
  MAT33_VECT3_MUL( offset_acc, state_estimation.gain.acc_err, state_estimation.err );
  delfly_model_add_states( &state_estimation.states, offset_pos, offset_vel, offset_acc );
  // update estimate co-variance
  state_estimation_update_covariance( &state_estimation.covariance.estimate, state_estimation.gain );

  /* aftermath */
  delfly_model_assign_eulers( &state_estimation.states, state_estimation.out.att, state_estimation.out.rot );

  state_estimation_aftermath();
}


void state_estimation_aftermath (void) {

  struct Int32Vect3 pos, vel, acc;
  VECT3_COPY(pos, state_estimation.states.pos);
  VECT3_COPY(vel, state_estimation.states.vel);
  VECT3_COPY(acc, state_estimation.states.acc);

  set_position( &pos );
  set_velocity( &vel );
  //TODO: get airspeed
  set_airspeed( &vel );
  set_acceleration( &acc );

  delfly_state.flap_freq = rpm_sensor.motor_frequency;

  //TODO: get heading, azimuth
  delfly_state.h.heading = state_estimation.states.att.psi;
  delfly_state.h.azimuth = stateGetHorizontalSpeedDir_i();
  //TODO: get heading rate

  set_speed_vel( int32_vect2_norm(&delfly_state.h.vel) );
  set_speed_air( int32_vect2_norm(&delfly_state.h.air) );
  set_speed_acc( int32_vect2_norm(&delfly_state.h.acc) );
}
