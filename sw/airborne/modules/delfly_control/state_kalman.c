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
 * @file /modules/delfly_control/state_kalman.h
 * @author Torbjoern Cunis
 */

#include "delfly_state.h"

#include "delfly_control.h"

#include "generated/airframe.h"

#include "matlab_include.h"
#include "delfly_algebra_int.h"

#include "state.h"
#include "subsystems/sensors/rpm_sensor.h"

#ifndef INS_INT_IMU_ID
#define INS_INT_IMU_ID ABI_BROADCAST
#endif


struct StateKalmanCovariances {
  /* state estimation co-variance matrix
   * with #INT32_MATLAB_FRAC              */
  struct DelflyModelCovariance estimate;

  /* residual co-variance matrix
   * with #INT32_MATLAB_FRAC              */
  struct Int32Mat33 residual;

  /* inverted co-variance matrix
   * with #INT32_MATLAB_FRAC
   * <for debug>                          */
  struct Int32Mat33 residual_inv;
};

struct StateKalmanGain {
  struct Int32Mat33 pos_err;
  struct Int32Mat33 vel_err;
  struct Int32Mat33 acc_err;
};


struct StateKalmanFilter {

  /* predicted states
   * in m,    with #INT32_POS_FRAC
   * in m/s,  with #INT32_SPEED_FRAC
   * in m/s2, with #INT32_ACCEL_FRAC
   */
  struct DelflyModelStates states;

  /* measured states
   * in m,    with #INT32_POS_FRAC
   * in m/s,  with #INT32_SPEED_FRAC
   * in m/s2, with #INT32_ACCEL_FRAC
   */
  struct DelflyModelStates out;

  struct StateKalmanCovariances covariance;

  /* Kalman filter gain matrix
   * with #INT32_MATLAB_FRAC      */
  struct StateKalmanGain gain;

  /* position measurement residual
   * in m, with #INT32_POS_FRAC 	*/
  struct Int32Vect3 res;

  /* filter period
    * in s, with #INT32_TIME_FRAC  */
   int32_t period;
};

struct StateKalmanFilter state_kalman;


struct DelflyModelCovariance delfly_model_process_covariance;

/* measurement noise co-variance matrix
 * with #INT32_MATLAB_FRAC                */
struct Int32Mat33 state_kalman_noise_covariance;


static void state_kalman_init (void);
static void state_kalman_enter (void);
static void state_kalman_getoutput (void);
static void state_kalman_aftermath (void);

static void state_kalman_run (void);


void state_kalman_init (void) {

  delfly_model_init_states( &state_kalman.states );
  delfly_model_init_states( &state_kalman.out );

  VECT3_ZERO( state_kalman.res );

  state_kalman.period = BFP_OF_REAL(STATE_ESTIMATION_RUN_PERIOD, INT32_TIME_FRAC);

  INT32_MAT33_DIAG(state_kalman_noise_covariance,
		  	  	  	    SQUARE(matlab_noise_distribution.x)/(1<<INT32_MATLAB_FRAC),
		  	  	  	      SQUARE(matlab_noise_distribution.y)/(1<<INT32_MATLAB_FRAC),
		  	  	  	        SQUARE(matlab_noise_distribution.z)/(1<<INT32_MATLAB_FRAC)
  );
  delfly_model_assign_covariance( &delfly_model_process_covariance, state_kalman.period, matlab_disturbance_distribution );

  //TODO: use OptiTrack mean error as well
  delfly_model_init_covariance( &state_kalman.covariance.estimate );
}


void state_kalman_getoutput (void) {

  VECT3_COPY(state_kalman.out.pos, *stateGetPositionNed_i());
  VECT3_COPY(state_kalman.out.vel, *stateGetSpeedNed_i());
  //VECT3_COPY(state_kalman.out.acc, *stateGetAccelNed_i());

  delfly_model_assign_eulers( &state_kalman.out,
		  	  	  	  	  	  	  *stateGetNedToBodyEulers_i(),
		  	  	  	  	  	  	  *stateGetBodyRates_i()
  );
}


void state_kalman_enter (void) {

  state_kalman_getoutput();

  struct Int32Vect3 pos, vel, acc;


  VECT3_COPY(pos, state_kalman.out.pos);

  VECT3_ZERO(vel);
  VECT3_ZERO(acc);


  delfly_model_assign_states( &state_kalman.states, pos, vel, acc );
  delfly_model_assign_eulers( &state_kalman.states,
		  	  	  	  	  	  	  state_kalman.out.att,
		  	  	  	  	  	  	  state_kalman.out.rot
  );
}



void state_kalman_run (void) {

  state_kalman_getoutput();

  /* residual co-variance matrix inverse
   * with #INT32_MATLAB_FRAC              */
  // <for debug>: use state_kalman.covariance.residual_inv
  //struct Int32Mat33 residual_inv;

  /* position estimation offset
   * with #INT32_POS_FRAC                 */
  struct Int32Vect3 offset_pos;
  /* velocity estimation offset
   * with #INT32_SPEED_FRAC                 */
  struct Int32Vect3 offset_vel;
  /* acceleration estimation offset
	 * with #INT32_ACCEL_FRAC                 */
  struct Int32Vect3 offset_acc;

  MAT33_ZERO( state_kalman.covariance.residual_inv );

  /* Kalman filter -- predict */
  // a-priori state estimate 			  x_(k|k-1) = A x_(k-1|k-1)           -- with states fracs
  delfly_model_predict_states( &state_kalman.states, state_kalman.period );
  // a-priori estimate co-variance	P_(k|k-1) = A P_(k-1|k-1) A' + Q    -- with #INT32_MATLAB_FRAC
  delfly_model_predict_covariance( &state_kalman.covariance.estimate, state_kalman.period );

  /* Kalman filter -- calculate gain */
  // measurement residual				    y_k = z_k - H x_(k|k-1)             -- with #INT32_POS_FRAC
  VECT3_DIFF( state_kalman.res, state_kalman.out.pos, state_kalman.states.pos );
  // residual co-variance				    S_k = H P_(k|k-1) H' + R
  //                                    =  P11_(k|k-1)   + R            -- with #INT32_MATLAB_FRAC
  MAT33_COPY( state_kalman.covariance.residual, state_kalman.covariance.estimate.pos_pos );
  MAT33_ADD(  state_kalman.covariance.residual, state_kalman_noise_covariance );
  // optimal Kalman gain				    K_k = P_(k|k-1) H' inv(S_k)         -- with #INT32_MATLAB_FRAC
  INT32_MAT33_INV( state_kalman.covariance.residual_inv, state_kalman.covariance.residual, INT32_MATLAB_FRAC );
  MAT33_MULT2( state_kalman.gain.pos_err, state_kalman.covariance.estimate.pos_pos, state_kalman.covariance.residual_inv, 1, (1<<INT32_MATLAB_FRAC) );
  MAT33_MULT2( state_kalman.gain.vel_err, state_kalman.covariance.estimate.vel_pos, state_kalman.covariance.residual_inv, 1, (1<<INT32_MATLAB_FRAC) );
  MAT33_MULT2( state_kalman.gain.acc_err, state_kalman.covariance.estimate.acc_pos, state_kalman.covariance.residual_inv, 1, (1<<INT32_MATLAB_FRAC) );

  /* Kalman filter -- update */
  // update states                  x_(k|k) = x_(k|k-1) + K_k y_k       -- with states fracs
  MAT33_VECT3_MULT2( offset_pos, state_kalman.gain.pos_err, state_kalman.res, 1, (1<<(INT32_MATLAB_FRAC)) );
  MAT33_VECT3_MULT2( offset_vel, state_kalman.gain.vel_err, state_kalman.res, 1, (1<<(INT32_MATLAB_FRAC+INT32_POS_FRAC-INT32_SPEED_FRAC)) );
  MAT33_VECT3_MULT2( offset_acc, state_kalman.gain.acc_err, state_kalman.res, 1, (1<<(INT32_MATLAB_FRAC+INT32_POS_FRAC-INT32_ACCEL_FRAC)) );
  delfly_model_add_states( &state_kalman.states, offset_pos, offset_vel, offset_acc );
  // update estimate co-variance    P_(k|k) = (I - K_k H_k) P_(k|k-1)   -- with #INT32_MATLAB_FRAC
  state_kalman_update_covariance( &state_kalman.covariance.estimate, state_kalman.gain );

  /* aftermath */
  delfly_model_assign_eulers( &state_kalman.states, state_kalman.out.att, state_kalman.out.rot );

  state_kalman_aftermath();
}


void state_kalman_aftermath (void) {

  struct Int32Vect3 pos, vel, acc;
  VECT3_COPY(pos, state_kalman.states.pos);
  VECT3_COPY(vel, state_kalman.states.vel);
  VECT3_COPY(acc, state_kalman.states.acc);

  set_position( &pos );
  set_velocity( &vel );
  //TODO: get airspeed
  set_airspeed( &vel );
  set_acceleration( &acc );

  delfly_state.flap_freq = rpm_sensor.motor_frequency;

  //TODO: get heading, azimuth
  delfly_state.h.heading = state_kalman.states.att.psi;
  delfly_state.h.azimuth = stateGetHorizontalSpeedDir_i();
  //TODO: get heading rate

  set_speed_vel( int32_vect2_norm(&delfly_state.h.vel) );
  set_speed_air( int32_vect2_norm(&delfly_state.h.air) );
  set_speed_acc( int32_vect2_norm(&delfly_state.h.acc) );
}


/* updates state estimation co-variance matrix using Kalman gain matrix
 * P_(k|k) = (I - K_k H_k) P_(k|k-1)
 * cov  -- co-variance matrix P_(k|k-1), with #INT32_MATLAB_FRAC
 * gain -- Kalman gain matrix K_k,       with #INT32_MATLAB_FRAC
 * returns P_(k|k), with #INT32_MATLAB_FRAC
 */
static inline void state_kalman_update_covariance ( struct DelflyModelCovariance* cov, struct StateKalmanGain gain ) {

  /* co-variance matrix P_(k|k) after update, with #INT32_MATLAB_FRAC */
  struct DelflyModelCovariance covK;

  // P'_(k|k) = -K_k H_k P_(k|k-1), with #INT32_MATLAB_FRAC
  MAT33_MULT2(covK.pos_pos, gain.pos_err, cov->pos_pos, -1, (1<<INT32_MATLAB_FRAC));
  MAT33_MULT2(covK.pos_vel, gain.pos_err, cov->pos_vel, -1, (1<<INT32_MATLAB_FRAC));
  MAT33_MULT2(covK.pos_acc, gain.pos_err, cov->pos_acc, -1, (1<<INT32_MATLAB_FRAC));
  MAT33_MULT2(covK.vel_pos, gain.vel_err, cov->pos_pos, -1, (1<<INT32_MATLAB_FRAC));
  MAT33_MULT2(covK.vel_vel, gain.vel_err, cov->pos_vel, -1, (1<<INT32_MATLAB_FRAC));
  MAT33_MULT2(covK.vel_acc, gain.vel_err, cov->pos_acc, -1, (1<<INT32_MATLAB_FRAC));
  MAT33_MULT2(covK.acc_pos, gain.acc_err, cov->pos_pos, -1, (1<<INT32_MATLAB_FRAC));
  MAT33_MULT2(covK.acc_vel, gain.acc_err, cov->pos_vel, -1, (1<<INT32_MATLAB_FRAC));
  MAT33_MULT2(covK.acc_acc, gain.acc_err, cov->pos_acc, -1, (1<<INT32_MATLAB_FRAC));

  // P_(k|k) = P'_(k|k) + P_(k|k-1), with #INT32_MATLAB_FRAC
  //         = P_(k|k-1) - K_k H_k P_(k|k-1)
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
