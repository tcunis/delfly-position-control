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

#include "generated/airframe.h"
#include "generated/modules.h"

#include "matlab_include.h"
#include "delfly_algebra_int.h"

#include "state.h"
#include "subsystems/imu.h"
#include "subsystems/gps.h"
#include "subsystems/abi.h"
#include "subsystems/ins/ins_int.h"
#include "subsystems/sensors/rpm_sensor.h"

#ifndef INS_INT_IMU_ID
#define INS_INT_IMU_ID ABI_BROADCAST
#endif


struct DelflyState delfly_state;

struct StateEstimation state_estimation;


struct DelflyModelCovariance delfly_model_process_covariance;

/* measurement noise co-variance matrix
 * with #INT32_MATLAB_FRAC                */
struct Int32Mat33 state_estimation_noise_covariance;

static abi_event accel_ev;
static abi_event gps_ev;


static void state_estimation_enter_mode (uint8_t mode);
static void state_estimation_getoutput (void);
static void state_estimation_aftermath (void);

static void acc_cb (uint8_t sender_id __attribute__((unused)), uint32_t stamp __attribute__((unused)), struct Int32Vect3 *accel);
static void gps_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp __attribute__((unused)), struct GpsState *gps_s);

static void state_estimation_run_kalman (void);


void state_estimation_init (void) {

  state_estimation.mode = STATE_ESTIMATION_MODE;
  state_estimation.type = STATE_ESTIMATION_TYPE;
  state_estimation.gps_freq = STATE_ESTIMATION_GPS_FREQ;

  delfly_model_init_states( &state_estimation.states );
  delfly_model_init_states( &state_estimation.out );

  VECT3_ZERO( state_estimation.res );

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

  AbiBindMsgIMU_ACCEL_INT32(INS_INT_IMU_ID, &accel_ev, acc_cb);
  AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, gps_cb);
}


void state_estimation_getoutput (void) {

  VECT3_COPY(state_estimation.out.pos, *stateGetPositionNed_i());
  VECT3_COPY(state_estimation.out.vel, *stateGetSpeedNed_i());
  //VECT3_COPY(state_estimation.out.acc, *stateGetAccelNed_i());

  delfly_model_assign_eulers( &state_estimation.out,
		  	  	  	  	  	  	  *stateGetNedToBodyEulers_i(),
		  	  	  	  	  	  	  *stateGetBodyRates_i()
  );
}


static void acc_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp __attribute__((unused)), struct Int32Vect3 *accel) {

  /* untilt accels */
  struct Int32Vect3 accel_meas_body;
  struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&imu.body_to_imu);
  int32_rmat_transp_vmult(&accel_meas_body, body_to_imu_rmat, accel);
  struct Int32Vect3 accel_meas_ltp;
  int32_rmat_transp_vmult(&accel_meas_ltp, stateGetNedToBodyRMat_i(), &accel_meas_body);

  VECT3_COPY(state_estimation.out.acc, accel_meas_ltp);
  state_estimation.out.acc.z += ACCEL_BFP_OF_REAL(9.81);
}

static void gps_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp __attribute__((unused)), struct GpsState *gps_s) {

  static struct LtpDef_i ltp_def;
  ltp_def_from_ecef_i(&ltp_def, &gps.ecef_pos);

  struct NedCoor_i gps_pos_cm_ned;
  ned_of_ecef_point_i(&gps_pos_cm_ned, &ltp_def, &gps_s->ecef_pos);

  //todo: log gps ned position
  gps_diagnostics_log_pos( &gps_pos_cm_ned );
}


void ins_module_int_propagate(struct NedCoor_i* acc, float dt) {}
void ins_module_int_update_gps(struct NedCoor_i* pos, struct NedCoor_i* vel, float dt) {}


void state_estimation_enter (void) {

  state_estimation_enter_mode( state_estimation.mode );
}

void state_estimation_enter_mode (uint8_t mode) {

  state_estimation_getoutput();

  struct Int32Vect3 pos, vel, acc;


  VECT3_COPY(pos, state_estimation.out.pos);

  switch (mode) {
    /* PREDICATE/COMPLEMENTARY:
     * nothing to do yet				*/
    case STATE_ESTIMATION_TYPE_PRED:
    case STATE_ESTIMATION_TYPE_COMP: {
      VECT3_ZERO(vel);
      VECT3_COPY(acc, state_estimation.out.acc);
    } break;

    /* GPS/KALMAN: assume zero velocity
     * & acceleration during enter.		*/
    case STATE_ESTIMATION_TYPE_GPS:
    case STATE_ESTIMATION_TYPE_KALMAN: {
      VECT3_ZERO(vel);
      VECT3_ZERO(acc);
    } break;

    case STATE_ESTIMATION_MODE_ENTER:
    case STATE_ESTIMATION_MODE_ESTIMATE:
    case STATE_ESTIMATION_MODE_OFF:
    default: {
      VECT3_COPY(vel, state_estimation.out.vel);
      VECT3_COPY(acc, state_estimation.out.acc);
    } break;
  }


  delfly_model_assign_states( &state_estimation.states, pos, vel, acc );
  delfly_model_assign_eulers( &state_estimation.states,
		  	  	  	  	  	  	  state_estimation.out.att,
		  	  	  	  	  	  	  state_estimation.out.rot
  );

  state_estimation.mode = mode;
}



void state_estimation_run (void) {

  state_estimation_getoutput();

  switch (state_estimation.mode) {

    case STATE_ESTIMATION_MODE_ENTER: {
      state_estimation_enter_mode(state_estimation.type);
    } break;

    case STATE_ESTIMATION_MODE_ESTIMATE:
      break;

    case STATE_ESTIMATION_TYPE_KALMAN:
      state_estimation_run_kalman();
      break;

    case STATE_ESTIMATION_TYPE_GPS: {
      static uint32_t last_time = 0, last_ticks = 0;
      struct Int32Vect3 pos_diff, vel_diff;
      struct Int32Vect3 last_pos, last_vel;
      if ( gps.last_msg_time == last_time && gps.last_msg_ticks == last_ticks )
      {  }
      else // new gps message
      {
    	  VECT3_COPY(last_pos, state_estimation.states.pos);
      	VECT3_COPY(last_vel, state_estimation.states.vel);

      	// get position from gps
      	VECT3_COPY(state_estimation.states.pos, state_estimation.out.pos);

      	// get velocity as 1st discrete time derivative
      	VECT3_DIFF(pos_diff, state_estimation.states.pos, last_pos);
    	  VECT3_ASSIGN_SCALED2(state_estimation.states.vel, pos_diff, state_estimation.gps_freq*(1<<(INT32_SPEED_FRAC-INT32_POS_FRAC)), 1);

    	  // get acceleration as 2nd discrete time derivative
    	  VECT3_DIFF(vel_diff, state_estimation.states.vel, last_vel);
    	  VECT3_ASSIGN_SCALED2(state_estimation.states.acc, vel_diff, state_estimation.gps_freq, (1<<(INT32_SPEED_FRAC-INT32_ACCEL_FRAC)));

    	  last_time = gps.last_msg_time;
    	  last_ticks = gps.last_msg_ticks;
      }
    } break;

    case STATE_ESTIMATION_MODE_OFF:
    default: {
      state_estimation_enter();
    } break;
  }


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


/* updates state estimation co-variance matrix using Kalman gain matrix
 * P_(k|k) = (I - K_k H_k) P_(k|k-1)
 * cov  -- co-variance matrix P_(k|k-1), with #INT32_MATLAB_FRAC
 * gain -- Kalman gain matrix K_k,       with #INT32_MATLAB_FRAC
 * returns P_(k|k), with #INT32_MATLAB_FRAC
 */
static inline void state_estimation_update_covariance ( struct DelflyModelCovariance* cov, struct StateEstimationGain gain ) {

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

static void state_estimation_run_kalman (void) {
  /* residual co-variance matrix inverse
   * with #INT32_MATLAB_FRAC              */
  // <for debug>: use state_estimation.covariance.residual_inv
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

  MAT33_ZERO( state_estimation.covariance.residual_inv );

  /* Kalman filter -- predict */
  // a-priori state estimate 			  x_(k|k-1) = A x_(k-1|k-1)           -- with states fracs
  delfly_model_predict_states( &state_estimation.states, state_estimation.period );
  // a-priori estimate co-variance	P_(k|k-1) = A P_(k-1|k-1) A' + Q    -- with #INT32_MATLAB_FRAC
  delfly_model_predict_covariance( &state_estimation.covariance.estimate, state_estimation.period );

  /* Kalman filter -- calculate gain */
  // measurement residual				    y_k = z_k - H x_(k|k-1)             -- with #INT32_POS_FRAC
  VECT3_DIFF( state_estimation.res, state_estimation.out.pos, state_estimation.states.pos );
  // residual co-variance				    S_k = H P_(k|k-1) H' + R
  //                                    =  P11_(k|k-1)   + R            -- with #INT32_MATLAB_FRAC
  MAT33_COPY( state_estimation.covariance.residual, state_estimation.covariance.estimate.pos_pos );
  MAT33_ADD(  state_estimation.covariance.residual, state_estimation_noise_covariance );
  // optimal Kalman gain				    K_k = P_(k|k-1) H' inv(S_k)         -- with #INT32_MATLAB_FRAC
  INT32_MAT33_INV( state_estimation.covariance.residual_inv, state_estimation.covariance.residual, INT32_MATLAB_FRAC );
  MAT33_MULT2( state_estimation.gain.pos_err, state_estimation.covariance.estimate.pos_pos, state_estimation.covariance.residual_inv, 1, (1<<INT32_MATLAB_FRAC) );
  MAT33_MULT2( state_estimation.gain.vel_err, state_estimation.covariance.estimate.vel_pos, state_estimation.covariance.residual_inv, 1, (1<<INT32_MATLAB_FRAC) );
  MAT33_MULT2( state_estimation.gain.acc_err, state_estimation.covariance.estimate.acc_pos, state_estimation.covariance.residual_inv, 1, (1<<INT32_MATLAB_FRAC) );

  /* Kalman filter -- update */
  // update states                  x_(k|k) = x_(k|k-1) + K_k y_k       -- with states fracs
  MAT33_VECT3_MULT2( offset_pos, state_estimation.gain.pos_err, state_estimation.res, 1, (1<<(INT32_MATLAB_FRAC)) );
  MAT33_VECT3_MULT2( offset_vel, state_estimation.gain.vel_err, state_estimation.res, 1, (1<<(INT32_MATLAB_FRAC+INT32_POS_FRAC-INT32_SPEED_FRAC)) );
  MAT33_VECT3_MULT2( offset_acc, state_estimation.gain.acc_err, state_estimation.res, 1, (1<<(INT32_MATLAB_FRAC+INT32_POS_FRAC-INT32_ACCEL_FRAC)) );
//  delfly_model_add_states( &state_estimation.states, offset_pos, offset_vel, offset_acc );
  // update estimate co-variance    P_(k|k) = (I - K_k H_k) P_(k|k-1)   -- with #INT32_MATLAB_FRAC
  state_estimation_update_covariance( &state_estimation.covariance.estimate, state_estimation.gain );

  /* aftermath */
  delfly_model_assign_eulers( &state_estimation.states, state_estimation.out.att, state_estimation.out.rot );
}
