/*
 * Copyright (C) 2015 Torbjoern Cunis <t.cunis@tudelft.nl>
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
 * @file /paparazzi/paparazzi/sw/airborne/modules/delfly_control/delfly_model.h
 * @author Torbjoern Cunis
 */

#ifndef DELFLY_MODEL_H_
#define DELFLY_MODEL_H_


#include "delfly_algebra_int.h"
#include "matlab_include.h"


struct DelflyModelStates {
  /* position in m, with #INT32_POS_FRAC */
  struct Int32Vect3 pos;

  /* velocity in m/s, with #INT32_SPEED_FRAC */
  struct Int32Vect3 vel;

  /* acceleration in m/s2, with #INT32_ACCEL_FRAC */
  struct Int32Vect3 acc;

  /* attitude in rad, with #INT32_EULER_FRAC */
  struct Int32Eulers att;

  /* rotation speed in rad/s, with #INT32_RATE_FRAC */
  struct Int32Rates rot;
};


struct DelflyModelCovariance {
  struct Int32Mat33 pos_pos;  struct Int32Mat33 pos_vel;  struct Int32Mat33 pos_acc;
  struct Int32Mat33 vel_pos;  struct Int32Mat33 vel_vel;  struct Int32Mat33 vel_acc;
  struct Int32Mat33 acc_pos;  struct Int32Mat33 acc_vel;  struct Int32Mat33 acc_acc;
};


struct DelflyModel {

	struct DelflyModelStates states;
};


extern struct DelflyModel delfly_model;


/* state disturbance co-variance matrix
 * with #INT32_MATLAB_FRAC                */
extern struct DelflyModelCovariance delfly_model_process_covariance;


extern void delfly_model_init (void);

extern void delfly_model_enter (void);

extern void delfly_model_set_cmd (int32_t cmd_h_acc, int32_t cmd_v_acc);

extern void delfly_model_run (void);


static inline void delfly_model_init_states ( struct DelflyModelStates* states ) {

  VECT3_ZERO(states->pos);
  VECT3_ZERO(states->vel);
  VECT3_ZERO(states->acc);

  EULERS_ZERO(states->att);
  RATES_ZERO(states->rot);
}
static inline void delfly_model_init_covariance ( struct DelflyModelCovariance* cov ) {
  MAT33_ZERO(cov->pos_pos);
  MAT33_ZERO(cov->pos_vel);
  MAT33_ZERO(cov->pos_acc);
  MAT33_ZERO(cov->vel_pos);
  MAT33_ZERO(cov->vel_vel);
  MAT33_ZERO(cov->vel_acc);
  MAT33_ZERO(cov->acc_pos);
  MAT33_ZERO(cov->acc_vel);
  MAT33_ZERO(cov->acc_acc);
}

static inline void delfly_model_assign_covariance ( struct DelflyModelCovariance* cov, int32_t period, struct Int32Vect3 acc_dist ) {

  struct Int32Mat33 dist_mat;
  float acc_dist_x = FLOAT_OF_BFP(acc_dist.x, INT32_MATLAB_FRAC),
        acc_dist_y = FLOAT_OF_BFP(acc_dist.y, INT32_MATLAB_FRAC),
        acc_dist_z = FLOAT_OF_BFP(acc_dist.z, INT32_MATLAB_FRAC);
  INT32_MAT33_DIAG( dist_mat, BFP_OF_REAL(SQUARE(acc_dist_x), INT32_MATLAB_FRAC),
		  	  	  	  	  	  	    BFP_OF_REAL(SQUARE(acc_dist_y), INT32_MATLAB_FRAC),
		  	  	  	  	  	  	      BFP_OF_REAL(SQUARE(acc_dist_z), INT32_MATLAB_FRAC) );

  struct Int64Mat33 temp;
  float period_f = FLOAT_OF_BFP(period, INT32_TIME_FRAC);

  MAT33_ZERO(temp);
  MAT33_ADD_SCALED2( temp, dist_mat, HYCUBE(period_f), 4 );
  MAT33_COPY(cov->pos_pos, temp);
  MAT33_ZERO(temp);
  MAT33_ADD_SCALED2( temp, dist_mat, CUBE(period_f),   2 );
  MAT33_COPY(cov->pos_vel, temp);
  MAT33_ZERO(temp);
  MAT33_ADD_SCALED2( temp, dist_mat, SQUARE(period), (2<<(2*INT32_TIME_FRAC)) );
  MAT33_COPY(cov->pos_acc, temp);

  MAT33_ZERO(temp);
  MAT33_ADD_SCALED2( temp, dist_mat, CUBE(period_f),   2 );
  MAT33_COPY(cov->vel_pos, temp);
  MAT33_ZERO(temp);
  MAT33_ADD_SCALED2( temp, dist_mat, SQUARE(period), (1<<(2*INT32_TIME_FRAC)) );
  MAT33_COPY(cov->vel_vel, temp);
  MAT33_ZERO(temp);
  MAT33_ADD_SCALED2( temp, dist_mat, period,         (1<<(INT32_TIME_FRAC)) );
  MAT33_COPY(cov->acc_acc, temp);

  MAT33_ZERO(temp);
  MAT33_ADD_SCALED2( temp, dist_mat, SQUARE(period), (2<<(2*INT32_TIME_FRAC)) );
  MAT33_COPY(cov->acc_pos, temp);
  MAT33_ZERO(temp);
  MAT33_ADD_SCALED2( temp, dist_mat, period,         (1<<(INT32_TIME_FRAC)) );
  MAT33_COPY(cov->acc_vel, temp);
  MAT33_ZERO(temp);
  MAT33_ADD_SCALED2( temp, dist_mat, 1,              1 );
  MAT33_COPY(cov->acc_acc, temp);
}

static inline void delfly_model_assign_states ( struct DelflyModelStates* states, struct Int32Vect3 pos, struct Int32Vect3 vel, struct Int32Vect3 acc ) {

  VECT3_COPY(states->pos, pos);
  VECT3_COPY(states->vel, vel);
  VECT3_COPY(states->acc, acc);
}
static inline void delfly_model_assign_eulers ( struct DelflyModelStates* states, struct Int32Eulers att, struct Int32Rates rot ) {

  EULERS_COPY(states->att, att);
  RATES_COPY(states->rot, rot);
}
static inline void delfly_model_add_states ( struct DelflyModelStates* states, struct Int32Vect3 pos, struct Int32Vect3 vel, struct Int32Vect3 acc ) {

  VECT3_ADD(states->pos, pos);
  VECT3_ADD(states->vel, vel);
  VECT3_ADD(states->acc, acc);
}

static inline void delfly_model_predict_states ( struct DelflyModelStates* states, int32_t period ) {

  struct Int64Vect3 temp;
  VECT3_COPY(temp, states->pos);
  VECT3_ADD_SCALED2( temp, states->vel, period, (1<<(INT32_TIME_FRAC+INT32_SPEED_FRAC-INT32_POS_FRAC)) );
  VECT3_ADD_SCALED2( temp, states->acc, SQUARE(period), (2<<(2*INT32_TIME_FRAC+INT32_ACCEL_FRAC-INT32_POS_FRAC)) );
  VECT3_COPY(states->pos, temp);

  VECT3_ADD_SCALED2( states->vel, states->acc, period, (1<<(INT32_TIME_FRAC+INT32_ACCEL_FRAC-INT32_SPEED_FRAC)) );

  // states->acc = states->acc

  //TODO: update rotation
}

static inline void delfly_model_predict_covariance ( struct DelflyModelCovariance* cov, int32_t period ) {

  //struct DelflyModelCovariance covK = *cov;
  struct Int64Mat33 temp;

  static const int64_t TIME_SQUARE = 1<<(2*INT32_TIME_FRAC);
  static const int64_t TIME_CUBE   = 1<<(3*INT32_TIME_FRAC);

  MAT33_ADD_SCALED2( cov->pos_pos, cov->vel_pos, period,         (1<<INT32_TIME_FRAC) );
  MAT33_ADD_SCALED2( cov->pos_pos, cov->pos_vel, period,         (1<<INT32_TIME_FRAC) );
  MAT33_ADD_SCALED2( cov->pos_pos, cov->acc_pos, SQUARE(period), (2*TIME_SQUARE) );
  MAT33_ADD_SCALED2( cov->pos_pos, cov->vel_vel, SQUARE(period), (1*TIME_SQUARE) );
  MAT33_ADD_SCALED2( cov->pos_pos, cov->pos_acc, SQUARE(period), (2*TIME_SQUARE) );
  MAT33_ADD_SCALED2( cov->pos_pos, cov->acc_vel, CUBE(period),   (2*TIME_CUBE) );
  MAT33_ADD_SCALED2( cov->pos_pos, cov->vel_acc, CUBE(period),   (2*TIME_CUBE) );
  MAT33_ZERO(temp);
  MAT33_ADD_SCALED2( temp, cov->acc_acc, HYCUBE(period)/(1<<INT32_TIME_FRAC), (1<<(3*INT32_TIME_FRAC)) );
  MAT33_ADD_SCALED2( cov->pos_pos, temp, 1, 4);
  MAT33_ADD( cov->pos_pos, delfly_model_process_covariance.pos_pos );

  MAT33_ADD_SCALED2( cov->pos_vel, cov->vel_vel, period,         (1<<INT32_TIME_FRAC) );
  MAT33_ADD_SCALED2( cov->pos_vel, cov->pos_acc, period,         (1<<INT32_TIME_FRAC) );
  MAT33_ADD_SCALED2( cov->pos_vel, cov->acc_vel, SQUARE(period), (2*TIME_SQUARE) );
  MAT33_ADD_SCALED2( cov->pos_vel, cov->vel_acc, SQUARE(period), (1*TIME_SQUARE) );
  MAT33_ADD_SCALED2( cov->pos_vel, cov->acc_acc, CUBE(period),   (2*TIME_CUBE) );
  MAT33_ADD( cov->pos_vel, delfly_model_process_covariance.pos_vel );

  MAT33_ADD_SCALED2( cov->pos_acc, cov->vel_acc, period,         (1<<INT32_TIME_FRAC) );
  MAT33_ADD_SCALED2( cov->pos_acc, cov->acc_vel, SQUARE(period), (2*TIME_SQUARE) );
  MAT33_ADD( cov->pos_acc, delfly_model_process_covariance.pos_acc );

  MAT33_ADD_SCALED2( cov->vel_pos, cov->acc_pos, period,         (1<<INT32_TIME_FRAC) );
  MAT33_ADD_SCALED2( cov->vel_pos, cov->vel_vel, period,         (1<<INT32_TIME_FRAC) );
  MAT33_ADD_SCALED2( cov->vel_pos, cov->acc_vel, SQUARE(period), (1*TIME_SQUARE) );
  MAT33_ADD_SCALED2( cov->vel_pos, cov->vel_acc, SQUARE(period), (2*TIME_SQUARE) );
  MAT33_ADD_SCALED2( cov->vel_pos, cov->acc_acc, CUBE(period),   (2*TIME_CUBE) );
  MAT33_ADD( cov->vel_pos, delfly_model_process_covariance.vel_pos );

  MAT33_ADD_SCALED2( cov->vel_vel, cov->acc_vel, period,         (1<<INT32_TIME_FRAC) );
  MAT33_ADD_SCALED2( cov->vel_vel, cov->vel_acc, period,         (1<<INT32_TIME_FRAC) );
  MAT33_ADD_SCALED2( cov->vel_vel, cov->acc_acc, SQUARE(period), (1*TIME_SQUARE) );
  MAT33_ADD( cov->vel_vel, delfly_model_process_covariance.vel_vel );

  MAT33_ADD_SCALED2( cov->vel_acc, cov->acc_acc, period, (1<<INT32_TIME_FRAC) );
  MAT33_ADD( cov->vel_acc, delfly_model_process_covariance.vel_acc );

  MAT33_ADD_SCALED2( cov->acc_pos, cov->acc_vel, period,         (1<<INT32_TIME_FRAC) );
  MAT33_ADD_SCALED2( cov->acc_pos, cov->acc_acc, SQUARE(period), (2*TIME_SQUARE) );
  MAT33_ADD( cov->acc_pos, delfly_model_process_covariance.acc_pos );

  MAT33_ADD_SCALED2( cov->acc_vel, cov->acc_acc, period, (1<<INT32_TIME_FRAC) );
  MAT33_ADD( cov->acc_vel, delfly_model_process_covariance.acc_vel );

  MAT33_ADD( cov->acc_acc, delfly_model_process_covariance.acc_acc );
}


#endif /* DELFLY_MODEL_H_ */
