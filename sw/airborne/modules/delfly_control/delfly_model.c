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
 * @file /paparazzi/paparazzi/sw/airborne/modules/delfly_control/delfly_model.c
 * @author Torbjoern Cunis
 */

#include "delfly_model.h"

#include "delfly_control.h"

#include "state_estimation.h"

#include "delfly_algebra_int.h"


struct DelflyModel delfly_model;


void delfly_model_init (void) {

  delfly_model.mode = DELFLY_MODEL_MODE_OFF;

  delfly_model_init_states( &delfly_model.states );

  VECT3_ZERO(delfly_model.cmd.acc);
  RATES_ZERO(delfly_model.cmd.rot);
}


void delfly_model_enter (void) {

  delfly_model.mode = DELFLY_MODEL_MODE_ENTER;

  delfly_model_assign_states( &delfly_model.states, state_estimation.states.pos, state_estimation.states.vel, state_estimation.states.acc );
  delfly_model_assign_eulers( &delfly_model.states, state_estimation.states.att, state_estimation.states.rot );
}

void delfly_model_set_cmd (int32_t cmd_h_acc, int32_t cmd_v_acc) {

  delfly_model.cmd.acc_fv.fv.fwd = cmd_h_acc;
  delfly_model.cmd.acc_fv.fv.ver = cmd_v_acc;

  delfly_model.cmd.acc.x = ( cmd_h_acc * pprz_itrig_cos(delfly_model.states.att.psi) )/(1<<INT32_TRIG_FRAC);
  delfly_model.cmd.acc.y = ( cmd_h_acc * pprz_itrig_sin(delfly_model.states.att.psi) )/(1<<INT32_TRIG_FRAC);
  delfly_model.cmd.acc.z =   cmd_v_acc;
}

void delfly_model_run (void) {

  delfly_model.mode = DELFLY_MODEL_MODE_EVOLUTE;

  VECT2_COPY(delfly_model.states.acc_fv.xy, delfly_model.cmd.acc_fv.xy);
  VECT3_COPY(delfly_model.states.acc, delfly_model.cmd.acc);

  delfly_model_predict_states( &delfly_model.states, DELFLY_MODEL_RUN_PERIOD*(1<<INT32_TIME_FRAC) );

  delfly_model.states.vel_fv.fv.fwd = INT32_VECT2_NORM(delfly_model.states.vel);
  delfly_model.states.vel_fv.fv.ver = delfly_model.states.vel.z;

  delfly_model_assign_eulers( &delfly_model.states, state_estimation.states.att, state_estimation.states.rot );
}

void delfly_model_evolute (float dt) {

  delfly_model.mode = DELFLY_MODEL_MODE_EVOLUTE;

  VECT2_COPY(delfly_model.states.acc_fv.xy, delfly_model.cmd.acc_fv.xy);
  VECT3_COPY(delfly_model.states.acc, delfly_model.cmd.acc);

  delfly_model_predict_states_f( &delfly_model.states, dt );

  delfly_model.states.vel_fv.fv.fwd = INT32_VECT2_NORM(delfly_model.states.vel);
  delfly_model.states.vel_fv.fv.ver = delfly_model.states.vel.z;

  delfly_model_assign_eulers( &delfly_model.states, state_estimation.states.att, state_estimation.states.rot );
}
