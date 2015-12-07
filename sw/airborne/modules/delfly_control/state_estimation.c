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

#include "state.h"
#include "subsystems/sensors/rpm_sensor.h"



struct DelflyState delfly_state;

struct StateEstimation state_estimation;


static void state_estimation_getoutput (void);
static void state_estimation_aftermath (void);


void state_estimation_init (void) {

  delfly_model_init_states( &state_estimation.states );
  delfly_model_init_states( &state_estimation.out );

  VECT3_ZERO( state_estimation.error );

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

  delfly_model_assign_states( &state_estimation.out,
		  	  	  	  	  	  	  *stateGetPositionNed_i(),
								  *stateGetSpeedNed_i(),
								  *stateGetAccelNed_i()
  );
}


void state_estimation_enter (void) {

  state_estimation_getoutput();

  delfly_model_assign_states( &state_estimation.states,
		  	  	  	  	  	  	  state_estimation.out.pos,
								  state_estimation.out.vel,
								  state_estimation.out.acc
  );
}


void state_estimation_run (void) {

  state_estimation_getoutput();

  delfly_model_update_states( &state_estimation.states, STATE_ESTIMATION_RUN_PERIOD );

  //TODO prediction

  VECT3_DIFF( state_estimation.error, state_estimation.out.pos, state_estimation.states.pos );

  //TODO update

  state_estimation_aftermath();
}


void state_estimation_aftermath (void) {

  set_position_ned_i( state_estimation.states.pos );
  set_velocity_ned_i( state_estimation.states.vel );
  //TODO: get airspeed
  set_airspeed_ned_i( state_estimation.states.vel );
  set_acceleration_ned_i( state_estimation.states.acc );

  delfly_state.flap_freq = rpm_sensor.motor_frequency;

  //TODO: get heading, azimuth
  delfly_state.h.heading = stateGetNedToBodyEulers_i()->psi;
  delfly_state.h.azimuth = stateGetHorizontalSpeedDir_i();
  //TODO: get heading rate

  set_speed_vel( int32_vect2_norm(&delfly_state.h.vel) );
  set_speed_air( int32_vect2_norm(&delfly_state.h.air) );
  set_speed_acc( int32_vect2_norm(&delfly_state.h.acc) );
}
