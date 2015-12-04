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

#include "state.h"
#include "subsystems/sensors/rpm_sensor.h"


#define INT32_ZERO(_i)    { _i = 0; }


struct DelflyState delfly_state;


void state_estimation_init (void) {

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


void state_estimation_enter (void) {
  set_position_ned_i( stateGetPositionNed_i() );
  set_velocity_ned_i( stateGetSpeedNed_i() );
  //TODO: get airspeed
  set_airspeed_ned_i( stateGetSpeedNed_i() );
  set_acceleration_ned_i( stateGetAccelNed_i() );

  delfly_state.flap_freq = rpm_sensor.motor_frequency;

  //TODO: get heading, azimuth
  delfly_state.h.heading = stateGetNedToBodyEulers_i()->psi;
  delfly_state.h.azimuth = stateGetHorizontalSpeedDir_i();
  //TODO: get heading rate
}

void state_estimation_run (void) {

  state_estimation_enter();

  set_speed_vel( int32_vect2_norm(&delfly_state.h.vel) );
  set_speed_air( int32_vect2_norm(&delfly_state.h.air) );
  set_speed_acc( int32_vect2_norm(&delfly_state.h.acc) );
}
