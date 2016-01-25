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
#include "subsystems/ins/ins_int.h"
#include "subsystems/sensors/rpm_sensor.h"

#ifndef INS_INT_IMU_ID
#define INS_INT_IMU_ID ABI_BROADCAST
#endif

#define init_butterworth_2_low_pass_vect3(__FILTER__,__SAMPLE__,__CUTOFF__,__VAL0__)  {     \
    init_butterworth_2_low_pass_int(&(__FILTER__)->x, __SAMPLE__, __CUTOFF__, (__VAL0__).x);  \
    init_butterworth_2_low_pass_int(&(__FILTER__)->y, __SAMPLE__, __CUTOFF__, (__VAL0__).y);  \
    init_butterworth_2_low_pass_int(&(__FILTER__)->z, __SAMPLE__, __CUTOFF__, (__VAL0__).z);  \
  }

#define update_butterworth_2_low_pass_vect3(__FILTER__,__VALUE__)  {      \
    update_butterworth_2_low_pass_int(&(__FILTER__)->x, (__VALUE__).x);   \
    update_butterworth_2_low_pass_int(&(__FILTER__)->y, (__VALUE__).y);   \
    update_butterworth_2_low_pass_int(&(__FILTER__)->z, (__VALUE__).z);   \
  }

#define get_butterworth_2_low_pass_vect3(__RETURN__, __FILTER__)  {     \
    (__RETURN__).x = get_butterworth_2_low_pass_int(&(__FILTER__)->x);  \
    (__RETURN__).y = get_butterworth_2_low_pass_int(&(__FILTER__)->y);  \
    (__RETURN__).z = get_butterworth_2_low_pass_int(&(__FILTER__)->z);  \
  }



struct DelflyState delfly_state;

struct StateEstimation state_estimation = {.mode = STATE_ESTIMATION_MODE_OFF};

struct StateFilter state_filter;

struct FlapFilter average_filter;


static void state_estimation_aftermath (void);



void state_estimation_init (void) {

  if ( state_estimation.mode != STATE_ESTIMATION_MODE_OFF )
    return;

  //else:
  state_estimation.mode = STATE_ESTIMATION_MODE_ENTER;
  state_estimation.type = STATE_ESTIMATION_TYPE;

  delfly_model_init_states( &state_estimation.states );
  delfly_model_init_states( &state_estimation.out );

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


void ins_module_int_init (void) {

  if ( state_estimation.mode == STATE_ESTIMATION_MODE_OFF ) {
    state_estimation_init();
  }

  ins_module_int_reset_local_origin();

  state_estimation.mode = state_estimation.type;
}

void ins_module_int_reset_local_origin (void) {

  VECT3_COPY(state_estimation.states.pos, ins_int.ltp_pos);
  VECT3_COPY(state_estimation.states.vel, ins_int.ltp_speed);
  VECT3_COPY(state_estimation.states.acc, ins_int.ltp_accel);

  state_filter.sample_time = 1.0/(STATE_ESTIMATION_GPS_FREQ);
  state_filter.cut_off     = STATE_ESTIMATION_FILTER_CUTOFF;

  init_butterworth_2_low_pass_vect3( &state_filter.pos, state_filter.cut_off, state_filter.sample_time, state_estimation.states.pos );
}


void ins_module_int_propagate(struct Int32Vect3* acc, float dt __attribute__((unused))) {

  VECT3_COPY(state_estimation.out.acc, *acc);
  state_estimation.out.acc.z += ACCEL_BFP_OF_REAL(9.81);

  switch (state_estimation.mode) {

  case STATE_ESTIMATION_TYPE_GPS:
  case STATE_ESTIMATION_TYPE_GPS_FILTER: {
    // ignore accelerometer data
    } break;

  default: {
    // accelerometer pass-through
    VECT3_COPY(ins_int.ltp_accel, state_estimation.out.acc);
    } break;
  }
}


void ins_module_int_update_gps(struct NedCoor_i* pos, struct NedCoor_i* vel, float dt) {

  VECT3_COPY(state_estimation.out.pos, *pos);
  VECT3_COPY(state_estimation.out.vel, *vel);

  // default: get current position from gps
  struct Int32Vect3 this_pos, ltp_pos;
  VECT3_COPY(this_pos, state_estimation.out.pos);
  VECT3_COPY(ltp_pos,  state_estimation.out.pos);

  /* for all types:
   *  - store last position and velocity state;
   *  - put current position into filter        */
  struct Int32Vect3 last_pos, last_vel;
  VECT3_COPY(last_pos, state_estimation.states.pos);
  VECT3_COPY(last_vel, state_estimation.states.vel);
  //get_butterworth_2_low_pass_vect3(last_pos_filter, &state_filter.pos);
  update_butterworth_2_low_pass_vect3(&state_filter.pos, state_estimation.out.pos);

  // get current heading from gps
  state_estimation.states.att.psi = gps.course;

  switch (state_estimation.mode) {

  case STATE_ESTIMATION_TYPE_GPS_AVERAGE:
  case STATE_ESTIMATION_TYPE_GPS_FILTER:
    if ( state_estimation.mode == STATE_ESTIMATION_TYPE_GPS_AVERAGE) {
      // get cycle-averaged position
      if ( rpm_sensor.average_frequency == 0 ) {
        //nothing to do, use this_pos = out.pos
      } else {
        //update average filter
        VECT3_ADD(average_filter.sum_pos, this_pos);
        average_filter.sample_count++;
        //integrate flapping period
        average_filter.sum_dt += dt;

        if ( rpm_sensor.rot_count > average_filter.flap_count ) {
          //flapping cycle passed:
          //calculate average position
          VECT3_SDIV(this_pos, average_filter.sum_pos, average_filter.sample_count);
          VECT3_COPY(ltp_pos, this_pos);
          //use flapping period to derivate velocity and acceleration
          dt = average_filter.sum_dt;
          //zero integrator and counter
          INT32_VECT3_ZERO(average_filter.sum_pos);
          average_filter.sum_dt = 0;
          average_filter.sample_count = 0;
          //remember flapping cycle count
          average_filter.flap_count = rpm_sensor.rot_count;
        } else {
          //within flapping cycle:
          //use last position
          VECT3_COPY(this_pos, state_estimation.states.pos);
          VECT3_COPY(ltp_pos,  this_pos);

          dt = 0; //DO NOT calculate velocity and acceleration!
        }
      }
    } else {
      // get position from filter
      get_butterworth_2_low_pass_vect3(this_pos, &state_filter.pos);
    }
    //no break
	case STATE_ESTIMATION_TYPE_GPS: {
      struct Int32Vect3 pos_diff, vel_diff;

      /* get estimation state position
       * (according to type)                 */
      VECT3_COPY(state_estimation.states.pos, this_pos);

      if ( dt > 0 ) { // new gps message

        // get velocity as 1st discrete time derivative
        VECT3_DIFF(pos_diff, state_estimation.states.pos, last_pos);
        VECT3_ASSIGN_SCALED2(state_estimation.states.vel, pos_diff, (1<<(INT32_SPEED_FRAC-INT32_POS_FRAC)), dt);

        // get acceleration as 2nd discrete time derivative
        VECT3_DIFF(vel_diff, state_estimation.states.vel, last_vel);
        VECT3_ASSIGN_SCALED2(state_estimation.states.acc, vel_diff, 1, dt*(1<<(INT32_SPEED_FRAC-INT32_ACCEL_FRAC)));
      }

      /* get ltp state position from gps
       * (both type GPS and GPS_FILTER)      */
      VECT3_COPY(ins_int.ltp_pos, ltp_pos);
      /* get ltp state speed & acceleration
       * from state estimation               */
      VECT3_COPY(ins_int.ltp_speed, state_estimation.states.vel);
      VECT3_COPY(ins_int.ltp_accel, state_estimation.states.acc);
    } break;

	default: {
      // GPS pass-through
      VECT3_COPY(ins_int.ltp_pos, state_estimation.out.pos);
      VECT3_COPY(ins_int.ltp_speed, state_estimation.out.vel);
    } break;
  }

  gps_diagnostics_log_pos( pos );
}


void state_estimation_run (void) {

  switch (state_estimation.mode) {
    case STATE_ESTIMATION_MODE_OFF:
    case STATE_ESTIMATION_MODE_ENTER:
      return; //nothing to do, init first

    default:
      break;
  }
  //else:
  state_estimation.mode = state_estimation.type;

  struct Int32Vect3 pos, vel, air, acc;
  set_position_ned_i( &pos, stateGetPositionNed_i() );
  set_velocity_ned_i( &vel, stateGetSpeedNed_i() );
  //TODO: get airspeed
  set_airspeed_ned_i( &air, stateGetSpeedNed_i() );
  set_acceleration_ned_i( &acc, stateGetAccelNed_i() );

  set_position( &pos );
  set_velocity( &vel );
  set_airspeed( &air );
  set_acceleration( &acc );

  delfly_state.flap_freq = round(rpm_sensor.motor_frequency*10)/10;

  //TODO: get heading, azimuth
  delfly_state.h.heading = stateGetNedToBodyEulers_i()->psi;
  delfly_state.h.azimuth = stateGetHorizontalSpeedDir_i();
  //TODO: get heading rate

  state_estimation_aftermath();
}


void state_estimation_aftermath (void) {

  set_speed_vel( int32_vect2_norm(&delfly_state.h.vel) );
  set_speed_air( int32_vect2_norm(&delfly_state.h.air) );
  set_speed_acc( int32_vect2_norm(&delfly_state.h.acc) );
}
