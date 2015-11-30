/*
 * Copyright (C) 2015 Torbjoern Cunis <t.cunis@tudelft.nl>
 *
 * Delfly State encapsulates necessary state informations for the
 * DelFly Control module.
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
 * @file /modules/delfly_control/delfly_state.h
 * @author Torbjoern Cunis
 */

/*
 * delfly_state.h
 *
 *  Created on: Nov 23, 2015
 *      Author: cunis
 */

#ifndef DELFLY_STATE_H_
#define DELFLY_STATE_H_


#include "math/pprz_algebra_int.h"
#include "math/pprz_geodetic_int.h"


#define VECT2_ZERO(_v)    VECT2_ASSIGN(_v, 0, 0)


/**
 * Delfly state in the horizontal, xy-plane.
 */
struct DelflyHorizontalState {
  /* xy-position in m, with #INT32_POS_FRAC */
  struct Int32Vect2 pos;
  /* xy-velocity in m/s, with #INT32_VEL_FRAC */
  struct Int32Vect2 vel;
  /* xy-airspeed in m/s, with #INT32_VEL_FRAC */
  struct Int32Vect2 air;
  /* xy-acceleration in m/s2, with #INT32_ACC_FRAC */
  struct Int32Vect2 acc;

  /* aero-dynamic velocity in m/s, with #INT32_VEL_FRAC */
  int32_t aero_dynamic_vel;
  /* aero-dynamic acceleration in m/s2, with #INT32_ACC_FRAC */
  int32_t aero_dynamic_acc;

  /* heading in rad, with #INT32_ANGLE_FRAC */
  int32_t heading;
  /* flight-path azimuth in rad, with #INT32_ANGLE_FRAC */
  int32_t azimuth;
  /* heading rate in rad/s, with #INT32_RATE_FRAC */
  int32_t head_rate;
};

/**
 * Delfly state in vertical, z-axis;
 * note, that z is pointing downwards!
 */
struct DelflyVerticalState {
  /* z-position in m, with #INT32_POS_FRAC */
  int32_t pos;
  /* z-velocity in m/s, with #INT32_SPEED_FRAC */
  int32_t vel;
  /* z-airspeed in m/s, with #INT32_SPEED_FRAC */
  int32_t air;
  /* z-acceleration in m/s2, with #INT32_ACCEL_FRAC */
  int32_t acc;
};


union Int32VectLong {
  struct Int32Vect2 xy;
  struct {
    int32_t fwd;
    int32_t ver;
  } fv;
};

/**
 * Delfly state in longitudinal, xz-plane.
 */
struct DelflyLongitudinalState {
  /* xz-velocity in m/s, with #INT32_VEL_FRAC */
  union Int32VectLong vel;
  /* xz-airspeed in m/s, with #INT32_VEL_FRAC */
  union Int32VectLong air;
  /* xz-acceleration in m/s2, with #INT32_ACC_FRAC */
  union Int32VectLong acc;
};


struct DelflyState {
  struct DelflyHorizontalState h;
  struct DelflyVerticalState v;

  struct DelflyLongitudinalState fv;
};


extern struct DelflyState delfly_state;


static inline struct Int32Vect2 set_horizontal_position ( struct Int32Vect2 pos ) { return delfly_state.h.pos = pos; }
static inline struct Int32Vect2 set_horizontal_velocity ( struct Int32Vect2 vel ) { return delfly_state.h.vel = vel; }
static inline struct Int32Vect2 set_horizontal_airspeed ( struct Int32Vect2 air ) { return delfly_state.h.air = air; }
static inline struct Int32Vect2 set_horizontal_acceleration ( struct Int32Vect2 acc ) { return delfly_state.h.acc = acc; }

static inline int32_t set_vertical_position ( int32_t alt ) { return delfly_state.v.pos = alt; }
static inline int32_t set_vertical_velocity ( int32_t vel ) {
  return delfly_state.v.vel         =
         delfly_state.fv.vel.fv.ver = vel;
}
static inline int32_t set_vertical_airspeed ( int32_t air ) {
  return delfly_state.v.air         =
         delfly_state.fv.air.fv.ver = air;
}
static inline int32_t set_vertical_acceleration ( int32_t acc ) {
  return delfly_state.v.acc         =
         delfly_state.fv.acc.fv.ver = acc;
}

static inline int32_t set_aero_dynamic_vel ( int32_t vel ) {
  return delfly_state.h.aero_dynamic_vel =
         delfly_state.fv.air.fv.fwd      = vel;
}
static inline int32_t set_aero_dynamic_acc ( int32_t acc ) {
  return delfly_state.h.aero_dynamic_acc =
         delfly_state.fv.acc.fv.fwd      = acc;
}

static inline int32_t set_heading ( int32_t heading ) { return delfly_state.h.heading = heading; }
static inline int32_t set_azimuth ( int32_t azimuth ) { return delfly_state.h.azimuth = azimuth; }
static inline int32_t set_heading_rate ( int32_t head_rate ) { return delfly_state.h.head_rate = head_rate; }


static inline void set_position_ned_i ( struct NedCoor_i* ned_i ) {
  delfly_state.h.pos.x = ned_i->x;
  delfly_state.h.pos.y = ned_i->y;
  delfly_state.v.pos   = ned_i->z;
}
static inline void set_velocity_ned_i ( struct NedCoor_i* ned_i ) {
  delfly_state.h.vel.x = ned_i->x;
  delfly_state.h.vel.y = ned_i->y;
  set_vertical_velocity( ned_i->z );
}
static inline void set_airspeed_ned_i ( struct NedCoor_i* ned_i ) {
  delfly_state.h.air.x = ned_i->x;
  delfly_state.h.air.y = ned_i->y;
  set_vertical_airspeed( ned_i->z );
}
static inline void set_acceleration_ned_i ( struct NedCoor_i* ned_i ) {
  delfly_state.h.acc.x = ned_i->x;
  delfly_state.h.acc.y = ned_i->y;
  set_vertical_acceleration( ned_i->z );
}


#endif /* DELFLY_STATE_H_ */
