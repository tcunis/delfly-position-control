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


struct DelflyModelStates {
  struct Int32Vect3 pos;
  struct Int32Vect3 vel;
  struct Int32Vect3 acc;
};


inline void delfly_model_init_states ( struct DelflyModelStates* states ) {

  VECT3_ZERO(states->pos);
  VECT3_ZERO(states->vel);
  VECT3_ZERO(states->acc);
}

inline void delfly_model_assign_states ( struct DelflyModelStates* states, struct Int32Vect3 pos, struct Int32Vect3 vel, struct Int32Vect3 acc ) {

  VECT3_COPY(states->pos, pos);
  VECT3_COPY(states->vel, vel);
  VECT3_COPY(states->acc, acc);
}

inline void delfly_model_update_states ( struct DelflyModelStates* states, double period ) {

  VECT3_ADD_SCALED( states->pos, states->vel, period/(1<<(INT32_SPEED_FRAC-INT32_POS_FRAC)) );
  VECT3_ADD_SCALED( states->pos, states->acc, period*period/(2<<INT32_ACCEL_FRAC-INT32_POS_FRAC) );

  VECT3_ADD_SCALED( states->vel, states->acc, period*(1<<(INT32_SPEED_FRAC-INT32_ACCEL_FRAC)) );
}


#endif /* DELFLY_MODEL_H_ */
