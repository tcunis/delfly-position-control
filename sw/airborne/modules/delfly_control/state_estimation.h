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

#ifndef STATE_ESTIMATION_H_
#define STATE_ESTIMATION_H_


#include "delfly_model.h"

#include "filters/low_pass_filter.h"



#define STATE_ESTIMATION_MODE_OFF		    0
#define STATE_ESTIMATION_MODE_ENTER         1
#define STATE_ESTIMATION_MODE_ESTIMATE	    2

#define STATE_ESTIMATION_TYPE_GPS		        3
#define STATE_ESTIMATION_TYPE_GPS_FILTER    4
#define STATE_ESTIMATION_TYPE_GPS_AVERAGE   5
#define STATE_ESTIMATION_TYPE_PRED		      6
#define STATE_ESTIMATION_TYPE_COMP		      7
#define STATE_ESTIMATION_TYPE_KALMAN	      8

#ifndef STATE_ESTIMATION_MODE
#define STATE_ESTIMATION_MODE           STATE_ESTIMATION_MODE_OFF
#endif


typedef struct {
  Butterworth2LowPass_int x;
  Butterworth2LowPass_int y;
  Butterworth2LowPass_int z;
} Butterworth2LowPass_vect3;


struct StateFilter {

  Butterworth2LowPass_vect3 pos;

  float sample_time;
  float cut_off;
};

struct FlapFilter {
  int32_t sample_count;
  uint32_t flap_count;
  struct Int32Vect3 sum_pos;
  float sum_dt;
};


struct StateEstimation {

  uint8_t mode;
  uint8_t type;

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
};


extern struct StateEstimation state_estimation;
extern struct StateFilter state_filter;
extern struct FlapFilter average_filter;

extern void state_estimation_init(void);

extern void state_estimation_enter(void);

extern void state_estimation_run(void);


#endif /* STATE_ESTIMATION_H_ */
