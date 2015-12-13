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
 * @file /paparazzi/paparazzi/sw/airborne/modules/delfly_control/delfly_guidance.h
 * @author Torbjoern Cunis
 */

#ifndef DELFLY_GUIDANCE_H_
#define DELFLY_GUIDANCE_H_


#include "delfly_algebra_int.h"


#define DELFLY_GUIDANCE_MODE_OFF			0
#define DELFLY_GUIDANCE_MODE_MODULE			1
#define DELFLY_GUIDANCE_MODE_NAV			2
//#define DELFLY_GUIDANCE_MODE_NAV_WAYPOINT		6
//#define DELFLY_GUIDANCE_MODE_NAV_SPEED		7


struct DelflyGuidanceCommand {
  /* horizontal acceleration command
   * in m/s2, with #INT32_ACCEL_FRAC */
  int32_t h_acc;
  /* vertical acceleration command
   * in m/s2, with #INT32_ACCEL_FRAC */
  int32_t v_acc;
  /* heading angle command
   * in rad, with #INT32_ANGLE_FRAC  */
  int32_t heading;
};

struct DelflyGuidanceError {

  /* position error in geographic frame
   * in m, with #INT32_POS_FRAC      */
  struct Int32Vect3 pos;
  /* velocity error in geographic frame
   * in m/s, with #INT32_SPEED_FRAC  */
  struct Int32Vect3 vel;

  /* forward error vector
   * in m,   with #INT32_POS_FRAC;
   * in m/s, with #INT32_SPEED_FRAC  */
  union Int32VectState2 fwd;
  /* lateral error vector
   * in m,   with #INT32_POS_FRAC;
   * in m/s, with #INT32_SPEED_FRAC  */
  union Int32VectState2 lat;
  /* vertical error vector
   * in m,   with #INT32_POS_FRAC;
   * in m/s, with #INT32_SPEED_FRAC  */
  union Int32VectState2 ver;
};

struct DelflyGuidanceGains {
  /* forward gain matrix
   * with #INT32_MATLAB_FRAC         */
  union Int32VectState2 fwd;
  /* lateral gain matrix
   * with #INT32_MATLAB_FRAC         */
  union Int32VectState2 lat;
  /* forward gain matrix
   * with #INT32_MATLAB_FRAC         */
  union Int32VectState2 ver;
};

struct DelflyGuidanceSetPoint {

  /* radio-control velocity set-point
   * in m/s, with #INT32_SPEED_FRAC  */
  union Int32VectLong vel_rc;

  /* radio-control attitude set-point
   * in rad, with #INT32_ANGLE_FRAC */
  struct Int32Eulers att_rc;

  /* position set-point
   * in m, with #INT32_POS_FRAC */
  struct Int32Vect3 pos;

  /* velocity set-point
   * in m/s, with #INT32_SPEED_FRAC */
  struct Int32Vect3 vel;

  /* heading set-point
   * in rad, with #INT32_ANGLE_FRAC */
  int32_t heading;
};


struct DelflyGuidance {

  uint8_t mode;

  struct DelflyGuidanceSetPoint sp;

  struct DelflyGuidanceGains gains;
  struct DelflyGuidanceError err;
  struct DelflyGuidanceCommand cmd;
};


extern struct DelflyGuidance delfly_guidance;

extern void delfly_guidance_int (void);

extern void delfly_guidance_enter (void);
extern void delfly_guidance_run (void);


#endif /* DELFLY_GUIDANCE_H_ */
