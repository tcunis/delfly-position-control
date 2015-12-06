/*
 * Copyright (C) 2015 by Torbjoern Cunis <t.cunis@tudelft.nl>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file firmwares/rotorcraft/guidance/guidance_lat.c
 *  Lateral guidance for the DelFly.
 *
 */

#include "generated/airframe.h"

#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_lat.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/radio_control.h"

#include "firmwares/rotorcraft/stabilization/stabilization_none.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

#include "state.h"

/* warning if guidance_indi is used in parallel */
#ifdef GUIDANCE_INDI
#if GUIDANCE_INDI
#warning "GUIDANCE_INDI and GUIDANCE_DELFLY are exclusive. Won't use lateral guidance."
#endif
#endif

/* error if some gains are negative */
#if (GUIDANCE_LAT_PGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif

#if (!GUIDANCE_DELFLY)
#warning "Won't use delfly guidance system."
#endif


#ifndef GUIDANCE_LAT_MAX_HEAD_CORR
#define GUIDANCE_LAT_MAX_HEAD_CORR RadOfDeg(45)
#endif


struct LateralGuidance guidance_lat;


void guidance_lat_init(void) {

  guidance_lat.gains.p = GUIDANCE_LAT_PGAIN;
}


/* with a pgain of 100 and a scale of 3, fraction of 10,
 * you g an angle of 0.3 rad (~16.5 deg) for 1m lateral pos error. */
#define GL_GAIN_SCALE  3
#define GL_GAIN_FRAC  10

void guidance_lat_adjust_heading(bool_t in_flight, int32_t* cmd_heading, struct Int32Vect2 h_pos_err) {

  /* lateral offset (error) to trajectory along heading sp -- with #INT32_POS_FRAC */
  int32_t virtual_y_err = (  h_pos_err.x*pprz_itrig_sin(guidance_h.sp.heading)
  	  	  	  	  	  	   + h_pos_err.y*pprz_itrig_sin(guidance_h.sp.heading)
  	  	  	  	  	  	  )/(1<<INT32_TRIG_FRAC);

  /* heading correction w.r.t. sp (rad) in order to reduce lateral error -- with #INT32_ANGLE_FRAC */
  int32_t heading_correct = virtual_y_err*guidance_lat.gains.p*GL_GAIN_SCALE/(1<<(GL_GAIN_FRAC+INT32_POS_FRAC-INT32_ANGLE_FRAC));

  /* trim */
  static const int32_t adj_max_corr = ANGLE_BFP_OF_REAL(GUIDANCE_LAT_MAX_HEAD_CORR);
  heading_correct = (heading_correct >  adj_max_corr) ?  adj_max_corr : (
		  	  	  	(heading_correct < -adj_max_corr) ? -adj_max_corr : heading_correct );

  /* set the heading command */
  *cmd_heading =  guidance_h.sp.heading + heading_correct; //in radians
}


void guidance_lat_enter(void) {
	//nothing to do yet.
}
