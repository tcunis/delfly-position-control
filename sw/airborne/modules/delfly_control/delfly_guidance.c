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
 * @file /paparazzi/paparazzi/sw/airborne/modules/delfly_control/delfly_guidance.c
 * @author Torbjoern Cunis
 */


#include "delfly_guidance.h"

#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/navigation.h"



struct DelflyGuidance delfly_guidance;


void delfly_guidance_init (void) {

  delfly_guidance.mode = DELFLY_GUIDANCE_MODE_OFF;

  INT32_ZERO(delfly_guidance.cmd.h_acc);
  INT32_ZERO(delfly_guidance.cmd.heading);
  VECT2_ZERO(delfly_guidance.err.fwd.xy);
  VECT2_ZERO(delfly_guidance.err.lat.xy);
  VECT2_ZERO(delfly_guidance.sp.vel_rc.xy);

}


void delfly_guidance_enter (void) {

  switch (autopilot_mode) {

  case AP_MODE_MODULE:
	delfly_guidance.mode = DELFLY_GUIDANCE_MODE_MODULE;
	{
	  delfly_guidance.sp.pos.z = delfly_state.v.pos;
	  delfly_guidance.sp.vel.z = delfly_state.v.vel;
	}
	break;
  case AP_MODE_NAV:
	delfly_guidance.mode = DELFLY_GUIDANCE_MODE_NAV;
	break;
  default:
	delfly_guidance.mode = DELFLY_GUIDANCE_MODE_OFF;
  }
}


void delfly_guidance_run (void) {

  switch (delfly_guidance.mode) {

  case DELFLY_GUIDANCE_MODE_MODULE:
  {
    delfly_guidance.sp.heading = delfly_guidance.sp.att_rc.psi;
    //TODO: set velocity set-point
  } break;

  case DELFLY_GUIDANCE_MODE_NAV:
  {
	delfly_guidance.sp.heading = nav_heading;
	//TODO: set position and velocity set-point
	ENU_OF_TO_NED(delfly_guidance.sp.pos, navigation_carrot);
  } break;

  default:
	//nothing to do
  }
}
