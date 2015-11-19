/*
 * Copyright (C) 2015 Torbjoern Cunis <poinix@gmail.com>
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

/** @file firmwares/rotorcraft/guidance/guidance_h.h
 *  Lateral guidance for the DelFly extending rotorcraft guidance_h.h.
 *
 */

#ifndef GUIDANCE_L_H
#define GUIDANCE_L_H


#include "math/pprz_algebra_int.h"

#include "firmwares/rotorcraft/guidance/guidance_h_ref.h"
#include "generated/airframe.h"
#include "std.h"


struct LateralGuidanceGains {
  int32_t p;
};

struct LateralGuidance {
  struct LateralGuidanceGains gains;
};

extern struct LateralGuidance guidance_lat;

extern void guidance_lat_init(void);
extern void guidance_lat_enter(void);

extern void guidance_lat_adjust_heading(bool_t, int32_t* cmd_heading,
												struct Int32Vect2 h_pos_err );

#endif /* GUIDANCE_L_H */
