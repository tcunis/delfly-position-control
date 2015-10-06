/*
 * Copyright (C) Torbjoern Cunis
 *
 * This file is part of paparazzi
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
 * @file "modules/windtunnel_posctrl/windtunnel_posctrl.h"
 * @author Torbjoern Cunis
 * 
 */

#ifndef WINDTUNNEL_POSCTRL_H
#define WINDTUNNEL_POSCTRL_H


#include "math/pprz_algebra_int.h"
#include "std.h"

/* Telemetry data */
extern struct EnuCoor_f wtposctrl_tm_position_ref,
                        wtposctrl_tm_position_now,
                        wtposctrl_tm_position_dif;
extern struct EnuCoor_f wtposctrl_tm_velocity_cmd,
                        wtposctrl_tm_velocity_now,
                        wtposctrl_tm_velocity_gps;
extern double wtposctrl_tm_velocity_cmd_f,
              wtposctrl_tm_velocity_now_f;


extern void wtposctrl_init ();

extern void wtposctrl_set_velocity_ref ( struct Int32Vect2 velocity_2d /*in ENU_i, with INT32_SPEED_FRAC */ );
extern void wtposctrl_set_heading ( int32_t heading /*in rad, with INT32_ANGLE_FRAC */ );


/* Implementing guidance_module interface (cf. guidance/guidance_module.h).
 * 
 * Set GUIDANCE_H_MODE_MODULE_SETTING to GUIDANCE_H_MODE in order to enable.
 *
 * Note that by now this module does not provide a vertical guidance controller
 * as well!
 */
#if GUIDANCE_H_MODE_MODULE_SETTING != GUIDANCE_H_MODE_MODULE
#warning "GUIDANCE_H_MODE_MODULE is not enabled (todo: remove)."
//#elif GUIDANCE_H_USE_SPEED_REF != TRUE
//#define GUIDANCE_H_MODE_MODULE_SETTING = GUIDANCE_H_MODE_HOVER //disable H_MODE_MODULE w/o H_USE_SPEED_REF!
//#error "GUIDANCE_H_USE_SPEED_REF has to be set in order to use this module's guidance mode."
#else

inline void guidance_h_module_init (void) { /*nothing to do yet*/ }

extern void guidance_h_module_enter (void);

extern void guidance_h_module_read_rc (void);
extern void guidance_h_module_run ( bool_t in_flight );




#endif

#if GUIDANCE_V_MODE_MODULE_SETTING == GUIDANCE_V_MODE
//#pragma warning "This module does not provide a vertical guidance mode (todo: remove)."
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_HOVER
#endif




#endif

