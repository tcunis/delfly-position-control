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

/* Telemetry data                                                                */
/* Position in Enu_f -- [m]^3                                                    */
extern struct EnuCoor_f wtposctrl_tm_position_ref,  /* reference position        */
                        wtposctrl_tm_position_now,  /* current position          */
                        wtposctrl_tm_position_err;  /* position error            */

/* Control status in Enu_f -- [m*s]^3, [m/s]^3, and [m/s^2]^3, resp.             */
extern struct EnuCoor_f wtposctrl_tm_position_err_int,  /* error integral        */
                        wtposctrl_tm_position_err_drv,  /* 1st error derivative  */
                        wtposctrl_tm_position_err_dv2;  /* 2nd error derivative  */

/* Velocity in Enu_f -- [m/s]^3                                                  */
extern struct EnuCoor_f wtposctrl_tm_velocity_cmd,  /* commanded velocity        */
                        wtposctrl_tm_velocity_now,  /* current velocity          */
                        wtposctrl_tm_velocity_gps;  /* current velocity (gps)    */

extern uint8_t wtposctrl_tm_posctrl_freq;

extern double wtposctrl_tm_position_err_f,
              wtposctrl_tm_position_err_int_f,
              wtposctrl_tm_position_err_drv_f,
              wtposctrl_tm_position_err_dv2_f,
              wtposctrl_tm_velocity_cmd_f,
              wtposctrl_tm_velocity_now_f;


extern void wtposctrl_init (void);

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

extern void guidance_h_module_init (void);

extern void guidance_h_module_enter (void);

extern void guidance_h_module_read_rc (void);
extern void guidance_h_module_run ( bool_t in_flight );




#endif

#if GUIDANCE_V_MODE_MODULE_SETTING == GUIDANCE_V_MODE
//#pragma warning "This module does not provide a vertical guidance mode (todo: remove)."
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_HOVER
#endif




#endif

