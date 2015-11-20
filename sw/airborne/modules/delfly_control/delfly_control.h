/*
 * Copyright (C) cunis
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
 * @file "modules/delfly_control/delfly_control.h"
 * @author cunis
 * 
 */

#ifndef DELFLY_CONTROL_H
#define DELFLY_CONTROL_H


#include "std.h"


/*    general module functions      */
extern void delfly_control_init(void);
extern void delfly_control_start(void);
extern void delfly_control_stop(void);

/*    delfly control periodic       */
extern void delfly_control_run(void);


/*    speed/thrust control          */
extern void speed_control_start(void);
extern void speed_control_stop(void);

extern void speed_control_enter(void);
extern void speed_control_run(bool_t in_flight);


/*    guidance control h/v          */
extern void guidance_control_start(void);
extern void guidance_control_stop(void);

extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(bool_t);
extern void guidance_h_module_run(bool_t);

extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool_t);


#endif

