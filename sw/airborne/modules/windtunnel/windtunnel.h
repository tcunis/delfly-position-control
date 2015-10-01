/*
 * Copyright (C) Torbjoern Cunis &lt;t.cunis@tudelft.nl>&gt;
 * MAVLab Delft University of Technology
 * 
 * This module provides (periodic) functions to simulate flying in a 
 * wind-tunnel outside.
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
 * Wind-tunnel simulation module.
 * 
 * @file "modules/windtunnel/windtunnel.h"
 * @author Torbjoern Cunis
 */

#ifndef WINDTUNNEL_H
#define WINDTUNNEL_H

#include "std.h"
#include "subsystems/navigation/waypoints.h"


extern volatile uint8_t windtunnel_velocity;

extern struct EnuCoor_f windtunnel_navigation_target;


extern void windtunnel_init (void);
extern void windtunnel_setMovingWaypoint( int8_t wpTemp, int8_t wpFrom, int8_t wpTo );

extern bool_t windtunnel_set_periodic ( bool_t );

extern void windtunnel_periodic (void);

extern void windtunnel_start_periodic (void);
extern void windtunnel_stop_periodic (void);


#endif

