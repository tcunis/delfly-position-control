/*
 * Copyright (C) Torbjoern Cunis <t.cunis@tudelft.nl>
 * MAVLab Delft University of Technology
 *
 * This sub-system provides control modes in order to control the position
 * flying in a wind-tunnel; as well as to test the desired solution outside.
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
 * MERCHANTABILITY or  * FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 * 
 * @file positionctrl_windtunnel.h
 */

#ifndef POSITIONCTRL_WINDTUNNEL_H
#define POSITIONCTRL_WINDTUNNEL_H

#include "std.h"

#include "math/pprz_geodetic_float.h"


extern double posctrl_pos_Kp;
extern double posctrl_pos_Ki;
extern double posctrl_pos_Kd;
extern double posctrl_pos_Kd2;


extern bool_t nav_InitPosControl (void);

extern bool_t nav_StopPosControl (void);


/*********** Windtunnel steps ********************************************/
extern bool_t nav_ShiftWaypointRelativeEnu_f ( int8_t wp, double x, double y, double z );


/*********** Moving waypoint *********************************************/
extern bool_t nav_StayWtNavTarget( int8_t wpTemp );

extern bool_t nav_FollowWtNavTarget( int8_t wpTemp );

extern bool_t nav_SetVelocityRef( int8_t wpTemp, double velocity_x, double velocity_y, double heading );


#endif //POSITIONCTRL_WINDTUNNEL_H
