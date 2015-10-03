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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 * @file positionctrl_windtunnel.c
 */


#include "firmwares/rotorcraft/positionctrl_windtunnel.h"

//#include "generated/airframe.h"
#include "state.h"

#include "navigation.h"

#include "windtunnel/windtunnel.h"


//volatile uint8_t windtunnel_velocity = POSITIONCTRL_WINDTUNNEL_WIND_VELOCITY;


/** Returns squared horizontal distance to given point */
float get_dist2_to_point_f(struct EnuCoor_f *p)
{
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  struct FloatVect2 pos_diff;
  pos_diff.x = POS_FLOAT_OF_BFP(p->x) - pos->x;
  pos_diff.y = POS_FLOAT_OF_BFP(p->y) - pos->y;
  //return pos_diff.x * pos_diff.x + pos_diff.y * pos_diff.y;
  return VECT2_NORM2( pos_diff );
}



/** Set stay at wind-tunnel navigation target */
bool_t nav_StayWtNavTarget( int8_t wpTemp ) {
    
    // goto navt
    horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
    VECT3_COPY(navigation_target, windtunnel_navigation_target);
    dist2_to_wp = get_dist2_to_point_f(&windtunnel_navigation_target);
    NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
    NavVerticalAltitudeMode(WaypointAlt(wpTemp), 0.);
    
    return TRUE;
}


/** Set follow wind-tunnel navigation target
    Unlike StayWtNavTarget(), this uses a velocity P controller
    instead of NavGotoWaypoint(). */
bool_t nav_FollowWtNavTarget( int8_t wpTemp ) {

    nav_set_heading_towards( windtunnel_navigation_target.x, windtunnel_navigtion_target.y );
    

    horizontal_mode = HORIZONTAL_MODE_ATTITUDE;
    guidance_h_mode_changed( GUIDANCE_H_MODE_HOVER );
    NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
    NavVerticalAltitudeMode(WaypointAlt(wpTemp), 0.);
}


