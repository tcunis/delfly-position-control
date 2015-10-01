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

#include "generated/airframe.h"
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


struct EnuCoor_f waypoint_from;
struct EnuCoor_f velocity_vector;
struct EnuCoor_f navigation_target_f;

int8_t wp_temporary;

double time_moving;


/** Init wind-tunnel set-up */
void wind_init ( void ) {
    
    VECT3_ASSIGN( waypoint_from, 0, 0, 0 );
    VECT3_ASSIGN( velocity_vector, 0, 0, 0 );
    VECT3_ASSIGN( navigation_target_f, 0, 0, 0 );
    
    wp_temporary = 0;
    time_moving = 0.0;
}


/** Set moving waypoint / target */
void wind_setMovingWaypoint( int8_t wpTemp, int8_t wpFrom, int8_t wpTo ) {
    
    VECT3_COPY( waypoint_from, waypoints[wpFrom].enu_i );
    wp_temporary = wpTemp;
    time_moving = 0.0;
    
    struct EnuCoor_f distance_vector;
    struct EnuCoor_f distance_normed;
    
    // dist = to - from
    VECT3_DIFF( distance_vector, waypoints[wpTo].enu_i, waypoints[wpFrom].enu_i );
    // d_no = dist / |dist|
    double distance = FLOAT_VECT3_NORM(distance_vector);
    VECT3_SDIV( distance_normed, distance_vector, distance );
    // velo = d_no * V_w
    VECT3_SMUL( velocity_vector, distance_normed, windtunnel_velocity );  
    
}

void wind_move_waypoint( int16_t freq ) {
    
    time_moving += 1.0/freq;
    
    // navt = from + distance_vector*( V_w * t / |dist|)
    VECT3_SUM_SCALED( navigation_target_f, waypoint_from, velocity_vector, time_moving );
    
    if ( wp_temporary > 0 ) { // temp != DUMMY
        // temp = WP(navt)
        VECT3_COPY( waypoints[wp_temporary].enu_i, navigation_target_f );
    }
}

/** Set stay at moving waypoint */
bool_t nav_StayMovingWaypoint( int8_t wpTemp, int8_t wpFrom, int8_t wpTo ) {
    
    struct EnuCoor_f distance_vector, nav_target_f;
    struct EnuCoor_f distance_normed;
    {
    // dist = to - from
    VECT3_DIFF( distance_vector, waypoints[wpTo].enu_i, waypoints[wpFrom].enu_i );
    // d_no = dist / |dist|
    double distance = FLOAT_VECT3_NORM(distance_vector);
    VECT3_SDIV( distance_normed, distance_vector, distance );
    // velo = d_no * V_w
    VECT3_SMUL( velocity_vector, distance_normed, windtunnel_velocity );
    
    
    // navt = from + distance_vector*( V_w * t / |dist|)
    //VECT3_SUM_SCALED( nav_target_f, waypoints[wpFrom].enu_i, distance_vector, stage_time/(distance*1.0/windtunnel_velocity) );
    VECT3_SUM_SCALED( nav_target_f, waypoints[wpFrom].enu_i, velocity_vector, stage_time );
    
    // temp = WP(navt)
    VECT3_COPY( waypoints[wpTemp].enu_i, nav_target_f );
    }
    
    // goto navt
    horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
    VECT3_COPY(navigation_target, nav_target_f);
    dist2_to_wp = get_dist2_to_point_f(&nav_target_f);
    NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
    NavVerticalAltitudeMode(WaypointAlt(wpTemp), 0.);
    
    return TRUE;
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

//bool_t nav_StayMovingWaypoint( int8_t wpTemp, int8_t wpFrom, int8_t wpTo ) {
//    double lambda = stage_time / 60.0;
//    
//    // temp = (1 - lambda)*from + lambda*to
//    VECT3_SUM_SCALED( waypoints[wpTemp].enu_i, waypoints[wpFrom].enu_i, waypoints[wpFrom].enu_i, -lambda );
//    VECT3_ADD_SCALED( waypoints[wpTemp].enu_i, waypoints[wpTo].enu_i, lambda );
//    
//    // stay at temp
//    NavGotoWaypoint(wpTemp);
//    NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
//    NavVerticalAltitudeMode(WaypointAlt(wpTemp), 0.);
//    
//    return TRUE;
//}