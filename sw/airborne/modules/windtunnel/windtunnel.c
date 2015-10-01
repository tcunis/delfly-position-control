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
 * @file "modules/windtunnel/windtunnel.c"
 * @author Torbjoern Cunis
 * 
 */

#include "modules/windtunnel/windtunnel.h"

#include "subsystems/datalink/telemetry.h"

#include "generated/modules.h"
#include "generated/airframe.h"


volatile uint8_t windtunnel_velocity = WINDTUNNEL_WIND_VELOCITY;


struct EnuCoor_f waypoint_from;
struct EnuCoor_f windtunnel_velocity_vector;
struct EnuCoor_f windtunnel_navigation_target;

int8_t wp_temporary;

volatile double time_moving;



static void send_windtunneltm ( struct transport_tx* trans, struct link_device* device ) {

     DOWNLINK_SEND_WINDTUNNEL( DefaultChannel, DefaultDevice,
        (uint8_t*)&windtunnel_velocity,
        &windtunnel_velocity_vector.x,
        &windtunnel_velocity_vector.y,
        &windtunnel_navigation_target.x,
        &windtunnel_navigation_target.y,
        (double*)&time_moving );
}


/** Init wind-tunnel set-up */
void windtunnel_init ( void ) {
    
    VECT3_ASSIGN( waypoint_from, 0, 0, 0 );
    VECT3_ASSIGN( windtunnel_velocity_vector, 0, 0, 0 );
    VECT3_ASSIGN( windtunnel_navigation_target, 0, 0, 0 );
    
    wp_temporary = 0;
    time_moving = 0.0;

    register_periodic_telemetry( DefaultPeriodic, "WINDTUNNEL", send_windtunneltm );
}


/** Set moving waypoint / target */
void windtunnel_setMovingWaypoint ( int8_t wpTemp, int8_t wpFrom, int8_t wpTo ) {
    
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
    VECT3_SMUL( windtunnel_velocity_vector, distance_normed, windtunnel_velocity );
    
    VECT3_COPY( windtunnel_navigation_target, waypoint_from );
}

void windtunnel_periodic () {
    
    time_moving += WINDTUNNEL_PERIODIC_PERIOD;
    
    // navt = from + distance_vector*( V_w * t / |dist|)
    VECT3_SUM_SCALED( windtunnel_navigation_target, waypoint_from, windtunnel_velocity_vector, time_moving );
    
    if ( wp_temporary > 0 ) { // temp != DUMMY
        // temp = WP(navt)
        VECT3_COPY( waypoints[wp_temporary].enu_i, windtunnel_navigation_target );
    }
}

void windtunnel_start_periodic (void) { time_moving = 0.0; }
void windtunnel_stop_periodic (void) {}

bool_t windtunnel_set_periodic ( bool_t bStart ) {
    if ( bStart ) {
        windtunnel_windtunnel_periodic_status = MODULES_START;
    } else {
        windtunnel_windtunnel_periodic_status = MODULES_STOP;
    }
    
    return ( windtunnel_windtunnel_periodic_status == MODULES_START || windtunnel_windtunnel_periodic_status == MODULES_RUN );
}
