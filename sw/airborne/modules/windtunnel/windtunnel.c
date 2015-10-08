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



double windtunnel_velocity = WINDTUNNEL_WIND_VELOCITY;      // in m/s
uint8_t windtunnel_delay = WINDTUNNEL_DELAY_TIME;           // in s    


struct EnuCoor_f waypoint_from;                     // in ENU_f -- [m]^3
struct EnuCoor_f windtunnel_direction;              // in ENU_f -- [m]^3
struct EnuCoor_f windtunnel_velocity_vector;        // in ENU_f -- [m]^3
struct EnuCoor_f windtunnel_navigation_target;      // in ENU_f -- [m]^3

int8_t wp_temporary;

volatile double time_moving;                        // in s



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




static inline void updateWindVelocityVector (void) {

    // velo = direct * V_w
    VECT3_SMUL( windtunnel_velocity_vector  /*in ENU_f -- [m/s]^3*/, 
                windtunnel_direction        /*in ENU_f*/, 
                windtunnel_velocity         /*in m/s*/ );
}


bool_t windtunnel_setWindVelocity ( double velocity_new ) {

    windtunnel_velocity = velocity_new;

    if ( time_moving > 0 ) {
        updateWindVelocityVector();
        VECT3_COPY( waypoint_from, windtunnel_navigation_target );
        time_moving = 0.0;
    }
    
    return FALSE; //call one
}


/** Set moving waypoint / target */
bool_t windtunnel_setMovingWaypoint ( int8_t wpTemp, int8_t wpFrom, int8_t wpTo, double default_velocity /*= 0.5*/, bool_t set_periodic /*=FALSE*/ ) {
    
    if ( windtunnel_velocity == 0.0 )
        windtunnel_velocity = default_velocity;
    
    VECT3_COPY( waypoint_from /*in ENU_f*/, waypoints[wpFrom].enu_f );
    wp_temporary = wpTemp;
    time_moving = 0.0;
    
    struct EnuCoor_f distance_vector;       // in ENU_f -- [m]^3
//    struct EnuCoor_f distance_normed;       // in ENU_f -- [m]^3, |d_no| = 1 m
    
    // dist = to - from
    VECT3_DIFF( distance_vector /*in ENU_f*/, waypoints[wpTo].enu_f, waypoints[wpFrom].enu_f );
    // direct = dist / |dist|
    double distance /*in m*/ = FLOAT_VECT3_NORM(distance_vector /*in ENU_f*/);
    VECT3_SDIV( windtunnel_direction /*in ENU_f*/, distance_vector /*in ENU_f -- [m]^3*/, distance /*in m*/ );

    updateWindVelocityVector();
    
    VECT3_COPY( windtunnel_navigation_target /*in ENU_f*/, waypoint_from /*in ENU_f*/ );
    
    windtunnel_set_periodic( set_periodic );

    return FALSE;
}

void windtunnel_periodic () {
    
    time_moving += WINDTUNNEL_PERIODIC_PERIOD;

    if ( time_moving < 0 )
        return; //nothing to do within delay
    
    // navt = from + distance_vector*( V_w * t / |dist|)
    VECT3_SUM_SCALED( windtunnel_navigation_target /*in ENU_f -- [m]^3*/, 
                      waypoint_from /*in ENU_f -- [m]^3*/, 
                      windtunnel_velocity_vector /*in ENU_f -- [m/s]^3*/,
                      (time_moving - windtunnel_delay) /*in s*/ );
    
    if ( wp_temporary > 0 ) { // temp != DUMMY
        // temp = WP(navt)
        waypoint_set_enu( wp_temporary, &windtunnel_navigation_target /*in ENU_f*/ );
    }
}

void windtunnel_start_periodic (void) { time_moving = -windtunnel_delay; /* set delay as negative count-down */ }
void windtunnel_stop_periodic (void) { time_moving = 0.0; }

bool_t windtunnel_set_periodic ( bool_t bStart ) {
    if ( bStart ) {
        windtunnel_windtunnel_periodic_status = MODULES_START;
    } else {
        windtunnel_windtunnel_periodic_status = MODULES_STOP;
    }
    
    return FALSE;
}
