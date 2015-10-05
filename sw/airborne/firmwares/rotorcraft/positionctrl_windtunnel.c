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
#include "guidance.h"

#include "windtunnel/windtunnel.h"
#include "windtunnel_posctrl/windtunnel_posctrl.h"

#include "math/pprz_algebra_double.h"



#define X(_V2_)                             ((_V2_)->x)
#define Y(_V2_)                             ((_V2_)->y)
#define Z(_V3_)                             ((_V3_)->z)
#define VECT2ptr_ASSIGN(_P0_,_X_,_Y_) {     \
    X(_P0_) = (_X_);                      \
    Y(_P0_) = (_Y_);                      \
  }
#define VECT2ptr_COPY(_P0_,_P1_)            VECT2ptr_ASSIGN( _P0_, (_P1_).x, (_P1_).y )
#define VECT2_DIFFptr(_P0_, _P1_, _P2_)     VECT2_ASSIGN( _P0_, X(_P1_)-X(_P2_), Y(_P1_)-Y(_P2_) )

#define VECT2_SPEEDS_BFP_OF_REAL(_ef, _ei) {      \
    (_ef).x = SPEED_BFP_OF_REAL((_ei).x);   \
    (_ef).y = SPEED_BFP_OF_REAL((_ei).y);   \
  }
#define VECT2ptr_SPEEDS_BFP_OF_REAL(_ef, _ei) {      \
    X(_ef) = SPEED_BFP_OF_REAL((_ei).x);   \
    Y(_ef) = SPEED_BFP_OF_REAL((_ei).y);   \
  }


//volatile uint8_t windtunnel_velocity = POSITIONCTRL_WINDTUNNEL_WIND_VELOCITY;

double posctrl_vel_Kp = POSITIONCTRL_VELOCITY_PGAIN;
double posctrl_vel_Ki = POSITIONCTRL_VELOCITY_IGAIN;


struct Int32Vect2 velocity_ref; ///< with INT32_SPEED_FRAC

struct DoubleVect3 scaled_position_error_int; // integral of scaled position errors in Enu_f -- [m*s]^3


static double get_dist2_to_point_f ( struct EnuCoor_f* p /*in ENU_f -- [m]^3*/ );
static double control_velocity ( struct Int32Vect2* velocity_ref /*in ENU_i -- [m]^2 with INT32_SPEED_FRAC*/, 
                                 struct EnuCoor_f position_ref, struct EnuCoor_f position_now /*both in ENU_f -- [m]^3*/ );



/** Set stay at wind-tunnel navigation target */
bool_t nav_StayWtNavTarget( int8_t wpTemp ) {
    
    // goto navt
    horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
    ENU_BFP_OF_REAL(navigation_target /*in ENU_i*/, windtunnel_navigation_target /*in ENU_f*/);
    dist2_to_wp = get_dist2_to_point_f(&windtunnel_navigation_target);
    NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
    NavVerticalAltitudeMode(WaypointAlt(wpTemp), 0.);
    
    return TRUE;
}


static void setVelocityRef( int8_t wpTemp, struct Int32Vect2 velocity_i /*in ENU_i, with INT32_SPEED_FRAC */, int32_t heading_i /*in rad, with INT32_ANGLE_FRAC */ ) {

    wtposctrl_set_heading( heading_i /*in rad, with INT32_ANGLE_FRAC */ );
    wtposctrl_set_velocity_ref( velocity_i /*in ENU_i -- [m]^2, with INT32_SPEED_FRAC*/ );
    guidance_h_mode_changed( GUIDANCE_H_MODE_MODULE );

    NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
    NavVerticalAltitudeMode(WaypointAlt(wpTemp), 0.);
}


/** Set velocity control reference. */
bool_t nav_SetVelocityRef( int8_t wpTemp, double velocity_x, double velocity_y, double heading ) {

    // for test only -- set velocity to windtunnel_velocity in x, 0 in y
    struct DoubleVect2 velocity_f;     // in ENU_f -- [m/s]^2
    VECT2_ASSIGN( velocity_f, velocity_x /*in m/s*/, velocity_y /*in m/s*/ );
    VECT2_SPEEDS_BFP_OF_REAL( velocity_ref /*in ENU_i*/, velocity_f );  // convert REAL (ENU_f) into BFP (ENU_i) with INT32_SPEED_FRAC

    setVelocityRef( wpTemp, velocity_ref, ANGLE_BFP_OF_REAL(heading) );

    return TRUE;
}


/** Set follow wind-tunnel navigation target
    Unlike StayWtNavTarget(), this uses a velocity P controller
    instead of NavGotoWaypoint(). */
bool_t nav_FollowWtNavTarget( int8_t wpTemp ) {

//    wtposctrl_tm_velocity_cmd_f = control_velocity( &velocity_ref, windtunnel_navigation_target, *stateGetPositionEnu_f() );

    // for debug: use temp waypoint for position control
    wtposctrl_tm_velocity_cmd_f = control_velocity( &velocity_ref, waypoints[wpTemp].enu_f, *stateGetPositionEnu_f() );


    int32_t heading_i = int32_atan2( velocity_ref.y, velocity_ref.x );

    setVelocityRef( wpTemp, velocity_ref, heading_i );

    //VECT2_SPEEDS_BFP_OF_REAL(guidance_h_pos_sp /*in ENU_i with INT32_POS_FRAC*/, windtunnel_navigation_target /*in ENU_f*/); // for display only
    //ENU_BFP_OF_REAL(navigation_target /*in ENU_i*/, windtunnel_navigation_target /*in ENU_f*/);
    
    return TRUE;
}


/** Returns squared horizontal distance in [m^2] to given point in [m]^3 */
static double get_dist2_to_point_f ( struct EnuCoor_f *p /*in ENU_f -- [m]^3*/ ) {

    struct EnuCoor_f *pos = stateGetPositionEnu_f();    // in ENU_f -- [m]^3;
    struct DoubleVect2 pos_diff;                        // [m]^2
    VECT2_DIFFptr( pos_diff, p, pos );                  // diff = p - pos

    return VECT2_NORM2( pos_diff );                     // return |diff|^2 -- [m^2]
}


static double control_velocity ( struct Int32Vect2* velocity_ref /*in ENU_i -- [m]^2 with INT32_SPEED_FRAC*/, 
                          struct EnuCoor_f position_ref, struct EnuCoor_f position_now /*both in ENU_f -- [m]^3*/ 
                        ) {
    struct DoubleVect3 position_diff, velocity_f; // in ENU_f -- [m]^3 and [m/s]^3, resp.
    VECT2_DIFF( position_diff, position_now, position_ref );    // diff = now - ref
    position_diff.z = 0; // regret z-axis error here
    
    // vel = Kp*diff + sum(Ki*diff)[0:t]
    VECT3_SMUL( velocity_f, position_diff, posctrl_vel_Kp );

    if ( posctrl_vel_Ki == 0 ) {
        // sum(Ki*diff) = 0
        VECT3_ASSIGN( scaled_position_error_int, 0, 0, 0 );
    } else {
        // sum(Ki*diff)[0:t] = sum(Ki*diff)[0:t-1] + Ki*diff(t)
        VECT3_ADD_SCALED( scaled_position_error_int, position_diff, posctrl_vel_Ki );
    }
    VECT3_ADD( velocity_f, scaled_position_error_int );

    VECT2ptr_SPEEDS_BFP_OF_REAL( velocity_ref, velocity_f );    // convert REAL (ENU_f) into BFP (ENU_i) with INT32_SPEED_FRAC

    /* set telemetry data */
    VECT2_COPY( wtposctrl_tm_position_now, position_now );
    VECT2_COPY( wtposctrl_tm_position_ref, position_ref );

    VECT2_COPY( wtposctrl_tm_velocity_cmd, velocity_f );
    
    return double_vect3_norm( &velocity_f );   // return |vel| -- [m]
}


