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
 * @file "modules/windtunnel_posctrl/windtunnel_posctrl.c"
 * @author Torbjoern Cunis
 * 
 */

#include "modules/windtunnel_posctrl/windtunnel_posctrl.h"

#include "subsystems/datalink/telemetry.h"
#include "subsystems/gps.h"

#include "math/pprz_algebra_int.h"



#define VECT2_SPEEDS_FLOAT_OF_BFP(_ef, _ei) {      \
    (_ef).x = SPEED_FLOAT_OF_BFP((_ei).x);   \
    (_ef).y = SPEED_FLOAT_OF_BFP((_ei).y);   \
  }


struct Int32Vect2 wtposctrl_guidance_h_speed_sp;    // in NED_i (!), with #INT32_SPEED_FRAC

int32_t wtposctrl_heading;                          // in rad, with #INT32_ANGLE_FRAC

/* Position in Enu_f -- [m]^3                                           */
struct EnuCoor_f wtposctrl_tm_position_ref,
                 wtposctrl_tm_position_now,
                 wtposctrl_tm_position_err;

/* Control status in Enu_f -- [m*s]^3, [m/s]^3, and [m/s^2]^3, resp.    */
struct EnuCoor_f wtposctrl_tm_position_err_int,
                 wtposctrl_tm_position_err_drv,
                 wtposctrl_tm_position_err_dv2;

/* Velocity in Enu_f -- [m/s]^3                                         */
struct EnuCoor_f wtposctrl_tm_velocity_cmd,
                 wtposctrl_tm_velocity_now,
                 wtposctrl_tm_velocity_gps;

uint8_t wtposctrl_tm_posctrl_freq;

double wtposctrl_tm_position_err_f,
       wtposctrl_tm_position_err_int_f,
       wtposctrl_tm_position_err_drv_f,
       wtposctrl_tm_position_err_dv2_f,
       wtposctrl_tm_velocity_cmd_f,
       wtposctrl_tm_velocity_now_f;


/*      in NED_f -- m                   */
struct NedCoor_f velocity_gps_ned_f;


static void send_posctrltm ( struct transport_tx* trans, struct link_device* device ) {

    DOWNLINK_SEND_POS_CTRL( DefaultChannel, DefaultDevice,
    /*                  position control                            */    
    /* pos_ref_x = */ &wtposctrl_tm_position_ref.x /* float in m    */,
    /* pos_ref_y = */ &wtposctrl_tm_position_ref.y /* float in m    */,
    /* pos_now_x = */ &wtposctrl_tm_position_now.x /* float in m    */,
    /* pos_now_y = */ &wtposctrl_tm_position_now.y /* float in m    */,
    /* pos_dif   = */ &wtposctrl_tm_position_err_f /* double in m   */,
    /* pos_dif_x = */ &wtposctrl_tm_position_err.x /* float in m    */,
    /* pos_dif_y = */ &wtposctrl_tm_position_err.y /* float in m    */,
    /*                  position control: status                              */
    /* pos_ctrl_freq = */ &wtposctrl_tm_posctrl_freq       /* int in 1/s      */,
    /* pos_dif_int   = */ &wtposctrl_tm_position_err_int_f /* double in m*s   */,
    /* pos_dif_int_x = */ &wtposctrl_tm_position_err_int.x /* float in m*s    */,
    /* pos_dif_int_y = */ &wtposctrl_tm_position_err_int.y /* float in m*s    */,
    /* pos_dif_drv   = */ &wtposctrl_tm_position_err_drv_f /* double in m/s   */,
    /* pos_dif_drv_x = */ &wtposctrl_tm_position_err_drv.x /* float in m/s    */,
    /* pos_dif_drv_y = */ &wtposctrl_tm_position_err_drv.y /* float in m/s    */,
    /* pos_dif_dv2   = */ &wtposctrl_tm_position_err_dv2_f /* double in m/s^2 */,    
    /* pos_dif_dv2_x = */ &wtposctrl_tm_position_err_dv2.x /* float in m/s^2  */,
    /* pos_dif_dv2_y = */ &wtposctrl_tm_position_err_dv2.y /* float in m/s^2  */,
    /*                  position / velocity control                 */
    /* vel_cmd   = */ &wtposctrl_tm_velocity_cmd_f /* double in m/s */,
    /* vel_cmd_x = */ &wtposctrl_tm_velocity_cmd.x /* float in m/s  */,
    /* vel_cmd_y = */ &wtposctrl_tm_velocity_cmd.y /* float in m/s  */,
    /*                  velocity control                            */
    /* vel_now   = */ &wtposctrl_tm_velocity_now_f /* double in m/s */,
    /* vel_now_x = */ &wtposctrl_tm_velocity_now.x /* float in m/s  */,
    /* vel_now_y = */ &wtposctrl_tm_velocity_now.y /* float in m/s  */,
    /* vel_gps_x = */ &wtposctrl_tm_velocity_gps.x /* float in m/s  */,
    /* vel_gps_y = */ &wtposctrl_tm_velocity_gps.y /* float in m/s  */
    );
}


void wtposctrl_init (void) {

    INT_VECT2_ZERO( wtposctrl_guidance_h_speed_sp );
    wtposctrl_heading = 0;

    INT_VECT3_ZERO( wtposctrl_tm_position_ref );
    INT_VECT3_ZERO( wtposctrl_tm_position_now );
    INT_VECT3_ZERO( wtposctrl_tm_position_err );
    INT_VECT3_ZERO( wtposctrl_tm_position_err_int );
    INT_VECT3_ZERO( wtposctrl_tm_position_err_drv );
    INT_VECT3_ZERO( wtposctrl_tm_position_err_dv2 );
    INT_VECT3_ZERO( wtposctrl_tm_velocity_cmd );
    INT_VECT3_ZERO( wtposctrl_tm_velocity_now );
    INT_VECT3_ZERO( wtposctrl_tm_velocity_gps );
    wtposctrl_tm_position_err_f     = 0.0;
    wtposctrl_tm_position_err_int_f = 0.0;
    wtposctrl_tm_position_err_drv_f = 0.0;
    wtposctrl_tm_position_err_dv2_f = 0.0;
    wtposctrl_tm_velocity_cmd_f     = 0.0;
    wtposctrl_tm_velocity_now_f     = 0.0;

#if GUIDANCE_H_MODE_MODULE_SETTING == GUIDANCE_H_MODE_MODULE
    guidance_h_module_init();
#endif

    register_periodic_telemetry( DefaultPeriodic, "POS_CTRL", send_posctrltm );
}


void wtposctrl_set_velocity_ref ( struct Int32Vect2 velocity_2d /*in ENU_i, with INT32_SPEED_FRAC */ ) {

    INT32_VECT2_NED_OF_ENU( wtposctrl_guidance_h_speed_sp /*NED_i*/, velocity_2d /*ENU_i*/ );

    /* telemetry only -- written in positionctrl_windtunnel.c */
    VECT2_SPEEDS_FLOAT_OF_BFP( wtposctrl_tm_velocity_cmd, velocity_2d );
    wtposctrl_tm_velocity_cmd_f = FLOAT_VECT3_NORM( wtposctrl_tm_velocity_cmd );
}

void wtposctrl_set_heading ( int32_t heading /*in rad, with INT32_ANGLE_FRAC */ ) {
    
    wtposctrl_heading = heading;
}


#if GUIDANCE_H_MODE_MODULE_SETTING == GUIDANCE_H_MODE_MODULE


#include "generated/airframe.h"

#include "guidance/guidance_h.h"
#include "state.h"

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_none.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

#include "firmwares/rotorcraft/navigation.h"


static inline void reset_guidance_reference_from_current_position (void);
static void guidance_h_update_reference (void);
static void guidance_h_traj_run ( bool_t in_flight );

static inline void guidance_h_set_velocity_tm (void);


void guidance_h_module_init (void) { /*nothing to do yet*/ }


/* static definitions and functions, re-use from guidance/guidance_h.c */
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "subsystems/radio_control.h"


/* for guidance_v_thrust_coeff */
#include "firmwares/rotorcraft/guidance/guidance_v.h"


#ifndef GUIDANCE_H_MAX_BANK
#define GUIDANCE_H_MAX_BANK RadOfDeg(20)
#endif

#define MAX_POS_ERR   POS_BFP_OF_REAL(16.)
#define MAX_SPEED_ERR SPEED_BFP_OF_REAL(16.)

#ifndef GUIDANCE_H_THRUST_CMD_FILTER
#define GUIDANCE_H_THRUST_CMD_FILTER 10
#endif

/* with a pgain of 100 and a scale of 2,
 * you get an angle of 5.6 degrees for 1m pos error */
#define GH_GAIN_SCALE 2


static inline void guidance_h_set_velocity_tm (void) {

    /* current velocity */
    VECT2_COPY( wtposctrl_tm_velocity_now, *stateGetSpeedEnu_f() );
    wtposctrl_tm_velocity_now_f = FLOAT_VECT3_NORM( wtposctrl_tm_velocity_now );

    /* current velocity (gps) */
    ECEF_FLOAT_OF_BFP( velocity_gps_ned_f /* NED_f in m/s */, gps.ned_vel /* NED_i in cm/s */ );
    INT32_VECT2_ENU_OF_NED( wtposctrl_tm_velocity_gps /* ENU_f */, velocity_gps_ned_f /*NED_f */ );
}

static inline void reset_guidance_reference_from_current_position(void)
{
  VECT2_COPY(guidance_h_pos_ref, *stateGetPositionNed_i());
  VECT2_COPY(guidance_h_speed_ref, *stateGetSpeedNed_i());
  INT_VECT2_ZERO(guidance_h_accel_ref);
  gh_set_ref(guidance_h_pos_ref, guidance_h_speed_ref, guidance_h_accel_ref);

  INT_VECT2_ZERO(guidance_h_trim_att_integrator);
}


static void guidance_h_update_reference(void)
{
    gh_update_ref_from_speed_sp(wtposctrl_guidance_h_speed_sp);

    /* either use the reference or simply copy the pos setpoint */
    if (guidance_h_use_ref) {
        /* convert our reference to generic representation */
        INT32_VECT2_RSHIFT(guidance_h_pos_ref,   gh_ref.pos, (GH_POS_REF_FRAC - INT32_POS_FRAC));
        INT32_VECT2_LSHIFT(guidance_h_speed_ref, gh_ref.speed, (INT32_SPEED_FRAC - GH_SPEED_REF_FRAC));
        INT32_VECT2_LSHIFT(guidance_h_accel_ref, gh_ref.accel, (INT32_ACCEL_FRAC - GH_ACCEL_REF_FRAC));
    } else {
        VECT2_COPY(guidance_h_pos_ref, guidance_h_pos_sp);
        INT_VECT2_ZERO(guidance_h_speed_ref);
        INT_VECT2_ZERO(guidance_h_accel_ref);
    }

//    VECT2_COPY(guidance_h_pos_sp, guidance_h_pos_ref); // for display only
}


static void guidance_h_traj_run(bool_t in_flight)
{
  /* maximum bank angle: default 20 deg, max 40 deg*/
  static const int32_t traj_max_bank = Min(BFP_OF_REAL(GUIDANCE_H_MAX_BANK, INT32_ANGLE_FRAC),
                                       BFP_OF_REAL(RadOfDeg(40), INT32_ANGLE_FRAC));
  static const int32_t total_max_bank = BFP_OF_REAL(RadOfDeg(45), INT32_ANGLE_FRAC);

//  /* compute position error    */
//  VECT2_DIFF(guidance_h_pos_err, guidance_h_pos_ref, *stateGetPositionNed_i());
//  /* saturate it               */
//  VECT2_STRIM(guidance_h_pos_err, -MAX_POS_ERR, MAX_POS_ERR);

  /* compute speed error    */
  VECT2_DIFF(guidance_h_speed_err, guidance_h_speed_ref, *stateGetSpeedNed_i());
  /* saturate it               */
  VECT2_STRIM(guidance_h_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);

  /* run PID */
  int32_t pd_x =
//    ((guidance_h_pgain * guidance_h_pos_err.x) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_h_dgain * (guidance_h_speed_err.x >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
  int32_t pd_y =
//    ((guidance_h_pgain * guidance_h_pos_err.y) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_h_dgain * (guidance_h_speed_err.y >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
  guidance_h_cmd_earth.x =
    pd_x +
    ((guidance_h_vgain * guidance_h_speed_ref.x) >> 17) + /* speed feedforward gain */
    ((guidance_h_again * guidance_h_accel_ref.x) >> 8);   /* acceleration feedforward gain */
  guidance_h_cmd_earth.y =
    pd_y +
    ((guidance_h_vgain * guidance_h_speed_ref.y) >> 17) + /* speed feedforward gain */
    ((guidance_h_again * guidance_h_accel_ref.y) >> 8);   /* acceleration feedforward gain */

  /* trim max bank angle from PD */
  VECT2_STRIM(guidance_h_cmd_earth, -traj_max_bank, traj_max_bank);

  /* Update pos & speed error integral, zero it if not in_flight.
   * Integrate twice as fast when not only POS but also SPEED are wrong,
   * but do not integrate POS errors when the SPEED is already catching up.
   */
  if (in_flight) {
    /* ANGLE_FRAC (12) * GAIN (8) * LOOP_FREQ (9) -> INTEGRATOR HIGH RES ANGLE_FRAX (28) */
    guidance_h_trim_att_integrator.x += (guidance_h_igain * pd_x);
    guidance_h_trim_att_integrator.y += (guidance_h_igain * pd_y);
    /* saturate it  */
    VECT2_STRIM(guidance_h_trim_att_integrator, -(traj_max_bank << 16), (traj_max_bank << 16));
    /* add it to the command */
    guidance_h_cmd_earth.x += (guidance_h_trim_att_integrator.x >> 16);
    guidance_h_cmd_earth.y += (guidance_h_trim_att_integrator.y >> 16);
  } else {
    INT_VECT2_ZERO(guidance_h_trim_att_integrator);
  }

  /* compute a better approximation of force commands by taking thrust into account */
  if (guidance_h_approx_force_by_thrust && in_flight) {
    static int32_t thrust_cmd_filt;
    int32_t vertical_thrust = (stabilization_cmd[COMMAND_THRUST] * guidance_v_thrust_coeff) >> INT32_TRIG_FRAC;
    thrust_cmd_filt = (thrust_cmd_filt * GUIDANCE_H_THRUST_CMD_FILTER + vertical_thrust) /
                      (GUIDANCE_H_THRUST_CMD_FILTER + 1);
    guidance_h_cmd_earth.x = ANGLE_BFP_OF_REAL(atan2f((guidance_h_cmd_earth.x * MAX_PPRZ / INT32_ANGLE_PI_2),
                             thrust_cmd_filt));
    guidance_h_cmd_earth.y = ANGLE_BFP_OF_REAL(atan2f((guidance_h_cmd_earth.y * MAX_PPRZ / INT32_ANGLE_PI_2),
                             thrust_cmd_filt));
  }

  VECT2_STRIM(guidance_h_cmd_earth, -total_max_bank, total_max_bank);
}

/* End definition of static functions. */


void guidance_h_module_enter (void) {

    /* set horizontal speed setpoint to current speed */
    VECT2_COPY(wtposctrl_guidance_h_speed_sp, *stateGetSpeedNed_i());
    reset_guidance_reference_from_current_position();
    wtposctrl_heading = stateGetNedToBodyEulers_i()->psi;

#if NO_ATTITUDE_RESET_ON_MODE_CHANGE
    /* reset attitude stabilization if previous mode was not using it */
    if (guidance_h_mode == GUIDANCE_H_MODE_KILL ||
        guidance_h_mode == GUIDANCE_H_MODE_RATE ||
        guidance_h_mode == GUIDANCE_H_MODE_RC_DIRECT)
#endif
        stabilization_attitude_enter();
}


void guidance_h_module_read_rc (void) {
    //not yet implemented
}


void guidance_h_module_run ( bool_t in_flight ) {
    if (!in_flight)
        guidance_h_module_enter();

    //anyway
    //INT32_VECT2_NED_OF_ENU(wtposctrl_guidance_h_speed_sp, navigation_carrot);

    guidance_h_update_reference();

    /* set psi command */
    guidance_h_heading_sp = wtposctrl_heading;
    INT32_ANGLE_NORMALIZE(guidance_h_heading_sp);
    /* compute x,y earth commands */
    guidance_h_traj_run(in_flight);

    guidance_h_set_velocity_tm();

    /* set final attitude setpoint */
    stabilization_attitude_set_earth_cmd_i(&guidance_h_cmd_earth,
                                           guidance_h_heading_sp);
    stabilization_attitude_run(in_flight);
} 

#endif
