/*
 * Copyright (C) Torbjoern Cunis <t.cunis@tudelft.nl>
 *
 * Guidance h/v submodule implements guidance_module.h in order to
 * control vertical and horizontal position and velocity.
 *
 * This file is part of paparazzi:
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
 * @file "modules/delfly_control/guidance_h_module.c"
 * @author Torbjoern Cunis
 */

#include "delfly_control.h"

#include "generated/airframe.h"

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

#include "delfly_state.h"
#include "speed_control.h"

#include "guidance/guidance_h.h"
#include "guidance/guidance_lat.h"


/* horizontal acceleration command
 * in m/s2, with #INT32_ACCEL_FRAC */
int32_t guidance_cmd_h_accelerate;

/* heading command
 * in rad, with #INT32_ANGLE_FRAC  */
int32_t guidance_cmd_heading;

/* horizontal position and velocity error
 * in m, with #INT32_POS_FRAC;
 * in m/s, with #INT32_VEL_FRAC;
 */
struct Int32Vect2 guidance_h_module_pos_err;
struct Int32Vect2 guidance_h_module_vel_err;


static void guidance_h_module_run_traj( bool_t, int32_t* cmd_accelerate, int32_t* cmd_heading );


void guidance_h_module_init() {
	//nothing to do yet
}

void guidance_h_module_enter() {

	guidance_cmd_h_accelerate = 0;
	guidance_cmd_heading = 0;
}


void guidance_h_module_read_rc(bool_t in_flight) {

    stabilization_attitude_read_rc_setpoint_eulers(&guidance_h.rc_sp, in_flight, FALSE, FALSE);
#if GUIDANCE_H_USE_SPEED_REF
    read_rc_setpoint_speed_i(&guidance_h.sp.speed, in_flight);
#endif
}


void guidance_h_module_run(bool_t in_flight) {

	guidance_h_module_run_traj(in_flight, &guidance_cmd_h_accelerate, &guidance_cmd_heading);

	speed_control_set_cmd_h(guidance_cmd_h_accelerate, guidance_cmd_heading);
}


void guidance_h_module_run_traj( bool_t in_flight, int32_t* cmd_accelerate, int32_t* cmd_heading ) {

	/* compute position error    */
	VECT2_DIFF(guidance_h_module_pos_err, guidance_h.sp.pos, delfly_state.h.pos);
	/* saturate it               */
	//VECT2_STRIM(guidance_h_pos_err, -MAX_POS_ERR, MAX_POS_ERR);

	/* compute speed error    */
	VECT2_DIFF(guidance_h_module_vel_err, guidance_h.sp.speed, delfly_state.h.vel);
	/* saturate it               */
	//VECT2_STRIM(guidance_h_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);


	*cmd_accelerate = 0;
	*cmd_heading = delfly_state.h.heading;

	//guidance_lat_adjust_heading( in_flight, cmd_heading, guidance_h_module_pos_err );
}
