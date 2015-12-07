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

#include "delfly_state.h"
#include "speed_control.h"

#include "guidance/guidance_v.h"


/* vertical acceleration command
 * in m/s2, with #INT32_ACCEL_FRAC
 * NOTE, that z-axis points down! */
int32_t guidance_cmd_v_accelerate;


/* vertical position and velocity error
 * in m, with #INT32_POS_FRAC;
 * in m/s, with #INT32_VEL_FRAC;
 */
int32_t guidance_v_module_pos_err;
int32_t guidance_v_module_vel_err;


static void guidance_v_module_run_traj( bool_t, int32_t* cmd_accelerate );


void guidance_v_module_init(void) {

  guidance_cmd_v_accelerate = 0;
  guidance_v_module_pos_err = 0;
  guidance_v_module_vel_err = 0;
}


void guidance_v_module_enter(void) {

	guidance_cmd_v_accelerate = 0;
}


void guidance_v_module_run(bool_t in_flight) {

	guidance_v_module_run_traj(in_flight, &guidance_cmd_v_accelerate);

	speed_control_set_cmd_v(guidance_cmd_v_accelerate);
}


void guidance_v_module_run_traj( bool_t in_flight, int32_t* cmd_accelerate ) {

	/* compute position error    */
	guidance_v_module_pos_err = guidance_v_z_sp - delfly_state.v.pos;
	/* saturate it               */
	//VECT2_STRIM(guidance_h_pos_err, -MAX_POS_ERR, MAX_POS_ERR);

	/* compute speed error    */
	guidance_v_module_vel_err = guidance_v_z_sp - delfly_state.v.vel;
	/* saturate it               */
	//VECT2_STRIM(guidance_h_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);


	*cmd_accelerate = 0;
}
