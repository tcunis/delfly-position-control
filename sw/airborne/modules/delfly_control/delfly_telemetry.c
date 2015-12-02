/*
 * Copyright (C) 2015 Torbjoern Cunis <t.cunis@tudelft.nl>
 *
 * This defines functions in order to send Delfy control module telemtry
 * to ground control.
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
 * @file /paparazzi/paparazzi/sw/airborne/modules/delfly_control/delfly_telemtry.c
 * @author Torbjoern Cunis
 */

#include "delfly_telemetry.h"

#include "delfly_state.h"
#include "speed_control_var.h"


static void delfly_telemetry_send_guidance (struct transport_tx*, struct link_device*);

static void delfly_telemetry_send_state (struct transport_tx*, struct link_device*);
static void delfly_telemetry_send_stateraw (struct transport_tx*, struct link_device*);
static void delfly_telemetry_send_stateestimation (struct transport_tx*, struct link_device*);

static void delfly_telemetry_send_speedcontrol (struct transport_tx*, struct link_device*);


static void delfly_telemetry_send_guidance (struct transport_tx* trans, struct link_device* dev) {
//	DOWNLINK_SEND_DELFLY_GUIDANCE( DefaultChannel, DefaultDevice,
//		&guidance_h.mode,
//		&guidance_v_mode
//	);
}

static void delfly_telemetry_send_state (struct transport_tx* trans, struct link_device* dev) {
	DOWNLINK_SEND_DELFLY_STATE(DefaultChannel, DefaultDevice,
	    &delfly_state.h.pos.x,
	    &delfly_state.h.pos.y,
	    &delfly_state.v.pos,
	    &delfly_state.h.vel.x,
	    &delfly_state.h.vel.y,
	    &delfly_state.v.vel,
	    &delfly_state.h.air.x,
	    &delfly_state.h.air.y,
	    &delfly_state.v.air,
	    &delfly_state.h.acc.x,
	    &delfly_state.h.acc.y,
	    &delfly_state.v.acc,
	    &delfly_state.fv.air.fv.fwd,
	    &delfly_state.fv.vel.fv.fwd,
	    &delfly_state.h.heading,
	    &delfly_state.h.head_rate,
	    &delfly_state.h.azimuth
	);
}

static void delfly_telemetry_send_stateraw (struct transport_tx* trans, struct link_device* dev) {
//	DOWNLINK_SEND_DELFLY_STATERAW(DefaultChannel, DefaultDevice
//	);
}

static void delfly_telemetry_send_stateestimation (struct transport_tx* trans, struct link_device* dev) {
//	DOWNLINK_SEND_DELFLY_STATEESTIMATION(struct transport_tx*, struct link_device*
//	);
}

static void delfly_telemetry_send_speedcontrol (struct transport_tx* trans, struct link_device* dev) {
	DOWNLINK_SEND_DELFLY_SPEEDCONTROL(DefaultChannel, DefaultDevice,
		&speed_control.mode,
		&speed_control.sp.acceleration.fv.fwd,
		&speed_control.sp.acceleration.fv.ver,
		&speed_control_var.now.air_speed,
		&speed_control_var.eq.pitch,
		&speed_control_var.eq.throttle,
		&speed_control_var.mat.pitch.y,
		&speed_control_var.mat.pitch.x,
		&speed_control_var.mat.throttle.y,
		&speed_control_var.mat.throttle.x,
		&speed_control_var.err.acceleration.fv.fwd,
		&speed_control_var.err.acceleration.fv.ver,
		&speed_control_var.err.velocity.fv.fwd,
		&speed_control_var.err.velocity.fv.ver,
		&speed_control_var.ff_cmd.pitch,
		&speed_control_var.ff_cmd.throttle,
		&speed_control_var.fb_cmd.pitch,
		&speed_control_var.fb_cmd.throttle,
		&speed_control_var.cmd.pitch,
		&speed_control_var.cmd.throttle
	);
}


void delfly_telemetry_init_all (void) {

	register_periodic_telemetry(DefaultPeriodic, "DELFLY_GUIDANCE", &delfly_telemetry_send_guidance);
	register_periodic_telemetry(DefaultPeriodic, "DELFLY_STATE", &delfly_telemetry_send_state);
	register_periodic_telemetry(DefaultPeriodic, "DELFLY_STATERAW", &delfly_telemetry_send_stateraw);
	register_periodic_telemetry(DefaultPeriodic, "DELFLY_STATEESTIMATION", &delfly_telemetry_send_stateestimation);
	register_periodic_telemetry(DefaultPeriodic, "DELFLY_SPEEDCONTROL", &delfly_telemetry_send_speedcontrol);
}
