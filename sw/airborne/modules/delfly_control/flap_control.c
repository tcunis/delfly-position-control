/*
 * Copyright (C) 2016 Torbjoern Cunis <t.cunis@tudelft.nl>
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
 * @file /paparazzi/paparazzi/sw/airborne/modules/delfly_control/flap_control.c
 * @author cunis
 */


#include "flap_control.h"

#include "paparazzi.h"
#include "generated/airframe.h"

#include "subsystems/sensors/rpm_sensor.h"
#include "firmwares/rotorcraft/stabilization.h"


#ifndef FLAP_CONTROL_PGAIN
#define FLAP_CONTROL_PGAIN    0
#endif
#ifndef FLAP_CONTROL_IGAIN
#define FLAP_CONTROL_IGAIN    0
#endif
#ifndef FLAP_CONTROL_ADAPT
#define FLAP_CONTROL_ADAPT    0
#endif


struct FlapControl flap_control;

bool_t flap_control_antiwindup;


void flap_control_init(void) {

  flap_control.gains.p = FLAP_CONTROL_PGAIN;
  flap_control.gains.i = FLAP_CONTROL_IGAIN;
  flap_control.gains.gamma = FLAP_CONTROL_ADAPT;

  flap_control.freq_sp = 0.f;

  flap_control.throttle_cmd = 0;

  flap_control_antiwindup = FALSE;
}


void flap_control_enter(void) {}


bool_t flap_control_run(void) {

  flap_control.freq_now = rpm_sensor.motor_frequency;
  flap_control.freq_err = flap_control.freq_sp - flap_control.freq_now;

  //todo:
  int32_t throttle_cmd = flap_control.gains.p*MAX_PPRZ/100;

  flap_control.throttle_cmd = TRIM_UPPRZ(throttle_cmd);

  stabilization_cmd[COMMAND_THRUST] = flap_control.throttle_cmd;

  return !(flap_control_antiwindup = (flap_control.throttle_cmd != throttle_cmd));
}

extern void flap_control_set(float freq_cmd) { flap_control.freq_sp = freq_cmd; }

bool_t flap_control_reachable(void) {

  return !flap_control_antiwindup;
}
