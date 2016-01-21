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

#include "delfly_control.h"

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
#ifndef FLAP_CONTROL_RATIO
#define FLAP_CONTROL_RATIO    0
#endif


#define FLAP_CONTROL_MODE_OFF         0
#define FLAP_CONTROL_MODE_THROTTLE    1
#define FLAP_CONTROL_MODE_FLAP        2
#define FLAP_CONTROL_MODE_FLAPCTRL    3



struct FlapControl flap_control;

bool_t flap_control_antiwindup;

uint8_t flap_control_mode;


void flap_control_init(void) {

  flap_control.gains.p = FLAP_CONTROL_PGAIN;
  flap_control.gains.i = FLAP_CONTROL_IGAIN;
  flap_control.gains.gamma = FLAP_CONTROL_ADAPT;

  flap_control.freq_sp = 0.f;

  flap_control.throttle_cmd = 0;

  flap_control.ratio = FLAP_CONTROL_RATIO;

  flap_control_antiwindup = FALSE;

  flap_control_mode = FLAP_CONTROL_MODE_THROTTLE;
}


void flap_control_enter(void) {
}


bool_t flap_control_run(void) {

  static int32_t throttle_cmd = 0;
  static bool_t antiwindup_min = FALSE,
                antiwindup_max = FALSE;
  //todo: implement flap model, if necessary
  static float freq_ref = -1;

  switch (flap_control_mode) {
  case FLAP_CONTROL_MODE_OFF:
    return FALSE;
  case FLAP_CONTROL_MODE_THROTTLE:
    {
      if ( flap_control.gains.p > 0 && !flap_control_antiwindup ) {
        static uint32_t time = 0;
        time++;
        if ( time > (flap_control.gains.p*DELFLY_CONTROL_RUN_FREQ) ) {
          throttle_cmd += flap_control.gains.i*MAX_PPRZ/100;
          time = 0;
        }
      } else {
        throttle_cmd = 70*MAX_PPRZ/100;
      }
    }
    break;

  case FLAP_CONTROL_MODE_FLAP:
    {
      if ( flap_control.gains.p > 0 && !flap_control_antiwindup ) {
        static uint32_t time = 0;
        time++;
        if ( time > (flap_control.gains.p*DELFLY_CONTROL_RUN_FREQ) ) {
          flap_control.freq_sp += flap_control.gains.i/100;
          time = 0;
        }
      } else {
        flap_control.freq_sp = 5.0;
      }
    }
    /* no break */
  case FLAP_CONTROL_MODE_FLAPCTRL:
    {
      //debug: set sp
      if ( flap_control_mode == FLAP_CONTROL_MODE_FLAPCTRL ) {
        flap_control.freq_sp = flap_control.gains.p / 10.0;
      }

      flap_control.freq_now = rpm_sensor.average_frequency;

    //  if ( !flap_control_antiwindup || flap_control.freq_sp < flap_control.freq_now ) {
    //    freq_ref = flap_control.freq_sp;
    //  }
      if ( (freq_ref == -1) || (antiwindup_max && flap_control.freq_sp > flap_control.freq_now)
                            || (antiwindup_min && flap_control.freq_sp < flap_control.freq_now) ) {
        freq_ref = flap_control.freq_now;
      }
      flap_control.freq_err = freq_ref - flap_control.freq_now;

      //store frequency sp for next iteration
      freq_ref = flap_control.freq_sp;

      //change of adaptable parameter, d(ratio)/dt = - gamma/100 * (now - ref) in (%/Hz)/s
      float ratio_dt = flap_control.gains.gamma * (flap_control.freq_err) / 100;

      //integrate parameter throttle to flapping-frequency (in %) ratio in %/Hz
      flap_control.ratio += ratio_dt * FLAP_CONTROL_RUN_PERIOD;

      //calculate current throttle command: frequency set-point * throttle-to-frequency-ratio * MAX_PPRZ/100
      throttle_cmd = flap_control.ratio * flap_control.freq_sp * MAX_PPRZ / 100;
    }
    break;
  }

  flap_control.throttle_cmd = TRIM_UPPRZ(throttle_cmd);

  stabilization_cmd[COMMAND_THRUST] = flap_control.throttle_cmd;

  antiwindup_min = (flap_control.throttle_cmd > throttle_cmd); //actual throttle cmd below UPPRZ_MIN: left anti-windup active
  antiwindup_max = (flap_control.throttle_cmd < throttle_cmd); //actual throttle cmd above UPPRZ_MAX: right anti-windup active
  //return !(flap_control_antiwindup = (flap_control.throttle_cmd != throttle_cmd));
  return !(flap_control_antiwindup = antiwindup_min || antiwindup_max);
}

extern void flap_control_set(float freq_cmd) { flap_control.freq_sp = freq_cmd; }

bool_t flap_control_reachable(void) {

  return !flap_control_antiwindup;
}
