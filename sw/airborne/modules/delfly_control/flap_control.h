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
 * @file /paparazzi/paparazzi/sw/airborne/modules/delfly_control/flap_control.h
 * @author Torbjoern Cunis
 */

#ifndef FLAP_CONTROL_H_
#define FLAP_CONTROL_H_


#include "std.h"


struct FlapControlGains {
  int32_t p;
  int32_t i;
  int32_t gamma;
};

struct FlapControl {
  struct FlapControlGains gains;

  float freq_sp;
  float freq_now;
  float freq_err;
  int32_t throttle_cmd;
};

extern struct FlapControl flap_control;

extern void flap_control_init(void);
extern void flap_control_enter(void);
extern bool_t flap_control_run(void);

extern void flap_control_set(float freq_cmd);
extern bool_t flap_control_reachable(void);


#endif /* FLAP_CONTROL_H_ */
