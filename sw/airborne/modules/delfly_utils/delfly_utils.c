/*
 * Copyright (C) Torbjoern Cunis <t.cunis@tudelft.nl>
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
 * @file "modules/delfly_utils/delfly_utils.c"
 * @author Torbjoern Cunis <t.cunis@tudelft.nl>
 * 
 */

#include "delfly_utils.h"

#include "generated/airframe.h"
#include "subsystems/radio_control.h"


uint8_t LEDS_switch = DELFLY_UTILS_LEDS_SWITCH;
uint8_t SRVO_kill = DELFLY_UTILS_SRVO_KILL;

extern void util_init(void) {
}

extern void util_run_periodic(void) {
  // refer values greater than -1000 to be non-negative
  LEDS_switch = ( radio_control.values[RADIO_MIX] > -1000 );
}
