/*
 * Copyright (C) cunis
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
 * @file "modules/delfly_control/delfly_control.c"
 * @author cunis
 * 
 */

#include "modules/delfly_control/delfly_control.h"

#include "guidance/guidance_h.h";
#include "guidance/guidance_v.h";

#include "state.h"


uint8_t h_mode_alt = 0;
uint8_t v_mode_alt = 0;


/*    general module functions      */
void delfly_control_init(void){

  //nothing to do yet
}

void delfly_control_start(void){

  speed_control_start();

  h_mode_alt = guidance_h.mode;
  v_mode_alt = guidance_v_mode;

  guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
  guidance_v_mode_changed(GUIDANCE_V_MODE_MODULE);
}


void delfly_control_stop(void){
  speed_control_stop();

  guidance_h_mode_changed(h_mode_alt);
  guidance_v_mode_changed(v_mode_alt);
}

/*    delfly control periodic       */
void delfly_control_run(void) {

  //TODO: determin in_flight status.
  speed_control_run(TRUE);
}

