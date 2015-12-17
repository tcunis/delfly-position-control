/*
 * Copyright (C) 2015 Torbjoern Cunis <t.cunis@tudelft.nl>
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
 * @file /paparazzi/paparazzi/sw/airborne/subsystems/ins/ins_module_int.h
 * @author Torbjoern Cunis
 *
 * State filter in a module file.
 *
 * Implement a customn state filter in a module.
 *
 * One must implement:
 *
 * - void ins_module_int_init (void)
 * - void ins_module_int_propagate (struct Int32Vect3* accel_meas_ltp, float dt)
 *
 * #if USE_GPS
 * - void ins_module_int_update_gps (struct NedCoor_i* gps_pos_ned, struct NedCoor_i* gps_speed_ned, float dt)
 * #endif
 *
 * TODO: add update_sonar, update_baro
 */

#ifndef INS_MODULE_INT_H_
#define INS_MODULE_INT_H_


#include "generated/modules.h"


#endif /* INS_MODULE_INT_H_ */
