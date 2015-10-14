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
 * @file "modules/gps_diagnostics/gps_diagnostics.h"
 * @author Torbjoern Cunis <t.cunis@tudelft.nl>
 * 
 */

#ifndef GPS_DIAGNOSTICS_H
#define GPS_DIAGNOSTICS_H


extern void gps_diagnostics_init(void);
extern void gps_diagnostics_periodic(void);
extern void gps_diagnostics_datalink_event(void);
extern void gps_diagnostics_datalink_small_event(void);

#endif

