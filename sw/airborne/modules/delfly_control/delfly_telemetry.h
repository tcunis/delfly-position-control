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
 * @file /paparazzi/paparazzi/sw/airborne/modules/delfly_control/delfly_telemetry.h
 * @author Torbjoern Cunis
 */

#ifndef DELFLY_TELEMETRY_H_
#define DELFLY_TELEMETRY_H_


#include "subsystems/datalink/telemetry.h"


extern static void delfly_telemetry_send_guidance (struct transport_tx*, struct link_device*);

extern static void delfly_telemetry_send_state (struct transport_tx*, struct link_device*);
extern static void delfly_telemetry_send_stateraw (struct transport_tx*, struct link_device*);
extern static void delfly_telemetry_send_stateestimation (struct transport_tx*, struct link_device*);

extern static void delfly_telemetry_send_speedcontrol (struct transport_tx*, struct link_device*);

extern void delfly_telemetry_init (void);


#endif /* DELFLY_TELEMETRY_H_ */
