/*
 * Copyright (C) Bart Slinger
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
 * @file "modules/rpm_sensor/rpm_sensor.h"
 * @author Bart Slinger
 * Measure the ppm signal of the RPM sensor
 */

#ifndef RPM_SENSOR_H
#define RPM_SENSOR_H

#include "std.h"

/**
 * Architecture dependant code
 */
#include "subsystems/sensors/rpm_sensor_arch.h"
/* must be implemented by arch dependant code */


struct RpmSensor {
  uint16_t previous_cnt;
  float previous_frequency;
  float motor_frequency;
};

extern struct RpmSensor rpm_sensor;

extern void rpm_sensor_init(void);
extern void rpm_sensor_process_pulse(uint16_t cnt, uint8_t overflow_cnt);

#endif

