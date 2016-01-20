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
 * @file "modules/rpm_sensor/rpm_sensor.c"
 * @author Bart Slinger
 * Measure the ppm signal of the RPM sensor
 */

// Telemetry to test values
//#include "subsystems/datalink/telemetry.h"

#include "subsystems/sensors/rpm_sensor.h"
#include "subsystems/datalink/telemetry.h"

#include "firmwares/rotorcraft/stabilization.h"

#include "filters/averaging_filter.h"


#ifndef PULSES_PER_REVOLUTION
#define PULSES_PER_REVOLUTION   PULSES_PER_ROTATION
#endif

struct RpmSensor rpm_sensor;

const float pulse_per_rot = PULSES_PER_ROTATION;
const float pulse_per_rev = PULSES_PER_REVOLUTION;
const float rev_per_rot   = (PULSES_PER_ROTATION) / (PULSES_PER_REVOLUTION);


static void rpm_sensor_send_tm(struct transport_tx*, struct link_device*);


void rpm_sensor_init(void)
{
  rpm_sensor_arch_init();

  rpm_sensor.pulse_count = 0;
  rpm_sensor.sample_count = 0;

  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RPM_SENSOR, &rpm_sensor_send_tm);
}

void rpm_sensor_process_pulse(uint16_t cnt, uint8_t overflow_cnt)
{
  (void) overflow_cnt;
  static uint32_t last_rot_count = 0;
  static int32_t last_throttle = 0;
  static uint32_t total_cnt_diff = 0;

  if ( stabilization_cmd[COMMAND_THRUST] <= 0 || overflow_cnt > 1 ) {
    rpm_sensor.previous_frequency = rpm_sensor.rotation_frequency;
    rpm_sensor.motor_frequency    = 0.f;
    rpm_sensor.rotation_frequency = 0.f;
    rpm_sensor.average_frequency  = 0.f;
    rpm_sensor.pulse_count = 0;
    last_rot_count = 0;
    last_throttle = 0;

  } else {

    if ( stabilization_cmd[COMMAND_THRUST] != last_throttle ) {
      //rpm_sensor.sample_count = 0;
      last_throttle = stabilization_cmd[COMMAND_THRUST];
    }

    rpm_sensor.pulse_count++;
    rpm_sensor.rot_count = (rpm_sensor.pulse_count / pulse_per_rot);

    int32_t cnt_diff = cnt - rpm_sensor.previous_cnt;

    if ( overflow_cnt > 0 ) {
      cnt_diff += 0xFFFF; // add uint16_max
    }

    rpm_sensor.previous_frequency = rpm_sensor.motor_frequency;
    rpm_sensor.motor_frequency = 281250.0/cnt_diff/pulse_per_rev;

    static float sum_motor_frequency = 0.f;
    /* Check motor frequency maximum */
//    if ( rpm_sensor.motor_frequency < rpm_sensor.previous_frequency ) {
//      rpm_sensor.average_frequency = sum_frequency/rpm_sensor.sample_count;
//      sum_frequency = rpm_sensor.motor_frequency;
//      rpm_sensor.sample_count = 1;
//    } else {
      sum_motor_frequency += rpm_sensor.motor_frequency;
      rpm_sensor.sample_count++;
//    }

    total_cnt_diff += cnt_diff;

    if ( rpm_sensor.rot_count > last_rot_count ) {
      uint32_t rot_diff = rpm_sensor.rot_count - last_rot_count;

      //rpm_sensor.previous_frequency = rpm_sensor.rotation_frequency;
      rpm_sensor.rotation_frequency = 281250.0/total_cnt_diff/rot_diff;

      //rpm_sensor.average_frequency = (rpm_sensor.sample_count*rpm_sensor.average_frequency + rpm_sensor.rotation_frequency)/(++rpm_sensor.sample_count);
            rpm_sensor.average_frequency = (sum_motor_frequency/rpm_sensor.sample_count)/rev_per_rot;
            sum_motor_frequency = 0;
            rpm_sensor.sample_count = 0;

      /* Remember count */
      last_rot_count = rpm_sensor.rot_count;

      /* Zero integrals */
      total_cnt_diff = 0;
      //rpm_sensor.pulse_count -= (rpm_sensor.rot_count*pulse_per_rot);
    }
  }

  rpm_sensor.previous_cnt = cnt;
}

void rpm_sensor_send_tm(struct transport_tx* trans, struct link_device* dev) {
  pprz_msg_send_RPM_SENSOR(trans, dev, AC_ID,
      &rpm_sensor.rot_count,
      &rpm_sensor.previous_cnt,
      &pulse_per_rot,
      &rpm_sensor.previous_frequency,
      &rpm_sensor.rotation_frequency,
      &rpm_sensor.average_frequency,
      &rpm_sensor.motor_frequency,
      &rpm_sensor.sample_count
  );
}
