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
 * @file "modules/gps_diagnostics/gps_diagnostics.c"
 * @author Torbjoern Cunis <t.cunis@tudelft.nl>
 * 
 */

#include "modules/gps_diagnostics/gps_diagnostics.h"

#include "gps.h"
#include "subsystems/datalink/telemetry.h"


#define SEC_OF_USEC(_USEC_)         ( (_USEC_)*1.0/USEC_OF_SEC(1) )
#define USEC_OF_GPS_MSG(_GPS_)      ( USEC_OF_SEC(_GPS_.last_msg_time) + usec_of_cpu_ticks(_GPS_.last_msg_ticks) )


uint16_t msg_count;         // total number of gps messages received since start-up
uint16_t dl_pkg_count;      // total number of datalink "gps" packages received

double   gps_freq;          // estimated frequence of gps message reception, in Hz
double   gps_period;        // estimated perdiod of gps message reception, in s

uint32_t last_msgtimediff;  // time-difference of last and last-but-one gps message, in usec
uint32_t totl_msgtimediff;  // in usec
double   avrg_msgtimediff;  // average time-difference between gps messages, in usec

uint8_t  count;

struct GpsState last_gps_state;
uint32_t last_msg_time;     // time of reception of last gps message, in usec





void gps_diagnostics_send_diagnostics( struct transport_tx* trans, struct link_device* device ) {

    DOWNLINK_SEND_GPS_DIAGNOSTICS( DefaultChannel, DefaultDevice,
        &msg_count,
        &dl_pgk_count,
        &gps_freq,
        &gps_period,
        &last_msgtimediff,
        &avrg_msgtimediff,
        &count
    );
}

void gps_diagnostics_init() {

    msg_count = 0;
    dl_pkg_count = 0;;
    gps_freq = 0.0;
    gps_period = 0;
    last_msgtimediff = 0;
    totl_msgtimediff = 0;
    avrg_msgtimediff = 0.0;
    count = 0;

    last_gps_state = gps;
    last_msg_time = USEC_OF_GPS_MSG(gps);

    register_periodic_telemetry(DefaultPeriodic, "GPS_DIAGNOSTICS", &gps_diagnostics_send_diagnostics);
}

void gps_diagnostics_periodic() {

    uint32_t msg_time = USEC_OF_GPS_MSG(gps);

    if ( msg_time <= last_msg_time ) {
        count++;
        return; //nothing to do    
    }
    
    //else:
    msg_count++;
    
    uint32_t msg_timediff = ( msg_time - last_msg_time );
    totl_msgtimediff += msg_timediff;
    avrg_msgtimediff = ( totl_msgtimediff * 1.0 / msg_count );
    last_msgtimediff = msg_timediff;
    
    gps_period = SEC_OF_USEC(avrg_msgtimediff);
    gps_freq   = ( 1 / gps_period );
    
    count = 0;
}

void gps_diagnostics_datalink_event() { dl_pkg_count++; }
void gps_diagnostics_datalink_small_event() { dl_pkg_count++; }

