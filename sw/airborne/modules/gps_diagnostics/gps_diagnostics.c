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

#include "subsystems/gps.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/datalink/pprzlog_transport.h"
#include "generated/modules.h"

//#include "generated/airframe.h"
#include "subsystems/ins/ins_int.h"
#include "subsystems/gps.h"


#define SEC_OF_USEC(_USEC_)         ( (_USEC_)*1.0/USEC_OF_SEC(1) )
#define USEC_OF_GPS_MSG(_GPS_)      ( USEC_OF_SEC(_GPS_.last_msg_time) + usec_of_cpu_ticks(_GPS_.last_msg_ticks) )

uint8_t msg_fmt_small;

uint16_t msg_count;         // total number of gps messages received since start-up
uint16_t dlpkg_count;      // total number of datalink "gps" packages received

double   gps_freq;          // estimated frequence of gps message reception, in Hz
double   gps_period;        // estimated perdiod of gps message reception, in s

double   dl_freq;
double   dl_period;

uint32_t prev_msgtimediff;  // previous time-difference of last and last-but-one gps message, in μs
uint32_t totl_msgtimediff;  // in μs
double   avrg_msgtimediff;  // average time-difference between gps messages, in μs

uint32_t dl_pkgtimediff;    // in μs

uint32_t prev_dlpkg_time;   // time of reception of previous datalink gps package in μs

uint8_t  count;

struct GpsState prev_gps_state; // last gps state received
uint32_t prev_msg_time;     // previous time of reception of last gps message, in μs

uint8_t  count_iteration;   // number of iterations since previous gps message

struct NedCoor_i gps_pos_cm_ned;


static void gps_diagnostics_send_diagnostics( struct transport_tx* trans, struct link_device* dev ) {

  pprz_msg_send_GPS_DIAGNOSTICS( trans, dev, AC_ID,
        &msg_fmt_small,
//        &msg_count,
        &dlpkg_count,
//        &gps_freq,
//        &gps_period,
        &dl_freq,
        &dl_period,
//        &prev_msgtimediff,
//        &avrg_msgtimediff,
        &dl_pkgtimediff,
//        &count,
		&gps_pos_cm_ned.x,
		&gps_pos_cm_ned.y,
		&gps_pos_cm_ned.z
    );
}

void gps_diagnostics_init(void) {

    msg_fmt_small = 0;
    msg_count = 0;
    dlpkg_count = 0;;
    gps_freq = 0.0;
    gps_period = 0.0;
    dl_freq = 0.0;
    dl_period = 0.0;
    prev_msgtimediff = 0;
    totl_msgtimediff = 0;
    avrg_msgtimediff = 0.0;
    dl_pkgtimediff = 0;
    count = -1;

    INT_VECT3_ZERO(gps_pos_cm_ned);

    prev_gps_state = gps;
    prev_msg_time = USEC_OF_GPS_MSG(gps);
    
    prev_dlpkg_time = 0;
    
    count_iteration = 0;

    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GPS_DIAGNOSTICS, &gps_diagnostics_send_diagnostics);
}

void gps_diagnostics_log_pos( struct NedCoor_i* pos_ned ) {

//	struct NedCoor_i gps_pos_cm_ned;
  ned_of_ecef_point_i(&gps_pos_cm_ned, &ins_int.ltp_def, &gps.ecef_pos);

//	gps_pos_ned.x = pos_ned->x;
//	gps_pos_ned.x = pos_ned->y;
//	gps_pos_ned.x = pos_ned->z;

	gps_diagnostics_send_diagnostics( &pprzlog_tp.trans_tx, &sdlogger_spi.device );
}

void gps_diagnostics_periodic(void) {

    // calculate time of last gps message received -- in us
    uint32_t msg_time = USEC_OF_GPS_MSG(gps);

    // if time of last received message is equals to (or lower than) previous,
    // nothing to do for no new message received.
    if ( msg_time <= prev_msg_time ) {
        count_iteration++;
        return; //nothing to do    
    }
    
    //else:
    // another message received:
    msg_count++;
    
    // calculate time-difference between last and last-but-one gps message;
    uint32_t msg_timediff = ( msg_time - prev_msg_time );

    // add recent time-difference to total,
    totl_msgtimediff += msg_timediff;
    // calculate current average time-difference,
    avrg_msgtimediff = ( totl_msgtimediff * 1.0 / msg_count );
    // store current time-difference for next iteration;
    prev_msgtimediff = msg_timediff;
    
    // store current gps state,get_sys_time_usec();
    prev_gps_state = gps;
    // store time of current last gps message received;
    prev_msg_time = msg_time;
    
    // calculate period of gps message reception as current time difference,
    gps_period = SEC_OF_USEC(msg_timediff);
    // calculate frequency of gps message reception;
    gps_freq   = ( 1 / gps_period );
    
    // set number of iterations between last and last-but-one 6836gps messages,
    count = count_iteration;
    // reset iteration counter.
    count_iteration = 0;
}

void gps_diagnostics_datalink_event(void) {

    msg_fmt_small = 0;

    dlpkg_count++;
    uint32_t dlpkg_time = get_sys_time_usec();
    
    if ( dlpkg_time <= prev_dlpkg_time )
        return;
        
    //else:
    if ( prev_dlpkg_time > 0 ) {
        dl_pkgtimediff = ( dlpkg_time - prev_dlpkg_time );
        
        dl_period = SEC_OF_USEC(dl_pkgtimediff);
        dl_freq   = ( 1 / dl_period );
    }
    prev_dlpkg_time = dlpkg_time;
}

void gps_diagnostics_datalink_small_event(void) {

    msg_fmt_small = 1;

    dlpkg_count++;
    uint32_t dlpkg_time = get_sys_time_usec();
    
    if ( dlpkg_time <= prev_dlpkg_time )
        return;
        
    //else:
    if ( prev_dlpkg_time > 0 ) {
        dl_pkgtimediff = ( dlpkg_time - prev_dlpkg_time );
        
        dl_period = SEC_OF_USEC(dl_pkgtimediff);
        dl_freq   = ( 1 / dl_period );
    }
    prev_dlpkg_time = dlpkg_time;
}

