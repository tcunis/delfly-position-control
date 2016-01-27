/*
 * Copyright (C) 2015 Torbjoern Cunis <t.cunis@tudelft.nl>
 *
 * MATLAB include utility for import of MATLAB-generated values.
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
 * @file /modules/delfly_control/matlab_include.h
 * @author Torbjoern Cunis
 */

#ifndef MATLAB_INCLUDE_H
#define MATLAB_INCLUDE_H


#include "delfly_algebra_int.h"
#include "math/pprz_algebra_float.h"


extern const uint8_t INT32_MATLAB_FRAC;


/* static co-variances R & Q
 * with #INT32_MATLAB_FRAC
 */
extern struct Int32Vect3 matlab_noise_distribution;
extern struct Int32Vect3 matlab_disturbance_distribution;


/* optimal guidance forward and lateral
 * in m/s2, with #INT32_MATLAB_FRAC
 */
extern struct Int32Vect2 matlab_guidance_gain;
extern struct Int32Vect2 matlab_guidance_gain_fwd;
extern struct Int32Vect2 matlab_guidance_gain_lat;


/* speed at equilibria, in m/s with #INT32_MATLAB_FRAC */
extern int32_t matlab_airspeed_v04;
extern int32_t matlab_airspeed_v08;
extern int32_t matlab_airspeed_v12;
extern int32_t matlab_airspeed_v25;
extern int32_t matlab_airspeed_v50;

/* pitch angle at equilibria, in rad with #INT32_MATLAB_FRAC */
extern int32_t matlab_pitch_equilibrium_v04;
extern int32_t matlab_pitch_equilibrium_v08;
extern int32_t matlab_pitch_equilibrium_v12;
extern int32_t matlab_pitch_equilibrium_v25;
extern int32_t matlab_pitch_equilibrium_v50;

/* throttle at equilibria, in % with #INT32_MATLAB_FRAC/2 */
extern int32_t matlab_throttle_equilibrium_v04;
extern int32_t matlab_throttle_equilibrium_v08;
extern int32_t matlab_throttle_equilibrium_v12;
extern int32_t matlab_throttle_equilibrium_v25;
extern int32_t matlab_throttle_equilibrium_v50;

/* flapping frequency at equilibria, in Hz */
extern float matlab_flapfreq_equilibrium_v04;
extern float matlab_flapfreq_equilibrium_v08;
extern float matlab_flapfreq_equilibrium_v12;
extern float matlab_flapfreq_equilibrium_v25;
extern float matlab_flapfreq_equilibrium_v50;


/* pitch matrix in rad, with #INT32_MATLAB_FRAC */
extern struct Int32Vect2 matlab_pitch_matrix_v04;
extern struct Int32Vect2 matlab_pitch_matrix_v08;
extern struct Int32Vect2 matlab_pitch_matrix_v12;
extern struct Int32Vect2 matlab_pitch_matrix_v25;
extern struct Int32Vect2 matlab_pitch_matrix_v50;

/* throttle in %, with #INT32_MATLAB_FRAC/2 */
extern struct Int32Vect2 matlab_throttle_matrix_v04;
extern struct Int32Vect2 matlab_throttle_matrix_v08;
extern struct Int32Vect2 matlab_throttle_matrix_v12;
extern struct Int32Vect2 matlab_throttle_matrix_v25;
extern struct Int32Vect2 matlab_throttle_matrix_v50;

/* flapping frequency in Hz */
extern struct FloatVect2 matlab_flapfreq_matrix_v04;
extern struct FloatVect2 matlab_flapfreq_matrix_v08;
extern struct FloatVect2 matlab_flapfreq_matrix_v12;
extern struct FloatVect2 matlab_flapfreq_matrix_v25;
extern struct FloatVect2 matlab_flapfreq_matrix_v50;

#endif // MATLAB_INCLUDE_H
