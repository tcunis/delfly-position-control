/*
 * delfly_algebra_int.h
 *
 *  Created on: Dec 7, 2015
 *      Author: cunis
 */

#ifndef DELFLY_ALGEBRA_INT_H_
#define DELFLY_ALGEBRA_INT_H_


#include "math/pprz_algebra_int.h"


#define INT32_ZERO(_i)    { _i = 0; }


union Int32VectLong {
  struct Int32Vect2 xy;
  struct {
    int32_t fwd;
    int32_t ver;
  } fv;
};


union Int32VectState2 {
  struct Int32Vect2 xy;
  struct {
    int32_t pos;
    int32_t vel;
  } states;
};


#endif /* DELFLY_ALGEBRA_INT_H_ */
