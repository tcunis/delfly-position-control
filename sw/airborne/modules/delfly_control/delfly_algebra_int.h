/*
 * delfly_algebra_int.h
 *
 *  Created on: Dec 7, 2015
 *      Author: cunis
 */

#ifndef DELFLY_ALGEBRA_INT_H_2
#define DELFLY_ALGEBRA_INT_H_2


#include "math/pprz_algebra_int.h"


#define INT32_TIME_FRAC	  10

#define INT32_ZERO(_i)    { _i = 0; }

#define CUBE(_a)          ((_a)*(_a)*(_a))
#define HYCUBE(_a)        ((_a)*(_a)*(_a)*(_a))

#define VECT3_ZERO(_v)    INT32_VECT3_ZERO(_v)
#define MAT33_ZERO(_m)    INT32_MAT33_ZERO(_m)
#define EULERS_ZERO(_e)   EULERS_ASSIGN(_e, 0, 0, 0)
#define RATES_ZERO(_r)    RATES_ASSIGN(_r, 0, 0, 0)


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

/* 3x3 matrix                                    */
struct Int64Mat33 {
  int64_t m[3 * 3];
};

#define VECT3_ADD_SCALED2(_a, _b, _num, _den) {   \
    (_a).x += ((_b).x * (_num)) / (_den);        \
    (_a).y += ((_b).y * (_num)) / (_den);        \
    (_a).z += ((_b).z * (_num)) / (_den);        \
  }

#define MAT33_ADD(_mat1,_mat2) {     \
    MAT33_ELMT((_mat1),0,0) += MAT33_ELMT((_mat2),0,0);  \
    MAT33_ELMT((_mat1),0,1) += MAT33_ELMT((_mat2),0,1);  \
    MAT33_ELMT((_mat1),0,2) += MAT33_ELMT((_mat2),0,2);  \
    MAT33_ELMT((_mat1),1,0) += MAT33_ELMT((_mat2),1,0);  \
    MAT33_ELMT((_mat1),1,1) += MAT33_ELMT((_mat2),1,1);  \
    MAT33_ELMT((_mat1),1,2) += MAT33_ELMT((_mat2),1,2);  \
    MAT33_ELMT((_mat1),2,0) += MAT33_ELMT((_mat2),2,0);  \
    MAT33_ELMT((_mat1),2,1) += MAT33_ELMT((_mat2),2,1);  \
    MAT33_ELMT((_mat1),2,2) += MAT33_ELMT((_mat2),2,2);  \
  }

#define MAT33_ADD_SCALED2(_mat1,_mat2, _num, _den) {     \
    MAT33_ELMT((_mat1),0,0) += (MAT33_ELMT((_mat2),0,0) * (_num)) / (_den);  \
    MAT33_ELMT((_mat1),0,1) += (MAT33_ELMT((_mat2),0,1) * (_num)) / (_den);  \
    MAT33_ELMT((_mat1),0,2) += (MAT33_ELMT((_mat2),0,2) * (_num)) / (_den);  \
    MAT33_ELMT((_mat1),1,0) += (MAT33_ELMT((_mat2),1,0) * (_num)) / (_den);  \
    MAT33_ELMT((_mat1),1,1) += (MAT33_ELMT((_mat2),1,1) * (_num)) / (_den);  \
    MAT33_ELMT((_mat1),1,2) += (MAT33_ELMT((_mat2),1,2) * (_num)) / (_den);  \
    MAT33_ELMT((_mat1),2,0) += (MAT33_ELMT((_mat2),2,0) * (_num)) / (_den);  \
    MAT33_ELMT((_mat1),2,1) += (MAT33_ELMT((_mat2),2,1) * (_num)) / (_den);  \
    MAT33_ELMT((_mat1),2,2) += (MAT33_ELMT((_mat2),2,2) * (_num)) / (_den);  \
  }

#define MAT33_MULT2(_mat0, _mat1, _mat2, _s) {         \
	MAT33_ELMT((_mat0),0,0) = (MAT33_ELMT((_mat1),0,0)*MAT33_ELMT((_mat2),0,0) + MAT33_ELMT((_mat1),0,1)*MAT33_ELMT((_mat2),1,0) + MAT33_ELMT((_mat1),0,2)*MAT33_ELMT((_mat2),2,0)) * (_s);  \
	MAT33_ELMT((_mat0),0,1) = (MAT33_ELMT((_mat1),0,0)*MAT33_ELMT((_mat2),0,1) + MAT33_ELMT((_mat1),0,1)*MAT33_ELMT((_mat2),1,1) + MAT33_ELMT((_mat1),0,2)*MAT33_ELMT((_mat2),2,1)) * (_s);  \
	MAT33_ELMT((_mat0),0,2) = (MAT33_ELMT((_mat1),0,0)*MAT33_ELMT((_mat2),0,2) + MAT33_ELMT((_mat1),0,1)*MAT33_ELMT((_mat2),1,2) + MAT33_ELMT((_mat1),0,2)*MAT33_ELMT((_mat2),2,2)) * (_s);  \
	MAT33_ELMT((_mat0),1,0) = (MAT33_ELMT((_mat1),1,0)*MAT33_ELMT((_mat2),0,0) + MAT33_ELMT((_mat1),1,1)*MAT33_ELMT((_mat2),1,0) + MAT33_ELMT((_mat1),1,2)*MAT33_ELMT((_mat2),2,0)) * (_s);  \
	MAT33_ELMT((_mat0),1,1) = (MAT33_ELMT((_mat1),1,0)*MAT33_ELMT((_mat2),0,1) + MAT33_ELMT((_mat1),1,1)*MAT33_ELMT((_mat2),1,1) + MAT33_ELMT((_mat1),1,2)*MAT33_ELMT((_mat2),2,1)) * (_s);  \
	MAT33_ELMT((_mat0),1,2) = (MAT33_ELMT((_mat1),1,0)*MAT33_ELMT((_mat2),0,2) + MAT33_ELMT((_mat1),1,1)*MAT33_ELMT((_mat2),1,2) + MAT33_ELMT((_mat1),1,2)*MAT33_ELMT((_mat2),2,2)) * (_s);  \
	MAT33_ELMT((_mat0),2,0) = (MAT33_ELMT((_mat1),2,0)*MAT33_ELMT((_mat2),0,0) + MAT33_ELMT((_mat1),2,1)*MAT33_ELMT((_mat2),1,0) + MAT33_ELMT((_mat1),2,2)*MAT33_ELMT((_mat2),2,0)) * (_s);  \
	MAT33_ELMT((_mat0),2,1) = (MAT33_ELMT((_mat1),2,0)*MAT33_ELMT((_mat2),0,1) + MAT33_ELMT((_mat1),2,1)*MAT33_ELMT((_mat2),1,1) + MAT33_ELMT((_mat1),2,2)*MAT33_ELMT((_mat2),2,1)) * (_s);  \
	MAT33_ELMT((_mat0),2,2) = (MAT33_ELMT((_mat1),2,0)*MAT33_ELMT((_mat2),0,2) + MAT33_ELMT((_mat1),2,1)*MAT33_ELMT((_mat2),1,2) + MAT33_ELMT((_mat1),2,2)*MAT33_ELMT((_mat2),2,2)) * (_s);  \
  }

#define MAT33_MULT(_mat0, _mat1, _mat2)    MAT33_MULT2(_mat0, _mat1, _mat2, 1)

#endif /* DELFLY_ALGEBRA_INT_H_2 */
