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
#define INT32_SAT(_i, _m) INT32_SAT_2(_i, -(_m), _m) //(((_i) > (_m))? (_m) : (((_i) < -(_m))? -(_m) : (_i)))
#define INT32_SAT_2(_i, _min, _max) (((_i) > (_max))? (_max) : (((_i) < (_min))? (_min) : (_i)))
#define INT32_STRIM(_i, _m)  { _i = INT32_SAT(_i, _m); }

#define INT32_SIGN(_i)    ((_i >= 0)? 1 : -1)

#define CUBE(_a)          ((_a)*(_a)*(_a))
#define HYCUBE(_a)        ((_a)*(_a)*(_a)*(_a))

#define VECT3_ZERO(_v)    INT32_VECT3_ZERO(_v)
#define MAT33_ZERO(_m)    INT32_MAT33_ZERO(_m)
#define EULERS_ZERO(_e)   EULERS_ASSIGN(_e, 0, 0, 0)
#define RATES_ZERO(_r)    RATES_ASSIGN(_r, 0, 0, 0)


struct Int32VectL {
  int32_t fwd;
  int32_t ver;
};

union Int32VectLong {
  struct Int32Vect2 xy;
  struct Int32VectL fv;
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

/* a += b * num / den */
#define VECT2_ADD_SCALED2(_a, _b, _num, _den) {   \
    (_a).x += ((_b).x * (_num)) / (_den);           \
    (_a).y += ((_b).y * (_num)) / (_den);           \
  }

/* a += b * s */
#define VECT2_ADD_SCALED(_a, _b, _s)		VECT2_ADD_SCALED2(_a, _b, _s, 1)

/* c = (a - b) * num / den */
#define VECT2_DIFF_SCALED2(_c, _a, _b, _num, _den) { 	\
	(_c).x = (((_a).x - (_b).x) * (_num)) / (_den);			\
	(_c).y = (((_a).y - (_b).y) * (_num)) / (_den);			\
  }

/* c = (a - b) * s */
#define VECT2_DIFF_SCALED(_c, _a, _b, _s)   VECT2_DIFF_SCALED2(_a, _b, _s, 1)

/* a o b, o in {<, <=, ==, !=, >=, >} */
#define VECT2_CP(_a, _cp, _b)     ( \
    (_a).x _cp (_b).x &&            \
    (_a).y _cp (_b).y               \
  )

/* a == b */
#define VECT2_EQUALS(_a, _b)      VECT2_CP(_a, ==, _b)

/* element-wise a o b, o in {<, <=, ==, !=, >=, >} */
#define VECT2_EW_CP(_a, _cp, _s)  ( \
    (_a).x _cp (_s) &&     \
    (_a).y _cp (_s)        \
  )

#define VECT2_EQUALS_ZERO(_v)     VECT2_EW_CP(_v, ==, 0)

#define VECT2_GET_FWD(_v,_h)				(((_v).x*pprz_itrig_cos(_h) - (_v).y*pprz_itrig_sin(_h))/(1<<INT32_TRIG_FRAC))
#define VECT2_GET_LAT(_v,_h)				(((_v).x*pprz_itrig_sin(_h) + (_v).y*pprz_itrig_cos(_h))/(1<<INT32_TRIG_FRAC))

/* a += b * num / den */
#define VECT3_ADD_SCALED2(_a, _b, _num, _den) {   \
    (_a).x += ((_b).x * (_num)) / (_den);        \
    (_a).y += ((_b).y * (_num)) / (_den);        \
    (_a).z += ((_b).z * (_num)) / (_den);        \
  }

/* a = b * num / den */
#define VECT3_ASSIGN_SCALED2(_a, _b, _num, _den) {   \
    (_a).x = ((_b).x * (_num)) / (_den);        \
    (_a).y = ((_b).y * (_num)) / (_den);        \
    (_a).z = ((_b).z * (_num)) / (_den);        \
  }

/* a o b, o in {<, <=, ==, !=, >=, >} */
#define VECT3_CP(_a, _cp, _b)   	(	\
    (_a).x _cp (_b).x &&	    			\
    (_a).y _cp (_b).y &&		    		\
    (_a).z _cp (_b).z				        \
  )

/* a == b */
#define VECT3_EQUALS(_a, _b)      VECT3_CP(_a, ==, _b)

/* element-wise a o b, o in {<, <=, ==, !=, >=, >} */
#define VECT3_EW_CP(_a, _cp, _s)  ( \
    (_a).x _cp (_s) &&     \
    (_a).y _cp (_s) &&     \
    (_a).z _cp (_s)        \
  )

/* m1 += m2 */
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

/* m1 += m2 * num / den */
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

/* determinant of matrix with fraction
 * |_m| with same fraction */
#define INT32_MAT33_DET(_det, _m, _frac) {		\
	const int32_t m00 = MAT33_ELMT((_m),1,1)*MAT33_ELMT((_m),2,2) - MAT33_ELMT((_m),1,2)*MAT33_ELMT((_m),2,1);    \
	const int32_t m10 = MAT33_ELMT((_m),0,1)*MAT33_ELMT((_m),2,2) - MAT33_ELMT((_m),0,2)*MAT33_ELMT((_m),2,1);    \
	const int32_t m20 = MAT33_ELMT((_m),0,1)*MAT33_ELMT((_m),1,2) - MAT33_ELMT((_m),0,2)*MAT33_ELMT((_m),1,1);    \
	_det = (MAT33_ELMT((_m),0,0)*m00 - MAT33_ELMT((_m),1,0)*m10 + MAT33_ELMT((_m),2,0)*m20)/(1<<(2*_frac)); \
}

/* invariant of matrix with fraction
 * invS = 1/det(S) com(S)'
 * invS with same fraction */
#define INT32_MAT33_INV(_minv, _m, _frac) {            \
    /*pair-wise element products, with double fraction */  \
    const int32_t m00 = MAT33_ELMT((_m),1,1)*MAT33_ELMT((_m),2,2) - MAT33_ELMT((_m),1,2)*MAT33_ELMT((_m),2,1);    \
    const int32_t m10 = MAT33_ELMT((_m),0,1)*MAT33_ELMT((_m),2,2) - MAT33_ELMT((_m),0,2)*MAT33_ELMT((_m),2,1);    \
    const int32_t m20 = MAT33_ELMT((_m),0,1)*MAT33_ELMT((_m),1,2) - MAT33_ELMT((_m),0,2)*MAT33_ELMT((_m),1,1);    \
    const int32_t m01 = MAT33_ELMT((_m),1,0)*MAT33_ELMT((_m),2,2) - MAT33_ELMT((_m),1,2)*MAT33_ELMT((_m),2,0);    \
    const int32_t m11 = MAT33_ELMT((_m),0,0)*MAT33_ELMT((_m),2,2) - MAT33_ELMT((_m),0,2)*MAT33_ELMT((_m),2,0);    \
    const int32_t m21 = MAT33_ELMT((_m),0,0)*MAT33_ELMT((_m),1,2) - MAT33_ELMT((_m),0,2)*MAT33_ELMT((_m),1,0);    \
    const int32_t m02 = MAT33_ELMT((_m),1,0)*MAT33_ELMT((_m),2,1) - MAT33_ELMT((_m),1,1)*MAT33_ELMT((_m),2,0);    \
    const int32_t m12 = MAT33_ELMT((_m),0,0)*MAT33_ELMT((_m),2,1) - MAT33_ELMT((_m),0,1)*MAT33_ELMT((_m),2,0);    \
    const int32_t m22 = MAT33_ELMT((_m),0,0)*MAT33_ELMT((_m),1,1) - MAT33_ELMT((_m),0,1)*MAT33_ELMT((_m),1,0);    \
    /* determinant with same fraction */    \
    const int32_t det = (MAT33_ELMT((_m),0,0)*m00 - MAT33_ELMT((_m),1,0)*m10 + MAT33_ELMT((_m),2,0)*m20)/(1<<(2*_frac)); \
    if (det != 0) {          \
      MAT33_ELMT((_minv),0,0) =  m00 / det;           \
      MAT33_ELMT((_minv),1,0) = -m01 / det;           \
      MAT33_ELMT((_minv),2,0) =  m02 / det;           \
      MAT33_ELMT((_minv),0,1) = -m10 / det;           \
      MAT33_ELMT((_minv),1,1) =  m11 / det;           \
      MAT33_ELMT((_minv),2,1) = -m12 / det;           \
      MAT33_ELMT((_minv),0,2) =  m20 / det;           \
      MAT33_ELMT((_minv),1,2) = -m21 / det;           \
      MAT33_ELMT((_minv),2,2) =  m22 / det;           \
    }                 \
  }

//const float m01 = MAT33_ELMT((_m),1,0)*MAT33_ELMT((_m),2,2) - MAT33_ELMT((_m),1,2)*MAT33_ELMT((_m),2,0);
//const float m11 = MAT33_ELMT((_m),0,0)*MAT33_ELMT((_m),2,2) - MAT33_ELMT((_m),0,2)*MAT33_ELMT((_m),2,0);
//const float m21 = MAT33_ELMT((_m),0,0)*MAT33_ELMT((_m),1,2) - MAT33_ELMT((_m),0,2)*MAT33_ELMT((_m),1,0);
//const float m02 = MAT33_ELMT((_m),1,0)*MAT33_ELMT((_m),2,1) - MAT33_ELMT((_m),1,1)*MAT33_ELMT((_m),2,0);
//const float m12 = MAT33_ELMT((_m),0,0)*MAT33_ELMT((_m),2,1) - MAT33_ELMT((_m),0,1)*MAT33_ELMT((_m),2,0);
//const float m22 = MAT33_ELMT((_m),0,0)*MAT33_ELMT((_m),1,1) - MAT33_ELMT((_m),0,1)*MAT33_ELMT((_m),1,0);

/* m0 = m1 * m2 * num / den */
#define MAT33_MULT2(_mat0, _mat1, _mat2, _num, _den) {         \
	MAT33_ELMT((_mat0),0,0) = ( (MAT33_ELMT((_mat1),0,0)*MAT33_ELMT((_mat2),0,0) + MAT33_ELMT((_mat1),0,1)*MAT33_ELMT((_mat2),1,0) + MAT33_ELMT((_mat1),0,2)*MAT33_ELMT((_mat2),2,0)) * (_num) ) / (_den);  \
	MAT33_ELMT((_mat0),0,1) = ( (MAT33_ELMT((_mat1),0,0)*MAT33_ELMT((_mat2),0,1) + MAT33_ELMT((_mat1),0,1)*MAT33_ELMT((_mat2),1,1) + MAT33_ELMT((_mat1),0,2)*MAT33_ELMT((_mat2),2,1)) * (_num) ) / (_den);  \
	MAT33_ELMT((_mat0),0,2) = ( (MAT33_ELMT((_mat1),0,0)*MAT33_ELMT((_mat2),0,2) + MAT33_ELMT((_mat1),0,1)*MAT33_ELMT((_mat2),1,2) + MAT33_ELMT((_mat1),0,2)*MAT33_ELMT((_mat2),2,2)) * (_num) ) / (_den);  \
	MAT33_ELMT((_mat0),1,0) = ( (MAT33_ELMT((_mat1),1,0)*MAT33_ELMT((_mat2),0,0) + MAT33_ELMT((_mat1),1,1)*MAT33_ELMT((_mat2),1,0) + MAT33_ELMT((_mat1),1,2)*MAT33_ELMT((_mat2),2,0)) * (_num) ) / (_den);  \
	MAT33_ELMT((_mat0),1,1) = ( (MAT33_ELMT((_mat1),1,0)*MAT33_ELMT((_mat2),0,1) + MAT33_ELMT((_mat1),1,1)*MAT33_ELMT((_mat2),1,1) + MAT33_ELMT((_mat1),1,2)*MAT33_ELMT((_mat2),2,1)) * (_num) ) / (_den);  \
	MAT33_ELMT((_mat0),1,2) = ( (MAT33_ELMT((_mat1),1,0)*MAT33_ELMT((_mat2),0,2) + MAT33_ELMT((_mat1),1,1)*MAT33_ELMT((_mat2),1,2) + MAT33_ELMT((_mat1),1,2)*MAT33_ELMT((_mat2),2,2)) * (_num) ) / (_den);  \
	MAT33_ELMT((_mat0),2,0) = ( (MAT33_ELMT((_mat1),2,0)*MAT33_ELMT((_mat2),0,0) + MAT33_ELMT((_mat1),2,1)*MAT33_ELMT((_mat2),1,0) + MAT33_ELMT((_mat1),2,2)*MAT33_ELMT((_mat2),2,0)) * (_num) ) / (_den);  \
	MAT33_ELMT((_mat0),2,1) = ( (MAT33_ELMT((_mat1),2,0)*MAT33_ELMT((_mat2),0,1) + MAT33_ELMT((_mat1),2,1)*MAT33_ELMT((_mat2),1,1) + MAT33_ELMT((_mat1),2,2)*MAT33_ELMT((_mat2),2,1)) * (_num) ) / (_den);  \
	MAT33_ELMT((_mat0),2,2) = ( (MAT33_ELMT((_mat1),2,0)*MAT33_ELMT((_mat2),0,2) + MAT33_ELMT((_mat1),2,1)*MAT33_ELMT((_mat2),1,2) + MAT33_ELMT((_mat1),2,2)*MAT33_ELMT((_mat2),2,2)) * (_num) ) / (_den);  \
  }

/* m0 = m1 * m2 */
#define MAT33_MULT(_mat0, _mat1, _mat2)    MAT33_MULT2(_mat0, _mat1, _mat2, 1, 1)

/* v0 = m * v1 * num / den */
#define MAT33_VECT3_MULT2(_vout, _mat, _vin, _num, _den) {    \
    (_vout).x = ( (MAT33_ELMT((_mat), 0, 0) * (_vin).x + \
                  MAT33_ELMT((_mat), 0, 1) * (_vin).y + \
                  MAT33_ELMT((_mat), 0, 2) * (_vin).z)*(_num)) / (_den);  \
    (_vout).y = ( (MAT33_ELMT((_mat), 1, 0) * (_vin).x +   \
                  MAT33_ELMT((_mat), 1, 1) * (_vin).y +   \
                  MAT33_ELMT((_mat), 1, 2) * (_vin).z)*(_num)) / (_den);  \
    (_vout).z = ( (MAT33_ELMT((_mat), 2, 0) * (_vin).x + \
                  MAT33_ELMT((_mat), 2, 1) * (_vin).y + \
                  MAT33_ELMT((_mat), 2, 2) * (_vin).z)*(_num)) / (_den);  \
  }

#endif /* DELFLY_ALGEBRA_INT_H_2 */
