/*
 * Copyright (C) 2016 Torbjoern Cunis <t.cunis@tudelft.nl>
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
 * @file /paparazzi/paparazzi/sw/airborne/filters/averaging_filter.h
 * @author Torbjoern Cunis
 * Averaging filter of fixed and dynamic length.
 */

#ifndef AVERAGING_FILTER_H_
#define AVERAGING_FILTER_H_

#include "std.h"


#define AVERAGING_MAX_CAPACITY    20

/* Increments the pointer to a ring-buffer. */
#define INCR_PTR(index)           index = ((index+1) % AVERAGING_MAX_CAPACITY)


struct AveragingFilter {
  float data[AVERAGING_MAX_CAPACITY];
  float sum;
  uint8_t capacity;
  uint8_t length;
  uint8_t index_head; //index of element inserted earliest
  uint8_t index_tail; //index AFTER element inserted last
};


static inline void init_averaging_filter ( struct AveragingFilter* filter, uint8_t capacity ) {

  filter->sum = 0;
  filter->length = 0;
  filter->capacity = Min(capacity, AVERAGING_MAX_CAPACITY);
  filter->index_head = filter->index_tail = 0;

  for (int i_ = 0; i_ < AVERAGING_MAX_CAPACITY; i_++) { filter->data[i_] = 0; }
}

static inline float pop_averaging_filter ( struct AveragingFilter* filter ) {

  if ( filter->length == 0 ) {
    return 0.f; //no element to pop
  }

  //else: remove and return head value
  float head_value = filter->data[filter->index_head];
  INCR_PTR(filter->index_head);

  //update sum and length
  filter->sum -= head_value;
  filter->length--;

  return head_value;
}

/* Puts a new value to the end of the averaging filter. */
static inline void update_averaging_filter ( struct AveragingFilter* filter, float value ) {

  //if filter capacity is reached remove head element
  if ( filter->length >= filter->capacity ) {
    pop_averaging_filter(filter);
  }

  //add new value
  filter->data[filter->index_tail] = value;
  INCR_PTR(filter->index_tail);

  //update sum and length
  filter->sum += value;
  filter->length++;
}

static inline float get_averaging_filter ( struct AveragingFilter* filter ) {

  if ( filter->length == 0 ) {
    return 0.f; //an empty filter has zero average
  }

  //else:
  return ( filter->sum / filter->length );
}

static inline void shrink_averaging_filter ( struct AveragingFilter* filter ) {

  if ( filter->capacity == 1 ) {
    return; //minimum capacity reached, nothing to do
  }

  //else:
  //if filter length equals capacity remove head element
  if ( filter->length >= filter->capacity ) {
    pop_averaging_filter(filter);
  }

  filter->capacity--;
}

static inline void expand_averaging_filter ( struct AveragingFilter* filter ) {

  if ( filter->capacity == AVERAGING_MAX_CAPACITY ) {
    return; //maximum capacity reached, nothing to do
  }

  //else
  filter->capacity++;
}


#endif /* AVERAGING_FILTER_H_ */
