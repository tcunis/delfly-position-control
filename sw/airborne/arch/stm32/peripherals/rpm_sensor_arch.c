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

#include "modules/rpm_sensor/rpm_sensor.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include "mcu_periph/gpio.h"

// for timer_get_frequency
#include "mcu_arch.h"

// Telemetry to test values
#include "subsystems/datalink/telemetry.h"

#define ONE_MHZ_CLK 1000000
#ifdef NVIC_TIM_IRQ_PRIO
#define RPM_PPM_IRQ_PRIO  NVIC_TIM_IRQ_PRIO
#else
#define RPM_PPM_IRQ_PRIO 2
#endif

#define RPM_RCC_TIM_PPM         RCC_TIM3
#define RPM_PPM_TIMER           TIM3
#define RPM_PPM_CHANNEL         TIM_IC1
#define RPM_PPM_GPIO_PORT       GPIOC
#define RPM_PPM_GPIO_PIN        GPIO6
#define RPM_PPM_GPIO_AF         0
#define RPM_PPM_TICKS_PER_USEC  6
#define RPM_PPM_TIMER_INPUT     TIM_IC_IN_TI1
#define RPM_PPM_IRQ             NVIC_TIM3_IRQ
#define RPM_PPM_CC_IE           TIM_DIER_CC1IE
#define RPM_PPM_CC_IF           TIM_SR_CC1IF

static uint32_t timer_rollover_cnt;
uint32_t rpm_last_ppm;

static void send_rpm(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_CSC_CAN_DEBUG(trans, dev, AC_ID, &rpm_last_ppm, 0);
}

void rpm_sensor_arch_init(void)
{
  register_periodic_telemetry(DefaultPeriodic, "CSC_CAN_DEBUG", send_rpm);

  /* timer clock enable */
  rcc_periph_clock_enable(RPM_RCC_TIM_PPM);
  rcc_periph_clock_enable(RCC_AFIO);

  /* GPIO configuration as input capture for timer
   * Using RADIO_IRQ pin on Lisa/S (=PC6).
   * Requires remapping to get timer 3 channel 1.
   */
  gpio_enable_clock(RPM_PPM_GPIO_PORT);
  gpio_primary_remap(FALSE, AFIO_MAPR_TIM3_REMAP_FULL_REMAP);
  gpio_set_mode(RPM_PPM_GPIO_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, RPM_PPM_GPIO_PIN);

  /* Time Base configuration */
  timer_reset(RPM_PPM_TIMER);
  timer_set_mode(RPM_PPM_TIMER, TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_period(RPM_PPM_TIMER, 0xFFFF);
  //uint32_t timer_clk = timer_get_frequency(RPM_PPM_TIMER);
  //timer_set_prescaler(RPM_PPM_TIMER, (timer_clk / (RPM_PPM_TICKS_PER_USEC * ONE_MHZ_CLK)) - 1);
  timer_set_prescaler(RPM_PPM_TIMER, 256);

  /* TIM configuration: Input Capture mode ---------------------
      The Rising edge is used as active edge
   ------------------------------------------------------------ */
  timer_ic_set_polarity(RPM_PPM_TIMER, RPM_PPM_CHANNEL, TIM_IC_RISING);
  timer_ic_set_input(RPM_PPM_TIMER, RPM_PPM_CHANNEL, RPM_PPM_TIMER_INPUT);
  timer_ic_set_prescaler(RPM_PPM_TIMER, RPM_PPM_CHANNEL, TIM_IC_PSC_OFF);
  timer_ic_set_filter(RPM_PPM_TIMER, RPM_PPM_CHANNEL, TIM_IC_OFF);

  /* Enable timer Interrupt(s). */
  nvic_set_priority(RPM_PPM_IRQ, RPM_PPM_IRQ_PRIO);
  nvic_enable_irq(RPM_PPM_IRQ);

  /* Enable the Capture/Compare and Update interrupt requests. */
  timer_enable_irq(RPM_PPM_TIMER, (RPM_PPM_CC_IE));// | TIM_DIER_UIE));

  /* Enable capture channel. */
  timer_ic_enable(RPM_PPM_TIMER, RPM_PPM_CHANNEL);

  /* TIM enable counter */
  timer_enable_counter(RPM_PPM_TIMER);

  timer_rollover_cnt = 0;
}


void tim3_isr(void)
{
  rpm_last_ppm++;
  if ((TIM3_SR & RPM_PPM_CC_IF) != 0) {
    timer_clear_flag(TIM3, RPM_PPM_CC_IF);

    uint32_t now = timer_get_counter(TIM3) + timer_rollover_cnt;
    //rpm_last_ppm++;
    // call a function here
    //send_rpm();
    timer_set_counter(RPM_PPM_TIMER, 0);
  } else if ((TIM3_SR & TIM_SR_UIF) != 0) {
    timer_rollover_cnt += (1 << 16);
    timer_clear_flag(TIM3, TIM_SR_UIF);
  }
}


