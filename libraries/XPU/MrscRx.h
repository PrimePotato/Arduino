#ifndef mrsc_rx_h
#define mrsc_rx_h

#define SERIAL_PORT_SPEED 57600

#define RC_NUM_CHANNELS  3
#define RC_STEERING 0
#define RC_THROTTLE 1
#define RC_CH1 0
#define RC_CH2 1
#define RC_CH3 2

#define RC_CH1_INPUT A0
#define RC_CH2_INPUT A1
#define RC_CH3_INPUT A2

#include "Arduino.h"
#include <EnableInterrupt.h>

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

uint16_t rx_servo_vals_old[RC_NUM_CHANNELS], rx_servo_vals[RC_NUM_CHANNELS];

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }

void rx_setup() {
  Serial.begin(SERIAL_PORT_SPEED);

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);

}

void rx_loop() {
  rc_read_values();
}

#endif