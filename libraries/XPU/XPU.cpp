
#define LIBCALL_ENABLEINTERRUPT

#include "Arduino.h"
#include <EnableInterrupt.h>
#include <Wire.h>
#include <Servo.h>
#include <XPU.h>

XPU::XPU()
{
}

void XPU::calc_input(uint8_t channel, uint8_t input_pin)
{
  if (digitalRead(input_pin) == HIGH)
  {
    rc_start[channel] = micros();
  }
  else
  {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void XPU::calc_ch1()
{
  this->calc_input(RC_CH1, RC_CH1_INPUT);
}
void XPU::calc_ch2()
{
  this->calc_input(RC_CH2, RC_CH2_INPUT);
}
void XPU::calc_ch3()
{
  this->calc_input(RC_CH3, RC_CH3_INPUT);
}

void XPU::rc_read_values()
{
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void XPU::initialise_rx()
{
}

void XPU::initialise_gyro()
{
  initialise_servos();
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(SERIAL_PORT_SPEED);
}

void XPU::retrieve_gyro_data()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  for (int i = 0; i < GYRO_NUM_CHANNELS; i++)
  {
    gyro_values[i] = Wire.read() << 8 | Wire.read();
  }
}

double XPU::ewma(double new_val, double old_val, double lambda)
{
  return new_val * lambda + (1 - lambda) * old_val;
}

void XPU::map_rc_values_to_servo()
{
  for (int i = 0; i < RC_NUM_CHANNELS; i++)
  {
    if (rc_values[i] < ch_lims[i][0])
    {
      rx_servo_vals[i] = ch_lims[i][2];
    }
    else if (rc_values[i] > ch_lims[i][1])
    {
      rx_servo_vals[i] = ch_lims[i][3];
    }
    else
    {
      rx_servo_vals[i] = map(rc_values[i], ch_lims[i][0], ch_lims[i][1], ch_lims[i][2], ch_lims[i][3]);
    }
  }
}

void XPU::output_rx_values()
{
  for (int i = 0; i < RC_NUM_CHANNELS; i++)
  {
    Serial.print("CH");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(rc_values[i]);
    Serial.print("\t");
    Serial.print("SERVO");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(rx_servo_vals[i]);
    Serial.print("\t");
  }
  Serial.println("");
}

void XPU::output_gyro_values()
{
  for (int i = 0; i < GYRO_NUM_CHANNELS; i++)
  {
    Serial.print("Gyro Value ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(gyro_values[i]);
  }
  Serial.println("");
}

void XPU::output_servo_values()
{
  Serial.print("Steering Value: ");
  Serial.print(steering_value);
  Serial.print("\t");
  Serial.println("Throttle Value:  ");
  Serial.print(throttle_value);
  Serial.print("\t");
  Serial.println("Steering Weight:  ");
  Serial.print(steering_weight);
  Serial.print("\t");
  Serial.println("YAW Corection:  ");
  Serial.println(yaw_correction);
}

void XPU::initialise_servos()
{
  steering_value = int((RC_STR_LIM_LEFT + RC_STR_LIM_RIGHT) * 0.5);
  throttle_value = 90;

  servo_throttle.attach(8);
  servo_steering.attach(9);

  servo_steering.write(steering_value);
  servo_throttle.write(throttle_value);
}
