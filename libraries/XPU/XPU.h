#ifndef xpu_h
#define xpu_h

#define LIBCALL_ENABLEINTERRUPT

#include "Arduino.h"
#include <EnableInterrupt.h>
#include <Wire.h>
#include <Servo.h>
#include <XPU.h>

#define SERIAL_PORT_SPEED 57600

#define RC_NUM_CHANNELS  3
#define RC_STR_LIM_LEFT 30
#define RC_STR_LIM_RIGHT 140

#define GYRO_NUM_CHANNELS 7

#define RC_STEERING 0
#define RC_THROTTLE 1
#define RC_CH1 0
#define RC_CH2 1
#define RC_CH3 2

#define RC_CH1_INPUT A0
#define RC_CH2_INPUT A1
#define RC_CH3_INPUT A2

class XPU
{
  public:
    XPU();

    int ch_lims[RC_NUM_CHANNELS][4] = {
      {1000, 2000, 0, 180},
      {1000, 2000, 0, 180},
      {1000, 2000, 0, 180}
    };

    void rc_read_values();
    void retrieve_gyro_data();
    double ewma(double new_val, double old_val, double lambda);
    void map_rc_values_to_servo();
    void output_rx_values();
    void output_gyro_values();
    void output_servo_values();
    void initialise_servos();
    void initialise_gyro(); 
    void initialise_rx();           
    void calc_ch1();
    void calc_ch2();
    void calc_ch3();
    void calc_input(uint8_t channel, uint8_t input_pin);
    

    Servo servo_steering;
    Servo servo_throttle;

  private:
    uint32_t rc_start[RC_NUM_CHANNELS];
    uint16_t gyro_values[GYRO_NUM_CHANNELS], rc_values[RC_NUM_CHANNELS], gyro_values_old[GYRO_NUM_CHANNELS], 
      rx_servo_vals_old[RC_NUM_CHANNELS], rx_servo_vals[RC_NUM_CHANNELS];
    int16_t steering_value, throttle_value, yaw_correction;
    volatile uint16_t rc_shared[RC_NUM_CHANNELS];
    volatile double steering_weight;
    const int MPU_addr = 0x68; // I2C address of the MPU-6050
};

#endif
