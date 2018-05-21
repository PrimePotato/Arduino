#include <MrscGyro.h>
#include <MrscRx.h>
#include <MrscServo.h>

#define YAW_LAMBDA 0.5
#define YAW_STABILITY_CONTROL 45
#define RC_STEERING_LAMBDA 0.5
#define RC_THROTTLE_LAMBDA 0.2

MrscServo servo;
MrscGyro gyro;

uint16_t yaw_correction;
uint16_t throttle_value;
uint16_t steering_value;
uint16_t steering_weight;

void setup() {
  rx_setup();
  gyro.initialise_gyro();
  gyro.retrieve_gyro_data();
  servo.initialise_servos();
}



double ewma(double new_val, double old_val, double lambda)
{
  return new_val * lambda + (1 - lambda) * old_val;
}


double log_sgn(double x)
 {
  if (x>=0) return log(x-1);
  else return log(1-x);
 }

void loop() {
  rc_read_values();
  gyro.retrieve_gyro_data();
  servo.initialise_servos();
  servo.map_rc_values_to_servo(rc_values);

  yaw_correction = ewma(gyro.yaw_rate(), yaw_correction, YAW_LAMBDA);
  yaw_correction = map(log_sgn(yaw_correction), log_sgn(-40000), log_sgn(40000), -YAW_STABILITY_CONTROL, YAW_STABILITY_CONTROL);

  steering_value = ewma(rx_servo_vals[RC_STEERING], servo.get_throttle(), RC_STEERING_LAMBDA);
  steering_weight = map(abs(90. - servo.get_steering()), 0., 90., 1. , 0.);
  steering_value = yaw_correction + steering_value;
  throttle_value = ewma(rx_servo_vals[RC_THROTTLE], servo.get_throttle(), RC_THROTTLE_LAMBDA);
  
  servo.set_steering(steering_value);
  servo.set_throttle(throttle_value);
  
}
