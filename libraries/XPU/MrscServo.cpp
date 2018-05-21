#include "Arduino.h"
#include <Servo.h>
#include <MrscServo.h>


MrscServo::MrscServo()
{
}


void MrscServo::map_rc_values_to_servo(uint16_t rcv[RC_NUM_CHANNELS])
{
  for (int i = 0; i < RC_NUM_CHANNELS; i++)
  {
    if (rcv[i] < ch_lims[i][0])
    {
      rxsv[i] = ch_lims[i][2];
    }
    else if (rcv[i] > ch_lims[i][1])
    {
      rxsv[i] = ch_lims[i][3];
    }
    else
    {
      rxsv[i] = map(rcv[i], ch_lims[i][0], ch_lims[i][1], ch_lims[i][2], ch_lims[i][3]);
    }
  }
}
void MrscServo::set_throttle(uint16_t speed){
    throttle_value = speed;
    servo_throttle.write(speed);
}
uint16_t MrscServo::get_throttle(){
    return throttle_value;
}
void MrscServo::set_steering(uint16_t angle){
    steering_value= angle;
    servo_steering.write(angle);
}
uint16_t MrscServo::get_steering(){
    return steering_value;
}
void MrscServo::output_servo_values()
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

void MrscServo::initialise_servos()
{
  steering_value = int((RC_STR_LIM_LEFT + RC_STR_LIM_RIGHT) * 0.5);
  throttle_value = 90;

  servo_throttle.attach(8);
  servo_steering.attach(9);

  servo_steering.write(steering_value);
  servo_throttle.write(throttle_value);
}
