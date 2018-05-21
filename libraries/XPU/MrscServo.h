#ifndef MrscServo_h
#define MrscServo_h

#define LIBCALL_ENABLEINTERRUPT

#include <Arduino.h>
#include <Servo.h>

#define SERIAL_PORT_SPEED 57600

#define RC_NUM_CHANNELS 3
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

class MrscServo
{
  public:
    MrscServo();

    void output_servo_values();
    void initialise_servos();
    void map_rc_values_to_servo(uint16_t rcv[RC_NUM_CHANNELS]);

    void set_throttle(uint16_t speed);
    void set_steering(uint16_t angle);
    uint16_t get_throttle();
    uint16_t get_steering();

    int ch_lims[RC_NUM_CHANNELS][4] = {
        {1000, 2000, 0, 180},
        {1000, 2000, 0, 180},
        {1000, 2000, 0, 180}};

    Servo servo_steering;
    Servo servo_throttle;

    uint32_t rxsv[RC_NUM_CHANNELS];

  private:
    int16_t steering_value = 90, throttle_value = 90, yaw_correction = 0;
    volatile double steering_weight=0;
    const int MPU_addr = 0x68;
};

#endif
