#ifndef MrscGyro_h
#define MrscGyro_h

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

#define GYRO_AcX 0
#define GYRO_AcY 1
#define GYRO_AcZ 2
#define GYRO_Tmp 3
#define GYRO_GyX 4
#define GYRO_GyY 5
#define GYRO_GyZ 6

#define GYRO_NUM_CHANNELS 7
#define SERIAL_PORT_SPEED 57600

class MrscGyro
{
  public:
    MrscGyro();

    uint16_t yaw_rate();
    void retrieve_gyro_data();
    void output_gyro_values();
    void initialise_gyro(); 
    

  private:
    const int MPU_addr = 0x68;
    uint16_t gyro_values[GYRO_NUM_CHANNELS];
};

#endif