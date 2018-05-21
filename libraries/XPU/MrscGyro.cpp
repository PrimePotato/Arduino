#include <MrscGyro.h>

MrscGyro::MrscGyro(){

}

uint16_t MrscGyro::yaw_rate(){
  return gyro_values[GYRO_GyZ];
}

void MrscGyro::output_gyro_values()
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

void MrscGyro::initialise_gyro()
{
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(SERIAL_PORT_SPEED);
}

void MrscGyro::retrieve_gyro_data()
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