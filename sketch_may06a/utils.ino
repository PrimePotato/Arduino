#include <EnableInterrupt.h>
#include <Wire.h>
#include <Servo.h>

#define SERIAL_PORT_SPEED 57600
#define RC_NUM_CHANNELS  3
#define GYRO_NUM_CHANNELS 7
#define SERVO_NUM_CHANNELS 3

#define YAW_LAMBDA 0.9
#define RC_STEERING_LAMBDA 0.9
#define RC_THROTTLE_LAMBDA 0.6

#define RC_STR_LIM_LEFT 30
#define RC_STR_LIM_RIGHT 140

#define RC_STEERING  0
#define RC_THROTTLE  1
#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2

#define RC_CH1_INPUT  A0
#define RC_CH2_INPUT  A1
#define RC_CH3_INPUT  A2

#define GYRO_AcX 0
#define GYRO_AcY 1
#define GYRO_AcZ 2
#define GYRO_Tmp 3
#define GYRO_GyX 4
#define GYRO_GyY 5
#define GYRO_GyZ 6

int ch_lims[RC_NUM_CHANNELS][4] = {{1000, 2000, 0, 180}, {1000, 2000, 0, 180}, {1000, 2000, 0, 180}};
long loop_count = 0;
uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
uint16_t gyro_values[GYRO_NUM_CHANNELS];
uint16_t gyro_values_old[GYRO_NUM_CHANNELS];
uint16_t rx_servo_vals_old[RC_NUM_CHANNELS];
uint16_t rx_servo_vals[RC_NUM_CHANNELS];
int16_t steering_value, throttle_value, yaw_correction;
double steering_weight;

Servo servo_steering;
Servo servo_throttle;

volatile uint16_t rc_shared[RC_NUM_CHANNELS];
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

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


void retrieve_gyro_data() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  for (int i = 0; i < GYRO_NUM_CHANNELS; i++) {
    gyro_values[i] = Wire.read() << 8 | Wire.read();
  }
}

void calc_ch1() {
  calc_input(RC_CH1, RC_CH1_INPUT);
}
void calc_ch2() {
  calc_input(RC_CH2, RC_CH2_INPUT);
}
void calc_ch3() {
  calc_input(RC_CH3, RC_CH3_INPUT);
}

double ewma(double new_val, double old_val, double lambda) {
  return new_val * lambda + (1 - lambda) * old_val;
}


void map_rc_values_to_servo() {
  for (int i = 0; i < RC_NUM_CHANNELS; i++) {
    if (rc_values[i] < ch_lims[i][0]) {
      rx_servo_vals[i] = ch_lims[i][2];
    }
    else if (rc_values[i] > ch_lims[i][1]) {
      rx_servo_vals[i] = ch_lims[i][3];
    }
    else {
      rx_servo_vals[i] = map(rc_values[i], ch_lims[i][0], ch_lims[i][1], ch_lims[i][2], ch_lims[i][3]);
    }
  }
}

void output_rx_values() {
  for (int i = 0; i < RC_NUM_CHANNELS; i++) {
    Serial.print("CH"); Serial.print(i); Serial.print(": "); Serial.print(rc_values[i]); Serial.print("\t");
    Serial.print("SERVO"); Serial.print(i); Serial.print(": "); Serial.print(rx_servo_vals[i]); Serial.print("\t");
  }
  Serial.println("");
}

void output_gyro_values() {
  for (int i = 0; i < GYRO_NUM_CHANNELS; i++) {
    Serial.print("Gyro Value "); Serial.print(i); Serial.print(": "); Serial.print(gyro_values[i]);
  }
  Serial.println("");
}


void output_servo_values() {
  Serial.print("Steering Value: "); Serial.print(steering_value); Serial.print("\t");
  Serial.println("Throttle Value:  "); Serial.print(throttle_value);  Serial.print("\t");
  Serial.println("Steering Weight:  "); Serial.print(steering_weight);  Serial.print("\t");
  Serial.println("YAW Corection:  "); Serial.println(yaw_correction);
}


void initialise_servos() {
  steering_value = int((RC_STR_LIM_LEFT + RC_STR_LIM_RIGHT) * 0.5);
  throttle_value = 90;

  servo_throttle.attach(8);
  servo_steering.attach(9);

  servo_steering.write(steering_value);
  servo_throttle.write(throttle_value);
  
}


void setup() {
  initialise_servos();
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(SERIAL_PORT_SPEED);

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);



}


void loop() {
  rc_read_values();
  retrieve_gyro_data();
  map_rc_values_to_servo();

  //  output_rx_values();
  //  output_gyro_values();

  yaw_correction = ewma(gyro_values[GYRO_GyZ], yaw_correction, YAW_LAMBDA);
  yaw_correction = map(yaw_correction, -40000, 30000, -90, 90);

  steering_value = ewma(rx_servo_vals[RC_STEERING], steering_value, RC_STEERING_LAMBDA);
  steering_weight = map(abs(90. - double(steering_value)), 0., 90., 1. , 0.);

  steering_value = yaw_correction + steering_value;
  throttle_value = ewma(rx_servo_vals[RC_THROTTLE], throttle_value, RC_THROTTLE_LAMBDA);

  servo_steering.write(steering_value);
  servo_throttle.write(throttle_value);

  if (loop_count % 500 == 0) output_servo_values();

  loop_count++;

}
