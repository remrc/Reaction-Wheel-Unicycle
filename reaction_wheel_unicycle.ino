#include <SimpleFOC.h>  // ver. 1.61 !!!

BLDCMotor motorX = BLDCMotor(3, 5, 6, 7);  
BLDCMotor motorY = BLDCMotor(9, 10, 11, 7); 
MagneticSensorAnalog sensorX = MagneticSensorAnalog(A1, 14, 1020);
MagneticSensorAnalog sensorY = MagneticSensorAnalog(A0, 14, 1020);

#define MPU6050 0x68              // Device address
#define ACCEL_CONFIG 0x1C         // Accelerometer configuration address
#define GYRO_CONFIG 0x1B          // Gyro configuration address

//Registers: Accelerometer, Temp, Gyroscope
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

//Sensor output scaling
#define accSens 0             // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1            // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s

int16_t  AcX, AcY, AcZ, GyY, GyX, GyZ; 
float gyroX, gyroY;

//IMU offset values
int16_t  AcX_offset = -750;
int16_t  AcY_offset = 280;
int16_t  AcZ_offset = 100;
int16_t  GyY_offset = 0, GyZ_offset = 0 ;
int32_t  GyY_offset_sum = 0, GyZ_offset_sum = 0;

float robot_angleX, robot_angleY, gyroXfilt, gyroYfilt;
float Acc_angleX,  Acc_angleY; 
float angle_offsetX = -5.60, angle_offsetY = 2.03;

float K1X = 1.67;
float K2X = 0.28;
float K3X = 0.190;
float K1Y = 0.74;
float K2Y = 0.09;
float K3Y = 0.200;

int loop_time = 4;

boolean vertical = false;
boolean test = false;  // if true - motors and sensors test. Type in serial monitor t+ to enable and t- to disable

#define Gyro_amount 0.996   // percent of gyro in complementary filter T/(T+del_t) del_t: sampling rate, T acc. timeconstant ~1s
float alphaX = 0.25; // low pass
float alphaY = 0.75; // low pass

void setup() {

  Serial.begin(115200);
  delay(1000);

  angle_setup();

  sensorX.init();                                                // initialise magnetic sensor hardware
  motorX.linkSensor(&sensorX);                                   // link the motor to the sensor
  motorX.voltage_power_supply = 12;
  motorX.controller = ControlType::voltage;                      // set control loop type to be used
  motorX.voltage_sensor_align = 4;                               // aligning voltage 
  motorX.foc_modulation = FOCModulationType::SpaceVectorPWM;     // choose FOC modulation (optional)
  motorX.init();                                                 // initialize motor
  motorX.initFOC();                                              // align encoder and start FOC

  sensorY.init();                                                // initialise magnetic sensor hardware
  motorY.linkSensor(&sensorY);                                   // link the motor to the sensor
  motorY.voltage_power_supply = 12;
  motorY.controller = ControlType::voltage;                      // set control loop type to be used
  motorY.voltage_sensor_align = 4;                               // aligning voltage 
  motorY.foc_modulation = FOCModulationType::SpaceVectorPWM;   // choose FOC modulation (optional)
  motorY.init();                                                 // initialize motor
  motorY.initFOC();                                              // align encoder and start FOC
}

long loop_count = 0;
float m_speedX = 0, m_speedY = 0;
float move_Speed;

void loop() {
  // ~1ms 
  motorX.loopFOC();
  motorY.loopFOC();

  Tuning();
  angle_calc();
  
  if (abs(robot_angleX - angle_offsetX) > 8 || abs(robot_angleY - angle_offsetY) > 8) vertical = false;
  if (abs(robot_angleX - angle_offsetX) < 0.3 && abs(robot_angleY - angle_offsetY) < 0.3 && !vertical) vertical = true;

  gyroX = GyZ / 131.0; // Convert to deg/s
  gyroY = GyY / 131.0; // Convert to deg/s

  gyroXfilt = alphaX * gyroX + (1 - alphaX) * gyroXfilt;
  gyroYfilt = alphaY * gyroY + (1 - alphaY) * gyroYfilt;

  if (test) {
    motorX.move(4 * (motorY.shaft_angle - motorX.shaft_angle));
    motorY.move(4 * (motorX.shaft_angle - motorY.shaft_angle));
  } else if (loop_count++ > loop_time) {  // control loop 
    float target_voltageX = 0, target_voltageY = 0;
    if (vertical) {      
      target_voltageX = XcontrolleR(robot_angleX - angle_offsetX, gyroXfilt, sensorX.getVelocity() + m_speedX);
      m_speedX += sensorX.getVelocity() / 50; 
      target_voltageY = YcontrolleR(robot_angleY - angle_offsetY, gyroYfilt, sensorY.getVelocity() + m_speedY);
      m_speedY += sensorY.getVelocity() / 40 - move_Speed; 

      motorX.move(target_voltageX);
      motorY.move(-target_voltageY);
    } else {
      motorX.move(target_voltageX);
      motorY.move(target_voltageY);
      m_speedX = 0;
      m_speedY = 0;
    }
    loop_count = 0;
  }   
}

int sign(int x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  if (x = 0) return 0;
}

float XcontrolleR(float p_angle, float p_vel, float m_vel) {
  float u =  K1X * p_angle + K2X * p_vel + K3X * m_vel;
  if (abs(u) > motorX.voltage_power_supply * 0.8) 
    u = sign(u) * motorX.voltage_power_supply * 0.8;
  return u;
}

float YcontrolleR(float p_angle, float p_vel, float m_vel) {
  float u =  K1Y * p_angle + K2Y * p_vel + K3Y * -m_vel;
  if (abs(u) > motorY.voltage_power_supply * 0.8) 
    u = sign(u) * motorY.voltage_power_supply * 0.8;
  return u;
}

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

//setup MPU6050
void angle_setup() {
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);

  Serial.println("Calibrating gyroscope...");
   
  // calc gyro offset by averaging 1024 values
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyZ_offset_sum += GyZ;
    GyY_offset_sum += GyY;
    delay(5);
  }
  GyZ_offset = GyZ_offset_sum >> 10;
  Serial.print("GyZ offset value = "); Serial.println(GyZ_offset);
  GyY_offset = GyY_offset_sum >> 10;
  Serial.print("GyY offset value = "); Serial.println(GyY_offset);
}

void angle_calc() {
  
  // read raw accel/gyro measurements from device
  Wire.beginTransmission(MPU6050);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);  
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Wire.beginTransmission(MPU6050);
  Wire.write(ACCEL_XOUT_H);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true); 
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  // add mpu6050 offset values
  AcX += AcX_offset;
  AcY += AcY_offset;  
  AcZ += AcZ_offset;
  GyZ -= GyZ_offset;
  GyY -= GyY_offset;

  robot_angleX += GyZ * 6.07968E-5; 
  Acc_angleX = -atan2(AcY, AcX) * 57.2958;       // angle from acc. values       * 57.2958 (deg/rad)
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);

  robot_angleY += GyY * 6.07968E-5;  
  Acc_angleY = atan2(AcZ, AcX) * 57.2958;        // angle from acc. values       * 57.2958 (deg/rad)
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);

  //Serial.print("AngleX: "); Serial.print(robot_angleX); Serial.print(" AngleY: "); Serial.println(robot_angleY);

}


