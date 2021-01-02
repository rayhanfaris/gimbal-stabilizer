/* Gimbal self-stabilizer 2-axis (pitch, roll)
 * using servo and MPU6050
 * with PID control, kalman filter
 */

#include <PID_v1.h>
#include <Wire.h>
#include <Servo.h>

#define MPU6050_ADDRESS             0x68
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_PWR_MGMT_1       0x6B

Servo rollServo;
Servo pitchServo;

// === PID ===
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

void setup() {
  // SENSOR SET UP
  Serial.begin(19200);
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_RA_PWR_MGMT_1);         //akses PWR_MGMT_1 untuk exit sleep mode
  Wire.write(0x00);                          
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_RA_ACCEL_XOUT_H);
  Wire.write(0x00);                         //Setting accelerometer scale +- 2g
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_RA_GYRO_XOUT_H);
  Wire.write(0x00);                         //Setting gyroscoe scale +- 200 deg/s
  Wire.endTransmission(true);
  
  rollServo.attach(10);
  pitchServo.attach(9);

  //Input = analogRead(PIN_INPUT);
  Setpoint = 0;  //100

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  while(!Serial);
}

void loop() {
  // DATA ACQUISITION
  Wire.beginTransmission(MPU6050_ADDRESS);

  // ACCELEROMETER
  Wire.write(MPU6050_RA_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 6, true);
  AccelX = (Wire.read() << 8 | Wire.read()) / 16384.0;        
  AccelY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccelZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

  //rubah menjadi Acc angle menggunakan rumus Euler
  AccelAngleX = (atan(AccelY / sqrt(pow(AccelX, 2) + pow(AccelZ, 2))) * 180 / PI) + 0.02;       //hasilnya dikurang dengan error   
  AccelAngleY = (atan(-1 * AccelX / sqrt(pow(AccelY, 2) + pow(AccelZ, 2))) * 180 / PI) + 0.02;


  // GYROSCOPE
  prevTime = currTime;                                //kalo udh ngeloop set waktu sekarang sebagai waktu sebelum 
  currTime = millis();                                // Membaca waktu sekarang
  elapsedTime = (currTime - prevTime) / 1000;         // bagi 1000 untuk dapet detik
  
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_RA_GYRO_XOUT_H);                 //ke address gyro output
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU6050_RA_GYRO_XOUT_H, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;   //merubah raw data ke +-250 deg/s
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  //rubah raw value menjadi deg dengan mengalikan dengan s
  GyroAngleX = GyroAngleX + GyroX * elapsedTime;       // deg/s * s = deg
  GyroAngleY = GyroAngleY + GyroY * elapsedTime;
  

  // === PID CONTROL === 
  Input = analogRead(PIN_INPUT);

  double gap = abs(Setpoint-Input); //distance away from setpoint
  if (gap < 10) {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }

  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
}
