/* Code for Gimbal Self-stabilizer
 * 2-axis Servo (pitch & roll)
 * using MPU6050 and PID control */
  #include <Wire.h>              //Library buat komunikasi I2C
#include <Servo.h>

#define mpuAddr 0x68           //I2C address 1101000 (AD0 LOW)

#define MPU6050_ADDRESS             0x68
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
//#define MPU6050_RA_ACCEL_XOUT_L     0x3C
//#define MPU6050_RA_ACCEL_YOUT_H     0x3D
//#define MPU6050_RA_ACCEL_YOUT_L     0x3E
//#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
//#define MPU6050_RA_ACCEL_ZOUT_L     0x40
//#define MPU6050_RA_TEMP_OUT_H       0x41
//#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
//#define MPU6050_RA_GYRO_XOUT_L      0x44
//#define MPU6050_RA_GYRO_YOUT_H      0x45
//#define MPU6050_RA_GYRO_YOUT_L      0x46
//#define MPU6050_RA_GYRO_ZOUT_H      0x47
//#define MPU6050_RA_GYRO_ZOUT_L      0x48
#define MPU6050_RA_PWR_MGMT_1       0x6B

Servo rollServo;
Servo pitchServo;

float AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ;                                //buat simpen raw value accel & gyro
float AccelAngleX, AccelAngleY, AccelAngleZ, GyroAngleX, GyroAngleY, GyroAngleZ;
float prevTime, currTime, elapsedTime;                                            //rekam waktu buat dapet derajat dari data gyro

float roll, pitch;
float desired_roll_angle = 0, desired_pitch_angle = 0;                            //deg roll & pitch yang diinginkan
float rollError, pitchError, prevRollError, prevPitchError;
float roll_kp, roll_ki, roll_kd, pitch_kp, pitch_ki, pitch_kd;
float nilaiPRoll, nilaiPPitch, nilaiIRoll, nilaiIPitch, nilaiDRoll, nilaiDPitch;
float rollPID, pitchPID;
int pitchPWM, rollPWM;

void setup() {
  // put your setup code here, to run once:
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
  
  IMU_ERROR_VALUES(); //kalkulasi error

  // Setting Servo
  rollServo.attach(10);
  pitchServo.attach(9);
  while(!Serial);
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_RA_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 6, true);                 //ambil data accelerometer
  AccelX = (Wire.read() << 8 | Wire.read()) / 16384.0;        //merubah raw data ke +- 2 g
  AccelY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccelZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

  //rubah menjadi Acc angle menggunakan rumus Euler
  AccelAngleX = (atan(AccelY / sqrt(pow(AccelX, 2) + pow(AccelZ, 2))) * 180 / PI) + 0.02;       //hasilnya dikurang dengan error   
  AccelAngleY = (atan(-1 * AccelX / sqrt(pow(AccelY, 2) + pow(AccelZ, 2))) * 180 / PI) + 0.02;

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
//  GyroAngleX = GyroAngleX + GyroX * elapsedTime;       // deg/s * s = deg
//  GyroAngleY = GyroAngleY + GyroY * elapsedTime;

  GyroAngleX = GyroX * elapsedTime;       // deg/s * s = deg
  GyroAngleY = GyroY * elapsedTime;

  //complementary filter
  roll = 0.96 * GyroAngleX + 0.04 * AccelAngleX;
  pitch = 0.96 * GyroAngleY + 0.04 * AccelAngleY;


  //PID control
  //menghitung selisih antara angle roll & pitch sekarang
  //dengan nilai deg yang diinginkan
  roll_kp = 1;
  roll_ki = 0;
  roll_kd = 0;
  pitch_kp = 1;
  pitch_ki = 0;
  pitch_kd = 0;
  
  rollError = roll - desired_roll_angle;
  pitchError = pitch - desired_pitch_angle;

  //nilai proportional gain diperoleh dengan mengalikan kp
  //dengan error
  nilaiPRoll = roll_kp*rollError;
  nilaiPPitch = pitch_kp*pitchError;  


  //integral gain diperoleh dengan nilai integral prev ditambah dengan
  //hasil dari error roll dan konstanta integral
  if(-3 < rollError <3){
    nilaiIRoll = nilaiIRoll+(roll_ki*rollError);  
    }
  if(-3 < rollError < 3){
    nilaiIPitch = nilaiIPitch+(pitch_ki*pitchError);  
    }

  /*The last part is the derivate. The derivate acts upon the speed of the error.
    As we know the speed is the amount of error that produced in a certain amount of
    time divided by that time. For taht we will use a variable called previous_error.
    We substract that value from the actual error and divide all by the elapsed time. 
    Finnaly we multiply the result by the derivate constant*/
    nilaiDRoll = roll_kd*((rollError - prevRollError)/elapsedTime);
    nilaiDPitch = pitch_kd*((pitchError - prevPitchError)/elapsedTime);
    /*The final PID values is the sum of each of this 3 parts*/
    rollPID = nilaiPRoll + nilaiIRoll + nilaiDRoll ;
    pitchPID = nilaiPPitch + nilaiIPitch + nilaiDPitch ;
    /*We know taht the min value of PWM signal is -90 (usingservo.write) and the max is 90. So that
    tells us that the PID value can/s oscilate more than -90 and 90 so we constrain those values below*/
    if(rollPID < -90){
      rollPID = -90;
      }
    if(rollPID > 90){
      rollPID = 90; 
      }
    if(pitchPID < -90){
      pitchPID = -90;
      }
    if(pitchPID > 90){
      pitchPID = 90;
      }
    
    prevRollError = rollError;     //Remember to store the previous error.
    prevPitchError = pitchError;   //Remember to store the previous error.

    pitchPWM = 90 + pitchPID;           //Angle for each motor is 90 plus/minus the PID value
    rollPWM = 90 - rollPID;

    pitchPWM = map(pitchPWM, -90, 90, 0, 180);
    rollPWM = map(rollPWM, -90, 90, 0, 180);
    
    pitchServo.write(pitchPWM);               //Finally we write the angle to the servos
    rollServo.write(rollPWM);

//  Serial.print("P: ");
//  Serial.print(roll);
//  Serial.print("/ R: ");
//  Serial.print(pitch);
//  Serial.print("/");
//  Serial.println("");
  delay(20);
}

void IMU_ERROR_VALUES(){
  int c = 0, numOfReading = 200;
  while(c < numOfReading){
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_RA_ACCEL_XOUT_H);                 //ke address accel output
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS, 6, true);  
    AccelX = (Wire.read() << 8 | Wire.read()) / 16384.0;  //merubah ke unit +-2 g
    AccelY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccelZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
  
    AccelAngleX = (atan(AccelY / sqrt(pow(AccelX, 2) + pow(AccelZ, 2))) * 180 / PI);
    AccelAngleY = (atan(-1 * AccelX / sqrt(pow(AccelY, 2) + pow(AccelZ, 2))) * 180 / PI);
    c++;
  }
  AccelAngleX = AccelAngleX / numOfReading ;            //error value
  AccelAngleY = AccelAngleY / numOfReading;

  c = 0;
  while(c < numOfReading){
    prevTime = currTime;                                //kalo udh ngeloop set waktu sekarang sebagai waktu sebelum 
    currTime = millis();                                // Membaca waktu sekarang
    elapsedTime = (currTime - prevTime) / 1000;         // bagi 1000 untuk dapet detik
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_RA_GYRO_XOUT_H);                 //ke address gyro output
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_RA_GYRO_XOUT_H, 6, true);
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;   //merubah ke unit 250deg/s
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
    
    GyroAngleX = GyroX * elapsedTime;      // deg/s * s = deg
    GyroAngleY = GyroY * elapsedTime;
    c++;
  }
  GyroAngleX = GyroAngleX / numOfReading; //error value
  GyroAngleY = GyroAngleY / numOfReading;
  GyroAngleZ = GyroAngleZ / numOfReading;

  //Print out error values
  Serial.print("AccelAngleX Error: ");
  Serial.println(AccelAngleX);
  Serial.print("AccelAngleY Error: ");
  Serial.println(AccelAngleY);
  Serial.print("GyroAngleX Error: ");
  Serial.println(GyroAngleX);
  Serial.print("GyroAngleY Error: ");
  Serial.println(GyroAngleY);
  Serial.print("GyroAngleZ Error: ");
  Serial.println(GyroAngleZ);
  delay(250);
}
