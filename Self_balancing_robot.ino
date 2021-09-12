#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

//BEGINNING OF FILTER DECLARATION//
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine
//END OF FILTER DECLARATION//

///////////////////////BEGINING OF OTHER DECLARATION//

////////////////VARIABLE DEFINATION///////////////
int ENA = A0;
int IN1 = 9;
int IN2 = 10;
int ENB = A1;
int IN3 = 7;
int IN4 = 8;
int mspeed = 10;
float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
////////////////////////PID CONSTANT/////////////////////
float kp = 5;
float ki = 0;//1.5;
float kd = 1;//1.0;
float desired_angle = 0;//////////////TARGET ANGLE/////////////

float elapsedTime, time, timePrev;

////////////////END OF OTHER DECLARATION//

void setup() {
  Serial.begin(115200);
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    //Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
  time = millis();
}

void loop() {
  timePrev = time;
  time =  millis();
  elapsedTime = (time - timePrev) / 1000;
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  Serial.print("Roll: ");
  Serial.print(roll); Serial.print(" gyroRoll: ");
  Serial.print(gyroXangle); Serial.print(" compRoll: ");
  Serial.print(compAngleX); Serial.print(" kalRoll: ");
  Serial.print(kalAngleX);
  
  Serial.print(" Pitch: ");
  Serial.print(pitch); Serial.print(" gyroPitch: ");
  Serial.print(gyroYangle); Serial.print(" compPitch: ");
  Serial.print(compAngleY); Serial.print(" kalPitch: ");
  Serial.println(kalAngleY);
  delay(1000);

  ///////////////////////PID SETTING IN VOID LOOOOOOP/////////////////////
  ////WE CAN USE EITHER OF PITCH, COMPANGLEY, KALANGLEY////////////
  //error = pitch - desired_angle; /////////////////ERROR CALCULATION USING PITCH////////////////////
  error = compAngleX - desired_angle; /////////////////ERROR CALCULATION USING COMP////////////////////
  //error = kalAngleY - desired_angle; /////////////////ERROR CALCULATION USING KALMAN////////////////////
  //Serial.print(" Error: ");
  //Serial.print(error);
  ///////////////////////PROPORTIONAL ERROR//////////////
  pid_p = kp * error;
  ////Serial.print(" pid_p: ");
  ////Serial.print(pid_p);
  ///////////////////////INTERGRAL ERROR/////////////////
  pid_i = pid_i + (ki * error * elapsedTime);
  ////Serial.print(" pid_i: ");
  ////Serial.print(pid_i);
  ///////////////////////DIFFERENTIAL ERROR//////////////
  pid_d = kd * ((error - previous_error) / elapsedTime);
  ////Serial.print(" pid_d: ");
  ////Serial.print(pid_d);
  ///////////////////////TOTAL PID VALUE/////////////////
  PID = pid_p + pid_d + pid_i;
  ///////////////////////UPDATING THE ERROR VALUE////////
  previous_error = error;
  //Serial.print(" PID: ");
  //Serial.println(PID);                     //////////UNCOMMENT FOR DDEBUGGING//////////////
  //delay(60);                               //////////UNCOMMENT FOR DDEBUGGING//////////////

  /////////////////CONVERTING PID VALUES TO ABSOLUTE VALUES//////////////////////////////////
  mspeed = abs(PID);
//  Serial.print(" MotorSpeed: ");
//  Serial.println(mspeed);                  //////////UNCOMMENT FOR DDEBUGGING//////////////
  ///////////////SELF EXPLANATORY///////////////
  if (compAngleX < 0)
  {
    clockw();
    anti();
  }
  if (compAngleX > 0)
  {
    clockw();
  }
  if (compAngleX > 45)
    halt();
  if (compAngleX < -45)
    halt();
}

//////////////MOVEMENT FUNCTION///////////////////
void clockw()
{
  analogWrite(ENA, mspeed);
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  analogWrite(ENB, mspeed);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
}
void anti()
{
  analogWrite(ENA, mspeed);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  analogWrite(ENB, mspeed);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
}
void halt()
{

  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);

}
