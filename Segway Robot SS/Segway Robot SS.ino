
/******************** Initialize Bricktronics Shield ********************/
// Include the Bricktronics libraries
#include <BricktronicsMegashield.h>
#include <BricktronicsMotor.h>
#include <BricktronicsUltrasonic.h>

// Select the sensor port for the ULTRASONIC sensor (SENSOR_1 through SENSOR_4) below.
// Use the jumpers to connect pins 1-2 and 4-5 for the ultrasonic sensor.
BricktronicsUltrasonic u(BricktronicsMegashield::SENSOR_4);

// Select the desired motor port (MOTOR_1 through MOTOR_6) in the constructor below.
BricktronicsMotor m1(BricktronicsMegashield::MOTOR_4);
BricktronicsMotor m2(BricktronicsMegashield::MOTOR_2);


/******************** Initialize MPU9250 ********************/
/*
  Hardware setup:
  MPU9250 Breakout --------- Arduino
  VDD ---------------------- 3.3V
  VDDI --------------------- 3.3V
  SDA ----------------------- A4
  SCL ----------------------- A5
  GND ---------------------- GND
*/

#include "quaternionFilters.h"
#include "MPU9250.h"

#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

MPU9250 myIMU;

/******************** Define Global Variables ********************/
int angle; // motor angle

// define robot charateristics
float T = 0.2;    // motor time constant (sec)
int Kdc = 11;     // motor CC velocity gain (deg/sec)/(lego count)
float m = 0.727;    // mass of the whole body + motors (kg)
float R = 0.0425;  // radius of the wheels (m)
float L = 0.15;    // height of the c.g. above the wheel axle (m)
float J = (m / 12) * (0.01 + 0.016); // moment of inertia of the body (kg*m^2) Note: original equation: (0.1^2+0.04^2)
float Jeff = (J + m*R * L + m*L*L);   // effective total moment of inertia (kg*m^2)
float g = 9.8;    // acceleration due to gravity (m/s^2)

// define state space variables for x
float imu_prev = 0;   // pitch value from previous time period
float imu_curr = 0;   // pitch value from current time period
float imu_change = 0; // rate of change in pitch value

int motor_prev = 0;   // motor angle from previous time period
int motor_curr = 0;   // motor angle from current time period
float motor_change = 0; // rate of change in motor angle

float delta_t = 0.01;  // 5 millisecond sampling rate
int spd = 0;          // output to motor speed control (based on u = -Kx)

// define Matrix K as individual variables
// old float k1 = -67.3508;  float k2 = -7.0757; float k3 = -.3456;  float k4 = -0.2935;
// low weight float k1 = -15.6480;  float k2 = -1.6302; float k3 = -.3401;  float k4 = -0.2903;
float k1 = -54.4569;  float k2 = -7.7038; float k3 = 0;  float k4 = 0;
float updatedRoll;

/******************** Define Functions ********************/
int getMotorAngle()
{
  int angle1 = m1.getAngle(); // Returns the current angle of motor 1, in the range of (0 - 359) degrees.
  int angle2 = m2.getAngle(); // Returns the current angle of motor 2, in the range of (0 - 359) degrees.
  angle = (angle1 + angle2) / 2; // calculated the average of the motor angles
  return angle;
}

int calculateMotorSpeed(int k1, int k2, int k3, int k4, int pitch_curr,
                        int pitch_change, int motor_curr, int motor_change)
{
  int temp_spd =  (k1 * pitch_curr + k2 * pitch_change + k3 * motor_curr + k4 * motor_change);
  //temp_spd = map(temp_spd, -45, 25, -254, 254);
  if (temp_spd > 254) temp_spd = 254;
  if (temp_spd < -254) temp_spd = -254;
  return temp_spd;
}

float getMPUData()
{
  /******************** Obtain MPU9250 Data ********************/
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.magCalibration[2] -
               myIMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 1)
    {
      if (SerialDebug)
      {
        /* COMMENTED OUT FOR DEBUGGING
          // Print acceleration values in milligs!
          Serial.print("X-acceleration: "); Serial.print(1000*myIMU.ax);
          Serial.print(" mg ");
          Serial.print("Y-acceleration: "); Serial.print(1000*myIMU.ay);
          Serial.print(" mg ");
          Serial.print("Z-acceleration: "); Serial.print(1000*myIMU.az);
          Serial.println(" mg ");

          // Print gyro values in degree/sec
          Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
          Serial.print(" degrees/sec ");
          Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
          Serial.print(" degrees/sec ");
          Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
          Serial.println(" degrees/sec");

          // Print mag values in degree/sec
          Serial.print("X-mag field: "); Serial.print(myIMU.mx);
          Serial.print(" mG ");
          Serial.print("Y-mag field: "); Serial.print(myIMU.my);
          Serial.print(" mG ");
          Serial.print("Z-mag field: "); Serial.print(myIMU.mz);
          Serial.println(" mG");

          myIMU.tempCount = myIMU.readTempData();  // Read the adc values
          // Temperature in degrees Centigrade
          myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
          // Print temperature in degrees Centigrade
          Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
          Serial.println(" degrees C");
        */
      }

      myIMU.count = millis();
      digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    } // if (myIMU.delt_t > 1)
  } // if (!AHRS)

  else // if (AHRS)
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;

    // update serial monitor once per half-second independent of read rate
    if (myIMU.delt_t > 1)
    {
      if (SerialDebug)
      {
        /* COMMENTED OUT FOR DEBUGGING
          Serial.print("ax = "); Serial.print((int)1000*myIMU.ax);
          Serial.print(" ay = "); Serial.print((int)1000*myIMU.ay);
          Serial.print(" az = "); Serial.print((int)1000*myIMU.az);
          Serial.println(" mg");

          Serial.print("gx = "); Serial.print( myIMU.gx, 2);
          Serial.print(" gy = "); Serial.print( myIMU.gy, 2);
          Serial.print(" gz = "); Serial.print( myIMU.gz, 2);
          Serial.println(" deg/s");

          Serial.print("mx = "); Serial.print( (int)myIMU.mx );
          Serial.print(" my = "); Serial.print( (int)myIMU.my );
          Serial.print(" mz = "); Serial.print( (int)myIMU.mz );
          Serial.println(" mG");

          Serial.print("q0 = "); Serial.print(*getQ());
          Serial.print(" qx = "); Serial.print(*(getQ() + 1));
          Serial.print(" qy = "); Serial.print(*(getQ() + 2));
          Serial.print(" qz = "); Serial.println(*(getQ() + 3));
        */
      }

      // Define output variables from updated quaternion---these are Tait-Bryan
      // angles, commonly used in aircraft orientation. In this coordinate system,
      // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
      // x-axis and Earth magnetic North (or true North if corrected for local
      // declination, looking down on the sensor positive yaw is counterclockwise.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the
      // Earth is positive, up toward the sky is negative. Roll is angle between
      // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
      // arise from the definition of the homogeneous rotation matrix constructed
      // from quaternions. Tait-Bryan angles as well as Euler angles are
      // non-commutative; that is, the get the correct orientation the rotations
      // must be applied in the correct order which for this configuration is yaw,
      // pitch, and then roll.
      // For more see
      // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
      // which has additional links.
      myIMU.yaw   = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *
                                  *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1) * *(getQ() + 1)
                          - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3) * *(getQ() + 3));
      myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
                                  *(getQ() + 2)));

      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2) *
                                  *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1) * *(getQ() + 1)
                          - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3) * *(getQ() + 3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;
      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw   -= 8.5;
      myIMU.roll  *= RAD_TO_DEG;

      if (SerialDebug)
      {
        /* COMMENTED OUT FOR DEBUGGING
          Serial.print("Yaw, Pitch, Roll: ");
          Serial.print(myIMU.yaw, 2);
          Serial.print(", ");
          Serial.print(myIMU.pitch, 2);
          Serial.print(", ");
          Serial.println(myIMU.roll, 2);

          Serial.print("rate = ");
          Serial.print((float)myIMU.sumCount/myIMU.sum, 2);
          Serial.println(" Hz");
        */
      }

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 1)
  } // if (AHRS)

  return myIMU.pitch;
}

void setup()
{
  /******************** Setup Serial Connection ********************/
  Serial.begin(115200);
  Serial.println("Serial port open.");

  /******************** Setup Bricktronics Shield ********************/
  // Initialize the ultrasonic sensor connections
  u.begin();

  // Initialize the motor connections
  m1.begin();
  m2.begin();

  /******************** Setup MPU9250 ********************/
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  Serial.println("Connection check...");
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[0], 1); Serial.println("// of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[1], 1); Serial.println("// of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[2], 1); Serial.println("// of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[3], 1); Serial.println("// of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[4], 1); Serial.println("// of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[5], 1); Serial.println("// of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");
    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[2], 2);
    }

  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while (1) ; // Loop forever if communication doesn't happen
  }

  Serial.println("Done with setup....");
}

/******************** Main Program ********************/
void loop()
{
  getMPUData();
  imu_prev = imu_curr;    // save previous pitch
  imu_curr = myIMU.ay;   // update current pitch
  imu_change = (imu_curr - imu_prev) / delta_t; // calculate rate of change in pitch

  motor_prev = motor_curr;            // save previous motor angle
  motor_curr = getMotorAngle();  // update current motor angle
  motor_change = ((motor_curr - motor_prev) / delta_t); // calculate rate of change in motor angle

  if (myIMU.roll > 0)
  {
    updatedRoll = 180 - myIMU.roll;
  }
  else
  {
    updatedRoll = -180 - myIMU.roll;
  }
  spd = -calculateMotorSpeed(k1, k2, k3, k4, updatedRoll, myIMU.gx, motor_curr, motor_change);

  // Display values for debugging purposes
  //  Serial.print("updatedRoll: "); Serial.print(updatedRoll);
  //  Serial.print( "   imu_curr: "); Serial.print(imu_curr);
  //  Serial.print("   imu Change: "); Serial.print(imu_change);
  //  Serial.print("   myIMU.gx: "); Serial.print(myIMU.gx);
  //  Serial.print("   timeLapse: "); Serial.print(timeLapse);
  //  Serial.print("   timeLapse/1000: "); Serial.print((float)timeLapse/1000.0);
  //  Serial.print("   Motor_ prev: "); Serial.print(motor_prev);
  //  Serial.print("   Motor_curr: "); Serial.print(motor_curr);
  //  Serial.print("   Motor Change: "); Serial.print(motor_change);
  //  Serial.print("   Motor Speed: "); Serial.println(spd);

  m1.setFixedDrive(spd);
  m2.setFixedDrive(spd);
  m1.update();
  m2.update();
}

