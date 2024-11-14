/*
Madgwick Fusion Library:
	The MIT License (MIT)
	Copyright (c) 2021 x-io Technologies
BolderFlight invensense-imu library
	The MIT License (MIT)
	Copyright (c) 2022 Bolder Flight Systems Inc

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "mpu9250.h"
#include "Fusion.h"
#include <stdbool.h>
#include <stdio.h>

#include <Wire.h>
#include "Streaming.h"
#include <string>

#include "constants.h" 

/* Mpu9250 object */
bfs::Mpu9250 imu;

void gyroCalibration();

void telemetryPortOut();
void print_float_array(float *arr, int size);

bool dataRdy;


//#define use_freeimu
//#define use_magneto
#define use_minmax
//#define use_motioncal


// scale factors
float accelScale, gyroScale;
float magScale[3];

FusionOffset offset;
FusionAhrs ahrs;
FusionQuaternion q;
FusionEuler euler;
FusionAhrsFlags flags;

// Define calibration
const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
//gyroscop offsets and sensitity configured in gyroCalibration()
FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
//Accelerometer calibration configured in getCalIMU();
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};


/*
 * accelerometer corrections for freeimu and motioncal in getCalIMU()
 * freeimu uses Magneto 1.2 results since for the MPU-9250 I am currently
 * using a nice sphere isn't generated but stays elongated
 */
#if defined(use_magneto)
  const FusionMatrix softIronMatrix = {1.016476, -0.011290, 0.00759, -0.011290, 1.004871, 0.062744, 0.00759, 0.062744, 1.032701};
  const FusionVector hardIronOffset = {13.090573, 30.546150, -32.757856};
#elif defined(use_motioncal)
  const FusionMatrix softIronMatrix = {0.9895,-0.0176,-0.0027,-0.0176,0.9847,0.0544,-0.0027,0.0544,1.0296};
  const FusionVector hardIronOffset = {13.278,30.538,-33.388};
#endif
  
  //new data available
volatile int newIMUData;
uint32_t lastUpdate, now1;


void setup() {
  /* Serial to display data */
  while(!Serial && millis() < 5000) {}
  Serial.begin(115200);
  
  // If Teensy 4.x fails print Crashreport to serial monitor
  if (CrashReport) {
      Serial.print(CrashReport);
      Serial.println("Press any key to continue");
      while (Serial.read() != -1) {
      }
      while (Serial.read() == -1) {
      }
      while (Serial.read() != -1) {
      }
  }

  /* Start the I2C bus */
  Wire2.begin();
  Wire2.setClock(400000);
  /* I2C bus,  0x68 address */
  imu.Config(&Wire2, bfs::Mpu9250::I2C_ADDR_PRIM);
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }

  cout.println("IMU Connected!");

  /* Set the sample rate divider */
  // rate = 1000 / (srd + 1)
  // = 1000/20 = 50 hz
  // = 100 hz
  if (!imu.ConfigSrd(9)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }
  SAMPLE_RATE = 0.01f;	//100hz

  //Get MPU sensitivity values
  imu.getScales(&accelScale, &gyroScale, magScale);
  gyroCalibration();

  FusionOffsetInitialise(&offset, SAMPLE_RATE);
  FusionAhrsInitialise(&ahrs);
    
  // Set AHRS algorithm settings
  const FusionAhrsSettings settings = {
          .convention = FusionConventionNed,
          .gain = 1.5f, //1.5f,
          .gyroscopeRange = 2000.0f, // replace this with actual gyroscope range in degrees/s
          .accelerationRejection = 10.0f,
          .magneticRejection = 10.0f, //0.0f,
          .recoveryTriggerPeriod  = (uint8_t)(5.0f / SAMPLE_RATE ), // 5 seconds
  };

  FusionAhrsSetSettings(&ahrs, &settings);


  cout.println("Ready for commands....");
}

void loop() {
    if ( cout.available() ) {
      char rr;
      rr = cout.read();
      switch (rr) {
        case 'g':
          rr = '0';
          gyroCalibration();
          break;
        case 'r':
        {
          rr ='0';
          if(raw_values_on == 1) {
            raw_values_on = 0;
          } else if(raw_values_on == 0) {
            raw_values_on = 1;
          }
        }
          break;
        case 's':
        {
          rr ='0';
          if(serial_values_on == 1) {
            serial_values_on = 0;
          } else if(serial_values_on == 0) {
            serial_values_on = 1;
          }
        }
          break;
        case 'x':
        {
          rr ='0';
          if(x_values_on == 1) {
            x_values_on = 0;
          } else if(serial_values_on == 0) {
            x_values_on = 1;
          }
        }
            break;
        case 't':
        {
          rr ='0';
          if(telem_values_on == 1) {
            telem_values_on = 0;
          } else if(telem_values_on == 0) {
            telem_values_on = 1;
          }
        }
            break;
        case 'f':
        {
          rr = '0';
          if(fusion_on == 1) {
            fusion_on = 0;
          } else if(fusion_on == 0) {
            fusion_on = 1;
          }
        }
        case '\r':
        case '\n':
        case 'h': menu(); break;
      }
      while (cout.read() != -1) ; // remove rest of characters.
    }

  if(fusion_on == 1) {
    getFusion();
  }
}


void getCalIMU() {
  // read the sensor
  /* Check if data read */
  //NOTE for Fusion gyro data rate is the driver not the magnetomer
  //if (imu.Read_raw(raw_values) & imu.new_imu_data_()) {
  if (imu.Read_raw(raw_val)) {
    newIMUData = imu.new_imu_data();
    now1= micros(); // This is when the data reported READY
    val[4] = (float)raw_val[3];
    val[3] = (float)raw_val[4];
    val[5] = -(float)raw_val[5];

#if defined(use_freeimu)
    val[0] = ((float)(raw_val[1] - cal_acc_off[1]) / (float)cal_acc_scale[1]);
    val[1] = ((float)(raw_val[0] - cal_acc_off[0]) / (float)cal_acc_scale[0]); 
    val[2] = -((float)(raw_val[2] - cal_acc_off[2])  / (float)cal_acc_scale[2]);
  #if defined(use_magneto) || defined(use_motioncal)
    val[6] = (float)raw_val[6] * magScale[0];
    val[7] = (float)raw_val[7] * magScale[1];
    val[8] = (float)raw_val[8] * magScale[2];
  #else
    val[6] = ((float)(raw_val[6] - cal_magn_off[0]) / (float)cal_magn_scale[0]);
    val[7] =  ((float)(raw_val[7] - cal_magn_off[1]) / (float)cal_magn_scale[1]) ;
    val[8] = ((float)(raw_val[8] - cal_magn_off[2])  / (float)cal_magn_scale[2]) ;
  #endif

#elif defined(use_magneto) || defined(use_motioncal)  // note have to define accel cal
  //use min-max for acceleration???
    val[0] = (((float)raw_val[1] * accelScale) - acc_off[0]/g) * acc_scale[0] ;
    val[1] = (((float)raw_val[0] * accelScale) - acc_off[1]/g) * acc_scale[1] ;
    val[2] = ((-(float)raw_val[2] * accelScale) - acc_off[2]/g) * acc_scale[2];
    val[6] = (float)raw_val[6] * magScale[0];
    val[7] = (float)raw_val[7] * magScale[1];
    val[8] = (float)raw_val[8] * magScale[2];

#elif defined(use_minmax)
    val[6] = (((float)raw_val[6] * magScale[0]) - magn_off[0]) * magn_scale[0];
    val[7] = (((float)raw_val[7] * magScale[1]) - magn_off[1]) * magn_scale[1];
    val[8] = (((float)raw_val[8] * magScale[2]) - magn_off[2]) * magn_scale[2];
    val[0] = (((float)raw_val[1] * accelScale) - acc_off[0]/g) * acc_scale[0] ;
    val[1] = (((float)raw_val[0] * accelScale) - acc_off[1]/g) * acc_scale[1] ;
    val[2] = ((-(float)raw_val[2] * accelScale) - acc_off[2]/g) * acc_scale[2];
    
#endif
  }
}

void getFusion() {
  getCalIMU();

  if(newIMUData) {
    newIMUData = 0;
    dt = ((now1 - lastUpdate) * 0.000001);
    lastUpdate = now1;

    //Lets normalize magnetometer data
    //float mag_norm = sqrt(val[6]*val[6] + val[7]*val[7] + val[8]*val[8]);

    // Acquire latest sensor data
    FusionVector gyroscope = { val[3], val[4], val[5] }; // replace this with actual gyroscope data in degrees/s
    FusionVector accelerometer = { val[0], val[1], val[2] }; // replace this with actual accelerometer data in g
    FusionVector magnetometer = {  val[6], val[7], val[8] }; // replace this with actual magnetometer data in arbitrary units

    // Apply calibration
    gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
    accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
#if defined(use_magneto) || defined(use_motioncal)
    magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);
#endif
    // Update gyroscope offset correction algorithm
    //gyroscope = FusionOffsetUpdate(&offset, gyroscope);

    // Calculate delta time (in seconds) to account for gyroscope sample clock error
    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, dt);
    //FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, dt);

    flags = FusionAhrsGetFlags(&ahrs);
    euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    q = FusionAhrsGetQuaternion(&ahrs);
    fHeading = FusionCompassCalculateHeading(FusionConventionNed, accelerometer, magnetometer);


    //if(dump > 100) {
      if(raw_values_on == 1) {
        print_float_array(val, 9);
        Serial.println();
      }

      if(serial_values_on == 1) {
        Serial.printf("Fusion (RPY): %0.1f, %0.1f, %0.1f, %0.1f\n", fHeading, euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
      }
  
      if(x_values_on == 1) {
        timestamp = micros();
        char accelgyroBuffer[100];
        sprintf(accelgyroBuffer, "%c,%llu,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f", 'I', timestamp, gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z, accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z);
        Serial.printf ("%s\n",accelgyroBuffer);
        sprintf(accelgyroBuffer, "%c,%llu,%0.6f,%0.6f,%0.6f", 'M', timestamp, magnetometer.axis.x, magnetometer.axis.y, magnetometer.axis.z);
        Serial.printf ("%s\n",accelgyroBuffer);
        sprintf(accelgyroBuffer, "%c,%llu,%0.6f,%0.6f,%0.6f,%0.6f", 'Q', timestamp, q.element.w, q.element.x, q.element.y, q.element.z);
        Serial.printf ("%s\n",accelgyroBuffer);
      }

      if(telem_values_on == 1) telemetryPortOut();
      //dump = 0;
  }
}

void gyroCalibration() {
  int numSamples = 500;
  gyrox_off = 0;
  gyroy_off = 0;
  gyroz_off = 0;
  //imu.DisableDrdyInt();
  uint16_t icount = 0;
  while(icount < numSamples){
    imu.Read_raw(raw_val);
    if(imu.new_imu_data()){
      icount += 1;
      gyrox_off += raw_val[3];
      gyroy_off += raw_val[4];
      gyroz_off += raw_val[5];
    }
  }
  gyrox_off = gyrox_off / numSamples;
  gyroy_off = gyroy_off / numSamples;
  gyroz_off = gyroz_off / numSamples;
  //imu.EnableDrdyInt();
  Serial.printf("%f, %f, %f\n", gyrox_off, gyroy_off,gyroz_off );
  gyroscopeSensitivity = {gyroScale, gyroScale, gyroScale};
  gyroscopeOffset = {gyrox_off, gyrox_off, gyrox_off};
}

void menu()
{
  cout.println();
  cout.println("Menu Options:");
  cout.println("========================================");
  cout.println("\tx - x-IMU3 GUI Output");
  cout.println("\tt - Telemetry Viewer Output");
  cout.println("\ts - Serial Print Output (Euler Angles)");
  cout.println("\tf - Fusion On");
  cout.println("\tr - Print Values");
  cout.println("========================================");
  cout.println("\tg - Zero Gyroscope");

  cout.println("========================================");
  cout.println("\th - Menu");
  cout.println("========================================");
  cout.println();
}
