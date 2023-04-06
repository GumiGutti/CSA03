// incl
//sdf
// adxl
// mpu
// bmp
// bme
// ina

// időzítés definíciók

// task:
// for(;;):
// szenzor olvasások:
//   200Hz: MPU,ADXL,kvaternió,integrálás (pozíció)

TaskHandle_t hCore0task;

#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_ADXL375.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>
// #include <bme280.h>
// #include <Adafruit_INA219.h>


//#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//    #include "Wire.h"
//#endif 

Adafruit_BMP280 bmp;
TwoWire twi(0);
OneWire ds(ds18Pin);
//MPU6050 mpu;
//MPU6050 accelgyro;
Adafruit_BME280 bme;

// SPIClass hspi(HSPI);
// Adafruit_ADXL375 accel = Adafruit_ADXL375(accCS, &hspi, 12345);



bool dmpReady = false;   // set true if DMP init was successful
//uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer
int16_t ax, ay, az;      //For acceleration

// Variables for the IMU
Quaternion q;    // [w, x, y, z]         quaternion container
VectorInt16 aa;  // [x, y, z]            accel sensor measurements
VectorFloat gravity;
float ypr[3];

uint32_t tImuTrigger = 0;
uint32_t tImuDelay = 50; 

uint32_t tBMPdelay = 100;
uint32_t tBMPtrigger = 0;

uint32_t tDSdelay = 125; 
uint32_t tDStrigger = 0;

uint32_t tBMEdelay = 100;
uint32_t tBMEtrigger = 0;


enum taskState { sRun,
                 sError,
                 sStart };
taskState sImu = sStart;
taskState sBME = sStart;
taskState sDallas = sStart;
taskState sBMP = sStart;
bool firstRunCore0 = true;

void core0task(void* parameter);

void core0setup() {  // a.k.a. setup
  xTaskCreatePinnedToCore(
    core0task,
    "core0task",
    10000,
    NULL,
    CORE0TASKPRIO,
    &hCore0task,
    0);
}

void core0task(void* parameter) {  // a.k.a. loop
  s.println("core0task fut");
  for (;;) {
    if (firstRunCore0) {
      //twi.begin(i2cSDA, i2cSCL, 400000);
      s.println("Elsőkör");
      firstRunCore0 = false;
    }
    // imu .... 
    /*
    if (millis() - tImuTrigger > tImuDelay) {
      tImuTrigger = millis();
      switch (sImu) {
        case sRun:
          {
            GetAcceleration();
            GetOrientation();
            GetBMEdata();
            break;
          }
        case sError:
          {
            break;
          }
        case sStart:
          {
            setupIMU();
            //setupBME();
            break;
          }
        default:
          {  //itt baj van....}
          }
      }
      status.imuOK = (sImu == sRun);  // && range check OK
    }
     */ 
    if (millis() - tDStrigger > tDSdelay) {
      tDStrigger = millis();
      
      switch (sDallas) {
        case sRun:
          {
            uint16_t result;
            byte data[2];
            ds.reset();
            ds.write(0xcc);
            ds.write(0xbe);
            data[0] = ds.read();
            data[1] = ds.read();
            result = ((uint16_t)data[1] << 8) | data[0];
            ds.reset();
            ds.write(0xcc);
            ds.write(0x44, 1);
            if (data[1] & 128) {
              s.println((((~result) >> 2) + 1) / -4.0);
            } else {
              s.println((result >> 2) / 4.0);
            }
            break;
          }
        case sError:
          {
            break;
          }
        case sStart:
          {
            ds.write(0xcc);
            ds.write(0x4e, 1);
            ds.write(0x7f, 1);
            ds.write(0xfc, 1);
            ds.write(0x3f, 0);
            sDallas = sRun;
            break;
          }
        default:
          {  //itt baj van....}
          }
      }
    }
    
    if (millis() - tBMPtrigger > tBMPdelay) {
      tBMPtrigger = millis();
      
      switch (sBMP) {
        case sRun:
          {
            s.print(F("Temperature = "));
            s.print(bmp.readTemperature());
            s.println(" *C");

            s.print(F("Pressure = "));
            s.print(bmp.readPressure());
            s.println(" Pa");

            s.print(F("Approx altitude = "));
            s.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
            s.println(" m");
            break;
          }
        case sError:
          {
            break;
          }
        case sStart:
          {
            unsigned status;
            //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
            status = bmp.begin(0x76, 0x58);
            if (!status) {
              s.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
              sBMP = sError;
            }

            /* Default settings from datasheet. */
            bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                            Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                            Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                            Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                            Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
            sBMP = sRun;
            break;
          }
        default:
          {  //itt baj van....}
          }
      }
    }

    if (millis() - tBMEtrigger > tBMEdelay) {
      tBMEtrigger = millis();

      switch (sBME) {
        case sRun:
          {
            s.print(F("BMETemperature = "));
            s.print(bme.readTemperature());
            s.println(" *C");

            s.print(F("BMEPressure = "));
            s.print(bme.readPressure());
            s.println(" Pa");

            s.print(F("BMEApprox altitude = "));
            s.print(bme.readAltitude(1013.25)); /* Adjusted to local forecast! */
            s.println(" m");

            s.print("BMEHumidity = ");
            s.print(bme.readHumidity());
            s.println(" %");
            break;
          }
        case sError:
          {
            break;
          }
        case sStart:
          {
            unsigned statusBME;
            
            // default settings
            statusBME = bme.begin(0x77);  
            // You can also pass in a Wire library object like &Wire2
            // status = bme.begin(0x76, &Wire2)
            if (!statusBME) {
               s.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
               sBME = sError;
               }
            sBME = sRun;
            break;
          }
        default:
          {  //itt baj van....}
          }
      }
    }
  }
}
