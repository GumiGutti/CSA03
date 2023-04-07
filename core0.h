// incl
// adxl
// mpu
// bmp
// bme
// ina

// időzítés definíciók

// task:
// for(;;):
// szenzor olvasások:
// 200Hz: MPU,ADXL,kvaternió,integrálás (pozíció)

TaskHandle_t hCore0task;

#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA219.h>

bool debug = 1;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

Adafruit_INA219 ina219;
MPU6050 mpu;
Adafruit_BMP280 bmp;
TwoWire twi(0);
OneWire ds(ds18Pin);
Adafruit_BME280 bme;

SPIClass hspi(HSPI);
Adafruit_ADXL375 accel = Adafruit_ADXL375(accCS, &hspi, 12345);

bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// Variables for the IMU
Quaternion q;    // [w, x, y, z]         quaternion container
VectorInt16 aa;  // [x, y, z]            accel sensor measurements
VectorFloat gravity;
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
float ypr[3];

float disp_x = 0;
float disp_y = 0;
float disp_z = 0;  //imu számoláshoz elmozdulás

float velo_x = 0;
float velo_y = 0;
float velo_z = 0;  //imu számoláshoz sebesség

float deltaT;  //imu integráláshoz eltelt idő

float accel_x, accel_y, accel_z;     //adxl mérések ide jönnek
float Iaccel_x, Iaccel_y, Iaccel_z;  //imu mérések ide jönnek

//ina mérések ide jönnek
float busvoltage = 0;
float current_mA = 0;

volatile bool mpuInterrupt = false;
void ICACHE_RAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}

uint32_t tImuTrigger = 0;
uint32_t tImuDelay = 50;
uint32_t tIMUrestart = 5000;
uint32_t tIMUrestrigger = 0;

uint32_t tBMPdelay = 100;
uint32_t tBMPtrigger = 0;
uint32_t tBMPrestart = 5000;
uint32_t tBMPrestrigger = 0;

uint32_t tDSdelay = 125;
uint32_t tDStrigger = 0;

uint32_t tBMEdelay = 100;
uint32_t tBMEtrigger = 0;
uint32_t tBMErestart = 5000;
uint32_t tBMErestrigger = 0;

uint32_t tADXLdelay = 100;
uint32_t tADXLtrigger = 0;
uint32_t tADXLrestrigger = 0;
uint32_t tADXLrestart = 5000;

uint32_t tINAdelay = 100;
uint32_t tINAtrigger = 0;


enum taskState { sRun,
                 sError,
                 sStart };
taskState sImu = sStart;
taskState sBME = sStart;
taskState sDallas = sStart;
taskState sBMP = sStart;
taskState sADXL = sStart;
taskState sINA = sStart;

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

    if (millis() - tImuTrigger > tImuDelay) {
      tImuTrigger = millis();
      switch (sImu) {
        case sRun:
          {
            //kvaterniók
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            xSemaphoreTake(xQa, portMAX_DELAY);
            Qa = q.w;
            xSemaphoreGive(xQa);

            xSemaphoreTake(xQi, portMAX_DELAY);
            Qi = q.x;
            xSemaphoreGive(xQi);

            xSemaphoreTake(xQj, portMAX_DELAY);
            Qj = q.y;
            xSemaphoreGive(xQj);

            xSemaphoreTake(xQk, portMAX_DELAY);
            Qk = q.z;
            xSemaphoreGive(xQk);

            //Worldaccel
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            float Wacc_x = aaWorld.x;
            float Wacc_y = aaWorld.y;
            float Wacc_z = aaWorld.z;
            deltaT = millis() - tImuTrigger;

            velo_x += Wacc_x * deltaT;
            velo_y += Wacc_y * deltaT;
            velo_z += Wacc_z * deltaT;

            disp_x += velo_x * deltaT;
            disp_y += velo_y * deltaT;
            disp_z += velo_z * deltaT;

            xSemaphoreTake(xPosx, portMAX_DELAY);
            Posx = disp_x;
            xSemaphoreGive(xPosx);

            xSemaphoreTake(xPosy, portMAX_DELAY);
            Posy = disp_y;
            xSemaphoreGive(xPosy);

            xSemaphoreTake(xPosz, portMAX_DELAY);
            Posz = disp_z;
            xSemaphoreGive(xPosz);
            //s.println("Imu accs:");
            //s.println(aa.x);
            //s.println(aa.y);
            //s.println(aa.z);
            break;
          }
        case sError:
          {
            if (millis() - tIMUrestrigger > tIMUrestart) {
              tIMUrestrigger = millis();
              sImu = sStart;
            }
            break;
          }
        case sStart:
          {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
            Wire.begin();
            //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
            Fastwire::setup(400, true);
#endif

            // initialize device
            mpu.initialize();
            pinMode(INTERRUPT_PIN, INPUT);

            devStatus = mpu.dmpInitialize();

            // supply your own gyro offsets here, scaled for min sensitivity
            mpu.setXGyroOffset(220);
            mpu.setYGyroOffset(76);
            mpu.setZGyroOffset(-85);
            mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip

            if (devStatus == 0) {
              // Calibration Time: generate offsets and calibrate our MPU6050
              mpu.CalibrateAccel(6);
              mpu.CalibrateGyro(6);
              mpu.PrintActiveOffsets();
              mpu.setDMPEnabled(true);

              // enable Arduino interrupt detection
              digitalPinToInterrupt(INTERRUPT_PIN);
              attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
              mpuIntStatus = mpu.getIntStatus();

              // set our DMP Ready flag so the main loop() function knows it's okay to use it
              dmpReady = true;

              // get expected DMP packet size for later comparison
              packetSize = mpu.dmpGetFIFOPacketSize();
              xSemaphoreTake(xImuok, portMAX_DELAY);
              Imuok = 1;
              xSemaphoreGive(xImuok);
              sImu = sRun;
            } else {
              if (debug) {
                s.print(F("DMP Initialization failed (code "));
                s.print(devStatus);
                s.println(F(")"));
                xSemaphoreTake(xImuok, portMAX_DELAY);
                Imuok = 0;
                xSemaphoreGive(xImuok);
              }

              sImu = sError;
            }
            break;
          }
        default:
          {  //itt baj van....}
          }
      }
      //status.imuOK = (sImu == sRun);  // && range check OK
    }

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
            xSemaphoreTake(xTemp, portMAX_DELAY);
            if (data[1] & 128) {
              Temp = (((~result) >> 2) + 1) / -4.0;
            } else {
              Temp = (result >> 2) / 4.0;
            }
            xSemaphoreGive(xTemp);
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
            xSemaphoreTake(xDsok, portMAX_DELAY);
            Dsok = 1;
            xSemaphoreGive(xDsok);
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
            /*
            s.print(F("Temperature = "));
            s.print(bmp.readTemperature());
            s.println(" *C");

            s.print(F("Pressure = "));
            s.print(bmp.readPressure());
            s.println(" Pa");

            s.print(F("Approx altitude = "));
            s.print(bmp.readAltitude(1013.25)); // Adjusted to local forecast! 
            s.println(" m");
            */
            break;
          }
        case sError:
          {
            if (millis() - tBMPrestrigger > tBMPrestart) {
              tBMPrestrigger = millis();
              sBMP = sStart;
            }
            break;
          }
        case sStart:
          {
            unsigned status;
            //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
            status = bmp.begin(0x76, 0x58);
            if (!status) {
              if (debug) s.println(F("Could not find a valid BMP280 sensor, check wiring or "
                                     "try a different address!"));
              xSemaphoreTake(xBmpok, portMAX_DELAY);
              Bmpok = 0;
              xSemaphoreGive(xBmpok);
              sBMP = sError;
            }

            /* Default settings from datasheet. */
            bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                            Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                            Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                            Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                            Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
            xSemaphoreTake(xBmpok, portMAX_DELAY);
            Bmpok = 1;
            xSemaphoreGive(xBmpok);
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
            xSemaphoreTake(xPressure, portMAX_DELAY);
            Pressure = bme.readPressure();
            xSemaphoreGive(xPressure);

            xSemaphoreTake(xHumidity, portMAX_DELAY);
            Humidity = bme.readHumidity();
            xSemaphoreGive(xHumidity);
            break;
          }
        case sError:
          {
            if (millis() - tBMErestrigger > tBMErestart) {
              tBMErestrigger = millis();
              sBME = sStart;
            } else {  //Hiba esetén a BMP-től kéri az adatot
              xSemaphoreTake(xPressure, portMAX_DELAY);
              Pressure = bmp.readPressure();
              xSemaphoreGive(xPressure);

              xSemaphoreTake(xHumidity, portMAX_DELAY);
              Humidity = bme.readHumidity();
              xSemaphoreGive(xHumidity);
            }
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
              if (debug) s.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
              xSemaphoreTake(xBmeok, portMAX_DELAY);
              Bmeok = 0;
              xSemaphoreGive(xBmeok);
              sBME = sError;
            } else {
              xSemaphoreTake(xBmeok, portMAX_DELAY);
              Bmeok = 1;
              xSemaphoreGive(xBmeok);
              sBME = sRun;
            }
            break;
          }
        default:
          {  //itt baj van....}
          }
      }
    }

    if (millis() - tADXLtrigger > tADXLdelay) {
      tADXLtrigger = millis();

      switch (sADXL) {
        case sRun:
          {
            sensors_event_t event;
            accel.getEvent(&event);
            accel_x = event.acceleration.x;
            accel_y = event.acceleration.y;
            accel_z = event.acceleration.z;
            //s.println("ADXL accels");
            //s.println(accel_x * 0.49);
            //s.println(accel_y * 0.49);
            //s.println(accel_z * 0.49);
            break;
          }
        case sError:
          {
            if (millis() - tADXLrestrigger > tADXLrestart) {
              tADXLrestrigger = millis();
              sADXL = sStart;
            }
            break;
          }
        case sStart:
          {
            if (!accel.begin()) {
              if (debug) s.println("ADXL nem müksz");
              xSemaphoreTake(xAdxlok, portMAX_DELAY);
              Adxlok = 0;
              xSemaphoreGive(xAdxlok);
              sADXL = sError;
            } else {
              xSemaphoreTake(xAdxlok, portMAX_DELAY);
              Adxlok = 1;
              xSemaphoreGive(xAdxlok);
              sADXL = sRun;
            }
            break;
          }
        default:
          {  //itt baj van....}
          }
      }
    }

    if (millis() - tINAtrigger > tINAdelay) {
      tINAtrigger = millis();

      switch (sINA) {
        case sRun:
          {
            xSemaphoreTake(xCurrent, portMAX_DELAY);
            Current = ina219.getCurrent_mA();
            xSemaphoreGive(xCurrent);

            xSemaphoreTake(xVoltage, portMAX_DELAY);
            Voltage = ina219.getBusVoltage_V();
            xSemaphoreGive(xVoltage);
            break;
          }
        case sError:
          {
            break;
          }
        case sStart:
          {
            if (!ina219.begin()) {
              if (debug) s.println("ADXL nem müksz");
              xSemaphoreTake(xInaok, portMAX_DELAY);
              Inaok = 0;
              xSemaphoreGive(xInaok);
              sINA = sINA;
            } else {
              xSemaphoreTake(xInaok, portMAX_DELAY);
              Inaok = 1;
              xSemaphoreGive(xInaok);
              sINA = sRun;
            }
            break;
          }
        default:
          {  //itt baj van....}
          }
      }
    }
  }
}
