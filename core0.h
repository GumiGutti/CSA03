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
#include <MPU6500_WE.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA219.h>

bool debug = 0;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

Adafruit_INA219 ina219;
MPU6050 mpu;
MPU6050 accelgyro;
Adafruit_BMP280 bmp;
OneWire ds(ds18Pin);
Adafruit_BME280 bme;
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);

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

unsigned long deltaT = 0;  //imu integráláshoz eltelt idő
unsigned long current = 0;

float accel_x, accel_y, accel_z;  //adxl mérések ide jönnek
float xx, yy, zz;                 //adxl calibrated

float ax, ay, az;     //imu gyorsulások m/s2 ben
float gyx, gyy, gyz;  //imu gyro értékek deg/sec ben

//ina mérések ide jönnek
float busvoltage = 0;
float current_mA = 0;

int16_t x, y, z;  //adxl kalibrációhoz

float DStemp, BMEtemp, BMPtemp;  //Méréseknek
float BMEpress, BMPpress;

volatile bool mpuInterrupt = false;
void ICACHE_RAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}

uint32_t tImuTrigger = 0;
uint32_t tImuDelay = 150;
uint32_t tIMUrestart = 5000;
uint32_t tIMUrestrigger = 0;

uint32_t tBMPdelay = 100;
uint32_t tBMPtrigger = 0;
uint32_t tBMPrestart = 5000;
uint32_t tBMPrestrigger = 0;

uint32_t tFusionTrigger = 0;
uint32_t tFusionDelay = 100;

uint32_t tDSdelay = 100;
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
uint32_t tINArestart = 5000;
uint32_t tINArestrigger = 0;


enum taskState { sRun,
                 sError,
                 sStart,
                 sCalib };
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
  //s.println("core0task fut");
  for (;;) {
    if (firstRunCore0) {
      Wire.begin(i2cSDA, i2cSCL, 400000);
      /*
      byte error, address;
      int nDevices;
      s.println("Scanning I2C");
      s.println("IN219               0x40");
      s.println("GY-91 modul MPU6500 0x68");
      s.println("GY-91 modul BMP280  0x76");
      s.println("BME280              0x77");
      s.println("BME280 SDO pin direkt felhúzása nélkül azonos a két BME címe!");
      s.println("");
      nDevices = 0;
      for (address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
          s.print("I2C device found at address 0x");
          if (address < 16)
            s.print("0");
          s.print(address, HEX);
          s.println("  !");

          nDevices++;
        } else if (error == 4) {
          s.print("Unknown error at address 0x");
          if (address < 16)
            s.print("0");
          s.println(address, HEX);
        }
      }
      if (nDevices == 0)
        s.println("No I2C devices found\n");
      else
        s.println("done\n");
      //s.println("Elsőkör");
      */
      firstRunCore0 = false;
    }

    if (millis() - tImuTrigger > tImuDelay) {
      tImuTrigger = millis();
      switch (sImu) {
        case sRun:
          {
            xyzFloat gValue = myMPU6500.getGValues();
            xyzFloat gyr = myMPU6500.getGyrValues();
            ax = (gValue.x) * 9.81;
            ay = (gValue.y) * 9.81;
            az = (gValue.z) * 9.81;
            gyx = gyr.x;
            gyy = gyr.y;
            gyz = gyr.z;

            xSemaphoreTake(xQi, portMAX_DELAY);
            Qi = gyx;
            xSemaphoreGive(xQi);

            xSemaphoreTake(xQj, portMAX_DELAY);
            Qj = gyy;
            xSemaphoreGive(xQj);

            xSemaphoreTake(xQk, portMAX_DELAY);
            Qk = gyz;
            xSemaphoreGive(xQk);

            xSemaphoreTake(xPosx, portMAX_DELAY);
            Posx = gyx;
            xSemaphoreGive(xPosx);

            xSemaphoreTake(xPosy, portMAX_DELAY);
            Posy = gyx;
            xSemaphoreGive(xQy);

            xSemaphoreTake(xPosz, portMAX_DELAY);
            Posz = gyx;
            xSemaphoreGive(xPosy);

            if (debug) {
              s.print(ax); s.print("\t");
              s.print(ay); s.print("\t");
              s.print(az); s.println("\t");
            }
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
        case sCalib:
          {
            myMPU6500.autoOffsets();
            if (debug) s.println("imu kalibrálva");
            sImu = sRun;
            break;
          }
        case sStart:
          {
            if (!myMPU6500.init()) {
              if (debug) s.println("imu bajos");
              sImu = sError;
            } else {
              if (debug) s.println("imu fasza");
            }
            myMPU6500.enableGyrDLPF();
            myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
            myMPU6500.setSampleRateDivider(5);
            myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_2000);
            myMPU6500.setAccRange(MPU6500_ACC_RANGE_16G);
            myMPU6500.enableAccDLPF(true);
            myMPU6500.setAccDLPF(MPU6500_DLPF_6);
            sImu = sRun;
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
            if (data[1] & 128) {
              DStemp = (((~result) >> 2) + 1) / -4.0;
            } else {
              DStemp = (result >> 2) / 4.0;
            }
            //range check
            if (-6.0 < DStemp < 35.0) {
              xSemaphoreTake(xDsok, portMAX_DELAY);
              Dsok = 1;
              xSemaphoreGive(xDsok);
            } else {
              xSemaphoreTake(xDsok, portMAX_DELAY);
              Dsok = 0;
              xSemaphoreGive(xDsok);
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
            BMPtemp = bmp.readTemperature();
            if (-6.0 < BMPtemp < 35.0) {
              xSemaphoreTake(xBmpok, portMAX_DELAY);
              Bmpok = 1;
              xSemaphoreGive(xBmpok);
            } else {
              xSemaphoreTake(xBmpok, portMAX_DELAY);
              Bmpok = 0;
              xSemaphoreGive(xBmpok);
            }
            break;
            BMPpress = bmp.readPressure();
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
            xSemaphoreTake(xHumidity, portMAX_DELAY);
            Humidity = bme.readHumidity();
            xSemaphoreGive(xHumidity);

            BMEtemp = bme.readTemperature();
            if (-6.0 < BMEtemp < 35.0) {
              xSemaphoreTake(xBmeok, portMAX_DELAY);
              Bmeok = 1;
              xSemaphoreGive(xBmeok);
            } else {
              xSemaphoreTake(xBmeok, portMAX_DELAY);
              Bmeok = 0;
              xSemaphoreGive(xBmeok);
            }
            break;
            BMEpress = bme.readPressure();
            break;
          }
        case sError:
          {
            if (millis() - tBMErestrigger > tBMErestart) {
              tBMErestrigger = millis();
              sBME = sStart;
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
            accel_x = (event.acceleration.x);
            accel_y = (event.acceleration.y);
            accel_z = (event.acceleration.z);
            //s.println("X");
            //s.println(accel_x);
            //s.println(accel_y);
            //s.println(accel_z);
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
              //s.println("ADXL nem jóó ");
              xSemaphoreTake(xAdxlok, portMAX_DELAY);
              Adxlok = 0;
              xSemaphoreGive(xAdxlok);
              sADXL = sError;
            } else {
              //s.println("ADXL jóó");
              xSemaphoreTake(xAdxlok, portMAX_DELAY);
              Adxlok = 1;
              xSemaphoreGive(xAdxlok);
              sADXL = sRun;
            }
            break;
          }
        case sCalib:
          {
            //Hold accelerometer flat to set offsets to 0, 0, and -1g...
            accel.setTrimOffsets(0, 0, 0);
            xx = (float)accel.getX();
            yy = (float)accel.getY();
            zz = (float)accel.getZ();
            for (int i = 0; i < 1000; i++) {
              xx = xx * 0.99 + 0.01 * (float)accel.getX();
              yy = yy * 0.99 + 0.01 * (float)accel.getY();
              zz = zz * 0.99 + 0.01 * (float)accel.getZ();
              delay(5);
            }
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
            if (debug) {
              s.println("Ina äram és fesz:");
              s.println(ina219.getCurrent_mA());
              s.println(ina219.getBusVoltage_V());
            }
            break;
          }
        case sError:
          {
            if (millis() - tINArestrigger > tINArestart) {
              tINArestrigger = millis();
              sINA = sStart;
            }
            break;
          }
        case sStart:
          {
            if (!ina219.begin()) {
              if (debug) s.println("INA nem müksz");
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

    if (millis() - tFusionTrigger > tFusionDelay) {
      tFusionTrigger = millis();

      if (true) {                      // Temperature fusion
        if (Dsok && Bmeok && Bmpok) {  // Ha minden ok
          xSemaphoreTake(xTemp, portMAX_DELAY);
          Temp = DStemp;
          xSemaphoreGive(xTemp);

        } else if (!Bmpok && !Bmeok) {  //Ha 2 kiesik a 3-ból mindig a fennmaradót vegye alapul

          xSemaphoreTake(xTemp, portMAX_DELAY);
          Temp = DStemp;
          xSemaphoreGive(xTemp);
        } else if (!Dsok && !Bmeok) {

          xSemaphoreTake(xTemp, portMAX_DELAY);
          Temp = BMPtemp;
          xSemaphoreGive(xTemp);
        } else if (!Bmeok && !Dsok) {

          xSemaphoreTake(xTemp, portMAX_DELAY);
          Temp = BMEtemp;
          xSemaphoreGive(xTemp);

        } else if (Dsok && Bmeok && !Bmpok) {  //Ha egy kiesik a 3-ból mindig a másik 2 számtani közepét vegye

          xSemaphoreTake(xTemp, portMAX_DELAY);
          Temp = DStemp;
          xSemaphoreGive(xTemp);
        } else if (!Dsok && Bmeok && Bmpok) {

          xSemaphoreTake(xTemp, portMAX_DELAY);
          Temp = (BMEtemp + BMPtemp) / 2;
          xSemaphoreGive(xTemp);
        } else if (Dsok && !Bmeok && Bmpok) {

          xSemaphoreTake(xTemp, portMAX_DELAY);
          Temp = DStemp;
          xSemaphoreGive(xTemp);
        } else {
          xSemaphoreTake(xTemp, portMAX_DELAY);
          Temp = 16.0;  //Ha semmi se jó
          xSemaphoreGive(xTemp);
        }
      }

      if (true) {  // Pressure fusion
        if (Bmeok && Bmpok) {
          xSemaphoreTake(xPressure, portMAX_DELAY);
          Pressure = (BMEpress + BMPpress) / 2.0;
          xSemaphoreGive(xPressure);
        } else if (Bmeok && !Bmpok) {
          xSemaphoreTake(xPressure, portMAX_DELAY);
          Pressure = BMEpress;
          xSemaphoreGive(xPressure);
        } else if (!Bmeok && Bmpok) {
          xSemaphoreTake(xPressure, portMAX_DELAY);
          Pressure = BMPpress;
          xSemaphoreGive(xPressure);
        } else {
          xSemaphoreTake(xPressure, portMAX_DELAY);
          Pressure = 100000.0;  //Ha semmi se jó
          xSemaphoreGive(xPressure);
        }
      }
    }
  }
}
