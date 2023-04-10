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

bool debug = 0;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

Adafruit_INA219 ina219;
MPU6050 mpu;
MPU6050 accelgyro;
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

int16_t ax, ay, az;
int16_t gx, gy, gz;

//ina mérések ide jönnek
float busvoltage = 0;
float current_mA = 0;

int16_t x, y, z; //adxl kalibrációhoz

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


enum taskState { sRun,
                 sError,
                 sStart,
                 sCalib };
taskState sImu = sError;
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
      byte error, address;
      int nDevices;
      s.println("Scanning I2C");
      s.println("IN219               0x41");
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


      //twi.begin(i2cSDA, i2cSCL, 400000);
      //s.println("Elsőkör");
      firstRunCore0 = false;
    }

    if (millis() - tImuTrigger > tImuDelay) {
      tImuTrigger = millis();
      switch (sImu) {
        case sRun:
          {
            accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            //s.print("a/g:\t");
            //s.print((float)ax/ 16384.0); s.print("\t");
            //s.print((float)ay/ 16384.0); s.print("\t");
            //s.print((float)az/ 16384.0); s.print("\t");
            //s.print(gx); s.print("\t");
            //s.print(gy); s.print("\t");
            //s.println(gz);
            if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {            
              
              
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
              float Wacc_x = ((float)aaWorld.x / 16384.0);
              float Wacc_y = ((float)aaWorld.y / 16384.0);
              float Wacc_z = ((float)aaWorld.z / 16384.0);
              deltaT = 1000.0 / tImuDelay;

              velo_x += Wacc_x * deltaT;
              velo_y += Wacc_y * deltaT;
              velo_z += Wacc_z * deltaT;

              disp_x += velo_x * deltaT;
              disp_y += velo_y * deltaT;
              disp_z += velo_z * deltaT;
              if (debug) {
                s.println("Worldaccels");
                s.println(aaWorld.x);
                s.println(aaWorld.y);
                s.println(aaWorld.z);
                s.println("Displacements");
                s.println(disp_x);
                s.println(disp_y);
                s.println(disp_z);
              }

              xSemaphoreTake(xPosx, portMAX_DELAY);
              Posx = disp_x;
              xSemaphoreGive(xPosx);

              xSemaphoreTake(xPosy, portMAX_DELAY);
              Posy = disp_y;
              xSemaphoreGive(xPosy);

              xSemaphoreTake(xPosz, portMAX_DELAY);
              Posz = disp_z;
              xSemaphoreGive(xPosz);
            } else {
              if (debug) s.println("imu bajos");
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
        case sStart:
          {

            //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

            // initialize device
            mpu.initialize();
            accelgyro.initialize();

            devStatus = mpu.dmpInitialize();

            // supply your own gyro offsets here, scaled for min sensitivity
            mpu.setXGyroOffset(-157);
            mpu.setYGyroOffset(-377);
            mpu.setZGyroOffset(-6);
            mpu.setXAccelOffset(4459);
            mpu.setYAccelOffset(4615);
            mpu.setZAccelOffset(23235);  // 1688 factory default for my test chip


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
              xSemaphoreTake(xImuok, portMAX_DELAY);
              Imuok = 0;
              xSemaphoreGive(xImuok);
              if (debug) {
                s.print(F("DMP Initialization failed (code "));
                s.print(devStatus);
                s.println(F(")"));
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
            x = accel.getX();
            y = accel.getY();
            z = accel.getZ();
            accel.setTrimOffsets(-(x+2)/4, 
                       -(y+2)/4, 
                       -(z-20+2)/4);
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
              s.println(ina219.getCurrent_mA());
              s.println(ina219.getBusVoltage_V());
            }
            break;
          }
        case sError:
          {
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
          Temp = (BMEtemp + DStemp + BMPtemp) / 3;
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
          Temp = (BMEtemp + DStemp) / 2;
          xSemaphoreGive(xTemp);
        } else if (!Dsok && Bmeok && Bmpok) {

          xSemaphoreTake(xTemp, portMAX_DELAY);
          Temp = (BMEtemp + BMPtemp) / 2;
          xSemaphoreGive(xTemp);
        } else if (Dsok && !Bmeok && Bmpok) {

          xSemaphoreTake(xTemp, portMAX_DELAY);
          Temp = (BMPtemp + DStemp) / 2;
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
          Pressure = BMEpress;
          xSemaphoreGive(xPressure);
        }
      }
    }
  }
}
