// incl:
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
#include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_ADXL375.h>
#include <MPU6500_WE.h>
#include <Adafruit_BMP280.h>
// #include <bme280.h>
// #include <Adafruit_INA219.h>

TwoWire twi(0);
// #define MPU6500_ADDR 0x68
// SPIClass hspi(HSPI);
// Adafruit_ADXL375 accel = Adafruit_ADXL375(accCS, &hspi, 12345);

MPU6500_WE MPU6500 = MPU6500_WE(&twi, MPU6500_ADDR);


uint32_t tImuTrigger = 0;
uint32_t tImuDelay = 50;  // milliseconds = 200 Hz

enum taskState { sRun,
                 sError,
                 sStart };
taskState sImu = sStart;
bool firstRunCore0 = true;

void core0setup(void) {  // a.k.a. setup
  xTaskCreatePinnedToCore(
    core0task,
    "core0task",
    10000,
    NULL,
    1,
    &hCore0task,
    0);
}

void core0task(void) {  // a.k.a. loop
  for (;;) {
    if (firstRunCore0) {
      twi.begin(i2cSDA, i2cSCL, 400000);
      firstRunCore0 = false;
    }
    // imu ....
    if (millis() - tImuTrigger > tImuDelay) {
      tImuTrigger = millis();
      switch (sImu) {
        case sRun:
          {
            // mérés végr.
            // range check
            break;
          }
        case sError:
          {
            break;
          }
        case sStart:
          {
            if (!MPU6500.init()) {
              s.println("IMU NOK");
              sImu = sError;
              tImuDelay = 10000;  // esetleg később megpróbálunk újrainicializálni
              sImu = sRun;
            } else {
              MPU6500.autoOffsets();
              MPU6500.setGyrDLPF(MPU6500_DLPF_6);
              MPU6500.setSampleRateDivider(5);
              MPU6500.setGyrRange(MPU9250_GYRO_RANGE_2000);
              MPU6500.setAccRange(MPU6500_ACC_RANGE_16G);
              MPU6500.enableAccDLPF(true);
              MPU6500.setAccDLPF(MPU6500_DLPF_6);
              s.println("IMU OK");
              sImu = sRun;
            }
            break;
          }
        default:
          {  //itt baj van....}
          }
      }
      status.imuOK = (sImu == sRun);  // && range check OK
    }
    // bmp...
  }

  // SwitchStatesIMU();
  // DoYourMagicIMU();
  // SwitchStatesBME();
  // DoYourMagicBME();
