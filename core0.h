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
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_ADXL375.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
// #include <bme280.h>
// #include <Adafruit_INA219.h>

TwoWire twi(0);
// #define MPU6500_ADDR 0x68
// SPIClass hspi(HSPI);
// Adafruit_ADXL375 accel = Adafruit_ADXL375(accCS, &hspi, 12345);

MPU6050 mpu;
MPU6050 accelgyro;
Adafruit_BME280 bme;

bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
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


volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high

uint32_t tImuTrigger = 0;
uint32_t tImuDelay = 50;  // milliseconds = 200 Hz

void ICACHE_RAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}

enum taskState { sRun,
                 sError,
                 sStart };
taskState sImu = sStart;
taskState sBME = sStart;
bool firstRunCore0 = true;

void core0task(void* parameter);

void setupBME() {
  unsigned status;
  status = bme.begin(0x76, &twi); 
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x");
    Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  else {
    sBME = sError;
    tImuDelay = 10000;  // esetleg később megpróbálunk újrainicializálni
    s.println("BME nem müksz");
  }
}

void setupIMU() {
  mpu.initialize();
  accelgyro.initialize();
  mpu.setFullScaleGyroRange(3);   //set the gyro range to 2000
  mpu.setFullScaleAccelRange(3);  //set the accelerometer sensibilty to 16g
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  devStatus = mpu.dmpInitialize();
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  if (devStatus == 0) {  //0 ha jó, ha 1 baj van
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    s.println("DMP müksz");
  } else {
    sImu = sError;
    tImuDelay = 10000;  // esetleg később megpróbálunk újrainicializálni
    s.println("DMP nem müksz");
  }
  s.println("IMU OK");
  sImu = sRun;
}

void GetAcceleration() {
  accelgyro.getAcceleration(&ax, &ay, &az);
  float accelX = (float)ax / 2048;
  float accelY = (float)ay / 2048;
  float accelZ = (float)az / 2048;
}

void GetOrientation() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  float yaw = (ypr[0] * 180 / M_PI);
  float pitch = (ypr[1] * 180 / M_PI);
  float roll = (ypr[2] * 180 / M_PI);
}

void GetBMEdata() {
  float temp = bme.readTemperature();
  float pressure = bme.readPressure() / 100;
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float humidity = bme.readHumidity();
}

void core0setup() {  // a.k.a. setup
  xTaskCreatePinnedToCore(
    core0task,
    "core0task",
    10000,
    NULL,
    1,
    &hCore0task,
    0);
}

void core0task() {  // a.k.a. loop
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
            setupBME();
            break;
          }
        default:
          {  //itt baj van....}
          }
      }
      status.imuOK = (sImu == sRun);  // && range check OK
    }
  }
}
