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
// #include <bme280.h>
// #include <Adafruit_INA219.h>

/*
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif */

Adafruit_BMP280 bmp;
TwoWire twi(0);
OneWire ds(ds18Pin);
//MPU6050 mpu;
//MPU6050 accelgyro;
//Adafruit_BME280 bme;

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


enum taskState { sRun,
                 sError,
                 sStart };
taskState sImu = sStart;
taskState sBME = sStart;
taskState sDallas = sStart;
taskState sBMP = sStart;
bool firstRunCore0 = true;

void core0task(void* parameter);

void setupDS(){
  ds.write(0xcc);
  ds.write(0x4e, 1);
  ds.write(0x7f, 1);
  ds.write(0xfc, 1);
  ds.write(0x3f, 0);
  sDallas = sRun;
}

void BMPsetup(){
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin(0x76, 0x58);
  if (!status) {
    s.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    s.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    s.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    s.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    s.print("        ID of 0x60 represents a BME 280.\n");
    s.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  sBMP = sRun;
}

void BMPdata(){
  s.print(F("Temperature = "));
  s.print(bmp.readTemperature());
  s.println(" *C");

  s.print(F("Pressure = "));
  s.print(bmp.readPressure());
  s.println(" Pa");

  s.print(F("Approx altitude = "));
  s.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  s.println(" m");
}

void dallas(){
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
}
/*
void setupBME() {
  unsigned status;
  //status = bme.begin(0x76, &twi); 
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
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin(21, 22);
      //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  mpu.initialize();
  accelgyro.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  //mpu.setFullScaleGyroRange(3);   //set the gyro range to 2000
  //mpu.setFullScaleAccelRange(3);  //set the accelerometer sensibilty to 16g
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  devStatus = mpu.dmpInitialize();
  //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  if (devStatus == 0) {  //0 ha jó, ha 1 baj van
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    //mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    s.println("DMP müksz");
  } else {
    sImu = sError;
    tImuDelay = 10000;  // esetleg később megpróbálunk újrainicializálni
    s.println("DMP nem müksz");
    s.println(devStatus); //1- initial memory load failed, 2- DMP conf. updates failed
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
*/
void core0setup() {  // a.k.a. setup
  xTaskCreatePinnedToCore(
    core0task,
    "core0task",
    10000,
    NULL,
    0,
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
    // ds  ....
    
    if (millis() - tDStrigger > tDSdelay) {
      tDStrigger = millis();
      
      switch (sDallas) {
        case sRun:
          {
            dallas();
            break;
          }
        case sError:
          {
            break;
          }
        case sStart:
          {
            setupDS();
            break;
          }
        default:
          {  //itt baj van....}
          }
      }
    }
    
    // bmp ....
    
    if (millis() - tBMPtrigger > tBMPdelay) {
      tBMPtrigger = millis();
      
      switch (sBMP) {
        case sRun:
          {
            BMPdata();
            break;
          }
        case sError:
          {
            break;
          }
        case sStart:
          {
            BMPsetup();
            break;
          }
        default:
          {  //itt baj van....}
          }
      }
    }
  }
}
