// core0
// integrálás
// fúzió
// mutex
// bme,bmp,mpu,adxl,ds18,

// core1
// dc_motor
// bipolar_stepper
// eeprom struktúra
// light sensor
// motor microswitch
// core0 csere struktúra
// core0 mutex
// SD data prep
// cam transfer buffer
// cam transer protocol
// cam control
// mission phase calc
// ToA calc
// RSSI / SNR
// GPS task,  parse, flags
// lora regular
// lora backup layers
// flight control
// heap check - heap_caps_get_free_size(MALLOC_CAP_8BIT);




// időzítés definíciók

// osztályok
// globálok


// segédfüggvények

// task:
// for(;;):

// lora state machine (sm)
// intercom parser
// gps parser

// #include <EEPROM.h>
// #include "cansat03.h"
#include "lora.h"

TaskHandle_t hCore1task;

uint32_t tLoraTrigger = 0;
uint32_t tLoraDelay = 500;
taskState sLora = sStart;
const uint32_t ringSize = 512;
byte sdring[ringSize];
uint32_t sdhead = 0;
uint32_t sdtail = 0;
char c[100];
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte tikk;

bool firstRunCore1 = true;

void addSD(char* karcsi) {
  s.write(karcsi);
  // return true;
}
void core1task(void* parameter);

void core1setup(void) {  // a.k.a. setup
  xTaskCreatePinnedToCore(
    core1task,
    "core1task",
    10000,
    NULL,
    CORE1TASKPRIO,
    &hCore1task,
    1);
}


void core1task(void* parameter) {
  for (;;) {  // a.k.a. loop
    if (firstRunCore1) {
      firstRunCore1 = false;
      pinMode(loraRst, OUTPUT);
      pinMode(pinPin, OUTPUT);
      pinMode(pinPon, OUTPUT);
      gpsNoTO = 0;
      gpsLockOK = 0;
      gpsTimeOK = 0;
      bmpOK = 0;
      bmeOK = 0;
      adxlOK = 0;
      imuOK = 0;
      inaOK = 0;
      camOK = 0;
      sdcardOK = 0;
      bufferOK = 0;
      dsOK = 0;
      lightOK = 0;
      missionPhase = 0;


      temp = 1.1;
      pressure = 1.1;
      humidity = 1.1;
      posx = 1.1;
      posy = 1.1;
      posz = 1.1;
      qa = 1.1;
      qi = 1.1;
      qj = 1.1;
      qk = 1.1;
      lat = 1.1;
      current = 1.1;
      voltage = 1.1;

      lora.begin(115200, SERIAL_8N1, loraRx, loraTx);
      size_t dummy = lora.setRxBufferSize(2048);
    }
    // lora ....
    // ***
    missionPhase = tikk & 7;
    if (millis() - tLoraTrigger > tLoraDelay) {
      tLoraTrigger = millis();
      // vault handling: window << + >> data (mutex) | sdbuffer töltés, sdbuffer size check
      //
      // update data from window
      // if (1) {
      //   // core0 >> core1
      xSemaphoreTake(xBmpok, portMAX_DELAY);
      bmpOK = Bmpok;
      xSemaphoreGive(xBmpok);

      xSemaphoreTake(xBmeok, portMAX_DELAY);
      bmeOK = Bmeok;
      xSemaphoreGive(xBmeok);

      xSemaphoreTake(xAdxlok, portMAX_DELAY);
      adxlOK = Adxlok;
      xSemaphoreGive(xAdxlok);

      xSemaphoreTake(xImuok, portMAX_DELAY);
      imuOK = Imuok;
      xSemaphoreGive(xImuok);

      xSemaphoreTake(xInaok, portMAX_DELAY);
      inaOK = Inaok;
      xSemaphoreGive(xInaok);

      xSemaphoreTake(xTemp, portMAX_DELAY);
      temp = Temp;
      xSemaphoreGive(xTemp);

      xSemaphoreTake(xPressure, portMAX_DELAY);
      pressure = Pressure;
      xSemaphoreGive(xPressure);

      xSemaphoreTake(xHumidity, portMAX_DELAY);
      humidity = Humidity;
      xSemaphoreGive(xHumidity);

      xSemaphoreTake(xPosx, portMAX_DELAY);
      posx = Posx;
      xSemaphoreGive(xPosx);

      xSemaphoreTake(xPosy, portMAX_DELAY);
      posy = Posy;
      xSemaphoreGive(xPosy);

      xSemaphoreTake(xPosz, portMAX_DELAY);
      posz = Posz;
      xSemaphoreGive(xPosz);

      xSemaphoreTake(xQa, portMAX_DELAY);
      qa = Qa;
      xSemaphoreGive(xQa);

      xSemaphoreTake(xQi, portMAX_DELAY);
      qi = Qi;
      xSemaphoreGive(xQi);

      xSemaphoreTake(xQj, portMAX_DELAY);
      qj = Qj;
      xSemaphoreGive(xQj);

      xSemaphoreTake(xQk, portMAX_DELAY);
      qk = Qk;
      xSemaphoreGive(xQk);

      xSemaphoreTake(xCurrent, portMAX_DELAY);
      current = Current;
      xSemaphoreGive(xCurrent);

      xSemaphoreTake(xVoltage, portMAX_DELAY);
      voltage = Voltage;
      xSemaphoreGive(xVoltage);

      // core1 >> core0
      xSemaphoreTake(xGpsok, portMAX_DELAY);
      Gpsok = gpsNoTO & gpsLockOK;
      xSemaphoreGive(xGpsok);

      xSemaphoreTake(xMissionphase, portMAX_DELAY);
      Missionphase = missionPhase;
      xSemaphoreGive(xMissionphase);
      // }
      uint16_t sta = (gpsNoTO << 0) | (gpsLockOK << 1) | (gpsTimeOK << 2) | (bmpOK << 3) | (bmeOK << 4)
                     | (adxlOK << 5) | (imuOK << 6) | (inaOK << 7) | (camOK << 8)
                     | (sdcardOK << 9) | (bufferOK << 10) | (dsOK << 11) | (lightOK << 12) | (missionPhase << 13);
      if (1) {
        sprintf(c, "%10.3lf", tLoraTrigger / 1000.0);
        addSD(c);
        addSD("\t");
        for (int i = 0; i < 13; i++) {
          sprintf(c, " %d", (sta >> i) & 1);
          addSD(c);
        }
        // sprintf(c, "%d", (byte));
        // addSD(c);
        sprintf(c, "\t%d", missionPhase);
        addSD(c);
        addSD("\n");
      }  // end of window exchange
      switch (sLora) {
        case sRun:
          {  // tchk start
            digitalWrite(pinPin, 1);
            tikk++;
            txBufIdx = 0;
            addToTxBuff("radio tx ");
            // fill Tx buffer
            // msg class and length
            chk = 0x00;
            addByteToTxBuff(32);
            uint32_t txID = millis();
            // status field
            addByteToTxBuff(1 << (sta / 16));
            addByteToTxBuff(1 << (sta % 16));

            // message unique ID
            addByteToTxBuff((char)(txID >> 16) & 0xff);
            addByteToTxBuff((char)(txID >> 8) & 0xff);
            addByteToTxBuff((char)(txID >> 0) & 0xff);

            // relative position to GS
            int16_t messze = 128 - abs((int16_t)tikk - 128);
            addByteToTxBuff((char)(messze >> 8) & 0xff);
            addByteToTxBuff((char)(messze >> 0) & 0xff);
            addByteToTxBuff((char)(messze >> 8) & 0xff);
            addByteToTxBuff((char)(messze >> 0) & 0xff);
            addByteToTxBuff((char)(messze >> 8) & 0xff);
            addByteToTxBuff((char)(messze >> 0) & 0xff);

            // quaternion
            addByteToTxBuff((char)(tikk));
            addByteToTxBuff((char)(tikk));
            addByteToTxBuff((char)(255 - tikk));
            addByteToTxBuff((char)(255 - tikk));

            // RSSI
            addByteToTxBuff((char)(tikk));

            // SNR
            addByteToTxBuff((char)(255 - tikk));

            // absolute pressure
            uint16_t nyomas = 37100;
            addByteToTxBuff((char)(nyomas >> 8) & 0xff);
            addByteToTxBuff((char)(nyomas >> 0) & 0xff);

            // temperature
            uint32_t tz = micros();

            uint16_t tem = (uint16_t)((273.15 + temp) * 100.0);
            // s.printf("Core0 Temperature %+6.2f\n", (tem / 100.0) - 273.15);
            addByteToTxBuff((char)(tem >> 8) & 0xff);
            addByteToTxBuff((char)(tem >> 0) & 0xff);

            // GPS lat, lon, alt
            addByteToTxBuff(tikk);
            addByteToTxBuff(tikk);
            addByteToTxBuff(tikk);
            addByteToTxBuff(tikk);
            addByteToTxBuff(tikk);
            addByteToTxBuff(tikk);

            // current
            addByteToTxBuff(45 + tikk / 64);

            // energy %
            addByteToTxBuff(200 - tikk / 16);

            // voltage
            addByteToTxBuff(170 - tikk / 32);
            addByteToTxBuff(chk);

            addToTxBuff(" 1");
            putRadio(txBuf, 100000);  // ok
            digitalWrite(pinPon, 1);
            getRadio(100000);         // radio tx ok
            digitalWrite(pinPon, 0);  // radio time "net" = 41 ms
            getRadio(100000);         // # packet sent
            digitalWrite(pinPin, 0);
            tLoraTrigger = millis();

            // tchk full loraTask = 56.2 ms
            break;
          }
        case sError:
          {
            break;
          }
        case sStart:
          {
            delay(300);
            digitalWrite(loraRst, 1);
            delay(5);
            digitalWrite(loraRst, 0);
            delay(5);
            digitalWrite(loraRst, 1);
            delay(1000);
            while (lora.available()) {  // empty rx buffer
              byte c = (byte)lora.read();
              delay(1);
            }
            putRadio("sys reset", 1000000);
            while (lora.available()) {  // empty rx buffer
              byte c = (byte)lora.read();
              delay(1);
            }
            // putRadio("sys reset", 150000);
            putRadio("radio set sf sf7", 10000);
            putRadio("radio set freq 869100000", 10000);
            putRadio("radio set bw 250", 10000);
            putRadio("radio set rxbw 250", 10000);
            putRadio("radio set cr 4/5", 10000);
            byte b = putRadio("radio set crc off", 10000);
            if (b == 0) {
              s.println("LOR NOK");
            } else {
              s.println("LOR OK");
            }
            sLora = sRun;
            break;
          }
        default:
          {  //itt baj van....}
          }
      }
    }
    // heap_caps_get_free_size(MALLOC_CAP_8BIT);

    vTaskDelay(1);
  }
}
