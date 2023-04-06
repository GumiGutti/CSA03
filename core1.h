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
// am transfer buffer
// cam transer protocol
// cam control
// mission phase calc
// ToA calc
// RSSI / SNR
// GPS task,  parse, flags
// lora regular
// lora backup layers
// flight control




// időzítés definíciók

// osztályok
// globálok


// segédfüggvények

// task:
// for(;;):

// lora state machine (sm)
// intercom parser
// gps parser

// #include <OneWire.h>
// #include <DallasTemperature.h>
// #include <EEPROM.h>
#include "lora.h"
#include <OneWire.h>

TaskHandle_t hCore1task;
// OneWire ds(ds18Pin);

uint32_t tLoraTrigger = 0;
uint32_t tLoraDelay = 500;
taskState sLora = sStart;

// uint32_t tDallasTrigger = 0;
// uint32_t tDallasDelay = 100;  // 10Hz
// taskState sDallas = sStart;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t blink;
byte tikk;
float dsTemp;

bool firstRunCore1 = true;

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
      lora.begin(115200, SERIAL_8N1, loraRx, loraTx);
      size_t dummy = lora.setRxBufferSize(2048);
    }
    // lora ....
    if (millis() - tLoraTrigger > tLoraDelay) {
      tLoraTrigger = millis();
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
            uStatus.statusBits = status;

            // status field
            // addByteToTxBuff(uStatus.statusChars[1]);
            // addByteToTxBuff(uStatus.statusChars[0]);
            addByteToTxBuff(1 << (tikk / 16));
            addByteToTxBuff(1 << (15 - (tikk / 16)));

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

            xSemaphoreTake(xTemp, portMAX_DELAY);
            
            s.printf("Lock daley %d us - Temp %+06.2f fok\n", micros() - tz, Temp);
            uint16_t tem = (uint16_t)((273.15 + Temp) * 100.0);
            
            xSemaphoreGive(xTemp);
            
            s.printf("Core0 Temperature %+6.2f\n", (tem / 100.0)-273.15);
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

            // tchk full loraTask = 56.2 ms
            break;
          }
        case sError:
          {
            break;
          }
        case sStart:
          {


            digitalWrite(loraRst, 1);
            delay(5);
            digitalWrite(loraRst, 0);
            delay(5);
            digitalWrite(loraRst, 1);
            delay(240);
            while (lora.available()) {  // empty rx buffer
              byte c = (byte)lora.read();
              delay(1);
            }
            // putRadio("factoryRESET", 50000);
            putRadio("radio set sf sf7", 10000);
            putRadio("radio set freq 869100000", 10000);
            putRadio("radio set bw 250", 10000);
            putRadio("radio set rxbw 250", 10000);
            putRadio("radio set cr 4/5", 10000);
            byte b = putRadio("radio set crc off", 10000);
            // if (b == 0) {
            //   s.println("LOR NOK");
            // } else {
            //   s.println("LOR OK");
            // }





            sLora = sRun;
            break;
          }
        default:
          {  //itt baj van....}
          }
      }
    }
    // DS18B20
    // if (millis() - tDallasTrigger > tDallasDelay) {
    //   tDallasTrigger = millis();
    //   s.println("Dallas tick");
    //   switch (sDallas) {
    //     case sRun:
    //       {
    //         uint16_t result;
    //         byte data[2];
    //         ds.reset();
    //         ds.write(0xcc);
    //         ds.write(0xbe);
    //         data[0] = ds.read();
    //         data[1] = ds.read();
    //         result = ((uint16_t)data[1] << 8) | data[0];
    //         ds.reset();
    //         ds.write(0xcc);
    //         ds.write(0x44, 1);
    //         if (data[1] & 128) {
    //           dsTemp = (((~result) >> 2) + 1) / -4.0;
    //         } else {
    //           dsTemp = (result >> 2) / 4.0;
    //         }
    //         break;
    //       }
    //     case sError:
    //       {
    //         break;
    //       }
    //     case sStart:
    //       {
    //         ds.write(0xcc);
    //         ds.write(0x4e, 1);
    //         ds.write(0x7f, 1);
    //         ds.write(0xfc, 1);
    //         ds.write(0x3f, 0);
    //         sDallas = sRun;
    //         break;
    //       }
    //     default:
    //       {  //itt baj van....}
    //       }
    //   }
    // }
    vTaskDelay(1);
  }
}
