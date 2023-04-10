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
#include <math.h>

TaskHandle_t hCore1task;

uint32_t tLoraTrigger = 0;
uint32_t tLoraDelay = 500;
taskState sLora = sStart;
char c[200];
#define gpsBufSize 512
char gpsBuf[gpsBufSize];
uint32_t gpsBufIdx = 0;
byte lockState = 0;
uint32_t tLastGps;
bool gpsParse = false;
double gpsTim, gpsLat, gpsLon, gpsAlt;
uint32_t tLastRxLora;
byte gsTO;
byte commLayer;
byte sfNow;
uint16_t newFreq, freqNow;
float lat0 = 47.430336;
float lon0 = 19.013742;
float energy = 0.0;
int16_t rssi, snr;

#include "uart.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
      bufferOK = 1;
      dsOK = 0;
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


      gps.begin(9600, SERIAL_8N1, gpsRx, gpsTx);
      size_t dummy = gps.setRxBufferSize(2048);
      lora.begin(115200, SERIAL_8N1, loraRx, loraTx);
      dummy = lora.setRxBufferSize(2048);
      tLastRxLora = millis();
      gsTO = 8;
      commLayer = 0;
      sfNow = 10;
      freqNow = 8667;  // kHz - EEPROM-ból
      newFreq = freqNow;
    }
    // lora ....
    // ***
    missionPhase = 0;
    if (millis() - tLoraTrigger > tLoraDelay) {
      tLoraTrigger = millis();
      if (1) {
        // vault handling: window << + >> data (mutex) | sdbuffer töltés, sdbuffer size check
        // update data from window
        //
        // core0 >> core1
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
      }
      ringSend(256);
      uint16_t sta = (gpsNoTO << 0) | (gpsLockOK << 1) | (gpsTimeOK << 2) | (bmpOK << 3) | (bmeOK << 4)
                     | (adxlOK << 5) | (imuOK << 6) | (inaOK << 7) | (camOK << 8)
                     | (sdcardOK << 9) | (bufferOK << 10) | (dsOK << 11) | (dgpsOK << 12) | (missionPhase << 13);
      if (1) {
        sprintf(c, "%10.3lf", tLoraTrigger / 1000.0);
        addSD(c);

        sprintf(c, " %1d", missionPhase);
        addSD(c);

        addSD(" ");
        for (int i = 0; i < 13; i++) {
          sprintf(c, " %d", (sta >> i) & 1);
          addSD(c);
        }
        sprintf(c, " %6.1f %6.1f %6.1f %+8.3f %+8.3f %+8.3f %+8.3f", posx, posy, posz, qa, qi, qj, qk);
        addSD(c);
        sprintf(c, " %+04d %+4d", rssi, snr);
        addSD(c);
        sprintf(c, " %8.1f %6.2f", pressure, temp);
        addSD(c);
        sprintf(c, " %8.5f %8.5f", lat, lon);
        addSD(c);
        sprintf(c, " %8.3f", alt);
        addSD(c);
        sprintf(c, " %6.2f", current);
        addSD(c);
        sprintf(c, " %6.2f", voltage);
        addSD(c);
        sprintf(c, " %6.1f", energy);
        addSD(c);

        // sprintf(c, "%d", (byte));
        // addSD(c);
        addSD("\n");
        // end of window exchange
      }
      switch (sLora) {
        case sRun:
          {  // tchk start
            if (s.available()) {
              switch (s.read()) {
                case '0': gsTO = 0; break;
                case '1': gsTO = 4; break;
                case '2': gsTO = 8; break;
                case '3': newFreq = 8681; break;
                case '4': newFreq = 8690; break;
                default: s.println("nemáááá...");
              }
            }
            //////////////////////////////////////////////////////////////////////////////////////
            while (lora.available()) {
              char c = lora.read();
              if (rxBufIdx > bufSize - 1) {
                rxBufIdx = 0;
              }
              rxBuf[rxBufIdx] = c;
              if (c == LF) {
                loraParse = true;
              } else {
                rxBufIdx++;
              }
            }
            putRadio("radio rxstop", 100000);
            // lora PARSE
            if (loraParse) {
              loraParse = false;
              if (rxBufIdx > 12) {  // van levágandó eleje
                rxBuf[rxBufIdx] = 0;
                char kisbuf[3] = "xx";
                byte hexNo = (rxBufIdx - 10) / 2;
                chk = 0;
                int r;
                for (r = 0; r < hexNo; r++) {
                  kisbuf[0] = rxBuf[r * 2 + 9];
                  kisbuf[1] = rxBuf[r * 2 + 10];
                  buf[r] = (byte)strtol(kisbuf, 0, 16);
                  if (r < hexNo - 1) chk ^= buf[r];
                  // s.printf("0x%02x\t", buf[r]);
                }
                r--;
                // s.println("");
                // s.println(rxBuf); //rádió kiíratás
                if (hexNo > 1) {
                  if (chk == buf[r]) {
                    // s.println("CHK OK");
                    tLastRxLora = millis();
                    // ez egy valid üzenet
                    if (1) {
                      if (gsTO > 0) gsTO--;
                      if (buf[0] == 5) {
                        loraACK((uint32_t)buf[1] << 16 | (uint32_t)buf[2] << 8 | (uint32_t)buf[3]);
                      }
                      if (buf[0] == 7) {
                        loraACK((uint32_t)buf[1] << 16 | (uint32_t)buf[2] << 8 | (uint32_t)buf[3]);
                        newFreq = (uint16_t)buf[4] << 8 | (uint16_t)buf[5];
                      }
                    }
                  } else {
                    s.printf("CHK NOK 0x%02x expected, got 0x%02x\n", chk, buf[r]);
                  }
                }
                rxBufIdx = 0;
                int i = 0;
                int i2 = 0;
              } else {
                rxBufIdx = 0;
              }
            }
            putRadio("radio get snr", 20000);
            snr = (int16_t)atoi(buf);
            putRadio("radio get pktrssi", 20000);
            rssi = (int16_t)atoi(buf);

            //////////////////////////////////////////////////////////////////////////////////////
            if (newFreq != freqNow) {
              char fc[40];
              if ((newFreq > 8670) && (newFreq < 8700)) {
                sprintf(fc, "radio set freq %4d00000", newFreq);
                // s.println(fc);
                putRadio(fc, 10000);
                // s.println(buf);
                putRadio("radio get freq", 10000);
                // s.println(buf);
                freqNow = newFreq;
              }
            }

            if (millis() - tLastRxLora > tLoraDelay) {
              tLastRxLora = millis();
              if (gsTO < 8) gsTO++;
            }
            if (gsTO == 8 && sfNow != 10) {
              s.println("SF to 10");
              putRadio("radio set sf sf10", 20000);
              while (lora.available()) lora.read();
              // s.println(buf);
              sfNow = 10;
              commLayer = 0;
            }
            if (gsTO == 4 && sfNow != 7) {
              s.println("SF to 7");
              putRadio("radio set sf sf7", 20000);
              while (lora.available()) lora.read();
              // s.println(buf);
              sfNow = 7;
              commLayer = 2;
            }
            txBufIdx = 0;
            addToTxBuff("radio tx ");
            // fill Tx buffer
            // msg class and length
            chk = 0x00;
            if (commLayer > 0)
              addByteToTxBuff(32 + gsTO);
            else
              addByteToTxBuff(9 + gsTO);
            uint32_t txID = millis();
            if (commLayer > 0) {
              // status field
              addByteToTxBuff(sta / 16);
              addByteToTxBuff(sta % 16);
            }
            // message unique ID
            addByteToTxBuff((char)(txID >> 16) & 0xff);
            addByteToTxBuff((char)(txID >> 8) & 0xff);
            addByteToTxBuff((char)(txID >> 0) & 0xff);
            if (commLayer > 0) {
              // relative position to GS
              addByteToTxBuff(((int16_t)(posx * 2)) >> 8);
              addByteToTxBuff(((int16_t)(posx * 2)) & 0xff);
              addByteToTxBuff(((int16_t)(posy * 2)) >> 8);
              addByteToTxBuff(((int16_t)(posy * 2)) & 0xff);
              addByteToTxBuff(((int16_t)(posz * 2)) >> 8);
              addByteToTxBuff(((int16_t)(posz * 2)) & 0xff);

              // quaternion
              addByteToTxBuff((signed char)(qa * 128));
              addByteToTxBuff((signed char)(qi * 128));
              addByteToTxBuff((signed char)(qj * 128));
              addByteToTxBuff((signed char)(qk * 128));

              // RSSI
              addByteToTxBuff((char)abs(rssi));

              // SNR
              addByteToTxBuff((char)snr);

              // absolute pressure
              addByteToTxBuff(((char)(pressure - 60000.0)) >> 8);
              addByteToTxBuff(((char)(pressure - 60000.0)) & 0xff);

              // temperature
              uint16_t tem = (uint16_t)((273.15 + temp) * 100.0);
              addByteToTxBuff((char)(tem >> 8) & 0xff);
              addByteToTxBuff((char)(tem >> 0) & 0xff);
            }
            // GPS lat, lon, alt
            int16_t lai = (lat - lat0) * 500000;
            addByteToTxBuff(lai >> 8);
            addByteToTxBuff(lai & 0xff);
            int16_t loi = (lon - lon0) * 500000;
            addByteToTxBuff(loi >> 8);
            addByteToTxBuff(loi & 0xff);
            if (commLayer > 0) {
              addByteToTxBuff(((uint16_t)(alt * 10.0)) >> 8);
              addByteToTxBuff(((uint16_t)(alt * 10.0)) & 0xff);

              // current
              addByteToTxBuff((char)(current / 4.0));

              // energy %
              addByteToTxBuff((char)(energy * 2));

              // voltage
              addByteToTxBuff((char)((voltage-2.0)*100.0));
            }
            addByteToTxBuff(chk);
            addToTxBuff(" 1");
            putRadio(txBuf, 100000);  // ok
            getRadio(500000);         // radio tx ok
            getRadio(100000);         // # packet sent
            while (lora.available()) lora.read();
            putRadio("radio rx 0", 100000);  // ok
            // s.printf("%s @sf %d    @missed %d\n", txBuf, sfNow, gsTO); //rádió kiíratás
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
            delay(5);
            while (lora.available()) {  // empty rx buffer
              s.write((byte)lora.read());
              delay(1);
            }
            putRadio("radio set sf sf10", 10000);
            char fc[40];
            sprintf(fc, "radio set freq %4d00000", freqNow);
            s.println(fc);
            putRadio(fc, 10000);
            // s.println(buf);
            putRadio("radio get freq", 50000);
            // s.println(buf);
            putRadio("radio set bw 250", 10000);
            putRadio("radio set rxbw 250", 50000);
            putRadio("radio set cr 4/5", 10000);
            byte b = putRadio("radio set crc off", 10000);
            if (b == 0) {
              s.println("LOR NOK");
            } else {
              s.println("LOR OK");
            }
            tLoraTrigger = millis();
            sLora = sRun;
            break;
          }
        default:
          {  //itt baj van....}
          }
      }

      while (gps.available()) {
        char c = gps.read();
        if (char(c) == '$' || gpsBufIdx > gpsBufSize - 1) {
          gpsBufIdx = 0;
        }
        gpsBuf[gpsBufIdx] = c;
        if (c == LF) {
          tLastGps = millis();
          gpsParse = true;
        } else
          gpsBufIdx++;
        if (gpsParse) {
          gpsParse = false;
          gpsBuf[gpsBufIdx] = 0;
          String m(gpsBuf);
          // s.println(m);
          gpsBufIdx = 0;
          int i = 0;
          int i2 = 0;
          while (i2 >= 0) {
            double dbl;
            i2 = m.indexOf(',');
            if (i2 >= 0) {
              // 0123456789012345678901234567890123456789012345678901234567890123456789
              // $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,X,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh

              // s.printf("%3d : %s\n", i, m.substring(0, i2));
              String z = m.substring(0, i2);
              if (i == 0 && z != "$GPGGA") {
                i2 = -1;
              }
              if (i == 1) {
                char timStr[z.length() + 1];
                z.toCharArray(timStr, z.length());
                gpsTim = atof(timStr);
                gpsTimeOK = gpsTim == 0.0 ? 0 : 1;
              }
              if (i == 2) {
                char latStr[z.length() + 1];
                z.toCharArray(latStr, z.length());
                dbl = atof(latStr) / 100.0;
                double la = fmod(dbl, 1.0);
                lat = (dbl - la) + la / 6.0 * 10.0;
              }
              if (i == 4) {
                char lonStr[z.length() + 1];
                z.toCharArray(lonStr, z.length());
                dbl = atof(lonStr) / 100.0;
                double lo = fmod(dbl, 1.0);
                lon = (dbl - lo) + lo / 6.0 * 10.0;
              }
              if (i == 6) {
                // s.print("lockState as String <");
                // s.print(z);
                // s.print("> lockState as byte <");
                lockState = (byte)z.toInt();
                if (lockState > 0) {
                  gpsLockOK = 1;
                  dgpsOK = lockState == 2;
                } else {
                  gpsLockOK = 0;
                  dgpsOK = 0;
                }
              }
              if (i == 9) {
                char altStr[z.length() + 1];
                z.toCharArray(altStr, z.length());
                alt = atof(altStr);
              }
            }
            i++;
            m.remove(0, i2 + 1);
          }
        }  // end of gpsParse
      }    // gps rx buffer empty
      gpsNoTO = (millis() - tLastGps > 1100) ? 0 : 1;
      // s.printf("Time %12.2f\tLat %9.6f\t Lon %9.6f\tAlt %5.2f\tTime %1d\tLock %1d\tDGPS %1d\n", gpsTim, lat, lon, alt, gpsTimeOK, gpsLockOK, dgpsOK);
    }  // end of loratrigger
    vTaskDelay(1);
  }  // end of loop
  // *** heap_caps_get_free_size(MALLOC_CAP_8BIT);
}
