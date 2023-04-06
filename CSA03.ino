#include "Arduino.h"
#include <HardwareSerial.h>
#include "cansat03.h"
HardwareSerial s(0);     // use UART0 serial debug or unit A-B intercom
HardwareSerial lora(1);  // use UART1
HardwareSerial gps(2);   // use UART2
#define CORE0TASKPRIO 3
#define CORE1TASKPRIO 3
#include "core0.h"
#include "core1.h"



void setup() {

  s.begin(115200);
  s.println();
  s.println("CANSAT boot....");

  core0setup();
  core1setup();

  xTemp = xSemaphoreCreateMutex();  xSemaphoreGive((xTemp));
  xPressure = xSemaphoreCreateMutex();  xSemaphoreGive((xPressure));
  xHumidity = xSemaphoreCreateMutex();  xSemaphoreGive((xHumidity));
  xImuok = xSemaphoreCreateMutex();  xSemaphoreGive((xImuok));
  xBmpok = xSemaphoreCreateMutex();  xSemaphoreGive((xBmpok));
  xBmeok = xSemaphoreCreateMutex();  xSemaphoreGive((xBmeok));
  xAdxlok = xSemaphoreCreateMutex();  xSemaphoreGive((xAdxlok));
  xInaok = xSemaphoreCreateMutex();  xSemaphoreGive((xInaok));
  xDsok = xSemaphoreCreateMutex();  xSemaphoreGive((xDsok));
  xPosx = xSemaphoreCreateMutex();  xSemaphoreGive((xPosx));
  xPosy = xSemaphoreCreateMutex();  xSemaphoreGive((xPosy));
  xPosz = xSemaphoreCreateMutex();  xSemaphoreGive((xPosz));
  xQa = xSemaphoreCreateMutex();  xSemaphoreGive((xQa));
  xQi = xSemaphoreCreateMutex();  xSemaphoreGive((xQi));
  xQj = xSemaphoreCreateMutex();  xSemaphoreGive((xQj));
  xQk = xSemaphoreCreateMutex();  xSemaphoreGive((xQk));
  xLat = xSemaphoreCreateMutex();  xSemaphoreGive((xLat));
  xLon = xSemaphoreCreateMutex();  xSemaphoreGive((xLon));
  xAlt= xSemaphoreCreateMutex();  xSemaphoreGive((xAlt));
  xGpsok= xSemaphoreCreateMutex();  xSemaphoreGive((xGpsok));
}

void loop() {
  vTaskDelay(1);
}