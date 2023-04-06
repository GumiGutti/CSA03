#include "Arduino.h"
#include <HardwareSerial.h>
#include "cansat03.h"

#define CORE0TASKPRIO 3
#define CORE1TASKPRIO 3

HardwareSerial s(0);     // use UART0 serial debug or unit A-B intercom
HardwareSerial lora(1);  // use UART1
HardwareSerial gps(2);   // use UART2

#include "core0.h"
#include "core1.h"


void setup() {

  s.begin(115200);
  s.println();
  s.println("CANSAT boot....");

  xTemp = xSemaphoreCreateMutex();
  xSemaphoreGive((xTemp));


  SemaphoreHandle_t xTemp;      // temperature, calculated, fusion: DS, BME, BMP, 6500
SemaphoreHandle_t xPressure;  // calculated, fusion: BME, BMP
SemaphoreHandle_t xHumidity;  // measured (BME)
SemaphoreHandle_t xImuok; //
SemaphoreHandle_t xBmpok; //
SemaphoreHandle_t xBmeok; //
SemaphoreHandle_t xAdxlok; //
SemaphoreHandle_t xInaok; //
SemaphoreHandle_t xDsok; //
SemaphoreHandle_t xPosx; //
SemaphoreHandle_t xPosy; //
SemaphoreHandle_t xPosz; //
SemaphoreHandle_t xQa; //
SemaphoreHandle_t xQi; //
SemaphoreHandle_t xQj; //
SemaphoreHandle_t xQk; //
SemaphoreHandle_t xLat; //
SemaphoreHandle_t xLon; //
SemaphoreHandle_t xAlt; //

  core0setup();
  core1setup();

  while (1) vTaskDelay(1);
}

void loop() {
  // empty on purpose
}