//sdfsdf
// incl:
// DS18B20
// lora
// gps
// intercom
// dc_motor
// bipolar_stepper
// eeprom
// light sensor
// motor microswitch


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


TaskHandle_t hCore1task;

uint32_t tLoraTrigger = 0;
uint32_t tLoraDelay = 500;

taskState sLora = sStart;
bool firstRunCore1 = true;

void core1task(void* parameter);

void core1setup(void) {  // a.k.a. setup
  xTaskCreatePinnedToCore(
    core1task,
    "core1task",
    10000,
    NULL,
    0,
    &hCore1task,
    1);
}


void core1task(void* parameter) {  // a.k.a. loop
  for (;;) {
    if (firstRunCore1) {
      firstRunCore1 = false;
    }
    // lora ....
    if (millis() - tLoraTrigger > tLoraDelay) {
      tLoraTrigger = millis();
      switch (sLora) {
        case sRun:
          {
            break;
          }
        case sError:
          {
            break;
          }
        case sStart:
          {
            break;
          }
        default:
          {  //itt baj van....}
          }
      }
    }
  }
}
