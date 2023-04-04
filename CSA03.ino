// incl core0
// incl core1

// void setup() {
// core0 setup
// core1 setup
// }

// void loop() {
// üres
// }
#include "Arduino.h"
#include <HardwareSerial.h>
#include "cansat03.h"
HardwareSerial s(0);     // use UART0 serial debug or unit A-B intercom
HardwareSerial lora(1);  // use UART1 
HardwareSerial gps(2);   // use UART2 
#include "core0.h"
#include "core1.h"


void setup(void) {

  s.begin(115200);
  s.println();
  s.println("CANSAT boot....");

  core0setup();
  core1setup();

};

void loop(void){
  // empty on purpose
};