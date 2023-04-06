// mutex struktúra
// pin def
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

SemaphoreHandle_t i2cSemaphore;
void createSemaphore(SemaphoreHandle_t &sem){
    &sem = xSemaphoreCreateMutex();
    xSemaphoreGive( ( sem) );
}

// Lock the variable indefinietly. ( wait for it to be accessible )
void lock(SemaphoreHandle_t &sem){
    xSemaphoreTake(sem, portMAX_DELAY);
}

// give back the semaphore.
void unlock(SemaphoreHandle_t &sem){
    xSemaphoreGive(sem);
}

SemaphoreHandle_t xTemp; // temperature, calculated, fusion
SemaphoreHandle_t xPressure; // calculated, fusion
// SemaphoreHandle_t ; //
// SemaphoreHandle_t ; //
// SemaphoreHandle_t ; //
// SemaphoreHandle_t ; //
// SemaphoreHandle_t ; //
// SemaphoreHandle_t ; //
// SemaphoreHandle_t ; //


// TO általában: TimeOut

typedef struct statusfield_t {
  bool gpsNoTO : 1;
  bool gpsLockOK : 1;
  bool gpsTimeOK : 1;
  bool bmpOK : 1;
  bool bmeOK : 1;
  bool adxlOK : 1;
  bool imuOK : 1;
  bool inaOK : 1;
  bool camOK : 1;
  bool sdcardOK : 1;
  bool bufferOK : 1;
  bool motorOK : 1;
  bool lightOK : 1;
  bool missionPhase : 3;
};

statusfield_t status;

union uStatusUnion {
  statusfield_t statusBits;
  char statusChars[2];
} uStatus;

#define loraRst 0    // OUTPUT ONLY!
#define GPIO01 1     // N/A
#define loraTx 2     // generic
#define GPIO03 3     // N/A
#define loraRx 4     // generic
#define pinPin 5      // generic
#define GPIO0 6      // N/A
#define GPIO0 7      // N/A
#define GPIO0 8      // N/A
#define GPIO0 9      // N/A
#define GPIO10 10    // N/A
#define GPIO11 11    // N/A
#define hspiMISO 12  // generic
#define hspiMOSI 13  // generic
#define hspiCLK 14   // generic
#define GPIO15 15    // generic
#define gpsRx 16     // generic
#define gpsTx 17     // generic
#define pinPon 18    // generic
#define GPIO19 19    // generic
#define GPIO20 20    // N/A
#define i2cSDA 21    // generic
#define i2cSCL 22    // generic
#define GPIO23 23    // generic
#define GPIO24 24    // N/A
#define GPIO25 25    // generic
#define accCS 26     // generic
#define ds18Pin 27   // generic
#define GPIO28 28    // N/A
#define GPIO29 29    // N/A
#define GPIO30 30    // N/A
#define GPIO31 31    // N/A
#define mot1CW 32    // generic
#define mot1CCW 33   // generic
#define GPIO34 34    // INPUT ONLY!
#define GPIO35 35    // INPUT ONLY!
#define GPIO36 36    // INPUT ONLY!
#define mot1SW 37    // N/A
#define GPIO38 38    // N/A
#define photoRes 39  // INPUT ONLY!

#define SEALEVELPRESSURE_HPA (1013.25)