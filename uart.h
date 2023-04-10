const uint32_t ringSize = 4096;
byte sdring[ringSize];
uint32_t sdhead = 0;
uint32_t sdtail = 0;


uint32_t ringBytes(void) {
  return sdhead > sdtail ? sdhead - sdtail : sdhead + ringSize - sdtail;
}
uint32_t ringFree(void) {
  return ringSize - 1 - ringBytes();
}

void headPP() {
  sdhead++;
  if (sdhead == ringSize) sdhead = 0;
}

void tailPP() {
  sdtail++;
  if (sdtail == ringSize) sdtail = 0;
}

void ringSend(uint32_t chunkSize) {
  if (ringBytes() >= chunkSize) {
    s.write('0');
    while (chunkSize--) {
      s.write(sdring[sdtail]);
      tailPP();
    }
    s.write('1');
  }
}

void addSD(char* karcsi) {
  if (ringFree() > strlen(karcsi)) {
    while (*karcsi) {
      sdring[sdhead] = *karcsi;
      karcsi++;
      headPP();
    }
  } else
    bufferOK = 0;
}
