const byte bufSize = 254;  // ne vedd nagyobbra, kilógunk máshol!
char buf[bufSize + 1];
char txBuf[bufSize + 1];
char rxBuf[bufSize + 1];
byte txBufIdx = 0;
byte rxBufIdx = 0;
byte chk;
uint32_t tLastLoraRx = 0;
bool loraParse = false;

char getRadio(uint32_t to) {
  uint32_t start = micros();
  char b = 0;
  uint8_t u;
  // s.printf("%08d GET start\n", micros() / 1000);
  // s.printf("GET ");
  while (micros() - start < to && b < bufSize) {
    if (lora.available()) {
      buf[b++] = (byte)lora.read();
      // s.printf("%#02x  ", buf[b - 1]);
      if (buf[b - 1] == '\r') {
        // s.print("|CR|");
      } else if (buf[b - 1] != '\n') {
        // s.write(buf[b - 1]);
      } else {
        // s.println("|LF|");
        // s.println("");
        break;
      }
    }
  }
  buf[b] = '\0';
  // s.println();
  // if (b == 0)
  //   s.printf("%08d*** Timeout\n\n", micros() / 1000);
  // else
  //   s.printf("%08d ANS %s\n", micros() / 1000, buf);
  return (b);
}

char putRadio(const char *cmd, uint32_t to) {
  lora.println(cmd);
  // s.println(cmd);
  // s.printf("%08d PUT %s\r\n", micros() / 1000, cmd);
  char b = getRadio(to);
  // if (b == 0) s.println(cmd);
  return (b);
}

void addToTxBuff(char *karcsi) {  // egy byte két helyiértékű hex karaktereit beteszi a Tx bufferba, buffert nul karakterrel zárja
  byte karcsIdx = 0;              // karcsiban van a két hex karakter
  while (txBufIdx < bufSize && karcsi[karcsIdx]) {
    txBuf[txBufIdx++] = karcsi[karcsIdx++];
  }
  // *** BUG *** too long message not handled
  txBuf[txBufIdx] = '\0';
}

void addByteToTxBuff(char c) {
  if (txBufIdx < bufSize)
    txBuf[txBufIdx++] = c / 16 > 9 ? c / 16 + 87 : c / 16 + 48;
  if (txBufIdx < bufSize)
    txBuf[txBufIdx++] = c % 16 > 9 ? c % 16 + 87 : c % 16 + 48;
  txBuf[txBufIdx] = '\0';
  chk ^= c;
}

void loraACK(uint32_t ID) {
}
