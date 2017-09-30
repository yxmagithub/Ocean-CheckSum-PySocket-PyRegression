// Implementation of Tecnadyne motor controller for Arduino.

bool debug = true; // Turn on to see hex representations of binary sent to motor controller.
#warning implmement

uint8_t getBCC(uint8_t *buf, size_t len) {
  uint8_t result = 0;
  for(int i = 0; i < len-1; ++i) {
    result += buf[i];
  }
  return (uint8_t)result;
}

void setSpeedPWM(uint8_t address, uint8_t dir, uint8_t PWM_duty) {
  #warning This function needs better handling of DLE stuffing (sec 3.11 of manual).
  // Command string.
  size_t len = 11;
  uint8_t buf[] = {0x10, 0x01, address, 0x41, 0x20, 0x20, dir, PWM_duty, 0x10, 0x03, 0x00};

  // Compute checksum.
  uint8_t bcc = getBCC(buf, len);
  buf[len-1] = bcc;

  // Handle DLE stuffing.
  if(PWM_duty == 0x10) {
    ++len;
    for(int i = len - 1; i >= 9; --i) {
      buf[i] = buf[i-1];
    }
    buf[8] = 0x10;
  }

  //Serial.write(buf, len);
  for(int i = 0; i < len; ++i) {
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.print("\n");

  Serial.write(buf, len);
  Serial.print("\n");
}


