#include <Arduino.h>
#include "MFRC522.h"
#include <vector>

#define CS_PIN 17
#define MISO_PIN 16
#define MOSI_PIN 19
#define SCK_PIN 18

MFRC522 mfrc522(CS_PIN, UNUSED_PIN);
void setup() {
  Serial.begin(9600);
  SPI.begin();
  mfrc522.PCD_Init();

  pinMode(LED_BUILTIN, OUTPUT);
}

unsigned long blinkLedTimeout = 0;
void loop() {
  if (millis() - blinkLedTimeout > 5000 && millis() - blinkLedTimeout < 5500) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else if(millis() - blinkLedTimeout > 5500) {
    digitalWrite(LED_BUILTIN, LOW);
    blinkLedTimeout = millis();
  }

  if(mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial())
  {
    unsigned long cardId = mfrc522.uid.uidByte[0] + (mfrc522.uid.uidByte[1] << 8) + (mfrc522.uid.uidByte[2] << 16) + (mfrc522.uid.uidByte[3] << 24);
    Serial.println(cardId);

    mfrc522.PICC_HaltA();
  }
}