#include <Arduino.h>
#include "MFRC522.h"
#include <vector>

#define CS_PIN 17
#define MISO_PIN 16
#define MOSI_PIN 19
#define SCK_PIN 18

MFRC522 mfrc522(CS_PIN, UNUSED_PIN);
unsigned long lastCardId = 0;
std::vector<unsigned long> lastCards;

void setup() {
  Serial.begin(9600);
  // SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  SPI.begin();
  mfrc522.PCD_Init();

  pinMode(LED_BUILTIN, OUTPUT);
}

unsigned long cardScanTimeout = 0;
void loop() {
  if (millis() - cardScanTimeout > 5000 && millis() - cardScanTimeout < 5500) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else if(millis() - cardScanTimeout > 5500) {
    digitalWrite(LED_BUILTIN, LOW);
    cardScanTimeout = millis();
  }

  if(mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial())
  {
    unsigned long cardId = mfrc522.uid.uidByte[0] + (mfrc522.uid.uidByte[1] << 8) + (mfrc522.uid.uidByte[2] << 16) + (mfrc522.uid.uidByte[3] << 24);
    if (lastCardId != cardId) {
      bool duplicated = false;
      for (int i = 0; i < (int)lastCards.size(); i++) {
        if (lastCards[i] == cardId) {
          duplicated = true;
          break;
        }
      }

      if (duplicated) {
        Serial.print("DUPLICATE DETECTED!!!!! (");
        Serial.print(cardId);
        Serial.println(")");
      } else {
        lastCards.push_back(cardId);
      }
      /*
      if (std::find(lastCards.begin(), lastCards.end(), cardId) != lastCards.end()) {
        Serial.print("DUPLICATE DETECTED!!!!! (");
        Serial.print(cardId);
        Serial.println(")");
      } else {
        lastCards.push_back(cardId);
      }
      */

      Serial.print("Card ID: ");
      Serial.print(cardId);
      Serial.print(" | COUNT: ");
      Serial.println(lastCards.size());
      lastCardId = cardId;
    }
  }
}