#include <Arduino.h>
#include "MFRC522.h"
#include <vector>
#include <Adafruit_TinyUSB.h>

#define CS_PIN 17
#define MISO_PIN 16
#define MOSI_PIN 19
#define SCK_PIN 18

#define RELEASE_DELAY 5
#define KEY_DELAY 15

uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(1)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(2)),
    TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(3))};
Adafruit_USBD_HID usb_hid(desc_hid_report, sizeof(desc_hid_report), HID_ITF_PROTOCOL_NONE, 1, false);

MFRC522 mfrc522(CS_PIN, UNUSED_PIN);
void setup() {
  Serial.begin(9600);
  
  SPI.begin();
  mfrc522.PCD_Init();

  pinMode(LED_BUILTIN, OUTPUT);

  usb_hid.setStringDescriptor("Serial RFID Keyboard");
  // usb_hid.setReportCallback(NULL, hid_report_callback);
  usb_hid.begin();
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

    String cardIdStr = String(cardId);
    for (int i = 0; i < cardIdStr.length(); i++) {
      usb_hid.keyboardPress(1, cardIdStr.charAt(i));
      delay(RELEASE_DELAY);
      usb_hid.keyboardRelease(1);
      delay(KEY_DELAY);
    }

    uint8_t keycode[6] = {0};
    keycode[0] = 0x28; // ENTER
    usb_hid.keyboardReport(1, 0, keycode);
    delay(RELEASE_DELAY);
    usb_hid.keyboardRelease(1);

    mfrc522.PICC_HaltA();
  }
}
