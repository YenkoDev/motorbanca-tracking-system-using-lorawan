#include <Arduino_HS300x.h>
#include <Arduino_LPS22HB.h>
#include "Arduino_BMI270_BMM150.h"
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <TinyGPS++.h>

#define LORA_NSS   10
#define LORA_RST    9
#define LORA_DIO0   2
#define LORA_DIO1   3
#define LORA_DIO2   4
#define PUSH_BTN    5
#define BUZZER      6

TinyGPSPlus gps;
static const u1_t NWKSKEY[16] = { 0xD5, 0xC1, 0x2E, 0xA9, 0xB9, 0x63, 0xED, 0x91, 0x2D, 0x92, 0x66, 0xE7, 0x14, 0x91, 0x3D, 0xF5 };
static const u1_t APPSKEY[16] = { 0x5F, 0x4F, 0x1F, 0x73, 0xFD, 0xFE, 0xA8, 0x07, 0x0D, 0xCF, 0xCA, 0xBB, 0x52, 0x1A, 0xEE, 0xF0 };
static const u4_t DEVADDR = 0x27FC6763;

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

osjob_t sendjob;
const lmic_pinmap lmic_pins = { .nss = LORA_NSS,
                                .rxtx = LMIC_UNUSED_PIN,
                                .rst = LORA_RST,
                                .dio = { LORA_DIO0, LORA_DIO1, LORA_DIO2 } 
};

bool buttonState = false;
bool buttonPressed = false;
bool buzzerState = false;
unsigned long buzzerStartTime = 0;

void do_send(osjob_t* j) {
  char payload[128] = {0};
  while (Serial1.available() > 0) gps.encode(Serial1.read());

  float latitude = gps.location.isValid() ? gps.location.lat() : NULL;
  float longitude = gps.location.isValid() ? gps.location.lng() : NULL;
  float temperature = HS300x.readTemperature();
  float humidity = HS300x.readHumidity();
  float pressure = BARO.readPressure();

  float ax, ay, az, tiltAngle = 0.0;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    float norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm > 0) tiltAngle = acos(az / norm) * 180.0 / PI;
  }

  snprintf(payload, sizeof(payload), "%.2f&%.2f&%.2f&%.2f&%.6f&%.6f&%d", temperature, humidity, pressure, tiltAngle, latitude, longitude, buttonState);
  Serial.print("Sending: "); Serial.println(payload);

  LMIC_setTxData2(1, (uint8_t*)payload, strlen(payload), 0);
  buttonState = false; // Reset button state after transmission
}

void onEvent(ev_t ev) {
  Serial.print(os_getTime()); Serial.print(": ");
  switch(ev) {
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE"));
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(60), do_send);
      break;
    case EV_RXCOMPLETE:
      Serial.println("Payload Received!");
      if (LMIC.dataLen > 0 && LMIC.frame[LMIC.dataBeg] == '1') {
        buzzerState = true;
        buzzerStartTime = millis();
        digitalWrite(BUZZER, HIGH);
        Serial.println("Buzzer Activated!");
      }
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  pinMode(PUSH_BTN, INPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  HS300x.begin();
  BARO.begin();
  IMU.begin();
  Serial1.begin(9600);

  os_init();
  LMIC_reset();
  LMIC_setSession(0x1, DEVADDR, (xref2u1_t)NWKSKEY, (xref2u1_t)APPSKEY);
  LMIC_setLinkCheckMode(0);
  LMIC_setDrTxpow(DR_SF10, 14);

  do_send(&sendjob);
}

void loop() {
  os_runloop_once();

  if (digitalRead(PUSH_BTN) == LOW && !buttonPressed) {
    Serial.println("Button Pressed!");
    buttonState = true;
    buttonPressed = true;
  } else if (digitalRead(PUSH_BTN) == HIGH) {
    buttonPressed = false;
  }

  if (buzzerState && millis() - buzzerStartTime >= 20000) {
    buzzerState = false;
    digitalWrite(BUZZER, LOW);
  }
}
