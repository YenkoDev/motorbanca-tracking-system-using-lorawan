#include <Arduino_HS300x.h>         // Library to access the HTS221 sensor
#include <Arduino_LPS22HB.h>        // Library to access the barometric pressure sensor
#include "Arduino_BMI270_BMM150.h"  // Library to access IMU sensor
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <TinyGPS++.h>

// Define LoRa module pins
#define LORA_NSS   10
#define LORA_RST    9
#define LORA_DIO0   2
#define LORA_DIO1   3
#define LORA_DIO2   4

// Create a TinyGPS++ instance to parse GPS data
TinyGPSPlus gps;

// ABP credentials (replace these with your actual keys from TTN)
static const u1_t NWKSKEY[16] = { 0xD5, 0xC1, 0x2E, 0xA9, 0xB9, 0x63, 0xED, 0x91, 0x2D, 0x92, 0x66, 0xE7, 0x14, 0x91, 0x3D, 0xF5 };
static const u1_t APPSKEY[16] = { 0x5F, 0x4F, 0x1F, 0x73, 0xFD, 0xFE, 0xA8, 0x07, 0x0D, 0xCF, 0xCA, 0xBB, 0x52, 0x1A, 0xEE, 0xF0 };
static const u4_t DEVADDR = 0x27FC6763; // Example device address (update as needed)

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

osjob_t sendjob;  // Job for scheduling transmissions

// LMIC pin mapping
const lmic_pinmap lmic_pins = {
    .nss = LORA_NSS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LORA_RST,
    .dio = { LORA_DIO0, LORA_DIO1, LORA_DIO2 },
};

void do_send(osjob_t* j) {
  char payload[128] = {0};

  // Read available GPS data from Serial1 (hardware serial on pins 0 and 1)
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    gps.encode(c);
  }

  // Prepare payload based on GPS fix status
  float latitude, longitude;
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    // Format payload as: "lat:xx.xxxxxx,lon:yy.yyyyyy"
    Serial.println("Latitude: " + (String)latitude + ", Longitude: " + (String)longitude);
  } else {
    latitude = NULL;
    longitude = NULL;
  }

  // Read temperature and humidity from the onboard HTS221 sensor
  float temperature = HS300x.readTemperature();
  float humidity    = HS300x.readHumidity();
  Serial.println("Temperature: " + (String)temperature + " °C, Humidity: " + (String)humidity + " %");

  // --- Pressure Reading using LPS22HB Library ---
  // Read pressure (in hPa) from the onboard LPS22HB sensor.
  float pressure = BARO.readPressure();
  Serial.println("Pressure: " + (String)pressure + " hPa");

  // --- Tilt Angle Calculation using Arduino_BMI270_BMM150 Library ---
  // Read acceleration data from the BMI270 sensor and calculate tilt angle.
  float ax, ay, az;
  float tiltAngle = 0.0;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    float norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm > 0) {
        tiltAngle = acos(az / norm) * 180.0 / PI;
      }
  }
  Serial.println("Tilt Angle: " + (String)tiltAngle + " deg");

  snprintf(payload, sizeof(payload), "%.2f&%.2f&%.2f&%.2f&%.6f&%.6f", temperature, humidity, pressure, tiltAngle, latitude, longitude);
  // Print payload for debugging
  Serial.print("Sending: ");
  Serial.println(payload);

  // Prepare and schedule transmission on LoRaWAN (using port 1)
  LMIC_setTxData2(1, (uint8_t*)payload, strlen(payload), 0);
}

void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch(ev) {
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      // Schedule next transmission after 60 seconds
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(60), do_send);
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned)ev);
      break;
  }
}

void setup() {
  // Start serial communications for debugging over USB
  Serial.begin(115200);
  while (!Serial);

  // Initialize the onboard HTS221 sensor
  if (!HS300x.begin()) {
    Serial.println("Failed to initialize HTS sensor!");
    while (1); // Halt if sensor initialization fails
  }

  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
  }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // Initialize Serial1 for GPS communication
  // LMIC initialization
  os_init();
  Serial1.begin(9600);

  LMIC_reset();

  // Set static session parameters for ABP
  LMIC_setSession(0x1, DEVADDR, (xref2u1_t)NWKSKEY, (xref2u1_t)APPSKEY);

  // Disable link check validation (automatically enabled during join)
  LMIC_setLinkCheckMode(0);

  // Set data rate and transmit power (adjust as needed)
  LMIC_setDrTxpow(DR_SF7, 14);

  // Schedule the first transmission
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}
