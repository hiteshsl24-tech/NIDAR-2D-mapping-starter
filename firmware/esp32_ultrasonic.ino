/*
  NIDAR ESP32 + 4x HC-SR04 serial publisher.
  IMPORTANT: use a voltage divider on every HC-SR04 ECHO line before ESP32 GPIO.
  HC-SR04 ECHO is 5 V; ESP32 GPIO is not 5 V tolerant.
*/
#include <Arduino.h>

struct UltrasonicSensor {
  const char* name;
  uint8_t trig;
  uint8_t echo;
  long lastMm;
  bool valid;
};

UltrasonicSensor sensors[] = {
  {"front", 16, 17, -1, false},
  {"left",  18, 19, -1, false},
  {"right", 21, 22, -1, false},
  {"rear",  25, 26, -1, false}
};
const size_t SENSOR_COUNT = sizeof(sensors) / sizeof(sensors[0]);
uint32_t sequenceNumber = 0;

long readDistanceMm(UltrasonicSensor &s) {
  digitalWrite(s.trig, LOW);
  delayMicroseconds(3);
  digitalWrite(s.trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(s.trig, LOW);
  // 25 ms timeout ~4.3 m; limit output to 4 m.
  unsigned long duration = pulseIn(s.echo, HIGH, 25000UL);
  if (duration == 0) return -1;
  long mm = (long)(duration * 0.343 / 2.0);
  if (mm < 40 || mm > 4000) return -1;
  return mm;
}

void setup() {
  Serial.begin(115200);
  for (size_t i = 0; i < SENSOR_COUNT; ++i) {
    pinMode(sensors[i].trig, OUTPUT);
    pinMode(sensors[i].echo, INPUT);
    digitalWrite(sensors[i].trig, LOW);
  }
}

void loop() {
  // Sequential sampling reduces acoustic cross-talk.
  for (size_t i = 0; i < SENSOR_COUNT; ++i) {
    sensors[i].lastMm = readDistanceMm(sensors[i]);
    sensors[i].valid = sensors[i].lastMm > 0;
    delay(55);
  }

  Serial.print("{\"seq\":");
  Serial.print(sequenceNumber++);
  for (size_t i = 0; i < SENSOR_COUNT; ++i) {
    Serial.print(",\"");
    Serial.print(sensors[i].name);
    Serial.print("_mm\":");
    Serial.print(sensors[i].valid ? sensors[i].lastMm : -1);
  }
  Serial.print(",\"valid\":{");
  for (size_t i = 0; i < SENSOR_COUNT; ++i) {
    if (i) Serial.print(",");
    Serial.print("\"");
    Serial.print(sensors[i].name);
    Serial.print("\":");
    Serial.print(sensors[i].valid ? "true" : "false");
  }
  Serial.println("}}");
}
