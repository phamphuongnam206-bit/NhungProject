#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

constexpr uint32_t SENSOR_COUNT = 4;

const uint32_t xshutPins[SENSOR_COUNT] = {
  PA2, PA3, PA4, PA5
};

const uint8_t sensorAddresses[SENSOR_COUNT] = {
  0x30, 0x31, 0x32, 0x33
};

const char *sensorNames[SENSOR_COUNT] = {
  "J8", "J9", "J10", "J11"
};

VL53L0X sensors[SENSOR_COUNT];
bool sensorReady[SENSOR_COUNT] = {};

uint32_t counter = 0;

void disableSensor(uint32_t index) {
  digitalWrite(xshutPins[index], LOW);
  pinMode(xshutPins[index], OUTPUT);
}

void setup() {
  digitalWrite(PB0, LOW);
  pinMode(PB0, OUTPUT);

  Serial.setRx(PA10);
  Serial.setTx(PA9);
  Serial.begin(115200, SERIAL_8N1);

  delay(1000);
  Serial.println();
  Serial.println("STM32 STARTED");
  Serial.flush();

  for (uint32_t i = 0; i < SENSOR_COUNT; i++) {
    disableSensor(i);
  }

  delay(200);

  Serial.println("ALL XSHUT LOW");
  Serial.flush();

  Wire.setSCL(PB10);
  Wire.setSDA(PB11);

  Serial.println("BEFORE I2C BEGIN");
  Serial.flush();

  Wire.begin();
  Wire.setClock(50000);

  Serial.println("AFTER I2C BEGIN");
  Serial.flush();

  for (uint32_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print("START ");
    Serial.println(sensorNames[i]);
    Serial.flush();

    pinMode(xshutPins[i], INPUT);
    delay(200);

    Serial.print("XSHUT=");
    Serial.println(digitalRead(xshutPins[i]));
    Serial.flush();

    if (digitalRead(xshutPins[i]) == LOW) {
      Serial.print(sensorNames[i]);
      Serial.println(" XSHUT LOW");
      disableSensor(i);
      continue;
    }

    Serial.print(sensorNames[i]);
    Serial.println(" BEFORE PROBE");
    Serial.flush();

    Wire.beginTransmission(0x29);
    uint8_t defaultProbeResult = Wire.endTransmission();

    Serial.print(sensorNames[i]);
    Serial.print(" PROBE 0x29 RESULT=");
    Serial.println(defaultProbeResult);
    Serial.flush();

    if (defaultProbeResult != 0) {
      disableSensor(i);
      continue;
    }

    sensors[i].setBus(&Wire);
    sensors[i].setTimeout(500);

    Serial.print(sensorNames[i]);
    Serial.println(" BEFORE SENSOR INIT");
    Serial.flush();

    if (!sensors[i].init()) {
      Serial.print(sensorNames[i]);
      Serial.println(" INIT FAILED");
      Serial.flush();
      disableSensor(i);
      continue;
    }

    Serial.print(sensorNames[i]);
    Serial.println(" INIT OK");
    Serial.flush();

    sensors[i].setAddress(sensorAddresses[i]);

    uint8_t addressWriteResult = sensors[i].last_status;

    Wire.beginTransmission(sensorAddresses[i]);
    uint8_t newProbeResult = Wire.endTransmission();

    Serial.print(sensorNames[i]);
    Serial.print(" ADDRESS WRITE RESULT=");
    Serial.print(addressWriteResult);
    Serial.print(" NEW PROBE RESULT=");
    Serial.println(newProbeResult);
    Serial.flush();

    if (addressWriteResult != 0 || newProbeResult != 0) {
      disableSensor(i);
      continue;
    }

    sensorReady[i] = true;

    Serial.print(sensorNames[i]);
    Serial.print(" READY ADDRESS=0x");
    Serial.println(sensorAddresses[i], HEX);
    Serial.flush();
  }

  Serial.println("INITIALIZATION FINISHED");
  Serial.flush();
}

void loop() {
  Serial.print("TEST=");
  Serial.print(counter++);

  for (uint32_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(" | ");
    Serial.print(sensorNames[i]);
    Serial.print("=");

    if (!sensorReady[i]) {
      Serial.print("OFF");
      continue;
    }

    uint16_t distance =
        sensors[i].readRangeSingleMillimeters();

    if (sensors[i].timeoutOccurred()) {
      Serial.print("TIMEOUT");
    } else {
      Serial.print(distance);
      Serial.print(" mm");
    }
  }

  Serial.println();
  delay(200);
}