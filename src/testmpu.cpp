#include <Arduino.h>
#include <Wire.h>

uint8_t mpuAddr = 0;
bool mpuReady = false;

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t reg, uint8_t *buffer, uint8_t count) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t received = Wire.requestFrom(mpuAddr, count);

  if (received != count) {
    while (Wire.available()) Wire.read();
    return false;
  }

  for (uint8_t i = 0; i < count; i++) {
    buffer[i] = Wire.read();
  }

  return true;
}

int16_t signed16(const uint8_t *p) {
  return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

bool initMPU() {
  const uint8_t addresses[] = {0x68, 0x69};

  for (uint8_t i = 0; i < 2; i++) {
    mpuAddr = addresses[i];

    Wire.beginTransmission(mpuAddr);
    if (Wire.endTransmission() != 0) continue;

    uint8_t who = 0;
    if (!readRegs(0x75, &who, 1)) continue;

    Serial.print("Dia chi: 0x");
    Serial.print(mpuAddr, HEX);
    Serial.print(" | WHO_AM_I: 0x");
    Serial.println(who, HEX);

    if (who != 0x68) {
      Serial.println("ID khong dung MPU6050.");
      continue;
    }

    // Reset MPU6050
    if (!writeReg(0x6B, 0x80)) return false;
    delay(100);

    // Thoat sleep, dung clock PLL gyro X
    if (!writeReg(0x6B, 0x01)) return false;
    delay(100);

    // Bat tat ca cac truc
    if (!writeReg(0x6C, 0x00)) return false;

    // Bo loc thong thap: gyro ~42 Hz, accel ~44 Hz
    if (!writeReg(0x1A, 0x03)) return false;

    // Tan so lay mau: 1000 / (1 + 9) = 100 Hz
    if (!writeReg(0x19, 0x09)) return false;

    // Gyro: +/-250 do/giay
    if (!writeReg(0x1B, 0x00)) return false;

    // Gia toc: +/-2 g
    if (!writeReg(0x1C, 0x00)) return false;

    delay(100);
    return true;
  }

  return false;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("=== TEST MPU6050 - J17 ===");
  Serial.println("SDA = PB7 | SCL = PB6");

  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  Wire.setClock(100000);

  mpuReady = initMPU();

  if (mpuReady) {
    Serial.println("MPU6050 OK!");
    Serial.println("A: g | G: do/giay | T: do C");
  } else {
    Serial.println("LOI: Khong khoi tao duoc MPU6050.");
    Serial.println("Kiem tra nguon 3V3, GND, SDA PB7, SCL PB6.");
  }
}

void loop() {
  if (!mpuReady) {
    delay(2000);
    Serial.println("Dang thu ket noi lai...");
    mpuReady = initMPU();

    if (mpuReady) Serial.println("MPU6050 OK!");
    return;
  }

  uint8_t data[14];

  // Doc lien tiep accel, nhiet do va gyro
  if (!readRegs(0x3B, data, sizeof(data))) {
    Serial.println("LOI DOC I2C! Dang ket noi lai...");
    mpuReady = false;
    return;
  }

  float ax = signed16(&data[0])  / 16384.0f;
  float ay = signed16(&data[2])  / 16384.0f;
  float az = signed16(&data[4])  / 16384.0f;

  float temperature = signed16(&data[6]) / 340.0f + 36.53f;

  float gx = signed16(&data[8])  / 131.0f;
  float gy = signed16(&data[10]) / 131.0f;
  float gz = signed16(&data[12]) / 131.0f;

  Serial.print("AX="); Serial.print(ax, 3);
  Serial.print(" AY="); Serial.print(ay, 3);
  Serial.print(" AZ="); Serial.print(az, 3);

  Serial.print(" | GX="); Serial.print(gx, 2);
  Serial.print(" GY="); Serial.print(gy, 2);
  Serial.print(" GZ="); Serial.print(gz, 2);

  Serial.print(" | T=");
  Serial.print(temperature, 1);
  Serial.println(" C");

  delay(200);
}