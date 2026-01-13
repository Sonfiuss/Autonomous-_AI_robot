#include <Arduino.h>
#include <Wire.h>

namespace {

// -------------------- User-adjustable settings --------------------
constexpr uint8_t PCA9685_I2C_ADDR = 0x40;
// Two-servo setup (0-based PCA9685 channel indexing).
constexpr uint8_t SERVO1_CHANNEL = 0;      // Servo #1 channel 0..15
constexpr uint8_t SERVO2_CHANNEL = 1;      // Servo #2 channel 0..15
constexpr float PWM_FREQUENCY_HZ = 50.0f; // Typical for hobby servos

// Servo mapping (adjust to your servo + mechanics)
constexpr int SERVO_MIN_DEG = 0;
constexpr int SERVO_MAX_DEG = 180;
constexpr uint16_t SERVO_MIN_US = 500;   // typical: 500us
constexpr uint16_t SERVO_MAX_US = 2500;  // typical: 2500us

constexpr int DEFAULT_ORIGIN_DEG = 90;
constexpr int DELTA_DEG = 30;

// -------------------- PCA9685 minimal driver --------------------
class PCA9685 {
public:
  explicit PCA9685(uint8_t i2cAddress) : address_(i2cAddress) {}

  void begin() {
    Wire.begin();
#if defined(TWBR)
    Wire.setClock(400000);
#endif
    // MODE1: restart=0, extclk=0, auto-inc=1, sleep=0
    write8(0x00, 0x20);
    delay(5);
  }

  void setPWMFreq(float freqHz) {
    // Datasheet formula: prescale = round(osc/(4096*freq)) - 1
    // Default internal oscillator is typically 25MHz.
    constexpr float oscHz = 25000000.0f;
    float prescaleVal = (oscHz / (4096.0f * freqHz)) - 1.0f;
    uint8_t prescale = static_cast<uint8_t>(prescaleVal + 0.5f);

    uint8_t oldMode = read8(0x00);
    uint8_t sleepMode = (oldMode & 0x7F) | 0x10; // sleep=1
    write8(0x00, sleepMode);
    write8(0xFE, prescale); // PRE_SCALE
    write8(0x00, oldMode);
    delay(5);
    write8(0x00, oldMode | 0x80 | 0x20); // restart=1, auto-inc=1
  }

  void setPWM(uint8_t channel, uint16_t onTick, uint16_t offTick) {
    if (channel > 15) return;
    const uint8_t base = 0x06 + 4 * channel; // LED0_ON_L
    write8(base + 0, static_cast<uint8_t>(onTick & 0xFF));
    write8(base + 1, static_cast<uint8_t>(onTick >> 8));
    write8(base + 2, static_cast<uint8_t>(offTick & 0xFF));
    write8(base + 3, static_cast<uint8_t>(offTick >> 8));
  }

private:
  uint8_t address_;

  void write8(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
  }

  uint8_t read8(uint8_t reg) {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(static_cast<int>(address_), 1);
    if (Wire.available() < 1) return 0;
    return Wire.read();
  }
};

int clampInt(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

uint16_t angleToPulseUs(int deg) {
  deg = clampInt(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  const long spanDeg = static_cast<long>(SERVO_MAX_DEG - SERVO_MIN_DEG);
  const long spanUs = static_cast<long>(SERVO_MAX_US - SERVO_MIN_US);
  const long num = static_cast<long>(deg - SERVO_MIN_DEG) * spanUs;
  return static_cast<uint16_t>(SERVO_MIN_US + (num / spanDeg));
}

uint16_t pulseUsToTick(uint16_t pulseUs, float freqHz) {
  const float periodUs = 1000000.0f / freqHz;
  float tick = (static_cast<float>(pulseUs) * 4096.0f) / periodUs;
  if (tick < 0.0f) tick = 0.0f;
  if (tick > 4095.0f) tick = 4095.0f;
  return static_cast<uint16_t>(tick + 0.5f);
}

PCA9685 pca(PCA9685_I2C_ADDR);

struct ServoState {
  uint8_t channel;
  int originDeg;
  int currentDeg;
};

ServoState servo1{SERVO1_CHANNEL, DEFAULT_ORIGIN_DEG, DEFAULT_ORIGIN_DEG};
ServoState servo2{SERVO2_CHANNEL, DEFAULT_ORIGIN_DEG, DEFAULT_ORIGIN_DEG};

void commandServoToDeg(ServoState &servo, int deg) {
  servo.currentDeg = clampInt(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  const uint16_t pulseUs = angleToPulseUs(servo.currentDeg);
  const uint16_t offTick = pulseUsToTick(pulseUs, PWM_FREQUENCY_HZ);
  pca.setPWM(servo.channel, 0, offTick);

  Serial.print(F("Servo CH="));
  Serial.print(servo.channel);
  Serial.print(F("  deg="));
  Serial.print(servo.currentDeg);
  Serial.print(F("  pulse(us)="));
  Serial.print(pulseUs);
  Serial.print(F("  tick="));
  Serial.println(offTick);
}

void printHelp() {
  Serial.println();
  Serial.println(F("=== ControlARM: two-servo test (PCA9685) ==="));
  Serial.println(F("Commands:"));
  Serial.println(F("  A : servo1 to origin - 30 deg"));
  Serial.println(F("  D : servo1 to origin + 30 deg"));
  Serial.println(F("  S : servo1 to origin (center)"));
  Serial.println(F("  J : servo2 to origin - 30 deg"));
  Serial.println(F("  L : servo2 to origin + 30 deg"));
  Serial.println(F("  K : servo2 to origin (center)"));
  Serial.println(F("  O : center BOTH servos (servo1 + servo2 origin)"));
  Serial.println(F("Notes: Hobby servos have no position feedback; 'origin' is software-defined."));
  Serial.println(F("      Ensure separate servo power + common GND with Arduino."));
  Serial.println();
}

} // namespace

void setup() {
  Serial.begin(115200);
  delay(200);

  pca.begin();
  pca.setPWMFreq(PWM_FREQUENCY_HZ);

  servo1.originDeg = DEFAULT_ORIGIN_DEG;
  servo1.currentDeg = DEFAULT_ORIGIN_DEG;
  servo2.originDeg = DEFAULT_ORIGIN_DEG;
  servo2.currentDeg = DEFAULT_ORIGIN_DEG;

  printHelp();
  commandServoToDeg(servo1, servo1.originDeg);
  commandServoToDeg(servo2, servo2.originDeg);
}

void loop() {
  if (!Serial.available()) return;

  const char c = static_cast<char>(Serial.read());
  if (c == 'A' || c == 'a') {
    commandServoToDeg(servo1, servo1.originDeg - DELTA_DEG);
  } else if (c == 'D' || c == 'd') {
    commandServoToDeg(servo1, servo1.originDeg + DELTA_DEG);
  } else if (c == 'S' || c == 's') {
    commandServoToDeg(servo1, servo1.originDeg);
  } else if (c == 'J' || c == 'j') {
    commandServoToDeg(servo2, servo2.originDeg - DELTA_DEG);
  } else if (c == 'L' || c == 'l') {
    commandServoToDeg(servo2, servo2.originDeg + DELTA_DEG);
  } else if (c == 'K' || c == 'k') {
    commandServoToDeg(servo2, servo2.originDeg);
  } else if (c == 'O' || c == 'o') {
    commandServoToDeg(servo1, servo1.originDeg);
    commandServoToDeg(servo2, servo2.originDeg);
  } else if (c == 'H' || c == 'h' || c == '?') {
    printHelp();
  }
}