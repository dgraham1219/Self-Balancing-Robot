#include <Wire.h>

// Encoders (channel A on the two hardware-interrupt pins; channel B read in ISR)
const uint8_t ENC_L_A = 2;    // INT0
const uint8_t ENC_L_B = 4;    // read via PIND bit 4
const uint8_t ENC_R_A = 3;    // INT1
const uint8_t ENC_R_B = 7;    // read via PIND bit 7

// TB6612 channel A = LEFT motor
const uint8_t AIN1 = 5, AIN2 = 6, PWMA = 9;   // PWMA on Timer1
// TB6612 channel B = RIGHT motor
const uint8_t BIN1 = 8, BIN2 = 12, PWMB = 10; // PWMB on Timer1

// ---------- MPU-6050 ---------------------------------------------------------
const uint8_t MPU_ADDR   = 0x68;
const float   ACC_LSB    = 16384.0;  // ±2g  -> 16384 LSB/g
const float   GYRO_LSB   = 131.0;    // ±250 dps -> 131 LSB/(deg/s)

// ---------- Control loop timing ---------------------------------------------
const unsigned long LOOP_US = 5000;  // 5 ms -> 200 Hz
unsigned long       lastLoop = 0;

// ---------- Complementary filter --------------------------------------------
const float ALPHA = 0.98;            // ~0.25s time constant at 200 Hz
float pitch = 0.0;                   // fused tilt angle (deg)
float gyroRate = 0.0;                // raw rate about tilt axis (deg/s)
float gyroBias = 0.0;                // measured at startup

// ---------- Setpoint / trim --------------------------------------------------
// Find this experimentally: hold the bot at true balance, read the printed
// pitch, and put that value here. It will NOT be exactly 0.
float BALANCE_OFFSET = 0.0;          // deg
const float FALL_LIMIT = 45.0;       // deg -> cut motors past this

// ---------- Inner loop: ANGLE  ->  motor command -----------------------------
float Kp_a = 22.0, Ki_a = 90.0, Kd_a = 0.9;   // start here, then tune
float angleSetpoint = 0.0;
float aInteg = 0.0;

// ---------- Outer loop: VELOCITY  ->  angle setpoint -------------------------
float Kp_v = 0.0008, Ki_v = 0.0020;           // small; tune AFTER angle loop
float velSetpoint = 0.0;                       // 0 = hold position
float vInteg = 0.0;
float robotVel = 0.0;                          // filtered, counts/sec

// ---------- Motor shaping ----------------------------------------------------
const int MOTOR_DEADBAND = 12;       // PWM to overcome static friction/backlash
const int MOTOR_MAX = 255;

// ---------- Encoder counts (written only in ISRs) ----------------------------
volatile long encL = 0, encR = 0;

// =============================================================================
void isrL() { if (PIND & _BV(PIND4)) encL++; else encL--; }
void isrR() { if (PIND & _BV(PIND7)) encR++; else encR--; }

// =============================================================================
//   MPU-6050
// =============================================================================
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

void mpuInit() {
  mpuWrite(0x6B, 0x00);   // PWR_MGMT_1: wake up
  mpuWrite(0x1B, 0x00);   // GYRO_CONFIG:  ±250 dps
  mpuWrite(0x1C, 0x00);   // ACCEL_CONFIG: ±2g
  delay(50);
}

// Reads accel X/Z and gyro Y. Adjust the axes to match YOUR mounting:
// upright should give pitch ~ 0, tilting forward should give a consistent sign,
// and gyroRate should move the SAME direction the angle is changing.
void mpuRead(float &accelAngle, float &rate) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                       // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();  (void)ay;
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                        // skip temperature
  int16_t gx = (Wire.read() << 8) | Wire.read();  (void)gx;
  int16_t gy = (Wire.read() << 8) | Wire.read();
  // (gz left unread)

  accelAngle = atan2((float)ax, (float)az) * 57.2958;   // deg from vertical
  rate       = ((float)gy / GYRO_LSB) - gyroBias;       // deg/s about tilt axis
}

void calibrateGyro() {
  float a, r, sum = 0;
  for (int i = 0; i < 1000; i++) { mpuRead(a, r); sum += (r + gyroBias); delay(2); }
  gyroBias = sum / 1000.0;   // keep the bot STILL during this
}

// =============================================================================
//   Motors  (u > 0 = "drive forward"; signs may need flipping per wiring)
// =============================================================================
void driveMotor(int u, uint8_t in1, uint8_t in2, uint8_t pwm) {
  int mag = abs(u);
  if (mag > 0) mag += MOTOR_DEADBAND;
  mag = constrain(mag, 0, MOTOR_MAX);
  bool fwd = (u >= 0);
  digitalWrite(in1, fwd);
  digitalWrite(in2, !fwd);
  analogWrite(pwm, mag);
}
void setMotors(int uL, int uR) {
  driveMotor(uL, AIN1, AIN2, PWMA);
  driveMotor(uR, BIN1, BIN2, PWMB);
}

// =============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);

  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);

  // Timer1 prescaler = 1  ->  ~31 kHz PWM on D9/D10 (above audible).
  // Only touches the prescaler bits; analogWrite() still works (8-bit).
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;

  Wire.begin();
  Wire.setClock(400000);          // 400 kHz I2C
  mpuInit();

  setMotors(0, 0);
  calibrateGyro();                // hold the robot dead still here

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR, RISING);

  // Seed the filter with the accel angle so it doesn't ramp in from 0.
  float a, r; mpuRead(a, r); pitch = a;
  lastLoop = micros();
}

// =============================================================================
void loop() {
  unsigned long now = micros();
  if (now - lastLoop < LOOP_US) return;
  float dt = (now - lastLoop) * 1e-6;
  lastLoop = now;

  // ---- 1. Attitude: complementary filter -----------------------------------
  float accelAngle;
  mpuRead(accelAngle, gyroRate);
  pitch = ALPHA * (pitch + gyroRate * dt) + (1.0 - ALPHA) * accelAngle;

  // ---- 2. Fall guard --------------------------------------------------------
  if (fabs(pitch - BALANCE_OFFSET) > FALL_LIMIT) {
    setMotors(0, 0);
    aInteg = vInteg = 0;          // dump integrators so it doesn't wind up
    return;
  }

  // ---- 3. Wheel velocity (counts/sec), atomically sampled -------------------
  static long lastL = 0, lastR = 0;
  noInterrupts();
  long cl = encL, cr = encR;
  interrupts();
  float vL = (cl - lastL) / dt;
  float vR = (cr - lastR) / dt;
  lastL = cl; lastR = cr;
  // Average = forward speed. If pushing the bot forward reads NEGATIVE here,
  // flip the sign on one encoder's ISR (above), not here.
  float vMeas = 0.5 * (vL + vR);
  robotVel += 0.2 * (vMeas - robotVel);   // light low-pass on velocity

  // ---- 4. Outer loop: velocity -> angle setpoint ----------------------------
  float vErr = velSetpoint - robotVel;
  vInteg += vErr * dt;
  vInteg = constrain(vInteg, -300, 300);
  angleSetpoint = BALANCE_OFFSET + (Kp_v * vErr + Ki_v * vInteg);

  // ---- 5. Inner loop: angle -> motor command --------------------------------
  float aErr = angleSetpoint - pitch;
  aInteg += aErr * dt;
  aInteg = constrain(aInteg, -200, 200);   // anti-windup
  // P + I on error, D on raw gyro rate (= -d(pitch)/dt, no noisy differentiation)
  float u = Kp_a * aErr + Ki_a * aInteg - Kd_a * gyroRate;
  u = constrain(u, -MOTOR_MAX, MOTOR_MAX);

  // Both wheels get the same balancing command (no steering term yet).
  setMotors((int)u, (int)u);

  // ---- 6. Telemetry for tuning (throttled) ----------------------------------
  static uint8_t n = 0;
  if (++n >= 10) {                 // ~20 Hz print
    n = 0;
    Serial.print(pitch, 2);        Serial.print('\t');
    Serial.print(angleSetpoint, 2);Serial.print('\t');
    Serial.print(robotVel, 0);     Serial.print('\t');
    Serial.println(u, 0);
  }
}
