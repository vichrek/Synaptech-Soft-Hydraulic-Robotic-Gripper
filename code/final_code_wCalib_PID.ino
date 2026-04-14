const int BEND_PIN_1  = A0;
const int BEND_PIN_2  = A1;
const int BEND_PIN_3  = A2;
const int FORCE_PIN_1 = A3;
const int FORCE_PIN_2 = A4;
const int FORCE_PIN_3 = A5;

// ── L298N #1 Channel A — Pump 1 ───────────────────────────
const int IN1_P1 = 9;
const int IN2_P1 = 8;
const int ENA_P1 = 10;

// ── L298N #1 Channel B — Pump 2 ───────────────────────────
const int IN1_P2 = 7;
const int IN2_P2 = 6;
const int ENA_P2 = 11;

// ── L298N #2 Channel A — Pump 3 ───────────────────────────
const int IN1_P3 = 4;
const int IN2_P3 = 3;
const int ENA_P3 = 5;

const int BTN_BEND = 2;
const int BTN_STOP = 13;

// ── Velostat thresholds (inverted: lower = more force) ─────
const int FORCE_NO_CONTACT    = 1022;
const int FORCE_SOFT_OBJECT   = 640;
const int FORCE_MEDIUM_OBJECT = 500;
const int FORCE_HARD_OBJECT   = 300;
const int FORCE_MAX_SAFE      = 145;

// ── PID gains per object type ──────────────────────────────
// Soft object: low gains for gentle, compliant interaction
const float KP_SOFT   = 1.5,  KI_SOFT   = 0.03, KD_SOFT   = 0.8;
// Medium object: moderate gains
const float KP_MEDIUM = 2.5,  KI_MEDIUM = 0.05, KD_MEDIUM = 1.0;
// Hard object: higher gains for faster, stiffer response
const float KP_HARD   = 3.5,  KI_HARD   = 0.08, KD_HARD   = 1.2;

// ── PID limits ─────────────────────────────────────────────
const float INTEGRAL_LIMIT = 300.0;   // anti-windup clamp (ADC units)
const int   PID_DEADBAND   = 15;      // ±ADC units — no output inside this band
const int   MIN_PWM        = 60;      // minimum PWM to overcome stiction
const int   MAX_PWM        = 255;

// ── Loop timing ────────────────────────────────────────────
const unsigned long LOOP_MS = 50;     // 20 Hz
unsigned long lastLoopTime  = 0;

enum ObjectType {
  OBJ_NONE, OBJ_SOFT, OBJ_MEDIUM, OBJ_HARD, OBJ_OVERLOAD
};

struct Finger {
  int bendPin;
  int forcePin;
  int in1, in2, ena;

  int setpointSoft;
  int setpointMedium;
  int setpointHard;
  int activeSetpoint;
  int bendThreshold;

  // PID state
  float error;
  float prevError;
  float integral;
  int   pumpSpeed;

  // Gains (set per object type)
  float Kp, Ki, Kd;

  int bendReading;
  int forceReading;

  ObjectType detectedObject;
  ObjectType pendingObject;
  int forceStableCount;
  int hysteresis;
};

Finger f1, f2, f3;

// ── Button debounce ────────────────────────────────────────
unsigned long lastDebounceTime[2] = {0, 0};
const unsigned long debounceDelay  = 50;
bool lastButtonState[2] = {HIGH, HIGH};
bool buttonState[2]     = {HIGH, HIGH};

// ── Hold detection for button 2 ───────────────────────────
unsigned long btn2PressTime   = 0;
bool btn2WasPressed           = false;
bool btn2HoldTriggered        = false;
const unsigned long HOLD_THRESHOLD = 400;

// ── System state ───────────────────────────────────────────
enum SystemState { IDLE, BENDING, HOLDING, UNBENDING };
SystemState systemState = IDLE;

// ──────────────────────────────────────────────────────────
void initFinger(Finger &f,
                int bendPin, int forcePin,
                int in1, int in2, int ena,
                int spSoft, int spMedium, int spHard, int threshold) {
  f.bendPin  = bendPin;
  f.forcePin = forcePin;
  f.in1 = in1; f.in2 = in2; f.ena = ena;

  f.setpointSoft   = spSoft;
  f.setpointMedium = spMedium;
  f.setpointHard   = spHard;
  f.activeSetpoint = spSoft;
  f.bendThreshold  = threshold;

  f.error     = 0;
  f.prevError = 0;
  f.integral  = 0;
  f.pumpSpeed = 0;

  // Default to soft gains at startup
  f.Kp = KP_SOFT;
  f.Ki = KI_SOFT;
  f.Kd = KD_SOFT;

  f.bendReading  = 0;
  f.forceReading = 0;

  f.detectedObject   = OBJ_NONE;
  f.pendingObject    = OBJ_NONE;
  f.forceStableCount = 0;
  f.hysteresis       = 60;

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ena, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(ena, LOW);
}

// ──────────────────────────────────────────────────────────
void setup() {
  initFinger(f1, BEND_PIN_1, FORCE_PIN_1,
             IN1_P1, IN2_P1, ENA_P1,
             200, 150, 100, 230);

  initFinger(f2, BEND_PIN_2, FORCE_PIN_2,
             IN1_P2, IN2_P2, ENA_P2,
             200, 150, 100, 230);

  initFinger(f3, BEND_PIN_3, FORCE_PIN_3,
             IN1_P3, IN2_P3, ENA_P3,
             200, 150, 100, 230);

  pinMode(BTN_BEND, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println("=== READY (PID) ===");
  Serial.println("Pin 2: BEND | Pin 13: STOP (press) / UNBEND (hold)");
  Serial.println("------------------------------------------------------");

  lastLoopTime = millis();
}

// ──────────────────────────────────────────────────────────
void loop() {
  // Enforce fixed loop period for consistent PID dt
  unsigned long now = millis();
  if (now - lastLoopTime < LOOP_MS) return;
  lastLoopTime = now;

  f1.bendReading  = analogRead(f1.bendPin);
  f1.forceReading = analogRead(f1.forcePin);
  f2.bendReading  = analogRead(f2.bendPin);
  f2.forceReading = analogRead(f2.forcePin);
  f3.bendReading  = analogRead(f3.bendPin);
  f3.forceReading = analogRead(f3.forcePin);

  readBendButton();
  readStopUnbendButton();

  detectObject(f1, 1);
  detectObject(f2, 2);
  detectObject(f3, 3);

  switch (systemState) {
    case IDLE:
      stopFinger(f1);
      stopFinger(f2);
      stopFinger(f3);
      break;
    case BENDING:
      runPID(f1, 1);
      runPID(f2, 2);
      runPID(f3, 3);
      // Transition to HOLDING once all fingers are within deadband
      if (abs(f1.error) <= PID_DEADBAND &&
          abs(f2.error) <= PID_DEADBAND &&
          abs(f3.error) <= PID_DEADBAND) {
        systemState = HOLDING;
        Serial.println("\n>>> ALL FINGERS AT SETPOINT — HOLDING <<<");
      }
      break;
    case HOLDING:
      runPID(f1, 1);
      runPID(f2, 2);
      runPID(f3, 3);
      break;
    case UNBENDING:
      digitalWrite(f1.in1, LOW); digitalWrite(f1.in2, HIGH); analogWrite(f1.ena, MAX_PWM);
      digitalWrite(f2.in1, LOW); digitalWrite(f2.in2, HIGH); analogWrite(f2.ena, MAX_PWM);
      digitalWrite(f3.in1, LOW); digitalWrite(f3.in2, HIGH); analogWrite(f3.ena, MAX_PWM);
      break;
  }

  printStatus();
}

// ──────────────────────────────────────────────────────────
// Core PID — called every LOOP_MS (50 ms), so dt = 0.05 s
void runPID(Finger &f, int id) {
  const float dt = LOOP_MS / 1000.0;

  // Safety: hard cutoff if bend threshold exceeded
  if (f.bendReading >= f.bendThreshold) {
    stopFinger(f);
    Serial.print("\n!!! FINGER "); Serial.print(id);
    Serial.println(" — BEND THRESHOLD EXCEEDED !!!");
    return;
  }

  f.error = f.activeSetpoint - f.bendReading;

  // Deadband — treat small errors as zero to suppress noise
  if (abs(f.error) <= PID_DEADBAND) {
    stopFinger(f);
    f.integral = 0;        // reset integral inside deadband to prevent windup
    f.prevError = f.error;
    return;
  }

  // Integral with anti-windup clamp
  f.integral += f.error * dt;
  f.integral  = constrain(f.integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // Derivative (on error, not measurement — adequate here given 20 Hz rate)
  float derivative = (f.error - f.prevError) / dt;

  // PID output
  float output = f.Kp * f.error
               + f.Ki * f.integral
               + f.Kd * derivative;

  f.prevError = f.error;

  // Map output magnitude to PWM, enforce minimum to overcome stiction
  int pwm = (int)abs(output);
  pwm = constrain(pwm, MIN_PWM, MAX_PWM);
  f.pumpSpeed = pwm;

  // Direction: positive error → bend more (inflate); negative → deflate
  if (output > 0) {
    digitalWrite(f.in1, HIGH);
    digitalWrite(f.in2, LOW);
    analogWrite(f.ena, pwm);
  } else {
    digitalWrite(f.in1, LOW);
    digitalWrite(f.in2, HIGH);
    analogWrite(f.ena, pwm);
  }
}

// ──────────────────────────────────────────────────────────
void stopFinger(Finger &f) {
  digitalWrite(f.in1, LOW);
  digitalWrite(f.in2, LOW);
  digitalWrite(f.ena, LOW);
  f.pumpSpeed = 0;
}

void resetFingers() {
  Finger* fingers[3] = {&f1, &f2, &f3};
  for (int i = 0; i < 3; i++) {
    fingers[i]->detectedObject   = OBJ_NONE;
    fingers[i]->pendingObject    = OBJ_NONE;
    fingers[i]->activeSetpoint   = fingers[i]->setpointSoft;
    fingers[i]->forceStableCount = 0;
    fingers[i]->integral         = 0;
    fingers[i]->prevError        = 0;
    fingers[i]->error            = 0;
    fingers[i]->Kp = KP_SOFT;
    fingers[i]->Ki = KI_SOFT;
    fingers[i]->Kd = KD_SOFT;
  }
}

// ── Button 1 — press to bend ───────────────────────────────
void readBendButton() {
  int reading = digitalRead(BTN_BEND);
  if (reading != lastButtonState[0]) lastDebounceTime[0] = millis();
  if ((millis() - lastDebounceTime[0]) > debounceDelay) {
    if (reading != buttonState[0]) {
      buttonState[0] = reading;
      if (buttonState[0] == LOW) {
        if (systemState == IDLE || systemState == HOLDING ||
            systemState == UNBENDING) {
          systemState = BENDING;
          // Reset PID state on new grip attempt
          Finger* fingers[3] = {&f1, &f2, &f3};
          for (int i = 0; i < 3; i++) {
            fingers[i]->activeSetpoint = fingers[i]->setpointSoft;
            fingers[i]->integral       = 0;
            fingers[i]->prevError      = 0;
          }
          Serial.println("\n>>> BENDING <<<");
        }
      }
    }
  }
  lastButtonState[0] = reading;
}

// ── Button 2 — press to stop, hold to unbend ──────────────
void readStopUnbendButton() {
  int reading = digitalRead(BTN_STOP);

  if (reading != lastButtonState[1]) lastDebounceTime[1] = millis();
  if ((millis() - lastDebounceTime[1]) > debounceDelay) {
    if (reading != buttonState[1]) {
      buttonState[1] = reading;

      if (buttonState[1] == LOW) {
        btn2PressTime     = millis();
        btn2WasPressed    = true;
        btn2HoldTriggered = false;
      }

      if (buttonState[1] == HIGH && btn2WasPressed) {
        if (!btn2HoldTriggered) {
          systemState = IDLE;
          stopFinger(f1); stopFinger(f2); stopFinger(f3);
          resetFingers();
          Serial.println("\n>>> STOPPED — IDLE <<<");
          Serial.println("------------------------------------------------------");
        } else {
          systemState = IDLE;
          stopFinger(f1); stopFinger(f2); stopFinger(f3);
          resetFingers();
          Serial.println("\n>>> UNBEND STOPPED — IDLE <<<");
          Serial.println("------------------------------------------------------");
        }
        btn2WasPressed    = false;
        btn2HoldTriggered = false;
      }
    }
  }
  lastButtonState[1] = reading;

  if (btn2WasPressed && !btn2HoldTriggered &&
      (millis() - btn2PressTime) > HOLD_THRESHOLD) {
    btn2HoldTriggered = true;
    systemState       = UNBENDING;
    // Clear integral so PID doesn't fight unbend on re-grip
    f1.integral = 0; f2.integral = 0; f3.integral = 0;
    Serial.println("\n>>> UNBENDING (hold to continue) <<<");
  }
}

// ──────────────────────────────────────────────────────────
void detectObject(Finger &f, int id) {
  ObjectType newObject;

  if      (f.forceReading > FORCE_NO_CONTACT)    newObject = OBJ_NONE;
  else if (f.forceReading > FORCE_SOFT_OBJECT)   newObject = OBJ_SOFT;
  else if (f.forceReading > FORCE_MEDIUM_OBJECT) newObject = OBJ_MEDIUM;
  else if (f.forceReading > FORCE_HARD_OBJECT)   newObject = OBJ_HARD;
  else if (f.forceReading <= FORCE_MAX_SAFE)      newObject = OBJ_OVERLOAD;
  else                                            newObject = OBJ_HARD;

  if (newObject == OBJ_OVERLOAD &&
      systemState != IDLE && systemState != UNBENDING) {
    systemState = UNBENDING;
    f1.integral = 0; f2.integral = 0; f3.integral = 0;
    Serial.print("\n!!! FINGER "); Serial.print(id);
    Serial.println(" FORCE OVERLOAD — UNBENDING !!!");
    return;
  }

  if (newObject == f.pendingObject) {
    f.forceStableCount++;
  } else {
    f.pendingObject    = newObject;
    f.forceStableCount = 0;
  }

  if (f.forceStableCount >= 8 && newObject != f.detectedObject) {
    f.detectedObject   = newObject;
    f.forceStableCount = 0;
    updateSetpoint(f, id);
  }
}

// ──────────────────────────────────────────────────────────
void updateSetpoint(Finger &f, int id) {
  // Reset integral on setpoint change to prevent windup carryover
  f.integral  = 0;
  f.prevError = 0;

  switch (f.detectedObject) {
    case OBJ_NONE:
    case OBJ_SOFT:
      f.activeSetpoint = f.setpointSoft;
      f.Kp = KP_SOFT; f.Ki = KI_SOFT; f.Kd = KD_SOFT;
      break;
    case OBJ_MEDIUM:
      f.activeSetpoint = f.setpointMedium;
      f.Kp = KP_MEDIUM; f.Ki = KI_MEDIUM; f.Kd = KD_MEDIUM;
      break;
    case OBJ_HARD:
      f.activeSetpoint = f.setpointHard;
      f.Kp = KP_HARD; f.Ki = KI_HARD; f.Kd = KD_HARD;
      break;
    case OBJ_OVERLOAD:
      break;
  }

  Serial.print("[F"); Serial.print(id); Serial.print("] Object: ");
  switch (f.detectedObject) {
    case OBJ_NONE:     Serial.print("NONE");     break;
    case OBJ_SOFT:     Serial.print("SOFT");     break;
    case OBJ_MEDIUM:   Serial.print("MEDIUM");   break;
    case OBJ_HARD:     Serial.print("HARD");     break;
    case OBJ_OVERLOAD: Serial.print("OVERLOAD"); break;
  }
  Serial.print(" | Setpoint: "); Serial.print(f.activeSetpoint);
  Serial.print(" | Kp: ");       Serial.print(f.Kp);
  Serial.print(" Ki: ");         Serial.print(f.Ki);
  Serial.print(" Kd: ");         Serial.println(f.Kd);
}

// ──────────────────────────────────────────────────────────
void printStatus() {
  Serial.print("State: ");
  switch (systemState) {
    case IDLE:      Serial.print("IDLE    "); break;
    case BENDING:   Serial.print("BENDING "); break;
    case HOLDING:   Serial.print("HOLDING "); break;
    case UNBENDING: Serial.print("UNBEND  "); break;
  }

  // Finger 1
  Serial.print(" | F1 Bend: ");  Serial.print(f1.bendReading);
  Serial.print("/");             Serial.print(f1.activeSetpoint);
  Serial.print(" Err: ");        Serial.print(f1.error, 1);
  Serial.print(" Int: ");        Serial.print(f1.integral, 1);
  Serial.print(" PWM: ");        Serial.print(f1.pumpSpeed);
  Serial.print(" Force: ");      Serial.print(f1.forceReading);
  Serial.print(" [");
  switch (f1.detectedObject) {
    case OBJ_NONE:     Serial.print("NONE  "); break;
    case OBJ_SOFT:     Serial.print("SOFT  "); break;
    case OBJ_MEDIUM:   Serial.print("MEDIUM"); break;
    case OBJ_HARD:     Serial.print("HARD  "); break;
    case OBJ_OVERLOAD: Serial.print("OVRLD "); break;
  }
  Serial.print("]");

  // Finger 2
  Serial.print(" | F2 Bend: ");  Serial.print(f2.bendReading);
  Serial.print("/");             Serial.print(f2.activeSetpoint);
  Serial.print(" Err: ");        Serial.print(f2.error, 1);
  Serial.print(" Int: ");        Serial.print(f2.integral, 1);
  Serial.print(" PWM: ");        Serial.print(f2.pumpSpeed);
  Serial.print(" Force: ");      Serial.print(f2.forceReading);
  Serial.print(" [");
  switch (f2.detectedObject) {
    case OBJ_NONE:     Serial.print("NONE  "); break;
    case OBJ_SOFT:     Serial.print("SOFT  "); break;
    case OBJ_MEDIUM:   Serial.print("MEDIUM"); break;
    case OBJ_HARD:     Serial.print("HARD  "); break;
    case OBJ_OVERLOAD: Serial.print("OVRLD "); break;
  }
  Serial.print("]");

  // Finger 3
  Serial.print(" | F3 Bend: ");  Serial.print(f3.bendReading);
  Serial.print("/");             Serial.print(f3.activeSetpoint);
  Serial.print(" Err: ");        Serial.print(f3.error, 1);
  Serial.print(" Int: ");        Serial.print(f3.integral, 1);
  Serial.print(" PWM: ");        Serial.print(f3.pumpSpeed);
  Serial.print(" Force: ");      Serial.print(f3.forceReading);
  Serial.print(" [");
  switch (f3.detectedObject) {
    case OBJ_NONE:     Serial.print("NONE  "); break;
    case OBJ_SOFT:     Serial.print("SOFT  "); break;
    case OBJ_MEDIUM:   Serial.print("MEDIUM"); break;
    case OBJ_HARD:     Serial.print("HARD  "); break;
    case OBJ_OVERLOAD: Serial.print("OVRLD "); break;
  }
  Serial.println("]");

  // Calibration line for Python
  Serial.print("CALIB,");
  Serial.print(f1.bendReading); Serial.print(",");
  Serial.print(f2.bendReading); Serial.print(",");
  Serial.print(f3.bendReading); Serial.print(",");
  Serial.print(f1.forceReading); Serial.print(",");
  Serial.print(f2.forceReading); Serial.print(",");
  Serial.println(f3.forceReading);
}
