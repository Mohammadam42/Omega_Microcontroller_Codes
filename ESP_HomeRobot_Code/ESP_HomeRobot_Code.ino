/*************** عدد الدورات ***************/
int totalCycles = 1;
int currentCycle = 0;
bool running = false;

/*************** BTS7960 Pins ***************/
#define RPWM_PIN  5
#define LPWM_PIN  6
#define REN_PIN   7
#define LEN_PIN   8

/*************** Buttons / Limits (Short to GND = ACTIVE) ***************/
#define BTN_START     42
#define BTN_STOP      41
#define LIMIT_FORWARD 40
#define LIMIT_BACK    39

/*************** PWM ***************/
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;
const int SPEED    = 200;   // 0..255

/*************** فلتر (ديباونس/فلترة ضجيج) ***************/
const unsigned long FILTER_MS = 40;  // زِدها 60-100ms إذا الضجيج قوي

// INPUT_PULLUP: التفعيل يكون LOW (Short to GND)
bool stableActiveLow(int pin, unsigned long ms = FILTER_MS) {
  if (digitalRead(pin) != LOW) return false;
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (digitalRead(pin) != LOW) return false;
  }
  return true;
}

// فلترة للإشارات التي يجب أن تكون "مستمرة" (limits أثناء الحركة)
bool stableIsLow(int pin, unsigned long ms = FILTER_MS) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (digitalRead(pin) != LOW) return false;
  }
  return true;
}

/*************** دوال المحرك ***************/
void motorStop() {
  ledcWrite(RPWM_PIN, 0);
  ledcWrite(LPWM_PIN, 0);
  digitalWrite(REN_PIN, LOW);
  digitalWrite(LEN_PIN, LOW);
}

void motorEnable() {
  digitalWrite(REN_PIN, HIGH);
  digitalWrite(LEN_PIN, HIGH);
}

void motorForward() {
  motorEnable();
  ledcWrite(LPWM_PIN, 0);
  ledcWrite(RPWM_PIN, SPEED);
}

void motorBackward() {
  motorEnable();
  ledcWrite(RPWM_PIN, 0);
  ledcWrite(LPWM_PIN, SPEED);
}

/*************** أمان (اختياري) ***************/
const unsigned long MOVE_TIMEOUT_MS = 20000; // حد أقصى للحركة

bool waitLimitActiveLowFiltered(int limitPin) {
  unsigned long t0 = millis();
  while (true) {
    // STOP أولوية
    if (stableActiveLow(BTN_STOP)) return false;

    // إذا الليمت صار LOW بشكل ثابت
    if (digitalRead(limitPin) == LOW) {
      if (stableIsLow(limitPin)) return true;
    }

    // Timeout
    if (millis() - t0 > MOVE_TIMEOUT_MS) return false;
  }
}

/*************** دورة واحدة ***************/
bool oneCycle() {

  // Forward -> لحد LIMIT_FORWARD
  motorForward();
  if (!waitLimitActiveLowFiltered(LIMIT_FORWARD)) { motorStop(); return false; }
  motorStop();
  delay(200);

  // Backward -> لحد LIMIT_BACK
  motorBackward();
  if (!waitLimitActiveLowFiltered(LIMIT_BACK)) { motorStop(); return false; }
  motorStop();
  delay(200);

  return true;
}

/*************** setup ***************/
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(REN_PIN, OUTPUT);
  pinMode(LEN_PIN, OUTPUT);

  // كلهم Short to GND = Active -> INPUT_PULLUP
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);
  pinMode(LIMIT_FORWARD, INPUT_PULLUP);
  pinMode(LIMIT_BACK, INPUT_PULLUP);

  // PWM (ESP32 core 3.x)
  ledcAttach(RPWM_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(LPWM_PIN, PWM_FREQ, PWM_RES);

  motorStop();
  Serial.println("Ready (Active=Short to GND) + Filter ON");
}

/*************** loop ***************/
void loop() {

  // STOP فوري (بفلترة)
  if (stableActiveLow(BTN_STOP)) {
    motorStop();
    running = false;
    Serial.println("STOP");
    return;
  }

  // START edge + فلترة
  static bool lastStartRaw = HIGH;
  bool startRaw = digitalRead(BTN_START);

  if (startRaw == LOW && lastStartRaw == HIGH && !running) {
    if (stableActiveLow(BTN_START)) {
      Serial.println("START");

      running = true;
      currentCycle = 0;

      while (running && currentCycle < totalCycles) {
        if (stableActiveLow(BTN_STOP)) {
          motorStop();
          running = false;
          Serial.println("STOP during run");
          break;
        }

        Serial.print("Cycle ");
        Serial.print(currentCycle + 1);
        Serial.print(" / ");
        Serial.println(totalCycles);

        if (!oneCycle()) {
          running = false;
          Serial.println("Stopped (STOP/Timeout/Limit noise)");
          break;
        }

        currentCycle++;
        delay(200);
      }

      motorStop();
      running = false;
      Serial.println("DONE");
    }
  }

  lastStartRaw = startRaw;
}
