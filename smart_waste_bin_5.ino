#include <Servo.h>

// ==========================
// Capacity System - Ultrasonic Sensor
// ==========================
const int trigPin1 = 3;
const int echoPin1 = 2;

// ==========================
// Lid Control System - Ultrasonic Sensor
// ==========================
const int trigPin2 = 6;
const int echoPin2 = 5;

// ==========================
// LED / Buzzer / Servo Pins
// ==========================
#define LED_RED 12
#define LED_YELLOW 11
#define BUZZER_PIN 8
#define SERVO_PIN 13

Servo lidServo;

// ==========================
// Capacity thresholds (cm from sensor to waste surface)
// Bin depth = 25 cm, so:
//   70% full  = ~7.5 cm  -> yellow turns ON at 7 cm
//   90% full  = ~2.5 cm  -> red turns ON at 3 cm
//     (HC-SR04 cannot read reliably below 2 cm, so 3 cm
//      is the practical 90% mark)
//
// HYSTERESIS: leaving a state needs a BIGGER distance than
// entering it, so noise at the boundary cannot flip the LEDs.
// ==========================
const long RED_ENTER = 3;      // 90-100% full -> RED + buzzer
const long RED_EXIT = 5;       // leave RED only when distance > 5 cm
const long YELLOW_ENTER = 7;   // 70% full -> YELLOW
const long YELLOW_EXIT = 9;    // leave YELLOW only when distance > 9 cm

const unsigned long BEEP_INTERVAL = 3600000UL;  // 1 hour between beep cycles

// ==========================
// Bin state machine
// ==========================
enum BinState { NORMAL, ALMOST_FULL, FULL };
BinState binState = NORMAL;

unsigned long lastBeepCycle = 0;
unsigned long lastObjectDetected = 0;
bool lidOpen = false;

// ==========================
// Read one ultrasonic distance
// ==========================
long readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) {
    return 999;
  }
  return duration / 29 / 2;
}

// ==========================
// Median of 5 readings
// One noisy spike gets thrown away instead of
// changing the LED state.
// When bin is FULL, any reading above RED_EXIT
// is likely a sensor error at very close range.
// Force it to stay in FULL state.
// ==========================
long readDistanceMedian(int trigPin, int echoPin) {
  long r[5];
  for (int i = 0; i < 5; i++) {
    r[i] = readDistance(trigPin, echoPin);
    delay(30);
  }

  // Simple insertion sort
  for (int i = 1; i < 5; i++) {
    long key = r[i];
    int j = i - 1;
    while (j >= 0 && r[j] > key) {
      r[j + 1] = r[j];
      j--;
    }
    r[j + 1] = key;
  }

  long median = r[2];

  //
  if (binState == FULL && median > RED_EXIT && median <= YELLOW_EXIT) {
    return 1;
  }

  return median;
}

// ==========================
// Beep 3 times (~1.8 seconds)
// ==========================
void beepThreeTimes() {
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, 1000);
    delay(300);
    noTone(BUZZER_PIN);
    delay(300);
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  lidServo.attach(SERVO_PIN);
  lidServo.write(0);  // Lid starts closed
}

void loop() {

  // ==========================
  // Read sensors
  // ==========================
  long cm_capacity = readDistanceMedian(trigPin1, echoPin1);
  long cm_lid = readDistance(trigPin2, echoPin2);

  Serial.print("Capacity: ");
  Serial.print(cm_capacity);
  Serial.print(" cm | Lid: ");
  Serial.print(cm_lid);
  Serial.print(" cm | State: ");
  Serial.println(binState == FULL ? "FULL" : binState == ALMOST_FULL ? "ALMOST_FULL" : "NORMAL");

  // ==========================
  // CAPACITY STATE MACHINE (with hysteresis)
  // Matches the report's state diagram:
  // Idle -> Warning (yellow) -> Urgent (red + buzzer)
  // ==========================
  BinState previousState = binState;

  switch (binState) {

    case NORMAL:
      if (cm_capacity <= RED_ENTER) {
        binState = FULL;
      } else if (cm_capacity <= YELLOW_ENTER) {
        binState = ALMOST_FULL;
      }
      break;

    case ALMOST_FULL:
      if (cm_capacity <= RED_ENTER) {
        binState = FULL;
      } else if (cm_capacity > YELLOW_EXIT) {
        binState = NORMAL;
      }
      break;

    case FULL:
      if (cm_capacity > RED_EXIT) {
        if (cm_capacity > YELLOW_EXIT) {
          binState = NORMAL;
        } else {
          binState = ALMOST_FULL;
        }
      }
      break;
  }

  // ==========================
  // ACT ON STATE
  // ==========================
  digitalWrite(LED_YELLOW, binState == ALMOST_FULL ? HIGH : LOW);
  digitalWrite(LED_RED, binState == FULL ? HIGH : LOW);

  // Just ENTERED full: beep 3 times and start the hourly timer
  if (binState == FULL && previousState != FULL) {
    beepThreeTimes();
    lastBeepCycle = millis();
  }

  // Still full one hour later: beep 3 times again
  if (binState == FULL && (millis() - lastBeepCycle >= BEEP_INTERVAL)) {
    beepThreeTimes();
    lastBeepCycle = millis();
  }

  // ==========================
  // LID CONTROL SYSTEM
  // Opens within 25 cm, closes 5 s after object leaves
  // ==========================
  if (cm_lid <= 25) {
    if (!lidOpen) {
      lidServo.write(100);
      lidOpen = true;
    }
    lastObjectDetected = millis();
  }

  if (lidOpen && (millis() - lastObjectDetected >= 5000)) {
    lidServo.write(0);
    lidOpen = false;
  }

  delay(100);
}