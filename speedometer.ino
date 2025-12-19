#include <Wire.h>
#include "rgb_lcd.h"

// ------------ LCD Grove RGB ------------
rgb_lcd lcd;  // I2C (0x3E pour LCD et 0x62 pour RGB gérés par la lib)

// ------------ Broches ------------
const byte TRIG_PIN   = 9;
const byte ECHO_PIN   = 8;
const byte BUTTON_PIN = 2;   // bouton vers GND, INPUT_PULLUP

// ------------ Paramètres mesure ------------
const float TEMP_C = 20.0;
const float SOUND_SPEED = 331.3 + 0.606 * TEMP_C;   // m/s
const unsigned long ECHO_TIMEOUT_US = 25000UL;      // timeout pulseIn (~4.3 m)
const int SAMPLES_PER_POINT = 5;                    // moyenne de N lectures pour lisser
const int SAMPLE_SPACING_MS = 60;                   // espacement entre lectures
const float SPEED_NOISE_FLOOR_MS = 0.05;            // < seuil => affiché 0

// ------------ Etat / bouton ------------
enum State { IDLE, MEASURING, SHOW_MAX };
State state = IDLE;

volatile bool buttonEdge = false;  // mis à true par l'ISR quand un front arrive
bool useInterrupts = false;

static bool stableBtn = HIGH;       // INPUT_PULLUP : HIGH=relâché
static bool lastReadBtn = HIGH;
static unsigned long lastChangeMs = 0;
const unsigned long DEBOUNCE_MS = 30;

// ------------ Variables de mesure continue ------------
bool avgInProgress = false;
int sampleCount = 0;
int validCount = 0;
float sumDistance = 0.0f;
unsigned long nextSampleMs = 0;

float prevAvg = NAN;
unsigned long prevAvgTime = 0;

float currentSpeed_ms = 0.0f;  // instantané (absolu)
float maxSpeed_ms = 0.0f;

// ------------ Protos ------------
void onButtonChange();
void updateButton(unsigned long now);
void startMeasuring();
void stopMeasuringAndShowMax();
void measuringStep(unsigned long now);

float readDistanceMeters();

// ------------ ISR bouton ------------
void onButtonChange() {
  buttonEdge = true; // ISR minimale
}

// ------------ Setup ------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  delay(100);

  int intr = digitalPinToInterrupt(BUTTON_PIN);
  if (intr != NOT_AN_INTERRUPT) {
    attachInterrupt(intr, onButtonChange, CHANGE); // capte appui et relâchement
    useInterrupts = true;
    Serial.println("Interruption active sur CHANGE.");
  } else {
    useInterrupts = false;
    Serial.println("Pas d'interruption dispo: mode polling.");
  }

  // Init LCD Grove RGB
  lcd.begin(16, 2);
  lcd.setRGB(255, 255, 255); // blanc
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Vitesse HC-SR04");
  lcd.setCursor(0, 1); lcd.print("Appuyez & maint.");
  Serial.println("Pret. Maintenez le bouton pour mesurer; relachez pour afficher le max.");
}

// ------------ Loop ------------
void loop() {
  unsigned long now = millis();

  // Gestion bouton (debounce + détection de front)
  if (useInterrupts ? buttonEdge : true) {
    noInterrupts(); buttonEdge = false; interrupts();
    updateButton(now);
  } else {
    updateButton(now);
  }

  // Machine à états
  switch (state) {
    case IDLE:
      // attend appui
      break;

    case MEASURING:
      measuringStep(now);
      break;

    case SHOW_MAX:
      // attend un nouvel appui pour relancer
      break;
  }
}

// ------------ Gestion du bouton avec anti-rebond ------------
void updateButton(unsigned long now) {
  bool raw = digitalRead(BUTTON_PIN); // HIGH = relâché, LOW = appui

  if (raw != lastReadBtn) {
    lastReadBtn = raw;
    lastChangeMs = now;
  }

  if ((now - lastChangeMs) > DEBOUNCE_MS) {
    if (stableBtn != raw) {
      stableBtn = raw;

      if (stableBtn == LOW) {
        // Front descendant: appui confirmé
        if (state != MEASURING) {
          startMeasuring();
        }
      } else {
        // Front montant: relâchement confirmé
        if (state == MEASURING) {
          stopMeasuringAndShowMax();
        }
      }
    }
  }
}

// ------------ Démarrage / arrêt ------------
void startMeasuring() {
  state = MEASURING;
  digitalWrite(LED_BUILTIN, HIGH);

  avgInProgress = false;
  sampleCount = validCount = 0;
  sumDistance = 0.0f;
  nextSampleMs = 0;
  prevAvg = NAN;
  prevAvgTime = 0;
  currentSpeed_ms = 0.0f;
  maxSpeed_ms = 0.0f;

  lcd.setRGB(255, 200, 0); // jaune: mesure en cours
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Mesure en cours");
  lcd.setCursor(0, 1); lcd.print("Max 0.0 km/h   ");

  Serial.println("Mesure continue: maintenez le bouton...");
}

void stopMeasuringAndShowMax() {
  state = SHOW_MAX;
  digitalWrite(LED_BUILTIN, LOW);

  float max_kmh = maxSpeed_ms * 3.6f;

  lcd.setRGB(0, 200, 100); // vert: résultat final
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MAX atteint:");
  lcd.setCursor(0, 1);
  char buf[16];
  char msBuf[8], kmhBuf[8];
  dtostrf(maxSpeed_ms, 0, 2, msBuf);
  dtostrf(max_kmh,    0, 1, kmhBuf);
  snprintf(buf, sizeof(buf), "%sm/s %skm/h", msBuf, kmhBuf);
  lcd.print(buf);

  Serial.print("Max: ");
  Serial.print(maxSpeed_ms, 2); Serial.print(" m/s (");
  Serial.print(max_kmh, 1); Serial.println(" km/h)");
  Serial.println("Relancez: maintenez le bouton.");
}

// ------------ Etape de mesure continue (non bloquante) ------------
void measuringStep(unsigned long now) {
  // Lance une nouvelle rafale de moyenne si nécessaire
  if (!avgInProgress) {
    avgInProgress = true;
    sampleCount = 0;
    validCount = 0;
    sumDistance = 0.0f;
    nextSampleMs = 0; // prend tout de suite une 1ere mesure
  }

  // Echantillonnage espacé pour la moyenne
  if (nextSampleMs == 0 || now >= nextSampleMs) {
    float d = readDistanceMeters();
    if (!isnan(d)) {
      sumDistance += d;
      validCount++;
    }
    sampleCount++;
    nextSampleMs = now + SAMPLE_SPACING_MS;

    // Fin de rafale -> calcul de la vitesse instantanée
    if (sampleCount >= SAMPLES_PER_POINT) {
      float avg = NAN;
      if (validCount > 0) {
        avg = sumDistance / (float)validCount;
      }

      unsigned long t = millis();
      if (!isnan(avg) && !isnan(prevAvg)) {
        float dt = (float)(t - prevAvgTime) / 1000.0f;
        if (dt > 0.0f) {
          float v = (avg - prevAvg) / dt; // m/s (+ si l'objet s'éloigne)
          float a = fabs(v);
          if (a < SPEED_NOISE_FLOOR_MS) a = 0.0f;

          currentSpeed_ms = a;
          if (currentSpeed_ms > maxSpeed_ms) {
            maxSpeed_ms = currentSpeed_ms;
          }

          float cur_kmh = currentSpeed_ms * 3.6f;
          float max_kmh = maxSpeed_ms * 3.6f;

          // Ligne 0: "x.xxm/s y.ykm/h"
          lcd.setCursor(0, 0);
          char l0[17];
          char msBuf[8], kmhBuf[8];
          dtostrf(currentSpeed_ms, 0, 2, msBuf);
          dtostrf(cur_kmh,        0, 1, kmhBuf);
          snprintf(l0, sizeof(l0), "%sm/s %skm/h", msBuf, kmhBuf);
          lcd.print("                ");
          lcd.setCursor(0, 0);
          lcd.print(l0);

          // Ligne 1: "Max x.x km/h"
          lcd.setCursor(0, 1);
          char l1[17];
          char maxBuf[8];
          dtostrf(max_kmh, 0, 1, maxBuf);
          snprintf(l1, sizeof(l1), "Max %s km/h", maxBuf);
          lcd.print("                ");
          lcd.setCursor(0, 1);
          lcd.print(l1);

          // Série (optionnel)
          Serial.print("Instant: ");
          Serial.print(currentSpeed_ms, 2); Serial.print(" m/s (");
          Serial.print(cur_kmh, 1); Serial.print(" km/h) | Max: ");
          Serial.print(maxSpeed_ms, 2); Serial.print(" m/s (");
          Serial.print(max_kmh, 1); Serial.println(" km/h)");
        }
      }

      prevAvg = avg;
      prevAvgTime = t;

      // prête pour une nouvelle rafale
      avgInProgress = false;
    }
  }
}

// ------------ Lecture distance HC-SR04 ------------
float readDistanceMeters() {
  // Impulsion TRIG 10 µs
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Durée de l'écho
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return NAN;

  // distance = (durée s) * vitesse du son / 2
  float distance = (duration * 1e-6f) * (SOUND_SPEED / 2.0f);
  return distance; // en mètres
}
