#include <Wire.h>
#include "rgb_lcd.h"

// ------------ Paramètres matériels ------------
const byte TRIG_PIN   = 9;
const byte ECHO_PIN   = 8;
const byte BUTTON_PIN = 2;  // interrupt externe sur UNO

rgb_lcd lcd;

// ------------ Paramètres mesure ------------
const float TEMP_C = 20.0;                   // température approx. pour vitesse du son
const float SOUND_SPEED = 331.3 + 0.606*TEMP_C; // m/s
const unsigned long ECHO_TIMEOUT_US = 25000UL;  // ~4.3 m
const int SAMPLES_PER_POINT = 5;            // nb de lectures pour une moyenne
const int SAMPLE_SPACING_MS = 60;           // délai entre lectures
const int DELAY_BETWEEN_POINTS_MS = 300;    // délai entre point A et point B

// ------------ Gestion bouton (interruption + anti-rebond) ------------
volatile bool measureRequested = false;
volatile unsigned long lastButtonMs = 0;
const unsigned long DEBOUNCE_MS = 200;

void onButtonPress() {
  unsigned long now = millis();
  if (now - lastButtonMs > DEBOUNCE_MS) {
    measureRequested = true;
    lastButtonMs = now;
  }
}

// ------------ Fonctions capteur ------------
float readDistanceMeters() {
  // Envoi impulsion TRIG (10 us)
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Mesure de l'écho
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) {
    return NAN; // pas d'écho
  }

  // distance = (durée en s) * (vitesse du son) / 2
  float distance = (duration * 1e-6f) * (SOUND_SPEED / 2.0f); // en mètres
  return distance;
}

float readAveragedDistance(int n, int spacingMs, int& validCount) {
  float sum = 0.0f;
  validCount = 0;
  for (int i = 0; i < n; i++) {
    float d = readDistanceMeters();
    if (!isnan(d)) {
      sum += d;
      validCount++;
    }
    delay(spacingMs);
  }
  if (validCount == 0) return NAN;
  return sum / (float)validCount;
}

// ------------ Mesure de vitesse ------------
bool measureSpeed(float& speed_m_s, float& speed_km_h, bool& movingAway) {
  // Point A (moyenne)
  unsigned long tA0 = millis();
  int validA = 0;
  float dA = readAveragedDistance(SAMPLES_PER_POINT, SAMPLE_SPACING_MS, validA);
  unsigned long tA1 = millis();
  if (isnan(dA)) return false;

  delay(DELAY_BETWEEN_POINTS_MS);

  // Point B (moyenne)
  unsigned long tB0 = millis();
  int validB = 0;
  float dB = readAveragedDistance(SAMPLES_PER_POINT, SAMPLE_SPACING_MS, validB);
  unsigned long tB1 = millis();
  if (isnan(dB)) return false;

  // Δt approximé entre la fin de A et la fin de B
  float dt_s = (float)(tB1 - tA1) / 1000.0f;
  if (dt_s <= 0.0f) return false;

  float v = (dB - dA) / dt_s;   // m/s (positif si la distance augmente)
  movingAway = (v > 0.0f);
  speed_m_s = fabs(v);
  speed_km_h = speed_m_s * 3.6f;

  return true;
}

// ------------ Setup / Loop ------------
void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // bouton vers GND

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButtonPress, FALLING);

  lcd.begin(16, 2);
  //lcd.backlight();

  Serial.begin(115200);
  delay(200);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Vitesse HC-SR04");
  lcd.setCursor(0, 1);
  lcd.print("Appuyez bouton");
  Serial.println("Pret. Appuyez sur le bouton pour mesurer la vitesse.");
}

void loop() {
  if (measureRequested) {
    noInterrupts();
    measureRequested = false;
    interrupts();

    // Indication de mesure en cours
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Mesure en cours");

    float v_ms, v_kmh;
    bool away;
    bool ok = measureSpeed(v_ms, v_kmh, away);

    lcd.clear();
    if (!ok) {
      // Erreur capteur
      lcd.setCursor(0, 0);
      lcd.print("Erreur capteur");
      lcd.setCursor(0, 1);
      lcd.print("Reessayez");
      Serial.println("Erreur: mesure invalide (pas d'echo ou dt nul).");
    } else {
      // Affichages
      Serial.print("Vitesse: ");
      Serial.print(v_ms, 2);
      Serial.print(" m/s  (");
      Serial.print(v_kmh, 2);
      Serial.print(" km/h)  ");
      Serial.println(away ? "S'eloigne" : "Rapproche");

      // LCD: ligne 1 = m/s + km/h, ligne 2 = direction
      lcd.setCursor(0, 0);
      // 16 colonnes: "1.23m/s 4.4km/h" tient exactement sur 16 (8 + 1 + 7)
      lcd.print(String(v_ms, 2));
      lcd.print("m/s ");
      // ajuster format pour tenir
      char buf[8];
      dtostrf(v_kmh, 0, 1, buf);  // 1 décimale sur le LCD pour gagner de la place
      lcd.print(buf);
      lcd.print("km/h");

      lcd.setCursor(0, 1);
      lcd.print(away ? "S'eloigne     " : "Rapproche     ");
    }
  }

  // autre logique non bloquante ici si besoin
}
