#include <Arduino.h>

// --- Piny silników (z Twojego kodu) ---
const int pinDirTL = 15, pinPwmTL = 5;
const int pinDirTR = 11, pinPwmTR = 10;
const int pinDirBL = 19, pinPwmBL = 3;
const int pinDirBR = 16, pinPwmBR = 9;

// --- Piny enkoderów ---
const int pinEncATL = 2, pinEncBTL = 18;
const int pinEncATR = 12, pinEncBTR = 21;
const int pinEncABL = 4, pinEncBBL = 8;
const int pinEncABR = 6, pinEncBBR = 7;

// --- Liczniki impulsów ---
volatile long countTL = 0, countTR = 0, countBL = 0, countBR = 0;

// --- Przerwania enkoderów ---
void countPulseTL() { if (digitalRead(pinEncBTL)) countTL++; else countTL--; }
void countPulseTR() { if (digitalRead(pinEncBTR)) countTR++; else countTR--; }
void countPulseBL() { if (digitalRead(pinEncBBL)) countBL++; else countBL--; }
void countPulseBR() { if (digitalRead(pinEncBBR)) countBR++; else countBR--; }

// --- Konfiguracja pomiaru ---
const unsigned long sampleInterval = 20;  // ms
const int numSamples = 250;               // 5 sekund przy 100 Hz
unsigned long timeStamps[numSamples];
long dataTL[numSamples], dataTR[numSamples], dataBL[numSamples], dataBR[numSamples];

void setup() {
  Serial.begin(115200);
  delay(1000);

  // --- Ustawienia pinów silników ---
  pinMode(pinDirTL, OUTPUT); pinMode(pinPwmTL, OUTPUT);
  pinMode(pinDirTR, OUTPUT); pinMode(pinPwmTR, OUTPUT);
  pinMode(pinDirBL, OUTPUT); pinMode(pinPwmBL, OUTPUT);
  pinMode(pinDirBR, OUTPUT); pinMode(pinPwmBR, OUTPUT);

  // --- Kierunek jazdy (np. do przodu) ---
  digitalWrite(pinDirTL, HIGH);
  digitalWrite(pinDirTR, HIGH);
  digitalWrite(pinDirBL, HIGH);
  digitalWrite(pinDirBR, HIGH);

  // --- Enkodery ---
  pinMode(pinEncATL, INPUT_PULLUP); pinMode(pinEncBTL, INPUT_PULLUP);
  pinMode(pinEncATR, INPUT_PULLUP); pinMode(pinEncBTR, INPUT_PULLUP);
  pinMode(pinEncABL, INPUT_PULLUP); pinMode(pinEncBBL, INPUT_PULLUP);
  pinMode(pinEncABR, INPUT_PULLUP); pinMode(pinEncBBR, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pinEncATL), countPulseTL, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncATR), countPulseTR, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncABL), countPulseBL, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncABR), countPulseBR, RISING);

  Serial.println("Rozpoczynam pomiar za 2 sekundy...");
  delay(2000);

  // --- Start silników na max prędkość ---
  analogWrite(pinPwmTL, 255);
  analogWrite(pinPwmTR, 255);
  analogWrite(pinPwmBL, 255);
  analogWrite(pinPwmBR, 255);

  Serial.println("Pomiar rozpoczęty...");
  unsigned long t0 = millis();

  // --- Zbieranie próbek ---
  for (int i = 0; i < numSamples; i++) {
    unsigned long t = millis() - t0;
    noInterrupts();
    timeStamps[i] = t;
    dataTL[i] = countTL;
    dataTR[i] = countTR;
    dataBL[i] = countBL;
    dataBR[i] = countBR;
    interrupts();
    delay(sampleInterval);
  }

  // --- Zatrzymanie silników ---
  analogWrite(pinPwmTL, 0);
  analogWrite(pinPwmTR, 0);
  analogWrite(pinPwmBL, 0);
  analogWrite(pinPwmBR, 0);
  Serial.println("Pomiar zakonczony.");

  // --- Wypisanie danych ---
  Serial.println("time_ms,TL,TR,BL,BR");
  for (int i = 0; i < numSamples; i++) {
    Serial.print(timeStamps[i]); Serial.print(",");
    Serial.print(dataTL[i]); Serial.print(",");
    Serial.print(dataTR[i]); Serial.print(",");
    Serial.print(dataBL[i]); Serial.print(",");
    Serial.println(dataBR[i]);
  }

  Serial.println("Koniec danych.");
}

void loop() {
  // Nic nie robimy — pomiar już wykonany
}
