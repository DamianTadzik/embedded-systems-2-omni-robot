#include <Arduino.h>

// --- Definicje pinów enkoderów (z Twojego kodu) ---
const int pinEncATL = 2;   // Top Left - kanał A
const int pinEncBTL = 18;  // Top Left - kanał B

const int pinEncATR = 12;  // Top Right - kanał A
const int pinEncBTR = 21;  // Top Right - kanał B

const int pinEncABL = 4;   // Bottom Left - kanał A
const int pinEncBBL = 8;   // Bottom Left - kanał B

const int pinEncABR = 6;   // Bottom Right - kanał A
const int pinEncBBR = 7;   // Bottom Right - kanał B

// --- Liczniki impulsów ---
volatile long countTL = 0;
volatile long countTR = 0;
volatile long countBL = 0;
volatile long countBR = 0;

// --- Funkcje przerwań (zliczanie impulsów) ---
void countPulseTL() {
  if (digitalRead(pinEncBTL)) countTL++;
  else countTL--;
}
void countPulseTR() {
  if (digitalRead(pinEncBTR)) countTR++;
  else countTR--;
}
void countPulseBL() {
  if (digitalRead(pinEncBBL)) countBL++;
  else countBL--;
}
void countPulseBR() {
  if (digitalRead(pinEncBBR)) countBR++;
  else countBR--;
}

void setup() {
  Serial.begin(9600);

  // --- Konfiguracja pinów enkoderów ---
  pinMode(pinEncATL, INPUT_PULLUP);
  pinMode(pinEncBTL, INPUT_PULLUP);
  pinMode(pinEncATR, INPUT_PULLUP);
  pinMode(pinEncBTR, INPUT_PULLUP);
  pinMode(pinEncABL, INPUT_PULLUP);
  pinMode(pinEncBBL, INPUT_PULLUP);
  pinMode(pinEncABR, INPUT_PULLUP);
  pinMode(pinEncBBR, INPUT_PULLUP);

  // --- Przypisanie przerwań do kanałów A (RISING) ---
  attachInterrupt(digitalPinToInterrupt(pinEncATL), countPulseTL, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncATR), countPulseTR, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncABL), countPulseBL, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncABR), countPulseBR, RISING);

  Serial.println("Odczyt enkoderów uruchomiony...");
}

void loop() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();

    noInterrupts();
    long TL = countTL;
    long TR = countTR;
    long BL = countBL;
    long BR = countBR;
    interrupts();

    Serial.print("ENC,");
    Serial.print(TL); Serial.print(",");
    Serial.print(TR); Serial.print(",");
    Serial.print(BL); Serial.print(",");
    Serial.println(BR);
  }
}
