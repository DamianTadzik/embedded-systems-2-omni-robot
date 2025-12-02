#include <Arduino.h>

// === Definicje pinów ===
const int pinDirTL = 15, pinPwmTL = 5, pinEncATL = 2, pinEncBTL = 18;
const int pinDirTR = 11, pinPwmTR = 10, pinEncATR = 12, pinEncBTR = 21;
const int pinDirBL = 19, pinPwmBL = 3, pinEncABL = 4, pinEncBBL = 8;
const int pinDirBR = 16, pinPwmBR = 9, pinEncABR = 6, pinEncBBR = 7;

// === Enkodery === // liczba impulsów na obrót enkodera: 240
volatile int32_t countTL = 0, countTR = 0, countBL = 0, countBR = 0;
void countPulseTL() { if (digitalRead(pinEncBTL)) countTL++; else countTL--; }
void countPulseTR() { if (digitalRead(pinEncBTR)) countTR++; else countTR--; }
void countPulseBL() { if (digitalRead(pinEncBBL)) countBL++; else countBL--; }
void countPulseBR() { if (digitalRead(pinEncBBR)) countBR++; else countBR--; }

// === Timeout guard ===
unsigned long lastCommand = 0;

void setup() {
  // Ustawienie pinów jako wyjścia
  pinMode(pinDirTL, OUTPUT);
  pinMode(pinPwmTL, OUTPUT);
  pinMode(pinDirTR, OUTPUT);
  pinMode(pinPwmTR, OUTPUT);
  pinMode(pinDirBL, OUTPUT);
  pinMode(pinPwmBL, OUTPUT);
  pinMode(pinDirBR, OUTPUT);
  pinMode(pinPwmBR, OUTPUT);
  // Ustawienie pinów enkoderów jako wejścia z podciąganiem
  pinMode(pinEncATL, INPUT_PULLUP);
  pinMode(pinEncBTL, INPUT_PULLUP);
  pinMode(pinEncATR, INPUT_PULLUP);
  pinMode(pinEncBTR, INPUT_PULLUP);
  pinMode(pinEncABL, INPUT_PULLUP);
  pinMode(pinEncBBL, INPUT_PULLUP);
  pinMode(pinEncABR, INPUT_PULLUP);
  pinMode(pinEncBBR, INPUT_PULLUP);
  // Ustawienie funkcji do obsługi przerwań dla enkoderów
  attachInterrupt(digitalPinToInterrupt(pinEncATL), countPulseTL, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncATR), countPulseTR, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncABL), countPulseBL, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncABR), countPulseBR, RISING);
  // Inicjalizacja komunikacji szeregowej
  Serial.begin(230400);
  lastCommand = millis();
}


// === Funkcja do sterowania silnikami ===
// TL/TR/BL/BR ∈ [-127, 127] → kierunek i PWM
// 0 = stop, 127 = pełna prędkość do przodu, -127 = pełna prędkość do tyłu 
void actuateMotors(int8_t TL, int8_t TR, int8_t BL, int8_t BR) {
  // --- TOP LEFT ---
  digitalWrite(pinDirTL, (TL >= 0) ? HIGH : LOW);
  analogWrite(pinPwmTL, abs(TL)<<1); // LSHFT skalowanie do [0, 254]

  // --- TOP RIGHT ---
  digitalWrite(pinDirTR, (TR >= 0) ? HIGH : LOW);
  analogWrite(pinPwmTR, abs(TR)<<1);

  // --- BOTTOM LEFT ---
  digitalWrite(pinDirBL, (BL >= 0) ? HIGH : LOW);
  analogWrite(pinPwmBL, abs(BL)<<1);

  // --- BOTTOM RIGHT ---
  digitalWrite(pinDirBR, (BR >= 0) ? HIGH : LOW);
  analogWrite(pinPwmBR, abs(BR)<<1);
}


bool readControlFrame(int8_t* wheelCmd) {
  // Sprawdzenie dostępności danych
  if (Serial.available() < 6) return false;
  
  // Odczyt nagłówka
  uint8_t header = Serial.read();
  if (header != 0xAA) return false;

  // Odczyt ramki
  uint8_t data[5] = {0};
  for (int i = 0; i < 5; i++) {
    data[i] = Serial.read();
  }

  // Kalkulacja sumy kontrolnej
  uint8_t checksum = 0xAA;
  for (int i = 0; i < 4; i++) {
    checksum += data[i];
  }

  // Sprawdzenie sumy kontrolnej
  if (checksum != 4[data]) return false;

  // Wypełnienie komend dla kół
  wheelCmd[0] = (int8_t)(data[0]); // TL
  wheelCmd[1] = (int8_t)(data[1]); // TR
  wheelCmd[2] = (int8_t)(data[2]); // BL
  wheelCmd[3] = (int8_t)(data[3]); // BR

  return true;
}

void sendEncodersFrame() {
  uint16_t tl, tr, bl, br;
  noInterrupts(); // Blokada przerwań podczas odczytu liczników
  tl = (uint16_t)(countTL & 0xFFFF);
  tr = (uint16_t)(countTR & 0xFFFF);
  bl = (uint16_t)(countBL & 0xFFFF);
  br = (uint16_t)(countBR & 0xFFFF);
  interrupts(); // Odblokowanie przerwań

  // Konstrukcja ramki
  uint8_t frame[10];
  frame[0] = 0xAA; // Nagłówek
  frame[1] = (uint8_t)(tl & 0xFF);
  frame[2] = (uint8_t)((tl >> 8) & 0xFF);
  frame[3] = (uint8_t)(tr & 0xFF);
  frame[4] = (uint8_t)((tr >> 8) & 0xFF);
  frame[5] = (uint8_t)(bl & 0xFF);
  frame[6] = (uint8_t)((bl >> 8) & 0xFF);
  frame[7] = (uint8_t)(br & 0xFF);
  frame[8] = (uint8_t)((br >> 8) & 0xFF);

  // Kalkulacja sumy kontrolnej
  uint8_t checksum = 0x55;
  for (int i = 1; i <= 8; i++) {
    checksum += frame[i];
  }
  frame[9] = checksum & 0xFF;

  // Wysłanie ramki
  Serial.write(frame, 10);
}


void loop() {
  int8_t wheelCmd[4];

  if (readControlFrame(wheelCmd))
  {
    // Aktualizacja czasu ostatniej komendy
    lastCommand = millis(); 

    // Sterowanie silnikami
    actuateMotors(wheelCmd[0], wheelCmd[1], wheelCmd[2], wheelCmd[3]);

    // Wysłanie danych z enkoderów
    sendEncodersFrame();
  }

  // Safety stop if no command received for over 1 second
  if (millis() - lastCommand > 1000)
  {
    actuateMotors(0, 0, 0, 0); // Stop all motors
  }
}
