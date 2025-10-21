#include <Arduino.h>

// Definicje pinów
const int pinDirTL = 15, pinPwmTL = 5, pinEncATL = 2, pinEncBTL = 18;
const int pinDirTR = 11, pinPwmTR = 10, pinEncATR = 12, pinEncBTR = 21;

const int pinDirBL = 19, pinPwmBL = 3, pinEncABL = 4, pinEncBBL = 8;
const int pinDirBR = 16, pinPwmBR = 9, pinEncABR = 6, pinEncBBR = 7;

int distance = 0;
int expectedDataSize = 0;
int dataBuffer[9];
int dataIndex = 0;
int lastReceivedByte = 1000;

// Telemetry timing
unsigned long lastTelemetryTime = 0;
const unsigned long telemetryInterval = 10; // [10ms] how often to print counts

// Liczniki impulsów dla enkoderów
volatile int countTL = 0, countTR = 0, countBL = 0, countBR = 0;
bool advancedControl = false;
bool advancedControl_stop = false;

// Callbacks
void countPulseTL() {
  if (digitalRead(pinEncBTL)) countTL++;  // forward
  else countTL--;                          // backward
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

// Globalne zmienne do regulatora 
volatile float RPMSetpointTL = 0.0, RPMSetpointTR = 0.0, RPMSetpointBL = 0.0, RPMSetpointBR = 0.0;


// Globalne zmienne do przechowywania prędkości silników /XD
int currentSpeedTL = 0, currentSpeedTR = 0, currentSpeedBL = 0, currentSpeedBR = 0;
int targetSpeedTL = 0, targetSpeedTR = 0, targetSpeedBL = 0, targetSpeedBR = 0;
const int speedIncrement = 1; // Krok zmiany prędkości

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

  // Ustawienie przerwań dla enkoderów
  pinMode(pinEncATL, INPUT_PULLUP);
  pinMode(pinEncBTL, INPUT_PULLUP);
  pinMode(pinEncATR, INPUT_PULLUP);
  pinMode(pinEncBTR, INPUT_PULLUP);
  pinMode(pinEncABL, INPUT_PULLUP);
  pinMode(pinEncBBL, INPUT_PULLUP);
  pinMode(pinEncABR, INPUT_PULLUP);
  pinMode(pinEncBBR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncATL), countPulseTL, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncATR), countPulseTR, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncABL), countPulseBL, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncABR), countPulseBR, RISING);

  Serial.begin(9600);
}


void setMotorSpeed(int TL, int TR, int BL, int BR) {
 analogWrite(pinPwmTL, TL);
 analogWrite(pinPwmTR, TR);
 analogWrite(pinPwmBL, BL);
 analogWrite(pinPwmBR, BR);
}


void setMotorDirection(bool TL, bool TR, bool BL, bool BR) {
 digitalWrite(pinDirTL, TL);
 digitalWrite(pinDirTR, TR);
 digitalWrite(pinDirBL, BL);
 digitalWrite(pinDirBR, BR);
}


void stopMotors() {
 countTL = countTR = countBL = countBR = 0;
 targetSpeedTL = 0, targetSpeedTR = 0, targetSpeedBL = 0, targetSpeedBR = 0;
 delay(10);
}

void handleComplexCommand(int distance_, int speedTL, int speedTR, int speedBL, int speedBR, int dirTL, int dirTR, int dirBL, int dirBR) {
 advancedControl = true;
 distance = distance_ *10;
 setMotorDirection(dirTL == 1 ? HIGH : LOW, dirTR == 1 ? HIGH : LOW, dirBL == 1 ? HIGH : LOW, dirBR == 1 ? HIGH : LOW);
 targetSpeedTL = speedTL;
 targetSpeedTR = speedTR;
 targetSpeedBL = speedBL;
 targetSpeedBR = speedBR;
 
}


void handleTwoSpeedCommand(int distance_, int speedTL, int speedTR, int speedBL, int speedBR, int dirTL, int dirTR, int dirBL, int dirBR) {
    setMotorDirection(dirTL == 1 ? HIGH : LOW, dirTR == 1 ? HIGH : LOW, dirBL == 1 ? HIGH : LOW, dirBR == 1 ? HIGH : LOW);
    targetSpeedTL = speedTL;
    targetSpeedTR = speedTR;
    targetSpeedBL = speedBL;
    targetSpeedBR = speedBR;
}

void updateMotorSpeeds() {
    bool speedUpdated = false; // Flaga do śledzenia aktualizacji prędkości

    // Aktualizuj prędkość TL
    if (currentSpeedTL != targetSpeedTL) {
        if (currentSpeedTL < targetSpeedTL) {
            currentSpeedTL += speedIncrement;
            if (currentSpeedTL > targetSpeedTL) currentSpeedTL = targetSpeedTL;
        } else {
            currentSpeedTL -= speedIncrement;
            if (currentSpeedTL < targetSpeedTL) currentSpeedTL = targetSpeedTL;
        }
        speedUpdated = true;
    }

    // Aktualizuj prędkość TR
    if (currentSpeedTR != targetSpeedTR) {
        if (currentSpeedTR < targetSpeedTR) {
            currentSpeedTR += speedIncrement;
            if (currentSpeedTR > targetSpeedTR) currentSpeedTR = targetSpeedTR;
        } else {
            currentSpeedTR -= speedIncrement;
            if (currentSpeedTR < targetSpeedTR) currentSpeedTR = targetSpeedTR;
        }
        speedUpdated = true;
    }

    // Aktualizuj prędkość BL
    if (currentSpeedBL != targetSpeedBL) {
        if (currentSpeedBL < targetSpeedBL) {
            currentSpeedBL += speedIncrement;
            if (currentSpeedBL > targetSpeedBL) currentSpeedBL = targetSpeedBL;
        } else {
            currentSpeedBL -= speedIncrement;
            if (currentSpeedBL < targetSpeedBL) currentSpeedBL = targetSpeedBL;
        }
        speedUpdated = true;
    }

    // Aktualizuj prędkość BR
    if (currentSpeedBR != targetSpeedBR) {
        if (currentSpeedBR < targetSpeedBR) {
            currentSpeedBR += speedIncrement;
            if (currentSpeedBR > targetSpeedBR) currentSpeedBR = targetSpeedBR;
        } else {
            currentSpeedBR -= speedIncrement;
            if (currentSpeedBR < targetSpeedBR) currentSpeedBR = targetSpeedBR;
        }
        speedUpdated = true;
    }

    // Ustaw prędkości silników tylko jeśli zostały zaktualizowane
    if (speedUpdated) {
        setMotorSpeed(currentSpeedTL, currentSpeedTR, currentSpeedBL, currentSpeedBR);
    }
}

void loop() {
  // Warunek zatrzymania silników dla trybu zaawansowanego sterowania
  if (advancedControl && (countTL >= distance || countTR >= distance || countBL >= distance || countBR >= distance)) {
    stopMotors();
    delay(10);
    advancedControl = false; // Wyłączenie trybu zaawansowanego sterowania
    if (lastReceivedByte == 250) {
      advancedControl_stop = true; // Ustawienie flagi tylko jeśli ostatni otrzymany bajt to 250
      delay(10);
      lastReceivedByte = 1000;
    }
  }
  if (advancedControl_stop) {
    delay(10);
    Serial.println(advancedControl_stop ? "true" : "false");
    delay(10);
    advancedControl_stop = false;
  }

  updateMotorSpeeds();

  while (Serial.available() > 0) {
    int receivedByte = Serial.read();
    if (dataIndex == 0 && (receivedByte == 250 || receivedByte == 240 || receivedByte == 255 || receivedByte == 245)) {
      lastReceivedByte = receivedByte;
      dataIndex = 1;  // Zaczynamy zbierać dane komendy
      if (receivedByte == 240) {
        expectedDataSize = 9;  // Prędkość1 + prędkość2
      } else if (receivedByte == 250){
        expectedDataSize = 9;
      } else if (receivedByte == 255){
        expectedDataSize = 1;  // Komenda + prędkość + odległość
      } else if (receivedByte == 245) {
        expectedDataSize = 4; // 4 speeds (int8_t)
      } else {
        //nic
      }
    } else if (dataIndex > 0) {
      dataBuffer[dataIndex - 1] = receivedByte;  // Zapisujemy dane do bufora
      dataIndex++;

      // Jeśli otrzymaliśmy wszystkie dane komendy
      if (dataIndex == expectedDataSize + 1) {
        if (lastReceivedByte == 240) {
          countTL = countTR = countBL = countBR = 0;
          handleTwoSpeedCommand(dataBuffer[0], dataBuffer[1], dataBuffer[2], dataBuffer[3], dataBuffer[4], dataBuffer[5], dataBuffer[6], dataBuffer[7],dataBuffer[8]);  // prędkości i kierunki
        } else if (lastReceivedByte == 250){
          countTL = countTR = countBL = countBR = 0;
          handleComplexCommand(dataBuffer[0], dataBuffer[1], dataBuffer[2], dataBuffer[3], dataBuffer[4], dataBuffer[5], dataBuffer[6], dataBuffer[7],dataBuffer[8]);  // Obsługujemy złożoną komendę
        } else if (lastReceivedByte == 255){
          countTL = countTR = countBL = countBR = 0;
          stopMotors(); 
        } else if (lastReceivedByte == 245) {
          // interpret signed speeds
          int8_t sTL = (int8_t)dataBuffer[0];
          int8_t sTR = (int8_t)dataBuffer[1];
          int8_t sBL = (int8_t)dataBuffer[2];
          int8_t sBR = (int8_t)dataBuffer[3];

          // store as target speeds for the PID controller
          RPMSetpointTL = (float)sTL / 128.0;
          RPMSetpointTR = (float)sTR / 128.0;
          RPMSetpointBL = (float)sBL / 128.0;
          RPMSetpointBR = (float)sBR / 128.0;
        } else {
          //nic
        }
        dataIndex = 0;  // Resetujemy indeks danych
      }
    } else {
      // nic
    }
  }

  // Periodic telemetry
  if (millis() - lastTelemetryTime >= telemetryInterval) {
    noInterrupts();  // protect counts during read
    int TL = countTL;
    int TR = countTR;
    int BL = countBL;
    int BR = countBR;
    interrupts();

    Serial.print("ENC,");
    Serial.print(TL); Serial.print(",");
    Serial.print(TR); Serial.print(",");
    Serial.print(BL); Serial.print(",");
    Serial.println(BR);

    lastTelemetryTime = millis();
  }
  delay(10);
}
