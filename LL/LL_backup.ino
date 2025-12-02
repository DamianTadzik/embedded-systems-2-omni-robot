#include <Arduino.h>

// === Definicje pinów ===
const int pinDirTL = 15, pinPwmTL = 5, pinEncATL = 2, pinEncBTL = 18;
const int pinDirTR = 11, pinPwmTR = 10, pinEncATR = 12, pinEncBTR = 21;
const int pinDirBL = 19, pinPwmBL = 3, pinEncABL = 4, pinEncBBL = 8;
const int pinDirBR = 16, pinPwmBR = 9, pinEncABR = 6, pinEncBBR = 7;

// === Timery ===
unsigned long lastTelemetryTime = 0;
const unsigned long telemetryInterval = 1000; // ms

unsigned long lastRegulationTime = 0;
const unsigned long regulationInterval = 100; // ms

unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 1000; // ms, brak komendy → stop

// === Enkodery ===
volatile int32_t countTL = 0, countTR = 0, countBL = 0, countBR = 0;
volatile int32_t prev_count_TL = 0, prev_count_TR = 0, prev_count_BL = 0, prev_count_BR = 0;
// --- Parametry enkoderów ---
const float encoderPulsesPerRevolution = 240.0;
const float regulationInterval_s = regulationInterval / 1000.0; // 0.1 s
void countPulseTL() { if (digitalRead(pinEncBTL)) countTL++; else countTL--; }
void countPulseTR() { if (digitalRead(pinEncBTR)) countTR++; else countTR--; }
void countPulseBL() { if (digitalRead(pinEncBBL)) countBL++; else countBL--; }
void countPulseBR() { if (digitalRead(pinEncBBR)) countBR++; else countBR--; }

// === Regulator / sterowanie ===
// To jest requested speed setpoint
volatile int requested_speed_setpoint_TL = 0, requested_speed_setpoint_TR = 0, requested_speed_setpoint_BL = 0, requested_speed_setpoint_BR = 0;

// To jest calculated speed from encoder
volatile float current_speed_TL = 0.0, current_speed_TR = 0.0,  current_speed_BL = 0.0, current_speed_BR = 0.0;  // finish that TODO

// To leci na silniki calculated dutycycle
volatile int calculated_dutycycle_TL = 0, calculated_dutycycle_TR = 0, calculated_dutycycle_BL = 0, calculated_dutycycle_BR = 0;

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


// === Funkcja do sterowania silnikami ===
// TL/TR/BL/BR ∈ [-255, 255] → kierunek i PWM
void actuateMotors(int TL, int TR, int BL, int BR) {
  // --- TOP LEFT ---
  digitalWrite(pinDirTL, (TL >= 0) ? HIGH : LOW);
  analogWrite(pinPwmTL, abs(TL));

  // --- TOP RIGHT ---
  digitalWrite(pinDirTR, (TR >= 0) ? HIGH : LOW);
  analogWrite(pinPwmTR, abs(TR));

  // --- BOTTOM LEFT ---
  digitalWrite(pinDirBL, (BL >= 0) ? HIGH : LOW);
  analogWrite(pinPwmBL, abs(BL));

  // --- BOTTOM RIGHT ---
  digitalWrite(pinDirBR, (BR >= 0) ? HIGH : LOW);
  analogWrite(pinPwmBR, abs(BR));
}

// === UART ramka ===
// 255 param setpointspeedTL setpointspeedTR setpointspeedBL setpointspeedBR
#define PAYLOAD_BUF_LEN 5
uint8_t payload_buffer[PAYLOAD_BUF_LEN] = { 0 };
int index = 0;

// === Dekodowanie ramki ===
void handle_payload_buffer(void)
{
  int mult = payload_buffer[0];
  requested_speed_setpoint_TL = ((int)payload_buffer[1] - 100) * mult;
  requested_speed_setpoint_TR = ((int)payload_buffer[2] - 100) * mult;
  requested_speed_setpoint_BL = ((int)payload_buffer[3] - 100) * mult;
  requested_speed_setpoint_BR = ((int)payload_buffer[4] - 100) * mult;
}

// === Obliczanie prędkości z enkoderów ===
void calculate_current_encoder_speeds(void)
{
  noInterrupts();
  int32_t cur_TL = countTL;
  int32_t cur_TR = countTR;
  int32_t cur_BL = countBL;
  int32_t cur_BR = countBR;
  interrupts();

  // Zmiana liczby impulsów od ostatniego pomiaru
  int32_t dTL = cur_TL - prev_count_TL;
  int32_t dTR = cur_TR - prev_count_TR;
  int32_t dBL = cur_BL - prev_count_BL;
  int32_t dBR = cur_BR - prev_count_BR;

  prev_count_TL = cur_TL;
  prev_count_TR = cur_TR;
  prev_count_BL = cur_BL;
  prev_count_BR = cur_BR;

  // Prędkość w [obr/min]
  float conv = (60.0 / encoderPulsesPerRevolution) / regulationInterval_s; // (imp / rev) * (interval)
  current_speed_TL = dTL * conv;
  current_speed_TR = dTR * conv;
  current_speed_BL = dBL * conv;
  current_speed_BR = dBR * conv;
}

void loop() {
  // --- UART read ---
  while (Serial.available() > 0) 
  {
    int received_byte = Serial.read();
    
    if (received_byte == 255)  // Frame start byte xdddd
      index = 0;
    else if (index < PAYLOAD_BUF_LEN) {
      payload_buffer[index++] = received_byte;
      if (index == PAYLOAD_BUF_LEN) {
        handle_payload_buffer();
        lastCommandTime = millis();
      }
    }
  }

  // Periodic speed, regualtor and other actuations calculation
  if (millis() - lastRegulationTime >= regulationInterval){
    lastRegulationTime = millis();

    // Compute the speeds from the encoders
    calculate_current_encoder_speeds();

    // Manually select the regulator or just feedforward
    if (false)
    {
      // --- Simple P regulator ---
      const float Kp = 1.2f;  // tune it later xd

    // Compute the error, Compute the regulator output xd
      float err_TL = requested_speed_setpoint_TL - current_speed_TL;
      float err_TR = requested_speed_setpoint_TR - current_speed_TR;
      float err_BL = requested_speed_setpoint_BL - current_speed_BL;
      float err_BR = requested_speed_setpoint_BR - current_speed_BR;

      calculated_dutycycle_TL = constrain(-(int)(Kp * err_TL), -255, 255);
      calculated_dutycycle_TR = constrain((int)(Kp * err_TR), -255, 255);
      calculated_dutycycle_BL = constrain(-(int)(Kp * err_BL), -255, 255);
      calculated_dutycycle_BR = constrain((int)(Kp * err_BR), -255, 255);
    }
    else if (true)
    {
      // --- Simple PI regulator ---
      const float Kp = 1.2f;   // proportional gain
      const float Ki = 0.8f;   // integral gain — do strojenia
      const float Imax = 200.0f;  // anti-windup limit (to avoid integrator overflow)

      // static — zachowują wartość między wywołaniami
      static float I_TL = 0, I_TR = 0, I_BL = 0, I_BR = 0;

      // --- Compute errors ---
      float err_TL = requested_speed_setpoint_TL - current_speed_TL;
      float err_TR = requested_speed_setpoint_TR - current_speed_TR;
      float err_BL = requested_speed_setpoint_BL - current_speed_BL;
      float err_BR = requested_speed_setpoint_BR - current_speed_BR;

      // --- Integrate errors ---
      I_TL += err_TL * (regulationInterval / 1000.0f);
      I_TR += err_TR * (regulationInterval / 1000.0f);
      I_BL += err_BL * (regulationInterval / 1000.0f);
      I_BR += err_BR * (regulationInterval / 1000.0f);

      // --- Anti-windup clamp ---
      I_TL = constrain(I_TL, -Imax, Imax);
      I_TR = constrain(I_TR, -Imax, Imax);
      I_BL = constrain(I_BL, -Imax, Imax);
      I_BR = constrain(I_BR, -Imax, Imax);

      // --- Compute total control (P + I) ---
      float u_TL = Kp * err_TL + Ki * I_TL;
      float u_TR = Kp * err_TR + Ki * I_TR;
      float u_BL = Kp * err_BL + Ki * I_BL;
      float u_BR = Kp * err_BR + Ki * I_BR;

      // --- Apply direction flips as in your snippet ---
      calculated_dutycycle_TL = constrain(-(int)u_TL, -255, 255);
      calculated_dutycycle_TR = constrain((int)u_TR, -255, 255);
      calculated_dutycycle_BL = constrain(-(int)u_BL, -255, 255);
      calculated_dutycycle_BR = constrain((int)u_BR, -255, 255);
    }

    else
    {
      // --- Feedforward mode (no feedback) ---
      // Directly map requested speeds to duty cycle
      // Assuming requested_speed_setpoint_* ∈ [-255, 255]
      calculated_dutycycle_TL = constrain(requested_speed_setpoint_TL, -255, 255);
      calculated_dutycycle_TR = constrain(requested_speed_setpoint_TR, -255, 255);
      calculated_dutycycle_BL = constrain(requested_speed_setpoint_BL, -255, 255);
      calculated_dutycycle_BR = constrain(requested_speed_setpoint_BR, -255, 255);
    }
  }

  // --- if no command → stop ---
  if (millis() - lastCommandTime < commandTimeout) {
    actuateMotors(calculated_dutycycle_TL,
                  calculated_dutycycle_TR,
                  calculated_dutycycle_BL,
                  calculated_dutycycle_BR);
  } else {
    actuateMotors(0, 0, 0, 0);
  }

  // Periodic telemetry
  if (millis() - lastTelemetryTime >= telemetryInterval) {
    lastTelemetryTime = millis();

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
    Serial.print(BR);
    Serial.print(", RPM,");
    Serial.print(current_speed_TL); Serial.print(",");
    Serial.print(current_speed_TR); Serial.print(",");
    Serial.print(current_speed_BL); Serial.print(",");
    Serial.println(current_speed_BR);
  }
  delay(1);
}
