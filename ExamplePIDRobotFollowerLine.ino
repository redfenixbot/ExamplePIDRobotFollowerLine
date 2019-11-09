#include <QTRSensors.h>
#include <SparkFun_TB6612.h>

// Variaveis Biblioteca QTRSensors
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

// Variaveis da Biblioteca SparkFun_TB6612
#define PWMA 5
#define AIN2 2
#define AIN1 4
#define STBY 9
#define BIN1 7
#define BIN2 8
#define PWMB 6

const int offsetA =  1;
const int offsetB = -1;

Motor motorLeft  = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motorRight = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// Variaveis do controle PID
int error     = 0,
    lastError = 0;

int setpoint = 2500;

float kp = 0.0,
      kd = 0.0;

int P = 0,
    D = 0;

int valuePID = 0;

// Variaveis auxiliares dos motores
int speedBase = 150;

int speedMotorLeft  = 0,
    speedMotorRight = 0;

// LED
#define LED 13

void setup(){
  // Definindo os pinos como output ou input
  pinMode(LED, OUTPUT);

  // Configuração do Array de Sensores
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(2);

  // Rotina de Calibração dos Sensores
  delay(1000);
  
  digitalWrite(LED, HIGH); // Liga o LED

  for (uint16_t i = 0; i < 200; i++){qtr.calibrate();}

  digitalWrite(LED, LOW); // Desliga o LED
}

void loop() {
  //Leitura do Sensor
  uint16_t position = qtr.readLineWhite(sensorValues);

  //Calculo do Erro
  error = position - setpoint;

  P = error; //Proporcional = erro
  D = error - lastError; // Derivativo = erro - erroAnterior

  valuePID = (P*kp) + (D*kd);

  lastError = error;
  
  speedMotorLeft  = speedBase - valuePID;
  speedMotorRight = speedBase + valuePID;

  if(speedMotorLeft  <   0) speedMotorLeft  =   0;
  if(speedMotorLeft  > 255) speedMotorLeft  = 255;
  if(speedMotorRight <   0) speedMotorRight =   0;
  if(speedMotorRight > 255) speedMotorRight = 255;

  motorLeft.drive(speedMotorLeft);
  motorRight.drive(speedMotorRight);
}
