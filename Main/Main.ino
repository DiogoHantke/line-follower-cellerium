#include <Arduino.h>

// Protótipos das funções
int PIDLambo(int pos, float Kp, float Kd, int pot_limite);
void beep();
void WaitBoton();
void Peripherals_init();
void TB6612FNG_init();
void MotorIz(int value);
void MotorDe(int value);
void Motores(int left, int right);
void Sensors_init();
void calibracion();
void readSensors_BlackLine();
int GetPos_black();
void setup();
void stop();
void Curve_line(int leitura, bool *state_vel, int *vel_base, int *pot_limite);
void loop();

const int PINBOTON      = 2;
const int PINBUZZER     = 10;
const int PINLED        = 13;
const int PIN_Sensor_ON = 11;

const int AIN1 = 8;
const int AIN2 = 9;
const int PWMA = 5;

const int BIN1 = 4;
const int BIN2 = 7;
const int PWMB = 6;

const int vel_base_main = 190;
int vel_base            = 190;
const int STOP_PIN    = A0;              

int setpoint   = 0;
int last_error = 0;
int pot_limite = 250;
const int pot_limit_base = 250;
const int debounce_time = 120;

const int PIN_SENSOR_LATERAL = A7;
const int LIMIAR_BRANCO = 30;
const int LIMIAR_SIDE = LIMIAR_BRANCO;
bool last_side_state = false;
long tempo_lento_ms = 0;
unsigned long last_tempo_update = 0;

const int num_sensors = 6;

int sum_sensors_2 = 0;

void setup() {
  Serial.begin(115200);
  Peripherals_init();
  Sensors_init();
  TB6612FNG_init();

  digitalWrite(PINLED, LOW);
  delay(500);

  Serial.println("Calibragem dos sensores aperte o botão...");
  WaitBoton();
  calibracion();

  digitalWrite(PINLED, HIGH);
  Serial.println("Calibrado! Pressione botão para iniciar a linha.");
  WaitBoton();
}

void loop() {

  unsigned long int time = millis();
  static long int past_time = 0;
  int posicao = GetPos_black();
  int correcao = PIDLambo(posicao, 1.2, 4.3, pot_limite), limiar_time_loop = 9000; 
  static int state_pin = 0;

  int velE = vel_base - correcao;
  int velD = vel_base + correcao;
  
  int leitura = analogRead(PIN_SENSOR_LATERAL);

  static bool state_vel = false;

  Curve_line(leitura, &state_vel, &vel_base, &pot_limite);

  velE = constrain(velE, -255, 255);
  velD = constrain(velD, -255, 255);

  //Motores(velE, velD);
  if(time - past_time >= limiar_time_loop){
    beep();
    stop();
  }
}

int PIDLambo(int pos, float Kp, float Kd, int pot_limite) {
  int error = pos - setpoint;
  int derivative = error - last_error;
  last_error = error;

  int pot_giro = (error * Kp + derivative * Kd);

  if (pot_giro > pot_limite) pot_giro = pot_limite;
  else if (pot_giro < -pot_limite) pot_giro = -pot_limite;

  return pot_giro;
}

void beep() {
  tone(PINBUZZER, 1000, 100);
}

void WaitBoton() {
  while (!digitalRead(PINBOTON));
  beep();
}

void Peripherals_init() {
  pinMode(PINBOTON, INPUT);
  pinMode(PINBUZZER, OUTPUT);
  pinMode(PINLED, OUTPUT);
}

void TB6612FNG_init() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
}

void MotorIz(int value) {
  if (value >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    value *= -1;
  }
  analogWrite(PWMA, constrain(value, 0, 255));
}

void MotorDe(int value) {
  if (value >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    value *= -1;
  }
  analogWrite(PWMB, constrain(value, 0, 255));
}

void Motores(int left, int right) {
  MotorIz(left);
  MotorDe(right);
}

int v_s_min[num_sensors] = {1023,1023,1023,1023,1023,1023};
int v_s_max[num_sensors] = {0,0,0,0,0,0};
volatile int s_p[num_sensors];
bool online;
int pos, l_pos;

void Sensors_init() {
  pinMode(PIN_Sensor_ON, OUTPUT);
}

void calibracion() {
  digitalWrite(PIN_Sensor_ON, HIGH);
  int v_s[num_sensors];
  for (int j = 0; j < 100; j++) {

    delay(30);

    v_s[0] = analogRead(A6);
    v_s[1] = analogRead(A5);
    v_s[2] = analogRead(A4);
    v_s[3] = analogRead(A3);
    v_s[4] = analogRead(A2);
    v_s[5] = analogRead(A1);

    for (int i = 0; i < num_sensors; i++) {
      if (v_s[i] < v_s_min[i]) v_s_min[i] = v_s[i];
      if (v_s[i] > v_s_max[i]) v_s_max[i] = v_s[i];
    }
  }
  beep();
  beep();
  digitalWrite(PIN_Sensor_ON, LOW);
}

void readSensors_BlackLine() {
  digitalWrite(PIN_Sensor_ON, HIGH);

  int s[num_sensors];

  s[0] = analogRead(A6);
  s[1] = analogRead(A5);
  s[2] = analogRead(A4);
  s[3] = analogRead(A3);
  s[4] = analogRead(A2);
  s[5] = analogRead(A1);

  for (int i = 0; i < num_sensors; i++) {
    s[i] = constrain(s[i], v_s_min[i], v_s_max[i]);
    s_p[i] = map(s[i], v_s_min[i], v_s_max[i], 100, 0);
  }

  int sum = 0;
  
  for (int i = 0; i < num_sensors; i++) sum += s_p[i];
  online = (sum > 100);
  digitalWrite(PIN_Sensor_ON, LOW);
}

int GetPos_black() {
  readSensors_BlackLine();
  int prom = -2.5*s_p[0] -1.5*s_p[1] -0.5*s_p[2] +0.5*s_p[3] +1.5*s_p[4] +2.5*s_p[5];
  int sum = s_p[0]+s_p[1]+s_p[2]+s_p[3]+s_p[4]+s_p[5];

  if (online)
    pos = int(100.0 * prom / sum);
  else
    pos = (l_pos < 0) ? -255 : 255;
    l_pos = pos;
  return pos;
}

void stop(){
  
  // le a soma dos sensores ativos, >= 5, significa cruzamento, desabilita a leitura do sensor lateral por um tempo determinado
  bool state = false;
  static int sensor[num_sensors], limiar_time_stop = 9000;
  int sum_sensors_stop = 0, sensor_state[num_sensors];
  unsigned long int time = millis();
  static unsigned long int time_again = 0;

  sensor[0] = analogRead(A6);
  sensor[1] = analogRead(A5);
  sensor[2] = analogRead(A4);
  sensor[3] = analogRead(A3);
  sensor[4] = analogRead(A2);
  sensor[5] = analogRead(A1);

  for (int i = 0; i < num_sensors; i++) {
    sensor[i] = constrain(sensor[i], v_s_min[i], v_s_max[i]);
    sensor_state[i] = map(sensor[i], v_s_min[i], v_s_max[i], 100, 0);

    if(sensor_state[i] >= 10){
      sum_sensors_stop++;
    }
    Serial.print(sensor_state[i]);
    Serial.print(" ");
  }

  Serial.print("\n");

  if(sum_sensors_stop >= 5) state = true;
  
  if((time - time_again >= limiar_time_stop) && state){
    time_again = millis();
    if(analogRead(STOP_PIN) <= LIMIAR_BRANCO){
      while(1){
        if(analogRead(PINBOTON)) break;
        else continue;
      }
    }
  }else if((time - time_again >= limiar_time_stop) && !state && sum_sensors_stop < 5){
    while(1){
        if(analogRead(PINBOTON)) break;
        else continue;
    }
  }

  //delay(200);//tirar mais tarde
  // Serial.print("\n");
}

void Curve_line(int leitura, bool *state_vel, int *vel_base, int *pot_limite)
{
  bool side_state = (leitura >= LIMIAR_SIDE && analogRead(STOP_PIN) <= LIMIAR_SIDE);

  if (last_side_state == false && side_state == true && analogRead(STOP_PIN) <= LIMIAR_SIDE)
  {
    *state_vel = !(*state_vel);
  }

  last_side_state = side_state;

  if (*state_vel)
  {
    *vel_base = 110;
    *pot_limite = 110;
  }
  else
  {
    *vel_base = vel_base_main;
    *pot_limite = pot_limit_base;
  }
}