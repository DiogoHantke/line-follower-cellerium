#define PINBOTON 2
#define PINBUZZER 10
#define PINLED 13
#define PIN_Sensor_ON 11
#define AIN1 8
#define AIN2 9
#define PWMA 5
#define BIN1 4
#define BIN2 7
#define PWMB 6
#define vel_base_main 190
#define STOP_PIN A0
#define setpoint 0
#define last_error 0
#define pot_limite 250
#define pot_limit_base 250
#define debounce_time 120
#define PIN_SENSOR_LATERAL A7
#define LIMIAR_BRANCO 30
#define LIMIAR_SIDE LIMIAR_BRANCO
#define num_sensors 6

// Variables that change during execution
int vel_base;
int last_side_state;
int tempo_lento_ms;
int last_tempo_update;
int sum_sensors_2;