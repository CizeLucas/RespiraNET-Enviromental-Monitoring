#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <DHT.h>
#include "freertos_globals.h"

// --- ESTRUTURAS ---
typedef enum {
    SENSOR_TYPE_TEMPERATURE,
    SENSOR_TYPE_HUMIDITY,
    SENSOR_TYPE_LDR,
    SENSOR_TYPE_GAS_MQ135,
    SENSOR_TYPE_GAS_MQ2,
    SENSOR_TYPE_TIME_SYNC_REQUEST
} sensor_type_t;

typedef struct {
    char timestamp[21];
    sensor_type_t type;
    float value;
} sensor_payload_t;

typedef struct {
  sensor_type_t type;
  int sensor_gpio_pin;
  float min_value; // -999999 se não aplicável
  float max_value; // 999999 se não aplicável
} sensor_config_t;

// --- Configurações Sensores ---
extern const int DHT11_PIN;
extern const int LDR_PIN;
extern const int MQ135_PIN;
extern const int MQ2_PIN;
extern const sensor_config_t dht11_temperature_sensor;
extern const sensor_config_t dht11_humidity_sensor;
extern const sensor_config_t ldr_sensor;
extern const sensor_config_t mq135_sensor;
extern const sensor_config_t mq2_sensor;

// --- Objetos Globais ---
extern DHT dht;

// --- Variáveis Globais Externas ---
extern const int SENSOR_INTERVAL_MS;

// --- Protótipos de Funções ---
void get_iso_timestamp(char* buffer);
void check_value_bounds(sensor_payload_t* data, const sensor_config_t* config);

// --- Tarefas de Sensores Reais ---
void vSensorDHT11(void *pvParam);
void vSensorMQ135(void *pvParam);
void vSensorLDR(void *pvParam);

// --- Tarefas de Sensores Mockados ---
void vMockedSensorDHT11(void *pvParam);
void vMockedSensorMQ135(void *pvParam);
void vMockedSensorLDR(void *pvParam);

#endif // SENSORS_H
