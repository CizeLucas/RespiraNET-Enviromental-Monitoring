// Arquivo: sensor_data.h (Exemplo de nome)

#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

// Enum para identificar o tipo de dado
typedef enum {
    SENSOR_TYPE_TEMPERATURE,
    SENSOR_TYPE_HUMIDITY,
    SENSOR_TYPE_LDR,
    SENSOR_TYPE_MQ2
} sensor_type_t;

// Estrutura de dados para a fila e comunicação ESP-NOW
// ATENÇÃO: O ESP-NOW tem um limite de 250 bytes por mensagem. Esta struct é perfeitamente segura.
typedef struct {
    sensor_type_t type; // O tipo de sensor
    float value;        // O valor lido
} sensor_data_t;

#endif