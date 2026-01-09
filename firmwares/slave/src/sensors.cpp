#include "sensors.h"

// --- Configurações Sensores ---
const int DHT11_PIN = 32;
const int LDR_PIN = 34;
const int MQ135_PIN = 35;
const int MQ2_PIN = 36;
const sensor_config_t dht11_temperature_sensor = {SENSOR_TYPE_TEMPERATURE, DHT11_PIN, 16.0, 40.0};
const sensor_config_t dht11_humidity_sensor = {SENSOR_TYPE_HUMIDITY, DHT11_PIN, 20.0, 85.0};
const sensor_config_t ldr_sensor = {SENSOR_TYPE_LDR, LDR_PIN, 0.0, 4095.0};
const sensor_config_t mq135_sensor = {SENSOR_TYPE_GAS_MQ135, MQ135_PIN, 512.0, 3583.0};
const sensor_config_t mq2_sensor = {SENSOR_TYPE_GAS_MQ2, MQ2_PIN, 1023.0, 3069.0};

// --- Objetos Globais ---
DHT dht(DHT11_PIN, DHT11);

// --- TAREFAS: SENSORES REAIS ---
void vSensorDHT11(void *pvParam) {
    dht.begin();
    
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(SENSOR_INTERVAL_MS));
        
        sensor_payload_t data;

        // Leitura de Temperatura:
        float t = dht.readTemperature();
        if (!isnan(t)) {
            data.type = SENSOR_TYPE_TEMPERATURE;
            data.value = t;
            get_iso_timestamp(data.timestamp);
            check_value_bounds(&data, &dht11_temperature_sensor);
            
            // Só envia para fila se o tempo estiver sincronizado
            EventBits_t bits = xEventGroupGetBits(g_evt_group);
            if (bits & BIT_TIME_SYNCED) {
                xQueueSend(g_sensor_queue, &data, pdMS_TO_TICKS(100));
            }
        } else {
            Serial.println("[DHT11] Falha na leitura de Temperatura!");
        }

        // Leitura de Umidade:
        float h = dht.readHumidity();
        if (!isnan(h)) {
            data.type = SENSOR_TYPE_HUMIDITY;
            data.value = h;
            get_iso_timestamp(data.timestamp);
            check_value_bounds(&data, &dht11_humidity_sensor);
            
            // Só envia para fila se o tempo estiver sincronizado
            EventBits_t bits = xEventGroupGetBits(g_evt_group);
            if (bits & BIT_TIME_SYNCED) {
                xQueueSend(g_sensor_queue, &data, pdMS_TO_TICKS(100));
            }
        } else {
            Serial.println("[DHT11] Falha na leitura de Umidade!");
        }
    }
}

void vSensorMQ135(void *pvParam) {
    pinMode(MQ135_PIN, INPUT);
    
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(SENSOR_INTERVAL_MS));
        
        sensor_payload_t data;
        int raw_value = analogRead(MQ135_PIN);

        data.type = SENSOR_TYPE_GAS_MQ135;
        data.value = (float)raw_value; 
        get_iso_timestamp(data.timestamp);
        check_value_bounds(&data, &mq135_sensor);
        
        // Só envia para fila se o tempo estiver sincronizado
        EventBits_t bits = xEventGroupGetBits(g_evt_group);
        if (bits & BIT_TIME_SYNCED) {
            xQueueSend(g_sensor_queue, &data, pdMS_TO_TICKS(100));
        }
    }
}

void vSensorLDR(void *pvParam) {
    pinMode(LDR_PIN, INPUT);
    
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(SENSOR_INTERVAL_MS));
        
        sensor_payload_t data;
        int raw_value = analogRead(LDR_PIN);

        data.type = SENSOR_TYPE_LDR;
        data.value = (float)raw_value;
        get_iso_timestamp(data.timestamp);
        check_value_bounds(&data, &ldr_sensor);
        
        // Só envia para fila se o tempo estiver sincronizado
        EventBits_t bits = xEventGroupGetBits(g_evt_group);
        if (bits & BIT_TIME_SYNCED) {
            xQueueSend(g_sensor_queue, &data, pdMS_TO_TICKS(100));
        }
    }
}

// --- IMPLEMENTAÇÃO DE TAREFAS MOCKADAS DE SENSORES (TESTE) ---
void vMockedSensorDHT11(void *pvParam) {
    
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(SENSOR_INTERVAL_MS));
        
        sensor_payload_t data;

        // Leitura de Temperatura:
        data.type = SENSOR_TYPE_TEMPERATURE;
        data.value = random(2000, 3500) / 100.0; // Simula 20.00 a 35.00
        get_iso_timestamp(data.timestamp);
        check_value_bounds(&data, &dht11_temperature_sensor);
        
        // Só envia para fila se o tempo estiver sincronizado
        EventBits_t bits = xEventGroupGetBits(g_evt_group);
        if (bits & BIT_TIME_SYNCED) {
            xQueueSend(g_sensor_queue, &data, pdMS_TO_TICKS(100));
        }

        // Leitura de Umidade:
        data.type = SENSOR_TYPE_HUMIDITY;
        data.value = random(0, 100); // Simula 0.00 a 100.00
        get_iso_timestamp(data.timestamp);
        check_value_bounds(&data, &dht11_humidity_sensor);
        
        // Só envia para fila se o tempo estiver sincronizado
        bits = xEventGroupGetBits(g_evt_group);
        if (bits & BIT_TIME_SYNCED) {
            xQueueSend(g_sensor_queue, &data, pdMS_TO_TICKS(100));
        }
    }
}

void vMockedSensorMQ135(void *pvParam) {    
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(SENSOR_INTERVAL_MS));
        
        sensor_payload_t data;

        data.type = SENSOR_TYPE_GAS_MQ135;
        data.value = random(0, 4095); // Simula 0 a 4095
        get_iso_timestamp(data.timestamp);
        check_value_bounds(&data, &mq135_sensor);
        
        // Só envia para fila se o tempo estiver sincronizado
        EventBits_t bits = xEventGroupGetBits(g_evt_group);
        if (bits & BIT_TIME_SYNCED) {
            xQueueSend(g_sensor_queue, &data, pdMS_TO_TICKS(100));
        }
    }
}

void vMockedSensorLDR(void *pvParam) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(SENSOR_INTERVAL_MS));
        
        sensor_payload_t data;

        data.type = SENSOR_TYPE_GAS_MQ135;
        data.value = random(0, 4095); // Simula 0 a 4095
        get_iso_timestamp(data.timestamp);
        check_value_bounds(&data, &mq135_sensor);
        
        // Só envia para fila se o tempo estiver sincronizado
        EventBits_t bits = xEventGroupGetBits(g_evt_group);
        if (bits & BIT_TIME_SYNCED) {
            xQueueSend(g_sensor_queue, &data, pdMS_TO_TICKS(100));
        }
    }
}
