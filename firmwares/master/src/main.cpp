#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <PubSubClient.h>
#include "time.h"
#include "secrets.h"

// --- ESTRUTURAS DE DADOS ---
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
    unsigned long epoch;
} time_sync_response_t;

// Struct interna da Fila do Master (Junta MAC + Dados)
typedef struct {
    uint8_t mac_addr[6];
    sensor_payload_t data;
} master_queue_message_t;

// --- GLOBAIS ---
String masterMacAddress = "";
QueueHandle_t g_incoming_data_queue;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// --- HELPERS ---
String macToString(const uint8_t* mac) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(macStr);
}

String sensorTypeToString(sensor_type_t type) {
    switch(type) {
        case SENSOR_TYPE_TEMPERATURE: return "temperatura";
        case SENSOR_TYPE_HUMIDITY:    return "umidade";
        case SENSOR_TYPE_LDR:         return "ldr";
        case SENSOR_TYPE_GAS_MQ135:   return "mq135";
        case SENSOR_TYPE_GAS_MQ2:     return "mq2";
        default:                      return "desconhecido";
    }
}

// --- CALLBACK ESP-NOW (ISR - Executa rápido!) ---
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
    
    // 1. Verifica integridade básica
    if (len != sizeof(sensor_payload_t)) return;

    sensor_payload_t *recvData = (sensor_payload_t *)incomingData;

    // 2. CASO ESPECIAL: Pedido de Sincronia de Tempo
    if (recvData->type == SENSOR_TYPE_TIME_SYNC_REQUEST) {
        // Adiciona o Slave como peer dinamicamente (se não existir) para poder responder
        if (!esp_now_is_peer_exist(mac_addr)) {
            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, mac_addr, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;
            esp_now_add_peer(&peerInfo);

            // Verifica erro ao adicionar
            esp_err_t addStatus = esp_now_add_peer(&peerInfo);
            if (addStatus == ESP_ERR_ESPNOW_FULL) {
                Serial.println("ERRO: Lista de Peers cheia! O Master não pode responder.");
                // Opcional: Limpar um peer antigo aqui seria uma lógica avançada
            }
        }

        // Pega hora NTP
        time_t now; 
        time(&now);
        
        // Responde
        time_sync_response_t resp;
        resp.epoch = (unsigned long)now;
        esp_now_send(mac_addr, (uint8_t *)&resp, sizeof(resp));
        return; // Fim do processamento para sync
    }

    // 3. CASO NORMAL: Dado de Sensor -> Envia para Fila
    master_queue_message_t msg;
    memcpy(msg.mac_addr, mac_addr, 6);
    memcpy(&msg.data, incomingData, sizeof(sensor_payload_t));

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(g_incoming_data_queue, &msg, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

// --- TAREFA MQTT ---
void vMqttSenderTask(void *pvParameters) {
    master_queue_message_t msg;
    char topic[128];
    char payload_json[128];

    for (;;) {
        // Mantém conexão MQTT
        if (!mqttClient.connected()) {
            while (!mqttClient.connected()) {
                if (mqttClient.connect("ESP32Master", MQTT_USER, MQTT_PASS)) {
                    Serial.println("MQTT Conectado");
                } else {
                    Serial.println("Conexao com Broker MQTT Falhou, tentando novamente em 5s");
                    vTaskDelay(pdMS_TO_TICKS(5000));
                }
            }
        }
        mqttClient.loop();

        // Processa Fila
        while (xQueueReceive(g_incoming_data_queue, &msg, pdMS_TO_TICKS(10)) == pdTRUE) {
            String mac = macToString(msg.mac_addr);
            String type = sensorTypeToString(msg.data.type);

            // Tópico: env/master/MASTER_MAC/SENSOR_TYPE
            snprintf(topic, sizeof(topic), "env/master/%s/%s", masterMacAddress.c_str(), type.c_str());
            
            // Payload JSON: {"timestamp": "...", "value": 123.45}
            snprintf(payload_json, sizeof(payload_json), 
                     "{\"nodeMac\": \"%s\", \"timestamp\": \"%s\", \"value\": %.2f}", 
                     mac.c_str(), msg.data.timestamp, msg.data.value);

            mqttClient.publish(topic, payload_json);
            Serial.printf("Pub: %s -> %s\n", topic, payload_json);
        }
        // Adiciona um delay de 100ms no final do loop. 
        // Isso garante que a tarefa IDLE rode e limpe o Watchdog.
        vTaskDelay(pdMS_TO_TICKS(100)); //TODO: Confirmar se é necessário
    }
}

void setup() {
    Serial.begin(115200);
    
    // Wi-Fi
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    WiFi.setSleep(false);
    masterMacAddress = WiFi.macAddress();
    Serial.printf("\nWiFi OK. MAC: %s\n", masterMacAddress.c_str());

    // NTP
    configTime(GMT_OFFSET_SEC, DST_OFFSET_SEC, NTP_SERVER);
    
    // Aguarda sincronização NTP e imprime o tempo
    Serial.print("Aguardando sincronização NTP...");
    struct tm timeinfo;
    int retry = 0;
    while (!getLocalTime(&timeinfo) && retry < 10) {
        Serial.print(".");
        delay(1000);
        retry++;
    }
    if (retry < 10) {
        Serial.println(" OK!");
        Serial.println("Tempo obtido via NTP:");
        Serial.printf("Data/Hora: %02d/%02d/%04d %02d:%02d:%02d\n",
                     timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900,
                     timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        
        time_t now;
        time(&now);
        Serial.printf("Timestamp Unix: %lu\n", (unsigned long)now);
    } else {
        Serial.println(" FALHOU!");
    }

    // ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Erro ESP-NOW");
        ESP.restart();
    }
    esp_now_register_recv_cb(OnDataRecv);

    // FreeRTOS
    g_incoming_data_queue = xQueueCreate(20, sizeof(master_queue_message_t));
    xTaskCreatePinnedToCore(vMqttSenderTask, "MQTT", 8192, NULL, 1, NULL, 1);
    
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
}

void loop() {
  vTaskDelete(NULL);
}