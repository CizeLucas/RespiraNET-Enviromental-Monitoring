#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <SD.h>
#include <SPI.h>
#include "time.h"
#include "secrets.h"
#include "freertos_globals.h"
#include "sensors.h"

// ----------------- CONFIGURAÇÕES -----------------
const int SENSOR_INTERVAL_MS = 10000; // Ler a cada 30s
const int TIME_SYNC_INTERVAL_MS   = 3600000; // Sincronizar relógio a cada 1h
const int SEND_JITTER_MAX_MS = 500;   // Jitter aleatório
// -------------------------------------------------

// --- ESTRUTURAS ---
typedef struct {
    unsigned long epoch;
} time_sync_response_t;

// --- GLOBAIS ---
bool alertEnabled = false;
const int BEEP_DELAY_MS = 1000;
String nodeMacAddress = "";

// --- GLOBAIS FREERTOS ---
QueueHandle_t g_sensor_queue;
QueueHandle_t g_sdcard_queue;
EventGroupHandle_t g_evt_group;
#define BIT_ACK_SUCCESS  BIT0  // O envio ESP-NOW foi recebido com sucesso pelo destinatário
#define BIT_ACK_FAIL     BIT1  // O envio falhou (destinatário não recebeu)
#define BIT_SYNC_DONE    BIT2  // A resposta de sincronização de tempo chegou

// --- GLOBAIS DE TEMPO ---
unsigned long g_epoch_base = 0;
unsigned long g_millis_base = 0;
bool g_is_time_synced = false;

// --- Configurações de GPIO ---
const int PIN_BUILTIN_LED = 2;
const int PIN_BUZZER = 27;
const int PIN_SD_CS = 5; // Chip Select do SD Card

// Modo de Teste (Simula sensores com valores aleatórios)
const bool system_test_mode = false;

// Protótipos de funções
void ligarAlerta();
void scanForMaster();

// --- CALLBACKS ESP-NOW ---
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (status == ESP_NOW_SEND_SUCCESS) 
        xEventGroupSetBitsFromISR(g_evt_group, BIT_ACK_SUCCESS, &xHigherPriorityTaskWoken);
    else 
        xEventGroupSetBitsFromISR(g_evt_group, BIT_ACK_FAIL, &xHigherPriorityTaskWoken);
        
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    // Se receber resposta de tempo do Master
    if (len == sizeof(time_sync_response_t)) {
        time_sync_response_t *resp = (time_sync_response_t *)incomingData;
        g_epoch_base = resp->epoch;
        g_millis_base = millis();
        
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xEventGroupSetBitsFromISR(g_evt_group, BIT_SYNC_DONE, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
    }
}

// --- HELPER DE TIMESTAMP ---
void get_iso_timestamp(char* buffer) {
    // Calcula hora atual baseada no delta do millis
    unsigned long current_epoch = g_epoch_base + ((millis() - g_millis_base) / 1000);
    struct tm * ti;
    time_t rawtime = (time_t)current_epoch;
    ti = localtime(&rawtime);
    
    sprintf(buffer, "%04d-%02d-%02dT%02d:%02d:%02dZ", 
            ti->tm_year + 1900, ti->tm_mon + 1, ti->tm_mday, 
            ti->tm_hour, ti->tm_min, ti->tm_sec);
}

// --- TAREFA: SINCRONIZAÇÃO DE TEMPO ---
void vTimeSyncTask(void *pvParam) {
    int tries = 0;
    
    for (;;) {
        sensor_payload_t req;
        req.type = SENSOR_TYPE_TIME_SYNC_REQUEST;
        req.value = 0;
        strcpy(req.timestamp, ""); // Vazio

        Serial.println("[Sync] Solicitando hora ao Master...");
        esp_now_send(masterMacAddress, (uint8_t *)&req, sizeof(req));

        // Espera resposta por 2 segundos
        EventBits_t bits = xEventGroupWaitBits(g_evt_group, BIT_SYNC_DONE, pdTRUE, pdFALSE, pdMS_TO_TICKS(2000));
        
        if (bits & BIT_SYNC_DONE) {
            char timestamp[21];
            g_is_time_synced = true;
            xEventGroupSetBits(g_evt_group, BIT_TIME_SYNCED); // Libera sensores
            get_iso_timestamp(timestamp);
            Serial.printf("\n[Sync] Sucesso! Hora atual sincronizada (%s).\n", timestamp);
            tries = 0;
            vTaskDelay(pdMS_TO_TICKS(TIME_SYNC_INTERVAL_MS)); // Dorme 1h
        } else {
            tries++;
            Serial.printf("[Sync] Falha/Timeout (Tentativa %d/3). Tentando novamente em 10s...\n", tries);
            
            if (tries >= 3) {
                Serial.println("[Sync] 3 falhas consecutivas. Iniciando re-escaneamento do Master...");
                scanForMaster(); 
                tries = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
    }
}

void check_value_bounds(sensor_payload_t* data, const sensor_config_t* config) {
  bool isBelowMin = (config->min_value >= -999999) && (config->min_value > data->value);
  bool isAboveMax = (config->max_value <= 999999) && (config->max_value < data->value);
    if (isBelowMin || isAboveMax) {
        Serial.printf("[Alert] Valor fora dos limites para sensor %d: %.2f (Min: %.2f, Max: %.2f)\n", 
                      data->type, data->value, config->min_value, config->max_value);
        // Aciona modo alerta
        ligarAlerta();
    }
}

// --- TAREFA: ENVIO (Consumidor da Fila) ---
void vSenderTask(void *pvParam) {
    sensor_payload_t data;
    for (;;) {
        // Consome a fila e caso tenha a hora sincronizada, envia os dados
        if (xQueueReceive(g_sensor_queue, &data, portMAX_DELAY) == pdTRUE && g_is_time_synced) {
            bool sent = false;
            int retries = 0;
            
            Serial.printf("[Sender] Enviando dado do sensor %d: %.2f\n", data.type, data.value);

            while (!sent && retries < 3) {
                // JITTER: Espera aleatória 0-500ms
                vTaskDelay(pdMS_TO_TICKS(random(0, SEND_JITTER_MAX_MS)));

                xEventGroupClearBits(g_evt_group, BIT_ACK_SUCCESS | BIT_ACK_FAIL);
                esp_now_send(masterMacAddress, (uint8_t *)&data, sizeof(data));

                EventBits_t bits = xEventGroupWaitBits(g_evt_group, BIT_ACK_SUCCESS | BIT_ACK_FAIL, pdTRUE, pdFALSE, pdMS_TO_TICKS(1000));

                if (bits & BIT_ACK_SUCCESS) {
                    Serial.printf("[Sender] Enviado OK! Val: %.2f\n", data.value);
                    sent = true;
                } else {
                    Serial.printf("[Sender] Falha/Sem ACK. Tentando novamente... (Sensor: %d Val: %.2f)\n", data.type, data.value);
                    retries++;
                }
            }

            if (!sent) {
                Serial.printf("[Sender] Falha ao enviar dado %.2f do sensor %d após 3 tentativas.\n", data.value, data.type);
                
                // Grava no SD Card
                if (xQueueSend(g_sdcard_queue, &data, pdMS_TO_TICKS(100)) == pdTRUE) {
                    Serial.println("[Sender] Dado enfileirado para gravação no SD Card");
                }
                
                Serial.println("[Sender] Iniciando re-escaneamento do Master...");
                scanForMaster();
            }
        }
    }
}

// --- TAREFA: GRAVAÇÃO NO SD CARD ---
void vTaskSDCard(void *pvParam) {
    sensor_payload_t data;
    
    for (;;) {
        // Aguarda dados na fila de falhas para gravar no SD
        if (xQueueReceive(g_sdcard_queue, &data, portMAX_DELAY) == pdTRUE) {
            File file = SD.open("/slave_data.txt", FILE_APPEND);
            
            if (!file) {
                Serial.println("[SD] Erro ao abrir arquivo para escrita");
                continue;
            }
            
            // Grava: timestamp, tipo, valor
            file.printf("%s,%d,%.2f\n", data.timestamp, data.type, data.value);
            file.close();
            
            Serial.printf("[SD] Gravado: %s | Sensor %d | Val: %.2f\n", 
                         data.timestamp, data.type, data.value);
        }
    }
}

void vTaskBuzzer(void *parameter) {
  pinMode(PIN_BUZZER, OUTPUT);

  for(;;) { // Loop infinito da Task
    if (alertEnabled) {
      Serial.println("[Alert] ALERTA LIGADO!");
      digitalWrite(PIN_BUZZER, HIGH);
      vTaskDelay(pdMS_TO_TICKS(BEEP_DELAY_MS));
      digitalWrite(PIN_BUZZER, LOW);

      alertEnabled = false; // Desliga o modo alerta após um beep
      Serial.println("[Alert] ALERTA DESLIGADO!");
    } else {
      digitalWrite(PIN_BUZZER, LOW);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

void ligarAlerta() {
  alertEnabled = true;
}

// --- FUNÇÃO DE VARREDURA DE CANAL ---
void scanForMaster() {
    int32_t channel = 1;
    bool found = false;
    
    Serial.println("[Scan] Procurando o Master nos canais Wi-Fi...");

    while(!found) {
        // Tenta nos canais 1 a 13
        for (channel = 1; channel <= 13; channel++) {
            // Muda o canal do rádio
            esp_wifi_set_promiscuous(true);
            esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
            esp_wifi_set_promiscuous(false);
            
            Serial.printf("... Testando Canal %d\n", channel);

            // Envia um pacote de teste (Sync Request é leve)
            sensor_payload_t ping;
            ping.type = SENSOR_TYPE_TIME_SYNC_REQUEST;
            strcpy(ping.timestamp, "");
            ping.value = 0;

            // Limpa flags
            xEventGroupClearBits(g_evt_group, BIT_ACK_SUCCESS | BIT_ACK_FAIL);
            
            // Envia
            esp_now_send(masterMacAddress, (uint8_t *)&ping, sizeof(ping));

            // Espera ACK por 100ms (rápido)
            EventBits_t bits = xEventGroupWaitBits(g_evt_group, BIT_ACK_SUCCESS, pdTRUE, pdFALSE, pdMS_TO_TICKS(100));

            if (bits & BIT_ACK_SUCCESS) {
                Serial.printf("[Scan] Mestre encontrado no Canal %d!\n", channel);
                found = true;
                break; // Sai do loop, estamos no canal certo
            }
        }

        if (!found) {
            Serial.println("[Scan] Mestre NÃO encontrado. Tentando novamente...");
            vTaskDelay(pdMS_TO_TICKS(2000)); // Espera 2s antes de tentar novamente
        }
    }

}

void setup() {
    pinMode(PIN_BUILTIN_LED, OUTPUT);
    digitalWrite(PIN_BUILTIN_LED, LOW); // Desliga LED interno

    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW); // Desliga Buzzer

    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    nodeMacAddress = WiFi.macAddress();
    Serial.printf("\nWiFi OK. MAC: %s\n", nodeMacAddress.c_str());
    
    if (esp_now_init() != ESP_OK) { Serial.println("Erro ESP-NOW"); return; }
    
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv); // Necessário para receber Hora

    // Configura Peer (Master)
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, masterMacAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    // Inicialização do SD Card
    if (!SD.begin(PIN_SD_CS)) {
        Serial.println("[SD] Falha ao inicializar o cartão SD!");
    } else {
        Serial.println("[SD] Cartão SD inicializado com sucesso");
    }

    // FreeRTOS
    g_sensor_queue = xQueueCreate(10, sizeof(sensor_payload_t));
    g_sdcard_queue = xQueueCreate(20, sizeof(sensor_payload_t)); // Fila maior para armazenar falhas
    g_evt_group = xEventGroupCreate();

    scanForMaster();

    xTaskCreate(vTimeSyncTask, "Sync", 2048, NULL, 3, NULL); // Prioridade Alta
    xTaskCreate(vSenderTask, "Send", 4096, NULL, 2, NULL);
    xTaskCreate(vTaskSDCard, "SDCard", 4096, NULL, 2, NULL); // Mesma prioridade do Sender
    xTaskCreate(system_test_mode ? vMockedSensorDHT11 : vSensorDHT11, "DHT11", 2048, NULL, 1, NULL);
    xTaskCreate(system_test_mode ? vMockedSensorMQ135 : vSensorMQ135, "MQ135", 2048, NULL, 1, NULL);
    xTaskCreate(system_test_mode ? vMockedSensorLDR : vSensorLDR, "LDR", 2048, NULL, 1, NULL);
    xTaskCreate(vTaskBuzzer, "BuzzerTask", 1024, NULL, 1, NULL);
}

void loop() {
    vTaskDelete(NULL);
}