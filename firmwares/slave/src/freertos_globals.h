#ifndef FREERTOS_GLOBALS_H
#define FREERTOS_GLOBALS_H

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>

// --- GLOBAIS FREERTOS ---
extern QueueHandle_t g_sensor_queue;
extern QueueHandle_t g_sdcard_queue;
extern EventGroupHandle_t g_evt_group;

// --- Event Group Bits ---
#define BIT_ACK_SUCCESS  BIT0  // O envio ESP-NOW foi recebido com sucesso pelo destinatário
#define BIT_ACK_FAIL     BIT1  // O envio falhou (destinatário não recebeu)
#define BIT_SYNC_DONE    BIT2  // A resposta de sincronização de tempo chegou
#define BIT_TIME_SYNCED  BIT3  // Tempo sincronizado, libera sensores

#endif // FREERTOS_GLOBALS_H
