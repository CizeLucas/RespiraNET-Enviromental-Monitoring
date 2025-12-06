# Monitoramento Ambiental (Sistemas Embarcados)

Projeto da disciplina de Sistemas Embarcados: rede de nós remotos (slaves) que coletam dados ambientais e um nó central (master) que agrega e publica via MQTT para um servidor externo.

## Visão Geral
- **Slaves:** sensores remotos (ex.: temperatura/umidade) enviam leituras para o master.
- **Master:** recebe dados dos slaves e publica em tópicos MQTT para o servidor/broker configurado.
- **Comunicação Externa:** MQTT (ex.: Mosquitto, HiveMQ, etc.).

## Estrutura do Repositório
- `master/`: firmware do nó central (MQTT publisher)
- `slave/`: firmware dos nós remotos (coleta e envio ao master)
- Cada pasta possui `platformio.ini`, `src/main.cpp`, `include/`, `lib/`, `test/`.

## Requisitos
- PlatformIO instalado (VS Code extension ou CLI)
- Placa(s) suportada(s) configurada(s) em `platformio.ini` (ex.: ESP32/ESP8266)
- Acesso ao broker MQTT (host, porta, usuário/senha se aplicável)

## Configuração Rápida
1. Abra a pasta `master/` ou `slave/` no VS Code/PlatformIO.
2. Ajuste credenciais e parâmetros (Wi-Fi, broker MQTT, tópicos) nos arquivos de configuração/código em `src/main.cpp` e/ou `include/` conforme necessário.
3. Conecte a placa via USB.

## Build e Upload (PlatformIO CLI)
Use o VS Code com a extensão PlatformIO propriamente instalada (Build/Upload/Monitor).

## MQTT
- **Tópicos (exemplo):**
	- `env/master/<MAC_MASTER>/<LEITURA_DE_INTERESSE>` para dados dos nós escravos daquele master
- **Payload:** JSON com leituras (ex.: `{ "id": "<MAC_NÓ>", "value": 25.3, "timestamp": "2025-12-06T19:50:18Z" }`).

## Execução Básica
1. Suba o firmware dos slaves e verifique que publicam/encaminham dados ao master.
2. Suba o firmware do master e confirme conexão ao Wi-Fi e ao broker MQTT.
3. Assine os tópicos no broker para observar as mensagens.

## Troubleshooting
- Cheque o `serial monitor` para logs de conexão Wi-Fi e MQTT.
- Verifique SSID/senha e disponibilidade do broker.
- Garanta que os IDs/tópicos dos slaves e do master estão alinhados.

## Licença
Uso acadêmico na disciplina de Sistemas Embarcados.
