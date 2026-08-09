#include <DHT.h>
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"

// ---------------- Configuração LoRa ----------------
#define RF_FREQUENCY        915000000 // Hz (ajustado para 915 MHz)
#define TX_OUTPUT_POWER     20
#define LORA_BANDWIDTH      0         // 125 kHz
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE     1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define RX_TIMEOUT_VALUE    1000
#define BUFFER_SIZE         100

#define MY_ADDRESS   43   // Endereço do rádio do satélite
#define DEST_ADDRESS 42   // Endereço da estação base

// ---------------- Variáveis Globais ----------------
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

unsigned long lastTxTime = 0;
const unsigned long txInterval = 2000; // envia a cada 2s
bool lora_idle = true;
uint32_t packetCount = 0;
int16_t lastRSSI = 0;

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssiValue, int8_t snr);

// ---------------- Sensor DHT ----------------
#define DHTPIN 48
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// ---------------- Display OLED ----------------
// Renomeado para evitar colisão com a instância 'display' da biblioteca Heltec
SSD1306Wire myDisplay(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

void setup() {
  Serial.begin(115200);

  // Inicia sensor
  dht.begin();

  // Inicia display
  myDisplay.init();
  myDisplay.clear();
  myDisplay.setFont(ArialMT_Plain_10);
  myDisplay.drawString(0, 0, "Inicializando...");
  myDisplay.display();

  // Inicia rádio
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  Radio.Rx(0);

  Serial.println("Sistema iniciado...");
}

void loop() {
  Radio.IrqProcess();

  if (millis() - lastTxTime >= txInterval && lora_idle) {
    float temp = dht.readTemperature();
    float hum  = dht.readHumidity();

    if (isnan(temp) || isnan(hum)) {
      Serial.println("Erro ao ler DHT!");
      temp = 0.0;
      hum  = 0.0;
    }

    packetCount++;

    memset(txpacket, 0, BUFFER_SIZE);
    txpacket[0] = MY_ADDRESS;
    txpacket[1] = DEST_ADDRESS;

    // Monta pacote: número do pacote, temperatura, umidade, último RSSI recebido
    snprintf(&txpacket[2], BUFFER_SIZE - 2, "%lu:%.2f:%.2f:%d", 
             packetCount, temp, hum, lastRSSI);

    Serial.printf("Enviando -> Pacote %lu | Temp: %.2f | Umid: %.2f | RSSI: %d\n", 
                  packetCount, temp, hum, lastRSSI);

    // ---- Mostra no display ----
    myDisplay.clear();
    myDisplay.drawString(0, 0, "Enviando pacote:");
    myDisplay.drawString(0, 12, "ID: " + String(packetCount));
    myDisplay.drawString(0, 24, "Temp: " + String(temp, 1) + " C");
    myDisplay.drawString(0, 36, "Umid: " + String(hum, 1) + " %");
    myDisplay.drawString(0, 48, "RSSI: " + String(lastRSSI));
    myDisplay.display();

    Radio.Send((uint8_t *)txpacket, strlen((char *)txpacket));
    lora_idle = false;
    lastTxTime = millis();
  }
}

// ---------------- Callbacks LoRa ----------------
void OnTxDone() {
  lora_idle = true;
  Radio.Rx(0);
}

void OnTxTimeout() {
  Radio.Sleep();
  lora_idle = true;
  Radio.Rx(0);
}

// Quando recebe algo (ex: ACK da base com RSSI)
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssiValue, int8_t snr) {
  if (size < 2) return;

  uint8_t sender = payload[0];
  uint8_t receiver = payload[1];

  if (receiver != MY_ADDRESS || sender != DEST_ADDRESS) {
    lora_idle = true;
    Radio.Rx(0);
    return;
  }

  memset(rxpacket, 0, BUFFER_SIZE);
  memcpy(rxpacket, &payload[2], size - 2);
  rxpacket[size - 2] = '\0';

  lastRSSI = rssiValue; // salva RSSI do último pacote recebido

  Serial.printf("Recebido de %d -> %s | RSSI: %d dBm\n", sender, rxpacket, lastRSSI);

  // Atualiza display também
  myDisplay.clear();
  myDisplay.drawString(0, 0, "ACK recebido!");
  myDisplay.drawString(0, 16, "De: " + String(sender));
  myDisplay.drawString(0, 28, "Msg: " + String(rxpacket));
  myDisplay.drawString(0, 44, "RSSI: " + String(lastRSSI) + " dBm");
  myDisplay.display();

  lora_idle = true;
  Radio.Rx(0);
}
