#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"

// ---------- Configurações ----------
#define MY_ADDRESS 42
#define DEST_ADDRESS 43

#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm
#define LORA_BANDWIDTH                              0         // 125 kHz
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // 4/5
#define LORA_PREAMBLE_LENGTH                        8
#define LORA_SYMBOL_TIMEOUT                         0
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 300  

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;

int16_t txNumber;
int16_t rssi, rxSize;
uint32_t pacoteRecebidoCount = 0; // Contador de pacotes

bool lora_idle = true;
String inputString = "";

// Display OLED
SSD1306Wire factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

    Wire.begin(SDA_OLED, SCL_OLED);
    factory_display.init();
    factory_display.clear();
    factory_display.display();
    factory_display.drawString(0, 0, "Iniciando...");
    factory_display.display();

    txNumber = 0;
    rssi = 0;

    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    delay(1000);
    factory_display.clear();
    factory_display.drawString(0, 0, "Aguardando pacotes...");
    factory_display.display();
}

void loop() {
    // Envio via Serial
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') {
            inputString.trim();
            if (inputString.length() > 0 && inputString.length() < BUFFER_SIZE - 2) {
                memset(txpacket, 0, BUFFER_SIZE);
                txpacket[0] = MY_ADDRESS;       // Endereço de origem
                txpacket[1] = DEST_ADDRESS;     // Endereço de destino
                inputString.toCharArray(&txpacket[2], BUFFER_SIZE - 2);

                if (txpacket[1] == DEST_ADDRESS) {
                    int msgLen = inputString.length();
                    Radio.Send((uint8_t *)txpacket, msgLen + 2);  // Envia cabeçalho + mensagem
                } 
                lora_idle = false;
            }
            inputString = "";
        } else {
            inputString += inChar;
        }
    }

    if (lora_idle) {
        lora_idle = false;
        Radio.Rx(0);
    }

    Radio.IrqProcess();
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssiValue, int8_t snr) {
    rssi = rssiValue;
    rxSize = size;

    if (size < 2) return; // Pacote muito pequeno

    uint8_t sender = payload[0];
    uint8_t receiver = payload[1];

    if (receiver != MY_ADDRESS || sender != DEST_ADDRESS) {
        // Ignora se não for pra mim ou se não veio do 43
        lora_idle = true;
        Radio.Rx(0);
        return;
    }

    memcpy(rxpacket, &payload[2], size - 2);
    rxpacket[size - 2] = '\0';
    Radio.Sleep();

    String displayMessage = String((char*)rxpacket);
    pacoteRecebidoCount++;

    // Exibe o número de pacotes antes do RSSI e tamanho
    Serial.printf(" %s%lu:%d:%d\n", displayMessage.c_str(), pacoteRecebidoCount, rssi, rxSize);

    factory_display.clear();
    factory_display.drawString(0, 0, "Telemetria IREC");
    factory_display.drawString(0, 10, "Mensagem:");
    factory_display.drawString(0, 20, displayMessage);

    // Opcional: mostrar o número do pacote na tela também
    // factory_display.drawString(0, 40, "Pacote #: " + String(pacoteRecebidoCount));

    factory_display.display();

    lora_idle = true;
}

void OnTxDone() {
    lora_idle = true;
    Radio.Rx(0);
}

void OnTxTimeout() {
    Radio.Sleep();
    lora_idle = true;
    Radio.Rx(0);
}
