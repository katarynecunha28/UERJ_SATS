#include "DHT.h"
#include <SPI.h>
#include <LoRa.h>

// Configurações do DHT22
#define DHTPIN 12
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Configurações LoRa
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

int meuEndereco = 42;
char stemp[400];
int contadorPacotes = 0;

// Setup
void setup()
{
  Serial.begin(9600);

  // Inicializa o DHT22
  dht.begin();

  // Inicializa o LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(915E6))
  {
    Serial.println("Falha ao inicializar o LoRa!");
    while (1);
  }

  Serial.println("LoRa Inicializado!");
  delay(1000);
}

// Loop principal
void loop()
{
  // Lê os valores do sensor DHT22
  float umidade = dht.readHumidity();
  float temperaturaDHT = dht.readTemperature();

  // Cria a mensagem com os dados
  sprintf(stemp, "%.2f, %.2f, Pacote: %d", temperaturaDHT, umidade, contadorPacotes);

  // Envia os dados via LoRa
  enviarSensores();

  delay(1500);
}

// Função para enviar dados via LoRa
void enviarSensores()
{
  LoRa.beginPacket();
  LoRa.write(meuEndereco);
  LoRa.print(stemp);
  LoRa.endPacket();

  contadorPacotes++;

  Serial.print("Mensagem enviada via LoRa: ");
  Serial.println(stemp);

  delay(1000);
}
