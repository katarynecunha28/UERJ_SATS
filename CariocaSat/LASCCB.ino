#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "float16.h"

#define RF_FREQUENCY 915000000 // Hz
#define TX_OUTPUT_POWER 20

#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 300

#define MY_ADDRESS 43 //Endereço do rádio do satélite
#define DEST_ADDRESS 42 //Endereço da estação base

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

double txNumber;
bool lora_idle = true;

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssiValue, int8_t snr);  

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
HardwareSerial cncSerial(2);

#define SEALEVELPRESSURE_HPA (950)
#define DHTPIN 48
#define DHTTYPE DHT22
#define TIMEZONE_OFFSET -3

Adafruit_BME280 bme;
DHT dht(DHTPIN, DHTTYPE);
Adafruit_MPU6050 mpu;

unsigned long startTime;
unsigned long lastTxTime = 0;
const unsigned long txInterval = 2000; // envia a cada 2s

String powerSupplyData = " ";

class EE24CXXX {
private:
    byte _device_address;

public:
    EE24CXXX(byte device_address) : _device_address(device_address) {}
    void write(unsigned int eeaddress, unsigned char *data, unsigned int data_len);
    void read(unsigned int eeaddress, unsigned char *data, unsigned int data_len);

    template <class T>
    int write(unsigned int eeaddress, const T &value);
    template <class T>
    int read(unsigned int eeaddress, T &value);
};

void EE24CXXX::write(unsigned int eeaddress, unsigned char *data, unsigned int data_len) {
    while (data_len > 0) {
        Wire.beginTransmission(_device_address);
        Wire.write((int)(eeaddress >> 8));
        Wire.write((int)(eeaddress & 0xFF));

        byte bytesToWrite = min(data_len, (unsigned int)16);
        for (byte i = 0; i < bytesToWrite; i++) {
            Wire.write(data[i]);
        }

        Wire.endTransmission();
        eeaddress += bytesToWrite;
        data += bytesToWrite;
        data_len -= bytesToWrite;

        delay(5);
    }
}

void EE24CXXX::read(unsigned int eeaddress, unsigned char *data, unsigned int data_len) {
    while (data_len > 0) {
        Wire.beginTransmission(_device_address);
        Wire.write((int)(eeaddress >> 8));
        Wire.write((int)(eeaddress & 0xFF));
        Wire.endTransmission();

        byte bytesToRead = min(data_len, (unsigned int)28);
        Wire.requestFrom(_device_address, bytesToRead);

        for (byte i = 0; i < bytesToRead && Wire.available(); i++) {
            data[i] = Wire.read();
        }

        data += bytesToRead;
        eeaddress += bytesToRead;
        data_len -= bytesToRead;
    }
}

template <class T> int EE24CXXX::write(unsigned int eeaddress, const T &value) {
    write(eeaddress, (unsigned char *)&value, sizeof(T));
    return sizeof(T);
}

template <class T> int EE24CXXX::read(unsigned int eeaddress, T &value) {
    read(eeaddress, (unsigned char *)&value, sizeof(T));
    return sizeof(T);
}

EE24CXXX eeprom(0x50);
int currentAddress = 0;

struct sensorsData {
    unsigned short seconds;
    float temperatureDHT;
    float humidityDHT;
    float pressure;
    int sats;
    float altitude;
    float latitude;
    float longitude;
    float accelX;
    float accelY;
    float accelZ;
};

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600, SERIAL_8N1, 45, 46);
    cncSerial.begin(115200,SERIAL_8N1, 20,19);

    if (!bme.begin(0x76)) {
        Serial.println("Erro ao inicializar o BME280!");
    }

    dht.begin();
    Wire.begin();

    if (!mpu.begin()) {
        Serial.println("Erro ao inicializar o MPU6050!");
    }

    startTime = millis();

    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

    txNumber = 0;

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

}

void loop() {
    while (Serial.available()) {
        String receivedData = Serial.readStringUntil('\n');
        receivedData.trim();
        if (receivedData != "1") 
        {
            powerSupplyData = receivedData;
        }
    }

    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        gps.encode(c);
    }

    Radio.IrqProcess();
    
    //Contador

    if (millis() - lastTxTime >= txInterval && lora_idle) {
        sensorsData dados;
        unsigned short elapsedTime = (millis() - startTime) / 1000;
        dados.seconds = elapsedTime;
        
        //Dados DHT
        dados.temperatureDHT = dht.readTemperature();
        if (isnan(dados.temperatureDHT)) dados.temperatureDHT = 0.0;

        dados.humidityDHT = dht.readHumidity();
        if (isnan(dados.humidityDHT)) dados.humidityDHT = 0.0;
        
        //Dados do BME
        dados.pressure = bme.readPressure() / 100.0F;
        dados.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
        
        //Dados GPS
        dados.latitude = gps.location.lat();
        dados.longitude = gps.location.lng();
        dados.sats = gps.satellites.value();

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        //Dados GY
        dados.accelX = a.acceleration.x;
        dados.accelY = a.acceleration.y;
        dados.accelZ = a.acceleration.z;
        
        // Escrita na EEPROM
        // Só irão ser escritos de 20 em 20 bytes, e irá cessar no máximo da memoria (1484 bytes)
        if ((currentAddress <= 1480) && (dados.altitude >= 600.00)) {

          // float 4 bytes -> float 2 bytes
          eeprom.write(currentAddress, float16(dados.seconds)); currentAddress += 2;
          eeprom.write(currentAddress, float16(dados.temperatureDHT)); currentAddress += 2;
          eeprom.write(currentAddress, float16(dados.humidityDHT)); currentAddress += 2;
          eeprom.write(currentAddress, float16(dados.pressure)); currentAddress += 2;
          eeprom.write(currentAddress, float16(dados.altitude)); currentAddress += 2;
          eeprom.write(currentAddress, float16(dados.latitude)); currentAddress += 2;
          eeprom.write(currentAddress, float16(dados.longitude)); currentAddress += 2;
          eeprom.write(currentAddress, float16(dados.accelX)); currentAddress += 2;
          eeprom.write(currentAddress, float16(dados.accelY)); currentAddress += 2;
          eeprom.write(currentAddress, float16(dados.accelZ)); currentAddress += 2;

        }

        memset(txpacket, 0, BUFFER_SIZE);
        txpacket[0] = MY_ADDRESS;
        txpacket[1] = DEST_ADDRESS;
        
        //Monta o pacote LoRa
        snprintf(&txpacket[2], BUFFER_SIZE - 2, "%u:%.2f:%.2f:%.2f:%.2f:%d:%.6f:%.6f:%.2f:%.2f:%.2f:%s",
                 dados.seconds, dados.temperatureDHT,
                 dados.humidityDHT, dados.altitude,
                 dados.pressure, dados.sats, dados.latitude, dados.longitude,
                 dados.accelX, dados.accelY, dados.accelZ, powerSupplyData.c_str());
        
        //Envia dados para estação base
        if (txpacket[1] == DEST_ADDRESS) {
            Radio.Send((uint8_t *)txpacket, strlen((char *)txpacket));
            lora_idle = false;
            lastTxTime = millis();
        }
    }
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

//Recebe Telecomando
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
    Radio.Sleep();

    if (strcmp(rxpacket, "1") == 0) {
        cncSerial.println("$3=3");
        cncSerial.println("$23=7");
        cncSerial.println("$27=5");
        cncSerial.println("$H");
        //cncSerial.println("$H"); TEM QUE TROCAR AS CONFIGS 
        //cncSerial.println("G92 X0 Y0 Z0");
        //delay(5000);
        //cncSerial.println("G1 X5 F30");
        //delay(5000);
        //cncSerial.println("G1 Y5 F30");
        delay(120000);
        Serial.println("1");
        cncSerial.println("G1 Z4.5 F30");
        cncSerial.println("G1 X50 F30");
        cncSerial.println("G1 Y50 F30");
        cncSerial.println("G1 Z0 F30");
        cncSerial.println("G1 X0 F30");
        cncSerial.println("G1 Y0 F30");
        //delay();
        //cncSerial.println("G1 Z2 F30");
        //Serial.println("2");
    }
    else if(strcmp(rxpacket, "2") == 0) 
    {
        Serial.println("2");
    }
    else if(strcmp(rxpacket, "3") == 0)
    {
        cncSerial.println("$3=3");
        cncSerial.println("$23=7");
        cncSerial.println("$27=5");
        cncSerial.println("$H");
        delay(120000);
        Serial.println("1");
        cncSerial.println("G1 Y20 F30");
        cncSerial.println("G1 Z4.5 F30");
        cncSerial.println("G1 X50 F30");
        cncSerial.println("G1 Y5 F30");
        cncSerial.println("G1 Z0 F30");
        cncSerial.println("G1 X0 F30");
        cncSerial.println("G1 Y0 F30");
    }
    else if(strcmp(rxpacket, "4") == 0)
    {
        cncSerial.println("!");
        delay(100);
        cncSerial.println("0x18");
        cncSerial.println("$H");
    }
    else if(strcmp(rxpacket, "5") == 0)
    {
        Serial.println("1");
    }

    lora_idle = true;
}
