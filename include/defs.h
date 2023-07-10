#ifndef DEFS_H
#define DEFS_H

//pinos I2C do BMP
#define BMP_SDA 21
#define BMP_SCL 22

//pinos do GPS. Estão em INT pois SoftwareSerial gpsSerial(RXPin, TXPin) só aceita INT como parâmetro, #define não funcionou
int RXPin = 16;
int TXPin = 17;

//pinos do LoRa
#define csPin 5  // Chip Select ( Slave Select do protocolo SPI ) do modulo Lora
#define resetPin 4 // Reset do modulo LoRa
#define irqPin 2   // Pino DI0

#define BAND 915E6 // Frequência do módulo LoRa

// pinos SPI do módulo SD
#define SCK_PIN 18
#define MISO_PIN 19
#define MOSI_PIN 23
#define CS_PIN 5

#define P0 1013.25 // pressão atmosférica média no nível do mar caso precise, mas o bmp.readAltitude ja possui o parâmetro float seaLevelhPa = 1013.25 na documentação


#endif