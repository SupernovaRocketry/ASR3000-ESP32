#ifndef DEFS_H
#define DEFS_H

//pinos I2C do BMP
#define BMP_SDA 21
#define BMP_SCL 22

// RXPin e TXPin do GPS são 16 e 17 respectivamente, estão declarados no Main porque não pode usar #define Gps só aceita int 
//Colocando aqui o int, quando chamasse o include ia haver multiplas declarações da variavel


 // #define SERIAL_DEBUG
// #define LORA_DEBUG
 #define ACIONAMENTO_DEBUG
//#define SUPERVISORIO_DEBUG
//pinos do LoRa
#define csPin 5  // Chip Select ( Slave Select do protocolo SPI ) do modulo Lora
#define resetPin 2 // Reset do modulo LoRa
#define irqPin 4   // Pino DI0

#define BAND 915E6 // Frequência do módulo LoRa

//Definições de input, define cada pino para cada variável abaixo
#define PINO_BUZZER 32
#define PINO_BOTAO 34
#define PINO_LED 33
#define REC_MAIN 25
#define REC_DROGUE 26
#define RBF 27
// pinos SPI do módulo SD
#define SCK_PIN 14
#define MISO_PIN 12
#define MOSI_PIN 13
#define CS_PIN 15

#define P0 1013.25 // pressão atmosférica média no nível do mar caso precise, mas o bmp.readAltitude ja possui o parâmetro float seaLevelhPa = 1013.25 na documentação

#define THRESHOLD_DESCIDA 5.0   //em metros
#define THRESHOLD_SUBIDA 5.0  //em metros
#define ALTURA_MAIN 50.0 //em metros
#define ERRO_BMP 'b' //inicializa uma variável de erro para o BMP 

#define ERRO_SD 's' //inicializa uma variável de erro para o leitor SD
#define ERRO_LORA 'l' //inicializa uma variável de erro para o LoRa
#define ERRO_GPS 'g' //inicializa uma variável de erro para o GPS
#define ERRO_ACEL 'a' //inicializa uma variável de erro para o MPU
//definição de estados
#define ESTADO_GRAVANDO 'g'
#define ESTADO_FINALIZADO 'f'
#define ESTADO_RECUPERANDO 'r'
#define ESTADO_RECUPERAMAIN 'm'
#define ESTADO_ESPERA 'e'


#define SD_QUEUE_LENGTH 100
#define SD_MAX 50
#define LORA_QUEUE_LENGTH 10
#define LORA_MAX 1

#endif