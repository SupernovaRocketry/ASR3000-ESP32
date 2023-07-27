#include <iostream>
#include <Arduino.h> // permite o C++ compilar códigos para arduino/esp32
#include "SD.h" // biblioteca que proporciona o manuseamento do cartão SD
#include "FS.h" // biblioteca que auxilia no manuseamento do cartão SD
#include <SPI.h> // biblioteca que realiza a conexão SPI
#include <Adafruit_BMP280.h> // biblioteca do sensor BMP280
#include <Wire.h> // Na vdd eu n sei mto pq que eu usei isso, deve ser dependência do SPI ou do ADAFRUIT
#include <TinyGPS++.h> // biblioteca do gps
#include <SoftwareSerial.h> // usa no gps
#include <LoRa.h> // lora

//#include <main.h>
#include <dados.h>


// variáveis dos dados do sensor BMP280
extern double pressao_atual;
extern double altitude_atual;
extern double temperatura_atual;


// variáveis do sensor mpu
extern int MPU;
extern int AcX_atual, AcY_atual, AcZ_atual, Tmp, GyX_atual, GyY_atual, GyZ_atual;

// variáveis do GPS
extern int RXPin;
extern int TXPin;
extern int GPSBaud;
extern TinyGPSPlus gps;

// variaveis do lora
extern byte localAddress; // Endereco deste dispositivo LoRa
extern byte msgCount;         // Contador de mensagens enviadas
extern byte destination;  // Endereco do dispositivo para enviar a mensagem (0 xFF envia para todos devices )
extern String string_dados_lora;


//extern SoftwareSerial gpsSerial(RXPin, TXPin);
extern double latitude_atual;
extern double longitude_atual;
extern uint32_t tempo_atual;
extern uint32_t data_atual;

//contadores 
extern int contador;
extern int contador_bmp;



// string que recebe cada linha nova de dados que vai ser gravada no arquivo .txt no cartão SD
extern String string_dados_sd;

// listas dos valores de pressão, temperatura e altitude (e seus respectivos tamanhos)
extern int pressure_values[100];
extern int temperature_values[100];
extern int altitude_values[100];
extern double velocidade_values[100];
extern int AcX_values[100];
extern int AcY_values[100];
extern int AcZ_values[100];
extern int GyX_values[100];
extern int GyY_values[100];
extern int GyZ_values[100];
extern uint32_t data_values[100];
extern uint32_t tempo_values[100];
extern double latitude_values[100];
extern double longitude_values[100];
extern int pressure_values_size;
extern int temperature_values_size;
extern int altitude_values_size;
extern int velocidade_values_size;
extern int AcX_values_size;
extern int AcY_values_size;
extern int AcZ_values_size;
extern int GyX_values_size;
extern int GyY_values_size;
extern int GyZ_values_size;
extern int data_values_size;
extern int tempo_values_size;
extern int latitude_values_size;
extern int longitude_values_size;

// função que recolhe os dados do sensor e recebe como parâmetro o objeto da classe Adafruit_BMP280
// void sensor_bmp(Adafruit_BMP280& sensor)
// {
//     pressao_atual = sensor.readPressure();
//     temperatura_atual = sensor.readTemperature();
//     altitude_atual = sensor.readAltitude();
// }

// void sensor_mpu()
// {
//   Wire.beginTransmission(MPU);
//   Wire.write(0x3B);
//   Wire.endTransmission(0);
//   Wire.requestFrom(MPU, 14, 1);
//   AcX_atual = Wire.read() << 8 | Wire.read();
//   AcY_atual = Wire.read() << 8 | Wire.read();
//   AcZ_atual = Wire.read() << 8 | Wire.read();
//   // Tmp = Wire.read() << 8 | Wire.read();
//   GyX_atual = Wire.read() << 8 | Wire.read();
//   GyY_atual = Wire.read() << 8 | Wire.read();
//   GyZ_atual = Wire.read() << 8 | Wire.read();
//   Serial.print(" AcX = ");
//   Serial.print(AcX_atual);
//   Serial.print(" | AcY = ");
//   Serial.print(AcY_atual);
//   Serial.print(" | AcZ = ");
//   Serial.print(AcZ_atual);
//   // Serial.print(" | Tmp = ");
//   // Serial.print(Tmp / 340.00 + 36.53);
//   Serial.print(" | GyX = ");
//   Serial.print(GyX_atual);
//   Serial.print(" | GyY = ");
//   Serial.print(GyY_atual);
//   Serial.print(" | GyZ = ");
//   Serial.println(GyZ_atual);
// }

void sensor_GPS()
{
  if(gps.time.isValid()&&gps.date.isValid())
  {
    tempo_atual = gps.time.value();
    data_atual = gps.date.value();
  }

  if(gps.location.isValid())
  {
    latitude_atual = gps.location.lat();
    longitude_atual = gps.location.lng();
    Serial.println(gps.location.lat()); // IMPRIME NA SERIAL O VALOR DA LATIDUE LIDA
    Serial.println(gps.location.lng()); // IMPRIME NA SERIAL O VALOR DA LONGITUDE LIDA

  }
  if(millis() > 5000 && gps.charsProcessed() < 10) 
  {
    Serial.println("Sinal GPS não detectado");
    while (true) delay(10);
  }
}


// função que grava os arquivos no cartão SD quando chamada
void grava_SD(fs :: FS &fs , const char * path)
{
  contador ++;
  File file = fs.open(path, FILE_APPEND);

  if(file)
  {
    for(int i = 0; i < pressure_values_size; i++)
    {
      string_dados_sd += millis();
      string_dados_sd += ",";
      string_dados_sd += altitude_values[i];
      string_dados_sd += ",";
      string_dados_sd += temperature_values[i];
      string_dados_sd += ",";
      string_dados_sd += pressure_values[i];
      string_dados_sd += ",";
      string_dados_sd += AcX_values[i];
      string_dados_sd += ",";
      string_dados_sd += AcY_values[i];
      string_dados_sd += ",";
      string_dados_sd += AcZ_values[i];
      string_dados_sd += ",";
      string_dados_sd += GyX_values[i];
      string_dados_sd += ",";
      string_dados_sd += GyY_values[i];
      string_dados_sd += ",";
      string_dados_sd += GyZ_values[i];
      string_dados_sd += ",";
      string_dados_sd += data_values[i];
      string_dados_sd += ",";
      string_dados_sd += tempo_values[i];
      string_dados_sd += ",";
      string_dados_sd += latitude_values[i];
      string_dados_sd += ",";
      string_dados_sd += longitude_values[i];
      
      file.println(string_dados_sd);
    }
    file.close();
    string_dados_lora = string_dados_sd;
    string_dados_sd = "";

    Serial.println("Arquivo Atualizado");
    memset(pressure_values, 0, sizeof(pressure_values));
    memset(temperature_values, 0, sizeof(temperature_values));
    memset(pressure_values, 0, sizeof(altitude_values));
    memset(velocidade_values, 0, sizeof(velocidade_values));
    memset(AcX_values, 0, sizeof(AcX_values));
    memset(AcY_values, 0, sizeof(AcY_values));
    memset(AcZ_values, 0, sizeof(AcZ_values));
    memset(GyX_values, 0, sizeof(GyX_values));
    memset(GyY_values, 0, sizeof(GyY_values));
    memset(GyZ_values, 0, sizeof(GyZ_values));
    memset(data_values, 0, sizeof(data_values));
    memset(tempo_values, 0, sizeof(tempo_values));
    memset(latitude_values, 0, sizeof(latitude_values));
    memset(longitude_values, 0, sizeof(longitude_values));
  }
  else
  {
    Serial.println("Falha ao acessar o arquivo.");
  }
}


// void envia_LoRa()
// {
//   LoRa.beginPacket();
//   LoRa.write(destination);       // Adiciona o endereco de destino
//   LoRa.write(localAddress);
//   LoRa.write(msgCount);  
//   LoRa.write(string_dados_lora.length()); // Tamanho da mensagem em bytes
//   LoRa.print(string_dados_lora);          // Vetor da mensagem
//   msgCount++;                    // Contador do numero de mensagnes enviadas
//   LoRa.endPacket();

//   Serial.println(" Enviando os dados ao LoRa");
//   string_dados_lora = "";
//   msgCount++;   
//   vTaskDelay(1000);
// }