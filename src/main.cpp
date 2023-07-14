//Inclusão das bibliotecas utilizadas
#include <iostream>
#include <Arduino.h> // permite o C++ compilar códigos para arduino/esp32
#include "SD.h" // biblioteca que proporciona o manuseamento do cartão SD
#include "FS.h" // biblioteca que auxilia no manuseamento do cartão SD
#include <SPI.h> // biblioteca que realiza a conexão SPI
#include <Adafruit_BMP280.h> // biblioteca do sensor BMP280
#include <Wire.h> // Na vdd eu n sei mto pq que eu usei isso, deve ser dependência do SPI ou do ADAFRUIT
#include "freertos/FreeRTOS.h" // OS para realizar o multitasking
#include "freertos/task.h" // biblioteca que proporciona a criação e manuseamento das Tasks
#include <TinyGPS++.h> // biblioteca do GPS
#include <SoftwareSerial.h> // usa no gps
#include <LoRa.h> // lora

// headers
#include <main.h>
#include <dados.h>
#include <defs.h>

// variáveis dos dados do sensor BMP280
Adafruit_BMP280 bmp;
double pressao_atual;
double altitude_atual;
double temperatura_atual;

//variáveis do sensor MPU6050
int MPU = 0x68;
int AcX_atual, AcY_atual, AcZ_atual, Tmp, GyX_atual, GyY_atual, GyZ_atual;

// variáveis do GPS
int GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);
uint32_t data_atual = 0;
uint32_t tempo_atual = 0;
double latitude_atual = 0;
double longitude_atual = 0;

// variaveis do lora
byte localAddress = 0xBB; // Endereco deste dispositivo LoRa
byte msgCount = 0;         // Contador de mensagens enviadas
byte destination = 0xFF;  // Endereco do dispositivo para enviar a mensagem (0 xFF envia para todos devices )
String string_dados_lora;


//contadores 
int contador = 0;
int contador_bmp = 0;
int contador_mpu = 0;
int contador_gps = 0;

// string que recebe cada linha nova de dados que vai ser gravada no arquivo .txt no cartão SD
String string_dados_sd;

// listas dos valores de pressão, temperatura e altitude (e seus respectivos tamanhos)
int pressure_values[100] = {};
int temperature_values[100] = {};
int altitude_values[100] = {};
int pressure_values_size = sizeof(pressure_values) / sizeof(pressure_values[0]);
int temperature_values_size = sizeof(temperature_values) / sizeof(temperature_values[0]);
int altitude_values_size = sizeof(altitude_values) / sizeof(altitude_values[0]);

// listas dos valores do acelerômetro e giroscópio
int AcX_values[100] = {};
int AcY_values[100] = {};
int AcZ_values[100] = {};
int GyX_values[100] = {};
int GyY_values[100] = {};
int GyZ_values[100] = {};
int AcX_values_size = sizeof(AcX_values) / sizeof(AcX_values[0]);
int AcY_values_size = sizeof(AcY_values) / sizeof(AcY_values[0]);
int AcZ_values_size = sizeof(AcZ_values) / sizeof(AcZ_values[0]);
int GyX_values_size = sizeof(GyX_values) / sizeof(GyX_values[0]);
int GyY_values_size = sizeof(GyY_values) / sizeof(GyY_values[0]);
int GyZ_values_size = sizeof(GyZ_values) / sizeof(GyZ_values[0]);

// listas dos valores de hora, data, latitude e longitude do GPS
uint32_t data_values[100] = {};
uint32_t tempo_values[100] = {};
double latitude_values[100] = {};
double longitude_values[100] = {};
int data_values_size = sizeof(data_values) / sizeof(data_values[0]);
int tempo_values_size = sizeof(tempo_values) / sizeof(tempo_values[0]);
int latitude_values_size = sizeof(latitude_values) / sizeof(latitude_values[0]);
int longitude_values_size = sizeof(longitude_values) / sizeof(longitude_values[0]);


SemaphoreHandle_t xMutex; // objeto do semáforo das tasks


void task_bmp(void *pvParameters) // task do bmp
{
  while(1)
  {
    Serial.println("Task de apreensão dos dados do BMP iniciada");
    if(xMutex != NULL)
    {
      if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
      {
        while(contador_bmp < 100)
        {
          sensor_bmp(bmp);
          Serial.println(contador_bmp);
          String data_line = "Altitude atual: " + String(altitude_atual) + "| Temperatura: " + String(temperatura_atual) + " | Pressão: " + String(pressao_atual / 1013.25);
          Serial.println(data_line);
          altitude_values[contador_bmp] = altitude_atual;
          temperature_values[contador_bmp] = temperatura_atual;
          pressure_values[contador_bmp] = pressao_atual / P0;
          contador_bmp ++;
        }
      }
      Serial.println("acabou a task aqui");
      xSemaphoreGive(xMutex);
      contador_bmp = 0;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


void task_mpu (void * pvParameters) // task do mpu
{
  while(1)
  {
    Serial.println("Task de apreensão dos dados do MPU iniciada");
    if(xMutex != NULL)
    {
      if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
      {
        while(contador_mpu < 100)
        {
          sensor_mpu();
          Serial.println(contador_mpu);
          AcX_values[contador_mpu] = AcX_atual;
          AcY_values[contador_mpu] = AcY_atual;
          AcZ_values[contador_mpu] = AcZ_atual;
          GyX_values[contador_mpu] = GyX_atual;
          GyY_values[contador_mpu] = GyY_atual;
          GyZ_values[contador_mpu] = GyZ_atual;
          contador_mpu++;
        }
      }
      Serial.println("Acabou a task do MPU!!!_______");
      xSemaphoreGive(xMutex);
      contador_mpu = 0;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


void task_gps (void * pvParameters) // task do gps
{
  while(1)
  {
    Serial.println("Task de apreensão dos dados do GPS iniciada");
    // GPS Reading
    while (gpsSerial.available() > 0) 
    {
      if (gps.encode(gpsSerial.read())) 
      {
        while(contador_gps < 100)
        {
          sensor_GPS();
          data_values[contador_gps] = data_atual;
          tempo_values[contador_gps] = tempo_atual;
          latitude_values[contador_gps] = latitude_atual;
          longitude_values[contador_gps] = longitude_atual;
          contador_gps++;
        }
      }
      Serial.println("Acabou a task do gps");
      xSemaphoreGive(xMutex);
      contador_gps = 0;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


void task_gravaSD(void * pvParameters)// task do cartão SD
{
  while(1)
  {
    Serial.println("Task da gravação dos dados dos sensores no cartão SD iniciada");
    if(xMutex != NULL)
    {
      if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
      {
        grava_SD(SD, "/teste.txt");
      }
    }
    else xSemaphoreGive(xMutex);
    xSemaphoreGive(xMutex);
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);   
}


void task_envia_lora(void *pvParameters)//
{
  while(1)
  {
    Serial.println("Task do LoRa iniciada");
    if(xMutex != NULL)
    {
      if(xSemaphoreTake(xMutex, portMAX_DELAY)== pdTRUE)
      {
        envia_LoRa();
      }
    }
    xSemaphoreGive(xMutex);
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}


void setup() {
  xMutex = xSemaphoreCreateMutex(); // cria o objeto do semáforo xMutex
  Serial.begin(115200);
  delay(1000);
  Serial.println("Inicializando o cartão SD...");
  SPIClass spi = SPIClass (CS_PIN); //cria a classe SPI para litar com a conexão entre o cartão SD e o ESP32
  spi.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN); //inicia a conexão spi

  if(!SD.begin(CS_PIN)) //verifica se o cartão sd foi encontrado através da conexão CS do SPI
  {
    Serial.println("Cartão SD não encontrado.");
    return;
  }
  Serial.println("Cartão SD encontrado.");

  LoRa.setPins(csPin, resetPin, irqPin);

  while(!LoRa.begin(BAND)) 
  {
    Serial.println("Falha ao iniciar o módulo LoRa. Verifique as conexões.");
    delay(1000);
  }
  
  Wire.begin(); //mais uma vez, sei la pra que isso
  bmp.begin(0x76); //inicia o bmp neste endereço. Mudar para 0x78 ou parecido caso dê erro

  xTaskCreate(task_bmp, "task bmp", 3000, NULL, 1, NULL); //cria a task que trata os dados
  xTaskCreate(task_gps, "task gps", 3000, NULL, 1, NULL); //cria a task que salva no cartão SD
  xTaskCreate(task_mpu, "task mpu", 3000, NULL, 1, NULL); //cria a task que trata os dados
  xTaskCreate(task_gravaSD, "task sd", 3000, NULL, 1, NULL); //cria a task que salva no cartão SD
  xTaskCreate(task_envia_lora, "task lora", 3000, NULL, 1, NULL); //cria a task que envia os dados pelo LoRa
}


void loop() 
{

}
