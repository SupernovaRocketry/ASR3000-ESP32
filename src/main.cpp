//Inclusão das bibliotecas utilizadas
#include <Arduino.h> // permite o C++ compilar códigos para arduino/esp32
#include "SD.h" // biblioteca que proporciona o manuseamento do cartão SD
#include "FS.h" // biblioteca que auxilia no manuseamento do cartão SD
#include <SPI.h> // biblioteca que realiza a conexão SPI
#include <Adafruit_BMP280.h> // biblioteca do sensor BMP280
#include <Wire.h> // Na vdd eu n sei mto pq que eu usei isso, deve ser dependência do SPI ou do ADAFRUIT
#include "freertos/FreeRTOS.h" // OS para realizar o multitasking
#include "freertos/task.h" // biblioteca que proporciona a criação e manuseamento das Tasks

// arquivo de tratamento dos dados
#include <dados.cpp>

// variáveis dos dados do sensor BMP280
Adafruit_BMP280 bmp;
#define BMP_SDA 21//Definição dos pinos I2C
#define BMP_SCL 22
double pressao_atual;
double altitude_atual;
double temperatura_atual;

//contadores 
int contador = 0;
int contador2 = 0;

// string que recebe cada linha nova de dados que vai ser gravada no arquivo .txt no cartão SD
String string_dados;

// listas dos valores de pressão, temperatura e altitude (e seus respectivos tamanhos)
int pressure_values[100] = {};
int temperature_values[100] = {};
int altitude_values[100] = {};
int pressure_values_size = sizeof(pressure_values) / sizeof(pressure_values[0]);
int temperature_values_size = sizeof(temperature_values) / sizeof(temperature_values[0]);
int altitude_values_size = sizeof(altitude_values) / sizeof(altitude_values[0]);

// pinos SPI utilizados pelo módulo SD
#define SCK_PIN 18
#define MISO_PIN 19
#define MOSI_PIN 23
#define CS_PIN 5

#define P0 1013.25 // pressão atmosférica média no nível do mar caso precise, mas o bmp.readAltitude ja possui o parâmetro float seaLevelhPa = 1013.25 na documentação

SemaphoreHandle_t xMutex; // objeto do semáforo das tasks



// Task que faz uma espécie de callback para usar a void sensor_bmp() dentro de uma task
void data(void *pvParameters)
{
  while(1)
  {
    if(xMutex != NULL)
    {
      if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
      {
        while(contador2 < 100)
        {
          sensor_bmp(bmp);
          Serial.println(contador2);
          String data_line = "Altitude atual: " + String(altitude_atual) + "| Temperatura: " + String(temperatura_atual) + " | Pressão: " + String(pressao_atual / 1013.25);
          Serial.println(data_line);
          altitude_values[contador2] = altitude_atual;
          temperature_values[contador2] = temperatura_atual;
          pressure_values[contador2] = pressao_atual / P0;
          contador2 ++;
        }
      }
      Serial.println("acabou a task aqui");
      xSemaphoreGive(xMutex);
      contador2 = 0;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


// espécie de callback que usa a função void sd_manage() para coletar os dados gravados na sensor_bmp() e jogar eles no cartão sd através da sd_manage,
//e definitr tudo isso como uma Task
void record(void * pvParameters)
{
  while(1)
  {
    Serial.println(" Chegou aqui ------------------------------------------------");
    if(xMutex != NULL)
    {

      if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
      {
        sd_manage(SD, "/teste.txt");
      }
    }
    else xSemaphoreGive(xMutex);
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
  
  Wire.begin(); //mais uma vez, sei la pra que isso
  bmp.begin(0x76); //inicia o bmp neste endereço. Mudar para 0x78 ou parecido caso dê erro

  xTaskCreate(data, "task 1", 3000, NULL, 1, NULL); //cria a task que trata os dados
  xTaskCreate(record, "task 2", 3000, NULL, 1, NULL); //cria a task que salva no cartão SD
}


void loop() 
{

}
