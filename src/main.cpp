// Inclusão das bibliotecas utilizadas
#include <iostream>
#include <Arduino.h>           // permite o C++ compilar códigos para arduino/esp32
#include "SD.h"                // biblioteca que proporciona o manuseamento do cartão SD
#include "FS.h"                // biblioteca que auxilia no manuseamento do cartão SD
#include <SPI.h>               // biblioteca que realiza a conexão SPI
#include <Adafruit_BMP280.h>   // biblioteca do sensor BMP280
#include <Wire.h>              // Na vdd eu n sei mto pq que eu usei isso, deve ser dependência do SPI ou do ADAFRUIT NÃO, É DO I2C
#include "freertos/FreeRTOS.h" // OS para realizar o multitasking
#include "freertos/task.h"     // biblioteca que proporciona a criação e manuseamento das Tasks
#include <TinyGPS++.h>         // biblioteca do GPS
#include <SoftwareSerial.h>    // usa no gps
#include <LoRa.h>              // lora
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// headers
#include <main.h>
#include <dados.h>
#include <defs.h>
#include <recuperacao.h>
#include <estados.h>


// A FAZER
// USAR BIBLIOTECA MPU (SETAR EM 16Gs) feito 
// COMBINAR TASKS DE I2C (MPU E BMP) feito
// REMOVER FUNÇÕES DESNECESSÁRIAS (QUE SÓ APARECEM 1 VEZ NO CÓDIGO
// OTIMIZAÇÃO: VER QUESTÃO DE TIPO DE VARIÁVEIS
// COLOCAR #ifdef SERIAL_DEBUG antes de QUALQUER serial (inclusive inicialização) e #endif depois
// VER SE É NECESSÁRIO USAR SEMÁFORO EM OUTRAS TASKS
// VERIFICAR USO DE BIBLIOTECAS EM OUTROS ARQUIVOS


//ACIONAMENTO
//DEFINIR MÉTODO
//DEFINIR PARÂMETROS DE ACIONAMENTO
// NA INICIALIZAÇÃO, CALIBRAR SENSORES E DETERMINAR ALTITUDE INICIAL
// Determinar questão dos remove before flight
// Determinar questão do acionamento redundante
// Determinar questão da leitura de dados, 100 vezes e grava?



// #define SERIAL_DEBUG

// variáveis dos dados do sensor BMP280
Adafruit_BMP280 bmp;
double pressao_atual;
double altitude_atual;
double temperatura_atual;
double altitude_anterior = 0;
double tempo_anterior = 0;
double tempo_vel=0;
double velocidade_atual=0;


// variáveis do sensor MPU6050
int MPU = 0x68;
Adafruit_MPU6050 mpu;
int AcX_atual, AcY_atual, AcZ_atual, Tmp, GyX_atual, GyY_atual, GyZ_atual;

// variáveis do GPS
//pinos do GPS. Estão em INT pois SoftwareSerial gpsSerial(RXPin, TXPin) só aceita INT como parâmetro, #define não funcionou
int RXPin = 16;
int TXPin = 17;
int GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);
uint32_t data_atual = 0;
uint32_t tempo_atual = 0;
double latitude_atual = 0;
double longitude_atual = 0;

// variaveis do lora
byte localAddress = 0xBB; // Endereco deste dispositivo LoRa
byte msgCount = 0;        // Contador de mensagens enviadas
byte destination = 0xFF;  // Endereco do dispositivo para enviar a mensagem (0 xFF envia para todos devices )
String string_dados_lora;

//Variáveis de dados
double alturaInicial = 0;
double alturaMinima;
double alturaMaxima =  0;

// variáveis de controle
unsigned long millisRecMain = 2000000;
unsigned long millisRecDrogue = 1000000;
bool gravando = false;
bool abriuParaquedasMain = false;
bool abriuParaquedasDrogue = false;
bool abriuRedundanciaMain = false;
bool abriuRedundanciaDrogue = false;
char erro = false;
char  statusAtual;
bool estado;
bool descendo = false;
bool subindo = false;
// contadores

// mover contadores pra antes do while (1) da respectiva task
uint32_t contador = 0;
uint32_t contador_bmp = 0;
uint32_t contador_mpu = 0;
uint32_t contador_gps = 0;

// string que recebe cada linha nova de dados que vai ser gravada no arquivo .txt no cartão SD
String string_dados_sd;

// listas dos valores de pressão, temperatura e altitude (e seus respectivos tamanhos)
int pressure_values[100] = {};
int temperature_values[100] = {};
int altitude_values[100] = {};
double velocidade_values[100] = {};
int pressure_values_size = sizeof(pressure_values) / sizeof(pressure_values[0]);
int temperature_values_size = sizeof(temperature_values) / sizeof(temperature_values[0]);
int altitude_values_size = sizeof(altitude_values) / sizeof(altitude_values[0]);
int velocidade_values_size = sizeof(velocidade_values) / sizeof(velocidade_values[0]);

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

void task_i2c_sensores(void *pvParameters)
{
  uint32_t contador_i2c;
  String data_line;
  while(1)
  {
    if(xSemaphoreTake(xMutex, portMAX_DELAY)==pdTRUE)
    {
    #ifdef SERIAL_DEBUG
          Serial.println("Task dos sensores BMP e MPU iniciada.");
    #endif
      while(contador_i2c < 100)
      {
        pressao_atual = bmp.readPressure();
        temperatura_atual = bmp.readTemperature();
        altitude_atual = bmp.readAltitude();
        tempo_vel=millis()/1000; // em segundos
        velocidade_atual = (altitude_atual - altitude_anterior)/(tempo_vel - tempo_anterior);
        altitude_anterior = altitude_atual;
        tempo_anterior = tempo_vel;
        #ifdef SERIAL_DEBUG
                data_line = "Altitude atual: " + String(altitude_atual) + "| Temperatura: " + String(temperatura_atual) + " | Pressão: " + String(pressao_atual / 1013.25)+ " | Velocidade: " + String(velocidade_atual);
                Serial.println(data_line);
        #endif
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        AcX_atual = a.gyro.x;
        AcY_atual = a.gyro.y;
        AcZ_atual = a.gyro.z;
        GyX_atual = g.gyro.x;
        GyY_atual = g.gyro.y;
        GyZ_atual = g.gyro.z;
        altitude_values[contador_i2c] = altitude_atual;
        temperature_values[contador_i2c] = temperatura_atual;
        pressure_values[contador_i2c] = pressao_atual / P0;
        velocidade_values[contador_i2c] = velocidade_atual;
        AcX_values[contador_i2c] = AcX_atual;
        AcY_values[contador_i2c] = AcY_atual;
        AcZ_values[contador_i2c] = AcZ_atual;
        GyX_values[contador_i2c] = GyX_atual;
        GyY_values[contador_i2c] = GyY_atual;
        GyZ_values[contador_i2c] = GyZ_atual;
      }
    }
    contador_i2c = 0;
    xSemaphoreGive(xMutex);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void task_gps(void *pvParameters) // task do gps
{
  while (1)
  {
    Serial.println("Task de apreensão dos dados do GPS iniciada");
    // GPS Reading
    if (xMutex != NULL)
    {
      if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
      {
        while (gpsSerial.available() > 0)
        {
          if (gps.encode(gpsSerial.read()))
          {
            while (contador_gps < 100)
            {
              sensor_GPS();
              data_values[contador_gps] = data_atual;
              tempo_values[contador_gps] = tempo_atual;
              latitude_values[contador_gps] = latitude_atual;
              longitude_values[contador_gps] = longitude_atual;
              #ifdef SERIAL_DEBUG
                            Serial.println(data_atual);
                            Serial.println(tempo_atual);
                            Serial.println(latitude_atual);
                            Serial.println(longitude_atual);
              #endif
              contador_gps++;
            }
          }
        }
        Serial.println("Acabou a task do gps");
        xSemaphoreGive(xMutex);
        contador_gps = 0;
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void task_gravaSD(void *pvParameters) // task do cartão SD
{
  while (1)
  {
    Serial.println("Task da gravação dos dados dos sensores no cartão SD iniciada");
      if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
      {
        grava_SD(SD, "/teste.txt"); //N PRECISA
        xSemaphoreGive(xMutex); //VER SOBRE COLOCAR DENTRO DOS IFS OU FORA NAS OUTRAS TASKS
      }
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void task_envia_lora(void *pvParameters) //
{
  while (1)
  {
    Serial.println("Task do LoRa iniciada");
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {   
      LoRa.beginPacket();
      LoRa.write(destination);       // Adiciona o endereco de destino
      LoRa.write(localAddress);
      LoRa.write(msgCount);  
      LoRa.write(string_dados_lora.length()); // Tamanho da mensagem em bytes
      LoRa.print(string_dados_lora);          // Vetor da mensagem
      msgCount++;                    // Contador do numero de mensagnes enviadas
      LoRa.endPacket();
    
      Serial.println(" Enviando os dados ao LoRa");
      string_dados_lora = "";
      msgCount++;   
      vTaskDelay(1000);// tem que adicionar portTICK_PERIOD_MS?
    }
  }
  xSemaphoreGive(xMutex);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
  

void setup()
{
  xMutex = xSemaphoreCreateMutex(); // cria o objeto do semáforo xMutex
  #ifdef SERIAL_DEBUG
    Serial.begin(115200);
  #endif

  //Inicializando as portas
  pinMode(PINO_BOTAO,INPUT);
  pinMode(PINO_BUZZER,OUTPUT);
  pinMode(PINO_LED,OUTPUT);
  //iniciando recuperação
  pinMode(REC_MAIN, OUTPUT); //declara o pino do rec principal como output 
  pinMode(REC_DROGUE, OUTPUT); 
  digitalWrite(REC_MAIN, LOW); //inicializa em baixa 
  digitalWrite(REC_DROGUE, LOW);

  ledcAttachPin(PINO_BUZZER, 0);

  erro='\0';
  //Inicialização do SD
  #ifdef SERIAL_DEBUG
    Serial.println("Inicializando o cartão SD...");
  #endif
  SPIClass spi = SPIClass(HSPI);                  // cria a classe SPI para litar com a conexão entre o cartão SD e o ESP32
  spi.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN); // inicia a conexão spi
  while(!SD.begin(CS_PIN, spi, 80000000)) // verifica se o cartão sd foi encontrado através da conexão CS do SPI
  {
    erro = ERRO_SD;
    #ifdef SERIAL_DEBUG
      Serial.println("Falha ao iniciar o cartão SD. Verifique as conexões.");
    #endif
  }
  #ifdef SERIAL_DEBUG
    Serial.println("Cartão SD encontrado.");
  #endif

  //Inicialização do LoRa
  LoRa.setPins(csPin, resetPin, irqPin);
  while(!LoRa.begin(BAND)) //FREQUENCIA DO LORA 
  {
    erro = ERRO_LORA;
    #ifdef SERIAL_DEBUG
      Serial.println("Falha ao iniciar o módulo LoRa. Verifique as conexões.");
    #endif
  }

  //Inicialiazação do MPU6050
  while(!mpu.begin())
  {
    erro = ERRO_ACEL;
    #ifdef SERIAL_DEBUG
      Serial.println("Falha ao iniciar o MPU6050. Verifique as conexões.");
    #endif
  }
  #ifdef SERIAL_DEBUG
    Serial.println("MPU encontrado.");
  #endif
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  //Inicialização do BMP280
  while(!bmp.begin(0x76)) {
    erro = ERRO_BMP;
    #ifdef SERIAL_DEBUG
      Serial.println("Falha ao iniciar o BMP280. Verifique as conexões.");
    #endif
  }
  #ifdef SERIAL_DEBUG
    Serial.println("BMP encontrado.");
  #endif
  //PESQUISAR SOBRE MODOS E OPÇÕES DO BMP
  //Verificar necessidade do setSampling
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  
  //Inicialização do GPS
  gpsSerial.begin(9600);

  if(!erro){
    #ifdef SERIAL_DEBUG
      Serial.println("Todos os sensores foram inicializados com sucesso.");
    #endif
    statusAtual = ESTADO_ESPERA;
    ledcAttachPin(PINO_BUZZER, 1);
    vTaskDelay(1000/ portTICK_PERIOD_MS);
    ledcAttachPin(PINO_BUZZER, 0);
  }
  else{
    #ifdef SERIAL_DEBUG
      Serial.print("Altímetro com erro de inicialização código:");
      Serial.println(erro);
    #endif
    statusAtual = erro;
  }

  xTaskCreate(task_i2c_sensores, "task bmp", 3000, NULL, 1, NULL);         // cria a task que trata os dados
  xTaskCreate(task_gps, "task gps", 3000, NULL, 1, NULL);         // cria a task que salva no cartão SD
  xTaskCreate(task_gravaSD, "task sd", 3000, NULL, 1, NULL);      // cria a task que salva no cartão SD
  xTaskCreate(task_envia_lora, "task lora", 3000, NULL, 1, NULL); // cria a task que envia os dados pelo LoRa

}

void loop()
{
  vTaskDelay(100/ portTICK_PERIOD_MS);
}
