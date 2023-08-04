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
#include "freertos/queue.h"    // biblioteca que proporciona a criação e manuseamento das filas
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


QueueHandle_t SDdataQueue;
QueueHandle_t LORAdataQueue;
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
String string_dados_lora="";

//variaveis do sd
File arquivoLog;
//Variáveis de dados
double alturaInicial = 0;
double alturaMinima;
double alturaMaxima =  0;

// variáveis de controle
unsigned long millisRecMain = 2000000;
unsigned long millisRecDrogue = 1000000;
int n = 0;
bool gravando = false;
bool abriuParaquedasMain = false;
bool abriuParaquedasDrogue = false;
bool abriuRedundanciaMain = false;
bool abriuRedundanciaDrogue = false;
char erro = false;
char  statusAtual='\0';
bool estado;
bool descendo = false;
bool subindo = false;
char nomeConcat[16]; //nome do arquivo
// contadores

// mover contadores pra antes do while (1) da respectiva task
uint32_t contador = 0;
uint32_t contador_sd= 0;
uint32_t contador_lora= 0;

// string que recebe cada linha nova de dados que vai ser gravada no arquivo .txt no cartão SD
String string_dados_sd;

// listas dos valores de hora, data, latitude e longitude do GPS
uint32_t data_value;
uint32_t tempo_value;
double latitude_value;
double longitude_value;

SemaphoreHandle_t xMutex; // objeto do semáforo das tasks

void aquisicaoDados(void *pvParameters)
{
  String data_line;
  while(1)
  {
  if(xSemaphoreTake(xMutex, portMAX_DELAY)==pdTRUE)
  {
    // BMP
    #ifdef SERIAL_DEBUG
          Serial.println("Task dos sensores BMP e MPU iniciada.");
    #endif
    pressao_atual = bmp.readPressure();
    temperatura_atual = bmp.readTemperature();
    altitude_atual = bmp.readAltitude();
    tempo_vel=millis(); // em segundos
    velocidade_atual = (altitude_atual - altitude_anterior)*1000/(tempo_vel - tempo_anterior); // em metros por segundo
    altitude_anterior = altitude_atual;
    if (tempo_vel>tempo_anterior+1000){
      tempo_anterior = tempo_vel;
    }

    #ifdef SERIAL_DEBUG
            data_line = "Altitude atual: " + String(altitude_atual) + "| Temperatura: " + String(temperatura_atual) + " | Pressão: " + String(pressao_atual / 1013.25)+ " | Velocidade: " + String(velocidade_atual);
            Serial.println(data_line);
    #endif

    // MPU
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    AcX_atual = a.gyro.x;
    AcY_atual = a.gyro.y;
    AcZ_atual = a.gyro.z;
    GyX_atual = g.gyro.x;
    GyY_atual = g.gyro.y;
    GyZ_atual = g.gyro.z;
    #ifdef SERIAL_DEBUG
            data_line = "AcX: " + String(AcX_atual) + "| AcY: " + String(AcY_atual) + " | AcZ: " + String(AcZ_atual) + " | GyX: " + String(GyX_atual) + " | GyY: " + String(GyY_atual) + " | GyZ: " + String(GyZ_atual);
            Serial.println(data_line);
    #endif
  /*
    // GPS  
    if(gpsSerial.available() > 0)
    {
      if (gps.encode(gpsSerial.read()))
      {
          if(gps.time.isValid()&&gps.date.isValid())
          {
            tempo_atual = gps.time.value();
            data_atual = gps.date.value();
          }
          else
          {
            Serial.println("Tempo e data não detectados");
          }
          if(gps.location.isValid())
          {
            latitude_atual = gps.location.lat();
            longitude_atual = gps.location.lng();

          }
          else
          {
            Serial.println("Latitude e longitude não detectados");
          }

          data_value = data_atual;
          tempo_value = tempo_atual;
          latitude_value = latitude_atual;
          longitude_value = longitude_atual;
          #ifdef SERIAL_DEBUG
                        Serial.println(data_atual);
                        Serial.println(tempo_atual);
                        Serial.println(latitude_atual);
                        Serial.println(longitude_atual);
          #endif
        }
      if (gps.charsProcessed() < 10)
      {
        Serial.println("Sinal GPS não detectado");
      }
    }
    */
    string_dados_sd = "";
    string_dados_sd += tempo_vel;
    string_dados_sd += ",";
    string_dados_sd += altitude_atual;
    string_dados_sd += ",";
    string_dados_sd += temperatura_atual;
    string_dados_sd += ",";
    string_dados_sd += pressao_atual;
    string_dados_sd += ",";
    string_dados_sd += AcX_atual;
    string_dados_sd += ",";
    string_dados_sd += AcY_atual;
    string_dados_sd += ",";
    string_dados_sd += AcZ_atual;
    string_dados_sd += ",";
    string_dados_sd += GyX_atual;
    string_dados_sd += ",";
    string_dados_sd += GyY_atual;
    string_dados_sd += ",";
    string_dados_sd += GyZ_atual;
    string_dados_sd += ",";
    string_dados_sd += data_value;
    string_dados_sd += ",";
    string_dados_sd += tempo_value;
    string_dados_sd += ",";
    string_dados_sd += latitude_value;
    string_dados_sd += ",";
    string_dados_sd += longitude_value;
    string_dados_lora=string_dados_sd;
    if(uxQueueSpacesAvailable(SDdataQueue) != 0)
    {
    xQueueSend(SDdataQueue, &string_dados_sd, portMAX_DELAY);
    }
    if(uxQueueSpacesAvailable(LORAdataQueue) != 0)
    {
    xQueueSend(LORAdataQueue, &string_dados_lora, portMAX_DELAY);
    }
    xSemaphoreGive(xMutex);
    }
  
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void task_gravaSD(void *pvParameters) // task do cartão SD
{
  while (1)
  {
      if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
      {
        Serial.println(uxQueueSpacesAvailable(SDdataQueue));
        // grava_SD(SD); //N PRECISA
        if(uxQueueSpacesAvailable(SDdataQueue) == 0)
        {
          
          while(contador_sd< QUEUE_LENGTH){
            xQueueReceive(SDdataQueue, &string_dados_sd, portMAX_DELAY);
            
            // arquivoLog = SD.open(nomeConcat, FILE_APPEND);
            // arquivoLog.println(string_dados_sd);
            #ifdef SERIAL_DEBUG
              Serial.println("Dados gravados no cartão SD:");
              Serial.println(string_dados_sd);
            #endif
            // arquivoLog.close(); 
            contador_sd++;
            string_dados_lora=string_dados_sd;
          }
          contador_sd=0;
        }
         //VER SOBRE COLOCAR DENTRO DOS IFS OU FORA NAS OUTRAS TASKS
         xSemaphoreGive(xMutex);
      }
      
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void task_envia_lora(void *pvParameters) //
{
  while (1)
  {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {   
      if(uxQueueSpacesAvailable(LORAdataQueue) == 0)
      {
        while(contador_lora< QUEUE_LENGTH){
          xQueueReceive(LORAdataQueue, &string_dados_lora, portMAX_DELAY);
          LoRa.beginPacket();
          LoRa.write(destination);       // Adiciona o endereco de destino
          LoRa.write(localAddress);
          LoRa.write(msgCount);  
          LoRa.write(string_dados_lora.length()); // Tamanho da mensagem em bytes
          LoRa.print(string_dados_lora);          // Vetor da mensagem
          msgCount++;     // Contador do numero de mensagnes enviadas
          LoRa.endPacket();
          #ifdef SERIAL_DEBUG
            Serial.println("Dados enviados pelo LoRa:");
            Serial.println(string_dados_lora);
          #endif
          string_dados_lora = "";
          msgCount++; 
          contador_lora++;
        }
        contador_lora=0;
      }      
      xSemaphoreGive(xMutex);
    }
    
  }
  
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
  

void setup()
{
  xMutex = xSemaphoreCreateMutex(); // cria o objeto do semáforo xMutex
  SDdataQueue = xQueueCreate(QUEUE_LENGTH,sizeof(String));
  LORAdataQueue = xQueueCreate(QUEUE_LENGTH,sizeof(String));


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

  digitalWrite(PINO_BUZZER, 0);

  erro='\0';
  //Inicialização do SD
  #ifdef SERIAL_DEBUG
    Serial.println("Inicializando o cartão SD...");
  #endif
  SPIClass spi = SPIClass(HSPI);                // cria a classe SPI para litar com a conexão entre o cartão SD e o ESP32
  spi.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN); // inicia a conexão spi
  #ifdef SERIAL_DEBUG
    Serial.println("Conexão SPI iniciada.");
  #endif
  while(!SD.begin(CS_PIN,spi))// verifica se o cartão sd foi encontrado através da conexão CS do SPI
  {
    return;
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
      Serial.println(statusAtual);
      Serial.println(erro); 
    #endif
    statusAtual = ESTADO_ESPERA;
    digitalWrite(PINO_BUZZER, 1);
    vTaskDelay(1000/ portTICK_PERIOD_MS);
    digitalWrite(PINO_BUZZER, 0);

    int n = 1;
    bool parar = false;
    while (!parar)
    {
      #ifdef DEBUG_TEMP
            Serial.println("não deveria estar aqui com o sd ligado");
      #endif
      sprintf(nomeConcat, "/log%d.txt", n);
      if (SD.exists(nomeConcat))
        n++;
      else
        parar = true;
    }

    arquivoLog = SD.open(nomeConcat, FILE_WRITE);
    arquivoLog.println("tempo;altitude;temperatura;pressão;acelx;acely;acelz;girosX;girosY;girosZ;data;tempogps;lat;long");
    arquivoLog.close();
  }
  else{
    #ifdef SERIAL_DEBUG
      Serial.print("Altímetro com erro de inicialização código:");
      Serial.println(erro);
    #endif
  }

  xQueueReset(SDdataQueue);
  xQueueReset(LORAdataQueue);
  xTaskCreatePinnedToCore(aquisicaoDados, "task aquicaoDados", 3000, NULL, 1, NULL, 0);         // cria a task que trata os dados
  xTaskCreatePinnedToCore(task_gravaSD, "task sd", 3000, NULL, 1, NULL, 1);      // cria a task que salva no cartão SD
  xTaskCreatePinnedToCore(task_envia_lora, "task lora", 3000, NULL, 1, NULL, 1); // cria a task que envia os dados pelo LoRa

  vTaskStartScheduler();
}

void loop()
{
  vTaskDelay(100/ portTICK_PERIOD_MS);
}
