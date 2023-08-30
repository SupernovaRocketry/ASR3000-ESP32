// Inclusão das bibliotecas utilizadas
#include <iostream>
#include <Arduino.h>         // Permite o uso da interface Arduino IDE para programar o ESP32
#include "SD.h"              // Biblioteca que proporciona o manuseamento do cartão SD
#include "FS.h"              // Biblioteca que auxilia no manuseamento do cartão SD
#include <SPI.h>             // Biblioteca que realiza a conexão SPI
#include <Adafruit_BMP280.h> // Biblioteca do sensor BMP280
#include <Wire.h>            // Comunicação I2C
#include "freertos/task.h"   // Biblioteca que proporciona a criação e manuseamento das Tasks
#include "freertos/queue.h"  // Biblioteca que proporciona a criação e manuseamento das filas
#include <TinyGPS++.h>       // Biblioteca do GPS
#include <SoftwareSerial.h>  // Biblioteca utilizada pelo GPS
#include <LoRa.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h> // Biblioteca que traduz os dados para Json

// headers
#include <main.h>
#include <defs.h>

// A FAZER
// ACIONAMENTO
// DEFINIR MÉTODO
// DEFINIR PARÂMETROS DE ACIONAMENTO
// NA INICIALIZAÇÃO, CALIBRAR SENSORES E DETERMINAR ALTITUDE INICIAL
// Determinar questão do acionamento redundante
// ver se JsonObject Acelerometro = doc.createNestedObject("Acelerometro"); é necessário rodar toda vez ou apenas no início do código

// Filas para comunicação dos dados entre as tasks
QueueHandle_t SDdataQueue;
QueueHandle_t LORAdataQueue;

// Variáveis globais dos dados do sensor BMP280
Adafruit_BMP280 bmp;
float altitude_atual;
float altitude_anterior = 0;
float tempo_anterior = 0;
float tempo_vel = 0;
float velocidade_atual = 0;

// Variáveis do sensor MPU6050
int MPU = 0x68;
Adafruit_MPU6050 mpu;

// variáveis do GPS
// Pinos do GPS. Estão em INT pois SoftwareSerial gpsSerial(RXPin, TXPin) só aceita INT como parâmetro, #define não funcionou
int RXPin = 16;
int TXPin = 17;
int GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

// Variaveis do lora
String string_dados_lora = "";

// Variaveis do sd
File arquivoLog;
// Variáveis de dados
float alturaInicial = 0;
float alturaMinima = 0;
float alturaMaxima = 0;

// variáveis de controle
int n = 0;
bool abriuParaquedasMain = false;
bool abriuParaquedasDrogue = false;
bool abriuRedundanciaMain = false;
bool abriuRedundanciaDrogue = false;
char erro = false;
char statusAtual = '\0';
bool estado;
bool descendo = false;
bool subindo = false;
char nomeConcat[16]; // nome do arquivo

// string que recebe cada linha nova de dados que vai ser gravada no arquivo .txt no cartão SD
String string_dados_sd;
SPIClass spi = SPIClass(HSPI); // cria a classe SPI para lidar com a conexão entre o cartão SD e o ESP32

SemaphoreHandle_t xMutex; // objeto do semáforo das tasks

StaticJsonDocument<512> doc; // objeto json que recebe os dados para serem gravados no Cartão SD e no LoRa, usando 384 bytes (pode ser ajustado)

void aquisicaoDados(void *pvParameters)
{
  float pressao_atual, temperatura_atual;
  float AcX_atual, AcY_atual, AcZ_atual, Tmp, GyX_atual, GyY_atual, GyZ_atual;
  String data_line;
  uint32_t tempo_atual = 0;
  float latitude_atual = 0;
  float longitude_atual = 0;

  while (1)
  {
// BMP
#ifdef SERIAL_DEBUG
    // Serial.println("Task dos sensores BMP e MPU iniciada.");
#endif
    pressao_atual = bmp.readPressure();
    temperatura_atual = bmp.readTemperature();
    altitude_atual = bmp.readAltitude();
    tempo_vel = millis();                                                                          // em segundos
    velocidade_atual = (altitude_atual - altitude_anterior) * 1000 / (tempo_vel - tempo_anterior); // em metros por segundo
    altitude_anterior = altitude_atual;
    if (tempo_vel > tempo_anterior + 1000)
    {
      tempo_anterior = tempo_vel;
    }

#ifdef SERIAL_DEBUG
    data_line = "Altitude atual: " + String(altitude_atual) + "| Temperatura: " + String(temperatura_atual) + " | Pressão: " + String(pressao_atual / 1013.25) + " | Velocidade: " + String(velocidade_atual);
    Serial.println(data_line);
#endif

    // MPU
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    AcX_atual = a.acceleration.x;
    AcY_atual = a.acceleration.y;
    AcZ_atual = a.acceleration.z;
    GyX_atual = g.gyro.x;
    GyY_atual = g.gyro.y;
    GyZ_atual = g.gyro.z;
#ifdef SERIAL_DEBUG
    data_line = "AcX: " + String(AcX_atual) + "| AcY: " + String(AcY_atual) + " | AcZ: " + String(AcZ_atual) + " | GyX: " + String(GyX_atual) + " | GyY: " + String(GyY_atual) + " | GyZ: " + String(GyZ_atual);
    Serial.println(data_line);
#endif

    // GPS
    if (gpsSerial.available() > 0)
    {
      if (gps.encode(gpsSerial.read()))
      {
        if (gps.time.isValid())
        {
          tempo_atual = gps.time.value(); // fuso horário -3
        }
        else
        {
#ifdef SERIAL_DEBUG
          Serial.println("Tempo não detectado");
#endif
        }
        if (gps.location.isValid())
        {
          latitude_atual = gps.location.lat();
          longitude_atual = gps.location.lng();
#ifdef ACIONAMENTO_DEBUG
          Serial.println("Latitude:" + String(latitude_atual, 6) + " Longitude:" + String(longitude_atual, 6));
#endif
        }
        else
        {
#ifdef SERIAL_DEBUG
          Serial.println("Latitude e longitude não detectados");
#endif
        }
#ifdef SERIAL_DEBUG
        Serial.println(tempo_atual);
        Serial.println(latitude_atual);
        Serial.println(longitude_atual);
#endif
      }
      if (gps.charsProcessed() < 10)
      {
#ifdef SERIAL_DEBUG
        Serial.println("Sinal GPS não detectado");
#endif
      }
    }

    doc["Alt"] = altitude_atual;
    doc["Vel"] = velocidade_atual;
    doc["Lat"] = latitude_atual;
    doc["Long"] = longitude_atual;
    if (statusAtual == ESTADO_RECUPERANDO)
    {
      doc["PPE"] = true;
    }
    else
    {
      doc["PPE"] = false;
    }
    if (statusAtual == ESTADO_RECUPERAMAIN)
    {
      doc["PPP"] = true;
    }
    else
    {
      doc["PPP"] = false;
    }
    // duas casas decimais
    doc["Acel(x)"] = roundf(AcX_atual * 100) / 100.0;
    doc["Acel(y)"] = roundf(AcY_atual * 100) / 100.0;
    doc["Acel(z)"] = roundf(AcZ_atual * 100) / 100.0;
    doc["Gyro(x)"] = roundf(GyX_atual * 57.296 * 100) / 100.0; // conversão de rad/s para graus/s
    doc["Gyro(y)"] = roundf(GyY_atual * 57.296 * 100) / 100.0;
    doc["Gyro(z)"] = roundf(GyZ_atual * 57.296 * 100) / 100.0;
    doc["Time"] = tempo_atual;
    doc["Tmillis"] = tempo_vel;

#ifdef LORA_DEBUG
    Serial.println("Altitude enviada: " + String(altitude_atual));
#endif
    if (uxQueueSpacesAvailable(SDdataQueue) != 0)
    {
      xQueueSend(SDdataQueue, &doc, 0);
    }
    if (uxQueueSpacesAvailable(LORAdataQueue) != 0)
    {
      xQueueSend(LORAdataQueue, &doc, 0);
    }
  }
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

void task_gravaSD(void *pvParameters) // task do cartão SD
{
  uint32_t contador_sd = 0;
  String data_lineSD;
  while (1)
  {

    if (uxQueueMessagesWaiting(SDdataQueue) > SD_MAX && (statusAtual == ESTADO_GRAVANDO || statusAtual == ESTADO_RECUPERANDO || statusAtual == ESTADO_RECUPERAMAIN))
    {

      while (contador_sd < SD_MAX)
      {
        xQueueReceive(SDdataQueue, &doc, 0);
        // data_lineSD += string_dados_sd;
        // data_lineSD += "\n";
        doc["altminima"] = alturaMinima;
        doc["altmaxima"] = alturaMaxima;
        arquivoLog = SD.open(nomeConcat, FILE_APPEND);
        // arquivoLog.println(data_lineSD); //é possível manter um arquivoLog.println() em BRANCO para pular linha a cada gravação
        serializeJson(doc, arquivoLog); // funciona como um arquivoLog.print(doc), NÃO PULA LINHA IGUAL .println, ver serializeJsonPretty()
#ifdef SERIAL_DEBUG
        Serial.println("Dados gravados no cartão SD:");
        // Serial.println(data_lineSD);
        serializeJson(doc, Serial); // funciona como um Serial.print(doc)
#endif
        arquivoLog.close();
        contador_sd++;
      }
      arquivoLog = SD.open(nomeConcat, FILE_APPEND);
      // arquivoLog.println(data_lineSD); //é possível manter um arquivoLog.println() em BRANCO para pular linha a cada gravação
      serializeJson(doc, arquivoLog); // funciona como um arquivoLog.print(doc), NÃO PULA LINHA IGUAL .println, ver serializeJsonPretty()
      arquivoLog.print("Status Atual:");
      arquivoLog.println(statusAtual);
#ifdef SERIAL_DEBUG
      Serial.println("Dados gravados no cartão SD:");
      // Serial.println(data_lineSD);
      serializeJson(doc, Serial); // funciona como um Serial.print(doc)
#endif
      arquivoLog.close();
      contador_sd = 0;
    }
  }
  vTaskDelay(500 / portTICK_PERIOD_MS);
}

void task_envia_lora(void *pvParameters) //
{
  byte localAddress = 0xBB; // Endereco deste dispositivo LoRa
  byte msgCount = 0;        // Contador de mensagens enviadas
  byte destination = 0xFF;  // Endereco do dispositivo para enviar a mensagem (0xFF envia para todos devices )
  while (1)
  {
    if (uxQueueMessagesWaiting(LORAdataQueue) > LORA_MAX)
    {

      xQueueReceive(LORAdataQueue, &doc, 0);
      doc["altminima"] = alturaMinima;
      doc["altmaxima"] = alturaMaxima;
      LoRa.beginPacket();
      serializeJson(doc, LoRa); // funciona como um LoRa.print(doc)
      LoRa.endPacket();

#ifdef LORA_DEBUG
      Serial.println("Dados enviados pelo LoRa:");
      // Serial.println(string_dados_lora);
      serializeJson(doc, Serial);
      Serial.println(millis());
#endif
#ifdef SUPERVISORIO_DEBUG
      serializeJson(doc, Serial); // funciona como um Serial.print(doc)
      Serial.println();
#endif
    }
  }
  vTaskDelay(100 / portTICK_PERIOD_MS);
}
void checaCondicoes(void *pvParameters)
{
  if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
  {
    while (1)
    {
      {
        if (statusAtual == ESTADO_ESPERA)
        {
          if (digitalRead(RBF) == LOW)
          {
            alturaMinima = altitude_atual; // altura mínima registrada no momento de retirada do RBF
            statusAtual = ESTADO_GRAVANDO;
           
#ifdef ACIONAMENTO_DEBUG
            Serial.println("Remove before flight retirado!");
#endif
          }
        }
        if (statusAtual == ESTADO_GRAVANDO || statusAtual == ESTADO_RECUPERANDO || statusAtual == ESTADO_RECUPERAMAIN)
        {
          // alturaMinima
          if ((altitude_atual < alturaMinima))
          {
            alturaMinima = altitude_atual;
          }
          // alturaMaxima
          //Serial.println(statusAtual);
          if (!subindo && statusAtual == ESTADO_GRAVANDO)
          {
            alturaMaxima = 0;
          }
#ifdef ACIONAMENTO_DEBUG
          Serial.print("atitude atual:");
          Serial.println(altitude_atual);
          Serial.print("altura minima:");
          Serial.println(alturaMinima);
#endif
          // controle de subida
          if ((altitude_atual > alturaMinima + THRESHOLD_SUBIDA) && !subindo)
          {
            subindo = true; // Saiu da base e está subindo
#ifdef ACIONAMENTO_DEBUG
            Serial.print("atitude atual:");
            Serial.println(altitude_atual);
            Serial.print("altura minima:");
            Serial.println(alturaMinima);
#endif
          }

          // primeira referencia de altura maxima (REDUNDANTE)
          // if (subindo && (alturaMaxima == 0.0))
          //{
          //  alturaMaxima = altitude_atual;
          //}
          // verificar a altura máxima
          if ((altitude_atual > alturaMaxima) && subindo)
          {
            alturaMaxima = altitude_atual;
          }

// Controle de descida, usando um threshold para evitar disparos não
// intencionais

          if ((altitude_atual + THRESHOLD_DESCIDA < alturaMaxima) && subindo)
          {
            descendo = true;
            subindo = false;
            statusAtual = ESTADO_RECUPERANDO; // Ativar Drogue ,não sei se precisa mudar o status talvez para saber o exato momento de ativação
            // Teste usando led
            digitalWrite(REC_DROGUE, HIGH);
            digitalWrite(PINO_LED, HIGH);
          

          }
          if (altitude_atual < (ALTURA_MAIN + alturaMinima) && descendo)
          {
            statusAtual = ESTADO_RECUPERAMAIN; // Ativar Main
            digitalWrite(REC_MAIN, HIGH);
            digitalWrite(PINO_BUZZER, HIGH);

          }
        }
      }
      vTaskDelay(100 / portTICK_PERIOD_MS); // igual ou maior que o tempo que demora pra rodar a task de aquisição
    }
    xSemaphoreGive(xMutex);
  }
}
void setup()
{
  xMutex = xSemaphoreCreateMutex(); // cria o objeto do semáforo xMutex
  SDdataQueue = xQueueCreate(SD_QUEUE_LENGTH, sizeof(String));
  LORAdataQueue = xQueueCreate(LORA_QUEUE_LENGTH, sizeof(String));
#ifdef LORA_DEBUG
  Serial.begin(115200);
#endif
#ifdef SERIAL_DEBUG
  Serial.begin(115200);
#endif
#ifdef ACIONAMENTO_DEBUG
  Serial.begin(115200);
#endif
#ifdef SUPERVISORIO_DEBUG
  Serial.begin(115200);
#endif

  // Inicializando as portas
  pinMode(PINO_BOTAO, INPUT);
  pinMode(PINO_BUZZER, OUTPUT);
  pinMode(PINO_LED, OUTPUT);
  pinMode(RBF, INPUT_PULLUP);

  // iniciando recuperação
  pinMode(REC_MAIN, OUTPUT); // declara o pino do rec principal como output
  pinMode(REC_DROGUE, OUTPUT);
  digitalWrite(REC_MAIN, LOW); // inicializa em baixa
  digitalWrite(REC_DROGUE, LOW);

  digitalWrite(PINO_BUZZER, LOW);
  digitalWrite(PINO_LED, LOW);
  erro = '\0';
// Inicialização do SD
#ifdef SERIAL_DEBUG
  Serial.println("Inicializando o cartão SD...");
#endif
  spi.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN); // inicia a conexão spi
#ifdef SERIAL_DEBUG
  Serial.println("Conexão SPI iniciada.");
#endif
  if (!SD.begin(CS_PIN, spi)) // verifica se o cartão sd foi encontrado através da conexão CS do SPI
  {
    erro = ERRO_SD;
#ifdef SERIAL_DEBUG
    Serial.println("Falha ao iniciar o cartão SD. Verifique as conexões.");
#endif
  }
#ifdef SERIAL_DEBUG
  Serial.println("Cartão SD encontrado.");
#endif

  // Inicialização do LoRa
  LoRa.setPins(csPin, resetPin, irqPin);
  while (!LoRa.begin(BAND)) // FREQUENCIA DO LORA
  {
    erro = ERRO_LORA;
#ifdef SERIAL_DEBUG
    Serial.println("Falha ao iniciar o módulo LoRa. Verifique as conexões.");
#endif
  }

  // Inicialiazação do MPU6050
  while (!mpu.begin())
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
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Inicialização do BMP280
  while (!bmp.begin(0x76))
  {
    erro = ERRO_BMP;
#ifdef SERIAL_DEBUG
    Serial.println("Falha ao iniciar o BMP280. Verifique as conexões.");
#endif
  }
#ifdef SERIAL_DEBUG
  Serial.println("BMP encontrado.");
#endif
  // PESQUISAR SOBRE MODOS E OPÇÕES DO BMP
  // Verificar necessidade do setSampling
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,    /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,    /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,   /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,     /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_63); /* Standby time. */

  // Inicialização do GPS
  gpsSerial.begin(9600);

  if (!erro)
  {
#ifdef SERIAL_DEBUG
    Serial.println("Todos os sensores foram inicializados com sucesso.");
    Serial.println(statusAtual);
    Serial.println(erro);
#endif
    statusAtual = ESTADO_ESPERA;
    digitalWrite(PINO_BUZZER, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
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
    arquivoLog.close();
  }
  else
  {
#ifdef SERIAL_DEBUG
    Serial.print("Altímetro com erro de inicialização código:");
    Serial.println(erro);
#endif
  }
if(RBF == HIGH){
  xTaskCreatePinnedToCore(aquisicaoDados, "task aquisicaoDados", 3000, NULL, 1, NULL, 0); // cria a task que trata os dados
  xTaskCreatePinnedToCore(checaCondicoes, "task checaCondicoes", 3000, NULL, 0, NULL, 0); // cria a task que checa as condições de voo
  xTaskCreatePinnedToCore(task_gravaSD, "task sd", 3000, NULL, 1, NULL, 1);               // cria a task que salva no cartão SD
  xTaskCreatePinnedToCore(task_envia_lora, "task lora", 3000, NULL, 1, NULL, 1);          // cria a task que envia os dados pelo LoRa
  vTaskStartScheduler();
}
}

void loop()
{
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
