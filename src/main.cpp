// Inclusão das bibliotecas utilizadas
#include <iostream>
#include <Arduino.h>           // Permite o uso da interface Arduino IDE para programar o ESP32
#include "SD.h"                // Biblioteca que proporciona o manuseamento do cartão SD
#include "FS.h"                // Biblioteca que auxilia no manuseamento do cartão SD
#include <SPI.h>               // Biblioteca que realiza a conexão SPI
#include <Adafruit_BMP280.h>   // Biblioteca do sensor BMP280
#include <Wire.h>              // Comunicação I2C
#include "freertos/task.h"     // Biblioteca que proporciona a criação e manuseamento das Tasks
#include "freertos/queue.h"    // Biblioteca que proporciona a criação e manuseamento das filas
#include <TinyGPS++.h>         // Biblioteca do GPS
#include <SoftwareSerial.h>    // Biblioteca utilizada pelo GPS
#include <LoRa.h>              
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>       // Biblioteca que traduz os dados para Json

// headers
#include <main.h>
#include <defs.h>

// A FAZER
//ACIONAMENTO
//DEFINIR MÉTODO
//DEFINIR PARÂMETROS DE ACIONAMENTO
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
float tempo_vel=0;
float velocidade_atual=0;


// Variáveis do sensor MPU6050
int MPU = 0x68;
Adafruit_MPU6050 mpu;

// variáveis do GPS
//Pinos do GPS. Estão em INT pois SoftwareSerial gpsSerial(RXPin, TXPin) só aceita INT como parâmetro, #define não funcionou
int RXPin = 16;
int TXPin = 17;
int GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

// Variaveis do lora
String string_dados_lora="";

// Variaveis do sd
File arquivoLog;
// Variáveis de dados
float alturaInicial = 0;
float alturaMinima;
float alturaMaxima =  0;

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

// string que recebe cada linha nova de dados que vai ser gravada no arquivo .txt no cartão SD
String string_dados_sd;
SPIClass spi = SPIClass(HSPI);                // cria a classe SPI para lidar com a conexão entre o cartão SD e o ESP32

SemaphoreHandle_t xMutex;  // objeto do semáforo das tasks

StaticJsonDocument<384> doc; // objeto json que recebe os dados para serem gravados no Cartão SD e no LoRa, usando 384 bytes (pode ser ajustado)

void aquisicaoDados(void *pvParameters)
{
  float pressao_atual,temperatura_atual;
  int AcX_atual, AcY_atual, AcZ_atual, Tmp, GyX_atual, GyY_atual, GyZ_atual;
  String data_line;
  uint32_t tempo_atual = 0;
  float latitude_atual = 0;
  float longitude_atual = 0;

  while(1)
  {
    // BMP
    #ifdef SERIAL_DEBUG
          //Serial.println("Task dos sensores BMP e MPU iniciada.");
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
    
    // GPS  
    if(gpsSerial.available() > 0)
    {
      if (gps.encode(gpsSerial.read()))
      {
          if(gps.time.isValid())
          {
            tempo_atual = gps.time.value(); //fuso horário -3
          }
          else
          {
            Serial.println("Tempo não detectado");
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
          #ifdef SERIAL_DEBUG
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
    string_dados_sd += tempo_atual;
    string_dados_sd += ",";
    string_dados_sd += latitude_atual;
    string_dados_sd += ",";
    string_dados_sd += longitude_atual;
    string_dados_lora=string_dados_sd;

    doc["Altitude"] = altitude_atual;
    doc["Latitude"] = latitude_atual;
    doc["Longitude"] = longitude_atual;
    if(statusAtual==ESTADO_RECUPERANDO){
      doc["Principal Paraquedas Estabilizador"] = true;
      doc["Redundancia Paraquedas Estabilizador"] = true;
      doc["Comercial Paraquedas Estabilizador"] = true;
    }
    else{
      doc["Principal Paraquedas Estabilizador"] = false;
      doc["Redundancia Paraquedas Estabilizador"] = false;
      doc["Comercial Paraquedas Estabilizador"] = false;
    }
    if (statusAtual==ESTADO_RECUPERAMAIN)
    {
      doc["Principal Paraquedas Principal"] = true;
      doc["Comercial Paraquedas Principal"] = true;
    }
    else{
      doc["Principal Paraquedas Principal"] = false;
      doc["Comercial Paraquedas Principal"] = false;
    }
    
    JsonObject Acelerometro = doc.createNestedObject("Acelerometro"); // cria uma chave dentro da key "Acelerometro" do json, com subchaves "x", "y" e "z"
    Acelerometro["x"] = AcX_atual;
    Acelerometro["y"] = AcY_atual;
    Acelerometro["z"] = AcZ_atual;
    
    JsonObject Giroscopio = doc.createNestedObject("Giroscopio"); // cria uma chave dentro da key "Giroscopio" do json, com subchaves "x", "y" e "z"
    Giroscopio["x"] = GyX_atual;
    Giroscopio["y"] = GyY_atual;
    Giroscopio["z"] = GyZ_atual;
    doc["RSSI"] = 0;
    
    if(uxQueueSpacesAvailable(SDdataQueue) != 0)
    {
      xQueueSend(SDdataQueue, &doc, portMAX_DELAY);
    }
    if(uxQueueSpacesAvailable(LORAdataQueue) != 0)
    {
       xQueueSend(LORAdataQueue, &doc, portMAX_DELAY);
    }
    xSemaphoreGive(xMutex);
  }
  vTaskDelay(500 / portTICK_PERIOD_MS);
}

 void task_gravaSD(void *pvParameters) // task do cartão SD
{
  uint32_t contador_sd= 0;
  String data_lineSD;
  while (1)
  {

    if(uxQueueMessagesWaiting(SDdataQueue) > SD_MAX)
    {
      
      while(contador_sd< SD_MAX){
        xQueueReceive(SDdataQueue, &doc, 0);
        //data_lineSD += string_dados_sd;
        //data_lineSD += "\n";
        arquivoLog = SD.open(nomeConcat, FILE_APPEND);
        //arquivoLog.println(data_lineSD); //é possível manter um arquivoLog.println() em BRANCO para pular linha a cada gravação
        serializeJson(doc, arquivoLog); // funciona como um arquivoLog.print(doc), NÃO PULA LINHA IGUAL .println, ver serializeJsonPretty()
          #ifdef SERIAL_DEBUG   
            Serial.println("Dados gravados no cartão SD:");
            //Serial.println(data_lineSD);
            serializeJson(doc, Serial); // funciona como um Serial.print(doc)
        #endif
        arquivoLog.close(); 
        contador_sd++;
      }
      arquivoLog = SD.open(nomeConcat, FILE_APPEND);
      //arquivoLog.println(data_lineSD); //é possível manter um arquivoLog.println() em BRANCO para pular linha a cada gravação
      serializeJson(doc, arquivoLog); // funciona como um arquivoLog.print(doc), NÃO PULA LINHA IGUAL .println, ver serializeJsonPretty()
        #ifdef SERIAL_DEBUG   
          Serial.println("Dados gravados no cartão SD:");
          //Serial.println(data_lineSD);
          serializeJson(doc, Serial); // funciona como um Serial.print(doc)
        #endif
      arquivoLog.close(); 
      contador_sd=0;
      
    }
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void task_envia_lora(void *pvParameters) //
{
  byte localAddress = 0xBB; // Endereco deste dispositivo LoRa
  byte msgCount = 0;        // Contador de mensagens enviadas
  byte destination = 0xFF;  // Endereco do dispositivo para enviar a mensagem (0xFF envia para todos devices )
  while (1)
  { 
    if(uxQueueMessagesWaiting(LORAdataQueue) >LORA_MAX)
    {
      
      xQueueReceive(LORAdataQueue, &string_dados_lora, 0);
      LoRa.beginPacket();
      LoRa.write(destination);       // Adiciona o endereco de destino
      LoRa.write(localAddress);
      LoRa.write(msgCount);  
      //LoRa.write(string_dados_lora.length()); // Tamanho da mensagem em bytes
      //LoRa.print(string_dados_lora);          // Vetor da mensagem
      LoRa.write(measureJson(doc)); // Tamanho da mensagem em bytes
      serializeJson(doc, LoRa); // funciona como um LoRa.print(doc)
      
      msgCount++;     // Contador do numero de mensagnes enviadas
      LoRa.endPacket();
      
      #ifdef SERIAL_DEBUG
        Serial.println("Dados enviados pelo LoRa:");
        //Serial.println(string_dados_lora);
        serializeJson(doc, Serial);
      #endif
    }   
  }
  
}
  
void checaCondicoes(void *pvParameters){
  while(1){                                                                       
    if (statusAtual == ESTADO_ESPERA){
      if (digitalRead(RBF) == LOW){
        statusAtual = ESTADO_GRAVANDO;
        #ifdef SERIAL_DEBUG
          Serial.println("Remove before flight retirado!");
        #endif
      }
    }
    if (statusAtual== ESTADO_GRAVANDO){
        if (!gravando) {
            alturaMinima = altitude_atual; // altura mínima registrada no momento de retirada do RBF
            gravando = true;
        }
        //alturaMinima
        if ((altitude_atual < alturaMinima)){
            alturaMinima = altitude_atual;
        }

        //alturaMaxima
        if (!subindo){
            alturaMaxima = 0;
        }

        //controle de subida
        if ((altitude_atual > alturaMinima + THRESHOLD_SUBIDA) && !subindo ){
            subindo = true; // Saiu da base e está subindo
        }

        //primeira referencia de altura maxima
        if (subindo && (alturaMaxima == 0)){
            alturaMaxima = altitude_atual;
        }

        //verificar a altura máxima
        if ((altitude_atual > alturaMaxima) && subindo){
            alturaMaxima =  altitude_atual;
        }

        //Controle de descida, usando um threshold para evitar disparos não
        //intencionais
        if ((altitude_atual + THRESHOLD_DESCIDA < alturaMaxima) && subindo) {
            descendo = true;
            subindo = false;
            statusAtual = ESTADO_RECUPERANDO; // Ativar Drogue ,não sei se precisa mudar o status talvez para saber o exato momento de ativação
            //Teste usando led
            digitalWrite(PINO_LED, HIGH);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            digitalWrite(PINO_LED, LOW);
        }

        if(altitude_atual + THRESHOLD_DESCIDA < (ALTURA_MAIN + alturaMinima) && descendo){
            statusAtual = ESTADO_RECUPERAMAIN; // Ativar Main
        }
    }
    Serial.println("chequei");
  }
  
  vTaskDelay(1500 / portTICK_PERIOD_MS); // igual ou maior que o tempo que demora pra rodar a task de aquisição
}

// void verificaInicio(void *pvParameters){
//   while(1){ 
    
//   }
//   vTaskDelay(1000 / portTICK_PERIOD_MS);
// }
void setup()
{
  xMutex = xSemaphoreCreateMutex(); // cria o objeto do semáforo xMutex
  SDdataQueue = xQueueCreate(SD_QUEUE_LENGTH,sizeof(String));
  LORAdataQueue = xQueueCreate(LORA_QUEUE_LENGTH,sizeof(String));


  #ifdef SERIAL_DEBUG
    Serial.begin(115200);
  #endif

  //Inicializando as portas
  pinMode(PINO_BOTAO,INPUT);
  pinMode(PINO_BUZZER,OUTPUT);
  pinMode(PINO_LED,OUTPUT);
  pinMode(RBF,INPUT_PULLUP);

  
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
  
  spi.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN); // inicia a conexão spi
  #ifdef SERIAL_DEBUG
    Serial.println("Conexão SPI iniciada.");
  #endif
  if(!SD.begin(CS_PIN,spi))// verifica se o cartão sd foi encontrado através da conexão CS do SPI
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

  //xQueueReset(SDdataQueue);
  //xQueueReset(LORAdataQueue);

  xTaskCreatePinnedToCore(aquisicaoDados, "task aquisicaoDados", 3000, NULL, 1, NULL, 0);         // cria a task que trata os dados
  xTaskCreatePinnedToCore(checaCondicoes, "task checaCondicoes", 3000, NULL, 0, NULL, 0);      // cria a task que checa as condições de voo
  // xTaskCreatePinnedToCore(verificaInicio, "task verificaInicio", 3000, NULL, 1, NULL, 0);      // cria a task que verifica o início do voo

  xTaskCreatePinnedToCore(task_gravaSD, "task sd", 3000, NULL, 1, NULL, 1);      // cria a task que salva no cartão SD
  xTaskCreatePinnedToCore(task_envia_lora, "task lora", 3000, NULL, 1, NULL, 1); // cria a task que envia os dados pelo LoRa
  
  vTaskStartScheduler();
}

void loop()
{
  vTaskDelay(1000/portTICK_PERIOD_MS);
}
