//Bibliotecas usadas
#include <Arduino.h> // permite o C++ compilar códigos para arduino/esp32
#include "SD.h" // biblioteca que proporciona o manuseamento do cartão SD
#include "FS.h" // biblioteca que auxilia no manuseamento do cartão SD
#include <SPI.h> // biblioteca que realiza a conexão SPI
#include <Adafruit_BMP280.h> // biblioteca do sensor BMP280
#include <Wire.h> // Na vdd eu n sei mto pq que eu usei isso, deve ser dependência do SPI ou do ADAFRUIT

// variáveis dos dados do sensor BMP280
extern double pressao_atual;
extern double altitude_atual;
extern double temperatura_atual;

//contadores 
extern int contador = 0;
extern int contador2 = 0;

// string que recebe cada linha nova de dados que vai ser gravada no arquivo .txt no cartão SD
extern String string_dados;

// listas dos valores de pressão, temperatura e altitude (e seus respectivos tamanhos)
extern int pressure_values[100] = {};
extern int temperature_values[100] = {};
extern int altitude_values[100] = {};
extern int pressure_values_size = sizeof(pressure_values) / sizeof(pressure_values[0]);
extern int temperature_values_size = sizeof(temperature_values) / sizeof(temperature_values[0]);
extern int altitude_values_size = sizeof(altitude_values) / sizeof(altitude_values[0]);

// função que recolhe os dados do sensor e recebe como parâmetro o objeto da classe Adafruit_BMP280
void sensor_bmp(Adafruit_BMP280& sensor)
{
    pressao_atual = sensor.readPressure();
    temperatura_atual = sensor.readTemperature();
    altitude_atual = sensor.readAltitude();
}

// função que grava os arquivos no cartão SD quando chamada
void sd_manage(fs :: FS &fs , const char * path)
{
contador ++;
  File file = fs.open(path, FILE_APPEND);

  if(file)
  {
    for(int i = 0; i < pressure_values_size; i++)
    {
      string_dados = "";
      string_dados += millis();
      string_dados += ",";
      string_dados += altitude_values[i];
      string_dados += ",";
      string_dados += temperature_values[i];
      string_dados += ",";
      string_dados += pressure_values[i];
      
      file.println(string_dados);
    }
    file.close();
    Serial.println("Arquivo Atualizado");
    memset(pressure_values, 0, sizeof(pressure_values));
    memset(temperature_values, 0, sizeof(temperature_values));
    memset(pressure_values, 0, sizeof(altitude_values));
  }
  else
  {
    Serial.println("Falha ao acessar o arquivo.");
  }
}