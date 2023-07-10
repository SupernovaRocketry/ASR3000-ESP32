#ifndef DADOS_H
#define DADOS_H

void sensor_bmp(Adafruit_BMP280& sensor);

void sensor_mpu();

void sensor_GPS();

void grava_SD(fs :: FS &fs , const char * path);

void envia_LoRa();


#endif