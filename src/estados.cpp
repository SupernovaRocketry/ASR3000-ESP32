#include <Arduino.h>
#include <Adafruit_BMP280.h> 
#include "Wire.h"
#include <SD.h> 
#include <defs.h>
#include <main.h>


extern double altitude_atual;
extern double alturaInicial;
extern double alturaMinima;
extern double alturaMaxima;
extern bool gravando;
extern bool abriuParaquedasMain;
extern bool abriuParaquedasDrogue;
extern bool abriuRedundanciaMain ;
extern bool abriuRedundanciaDrogue ;
extern char erro;
extern char  statusAtual;
extern bool estado;
extern bool descendo;
extern bool subindo;

void checaCondicoes() {
  // Funcao responsavel por checar condicoes e atualizar variaveis de extremos
  // (altura maxima, altura minima, etc)

  if ((statusAtual == ESTADO_GRAVANDO) && !gravando ) {
    alturaMinima = alturaAtual;
    gravando = true;

    #ifdef DEBUG_COND
      arquivoCond.print("a");
    #endif
  }

  //alturaMinima
  if ((alturaAtual < alturaMinima) && (statusAtual == ESTADO_GRAVANDO)){
    alturaMinima = alturaAtual;
    
    #ifdef DEBUG_COND
      arquivoCond.print("b");
    #endif
  }
 

  //alturaMaxima
  if (!subiu && (statusAtual == ESTADO_GRAVANDO)){
    alturaMaxima = 0;

    #ifdef DEBUG_COND
      arquivoCond.print("c");
    #endif
  }

  //controle de subida
  if ((alturaAtual > alturaMinima + THRESHOLD_SUBIDA) && (statusAtual == ESTADO_GRAVANDO) && !subiu ){
    subiu = true;

    #ifdef DEBUG_COND
      arquivoCond.print("d");
    #endif
  }

  //primeira referencia de altura maxima
  if (subiu && (alturaMaxima == 0) && (statusAtual == ESTADO_GRAVANDO)){
    alturaMaxima = alturaAtual;

    #ifdef DEBUG_COND
      arquivoCond.print("e");
    #endif
  }

  //verificar a altura máxima
  if ((alturaAtual > alturaMaxima) && (statusAtual == ESTADO_GRAVANDO) && subiu){
    alturaMaxima =  alturaAtual;

    #ifdef DEBUG_COND
      arquivoCond.print("f");
    #endif
  }

  //Controle de descida, usando um threshold para evitar disparos não
  //intencionais
  if ((alturaAtual + THRESHOLD_DESCIDA < alturaMaxima) && (statusAtual == ESTADO_GRAVANDO) && subiu) {
    descendo = true;
    subiu = false;
    statusAtual = ESTADO_RECUPERANDO;

    #ifdef DEBUG_COND
      arquivoCond.print("g");
    #endif
  }

  #ifdef DEBUG_COND
    arquivoCond.println("");
    arquivoCond.close();
  #endif

  #ifdef DEBUG
    Serial.println("Chequei as condições!");
  #endif
}
