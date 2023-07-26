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
void verificaInicio(){
    // Relacionado ao Remove Before Flight 
    // Mudar estado de espera para gravando
    // statusAtual= ESTADO_GRAVANDO;
    #ifdef DEBUG
      Serial.println("Verifiquei o remove before flight");
    #endif
}
void checaCondicoes() {
  // Funcao responsavel por checar condicoes e atualizar variaveis de extremos
  // (altura maxima, altura minima, etc)
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
        if ((altitude_atual > alturaMaxima) && subiu){
            alturaMaxima =  altitude_atual;
        }

        //Controle de descida, usando um threshold para evitar disparos não
        //intencionais
        if ((altitude_atual + THRESHOLD_DESCIDA < alturaMaxima) && subindo) {
            descendo = true;
            subindo = false;
            statusAtual = ESTADO_RECUPERANDO; // Ativar Drogue 
        }

        if(altitude_atual + THRESHOLD_DESCIDA < ALTURA_MAIN && descendo){
            statusAtual = ESTADO_RECUPERAMAIN; // Ativar Main
        }
    }
    #ifdef SERIAL_DEBUG
        Serial.println("Chequei as condições!");
    #endif
}
void finaliza(){

}
