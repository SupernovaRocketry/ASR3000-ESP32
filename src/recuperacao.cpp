#include <Arduino.h>
#include <defs.h>


extern bool abriuParaquedasDrogue;
extern bool abriuParaquedasMain;
extern bool descendo;
extern bool abriuRedundanciaDrogue;
extern bool abriuRedundanciaMain;
extern unsigned long millisRecMain;
extern unsigned long millisRecDrogue;
extern char statusAtual;

    // hw_timer_t * timer = NULL;
    // void IRAM_ATTR redundanciaDrogue(){
    //     digitalWrite(REC_DROGUE, HIGH);
    //     abriuParaquedasDrogue = 1;
    // }
    // void IRAM_ATTR redundanciaMain(){
    //     digitalWrite(REC_MAIN, HIGH);
    //     abriuRedundanciaMain = 1;
    // } precisa da redundancia? 

void abreParaquedasMain(){
    #ifdef DEBUG
        Serial.println("Abrindo paraquedas principal");
    #endif
    digitalWrite(REC_MAIN, HIGH);
    millisRecMain = millis();
    abriuParaquedasMain = 1;
}
void abreParaquedasDrogue(){
    #ifdef DEBUG
        Serial.println("Abrindo paraquedas drogue");
    #endif
    digitalWrite(REC_DROGUE, HIGH);
    millisRecDrogue = millis();
    abriuParaquedasDrogue = 1;
}
void recupera(){
    if(descendo && !abriuParaquedasDrogue && statusAtual == ESTADO_RECUPERANDO){
        abreParaquedasDrogue();
    }
    if(descendo && !abreParaquedasMain && statusAtual == ESTADO_RECUPERAMAIN){
        abreParaquedasMain();
        ledcAttachPin(PINO_BUZZER, 1);
        digitalWrite(PINO_LED, HIGH);
    };
}
