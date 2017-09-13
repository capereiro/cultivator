/*
  Libreria utilizada para mensajes de Debugging
  Creacion: 29-NOV-2016
  Author: @charlyPereiro
  cacti.com.ar
*/

#include <Arduino.h>
#include "DebugCultivator.h"

#define VER 1.0

#define DEBUG_PRINT(x)  Serial.println(), Serial.print(x)
#define DEBUG_PRINTLN(x)  Serial.println(x)
#define DEBUG_SEPARATOR() Serial.println("----------------------------------------")


DebugCultivator::DebugCultivator(){}
DebugCultivator::~DebugCultivator(){}

void DebugCultivator::deb_ToolVersion(){
    DEBUG_SEPARATOR(); 
    Serial.println(F("[Debugging Tool Ready to Start]"));
    Serial.print(F("[Debugging Version: ]")); Serial.println(VER);
  }

void DebugCultivator::deb_ValvulaIngresoAguaON(int nivel){
    DEBUG_SEPARATOR(); 
    Serial.print(F("[VALVULA INGRESO AGUA]: ON - Nivel: ")); Serial.print(nivel); Serial.println(F(" Cm"));
  }

void DebugCultivator::deb_ValvulaIngresoAguaOFF(int nivel){
    DEBUG_SEPARATOR(); 
    Serial.print(F("[VALVULA INGRESO AGUA]: OFF - Nivel: ")); Serial.print(nivel); Serial.println(" Cm");
  }

void DebugCultivator::deb_Luminosidad(int lum){
    DEBUG_SEPARATOR(); 
    Serial.print(F("[LUX]: ")); Serial.println(lum);
  }

void DebugCultivator::deb_TemperaturaYhumedad(float tem, float hum){
  DEBUG_SEPARATOR(); 
  Serial.print(F("[TEMPERATURA]: ")); Serial.println(tem);
  Serial.print(F("[HUMEDAD]: ")); Serial.println(hum);
  }

void DebugCultivator::deb_Ph(float ph){
  DEBUG_SEPARATOR(); 
  Serial.print(F("[PH]: ")); Serial.println(ph,2);
  }

void DebugCultivator::deb_Distancia(long dis){
  DEBUG_SEPARATOR(); 
  Serial.print(F("[DISTANCE]: ")); Serial.println(dis);
  }


void DebugCultivator::deb_Cooler(boolean estadoTemp){
    DEBUG_SEPARATOR(); 
    DEBUG_PRINT("[COOLER]: ");
    DEBUG_PRINTLN(estadoTemp ? "ON" : "OFF");
}


void DebugCultivator::deb_IluminacionBandeja(boolean estadoIluminacion){
    DEBUG_SEPARATOR(); 
    DEBUG_PRINT("[LIGHT]: ");
    DEBUG_PRINTLN(estadoIluminacion ? "ON" : "OFF");
}

void DebugCultivator::deb_EstadoSalidaRelay(int nroValvula, boolean estado){
    DEBUG_SEPARATOR(); 
    switch(nroValvula){
          case 31:          // Relay 01::Activa valvula entrada de agua
            DEBUG_PRINT("[VALVULA INGRESO AGUA]: ");
            DEBUG_PRINTLN(estado ? "ON" : "OFF");
          break;
          case 32:          // Relay 02::Activa valvula entrada reactivo PH UP
            DEBUG_PRINT("[VALVULA INGRESO PH UP]: ");
            DEBUG_PRINTLN(estado ? "ON" : "OFF");
          break;
          case 33:          // Relay 03::Activa valvula entrada reactivo PH DW
            DEBUG_PRINT("[VALVULA INGRESO PH DW]: ");
            DEBUG_PRINTLN(estado ? "ON" : "OFF");
          break;
          case 34:          // Relay 04::Activa valvula entrada nutrientes
            DEBUG_PRINT("[VALVULA INGRESO NUTRI]: ");
            DEBUG_PRINTLN(estado ? "ON" : "OFF");
          break;
          case 35:          // Relay 05::Activa Cooler
            DEBUG_PRINT("[COOLER]: ");
            DEBUG_PRINTLN(estado ? "ON" : "OFF");
          break;
          case 36:          // Relay 05::Activa Cooler
            DEBUG_PRINT("[ILUMINACION]: ");
            DEBUG_PRINTLN(estado ? "ON" : "OFF");
          break;
      }
}
