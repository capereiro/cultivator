/*
  Libreria utilizada para mensajes de Debugging
  Creacion: 29-NOV-2016
  Author: @charlyPereiro
  cacti.com.ar
*/

#include<Arduino.h> 

#define H HIGH //Indica que el estado es Alto
#define L LOW //Indica que el estado es Bajo

class DebugCultivator{
   private:
//     byte _pin1; //Primer Pin

   public:
    DebugCultivator();      //Constructor
    ~DebugCultivator();     //Destructor
    void deb_ToolVersion();
    void deb_ValvulaIngresoAguaON(int nivel);
    void deb_ValvulaIngresoAguaOFF(int nivel);
    void deb_Luminosidad(int lum);
    void deb_TemperaturaYhumedad(float tem, float hum);
    void deb_Ph(float ph);
    void deb_Distancia(long dis);
    void deb_CoolerOFF(float temp, byte NIVEL_MIN_TEMP);
    void deb_CoolerON(float temp, byte NIVEL_MAX_TEMP);
    void deb_Cooler(boolean estadoTemp);
    void deb_IluminacionBandeja(boolean estadoIluminacion);
    void deb_EstadoSalidaRelay(int nroValvula, boolean estado);
};
