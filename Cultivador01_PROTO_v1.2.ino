
//======= 04/09/2015 =======
//======= 04/09/2015 =======
// V3.7.1 - 04/01/2016
// V3.7.2 - 29/02/2016
// V3.7.3 - 25/03/2016
// V3.7.4 - 02/11/2016
// V3.7.5 - 05/11/2016
// V3.7.6 - 17/11/2016
// V3.7.7 - 25/11/2016 :: New Temp & Humidity code. Code cleaner improvements.
// V3.7.8 - 27/11/2016 :: Implementacion de mensajes por consola desde libreria personalizada
// V3.7.9 - 11/12/2016 :: Implementacion de sensor de electroconductividad, relay para cooler
// V3.8.0 - 17/01/2017 :: Implementacion de modulo Clock
// V3.8.1 - 06/02/2017 :: Implementacion de relay para controlar iluminacion y algoritmo de seleccion de horas
// V3.8.2 - 20/02/2017 :: Emprolijamiento de codigo
// V3.8.3 - 07/03/2017 :: implementacion de algoritmo para medicion PH/EC cada x tiempo y apertura de valvulas correspondientes

// PROTO V1.0 - 09/04/2017 :: Implementación version para prototipo basada en v3.8.3!!! Se habilitó relay de nutrientes
// PROTO V1.1 - 02/07/2017 :: Se reprogramo ciclo de iluminacion mediante el panel digital
// PROTO V1.2 - 07/07/2017 :: 

/* NOTAS
 ===== Canales analogicos Utilizados
 Botonera	     					A0
 Sensor Ph 				PH_PIN		A1

 ===== Canales digitales Utilizados
 Sensor   Ultrasonido		TRIGGER			22
 Sensor   Ultrasonido		ECHO			24
 Sensor   temperatura 		DHT_PIN			02
 Sensor   luminosidad		TSL2651			32
 Relay01  Valvula H2O						31
 Relay02  Valvula PH-UP						32
 Relay03  Valvula PH-DOWN					33
 Relay04  COOLER							34
 Relay05  ILUMINACION						35
*/

#include "Time.h"
#include "DebugCultivator.h"		//Libreria personalizada para mensajes x Serial
#include <Wire.h>
#include <DS1307RTC.h>  			// a basic DS1307 library that returns time as a time_t
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <TSL2561.h>
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DHT.h>

/**************************************************************************/
// -- mensajes debug -------------------
DebugCultivator consola;

/**************************************************************************/
// -- relays -------------------
#define VALVULA_INGRESO_AGUA_PIN	31		// Relay 01::Activa valvula entrada de agua
#define VALVULA_INGRESO_PH_UP_PIN	32		// Relay 02::Activa valvula entrada reactivo PH UP
#define VALVULA_INGRESO_PH_DW_PIN	33		// Relay 03::Activa valvula entrada reactivo PH DW
#define VALVULA_INGRESO_NUTRI_PIN	34		// Relay 04::Activa valvula entrada nutrientes
#define COOLER_PIN					35		// Relay 05::Activa Cooler
#define ILUMINACION_PIN				36		// Relay 06::Activa iluminacion

#define RELAY_ON	0
#define RELAY_OFF	1

/**************************************************************************/
// --- temperatura y humedad --------
#define DHT_PIN 		2					// PIN Temp & Hum
#define DHTTYPE 		DHT11
#define TEMPE 			335
#define HMEDA   		90

#define NIVEL_MIN_TEMP	20					// Nivel inferior temperatura
#define NIVEL_MAX_TEMP	27					// Nivel superior temperatura


unsigned long ahora;

DHT dht(DHT_PIN, DHTTYPE);

long tiempoUltimaLectura=0; 				//Para guardar el tiempo de la última lectura
boolean estadoTemp=FALSE;

struct {
	float tempe; 
	float hume;
} tempeYhumedad;

/**************************************************************************/
// -- ultrasonido -------------------
#define USONIC_TRIGGER_PIN		22		// PIN Trigger
#define USONIC_ECHO_PIN			24		// PIN Echo
#define NIVEL_MIN_AGUA			10		// Nivel inferior agua (Cm)
#define NIVEL_MAX_AGUA			50  	// Nivel superior agua (Cm)

byte nivelAgua = 34;			//VERIFICAR

/**************************************************************************/
// --- clock --------
char bufDayTime[16];	// Para formato por pantalla

/**************************************************************************/
// --- ec metering --------
#define EC_START_CONVERT 		0
#define EC_READ_TEMP 			1
#define EC_PIN 					A4 //EC Meter analog output,pin on analog 4
#define EC_TEMP_PIN 			11 //DS18B20 (Temp) signal, pin on digital 11
#define EC_INTERVAL_SAMPLING	60000UL //10 minutos


#define ECMIN   				125 //**********AVERIGUAR DE QUE SE TRATA

unsigned long lastEcReadingTime = millis();


const byte ecNumReadings = 		20;     //the number of sample times

unsigned int EcAnalogSampleInterval=25,ecPrintInterval=700,ecTempSampleInterval=850;  //analog sample interval;serial print interval;ecTemperature sample interval
unsigned int ecReadings[ecNumReadings];      // the ecReadings from the analog input
byte ecIndex = 0;                  // the ecIndex of the current reading
unsigned long EcAnalogValueTotal = 0;                  // the running total
unsigned int EcAnalogAverage = 0,ecAverageVoltage=0;                // the average
unsigned long EcAnalogSampleTime,ecPrintTime,ecTempSampleTime;
float ecTemperature,ecCurrent; 
 
//ecTemperature chip i/o
OneWire ecTempSensor(EC_TEMP_PIN);  // on digital pin 2

/**************************************************************************/
// -- iluminacion bandejas ----------

boolean estadoIluminacionBandeja_01 	= TRUE;
boolean deteccionBandeja				= TRUE;

unsigned long inicioTiempoIluminacion	= 0UL;
unsigned long finTiempoIluminacion	= 0UL;

byte mminl	= 20;	// mminl: Hora de inicio del ciclo de iluminacion en formato 24HS. Valor por defecto 18hs
byte mmaxl	= 1;	// mmaxl: Cantidad de horas que debe dejar encendida las luces de bandeja. Valor por defecto 6hs de duracion

time_t tInicioIluminacion;
time_t tFinIluminacion;


time_t inicioIluminacion;
time_t periodoIluminacion;
time_t finIluminacion;


/**************************************************************************/
// -- luminosidad -------------------
#define NIVEL_MIN_LUX  			200		// Por debajo del nivel minimo se enciende luz ambiente (salida x relay) 
#define NIVEL_MIN_LUX  			35

int lumin, lu;

//uint16_t Luminosity;

// ADDR can be connected to ground, or vdd or left floating to change the i2c address
// The address will be different depending on whether you let
// the ADDR pin float (addr 0x39), or tie it to ground or vcc. In those cases
// use TSL2561_ADDR_LOW (0x29) or TSL2561_ADDR_HIGH (0x49) respectively

// TSL2561 tsl(TSL2561_ADDR_FLOAT); 

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);


/**************************************************************************/
// -- riego -------------------

#define RIEGO_TIEMPO_CICLO				1*1000UL	//1 segundo de duracion prefijado
unsigned long intervaloEntreCicloRiego	= 0UL;
unsigned long inicioContadorTiempoRiego	= millis();
unsigned long actualContadorTiempoRiego	= 0UL;
int mminr, mmaxr;

/**************************************************************************/
// -- ph ----------------------------
#define PH_PIN 						A1		//pH meter Analog output to Arduino Analog Input 0
#define PH_NIVEL_MIN    			65		//pH Neutro
#define PH_T18B2_TIME		  		100		//Tiempo para DS18B20
#define PH_ARRAY_LENTH 				40 		//times of collection
#define PH_SAMPLING_INTERVAL		20
#define PH_PRINT_INTERVAL  			800
#define PH_OFFSET   		    	1.5		//deviation compensate fijada en 1.5 con sensor C1
#define PH_INTERVAL_SAMPLING		60000UL //10 minutos

static float phvalue2;				
char str_phval[6];					//para passar variable float a string
int pHArray[PH_ARRAY_LENTH];    		//Store the average value of the sensor feedback
int pHArrayIndex=0;

unsigned long lastPhReadingTime;


/**************************************************************************/
// -- botonera ----------------------
#define TICKSN  500		//500 para una hora 
#define TICKSA    1		

#define NUM_KEYS  5		//Define cant de botones
#define RG        0		//Pulsador mueve derecha
#define UP        1		//Pulsador mueve arriba
#define DW        2		//Pulsador mueve abajo
#define LF        3		//Pulsador mueve izquierda
#define SL		  4		//Pulsador para seleccionar opciones

int key =- 1;
int oldkey =- 1;
const int adc_key_val[] = {50, 550, 700, 800, 900};		// Vector para identificar nro de boton


/**************************************************************************/
// -- LCD Screen ----------------------
LiquidCrystal lcd(8,9,4,5,6,7);   // Pines utilizados para LCD
char BuffLcd[22];       //Buffer para imprimir caracteres en el LCD limitado al ancho del display
boolean luzEncendida = true;

/**************************************************************************/
// -- Bump Setup Configuration ----------------------
int valorPorSerial = 0;
// Solo para prototipo - Se debe eliminar al finalizar el proceso

#define TSHOW   500

unsigned long time, tshow;
int estado, numop;

boolean incic = FALSE;

int err;

/**************************************************************************/
// -- Timer ----------------------
#define LEDP     13

volatile int min, seg, horas, ticks, nticks;
volatile int minu, segu;
volatile int tvent, nseg;

/**************************************************************************/
// -- Ciclado -------------------
boolean cct   = TRUE;
boolean ccr   = FALSE;
boolean cci   = FALSE;
int valph, valec, tempe, humte;
int mminv, mmaxv;

//----------FIN DECLARACION DE VARIABLES GLOBALES--------------------------


/**************************** S E T U P ***********************************/
void setup(){
	Serial.begin(9600);

	//I/O controlado por Timer
	pinMode(LEDP, OUTPUT);

	//I/O hacia Sensores
	pinMode(USONIC_TRIGGER_PIN, OUTPUT);

	//I/O hacia Relays --> Ver descripciones con nro de relay en definiciones de Variables Globales
	pinMode(VALVULA_INGRESO_AGUA_PIN, OUTPUT);		
	pinMode(VALVULA_INGRESO_PH_UP_PIN, OUTPUT);		
	pinMode(VALVULA_INGRESO_PH_DW_PIN, OUTPUT);
	pinMode(VALVULA_INGRESO_NUTRI_PIN, OUTPUT);		
	pinMode(COOLER_PIN, OUTPUT);					
	pinMode(ILUMINACION_PIN, OUTPUT);				
	
	//Inicializa relays::Inicializa la salida de relay a estado cerrado (ATENCION: el relay esta configurado para operar invertidamente)
	digitalWrite(VALVULA_INGRESO_AGUA_PIN, RELAY_OFF);
	digitalWrite(VALVULA_INGRESO_PH_UP_PIN, RELAY_OFF);
	digitalWrite(VALVULA_INGRESO_PH_DW_PIN, RELAY_OFF);
	digitalWrite(VALVULA_INGRESO_NUTRI_PIN, RELAY_OFF);
	digitalWrite(COOLER_PIN, RELAY_OFF);
	digitalWrite(ILUMINACION_PIN, RELAY_OFF);

	// -- Start RTC desde placa de Clock --	
  	setSyncProvider(RTC.get);   // the function to get the time from the RTC
  	if(timeStatus()!= timeSet) 
    	Serial.println("Unable to sync with the RTC");
  	else
		Serial.println("RTC has set the system time");  
	// -- End RTC --	



 	// -- Start Timers & Interrupts --
  	// Set Timer0 interrupt a 2kHz
  	TCCR1A = 0;                          // set entire TCCR2A register to 0
  	TCCR1B = 0;                          // same for TCCR2B
  	TCNT1  = 0;                          // initialize counter value to 0
	// set compare match register for 2khz increments
  	OCR1A = 124;                         // = (16*10^6) / (2000*64) - 1 (must be <256)
  	TCCR1A |= (1 << WGM01);              // turn on CTC mode
  	TCCR1B |= (1 << CS11) | (1 << CS10); // Set CS11 and CS10 bits for 64 prescaler
  	TIMSK1 |= (1 << OCIE0A);             // enable timer compare interrupt
 	// -- End Timers & Interrupts --


 	// -- Start Lux --
    /* Initialise the sensor */
    if(!tsl.begin())
    {
      Serial.print("Ooops, no se detecta sensor de luminosidad... Contacte al servicio tecnico!");
      while(1);
    }
    
    configureLuxSensor();     /* Setup the sensor gain and integration time */
  	// -- End Lux --

 	// -- Start Ec --
 	// initialize all the ecReadings to 0:
	  for (byte ecThisReading = 0; ecThisReading < ecNumReadings; ecThisReading++)
	    ecReadings[ecThisReading] = 0;
	  EcTempProcess(EC_START_CONVERT);   //let the DS18B20 start the convert
	  EcAnalogSampleTime=millis();
	  ecPrintTime=millis();
	  ecTempSampleTime=millis();
   	// -- End Ec --

 	// -- Start LCD Screen --  	
  	lcd.begin(20,4);
  	lcd.clear(); 
 	// -- End LCD Screen --  

	// -- Start Time & Interrups --
    // inicial setup
 	time 	= millis();
	ticks 	= 5;
	nticks 	= TICKSN;
	segu 	= minu 	= nseg = 0;
    horas 	= min 	= seg  = 1;
	// -- End Time & Interrups --


	incic = false;
	numop = estado = 0;

    valph = PH_NIVEL_MIN;
    valec = ECMIN;
    tempe = TEMPE;
    humte = HMEDA;
//	lumin = LUMIN_PIN; //Verificar porque me parece que le esta asignando el nro de un PIN
	lumin = NIVEL_MIN_LUX;    


	// Inicializacion de variables para control de ciclados
	mminr =  0;		// manual min riego
	mmaxr =  1;		// manual max riego (24)
	mminv =  0; 	
	mmaxv = 15;
	sei();

}		// Setup End

/***************************** L O O P ************************************/
void loop(){
/*
    horas = min = seg = 1;
	while(1){
		time = millis();
		if(millis()-tshow > 500){
			tshow = millis();
			lcd.setCursor(0,1);
			int temp = LeeTemp();
  			sprintf(BuffLcd, "H: %2d%c%02d%c%02d  ",  horas, ':', min, ':', seg);
  			lcd.print(BuffLcd); 

		}
	}
*/
	if(millis()-time > 60 * 1000UL){			// Si han pasado mas de 1 min salimos del menu
	    estado = incic = 0;
	    time = millis();
  		lcd.clear(); 
    	lcd.noBlink(); 
  	}

/*
	if(millis()-time > 60000UL){				// Si han pasado mas de 6 segundos apagamos la luz
		digitalWrite(BKLITE, LOW);
    	luzEncendida = false;
  	}
*/
/*	
	if(millis()-time > 50000UL){				// Si han pasado mas de 5 segundos apagamos el cursor
    	lcd.noBlink();
  	}
*/
	// -- Start Reading exteran values -- ONLY FOR PROTOTYPE VERSIONS--
	// Se utiliza para entrar un nro por consola terminal y que la bomba trabaje durante ese tiempo
  	if (Serial.available() > 0) {
	// read the incoming byte:
	valorPorSerial = Serial.read();
	valorPorSerial-= 48; //Se le resta 48 al nro ASCII leído para transformarlo a int de forma precaria
	
		if (valorPorSerial>0 and valorPorSerial<20)
		{
			digitalWrite(VALVULA_INGRESO_AGUA_PIN, RELAY_ON);
			delay(valorPorSerial*1000/1);
			digitalWrite(VALVULA_INGRESO_AGUA_PIN, RELAY_OFF);
			valorPorSerial=0;
		}
	}

	// -- Stop Reading exteran values -- ONLY FOR PROTOTYPE VERSIONS--


	// -- Start Temp&Humidity --
	getTemperaturaYhumedad(); // carga valores en una estructura declarada globalmente
  	// -- End Temp&Humidity --

	// -- Start Electroconductivity --
	/*
   	Every once in a while, sample the analog value and calculate the average.
	*/
	if(millis()-EcAnalogSampleTime>=EcAnalogSampleInterval)  
		{
		EcAnalogSampleTime=millis();
		// subtract the last reading:
		EcAnalogValueTotal = EcAnalogValueTotal - ecReadings[ecIndex];
		// read from the sensor:
		ecReadings[ecIndex] = analogRead(EC_PIN);
		// add the reading to the total:
		EcAnalogValueTotal = EcAnalogValueTotal + ecReadings[ecIndex];
		// advance to the next position in the array:
		ecIndex = ecIndex + 1;
		// if we're at the end of the array...
		if (ecIndex >= ecNumReadings)
			// ...wrap around to the beginning:
			ecIndex = 0;
		// calculate the average:
		EcAnalogAverage = EcAnalogValueTotal / ecNumReadings;
		}
	/*
	Every once in a while,MCU read the ecTemperature from the DS18B20 and then let the DS18B20 start the convert.
	Attention:The interval between start the convert and read the ecTemperature should be greater than 750 millisecond,or the ecTemperature is not accurate!
	*/
	if(millis()-ecTempSampleTime>=ecTempSampleInterval) 
		{
		ecTempSampleTime=millis();
		ecTemperature = EcTempProcess(EC_READ_TEMP);  //read the current ecTemperature from the  DS18B20
		EcTempProcess(EC_START_CONVERT);              //after the reading,start the convert for next reading
		}
	/*
	Every once in a while,print the information on the serial monitor.
	*/
	if(millis()-ecPrintTime>=ecPrintInterval)
		{
		ecPrintTime=millis();
		ecAverageVoltage=EcAnalogAverage*(float)5000/1024;
		Serial.print("Analog value:");
		Serial.print(EcAnalogAverage);   //analog average,from 0 to 1023
		Serial.print("    Voltage:");
		Serial.print(ecAverageVoltage);  //millivolt average,from 0mv to 4995mV
		Serial.print("mV    ");
		Serial.print("temp:");
		Serial.print(ecTemperature);    //current ecTemperature
		Serial.print("^C     EC:");

		float EcTempCoefficient=1.0+0.0185*(ecTemperature-25.0);    //ecTemperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.0185*(fTP-25.0));
		float EcCoefficientVolatge=(float)ecAverageVoltage/EcTempCoefficient;   
		//    if(EcCoefficientVolatge<150)Serial.println("No solution!");   //25^C 1413us/cm<-->about 216mv  if the voltage(compensate)<150,that is <1ms/cm,out of the range
		if(EcCoefficientVolatge<150) 
			{
			ecCurrent=0;
			Serial.println(ecCurrent); //0 = Sin solucion
			}//Serial.println("No solution!");   //25^C 1413us/cm<-->about 216mv  if the voltage(compensate)<150,that is <1ms/cm,out of the range
		else if(EcCoefficientVolatge>3300)
			{
			ecCurrent=99; //99 = Fuera de escala
			Serial.println(ecCurrent);
			}//Serial.println("Out of the range!");  //>20ms/cm,out of the range
			else
				{
				if(EcCoefficientVolatge<=448) ecCurrent=6.84*EcCoefficientVolatge-64.32;   //1ms/cm<EC<=3ms/cm
					else if(EcCoefficientVolatge<=1457) ecCurrent=6.98*EcCoefficientVolatge-127;  //3ms/cm<EC<=10ms/cm
						else ecCurrent=5.3*EcCoefficientVolatge+2278;                           //10ms/cm<EC<20ms/cm
				ecCurrent/=1000;    //convert us/cm to ms/cm
				Serial.print(500*ecCurrent,2);  //two decimal
			//	Serial.println("ms/cm");
				Serial.println("ppm");
				}
		}
  	// -- End Temp&Electroconductivity --

	// -- Start Lux --
 	/* Get a new sensor event */ 
 	sensors_event_t event;
 	tsl.getEvent(&event);
 
 	/* Display the results (light is measured in lux) */
  	if (event.light)
 	{
	    lu=event.light;
	    consola.deb_Luminosidad(lu);
  	}
	  else
		  {
		    /* If event.light = 0 lux the sensor is probably saturated
		       and no reliable data could be generated! */
		    Serial.println("Sensor overload");
		  }
  delay(250);

  		//MEJORA: PASAR ESTE DELAY A UNA FUNCION QUE CUENTE TIEMPO
// -- End Lux --


/**************************************************************************/
// Manejo de botonera
	
  	key = get_key(analogRead(A0));			// Devuelve el nro de boton presionado
  	if(key != oldkey){						// if keypress is detected
	    delay(50);							// Espera para evitar los rebotes de las pulsaciones
	    key = get_key(analogRead(A0));		// Obtenemos el boton pulsado
	    if(key != oldkey){
      		time = millis();				// TODO: falta la comprobacion de si se ha desbordado el tiempo
      		if(!luzEncendida){				// Al pulsar cualquier tecla encendemos la pantalla
//        		digitalWrite(BKLITE, HIGH); // intensidad*25);
        		luzEncendida = true;
      		} 
      		else{ 							// si la pantalla esta encendida seguimos funcionando normalmente
        		oldkey = key;
				switch(estado){
					case 0:					// Estado inicial
						if((key == UP) || 
						   (key == DW) || 
						   (key == LF) || 
						   (key == RG)){
								ProgMenu(0);
								estado = 21;
						}
											// Presionando SL: Se cambia de menu y el Estado en 1
						if(key == SL){
							estado++; incic = true;
							ticks = TICKSN;
						}
					break;
// ---- > LINEA 01					
        			case 1:		// Estado 1: Posiciona cursor para configuracion de PH (Linea 01)
        				ProgMenu(estado);
        				if(key == UP) valph++;
        				if(key == DW) valph--;
        				if(key == RG) estado++;		// Estado=2: Configuracion EC
						if(key == LF) estado=11;	// Salto a linea 04 --> Estado=11: Configuracion Manual Riego Min (NEW)
						if(key == SL) estado=3;		// Salto a linea 02 --> Estado=3: Configuracion LUM (salto forzado a estado=3)
					break;

        			case 2:		// Estado 2: Posiciona cursor para configuracion de EC (Linea 01)
						ProgMenu(estado);
        				if(key == UP) valec++;
        				if(key == DW) valec--;
        				if(key == RG) estado++;		// Salto a linea 02 --> Estado=3: Configuracion LUM (NEW)
        				if(key == LF) estado--;		// Estado=1: Configuracion PH
						if(key == SL) estado++; 	// Estado=3: Configuracion EC
					break;
// ---- > LINEA 02
        			case 3:		// Estado 3: Posiciona cursor para configuracion de LU (Linea 02)
        				ProgMenu(estado);
        				if(key == UP) lumin++;
        				if(key == DW) lumin--;
        				if(key == RG) estado++;		// Estado=4: Configuracion ciclado LU Min
        				if(key == LF) estado--;		// Salto a linea 01 --> Estado=2: Configuracion EC (NEW)
						if(key == SL) estado = 6;	// Salto a linea 03 --> Estado=6: Configuracion TE
					break;
					
        			case 4:		// Estado 4: Posiciona cursor para configuracion Manual de la hora de inicio del Ciclo Iluminacion (Linea 02)
						ProgMenu(estado);
        				if(key == UP) if(mminl < 24) { mminl++; resetIluminacion(); }
        				if(key == DW) if(mminl) 	 { mminl--; resetIluminacion(); }
  						if(key == RG) estado++; 	// Estado=5: Configuracion LU Max
						if(key == LF) estado--;		// Estado=3: Configuracion LU
						if(key == SL) estado = 6;	// Salto a linea 03 --> Estado=6: Configuracion TE
						if(mminl > 0) cci == TRUE; else cci == FALSE;
					break;

        			case 5:		// Estado 5: Posiciona cursor para configuracion Manual de la cantidad de horas del Ciclo Iluminacion (Linea 02)
						ProgMenu(estado);
        				if(key == UP) if(mmaxl < 24) { mmaxl++; resetIluminacion(); }
        				if(key == DW) if(mmaxl) 	 { mmaxl--; resetIluminacion(); }
						if(key == RG) estado++;		// Salto a linea 03 --> Estado=6: Configuracion TE (NEW)
						if(key == LF) estado--;		// Estado=4: Configuracion Manual de LU Min
						if(key == SL) estado = 6;	// Salto a linea 03 --> Estado=6: Configuracion TE
						if(mmaxl > 0) cci == TRUE; else cci == FALSE;
					break;
// ---- > LINEA 03
        			case 6:		// Estado 6: Posiciona cursor para configuracion de TE (Linea 03)
						ProgMenu(estado);
        				if(key == UP) if(tempe < 99) tempe++;
        				if(key == DW) if(tempe) tempe--;
        				if(key == RG) estado++; 	// Estado=7: Configuracion Manual de TE Min
        				if(key == LF) estado--;		// Salto a linea 02 --> Estado=5: Configuracion Manual de LU Max (NEW)
						if(key == SL) estado = 9; 	// Salto a linea 04 --> Estado=9: Configuracion Ht
					break;

        			case 7:		// Estado 7: Posiciona cursor para configuracion Manual TE Min (Linea 03)
						ProgMenu(estado);
        				if(key == UP) if(mminv < 99) mminv++;
        				if(key == DW) if(mminv) mminv--;
						if(key == RG) estado++;		// Estado=8: Configuracion Manual de TE Max
						if(key == LF) estado--;		// Estado=6: Configuracion Manual de TE Min
						if(key == SL) estado = 9;	// Salto a linea 04 --> Estado=9: Configuracion Ht
					break;

        			case 8:		// Estado 8: Posiciona cursor para configuracion Manual TE Max (Linea 03)
						ProgMenu(estado);
        				if(key == UP) if(mmaxv < 99) mmaxv++;
        				if(key == DW) if(mmaxv) mmaxv--;
						if(key == RG) estado++;		// Salto de linea --> Estado=9: Configuracion Ht (NEW)
						if(key == LF) estado--;		// Estado=7: Configuracion Manual de TE Min
						if(key == SL) estado = 9;	// Salto a linea 04 --> Estado=9: Configuracion Ht
					break;
// ---- > LINEA 04
        			case 9:		// Estado 9: Posiciona cursor para configuracion Ht (Linea 04)
						ProgMenu(estado);
        				if(key == UP) if(humte < 99) humte++;
        				if(key == DW) if(humte) humte--;
        				if(key == RG) estado++;		// Estado=10: Configuracion Manual de RIEGO Min
        				if(key == LF) estado--;		// Salto a linea 03 --> Estado=8: Configuracion Manual de TE Max (NEW)
						if(key == SL) estado = 20;	// Estado=20: Menu principal
					break;

        			case 10:		// Estado 10: Posiciona cursor para configuracion Manual de Riego Min (Linea 04)
						ProgMenu(estado);
        				if(key == UP) if(mminr < 60) { mminr++; inicioContadorTiempoRiego = millis(); }
        				if(key == DW) if(mminr) 	 { mminr--; inicioContadorTiempoRiego = millis(); }
						if(key == RG) estado++;		// Estado=11: Configuracion Manual de RIEGO Max
						if(key == LF) estado--;		// Estado=9: Configuracion Ht
						if(key == SL) estado = 20;	// Estado=20: Menu principal
					break;

        			case 11:		// Estado 11: Posiciona cursor para configuracion Manual de Riego Max (Linea 04)
						ProgMenu(estado);
        				if(key == UP) if(mmaxr < 24) mmaxr++;
        				if(key == DW) if(mmaxr) mmaxr--;
						if(key == RG) estado = 1;		// Salto a linea 01 --> Estado=1: Configuracion PH (NEW)
						if(key == LF) estado--;			// Estado=10: Configuracion Manual de RIEGO Min
						if(key == SL) estado = 20;		// Estado=20: Menu principal
					break;
// ---- >
        			case 20:	// Estado 20: Menu principal
						lcd.noBlink();
					    estado = incic = 0;
					break;

        			case 21:	// Aqui Programa
						if(key == SL){
						    estado = incic = 0;
						}
					break;
				}
      		}
    	}
  	}
/**************************************************************************/
// Pantalla principal

	if(!incic && !estado){
		time = millis();
		if(millis()-tshow > 500){
			tshow = millis();

			lcd.setCursor(0,0);
			sprintf(bufDayTime,"%d:%d - %d/%d/%d ",hour(),minute(),day(),month(),year());
			lcd.print(bufDayTime);

		//	lcd.print(F(" cacti - ecu v3.8.2"));
			lcd.setCursor(0,1);
  			sprintf(BuffLcd, "TE:%2d%c HA:%2d LU:%4d ", (int)tempeYhumedad.tempe, 223, (int)tempeYhumedad.hume, (int)lu); 
  			lcd.print(BuffLcd); 

			lcd.setCursor(0,2);
			dtostrf(phvalue2, 3, 1, str_phval);
  			sprintf(BuffLcd, "PH:%s EC:%2d  HS:XX", str_phval, (int)ecCurrent);
  			lcd.print(BuffLcd);

			lcd.setCursor(0,3);
			sprintf(BuffLcd, "Na:%2dcm ", nivelAgua);
  			lcd.print(BuffLcd);			
			if(cci) lcd.print(F("CCI "));		// Con Ciclo Iluminacion
			else    lcd.print(F("SCI "));		// Sin Ciclo Iluminacion
			if(ccr) lcd.print(F("CCR "));		// Con Ciclo Riego
			else    lcd.print(F("SCR "));		// Sin Ciclo Riego
			if(cct) lcd.print(F("CCT."));		// Con Ciclo Temperatura
			else    lcd.print(F("SCT."));		// Sin Ciclo Temperatura
		}
	}	//Fin pantalla principal
/**************************************************************************/
	
	nivelAgua = (int)UltraSens();
	if(nivelAgua > 99) nivelAgua = 99;		// VERIFICAR: limites
	
//	lumsens();
//	phmeter();
	
	phvalue2=phmeter2();
	
//	ecmeter();  // Verificar porque queda sin servicio
//	htmeter();
	delay(50);
  
/**************************************************************************/
//Salidas relays

//Control de nivel de Agua via sensor Ultrasonido

// Modificacion por Prototipo. Por el momento fuera de produccion 12/03/2017

	if(nivelAgua < NIVEL_MIN_AGUA){
		digitalWrite(VALVULA_INGRESO_AGUA_PIN, RELAY_OFF);
//		consola.deb_EstadoSalidaRelay(VALVULA_INGRESO_AGUA_PIN,FALSE);
//		consola.deb_ValvulaIngresoAguaOFF(nivelAgua);
	}

	if(nivelAgua > NIVEL_MAX_AGUA){
		digitalWrite(VALVULA_INGRESO_AGUA_PIN, RELAY_ON);
		consola.deb_EstadoSalidaRelay(VALVULA_INGRESO_AGUA_PIN,TRUE);

//		consola.deb_ValvulaIngresoAguaON(nivelAgua);
	}


/****************************************************/
//Control de nivel de Temperatura

	if(tempeYhumedad.tempe < NIVEL_MIN_TEMP){
		//Activa relay
		digitalWrite(COOLER_PIN, RELAY_OFF);
		estadoTemp=FALSE;
		consola.deb_EstadoSalidaRelay(COOLER_PIN,estadoTemp);
	}

	if(tempeYhumedad.tempe > NIVEL_MAX_TEMP){
		//Desactiva relay
		digitalWrite(COOLER_PIN, RELAY_ON);
		estadoTemp=TRUE;		
		consola.deb_EstadoSalidaRelay(COOLER_PIN,estadoTemp);
	}


/****************************************************/
//Control de ciclo de riego programado manualmente --
//Si mminr cumple con la condición, entra en loop de ciclo de riego según 
//programacion manual.
// mminr se configura por panel en periodo de DIAS (maximo periodo es de 5 dias)
// mmaxr se configura por panel en CANTIDAD
// De esta forma se puede configurar el ciclo como CANTIDAD de veces por DIA
if(mminr>0 || mminr<=5 ){
	actualContadorTiempoRiego 	= millis();
	intervaloEntreCicloRiego 	= (mmaxr*24*60*60*1000UL)/mminr; // horas pasadas a milisegundos
	if(actualContadorTiempoRiego - inicioContadorTiempoRiego >= intervaloEntreCicloRiego)
	{
		inicioContadorTiempoRiego=millis();
		activarValvulaXtiempo(RIEGO_TIEMPO_CICLO,VALVULA_INGRESO_AGUA_PIN);
	}
}
//Fin control de riego programado manualmente




/****************************************************/
//Control de pH


//					TBD


//Fin Control de pH

/****************************************************/
//Control de nutrientes


//					TBD


//Fin Control de nutrientes
/****************************************************/

//End Salidas relays



/****************************************************/
//Control de iluminacion de bandejas

gestionIluminacionBandejas (deteccionBandeja, mminl, mmaxl);

//End Control de iluminacion de bandejas

/**************************************************************************/

  	if(incic == true){
		switch(numop){
			case 0:
			break;
		}
  	}
} // Fin loop

/**************************************************************************/
/************************** F I N   L O O P *******************************/
/**************************************************************************/



/**************************************************************************/
/* 
	Funciones definidas por el programador
*/
/**************************************************************************/

/**************************************************************************/
//Funcion para activar una válvula por un periodo de tiempo y lueho desactivarla
//Entrada: periodo de tiempo, nro de valvula
//Salida: N/A

void activarValvulaXtiempo(unsigned long periodo, byte nroValvula){
	digitalWrite(nroValvula,RELAY_ON);
	consola.deb_EstadoSalidaRelay(nroValvula,TRUE);
	delay(periodo);
	digitalWrite(nroValvula,RELAY_OFF);
	consola.deb_EstadoSalidaRelay(nroValvula,FALSE);
}

/**************************************************************************/
//Funcion para medir la cantidad de fluido dentro del deposito de agua
//Entrada: N/A
//Salida: Distancia

long UltraSens() {
  	long duration, distance;
  	digitalWrite(USONIC_TRIGGER_PIN, LOW);
  	delayMicroseconds(2);
  	digitalWrite(USONIC_TRIGGER_PIN, HIGH);
  	delayMicroseconds(10);
  	digitalWrite(USONIC_TRIGGER_PIN, LOW);
  	duration = pulseIn(USONIC_ECHO_PIN, HIGH);
  	distance = (duration/2) / 29.1; // valor devuelto en centimetros
  	
  	consola.deb_Distancia(distance);
 
	return(distance);
}

/**************************************************************************/
// Convertimos el valor analogico leido  
// en un numero de boton pulsado
int get_key(unsigned int input){
int k;

	for(k = 0; k < NUM_KEYS; k++){
		if(input < adc_key_val[k]) return k;
  	}
	if(k >= NUM_KEYS) k = -1;  			// Error en la lectura.
	return k;
}

/**************************************************************************/
// Timer0 interrupt 2kHz toggles pin 13 (LED)
// Genera una frecuencia 2kHz/2 = 1kHz
ISR(TIMER1_COMPA_vect){
	if(!ticks--){  // 500 ticks para 1 seg
		ticks = nticks;
		digitalWrite(LEDP, !digitalRead(LEDP));
		if(!seg){
			if(!min){
				if(!horas){
					 
				}
				else{
					horas--;
					min = seg = 59;
				}
			}
			else{
				min--;
				seg = 59;
			} 
		} 
		else seg--;
	}
}


/**************************************************************************/
//Funcion empleada para mostrar la programación manual por pantalla (stage 2)
//y posicionar el cursor en ubicación recibida por parámetro 
//Entrada: posicion (pos) en donde se ubicará el cursor
//Salida: Muestra parámetros manuales por pantalla secundaria

void ProgMenu(int pos){
	lcd.blink();
	lcd.setCursor(0,0);
	sprintf(BuffLcd, "Ph:%1d.%1d Ec:%03d Na:%02d.", valph/10, valph%10, valec, nivelAgua);
	lcd.print(BuffLcd);

	lcd.setCursor(0,1); 
	if(mminl){
		sprintf(BuffLcd, "Lu: %02d   CCI  %02d.%02d.", lumin, mminl, mmaxl);
		lcd.print(BuffLcd);
	}
	else{
		sprintf(BuffLcd, "Lu: %02d   SCI  %02d.%02d.", lumin, mminl, mmaxl);
		lcd.print(BuffLcd);
	}

	lcd.setCursor(0,2);
	if(mminv){
		sprintf(BuffLcd, "Te: %2d.%1d CCT  %02d.%02d.", tempe/10, tempe%10, mminv, mmaxv);
		cct = true;
	}
	else{
		sprintf(BuffLcd, "Te: %2d.%1d SCT  %02d.%02d.", tempe/10, tempe%10, mminv, mmaxv);	
		cct = false;
	}
	lcd.print(BuffLcd);

	lcd.setCursor(0,3);
	if(mminr){
		sprintf(BuffLcd, "Ht: %2d%c  CCR  %02d.%02d.", humte, 37, mminr, mmaxr);
		ccr = true;
	}
	else{
		sprintf(BuffLcd, "Ht: %2d%c  SCR  %02d.%02d.", humte, 37, mminr, mmaxr);
		ccr = false;
	}
	lcd.print(BuffLcd);

	//Posiciona el curson en un punto dado
	if(pos ==  0) lcd.noBlink(); 
	if(pos ==  1) lcd.setCursor(6,0); 
	if(pos ==  2) lcd.setCursor(13,0);
	
	if(pos ==  3) lcd.setCursor(8,1); 
	if(pos ==  4) lcd.setCursor(16,1); 
	if(pos ==  5) lcd.setCursor(19,1);
	
	if(pos ==  6) lcd.setCursor(8,2);
	if(pos ==  7) lcd.setCursor(16,2);
	if(pos ==  8) lcd.setCursor(19,2);
	
	if(pos ==  9) lcd.setCursor(8,3);
	if(pos == 10) lcd.setCursor(16,3);
	if(pos == 11) lcd.setCursor(19,3);
}
/**************************************************************************/
//Funcion empleada para inicializar sensor de luminosidad
//Entrada: N/A
//Salida: solo inicia el sensor


// -- Start Lux --
/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displayLuxSensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("@Charly Pereiro");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureLuxSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */  
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}

//-- End Lux --


/**************************************************************************/
//Funcion para manejo de PH
//Entrada: N/A
//Salida: Valor de pH

static float phmeter2(void){

static unsigned long samplingTime = millis();
static unsigned long printTime = millis();
static float pHValue,voltage;
char str_pHValue[6];

	if(millis() - samplingTime > PH_SAMPLING_INTERVAL){
		pHArray[pHArrayIndex++]=analogRead(PH_PIN);
		if(pHArrayIndex==PH_ARRAY_LENTH) pHArrayIndex=0;				//Reset de pHArrayIndex
		voltage = avergearray(pHArray, PH_ARRAY_LENTH) * 5.0/1024;
		pHValue = 3.5 * voltage + PH_OFFSET;

		samplingTime=millis();
	}
	if(millis() - printTime > PH_PRINT_INTERVAL){ //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
		consola.deb_Ph(pHValue);
		printTime=millis();
	}
//	dtostrf(pHValue, 4, 1, str_pHValue);
	return pHValue;
}


/**************************************************************************/
//Funcion empleada dentro del manejo de PH
//Entrada: Array, Cant elementos
//Salida: Promedio

double avergearray(int* arr, int number){
int i;
int max,min;
double avg;
long amount=0;

	if(number <= 0){
		Serial.println("Error number for the array to avraging!/n");
		return 0;
	}
	if(number < 5){ //less than 5, calculated directly statistics
		for(i=0;i<number;i++){
			amount+=arr[i];
		}
		avg = amount/number;
		return avg;
	}
	else{
		if(arr[0]<arr[1]){
			min = arr[0]; 
			max = arr[1];
		}
		else{
			min=arr[1];max=arr[0];
		}

		for(i=2; i<number; i++){
			if(arr[i]<min){
				amount+=min; //arr<min
				min=arr[i];
			}
			else{
				if(arr[i]>max){
					amount+=max; //arr>max
					max=arr[i];
				}
				else{
					amount+=arr[i]; //min<=arr<=max
				}
			}//if
		}//for
		avg = (double)amount/(number-2);
	}//if
	return avg;
}

/**************************************************************************/
//Funcion empleada para Electroconductividad
//Entrada: 
//Salida: 

/*
ch=0,let the DS18B20 start the convert;ch=1,MCU read the current ecTemperature from the DS18B20.
*/
float EcTempProcess(bool ch)
{
  //returns the ecTemperature from one DS18B20 in DEG Celsius
  static byte ecData[12];
  static byte ecAddr[8];
  static float EcTemperatureSum;
  if(!ch){
          if ( !ecTempSensor.search(ecAddr)) {
              Serial.println("no more sensors on chain, reset search!");
              ecTempSensor.reset_search();
              return 0;
          }      
          if ( OneWire::crc8( ecAddr, 7) != ecAddr[7]) {
              Serial.println("CRC is not valid!");
              return 0;
          }        
          if ( ecAddr[0] != 0x10 && ecAddr[0] != 0x28) {
              Serial.print("Device is not recognized!");
              return 0;
          }      
          ecTempSensor.reset();
          ecTempSensor.select(ecAddr);
          ecTempSensor.write(0x44,1); // start conversion, with parasite power on at the end
  }
  else{  
          byte ecPresent = ecTempSensor.reset();
          ecTempSensor.select(ecAddr);    
          ecTempSensor.write(0xBE); // Read Scratchpad            
          for (int i = 0; i < 9; i++) { // we need 9 bytes
            ecData[i] = ecTempSensor.read();
          }         
          ecTempSensor.reset_search();           
          byte MSB = ecData[1];
          byte LSB = ecData[0];        
          float ecTempRead = ((MSB << 8) | LSB); //using two's compliment
          EcTemperatureSum = ecTempRead / 16;
    }
          return EcTemperatureSum;  
}




void ecmeter(){
	// HAY QUE IMPLEMENTAR O ELIMINAR
}

/**************************************************************************/
//Funcion empleada para Humedad en Tierra (Sustrato)
//Entrada: 
//Salida: 

void htmeter(){
	
}


/**************************************************************************/
//Funcion empleada para leer Temperatura y Humedad
//Entrada: No requiere
//Salida: carga valores en estructura tempeYhumedad definida globalmente

void getTemperaturaYhumedad(){
  if(millis()-tiempoUltimaLectura>2000)
  {    
  tempeYhumedad.tempe=dht.readTemperature();  //lectura de TEMP
  tempeYhumedad.hume=dht.readHumidity();    //lectura de HUM
  tiempoUltimaLectura=millis(); //actualizamos el tiempo de la última lectura

Serial.println(tempeYhumedad.tempe);

  consola.deb_TemperaturaYhumedad(tempeYhumedad.tempe,tempeYhumedad.hume);
  }
}

/**************************************************************************/
//Funcion empleada para modulo de Clock - Agrega un dos puntos para mejorar el formato de salida por consola
//Entrada: minutos o segundos
//Salida: dos puntos ":"

void printDots(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

//Funcion empleada para modulo de Clock - completa con "0" los meses/anos con un digito para mejorar el formato de salida por consola
//Entrada: mes/ano
//Salida: cifra con un cero antepuesto
void printZeros(int digits){
  // utility function for digital clock display: complete 0's
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
/**************************************************************************/
//Funcion empleada para manejo de la gestion de la iluminacion por bandeja
//Entrada: deteccionBandeja, inicioIluminacion, finIluminacion
//Salida: enciende o apaga iluminacion de bandeja

void gestionIluminacionBandejas(boolean deteccionBandeja, byte horaInicio, byte horaFin){
	if(deteccionBandeja){
		if(hour() == horaInicio) {
			digitalWrite(ILUMINACION_PIN, RELAY_ON);
			estadoIluminacionBandeja_01=TRUE;
		}
		else {
			if(hour() == horaFin) {
				digitalWrite(ILUMINACION_PIN, RELAY_OFF);
				estadoIluminacionBandeja_01=FALSE;
			}
		} 
	}
}	

/**************************************************************************/
//Funcion empleada para inicializar iluminacion de bandejas
//Entrada: 
//Salida: enciende o apaga iluminacion de bandeja

void resetIluminacion(){
	digitalWrite(ILUMINACION_PIN, RELAY_OFF);
	estadoIluminacionBandeja_01=FALSE;
}

/**************************************************************************/
//Funcion empleada para suministrar o cortar iluminacion cuando la hora de inicio 
// y fin ocurren dentro del mismo dia
//Entrada: deteccionBandeja, inicioIluminacion, finIluminacion
//Salida: N/A
/*
void sameDayAlgorithm(boolean deteccionBandeja, time_t inicioIluminacion, time_t finIluminacion){
	if(deteccionBandeja){
		if(hour() >= hour(inicioIluminacion) || hour() <= hour(finIluminacion)) {
			digitalWrite(ILUMINACION_PIN, RELAY_ON);
			estadoIluminacionBandeja_01=TRUE;
			Serial.println("-------------------------------------");
			Serial.println("sameDayAlgorithm 1");
			Serial.print("Hora inicio: "); Serial.println(hour(inicioIluminacion));
			Serial.print("Hora fin: "); Serial.println(hour(finIluminacion));
			Serial.println("-------------------------------------");
		}
		else {
			digitalWrite(ILUMINACION_PIN, RELAY_OFF);
			estadoIluminacionBandeja_01=FALSE;
			Serial.println("-------------------------------------");
			Serial.println("sameDayAlgorithm 2");
			Serial.print("Hora inicio: "); Serial.println(hour(inicioIluminacion));
			Serial.print("Hora fin: "); Serial.println(hour(finIluminacion));
			Serial.println("-------------------------------------");
		}
	}
}
*/
/**************************************************************************/
//Funcion empleada para suministrar o cortar iluminacion cuando la hora de inicio 
// y fin ocurren dentro de dos dias
//Entrada: estadoIluminacionBandeja_01, inicioIluminacion, finIluminacion
//Salida: N/A
/*
void splitDayAlgorithm(boolean deteccionBandeja, time_t inicioIluminacion, time_t finIluminacion){
	if(deteccionBandeja){
		if(hour() >= hour(inicioIluminacion) && hour() <= 23) {
			digitalWrite(ILUMINACION_PIN, RELAY_ON);
			estadoIluminacionBandeja_01=TRUE;
			Serial.println("-------------------------------------");
			Serial.println("splitDayAlgorithm 1");
			Serial.print("Hora inicio: "); Serial.println(hour(inicioIluminacion));
			Serial.print("Hora fin: "); Serial.println(hour(finIluminacion));
			Serial.println("-------------------------------------");
		}
		else {
			if(hour() >= hour(finIluminacion)){
				digitalWrite(ILUMINACION_PIN, RELAY_OFF);
				estadoIluminacionBandeja_01=FALSE;
				Serial.println("-------------------------------------");
				Serial.println("splitDayAlgorithm 2");
				Serial.print("Hora inicio: "); Serial.println(hour(inicioIluminacion));
				Serial.print("Hora fin: "); Serial.println(hour(finIluminacion));
				Serial.println("-------------------------------------");
			}
		}
	}
}
*/




/**************************************************************************/
//Funcion empleada para convertir Horas en Millisegundos
//Entrada: Horas de trabajo (byte)
//Salida: UL

unsigned long convertir_Hs2Milli(byte horas){
	return horas * 60 * 60 * 1000UL;
}


/**************************************************************************/
//Funcion empleada para configurar fechas en formato time_t
//Entrada: anio, mes, dia, hs, minutos, seg
//Salida: fecha en formato time_t
time_t SetFecha(int y, int m, int d, int h, int mi, int s  )
   {  tmElements_t Fecha ;
      Fecha.Second = s;
      Fecha.Minute = mi;
      Fecha.Hour = h;
      Fecha.Day = d ;
      Fecha.Month = m ;
      Fecha.Year = y -1970 ;
 
      return makeTime(Fecha);
   }

